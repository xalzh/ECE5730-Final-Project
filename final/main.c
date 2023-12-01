/**

 * HARDWARE CONNECTIONS
   - GPIO 2  ---> IN2 on the ULN2003 Driver Board
   - GPIO 3  ---> IN3 on the ULN2003 Driver Board
   - GPIO 4  ---> IN4 on the ULN2003 Driver Board

   - GPIO 2  ---> IN2 on the ULN2003 Driver Board

   - GPIO 16 ---> VGA Hsync
   - GPIO 17 ---> VGA Vsync
   - GPIO 18 ---> 330 ohm resistor ---> VGA Red
   - GPIO 19 ---> 330 ohm resistor ---> VGA Green
   - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
   - RP2040 GND ---> VGA GND

 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
// Include custom libraries
#include "vga_graphics.h"
#include "pt_cornell_rp2040_v1.h"

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)
#define SIGN(x) ((x > 0) - (x < 0))
#define PI 3.14159

// semaphore
static struct pt_sem vga_semaphore; // VGA drawing semaphore
const int dirPin = 15; // direction pin
const int stepPin = 14; // step pin
int stepsPerRevolution; // Steps per revolution
const int stepPads[4] = {2,3,4,5}; // step pads
const int sequence[4] = {3,6,12,9}; // sequence for the lift
const int reverse_sequence[4] = {9, 12, 6, 3}; // reverse sequence for the lift
float degree = 0.0; // degree of the rotation
int redius = 100; // redius of the circle
int rotate_flag = 0; // flag to rotate the plate
#define FRAME_RATE 16667 // 60 Hz frame rate
#define BASE_KEYPAD_PIN 7
#define KEYROWS         4
#define NUMKEYS         12
static int idx;
unsigned int keycodes[12] = {   0x28, 0x11, 0x21, 0x41, 0x12,
                                0x22, 0x42, 0x14, 0x24, 0x44,
                                0x18, 0x48} ;
unsigned int scancodes[4] = {   0x01, 0x02, 0x04, 0x08} ;
unsigned int button = 0x70 ;
int lift = 0; // stop the lift
int page = 0; // Main Page
int stop = 0; // stop flag for the rotation
static int new_x ; // new end x pos of the pointer on the circle
static int new_y ;  // new end y pos of the pointer on the circle
char deg[50]; // degree in string

// Interrupt service routine
void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));
    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

void draw_UI(){
    fillRect(440, 200, 200, 280, BLACK);
    drawRect(440, 200, 200, 280, WHITE);
    setCursor(450, 210);
    setTextColor2(WHITE, BLACK);
    switch (page){
        case 0: // at Main Page
            writeString("Main Page");
            setCursor(450, 230);
            writeString("1. Manual");
            setCursor(450, 250);
            writeString("2. Auto");
            break;
        case 1: // at Manual Page
            writeString("Manual Page");
            setCursor(450, 230);
            writeString("1. Rotation");
            setCursor(450, 250);
            writeString("2. Lift");
            setCursor(450, 270);
            writeString("#. Back");
            break;
        case 11: // at Manual Rotation Page
            writeString("Manual Rotation Page");
            setCursor(450, 230);
            writeString("0-9. Input");
            setCursor(450, 250);
            writeString("*. Confirm");
            setCursor(450, 270);
            writeString("#. Back");
            break;
        case 12: // at Manual Lift Page
            writeString("Manual Lift Page");
            setCursor(450, 230);
            writeString("2. Up");
            setCursor(450, 250);
            writeString("5. Down");
            setCursor(450, 270);
            writeString("3. Up 1cm");
            setCursor(450, 290);
            writeString("6. Down 1cm");
            setCursor(450, 310);
            writeString("#. Back");
            break;
        case 2: // at Auro Page
            writeString("Auto Page");
            break;
    }
}

void detect_keypad(){
    //Scan the keypad!
    static uint32_t keypad ;
    for (idx=0; idx<KEYROWS; idx++) {
        // Set a row high
        gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                        (scancodes[idx] << BASE_KEYPAD_PIN)) ;
        // Small delay required
        sleep_us(1) ; 
        // Read the keycode
        keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
        // Break if button(s) are pressed
        if (keypad & button) break ;
    }
    // If we found a button . . .
    if (keypad & button) {
        // Look for a valid keycode.
        for (idx=0; idx<NUMKEYS; idx++) {
            if (keypad == keycodes[idx]) break ;
        }
        // If we don't find one, report invalid keycode
        if (idx==NUMKEYS) (idx = -1) ;
    }
    // Otherwise, indicate invalid/non-pressed buttons
    else (idx=-1) ;
}

void draw_rotation_platform(){
    // draw circle
    drawCircle(320, 240, redius, WHITE);
    drawLine(320, 240, new_x, new_y, BLACK);
    new_x = 320 + redius * sin(degree*PI/180);
    new_y = 240 + redius * cos(degree*PI/180);
    drawLine(320, 240, new_x, new_y, WHITE);
}

void draw_rotation_text(){
    setCursor(250, 400);
    setTextColor2(BLACK, BLACK);
    writeString(deg);
    setCursor(250, 400);
    setTextColor2(WHITE, BLACK);
    sprintf(deg, "%s%.2f", "Current Degree is ", degree);
    writeString(deg);
}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;
    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;
    setTextSize(1);
    
    int temp_step = 0;
    int old_idx;
    draw_rotation_platform();
    draw_rotation_text();
    draw_UI();

    while(1) {
        // Measure time at start of thread
        begin_time = time_us_32();

        detect_keypad();

        switch(page){
            case 0: // at Main Page
                if (old_idx != idx){
                    if (idx == 1){
                        page = 1; // go to the manual page
                        draw_UI();
                    }else if (idx == 2){
                        page = 2; // go to the auto page
                        draw_UI();
                    }
                }
                break;
            case 1: // at Manual Page
                if (old_idx != idx){
                    if (idx == 1){
                        page = 11; // go to the rotation page
                        draw_UI();
                    }else if (idx == 2){
                        page = 12; // go to the lift page
                        draw_UI();
                    }else if (idx == 11){
                        page = 0; // go back to the main page
                        draw_UI();
                    }
                }
                break;
            case 11: // at Manual Rotation Page
                if (old_idx != idx){
                    if (stop == 1 && idx == 0){ // continue to rotate by one step
                        stop = 0;
                    }
                    else if (idx <= 9 && 0 <= idx){ // integer
                        temp_step = temp_step * 10 + idx;
                    }else if (idx == 10){ // confirm
                        stepsPerRevolution = temp_step;
                        rotate_flag = 1;
                        temp_step = 0;
                    }else if (idx == 11){ // go back to the manual page
                        page = 1; // go to manual page
                        temp_step = 0;
                        rotate_flag = 0;
                        stop = 1;
                        draw_UI();
                    }
                }
                break;
            case 12:
                if (idx == 2){ // go up manually
                    lift = 1; // up manual
                }else if (idx == 5){ // go down manually
                    lift = 2; // down manual
                }else if (idx == 3 && old_idx != idx){ // go up by 1cm
                    lift = 3; // up by 1cm
                }else if (idx == 6 && old_idx != idx){ // go down by 1cm
                    lift = 4; // down by 1cm
                }else if (idx == 11 && old_idx != idx){ // go back to the manual page
                    page = 1; // go to the manual page
                    lift = 0; // stop the lift
                    draw_UI();
                }
                break;
            case 2:
                if (idx == 11 && old_idx != idx){ // go back to the Main Page
                    page = 0; // go to the main page
                }
                break;
            default:
                break;
        }
        
        printf("%d %d %f\n",page, temp_step, degree);
        old_idx = idx;

        spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
        // yield for necessary amount of time
        PT_YIELD_usec(spare_time) ;
    } // END WHILE(1)
    

    // Indicate end of thread
    PT_END(pt);
}

static PT_THREAD (protothread_serial(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;
    int s; // step

    // Motor Rotation auto solution
    // if (rotate_flag == 1){
    //     int step = 200 / stepsPerRevolution;
    //     for(int i = 0; i < step; i++){
    //         // Set motor direction clockwise
    //         gpio_put(dirPin, 1);
    //         // Spin motor
    //         printf("%f",degree);
    //         degree += stepsPerRevolution * 1.8;
    //         for(int x = 0; x < stepsPerRevolution; x++){
    //             gpio_put(stepPin, 1);
    //             sleep_us(2000);
    //             gpio_put(stepPin, 0);
    //             sleep_us(2000);
    //         }
    //         sleep_ms(500);
    //     }
    //     rotate_flag = 0;
    //     degree = 0.0;
    // }

    if (rotate_flag == 1 && stop == 0){
        //Set motor direction clockwise
        gpio_put(dirPin, 1);
        // Spin motor
        degree += stepsPerRevolution * 1.8;
        draw_rotation_platform();
        draw_rotation_text();
        for(int x = 0; x < stepsPerRevolution; x++){
            gpio_put(stepPin, 1);
            sleep_us(2000);
            gpio_put(stepPin, 0);
            sleep_us(2000);
        }
        stop = 1;
        if (degree >= 360.0){
            rotate_flag = 0;
            degree = 0.0;
        }
    }

    // Motor Lift
    switch (lift){
        case 1: // if up manual
            for(int i = 0; i < 4; i ++){
            s = reverse_sequence[i];
            for (int j = 0; j < 4; j++){
                int m = 1 << j;
                if ((m & s) > 0){gpio_put(stepPads[j],1);}
                else{gpio_put(stepPads[j],0);}
            }
            sleep_ms(2);
            }
            lift = 0; // stop the lift
            break;
        case 2: // if down manual
            for(int i = 0; i < 4; i ++){
            s = sequence[i];
            for (int j = 0; j < 4; j++){
                int m = 1 << j;
                if ((m & s) > 0){gpio_put(stepPads[j],1);}
                else{gpio_put(stepPads[j],0);}
            }
            sleep_ms(2);
            }
            lift = 0; // stop the lift
            break;
        case 3: // if up by 1cm
            for (int k = 0; k < 650; k++){
                for(int i = 0; i < 4; i ++){
                s = reverse_sequence[i];
                for (int j = 0; j < 4; j++){
                    int m = 1 << j;
                    if ((m & s) > 0){gpio_put(stepPads[j],1);}
                    else{gpio_put(stepPads[j],0);}
                }
                sleep_ms(2);
                }
            }
            lift = 0; // stop the lift
            break;
        case 4: // if down by 1cm
            for (int k = 0; k < 650; k++){
                for(int i = 0; i < 4; i ++){
                s = sequence[i];
                for (int j = 0; j < 4; j++){
                    int m = 1 << j;
                    if ((m & s) > 0){gpio_put(stepPads[j],1);}
                    else{gpio_put(stepPads[j],0);}
                }
                sleep_ms(2);
                }
            }
            lift = 0; // stop the lift
            break;
        default:
            break;
    }

    // Indicate end of thread
    PT_END(pt);
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;
}

int main() {

    // Initialize stdio
    stdio_init_all();

    gpio_init(stepPin);
    gpio_init(dirPin);
    gpio_set_dir(stepPin, GPIO_OUT);
    gpio_set_dir(dirPin, GPIO_OUT);
    
    for(int i = 0; i < 4; i ++){
        gpio_init(stepPads[i]);
        gpio_set_dir(stepPads[i], GPIO_OUT);
        gpio_put(stepPads[i], 0);
    }

    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN)) ;
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)) ;
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0x0 << BASE_KEYPAD_PIN)) ;
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_down((BASE_KEYPAD_PIN + 4)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 5)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 6)) ;

    // Initialize VGA
    initVGA() ;

    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}
