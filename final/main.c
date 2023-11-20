/**

 * HARDWARE CONNECTIONS
 *  - GPIO 2  ---> IN2 on the ULN2003 Driver Board
 *  - GPIO 3  ---> IN3 on the ULN2003 Driver Board
 *  - GPIO 4  ---> IN4 on the ULN2003 Driver Board

 *  - GPIO 2  ---> IN2 on the ULN2003 Driver Board

 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND

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
#define PI 3.14159265358979323846

// semaphore
static struct pt_sem vga_semaphore;
const int dirPin = 15;
const int stepPin = 14;
int stepsPerRevolution = 5;
const int stepPads[4] = {2,3,4,5};
const int sequence[4] = {3,6,12,9};
const int reverse_sequence[4] = {9, 12, 6, 3};
float degree = 0.0;
int redius = 100;
int rotate_flag = 0; // flag to rotate the plate
#define FRAME_RATE 33333
#define BASE_KEYPAD_PIN 7
#define KEYROWS         4
#define NUMKEYS         12
static int idx;
unsigned int keycodes[12] = {   0x28, 0x11, 0x21, 0x41, 0x12,
                                0x22, 0x42, 0x14, 0x24, 0x44,
                                0x18, 0x48} ;
unsigned int scancodes[4] = {   0x01, 0x02, 0x04, 0x08} ;
unsigned int button = 0x70 ;
int up = 0;
int down = 0;
int s;

// Interrupt service routine
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;
    static int new_x ;
    static int new_y ;
    setTextSize(1);
    char deg[50];
    int page = 0;
    int temp_step = 0;
    int old_idx;

    while(1) {
        // Measure time at start of thread
        begin_time = time_us_32() ;      
        // draw circle
        drawCircle(320, 240, redius, WHITE);
        new_x = 320 + redius * sin(degree*PI/180);
        new_y = 240 + redius * cos(degree*PI/180);
        //printf("%d, %d\n",new_x,new_y);
        //printf("%f\n",degree);
        drawLine(320, 240, new_x, new_y, WHITE);
        spare_time = FRAME_RATE - (time_us_32() - begin_time) ;

        setCursor(250, 400);
        setTextColor2(WHITE, BLACK);
        sprintf(deg, "%s%.2f", "Current Degree is ", degree);
        writeString(deg);

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

        if (page == 0){ // at main page, 1 for rotate, 2 for lift, 3 for auto
            if (idx == 1){
                page = 1; // go to the rotate page
            }else if (idx == 2){
                page = 2; // go to the lift page
            }
        }else if (page == 1){ // at the rotate page
            if (idx <= 9 && 0 <= idx && old_idx != idx){ // integer
                temp_step = temp_step * 10 + idx;
            }else if (idx == 10){ // confirm
                stepsPerRevolution = temp_step;
                rotate_flag = 1;
            }else if (idx == 11){ // go back
                page = 0;
            }
        }else if (page == 2){ // at the lift page
            if (idx == 2){ // go up
                up = 1;
            }else if (idx == 5){ // go down
                down = 1;
            }else if (idx == 11){ // go back
                page = 0;
            }
        }

        printf("%d %d %f\n",page, temp_step, degree);
        
        old_idx = idx;

        // yield for necessary amount of time
        PT_YIELD_usec(spare_time) ;
        drawLine(320, 240, new_x, new_y, BLACK);
        setCursor(250, 400);
        setTextColor2(BLACK, BLACK);
        writeString(deg);
    } // END WHILE(1)
    

    // Indicate end of thread
    PT_END(pt);
}

static PT_THREAD (protothread_serial(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // Motor Rotation
    if (rotate_flag == 1){
        int step = 200 / stepsPerRevolution;
        for(int i = 0; i < step; i++){
            // Set motor direction clockwise
            gpio_put(dirPin, 1);
            // Spin motor
            printf("%f",degree);
            degree += stepsPerRevolution * 1.8;
            for(int x = 0; x < stepsPerRevolution; x++){
                gpio_put(stepPin, 1);
                sleep_us(2000);
                gpio_put(stepPin, 0);
                sleep_us(2000);
            }
            sleep_ms(500);
        }
        rotate_flag = 0;
        degree = 0.0;
    }


    // Motor Lift
    if (up == 1 || down == 1){
        for(int i = 0; i < 4; i ++){
            if (up == 1){
                s = reverse_sequence[i];
            }else if (down == 1){
                s = sequence[i];
            }
            
            for (int j = 0; j < 4; j++){
                int m = 1 << j;
                if ((m & s) > 0){
                    gpio_put(stepPads[j],1);
                }
                else{
                    gpio_put(stepPads[j],0);
                }
            }
            sleep_ms(2);
        }
        up = 0;
        down = 0;
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
