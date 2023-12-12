/**
 * This is the code for the final project of ECE 5730
 * Project Name: Microwave Imaging System
 * Author: Zehao Li(zl823) and Kapil Gangwar(kg434)
 * Date: 12/11/2023 
 * Description: This project is to build the Microwave Imaging System with two Vivaldi antennas 
   aims to achieve good-resolution 2D imaging through circular motion and 3D image reconstruction 
   by vertical movement of the subject using scattering parameter messurements.

 * HARDWARE CONNECTIONS
    - GPIO 2  ---> STEP on the A4988 Stepper Motor Driver 1
    - GPIO 3  ---> DIR on the A4988 Stepper Motor Driver 1
    - GPIO 7  ---> Pin1 on the Keyboard
    - GPIO 8  ---> Pin2 on the Keyboard
    - GPIO 9  ---> Pin3 on the Keyboard
    - GPIO 10 ---> Pin4 on the Keyboard
    - GPIO 11 ---> Pin5 on the Keyboard
    - GPIO 12 ---> Pin6 on the Keyboard
    - GPIO 13 ---> Pin7 on the Keyboard
    - GPIO 14 ---> STEP on the A4988 Stepper Motor Driver 2
    - GPIO 15 ---> DIR on the A4988 Stepper Motor Driver 2
    - GPIO 16 ---> VGA Hsync
    - GPIO 17 ---> VGA Vsync
    - GPIO 18 ---> 330 ohm resistor ---> VGA Red
    - GPIO 19 ---> 330 ohm resistor ---> VGA Green
    - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
    - RP2040 GND ---> VGA GND
    - RP2040 GND ---> Switch ---> RP2040 RUN
    - RP2040 GND ---> GND on the A4988 Stepper Motor Driver 1 and 2
    - RP2040 3V3 ---> VDD on the A4988 Stepper Motor Driver 1 and 2

    - External 12V power supply ---> VMOT on the A4988 Stepper Motor Driver 1 and 2
    - External 12V ground ---> GND on the A4988 Stepper Motor Driver 1 and 2
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
const int dirPin_rotation = 3; // rotation direction pin
const int stepPin_rotation = 2; // rotation step pin
const int dirPin_lift = 15; // lift direction pin
const int stepPin_lift = 14; // lift step pin
int stepsPerRevolution; // Steps per revolution
float degree = 0.0; // degree of the rotation
int redius = 100; // redius of the circle
int rotate_flag = 0; // flag to rotate the plate
#define FRAME_RATE 16667 // 60 Hz frame rate
#define BASE_KEYPAD_PIN 7
#define KEYROWS         4
#define NUMKEYS         12
static int idx;
static int old_idx;
unsigned int keycodes[12] = {   0x28, 0x11, 0x21, 0x41, 0x12,
                                0x22, 0x42, 0x14, 0x24, 0x44,
                                0x18, 0x48} ;
unsigned int scancodes[4] = {   0x01, 0x02, 0x04, 0x08} ;
unsigned int button = 0x70 ;
int lift = 0; // stop the lift
int page = 0; // Main Page
int stop = 0; // stop flag for the rotation
int manual_auto = 0; // Initial Stop Flag
int auto_index = 0; // indicator of the auto initial configuration step
int temp_step = 0; //temp hold for rotation degree
int temp_step2 = 0; // temp hold for rotation interval
int temp_step3 = 0; // temp hold for lift interval
int interval = 0; // set auto rotation interval
int interval2 = 0; // set auto lift interval
static int new_x ; // new end x pos of the pointer on the circle
static int new_y ;  // new end y pos of the pointer on the circle
char deg[50]; // degree in string
char temp[50]; // temp string for rotation degree
char temp1[50]; // temp string for auto rotation interval
char temp2[50]; // temp string for auto lift interval

// Interrupt service routine
void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));
    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

void draw_UI(){ // Only call Draw UI when the page is changed
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
            setCursor(450, 290);
            writeString(temp);
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
        case 2: // at Auto Page
            writeString("Auto Page");
            switch (auto_index){
                case 0:
                    setCursor(450, 230);
                    writeString("Rotation degree:");
                    setCursor(450, 250);
                    writeString("0-9. Input");
                    setCursor(450, 270);
                    writeString("*. Confirm");
                    setCursor(450, 290);
                    writeString("#. Back to main");
                    setCursor(450, 310);
                    writeString(temp);
                    break;
                case 1:
                    setCursor(450, 230);
                    writeString("Rotation interval in seconds:");
                    setCursor(450, 250);
                    writeString("0-9. Input");
                    setCursor(450, 270);
                    writeString("*. Confirm");
                    setCursor(450, 290);
                    writeString("#. Back to main");
                    setCursor(450, 310);
                    writeString(temp);
                    setCursor(450, 330);
                    writeString(temp1);
                    break;
                case 2:
                    setCursor(450, 230);
                    writeString("Manual Lift Control");
                    setCursor(450, 250);
                    writeString("2. Up");
                    setCursor(450, 270);
                    writeString("5. Down");
                    setCursor(450, 290);
                    writeString("*. Confirm");
                    setCursor(450, 310);
                    writeString("#. Back to main");
                    setCursor(450, 330);
                    writeString(temp);
                    setCursor(450, 350);
                    writeString(temp1);
                    break;
                case 3:
                    setCursor(450, 230);
                    writeString("Lift interval in cm:");
                    setCursor(450, 250);
                    writeString("0-9. Input");
                    setCursor(450, 270);
                    writeString("*. Confirm");
                    setCursor(450, 290);
                    writeString("#. Back to main");
                    setCursor(450, 310);
                    writeString(temp);
                    setCursor(450, 330);
                    writeString(temp1);
                    setCursor(450, 350);
                    writeString(temp2);
                    break;
            }
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
    new_x = 320 + redius * sin(-degree*PI/180);
    new_y = 240 + redius * cos(-degree*PI/180);
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
    
    sprintf(temp, "%s%.2f", "Current Degree is ", temp_step*1.8);
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
                manual_auto = 1; // at Manual Mode
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
                        manual_auto = 0; // change to stop
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
                        if (temp_step > 200){
                            temp_step = 200;
                        }
                        sprintf(temp, "%s%.2f", "Degree Selection:  ", temp_step*1.8);
                        draw_UI();
                    }else if (idx == 10){ // confirm
                        stepsPerRevolution = temp_step;
                        rotate_flag = 1;
                        temp_step = 0;
                        stop = 0;
                    }else if (idx == 11){ // go back to the manual page
                        page = 1; // go to manual page
                        temp_step = 0;
                        rotate_flag = 0;
                        stop = 1;
                        sprintf(temp, "%s%.2f", "Degree Selection:  ", temp_step*1.8);
                        draw_UI();
                    }
                }
                break;
            case 12: // at Manual Lift Page
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
            case 2: // at Auto Page
                switch (auto_index){
                    case 0: // at auto rotation degree selection page
                        if (old_idx != idx){
                            if (idx <= 9 && 0 <= idx){ // integer
                                temp_step = temp_step * 10 + idx;
                                if (temp_step > 200){
                                    temp_step = 200;
                                }
                                sprintf(temp, "%s%.2f", "Degree Selection:  ", temp_step*1.8);
                                draw_UI();
                            }else if (idx == 10){ // confirm
                                stepsPerRevolution = temp_step;
                                temp_step = 0;
                                temp_step2 = 0;
                                temp_step3 = 0;
                                auto_index = 1; // go to the lift page
                                draw_UI();
                            }else if (idx == 11){ // go back to the manual page
                                page = 0; // go to main page
                                temp_step = 0;
                                temp_step2 = 0;
                                temp_step3 = 0;
                                draw_UI();
                            }
                        }
                        break;
                    case 1: // at auto interval time selection page
                        if (old_idx != idx){
                            if (idx <= 9 && 0 <= idx){ // integer
                                temp_step2 = temp_step2 * 10 + idx;
                                sprintf(temp1, "%s%d%s", "Degree Interval:  ", temp_step2, "(s)");
                                draw_UI();
                            }else if (idx == 10){ // confirm
                                interval = temp_step2;
                                auto_index = 2; // go to height adjust page 
                                temp_step = 0;
                                temp_step2 = 0;
                                temp_step3 = 0;
                                draw_UI();
                            }else if (idx == 11){ // go back to the rotation degree selection page
                                page = 0; // go to main page
                                temp_step = 0;
                                temp_step2 = 0;
                                temp_step3 = 0;
                                draw_UI();
                            }
                        }
                        break;
                    case 2: // at auto lift page
                        if (idx == 2){ // go up
                            gpio_put(dirPin_lift, 0);
                            sleep_ms(2);
                            gpio_put(stepPin_lift, 1);
                            sleep_ms(2);
                            gpio_put(stepPin_lift, 0);
                            sleep_ms(2);
                        }else if (idx == 5){ // go down
                            gpio_put(dirPin_lift, 1);
                            sleep_ms(2);
                            gpio_put(stepPin_lift, 1);
                            sleep_ms(2);
                            gpio_put(stepPin_lift, 0);
                            sleep_ms(2);
                        }else if (idx == 10 && old_idx != idx){ // confirm
                            auto_index = 3; // go to lift selection page
                            temp_step = 0;
                            temp_step2 = 0;
                            temp_step3 = 0;
                            draw_UI();
                        }else if (idx == 11 && old_idx != idx){ // go back to the interval time selection page
                            page = 0; // go to main page
                            temp_step = 0;
                            temp_step2 = 0;
                            temp_step3 = 0;
                            draw_UI();
                        }
                        break;
                    case 3: // at auto lift selection page
                        if (old_idx != idx){
                            if (idx <= 9 && 0 <= idx){ // integer
                                temp_step3 = temp_step3 * 10 + idx;
                                sprintf(temp2, "%s%d%s", "Lift Interval:  ", temp_step3, "(cm)");
                                draw_UI();
                            }else if (idx == 10){ // confirm
                                interval2 = temp_step3;
                                temp_step = 0;
                                temp_step2 = 0;
                                temp_step3 = 0;
                                manual_auto = 2;
                                draw_UI();
                            }else if (idx == 11){ // go back to the rotation degree selection page
                                page = 0; // go to main page
                                temp_step = 0;
                                temp_step2 = 0;
                                temp_step3 = 0;
                                draw_UI();
                            }
                        }
                        break;
                }

                if (idx == 11 && old_idx != idx){ // go back to the Main Page
                    page = 0; // go to the main page
                    manual_auto = 0; // change to stop
                }
                break;
            default:
                break;
        }
        
        //printf("%d %d %f\n",page, temp_step, degree);
        //printf("idx: %d\n",idx);
        old_idx = idx;

        spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
        // yield for necessary amount of time
        PT_YIELD_usec(spare_time) ;
    } // END WHILE(1)
    

    // Indicate end of thread
    PT_END(pt);
}

int count = 0;;
static PT_THREAD (protothread_serial(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;
    int s; // step

    if (manual_auto==1){ // at manual mode
        // motor rotation
        if (rotate_flag == 1 && stop == 0){
            //Set motor direction clockwise
            gpio_put(dirPin_rotation, 1);
            // Spin motor
            degree += stepsPerRevolution * 1.8;
            draw_rotation_platform();
            draw_rotation_text();
            for(int x = 0; x < stepsPerRevolution; x++){
                gpio_put(stepPin_rotation, 1);
                sleep_ms(10);
                gpio_put(stepPin_rotation, 0);
                sleep_ms(10);
            }
            if (degree >= 360.0){
                rotate_flag = 0;
                degree = 0.0;
                sprintf(temp, "%s%.2f", "Current Degree is ", temp_step*1.8);
                draw_UI();
                draw_rotation_platform();
                draw_rotation_text();
            }
            stop = 1;
        }
        // Motor Lift
        switch (lift){
            case 1: // if up manual
                gpio_put(dirPin_lift, 0);
                sleep_ms(2);
                gpio_put(stepPin_lift, 1);
                sleep_ms(2);
                gpio_put(stepPin_lift, 0);
                sleep_ms(2);
                lift = 0; // stop the lift
                break;
            case 2: // if down manual
                gpio_put(dirPin_lift, 1);
                sleep_ms(2);
                gpio_put(stepPin_lift, 1);
                sleep_ms(2);
                gpio_put(stepPin_lift, 0);
                sleep_ms(2);
                lift = 0; // stop the lift
                break;
            case 3: // if up by 1cm
                gpio_put(dirPin_lift, 0);
                for(int x = 0; x < 545; x++){
                    gpio_put(stepPin_lift, 1);
                    sleep_ms(6);
                    gpio_put(stepPin_lift, 0);
                    sleep_ms(6);
                }
                lift = 0; // stop the lift
                break;
            case 4: // if down by 1cm
                gpio_put(dirPin_lift, 1);
                for(int x = 0; x < 545; x++){
                    gpio_put(stepPin_lift, 1);
                    sleep_ms(6);
                    gpio_put(stepPin_lift, 0);
                    sleep_ms(6);
                }
                lift = 0; // stop the lift
                break;
            default:
                break;
        }
    }else if (manual_auto == 2){ // at auto mode
        //Motor Rotation auto run
        int step = 200 / stepsPerRevolution;
        for(int i = 0; i < step; i++){
            // Set motor direction clockwise
            gpio_put(dirPin_rotation, 1);
            // Spin motor
            printf("%f",degree);
            degree += stepsPerRevolution * 1.8;
            draw_rotation_platform();
            draw_rotation_text();
            for(int x = 0; x < stepsPerRevolution; x++){
                gpio_put(stepPin_rotation, 1);
                sleep_ms(10);
                gpio_put(stepPin_rotation, 0);
                sleep_ms(10);
            }
            sleep_ms(interval*1000);
        }
        degree = 0.0;
        for (int i=0; i<interval2;i++){
            gpio_put(dirPin_lift, 0);
            for(int x = 0; x < 545; x++){
                gpio_put(stepPin_lift, 1);
                sleep_ms(6);
                gpio_put(stepPin_lift, 0);
                sleep_ms(6);
            }
            sleep_ms(interval*1000);
            //Motor Rotation auto run
            int step = 200 / stepsPerRevolution;
            for(int i = 0; i < step; i++){
                // Set motor direction clockwise
                gpio_put(dirPin_rotation, 1);
                // Spin motor
                degree += stepsPerRevolution * 1.8;
                draw_rotation_platform();
                draw_rotation_text();
                for(int x = 0; x < stepsPerRevolution; x++){
                    gpio_put(stepPin_rotation, 1);
                    sleep_ms(10);
                    gpio_put(stepPin_rotation, 0);
                    sleep_ms(10);
                }
                sleep_ms(interval*1000);
            }
            degree = 0.0;
        }
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

    gpio_init(stepPin_rotation);
    gpio_init(dirPin_rotation);
    gpio_init(stepPin_lift);
    gpio_init(dirPin_lift);
    gpio_set_dir(stepPin_lift, GPIO_OUT);
    gpio_set_dir(dirPin_lift, GPIO_OUT);
    gpio_set_dir(stepPin_rotation, GPIO_OUT);
    gpio_set_dir(dirPin_rotation, GPIO_OUT);
    

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
