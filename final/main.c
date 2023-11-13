/**

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

// semaphore
static struct pt_sem vga_semaphore;
const int dirPin = 15;
const int stepPin = 14;
const int stepsPerRevolution = 200;
const int stepPads[4] = {2,3,4,5};
const int sequence[4] = {3,6,12,9};


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

    // while(true){
    //     // Set motor direction clockwise
    //     gpio_put(dirPin, 1);
    //     // Spin motor slowly
    //     for(int x = 0; x < stepsPerRevolution; x++)
    //     {
    //         gpio_put(stepPin, 1);
    //         sleep_us(2000);
    //         gpio_put(stepPin, 0);
    //         sleep_us(2000);
    //     }
    //     sleep_ms(1000); // Wait a second
    //     // Set motor direction counterclockwise
    //     gpio_put(dirPin, 0);
    //     // Spin motor quickly
    //     for(int x = 0; x < stepsPerRevolution; x++)
    //     {
    //         gpio_put(stepPin, 1);
    //         sleep_us(2000);
    //         gpio_put(stepPin, 0);
    //         sleep_us(2000);
    //     }
    //     sleep_ms(1000); // Wait a second
    // }
    
    while (true){
        for(int i = 0; i < 4; i ++){
            int s = sequence[i];
            for (int j = 0; j < 4; j++){
                int m = 1 << j;
                if ((m & s) > 0){
                    gpio_put(stepPads[j],-1);
                }
                else{
                    gpio_put(stepPads[j],0);
                }
            }
            sleep_ms(2);
        }
    }
    

    // Indicate end of thread
    PT_END(pt);
}


static PT_THREAD (protothread_serial(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;
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

    // Initialize VGA
    initVGA() ;

    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}
