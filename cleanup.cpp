#include <pigpiod_if2.h>
#include <unistd.h>
#include <iostream>

// LED ============================================= 
const int LED_pin1 = 22;
const int LED_pin2 = 17;
const int LED_pin3 = 27;

// rotary encoder =============================================
const int encoder_pin1 = 24;  // to A
const int encoder_pin2 = 23;  // to B

// motor driver =============================================
const int motor_IN1 = 6; //31
const int motor_IN2 = 5; //29
const int motor_pwm = 12; // Motor driver PWM input 32

int main() {
    // Initialize pigpio
    int pi = pigpio_start(NULL,NULL);

    // ***** LED cleanup *****
    set_mode(pi, LED_pin1, PI_OUTPUT );   // red
    set_mode(pi, LED_pin2, PI_OUTPUT );
    set_mode(pi, LED_pin3, PI_OUTPUT );
    gpio_write(pi, LED_pin1, 0);
    gpio_write(pi, LED_pin2, 0);
    gpio_write(pi, LED_pin3, 0);

    // ***** rotary encoder cleanup *****
    // Nan

    // ***** motor driver cleanup *****
    set_mode(pi,motor_IN1, PI_OUTPUT);
    set_mode(pi,motor_IN2, PI_OUTPUT);
    set_mode(pi,motor_pwm, PI_OUTPUT);
    gpio_write(pi,motor_IN1, 0);
    gpio_write(pi,motor_IN2, 0);
    pigpio_stop(pi);

    return 0;
}
