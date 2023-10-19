#include <iostream>
#include <pigpiod_if2.h>
#include <cmath>
#include <unistd.h>

#define ACCL_ADDR 0x19

int pi;

float get_acc_data(int bus) {
    float theta_deg =0;
    unsigned char data[4];
    i2c_read_i2c_block_data(pi,bus, 0x04, (char*)data, 4);

    int y_data = ((data[0] & 0xF0) + (data[1] * 256)) / 16;
    if (y_data > 2047) {
        y_data -= 4096;
    }

    int z_data = ((data[2] & 0xF0) + (data[3] * 256)) / 16;
    if (z_data > 2047) {
        z_data -= 4096;
    }

    theta_deg = atan2( float(z_data),float(y_data)) * 57.29578f;
    return theta_deg;
}

int main(int argc, char **argv) {
    pi=pigpio_start(NULL,NULL);

    
    //gpioSetMode(LED_PIN, PI_OUTPUT);
    //led_control(1);
    sleep(1);
    //led_control(0);

    int bus = i2c_open(pi, 1, ACCL_ADDR, 0);
    i2c_write_byte_data(pi,bus, 0x0F, 0x03);
    i2c_write_byte_data(pi,bus, 0x10, 0x0F);

    while (true) {
        float theta_deg = get_acc_data(bus);
        std::cout << "theta = " << theta_deg << " degrees" << std::endl;
        sleep(1);  // 1 seconds delay
    }

    i2c_close(pi,bus);
    pigpio_stop(pi);
    return 0;
}