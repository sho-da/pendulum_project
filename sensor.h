#ifndef SENSOR_H
#define SENSOR_H
// #include <cstdlib>
// #include <cmath>
// #include <iostream>
// #include "pigpiod_if2.h"

//=========================================================
// Accelerometer (BMX055)
//=========================================================
// get data
float get_acc_data(int pi, int bus)
{
    unsigned char data[4];
    i2c_read_i2c_block_data(pi, bus, 0x04, (char *)data, 4);

    int y_data = ((data[0] & 0xF0) + (data[1] * 256)) / 16;
    if (y_data > 2047)
    {
        y_data -= 4096;
    }

    int z_data = ((data[2] & 0xF0) + (data[3] * 256)) / 16;
    if (z_data > 2047)
    {
        z_data -= 4096;
    }

    float theta1_deg = atan2(float(z_data), float(y_data)) * 57.29578f;
    return theta1_deg;
}


// statistical data of accelerometer
// By passing references to theta_mean and theta_variance, using "&",
// you can modify the values at their referenced locations.
void acc_init(int pi, int bus, int sample_num, float meas_interval, float &theta_mean, float &theta_variance)
{
    // initialize ACC register 0x0F (range)
    // Full scale = +/- 2 G
    i2c_write_byte_data(pi, bus, 0x0F, 0x03);
    // initialize ACC register 0x10 (band width)
    // Filter bandwidth = 1000 Hz
    i2c_write_byte_data(pi, bus, 0x10, 0x0F);

    // get data
    float theta_array[sample_num];
    for (int i = 0; i < sample_num; i++)
    {
        theta_array[i] = get_acc_data(pi, bus);
        usleep(meas_interval);
    }

    // calculate mean
    theta_mean = 0;
    for (int i = 0; i < sample_num; i++)
    {
        theta_mean += theta_array[i];
    }
    theta_mean /= sample_num;

    // calculate variance
    float temp;
    theta_variance = 0;
    for (int i = 0; i < sample_num; i++)
    {
        temp = theta_array[i] - theta_mean;
        theta_variance += temp * temp;
    }
    theta_variance /= sample_num;
    return;
}

//=========================================================
// Gyroscope (BMX055)
//=========================================================
// get data
float get_gyr_data(int pi, int bus)
{
    unsigned char data[2];
    i2c_read_i2c_block_data(pi, bus, 0x02, (char *)data, 2);

    int theta1_dot = data[0] + 256 * data[1];
    if (theta1_dot > 32767)
    {
        theta1_dot -= 65536;
    }
    theta1_dot = -1 * theta1_dot; // !caution!
    // +1000 (deg/sec) / 2^15 = 0.0305176
    return float(theta1_dot) * 0.0305176f;
}

// statistical data of gyro
void gyr_init(int pi, int bus, int sample_num, float meas_interval, float &theta_dot_mean, float &theta_dot_variance)
{
    // initialize Gyro register 0x0F (range)
    // Full scale = +/- 1000 deg/s
    i2c_write_byte_data(pi, bus, 0x0F, 0x01);
    // initialize Gyro register 0x10 (band width)
    // Data rate = 1000 Hz, Filter bandwidth = 116 Hz
    i2c_write_byte_data(pi, bus, 0x10, 0x02);

    // get data
    float theta_dot_array[sample_num];
    for (int i = 0; i < sample_num; i++)
    {
        theta_dot_array[i] = get_gyr_data(pi, bus);
        usleep(meas_interval);
    }

    // calculate mean
    theta_dot_mean = 0;
    for (int i = 0; i < sample_num; i++)
    {
        theta_dot_mean += theta_dot_array[i];
    }
    theta_dot_mean /= sample_num;

    // calculate variance
    float temp;
    theta_dot_variance = 0;
    for (int i = 0; i < sample_num; i++)
    {
        temp = theta_dot_array[i] - theta_dot_mean;
        theta_dot_variance += temp * temp;
    }
    theta_dot_variance /= sample_num;
    return;
}



#endif //SENSOR_H