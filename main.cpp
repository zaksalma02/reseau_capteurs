/*
 * Copyright (c) 2021, Koncepto.io
 * SPDX-License-Identifier: Apache-2.0
 */

#include "PinNameAliases.h"
#include "mbed.h"
#include "scd4x.h"

static void display_value_result(sixtron::SCD4X::ErrorType err, uint16_t co2, double t, double rh)
{
    if (err != sixtron::SCD4X::ErrorType::Ok) {
        printf("Error reading data: %d\n", err);
    } else {
        printf("CO2: %dppm, Temp: %f°C, RH: %f%%\n", co2, t, rh);
    }
}

static void test_get_serial(sixtron::SCD4X &sensor)
{
    uint16_t data[3];
    sixtron::SCD4X::ErrorType err;

    err = sensor.get_serial_number(data);
    if (err != sixtron::SCD4X::ErrorType::Ok) {
        printf("Could not get serial\n");
    } else {
        printf("Sensor serial: %2x%2x%2x\n", data[0], data[1], data[2]);
    }
}

static void test_low_power_measurement(sixtron::SCD4X &sensor)
{
    float t, rh;
    uint16_t co2;
    sixtron::SCD4X::ErrorType err;
    sixtron::scd4x_measurement_t meas;

    printf("Periodic low power measurement\n");
    err = sensor.start_low_power_periodic_measurement();
    if (err != sixtron::SCD4X::ErrorType::Ok) {
        printf("Periodic low power measurement cannot be started\n");
        sensor.stop_periodic_measurement();
        return;
    }

    for (int i = 0; i < 3; i++) {
        while (sensor.get_data_ready_status() != sixtron::SCD4X::ErrorType::Ok) {
            //ThisThread::sleep_for(1s);
        }

        err = sensor.read_measurement(&meas);
        display_value_result(err, meas.co2, meas.temperature, meas.rh);

        //ThisThread::sleep_for(10s);
    }
    sensor.stop_periodic_measurement();
    //ThisThread::sleep_for(500ms);
}
/*
static void test_periodic_measurement(sixtron::SCD4X &sensor)
{
    sixtron::scd4x_measurement_t meas;
    sixtron::SCD4X::ErrorType err;

    printf("Periodic measurement\n");
    err = sensor.start_periodic_measurement();
    if (err != sixtron::SCD4X::ErrorType::Ok) {
        printf("Periodic measurement cannot be started\n");
        sensor.stop_periodic_measurement();
        return;
    }
    while (true) {
        err = sensor.get_data_ready_status();
        if (err != sixtron::SCD4X::ErrorType::Ok) {
            ThisThread::sleep_for(1s);
            continue;
        }
        err = sensor.read_measurement(&meas);
        display_value_result(err, meas.co2, meas.temperature, meas.rh);

        ThisThread::sleep_for(10s);
    }
}

static void test_single_shot_measurement(sixtron::SCD4X &sensor)
{
    sixtron::scd4x_measurement_t meas;
    sixtron::SCD4X::ErrorType err;

    printf("Single shot measurement\n");

    sensor.measure_single_shot();
    ThisThread::sleep_for(5s);

    err = sensor.read_measurement(&meas);
    display_value_result(err, meas.co2, meas.temperature, meas.rh);
}

static void test_single_shot_measurement_rht_only(sixtron::SCD4X &sensor)
{
    sixtron::scd4x_measurement_t meas = { 0 };
    sixtron::SCD4X::ErrorType err;

    printf("Single shot measurement (RHT only)\n");

    sensor.measure_single_shot_rht_only();
    ThisThread::sleep_for(50ms);

    err = sensor.read_measurement(&meas);
    display_value_result(err, meas.co2, meas.temperature, meas.rh);
}
*/
static void test_selftest(sixtron::SCD4X &sensor)
{
    sixtron::SCD4X::ErrorType err;

    printf("Self test\n");
    err = sensor.perform_self_test();
    if (err != sixtron::SCD4X::ErrorType::Ok) {
        printf("Self test failed with error: %d\n", err);
    } else {
        printf("Self test succeeded\n");
    }
}

// main() runs in its own thread in the OS
int main()
{
    sixtron::SCD4X sensor(P1_I2C_SDA, P1_I2C_SCL);
    sixtron::SCD4X::ErrorType err;

    double t, rh;
    uint16_t co2;

    display_value_result(err, co2, t, rh);
    printf("\n\n-------------------\n");
    printf("--- SCD4X Demo ---\n");

    /* Force periodic measurement stop first and wait for its completion (500ms) */
    sensor.stop_periodic_measurement();
    //ThisThread::sleep_for(500ms);

    test_get_serial(sensor);

    //test_single_shot_measurement_rht_only(sensor);
    //test_single_shot_measurement(sensor);
    test_low_power_measurement(sensor);
    test_selftest(sensor);
    //test_periodic_measurement(sensor);
}