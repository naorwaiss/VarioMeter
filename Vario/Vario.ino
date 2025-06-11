#include <Arduino.h>
#include <Wire.h>
#include "src/imu_class.h"
#include "src/Gps_class.h"
#include "src/ElapsedTimer.h"
#include "src/Baro_class.h"
#include "src/kalman.h"

#define IMU_TIME 200
#define BARO_TIME 64
#define GPS_TIME 10

ElapsedTimer gps_timer(GPS_TIME);
ElapsedTimer imu_timer(IMU_TIME);
ElapsedTimer baro_timer(BARO_TIME);
IMU imu(IMU_TIME, 0.7f);
GPS gps(GPS_TIME);  // Using Serial for GPS
Baro baro;  // Assuming you have a Baro class defined
Kalman filter(&gps, &baro, &imu);  // Pass addresses of the objects

void setup() {
    Serial.begin(115200);  // Debug serial
    Wire.begin();
    gps.initialize_gps();
    imu.initialize_imu();
    baro.initialize_baro();  // Initialize the barometer
    filter.initialize();
}

void loop() {
    if (baro_timer.hasElapsed()) {
        baro.read_baro();
        baro_timer.reset();
    }

    if (gps_timer.hasElapsed()) {
        gps.read_gps();
        filter.main_kalman();
        gps_timer.reset();
    }

    if (imu_timer.hasElapsed()) {
        imu.mainIMU();
        imu_timer.reset();
    }
}
