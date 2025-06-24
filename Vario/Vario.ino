#include <Arduino.h>
#include <Wire.h>
#include "src/imu_class.h"
#include "src/Gps_class.h"
#include "src/ElapsedTimer.h"
#include "src/Baro_class.h"
#include "src/kalman.h"
#include <U8g2lib.h>
#include "src/ScreenControl.h"

#define IMU_TIME 200    // 1/200 seconds
#define BARO_TIME 64    // 1/64 seconds
#define GPS_TIME 10     // 1/10 seconds
#define SCREEN_TIME 10  // 1/10 seconds

ElapsedTimer gps_timer(GPS_TIME);
ElapsedTimer imu_timer(IMU_TIME);
ElapsedTimer baro_timer(BARO_TIME);
ElapsedTimer screen_timer(SCREEN_TIME);

IMU imu(IMU_TIME, 0.7f);
GPS gps(GPS_TIME);                 // Using Serial for GPS
Baro baro(BARO_TIME);              // Assuming you have a Baro class defined
Kalman filter(&gps, &baro, &imu);  // Pass addresses of the objects
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C screen(U8G2_R0);
ScreenControl screen_control(A0, &filter);
// Update callback function for GPS

void setup() {
    Serial.begin(9600);  // Debug serial
    screen_control.init();
    screen_control.start_screen();
    Wire.begin();
    gps.initialize_gps();
    imu.initialize_imu();
    baro.initialize_baro();  // Initialize the barometer
    filter.initialize();
    screen.begin();
}

void loop() {
    filter.calc_all();
    if (imu_timer.hasElapsed()) {
        imu.mainIMU();
        filter.predict();
        imu_timer.reset();
    }

    if (baro_timer.hasElapsed()) {
        baro.read_baro();
        filter.update(baro.baro_data.altitude, filter.R_baro);
        if (gps_timer.hasElapsed()) {
            gps.read_gps();
            if (gps._gps_data.sat_count > 10) {
                filter.update(gps._gps_data.altitude, filter.R_gps);
            }
            gps_timer.reset();
        }
        // print_screen();
        screen_control.main_screen();
        // screen_control.on_click();
        // screen_control.update_array_index();
        baro_timer.reset();
    }
}
void print_screen() {
    screen.clearBuffer();
    screen.setFont(u8g2_font_ncenB08_tr);
    screen.drawStr(0, 10, "ALTITUDE: ");
    screen.drawStr(80, 10, String(filter.x(0)).c_str());
    screen.drawStr(0, 20, "VELOCITY: ");
    screen.drawStr(80, 20, String(filter.velocityFilter.getRowMean(0)).c_str());
    screen.drawStr(0, 30, "SAT: ");
    screen.drawStr(80, 30, String(gps._gps_data.sat_count).c_str());
    screen.drawStr(0, 40, "DIR: ");
    screen.drawStr(80, 40, String(imu.imu_data.gravity_dir_normlize).c_str());

    screen.sendBuffer();
}

// sat_count = gps._gps_data.sat_count;