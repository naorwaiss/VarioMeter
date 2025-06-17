#include "Baro_class.h"

Baro::Baro(float dt_hz) {
    _dt_sec = 1.0f / dt_hz;
}

void Baro::initialize_baro() {
    if (!baro.begin_I2C()) {
        Serial.println("Failed to find DPS310 chip");
        while (1);
    }
    baro.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    baro.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
}

void Baro::read_baro() {
    sensors_event_t pressure_event, temp_event;
    baro.getEvents(&pressure_event, &temp_event);
    baro_data.pressure = pressure_event.pressure;
    baro_data.temperature = temp_event.temperature;
    baro_data.altitude = baro.readAltitude();
    calculate_altitude_speed();
}

void Baro::calculate_altitude_speed() {
    baro_data.altitude_speed = (baro_data.altitude - baro_data.prev_altitude) / _dt_sec;
    baro_data.prev_altitude = baro_data.altitude;
}
