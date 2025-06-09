#include "Baro_class.h"

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
}
