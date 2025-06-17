#ifndef BARO_CLASS_H
#define BARO_CLASS_H

#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>

struct baro_data {
  float pressure;
  float temperature;
  float altitude;
  float prev_altitude;
  float altitude_speed;
};

class Baro {
  public:
    Baro(float dt_hz);
    baro_data baro_data;
    void initialize_baro();
    void read_baro();
    void calculate_altitude_speed();
  private:
    Adafruit_DPS310 baro;
    float _dt_sec;
};

#endif // BARO_CLASS_H
