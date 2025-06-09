#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>

struct baro_data {
  float pressure;
  float temperature;
  float altitude;
};

class Baro {
  public:
    baro_data baro_data;
    void initialize_baro();
    void read_baro();
  private:
    Adafruit_DPS310 baro;
};
