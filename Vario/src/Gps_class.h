#ifndef GPS_CLASS_H
#define GPS_CLASS_H

#include <SparkFun_u-blox_GNSS_v3.h>  //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
#include <HardwareSerial.h>
#include <Arduino.h>

#define INIT_SAT_COUNT 6

#define GPS_SERIAL Serial0

struct gps_data_t {
    float sat_count;
    float latitude;
    float longitude;
    float altitude;
    float pre_altitude;
    float previous_altitude;
    float altitude_speed;
    float speed;
    float heading;
    float year;
    float month;
    float day;
    float hour;
    float minute;
    float second;
    float Vdop;
};

class GPS {
   public:
    GPS( double dt_sec);
    gps_data_t _gps_data;
    void initialize_gps();
    void read_gps();

   private:
    SFE_UBLOX_GNSS_SERIAL _myGPS;  // Create GPS object
    bool _fix;
    double _dt_sec;
};

#endif
