#include "Gps_class.h"

GPS::GPS( double dt_sec)
{
    _dt_sec = dt_sec;
}

void GPS::initialize_gps() {
    GPS_SERIAL.begin(115200);
    _myGPS.begin(GPS_SERIAL);
    _myGPS.setUART1Output(COM_TYPE_UBX);  // UBX only (no NMEA)
    _myGPS.setNavigationRate(10000);      // Set update rate to 1Hz (1000ms)
    _myGPS.saveConfiguration();           // Save settings to flash memory
}

void GPS::read_gps() {
    if (_myGPS.getPVT()) {  // Get Position, Velocity, and Time (PVT) in UBX format
        if (_myGPS.getSIV() >= INIT_SAT_COUNT) {
            _fix = true;
            _gps_data.latitude = _myGPS.getLatitude() / 1e7;
            _gps_data.longitude = _myGPS.getLongitude() / 1e7;
            _gps_data.altitude = _myGPS.getAltitude() / 1000.0f;
            _gps_data.speed = _myGPS.getGroundSpeed() / 1000.0f;
            _gps_data.sat_count = _myGPS.getSIV();
            _gps_data.year = _myGPS.getYear();
            _gps_data.month = _myGPS.getMonth();
            _gps_data.day = _myGPS.getDay();
            _gps_data.hour = _myGPS.getHour();
            _gps_data.minute = _myGPS.getMinute();
            _gps_data.second = _myGPS.getSecond();
        }   
        else {
            _gps_data.latitude = 0;
            _gps_data.longitude = 0;
            _gps_data.altitude = 0;
            _gps_data.speed = 0;
            _gps_data.sat_count = 0;
            _fix = false;
            Serial.println("No fix");
        }
    }
}

