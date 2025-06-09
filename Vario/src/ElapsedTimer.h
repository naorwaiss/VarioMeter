#ifndef ELAPSED_TIMER_H
#define ELAPSED_TIMER_H

#include <Arduino.h>

class ElapsedTimer {
public:
    ElapsedTimer(float dt_hz) {
        reset();
        _dt_hz = dt_hz;
        _dt_micros = 1000000.0f / _dt_hz;
    }

    void reset() {
        _loop_time = 0;
        _loop_time = micros();
    }

    bool hasElapsed() {
        bool flag = (micros() - _loop_time) >= _dt_micros;
        return flag;
    }

private:
    float _dt_hz;
    double _dt_micros;
    unsigned long _loop_time;
};

#endif 