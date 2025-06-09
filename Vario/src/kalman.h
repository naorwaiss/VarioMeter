#ifndef KALMAN_H
#define KALMAN_H

#include <Arduino.h>

class Kalman {
    public:
        Kalman();
        void initialize();
        void update();
    private:
        float kalman_gain;

};

#endif