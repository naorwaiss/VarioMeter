#include "kalman.h"
#include <functional>


Kalman::Kalman(GPS *gps, Baro *baro, IMU *imu) {
    _gps = gps;
    _baro = baro;
    _imu = imu;
}

void Kalman::initialize() {
    // State transition matrix
    F << 1, _imu->_dt_sec,
        0, 1;

    B << 0.5 * _imu->_dt_sec * _imu->_dt_sec, _imu->_dt_sec;
    float dt2 = _imu->_dt_sec * _imu->_dt_sec;
    float dt3 = dt2 * _imu->_dt_sec;
    float dt4 = dt2 * dt2;
    combine_cov = ACCEL_NOISE * ACCEL_NOISE + GYRO_NOISE * GYRO_NOISE;
    Q(0, 0) = combine_cov * dt4 / 4.0f;
    Q(0, 1) = combine_cov * dt3 / 2.0f;
    Q(1, 0) = Q(0, 1);
    Q(1, 1) = combine_cov * dt2;
    x.setZero();
    P.setIdentity();
    R.setIdentity();
    H << 1, 0;
    z.setZero();
    K.setZero();
    
}

void Kalman::predict() {
    x = F * x + B * _imu->imu_data.gravity_dir;
    P = F * P * F.transpose() + Q;
}

void Kalman::main_kalman(){
    predict();
}