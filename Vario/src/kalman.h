#ifndef KALMAN_H
#define KALMAN_H
#include "imu_class.h"
#include "Gps_class.h"
#include "Baro_class.h"
#include "eigen.h"
#include <Eigen/Dense>
#include <Arduino.h>
#include <functional>

class Kalman {
   public:
    Kalman(GPS *gps, Baro *baro, IMU *imu);
    void initialize();
    void main_kalman();
    using MeasurementCallback = std::function<void(Vector2f&)>;

   private:
    void update();
    void predict();
    GPS *_gps;
    Baro *_baro;
    IMU *_imu;
    Matrix<float, 2, 2> F;  // State transition matrix
    Vector2f x;             // State vector
    Vector2f B;             // Control input matrix
    Matrix<float, 2, 2> P;  // Error covariance matrix
    Matrix<float, 2, 2> Q;  // Process noise covariance
    Matrix<float, 2, 2> R;  // Measurement noise covariance
    float combine_cov;
    RowVector2f H;          // Measurement function
    Vector2f z;             // Measurement vector
    Matrix<float, 2, 2> K;  // Kalman gain
    // MPU6050 specific noise parameters
    const float ACCEL_NOISE = 0.16f;   // (0.4 m/s²)²
    const float GYRO_NOISE = 2.5e-5f;  // (0.005 rad/s)²
};

#endif