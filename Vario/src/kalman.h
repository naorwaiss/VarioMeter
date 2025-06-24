#ifndef KALMAN_H
#define KALMAN_H
#include "imu_class.h"
#include "Gps_class.h"
#include "Baro_class.h"
#include "eigen.h"
#include <Eigen/Dense>
#include <Arduino.h>

enum class Q_R {
    PREDICT = 1,
    UPDATE = 2
};

enum class BARO_GPS {
    BARO = 1,
    GPS = 2
};

struct MeanFilter {
    /*
    struct the andle the motor speed sample and make mean filter to the data -> for type 1
    */
    int sample;
    Eigen::MatrixXf sum;

    MeanFilter(int sample)
        : sample(sample), sum(Eigen::MatrixXf::Zero(1, sample)) {}

    void printMatrix() const {
        for (int i = 0; i < sum.rows(); ++i) {
            for (int j = 0; j < sum.cols(); ++j) {
                Serial.print(sum(i, j), 4);
                Serial.print("\t");
            }
            Serial.println();
        }
        Serial.println();
    }

    // Push a new value to a specific motor row
    void pushSample(int motor_index, float value) {
        if (motor_index < 0 || motor_index >= sum.rows()) return;

        // Shift all values to the right
        for (int j = sample - 1; j > 0; --j) {
            sum(motor_index, j) = sum(motor_index, j - 1);
        }
        sum(motor_index, 0) = value;
    }

    float getRowMean(int motor_index) const {
        if (motor_index < 0 || motor_index >= sum.rows()) return 0.0f;

        float row_sum = 0.0f;
        for (int j = 0; j < sample; ++j) {
            row_sum += sum(motor_index, j);
        }
        return row_sum / sample;
    }
};


class Kalman {
   public:
    Kalman(GPS *gps, Baro *baro, IMU *imu);  // Initialize with 20 samples
    void initialize();
    void predict();
    void update(float z_ati_mes, float R_ati_mes);
    void R_factor_function();
    void calc_all();
    void Kalman_QR_calc();
    void calc_R();
    void constarin_all();
    void simple_R();
    void Q_factor_function();

    Q_R Q_R_state = Q_R::PREDICT;  // Current state of the Kalman filter
    BARO_GPS baro_gps_state = BARO_GPS::BARO;
    float r_learn;  
    float R_baro;  // Measurement noise covariance
    float R_gps;   // Measurement noise covariance
    float Q_factor;
    Vector2f x;    // State vector
    MeanFilter velocityFilter;  // Mean filter for velocity smoothing
    MeanFilter imu_filter;
    GPS *_gps;
    Baro *_baro;
    IMU *_imu;


   private:
    Matrix<float, 2, 2> F;  // State transition matrix
    Vector2f B;             // Control input matrix
    Matrix<float, 2, 2> P;  // Error covariance matrix
    Matrix<float, 2, 2> Q;  // Process noise covariance
    RowVector2f H;  // Measurement function
    float y;
    Vector2f K;  // Kalman gain
    
    // MPU6050 specific noise parameters
    const float ACCEL_NOISE = 0.16f;   // (0.4 m/s²)²
    const float GYRO_NOISE = 2.5e-5f;  // (0.005 rad/s)²
};



#endif