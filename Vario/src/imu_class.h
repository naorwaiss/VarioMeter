#ifndef IMU_CLASS_H
#define IMU_CLASS_H

#include <MPU6050.h>
#include <Wire.h>
#include "eigen.h"
#include <Eigen/Dense>

struct vec16 {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct vec3 {
    float x;
    float y;
    float z;
};

struct imu_data_t {
    vec16 accel_raw;
    vec16 gyro_raw;
    vec3 accel;
    vec3 gyro;
};

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} atitude_t;

typedef struct
{
    vec3 gravity;
    float norm_gravity;
    float norm_accel;
} step_t;

using namespace Eigen;  // Eigen related statement; simplifies syntax for declaration of matrices

class IMU {
   public:
    IMU(double dt_sec, float beta);
    imu_data_t imu_data;
    void initialize_imu();
    void imu_read();
    void updateMagick(float gx, float gy, float gz, float ax, float ay, float az);
    void computeAngles();
    void mainIMU();
    void get_gravity();
    Matrix3f DOM;
    atitude_t angles;

   private:
    MPU6050 imu_instance;
    static float invSqrt(float x);
    char anglesComputed;
    // Scale factors for ±4g range
    const float ACCEL_SCALE = 9.81f / 8192.0f;    // Convert to m/s² (8192 LSB/g)
    const float GYRO_SCALE = 0.0174533f / 65.5f;  // Convert to rad/s (65.5 LSB/(°/s))

    double dt_sec;
    float q0 = 1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
    float _beta;
    float _sample_hz;
};

#endif
