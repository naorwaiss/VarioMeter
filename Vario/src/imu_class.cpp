#include "imu_class.h"

IMU::IMU(double dt_sec, float beta) {
    this->dt_sec = dt_sec;
    this->_beta = beta;
    this->_sample_hz = 1.0f / dt_sec;
}

void IMU::initialize_imu() {
    imu_instance.initialize();
    imu_instance.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);  // Set accelerometer full scale to ±8g
    imu_instance.setFullScaleGyroRange(MPU6050_GYRO_FS_2000); // Set gyroscope full scale to ±1000°/s
    imu_instance.CalibrateAccel();
    imu_instance.CalibrateGyro();

    if (!imu_instance.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    } else {
        Serial.println("MPU6050 connection successful");
    }
}

void IMU::imu_read() {
    imu_instance.getMotion6(&imu_data.accel_raw.x, &imu_data.accel_raw.y, &imu_data.accel_raw.z,
                            &imu_data.gyro_raw.x, &imu_data.gyro_raw.y, &imu_data.gyro_raw.z);

    imu_data.accel.x = (float)imu_data.accel_raw.x * ACCEL_SCALE;
    imu_data.accel.y = (float)imu_data.accel_raw.y * ACCEL_SCALE;
    imu_data.accel.z = (float)imu_data.accel_raw.z * ACCEL_SCALE;
    imu_data.gyro.x = (float)imu_data.gyro_raw.x * GYRO_SCALE;
    imu_data.gyro.y = (float)imu_data.gyro_raw.y * GYRO_SCALE;
    imu_data.gyro.z = (float)imu_data.gyro_raw.z * GYRO_SCALE;
}

void IMU::updateMagick(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= _beta * s0;
        qDot2 -= _beta * s1;
        qDot3 -= _beta * s2;
        qDot4 -= _beta * s3;
    }

    q0 += qDot1 * dt_sec;
    q1 += qDot2 * dt_sec;
    q2 += qDot3 * dt_sec;
    q3 += qDot4 * dt_sec;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    anglesComputed = 0;
}

void IMU::computeAngles() {
    angles.roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29578;
    angles.pitch = asinf(-2.0f * (q1 * q3 - q0 * q2)) * 57.29578;
    angles.yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29578 + 180.0f;
    anglesComputed = 1;
}

float IMU::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void IMU::get_gravity() {
    atitude_t euiler_rad;
    euiler_rad.roll = angles.roll * M_PI / 180.0;
    euiler_rad.pitch = angles.pitch * M_PI / 180.0;
    euiler_rad.yaw = angles.yaw * M_PI / 180.0;
    DOM(0, 0) = cos(euiler_rad.pitch);
    DOM(0, 1) = sin(euiler_rad.pitch) * sin(euiler_rad.roll);
    DOM(0, 2) = sin(euiler_rad.pitch) * cos(euiler_rad.roll);
    DOM(1, 0) = 0.0;
    DOM(1, 1) = cos(euiler_rad.roll);
    DOM(1, 2) = -sin(euiler_rad.roll);
    DOM(2, 0) = -sin(euiler_rad.pitch);
    DOM(2, 1) = sin(euiler_rad.roll) * cos(euiler_rad.pitch);
    DOM(2, 2) = cos(euiler_rad.pitch) * cos(euiler_rad.roll);

    Vector3f gravity_der = DOM.transpose() * Vector3f(0.0, 0.0, -1);
    Vector3f acc_data = Vector3f(imu_data.accel.x, imu_data.accel.y, imu_data.accel.z);
    float acc_gravity_dir = gravity_der.dot(acc_data);
    float norm = gravity_der.norm();
    // Calculate gravity vector from Euler angles
}

void IMU::mainIMU() {
    imu_read();
    updateMagick(imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z, imu_data.accel.x, imu_data.accel.y, imu_data.accel.z);
    computeAngles();
    get_gravity();
}
