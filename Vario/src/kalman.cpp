#include "kalman.h"

Kalman::Kalman(GPS *gps, Baro *baro, IMU *imu) : velocityFilter(3), imu_filter(5){
    _gps = gps;
    _baro = baro;
    _imu = imu;
    Q_factor = ACCEL_NOISE;
}

void Kalman::initialize() {
    // State transition matrix
    F << 1, _imu->_dt_sec,
        0, 1;

    // Control input matrix (for acceleration)
    B << 0.5 * _imu->_dt_sec * _imu->_dt_sec,
        _imu->_dt_sec;

    Q.setZero();
    x.setZero();
    P.setIdentity();
    P *= 2.0f;  // Reduced initial uncertainty for faster convergence
    H.setZero();
    H(0, 0) = 1.0f;
    K.setZero();
    _gps->_gps_data.sat_count = 1;
}

void Kalman::predict() {
    imu_filter.pushSample(0, _imu->imu_data.gravity_dir_normlize);
    float x_imu = imu_filter.getRowMean(0);

    if (abs(x_imu) < 0.05) {
        x(1) =0;
    }
    x = F * x + B * x_imu;
    P = F * P * F.transpose() + Q;
}

void Kalman::update(float z_ati_mes, float R_ati_mes) {
    y = z_ati_mes - H * x;
    float H_P_H_T = H * P * H.transpose() + R_ati_mes;
    K = P * H.transpose() * (1 / H_P_H_T);
    x = x + K * y;
    P = (Matrix2f::Identity() - K * H) * P;
    if (abs(y) < 0.05) {
        x(1) *= 0.7;
    }
    velocityFilter.pushSample(0, x(1));
}

void Kalman::Q_factor_function() {
    /// a factor of the imuu
    float acc_g_norm = sqrt((_imu->imu_data.accel.x * _imu->imu_data.accel.x + _imu->imu_data.accel.y * _imu->imu_data.accel.y + _imu->imu_data.accel.z * _imu->imu_data.accel.z)) / 9.81;
    if (acc_g_norm > 0.7 && acc_g_norm < 1.3) {  // stable condition if stable use more the gps  or baro
        Q_R_state = Q_R::PREDICT;
    } else {
        Q_R_state = Q_R::UPDATE;
    }
}

void Kalman::R_factor_function() {
    if (_gps->_gps_data.sat_count > 10) {
        baro_gps_state = BARO_GPS::GPS;
    } else {
        baro_gps_state = BARO_GPS::BARO;
    }
}

void Kalman::calc_all() {
    Q_factor_function();
    R_factor_function();
    Kalman_QR_calc();
    calc_R();
    constarin_all();
}

void Kalman::Kalman_QR_calc() {
    float learn_facton = 0.95;
    float inv_learn_facton = 2 - learn_facton;
    switch (Q_R_state) {
        case Q_R::UPDATE:
            Q_factor *= learn_facton;
            r_learn *= inv_learn_facton;
            return;
        case Q_R::PREDICT:
            Q_factor *= inv_learn_facton;
            r_learn *= learn_facton;
            return;
    }
}

void Kalman::calc_R() {
    float learn_facton = 0.95;
    float inv_learn_facton = 2 - learn_facton;

    switch (baro_gps_state) {
        case BARO_GPS::GPS:
            R_baro *= (inv_learn_facton * r_learn);
            R_gps *= (learn_facton * r_learn);
            return;
        case BARO_GPS::BARO:
            R_baro *= (learn_facton * r_learn);
            R_gps *= (inv_learn_facton * r_learn);
            return;
    }
}

void Kalman::constarin_all(){
    Q_factor = constrain(Q_factor,0.00001,0.001);
    R_baro = constrain(R_baro,0.0001,0.01);
    R_gps = constrain(R_gps,0.001,1.0);
    r_learn = constrain(r_learn,0.8,1.2);
    Q.setIdentity();   
    Q = Q*Q_factor;
}

void Kalman::simple_R(){
    R_gps = 10.0/((_gps->_gps_data.sat_count));
    R_baro = 0.05;
    Q_factor = 0.00005;
    Q.setIdentity();
    Q = Q*Q_factor;
}

