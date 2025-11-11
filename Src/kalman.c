//
// Created by dlin2 on 2025/7/23.
//

#include "kalman.h"

void Kalman_Init(KalmanData* k) {
    k->angle = 0.0f;
    k->bias = 0.0f;

    k->P[0][0] = 1.0f;
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;
    k->P[1][1] = 1.0f; // initialize to I

    k->Q_angle = 0.001f;
    k->Q_bias = 0.003f;
    k->R_angle = 0.5f;
}

void Kalman_Init_Yaw(KalmanData* k) {
    k->angle = 0.0f;
    k->bias = 0.0f;

    k->P[0][0] = 1.0f;
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;
    k->P[1][1] = 1.0f; // initialize to I

    k->Q_angle = 0.01f;
    k->Q_bias = 0.005f;
    k->R_angle = 0.2f;
}

// version 2, 2D state vector, roll and gyro bias. Used commonly by 3 euler angle
float Kalman_Update(KalmanData* k, float dt, float new_angle_acc, float input_gyrorate) {

    // state vector             x = [theta, bias]^T
    // measurement              z = theta_acc
    // control input            u = omega_gyro
    // state transition matrix  A = [1 dt][0  1]
    // control input matrix     B = [dt 0]^T
    // mapping matrix           H = [1 0]

    // predict state & error covariance
    float pred_angle = k->angle + dt * (input_gyrorate - k->bias); // input_gyrorate as control input, angular velocity
    float pred_bias = k->bias;

    // error convariance
    float pred_P[2][2];
    pred_P[0][0] = k->P[0][0] - (k->P[0][1] + k->P[1][0])*dt + k->P[1][1]*dt*dt + k->Q_angle;
    pred_P[0][1] = k->P[0][1] - k->P[1][1] * dt;
    pred_P[1][0] = k->P[1][0] - k->P[1][1] * dt;
    pred_P[1][1] = k->P[1][1] + k->Q_bias;

    // compute Kalman gain
    float kg[2]; // 2by1 matrix
    float tmp = pred_P[0][0] + k->R_angle;
    kg[0] = pred_P[0][0] / tmp;
    kg[1] = pred_P[1][0] / tmp;

    // update estimates
    k->angle = pred_angle + kg[0] * (new_angle_acc - pred_angle);
    k->bias = pred_bias + kg[1] * (new_angle_acc - pred_angle);

    // update error covariance
    k->P[0][0] = (1 - kg[0]) * pred_P[0][0];
    k->P[1][0] = (1 - kg[0]) * pred_P[0][1];
    k->P[0][1] = pred_P[1][0] - kg[1] * pred_P[0][0];
    k->P[1][1] = pred_P[1][1] - kg[1] * pred_P[0][1];
    k->P[0][1] = k->P[1][0] = (k->P[0][1] + k->P[1][0]) / 2.0f; // for symmetry

    return k->angle;
}

struct bar_d {
    float    pre_pa;
    float    temp_c;
    uint16_t osr_d1;
    uint16_t osr_d2;
} BaroData;