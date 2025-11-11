//
// Created by dlin2 on 2025/7/23.
//

#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float angle; // estimated angle
    float bias; // gyro bias


    float P[2][2]; // error covariance matrix
    float Q_angle; // process noise
    float Q_bias;
    float R_angle; // measurement noise from acc

} KalmanData;

void Kalman_Init(KalmanData* k);
void Kalman_Init_Yaw(KalmanData* k);
float Kalman_Update(KalmanData* k, float dt, float new_angle_acc, float input_gyrorate);

#endif //KALMAN_H
