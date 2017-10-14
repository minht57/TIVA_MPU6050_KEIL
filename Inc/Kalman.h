/*
 * Kalman.h
 *
 *  Created on: May 13, 2015
 *      Author: CTCNGH
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

//Define Kalman struct
typedef struct
{
    /* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
}
Kalman;

void initKalman(Kalman *KalmanStruct);

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getAngle(Kalman *KalmanStruct, float newAngle, float newRate, float dt);

void setAngle(Kalman *KalmanStruct, float angle); // Used to set angle, this should be set as the starting angle
float getRate(Kalman KalmanStruct); // Return the unbiased rate

/* These are used to tune the Kalman filter */
void setQangle(Kalman *KalmanStruct, float Q_angle);
void setQbias(Kalman *KalmanStruct, float Q_bias);
void setRmeasure(Kalman *KalmanStruct, float R_measure);

float getQangle(Kalman KalmanStruct);
float getQbias(Kalman KalmanStruct);
float getRmeasure(Kalman KalmanStruct);


#endif /* KALMAN_H_ */
