/*
 * Kalman.c
 *
 *  Created on: May 13, 2015
 *      Author: CTCNGH
 */

#include "Kalman.h"

void initKalman(Kalman *KalmanStruct)
{
	/* We will set the variables like so, these can also be tuned by the user */
	(*KalmanStruct).Q_angle = 0.001f;
	(*KalmanStruct).Q_bias = 0.003f;
	(*KalmanStruct).R_measure = 0.03;

	(*KalmanStruct).angle = 0.0f; // Reset the angle
	(*KalmanStruct).bias = 0.0f; // Reset bias

	(*KalmanStruct).P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
	(*KalmanStruct).P[0][1] = 0.0f;
	(*KalmanStruct).P[1][0] = 0.0f;
	(*KalmanStruct).P[1][1] = 0.0f;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getAngle(Kalman *KalmanStruct, float newAngle, float newRate, float dt)
{

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	(*KalmanStruct).rate= newRate - (*KalmanStruct).bias;
	(*KalmanStruct).angle += dt * (*KalmanStruct).rate;

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	(*KalmanStruct).P[0][0] += dt * (dt*(*KalmanStruct).P[1][1] - (*KalmanStruct).P[0][1] - (*KalmanStruct).P[1][0] + (*KalmanStruct).Q_angle);
	(*KalmanStruct).P[0][1] -= dt * (*KalmanStruct).P[1][1];
	(*KalmanStruct).P[1][0] -= dt * (*KalmanStruct).P[1][1];
	(*KalmanStruct).P[1][1] += (*KalmanStruct).Q_bias * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	float S = (*KalmanStruct).P[0][0] + (*KalmanStruct).R_measure; // Estimate error
	/* Step 5 */
	float K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = (*KalmanStruct).P[0][0] / S;
	K[1] = (*KalmanStruct).P[1][0] / S;

	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	float y = newAngle - (*KalmanStruct).angle; // Angle difference
	/* Step 6 */
	(*KalmanStruct).angle += K[0] * y;
	(*KalmanStruct).bias += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	float P00_temp = (*KalmanStruct).P[0][0];
	float P01_temp = (*KalmanStruct).P[0][1];

	(*KalmanStruct).P[0][0] -= K[0] * P00_temp;
	(*KalmanStruct).P[0][1] -= K[0] * P01_temp;
	(*KalmanStruct).P[1][0] -= K[1] * P00_temp;
	(*KalmanStruct).P[1][1] -= K[1] * P01_temp;

	return (*KalmanStruct).angle;
}

// Used to set angle, this should be set as the starting angle
void setAngle(Kalman *KalmanStruct, float angle)
{
	(*KalmanStruct).angle=angle;
}

// Return the unbiased rate
float getRate(Kalman KalmanStruct)
{
	return KalmanStruct.rate;
}

/* These are used to tune the Kalman filter */
void setQangle(Kalman *KalmanStruct, float Q_angle)
{
	(*KalmanStruct).Q_angle=Q_angle;
}
void setQbias(Kalman *KalmanStruct, float Q_bias)
{
	(*KalmanStruct).Q_bias=Q_bias;
}
void setRmeasure(Kalman *KalmanStruct, float R_measure)
{
	(*KalmanStruct).R_measure=R_measure;
}

float getQangle(Kalman KalmanStruct)
{
	return KalmanStruct.Q_angle;
}
float getQbias(Kalman KalmanStruct)
{
	return KalmanStruct.Q_bias;
}
float getRmeasure(Kalman KalmanStruct)
{
	return KalmanStruct.R_measure;
}



