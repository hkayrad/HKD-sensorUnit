/************************************************************************
																																				Source
Code Form License Notice
																								-------------------------------------------

	This Source Code Form is subject to the terms of the Mozilla Public
	License, v. 2.0. If a copy of the MPL was not distributed with this
	file, You can obtain one at http://mozilla.org/MPL/2.0/.

If it is not possible or desirable to put the notice in a particular
file, then You may include the notice in a location (such as a LICENSE
file in a relevant directory) where a recipient would be likely to look
for such a notice.
*************************************************************************/

#ifndef HKD_IMU_DATA_PROCESSING
#define HKD_IMU_DATA_PROCESSING

#include <Arduino.h>
#include <Deneyap_6EksenAtaletselOlcumBirimi.h>

//* CONSTANTS
const float g = 9.8066;

struct data {
	float gyroPRY[3], gyroErrorPRY[3], accelG[3], accelMps2[3], anglePRY[3];
	float kalman1DOutput[2] = {0, 0}, kalmanAnglePR[2], kalmanUncertainityAnglePR[2];
	int magnetometer[3];
	int altitutde;
	unsigned long tZero;
};

class IMU {
	public:
	IMU(int, int);

	int time;
	float dt;
	LSM6DSM imu;
	int imuAddress, calibrationDelay;
	struct data Data;

	float SFAccelPR[2] = {0, 0}, SFGyroPR[2] = {0, 0}, SFPredictedPR[2] = {0, 0}, SFCompPR[2] = {0, 0};
	float Predicted_roll = 0; // Output of Kalman filter
	float Q = 0.1;						// Prediction Estimate Initial Guess
	float R = 5;							// Prediction Estimate Initial Guess
	float P00 = 0.1;					// Prediction Estimate Initial Guess
	float P11 = 0.1;					// Prediction Estimate Initial Guess
	float P01 = 0.1;					// Prediction Estimate Initial Guess
	float Kk0, Kk1;


	void startIMU();
	void readData();

	private:
	void calculateAngle();
	void kalman1DFilter(float, float, float, float);
};

#endif