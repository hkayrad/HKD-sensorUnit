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

#include "HKD_IMU_Data_Processing.h"

IMU::IMU(int imuAddressInput, int calibrationDelayInput) {
	imuAddress = imuAddressInput;
	calibrationDelay = calibrationDelayInput;
	dt = 0.02;
}

void IMU::startIMU() {
	// Start IntegratedIMU
	while (imu.begin(imuAddress) != IMU_SUCCESS) {
		Serial.println("IMU Connection failed");
		delay(200);
	}

	/* ------------------------ Gyro calibration sequence ----------------------- */

	Serial.println("Gyro calibration will start please don't touch the plane");
	delay(calibrationDelay);
	Serial.println("Gyro calibration started");

	int gyroCalibrationIteration = 0;

	for (int i = 0; i < 3; i++) { // Set gyro error values to 0
		Data.gyroErrorPRY[i] = 0;
	}

	for (gyroCalibrationIteration = 0; gyroCalibrationIteration < 2000;
			 gyroCalibrationIteration++) {
		float *allAxesFloatData = new float[7]; // All axes float values
		imu.readAllAxesFloatData(allAxesFloatData);
		for (int i = 0; i < 3;
				 i++) { // Assign values to their corresponding variables
			Data.gyroErrorPRY[i] += allAxesFloatData[i];
		}
		delete[] allAxesFloatData; // Free the memory of allAxes
		delay(1);
	}

	for (int i = 0; i < 3; i++) { // Calculate average gyro error values
		Data.gyroErrorPRY[i] /= gyroCalibrationIteration;
	}

	Serial.println("Gyro calibration finished");

	/* ------------------------- End caliration sequence ------------------------ */
}

void IMU::readData() {
	float *allAxesFloatData = new float[7]; // All axes float values
	imu.readAllAxesFloatData(allAxesFloatData);
	for (int i = 0; i < 3;
			 i++) { // Assign values to their corresponding variables
		Data.gyroPRY[i] = ((allAxesFloatData[i] - Data.gyroErrorPRY[i]) > 1 ||
											 (allAxesFloatData[i] - Data.gyroErrorPRY[i]) < -1)
													? allAxesFloatData[i] - Data.gyroErrorPRY[i]
													: 0;
		Data.accelG[i] = (allAxesFloatData[i + 3]); // Error correction
		Data.accelMps2[i] = Data.accelG[i] * g;			// Convert acceleration to m/s^2
	}

	calculateAngle();

	/* ------------------- Filter data using 1D Kalman Filter ------------------- */

	kalman1DFilter(Data.kalmanAnglePR[0],
								 Data.kalmanUncertainityAnglePR[0], Data.gyroPRY[0],
								 Data.anglePRY[0]);
	Data.kalmanAnglePR[0] = Data.kalman1DOutput[0];
	Data.kalmanUncertainityAnglePR[0] = Data.kalman1DOutput[1];

	kalman1DFilter(Data.kalmanAnglePR[1],
								 Data.kalmanUncertainityAnglePR[1], Data.gyroPRY[1],
								 Data.anglePRY[1]);
	Data.kalmanAnglePR[1] = Data.kalman1DOutput[0];
	Data.kalmanUncertainityAnglePR[1] = Data.kalman1DOutput[1];

	/* ------------------------------- End filter ------------------------------- */

	/* ------------------------- Low cost sensor fusion ------------------------ */

	SFAccelPR[0] = atan2((Data.accelG[1] / 256), (Data.accelG[2] / 256) * 180 / PI);
	SFGyroPR[0] = SFGyroPR[0] + (Data.gyroPRY[0] / 14.375) * dt;

	SFCompPR[0] = SFGyroPR[0];
	SFPredictedPR[0] = SFPredictedPR[0] + (Data.gyroPRY[0] / 14.375) * dt;

	SFAccelPR[1] = atan2((Data.accelG[0] / 256), (Data.accelG[2] / 256) * 180 / PI);
	SFGyroPR[1] = SFGyroPR[1] - (Data.gyroPRY[1] / 14.375) * dt;

	SFCompPR[1] = SFGyroPR[1];
	SFPredictedPR[1] = SFPredictedPR[1] + (Data.gyroPRY[1] / 14.375) * dt;

	P00 += dt * (2 * P01 + dt * P11); // Projected error covariance terms from derivation result: Time Update step 2
	P01 += dt * P11;									// Projected error covariance terms from derivation result: Time Update step 2
	P00 += dt * Q;										// Projected error covariance terms from derivation result: Time Update step 2
	P11 += dt * Q;										// Projected error covariance terms from derivation result: Time Update step 2
	Kk0 = P00 / (P00 + R);						// Measurement Update step 1
	Kk1 = P01 / (P01 + R);						// Measurement Update step 1

	SFPredictedPR[0] += (SFAccelPR[0] - SFPredictedPR[0]) * Kk0; // Measurement Update step 2
	SFPredictedPR[1] += (SFAccelPR[1] - SFPredictedPR[1]) * Kk0; // Measurement Update step 2

	P00 *= (1 - Kk0); // Measurement Update step 3
	P01 *= (1 - Kk1); // Measurement Update step 3
	P11 -= Kk1 * P01; // Measurement Update step 3

	float alpha = 0.98;
	SFCompPR[0] = alpha * (SFCompPR[0] + SFCompPR[0] * dt) + (1.0 - alpha) * SFAccelPR[0]; // Complimentary filter
	SFCompPR[1] = alpha * (SFCompPR[1] + SFCompPR[1] * dt) + (1.0 - alpha) * SFAccelPR[1]; // Complimentary filter

	/* ---------------------------- End sensor fusion --------------------------- */

	time = millis() - time;
	time = (dt * 1000) - time;
	delay(time);
}

void IMU::calculateAngle() {
	Data.anglePRY[0] =
			atan(Data.accelG[1] / (sqrt(Data.accelG[0] * Data.accelG[0] + Data.accelG[2] * Data.accelG[2]))) *
			180 / PI;
	Data.anglePRY[1] =
			-atan(Data.accelG[0] / (sqrt(Data.accelG[1] * Data.accelG[1] + Data.accelG[2] * Data.accelG[2]))) *
			180 / PI;
}

void IMU::kalman1DFilter(float KalmanState, float KalmanUncertainity, float KalmanInput, float KalmanMeasurement) {
	KalmanState = KalmanState + 0.004 * KalmanInput;
	KalmanUncertainity = KalmanUncertainity + 0.004 * 4 * 4;
	float KalmanGain = KalmanUncertainity / (KalmanUncertainity + 3 * 3);
	KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
	KalmanUncertainity = (1 - KalmanGain) * KalmanUncertainity;
	Data.kalman1DOutput[0] = KalmanState;
	Data.kalman1DOutput[1] = KalmanUncertainity;
}