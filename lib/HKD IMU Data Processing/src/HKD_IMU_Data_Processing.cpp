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
	time = millis();
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

void IMU::plotValues() { // Plot values to the plotter
	Serial.println(">Gyro Pitch [°/s]: " + String(Data.gyroPRY[0]));
	Serial.println(">Gyro Roll [°/s]: " + String(Data.gyroPRY[1]));
	Serial.println(">Gyro Yaw [°/s]: " + String(Data.gyroPRY[2]));
	Serial.println(">Accel X [m/s^2]: " + String(Data.accelMps2[0]));
	Serial.println(">Accel Y [m/s^2]: " + String(Data.accelMps2[1]));
	Serial.println(">Accel Z [m/s^2]: " + String(Data.accelMps2[2]));
	Serial.println(">Angle Pitch [°]: " + String(Data.anglePRY[0]));
	Serial.println(">Angle Roll [°]: " + String(Data.anglePRY[1]));
	Serial.println(">Angle Yaw [°]: " + String(Data.anglePRY[2]));
	Serial.println(">Kalman Angle Pitch [°]: " +
								 String(Data.kalmanAnglePR[0]));
	Serial.println(">Kalman Angle Roll [°]: " +
								 String(Data.kalmanAnglePR[1]));
	delay(10);
}