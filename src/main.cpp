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
#include <Arduino.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <WiFi.h>

//* CONSTANTS
#define SSID "NGTelemetryNetwork"
#define PASSWORD "99733940"
#define INTEGRATED_IMU_ADDRESS 0x6A

//* GLOBAL VARIABLES
IMU IntegratedIMU(INTEGRATED_IMU_ADDRESS, 500);
HTTPClient http;

const char *ssid = "Redmi Note 12 Pro"; // WiFi SSID
const char *password = "A1234567";			// WiFi Password

//* FUNCTIONS
void initWifi() {
	WiFi.mode(WIFI_STA);				// Set WiFi mode to station
	WiFi.begin(ssid, password); // Connect to WiFi

	Serial.println("Connecting to WiFi....");

	while (WiFi.status() != WL_CONNECTED) { // Check if WiFi is connected
		Serial.println("WiFi connection failed");
		delay(500);
	}

	Serial.println("WiFi connected");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	Serial.println("RSSI: " + String(WiFi.RSSI()) + " dBm");

	delay(3000);
}

void setup() {
	http.begin("http://192.168.95.150:3001/api"); //! Don't forget to change IP
	http.addHeader("Content-Type", "application/json");

	Serial.begin(115200);
	Serial.println("Serial started.");

	IntegratedIMU.startIMU();
	initWifi();
}

void loop() {
	IntegratedIMU.readData();

	String payload =
			"{\"gyro\":{\"x\":" + String(IntegratedIMU.Data.gyroPRY[0]) +
			",\"y\":" + String(IntegratedIMU.Data.gyroPRY[1]) +
			",\"z\":" + String(IntegratedIMU.Data.gyroPRY[2]) +
			"},\"accel\":{\"x\":" + String(IntegratedIMU.Data.accelG[0]) +
			",\"y\":" + String(IntegratedIMU.Data.accelG[1]) +
			",\"z\":" + String(IntegratedIMU.Data.accelG[2]) +
			"} ,\"angle\":{\"roll\":" + String(IntegratedIMU.Data.anglePRY[1]) +
			",\"pitch\":" + String(IntegratedIMU.Data.anglePRY[0]) +
			",\"yaw\":" + String(IntegratedIMU.Data.anglePRY[2]) +
			"},\"timestamp\":" + String(millis()) + "}";
	http.POST(payload);
}