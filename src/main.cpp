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

//* CONSTANTS
#define SSID "NGTelemetryNetwork"
#define PASSWORD "99733940"
#define INTEGRATED_IMU_ADDRESS 0x6B

//* GLOBAL VARIABLES
IMU IntegratedIMU(INTEGRATED_IMU_ADDRESS, 500);

void setup() {
	Serial.begin(115200);
	Serial.println("Serial started.");

	IntegratedIMU.startIMU();
}

void loop() {
}