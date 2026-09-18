#include <Arduino.h>
#include "ahrs.h"
#include "battery_charging.h"
#include "ble.h"

void setup() {
	Serial.begin(115200);
	setupAHRS();
	setupBatteryCharging();
	setupBLE();
}

void loop() {
	updateAHRS();
	updateBatteryCharging();
	updateBLE();
}
