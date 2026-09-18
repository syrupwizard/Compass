// Arduino Example Code snippet
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "battery_charging.h"
#define VBATPIN A6

Adafruit_NeoPixel batteryLed(NEOPIXEL_NUM, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void updateBatteryLed(float voltage) {
	static bool flashOn = false;
	uint32_t color;

	if (voltage >= 3.8) {
		color = batteryLed.Color(0, 32, 0);
	} else if (voltage >= 3.5) {
		color = batteryLed.Color(32, 16, 0);
	} else {
		color = batteryLed.Color(32, 0, 0);
	}

	if (voltage > 4.1) {
		batteryLed.setPixelColor(0, batteryLed.Color(0, 32, 0));
	} else {
		flashOn = !flashOn;
		batteryLed.setPixelColor(0, flashOn ? color : 0);
	}
	batteryLed.show();
}

void setupBatteryCharging() {
	batteryLed.begin();
	batteryLed.setBrightness(64);
	batteryLed.clear();
	batteryLed.show();
}

void updateBatteryCharging() {
	static uint32_t lastUpdate = 0;
	if (millis() - lastUpdate < 1000) {
		return;
	}
	lastUpdate = millis();

	float measuredvbat = analogRead(VBATPIN);
	measuredvbat *= 2;
	measuredvbat *= 3.6;
	measuredvbat /= 1024;
	updateBatteryLed(measuredvbat);

	//Serial.print("VBATPIN: ");
	//Serial.println(VBATPIN);
	Serial.print("VBat: ");
	Serial.println(measuredvbat);

	//delay(1000);
}
