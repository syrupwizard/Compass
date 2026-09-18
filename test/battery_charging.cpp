// Arduino Example Code snippet
#include <Arduino.h>
#include <Wire.h>
#define VBATPIN A6



void setup() {
	Serial.begin(115200);

	float measuredvbat = analogRead(VBATPIN);
	measuredvbat *= 2;    // we divided by 2, so multiply back
	measuredvbat *= 3.6;  // Multiply by 3.6V, our reference voltage
	measuredvbat /= 1024; // convert to voltage
	Serial.print("VBat: ");
	Serial.println(measuredvbat);
}

void loop() {
}