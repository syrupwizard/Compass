#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // Keeps USB CDC serial alive

// Onboard LED Pin Definitions
#define RED_LED  13
#define BLUE_LED 7 

void setup() {
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  // Allow time for macOS USB CDC enumeration
  delay(1000); 
  Serial.begin(115200);
}

void loop() {
  // Alternate Red and Blue LEDs
  digitalWrite(RED_LED, HIGH);
  digitalWrite(BLUE_LED, LOW);
  delay(2000);

  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
  delay(2000);

  if (Serial) {
    Serial.println("Board active and USB connected.");
  }
}