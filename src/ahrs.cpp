// AHRS sensor initialization, calibration, and orientation updates.
#include <Arduino.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include "ahrs.h"

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "LSM6DS_LIS3MDL.h"

Adafruit_NXPSensorFusion filter;

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
Adafruit_Sensor_Calibration_EEPROM cal;
#else
Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10

uint32_t timestamp;

void setupAHRS() {
	while (!Serial) {
		yield();
	}

	Serial.println(F("Adafruit AHRS - IMU Calibration!"));

	if (!cal.begin()) {
		Serial.println("Failed to initialize calibration helper");
		while (1) {
			yield();
		}
	}

	if (!cal.loadCalibration()) {
		Serial.println("No calibration loaded/found... will start with defaults");
	}

	if (!init_sensors()) {
		Serial.println("Failed to find sensors");
		while (1) {
			delay(10);
		}
	}

	accelerometer->printSensorDetails();
	gyroscope->printSensorDetails();
	magnetometer->printSensorDetails();

	setup_sensors();
	filter.begin(FILTER_UPDATE_RATE_HZ);
	timestamp = millis();
	Wire.setClock(400000);
}

void updateAHRS() {
	static uint8_t counter = 0;

	if (millis() - timestamp < (1000 / FILTER_UPDATE_RATE_HZ)) {
		return;
	}
	timestamp = millis();

	sensors_event_t accel, gyro, mag;
	accelerometer->getEvent(&accel);
	gyroscope->getEvent(&gyro);
	magnetometer->getEvent(&mag);

	cal.calibrate(mag);
	cal.calibrate(accel);
	cal.calibrate(gyro);

	float gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
	float gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
	float gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

	filter.update(gx, gy, gz,
	              accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
	              mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

	if (counter++ <= PRINT_EVERY_N_UPDATES) {
		return;
	}
	counter = 0;

	float roll = filter.getRoll();
	float pitch = filter.getPitch();
	float heading = filter.getYaw();
	Serial.print("Orientation: ");
	Serial.print(heading);
	Serial.print(", ");
	Serial.print(pitch);
	Serial.print(", ");
	Serial.println(roll);

	float qw, qx, qy, qz;
	filter.getQuaternion(&qw, &qx, &qy, &qz);
	Serial.print("Quaternion: ");
	Serial.print(qw, 4);
	Serial.print(", ");
	Serial.print(qx, 4);
	Serial.print(", ");
	Serial.print(qy, 4);
	Serial.print(", ");
	Serial.println(qz, 4);
}
