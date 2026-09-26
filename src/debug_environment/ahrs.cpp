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
#define PRINT_EVERY_N_UPDATES 4

uint32_t timestamp;
float latest_ax, latest_ay, latest_az;
float latest_gx, latest_gy, latest_gz;
float latest_mx, latest_my, latest_mz;

void setupAHRS() {
  // CHANGED: wait for USB serial at most 2 s, so the board still boots on battery
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) {
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

Adafruit_Sensor *getAHRSAccelerometer() { return accelerometer; }
Adafruit_Sensor *getAHRSGyroscope() { return gyroscope; }
Adafruit_Sensor *getAHRSMagnetometer() { return magnetometer; }
Adafruit_Sensor_Calibration *getAHRSCalibration() { return &cal; }

// CHANGED: returns bool (true = filter ran this call) instead of void
bool updateAHRS() {
  static uint8_t counter = 0;

  if (millis() - timestamp < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return false;
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

  
  //gx -> roll
   //-gy -> pitch
  //-gz -> yaw
 

float gyro_roll = gx;
float gyro_pitch = -gy;
float gyro_yaw = -gz;


  // filter.update( gyro_roll, gyro_pitch, gyro_yaw,
  //               -accel.acceleration.x, 
  //               accel.acceleration.y, 
  //               accel.acceleration.z,
  //               mag.magnetic.x, 
  //               -mag.magnetic.y, 
  //               mag.magnetic.z);
  // return true;
 filter.update( gyro_roll, gyro_pitch, gyro_yaw,
                -accel.acceleration.x, 
                accel.acceleration.y, 
                accel.acceleration.z,
                mag.magnetic.x, 
                -mag.magnetic.y, 
                mag.magnetic.z);
  return true;

}

// NEW: lets the main sketch grab the latest quaternion
void getAHRSQuaternion(float *w, float *x, float *y, float *z) {
  filter.getQuaternion(w, x, y, z);
}

//lets the main sketch grab the latest heading, TEMP FIX TIL I FIGURE OUT Qs
void getAHRSHeading(float *yaw, float *pitch, float *roll) {
  *yaw = filter.getYaw();
  *pitch = filter.getPitch();
  *roll = filter.getRoll();

  //correct but not correct
  // *yaw = fmodf(360.0f - rawYaw, 360.0f);
  // *pitch = -filter.getPitch();
  // *roll = filter.getRoll();
}