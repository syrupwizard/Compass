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
#define PRINT_EVERY_N_UPDATES 1

uint32_t timestamp;

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
  gx = 0.0f;
  gy = 0.0f;
  gz = 0.0f;

  // produces:
  //cw rotation yaw with 180 flip
  //ccw pitch
  //cw roll
  // filter.update(gx, gy, gz,
  //               -accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
  //               -mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

   // produces:
  //ccw  yaw
  //ccw pitch
  //cw roll
  filter.update(gx, gy, gz,
                 accel.acceleration.x, 
                -accel.acceleration.y, 
                -accel.acceleration.z,
                 mag.magnetic.x, 
                -mag.magnetic.y, 
                -mag.magnetic.z);

  // Same print cadence as before (counter++ <= N returns early)
  // CHANGED: only print when a USB host is attached
  if (counter++ > PRINT_EVERY_N_UPDATES) {
    counter = 0;

    // if (Serial) {
    //   float roll = filter.getRoll();
    //   float pitch = filter.getPitch();
    //   float heading = filter.getYaw();
    //   Serial.print("Orientation: ");
    //   Serial.print(heading);
    //   Serial.print(", ");
    //   Serial.print(pitch);
    //   Serial.print(", ");
    //   Serial.println(roll);

      float qw, qx, qy, qz;
      filter.getQuaternion(&qw, &qx, &qy, &qz);
      //qx = -qx; qy = -qy; qz = -qz;   // conjugate: world->board becomes board->world
      Serial.print("Quaternion: ");
      Serial.print(qw, 4);
      Serial.print(", ");
      Serial.print(qx, 4);
      Serial.print(", ");
      Serial.print(qy, 4);
      Serial.print(", ");
      Serial.println(qz, 4);
      //Serial.println("Inverted");
    }
  }

  return true;
}

// NEW: lets the main sketch grab the latest quaternion
void getAHRSQuaternion(float *w, float *x, float *y, float *z) {
  filter.getQuaternion(w, x, y, z);
}
#define MAGNETIC_DECLINATION_DEG  -14.5f //14.5 degrees E, subtract from mag N to get true N
// NEW: lets the main sketch grab the latest heading, TEMP FIX TIL I FIGURE OUT Qs
void getAHRSHeading(float *heading) {
  float raw = filter.getYaw();
  *heading = raw;
  // float adjusted = 360.0f - (raw + MAGNETIC_DECLINATION_DEG);
  // adjusted = fmodf(adjusted, 360.0f);
  // if (adjusted < 0.0f) {
  //   adjusted += 360.0f;
  // }
  // *heading = adjusted;  // mirrors direction, keeps 0°=North fixed, and keeps heading in [0, 360)
}
