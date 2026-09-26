// main file which implements the BLE interface and calls ahrs and calibration modules
// 
#include <bluefruit.h>
#include <math.h>
#include "ahrs.h"
#include "sensor_calibration/sensor_calibration.h"

#define SEND_EVERY_N_UPDATES  10           // output every 100 Hz filter update


uint8_t send_count   = 0;
bool usbCalibrationMode = false;
bool recordingGyroMovement = false;

void setup()
{
  Serial.begin(115200);

  setupAHRS();   // loads saved calibration, inits sensors + filter
  delay(2000);
}

void loop()
{
  if (!updateAHRS()) {
    return;
  }

  if (++send_count < SEND_EVERY_N_UPDATES) {
    return;
  }
  send_count = 0;

  float qw, qx, qy, qz;
  getAHRSQuaternion(&qw, &qx, &qy, &qz);

  float yaw, pitch, roll;
  getAHRSHeading(&yaw, &pitch, &roll);

  // Serial.print(millis()); Serial.print(',');
  Serial.print("Quaternion: ");
  Serial.print(qw, 4); Serial.print(',');
  Serial.print(qx, 4); Serial.print(',');
  Serial.print(qy, 4); Serial.print(',');
  Serial.print(qz, 4); Serial.println();

  Serial.print(yaw, 3); Serial.print(',');
  Serial.print(pitch, 3); Serial.print(',');
  Serial.print(roll, 3); Serial.println();
}