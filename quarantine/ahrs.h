#pragma once

#include <Adafruit_Sensor.h>
#include <Adafruit_Sensor_Calibration.h>

void setupAHRS();

Adafruit_Sensor *getAHRSAccelerometer();
Adafruit_Sensor *getAHRSGyroscope();
Adafruit_Sensor *getAHRSMagnetometer();
Adafruit_Sensor_Calibration *getAHRSCalibration();

// Call as often as you like. Returns true only on calls where the filter
// actually ran (every 1/FILTER_UPDATE_RATE_HZ seconds).
bool updateAHRS();

// Latest quaternion from the fusion filter.
void getAHRSQuaternion(float *w, float *x, float *y, float *z);

// NEW: latest fused heading (yaw), 0-360 degrees.
void getAHRSHeading(float *yaw, float *pitch, float *roll);
