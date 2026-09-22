#pragma once

void setupAHRS();

// Call as often as you like. Returns true only on calls where the filter
// actually ran (every 1/FILTER_UPDATE_RATE_HZ seconds).
bool updateAHRS();

// Latest quaternion from the fusion filter.
void getAHRSQuaternion(float *w, float *x, float *y, float *z);

// NEW: latest fused heading (yaw), 0-360 degrees.
void getAHRSHeading(float *heading);