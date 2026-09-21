// Manual sensor calibration for the Feather Sense (LSM6DS33 + LIS3MDL).
//
//   a  accelerometer: 6-position zero-g offsets
//   g  gyroscope:     zero-rate offsets (board held still)
//   m  magnetometer:  hard-iron offsets + per-axis soft-iron scale (rotate the board)
//   p  print current values      s  save to flash      l  load from flash
//   z  reset to defaults (RAM only until you save)      ?  help
//
// Results are stored with Adafruit_Sensor_Calibration, the same helper your AHRS
// sketch loads and applies with cal.calibrate(), so nothing changes on that side.
// Send one character per command from the Serial Monitor (Ctrl-Enter / Send).

#include <Arduino.h>
#include <Adafruit_Sensor_Calibration.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "LSM6DS_LIS3MDL.h"   // same sensor setup as your AHRS sketch (copy it into this folder)

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
Adafruit_Sensor_Calibration_EEPROM cal;
#else
Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define GRAVITY_MS2      9.80665f
#define ACCEL_SAMPLES    300       // ~3 s per position
#define GYRO_SAMPLES     500       // ~5 s
#define SAMPLE_DELAY_MS  10
#define ACCEL_MAX_STD    0.15f     // m/s^2: reject a position if the board moved
#define GYRO_MAX_STD     0.02f     // rad/s: reject if the board moved

// ---------------------------------------------------------------- helpers ---

static void flushInput() {
  delay(20);
  while (Serial.available()) Serial.read();
}

static void waitForKey() {
  flushInput();
  while (!Serial.available()) yield();
  flushInput();
}

static void printVec(const char *label, const float v[3], int digits = 4) {
  Serial.print(label);
  for (int i = 0; i < 3; i++) {
    Serial.print(v[i], digits);
    Serial.print(i < 2 ? ", " : "\n");
  }
}

// Average n samples from a sensor. Returns false if any axis' standard
// deviation exceeds maxStd, which means the board was moving.
static bool captureStill(Adafruit_Sensor *s, int n, float maxStd, float mean[3]) {
  float m2[3] = {0, 0, 0};
  mean[0] = mean[1] = mean[2] = 0;

  for (int k = 1; k <= n; k++) {
    sensors_event_t e;
    s->getEvent(&e);
    for (int i = 0; i < 3; i++) {
      float d = e.data[i] - mean[i];
      mean[i] += d / k;
      m2[i] += d * (e.data[i] - mean[i]);
    }
    if (k % 50 == 0) Serial.print('.');
    delay(SAMPLE_DELAY_MS);
  }
  Serial.println();

  float sd[3];
  bool ok = true;
  for (int i = 0; i < 3; i++) {
    sd[i] = sqrtf(m2[i] / (n - 1));
    if (sd[i] > maxStd) ok = false;
  }
  if (!ok) printVec("  std dev too high: ", sd);
  return ok;
}

// ------------------------------------------------------------------- gyro ---

static void calibrateGyro() {
  Serial.println(F("\n== Gyro =="));
  Serial.println(F("Put the board on a still surface and don't touch it."));
  Serial.println(F("Send any character to start (takes about 5 s)."));
  waitForKey();

  float mean[3];
  if (!captureStill(gyroscope, GYRO_SAMPLES, GYRO_MAX_STD, mean)) {
    Serial.println(F("Board moved. Gyro calibration NOT changed. Try again."));
    return;
  }
  for (int i = 0; i < 3; i++) cal.gyro_zerorate[i] = mean[i];
  printVec("Gyro zero-rate (rad/s): ", cal.gyro_zerorate);
}

// ------------------------------------------------------------------ accel ---

static void calibrateAccel() {
  static const struct { const char *name; int axis; int sign; } pos[6] = {
    {"+Z up   (flat, component side up)",        2, +1},
    {"-Z up   (flat, upside down)",              2, -1},
    {"+X up   (stand the board on an edge)",     0, +1},
    {"-X up   (stand it on the opposite edge)",  0, -1},
    {"+Y up   (stand the board on an edge)",     1, +1},
    {"-Y up   (stand it on the opposite edge)",  1, -1},
  };
  float reading[3][2];   // [axis][0 = +axis up, 1 = -axis up], m/s^2

  Serial.println(F("\n== Accelerometer (6 positions) =="));
  Serial.println(F("For each position, tilt the board until the named axis reads about +/-9.8,"));
  Serial.println(F("hold it steady on a table, then send any character. Don't touch it while"));
  Serial.println(F("it samples (about 3 s)."));

  for (int p = 0; p < 6; p++) {
    for (;;) {
      Serial.print(F("\n["));
      Serial.print(p + 1);
      Serial.print(F("/6] Position: "));
      Serial.println(pos[p].name);

      flushInput();
      while (!Serial.available()) {
        sensors_event_t e;
        accelerometer->getEvent(&e);
        Serial.print(F("  x="));
        Serial.print(e.data[0], 2);
        Serial.print(F("  y="));
        Serial.print(e.data[1], 2);
        Serial.print(F("  z="));
        Serial.println(e.data[2], 2);
        delay(250);
      }
      flushInput();

      Serial.print(F("  sampling"));
      float mean[3];
      if (!captureStill(accelerometer, ACCEL_SAMPLES, ACCEL_MAX_STD, mean)) {
        Serial.println(F("  Board moved. Try this position again."));
        continue;
      }
      if (mean[pos[p].axis] * pos[p].sign < 0.8f * GRAVITY_MS2) {
        Serial.println(F("  That isn't the requested orientation. Try again."));
        continue;
      }
      reading[pos[p].axis][pos[p].sign > 0 ? 0 : 1] = mean[pos[p].axis];
      break;
    }
  }

  // Offset = midpoint of the +1 g and -1 g readings on each axis.
  // Scale is only reported: the library stores offsets, not scale factors.
  static const char axisName[3] = {'X', 'Y', 'Z'};
  Serial.println(F("\nResults:"));
  for (int a = 0; a < 3; a++) {
    float plus = reading[a][0], minus = reading[a][1];
    cal.accel_zerog[a] = 0.5f * (plus + minus);
    float scale = (plus - minus) / (2.0f * GRAVITY_MS2);
    Serial.print(F("  "));
    Serial.print(axisName[a]);
    Serial.print(F(": zero-g offset "));
    Serial.print(cal.accel_zerog[a], 4);
    Serial.print(F(" m/s^2, scale error "));
    Serial.print((scale - 1.0f) * 100.0f, 2);
    Serial.println(F(" % (not corrected)"));
  }
}

// -------------------------------------------------------------------- mag ---

static void calibrateMag() {
  Serial.println(F("\n== Magnetometer =="));
  Serial.println(F("Keep the board away from metal, magnets, laptops and phones."));
  Serial.println(F("After starting, rotate it slowly through EVERY orientation: figure-eights,"));
  Serial.println(F("flips, all faces up and down, all headings. Send any character to finish"));
  Serial.println(F("once the min/max values stop changing. Send any character to start."));
  waitForKey();

  float mn[3] = {1e9f, 1e9f, 1e9f};
  float mx[3] = {-1e9f, -1e9f, -1e9f};
  uint32_t lastPrint = 0;

  while (!Serial.available()) {
    sensors_event_t e;
    magnetometer->getEvent(&e);
    for (int i = 0; i < 3; i++) {
      if (e.data[i] < mn[i]) mn[i] = e.data[i];
      if (e.data[i] > mx[i]) mx[i] = e.data[i];
    }
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      printVec("  min (uT): ", mn, 1);
      printVec("  max (uT): ", mx, 1);
    }
    delay(10);
  }
  flushInput();

  float radius[3];
  for (int i = 0; i < 3; i++) {
    radius[i] = 0.5f * (mx[i] - mn[i]);
    if (radius[i] < 10.0f) {
      Serial.print(F("Axis "));
      Serial.print(i);
      Serial.println(F(" barely changed: rotate through more orientations. Mag calibration NOT changed."));
      return;
    }
  }
  float avg = (radius[0] + radius[1] + radius[2]) / 3.0f;

  for (int i = 0; i < 3; i++) cal.mag_hardiron[i] = 0.5f * (mx[i] + mn[i]);

  // Per-axis (diagonal) soft-iron scale so all three axes reach the same radius
  for (int i = 0; i < 9; i++) cal.mag_softiron[i] = 0;
  for (int i = 0; i < 3; i++) cal.mag_softiron[i * 4] = avg / radius[i];

  cal.mag_field = avg;

  Serial.println(F("\nResults:"));
  printVec("  hard-iron (uT): ", cal.mag_hardiron);
  float soft[3] = {cal.mag_softiron[0], cal.mag_softiron[4], cal.mag_softiron[8]};
  printVec("  soft-iron scale x, y, z: ", soft);
  Serial.print(F("  field strength (uT): "));
  Serial.println(cal.mag_field, 2);
  if (avg < 25.0f || avg > 65.0f) {
    Serial.println(F("  WARNING: Earth's field is roughly 25-65 uT. This value is outside that,"));
    Serial.println(F("  so coverage was probably poor or something magnetic was nearby. Redo it."));
  }
}

// ------------------------------------------------------------------- misc ---

static void printCal() {
  Serial.println(F("\nCurrent values (RAM, not saved until you send 's'):"));
  printVec("  accel zero-g (m/s^2): ", cal.accel_zerog);
  printVec("  gyro zero-rate (rad/s): ", cal.gyro_zerorate);
  printVec("  mag hard-iron (uT): ", cal.mag_hardiron);
  Serial.print(F("  mag soft-iron: "));
  for (int i = 0; i < 9; i++) {
    Serial.print(cal.mag_softiron[i], 4);
    Serial.print(i < 8 ? ", " : "\n");
  }
  Serial.print(F("  mag field (uT): "));
  Serial.println(cal.mag_field, 2);
}

static void resetCal() {
  for (int i = 0; i < 3; i++) {
    cal.accel_zerog[i] = 0;
    cal.gyro_zerorate[i] = 0;
    cal.mag_hardiron[i] = 0;
  }
  for (int i = 0; i < 9; i++) cal.mag_softiron[i] = (i % 4 == 0) ? 1.0f : 0.0f;
  cal.mag_field = 50;
  Serial.println(F("Reset to defaults in RAM. Send 's' to overwrite the saved calibration."));
}

static void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  a  calibrate accelerometer (6 positions)"));
  Serial.println(F("  g  calibrate gyro (keep still)"));
  Serial.println(F("  m  calibrate magnetometer (rotate the board)"));
  Serial.println(F("  p  print current values"));
  Serial.println(F("  s  save to flash"));
  Serial.println(F("  l  load from flash"));
  Serial.println(F("  z  reset to defaults (RAM only)"));
  Serial.println(F("  ?  this help"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);   // bench tool: wait for the Serial Monitor
  Serial.println(F("Sensor calibration"));

  if (!cal.begin()) {
    Serial.println(F("Failed to initialize calibration storage (is the flash FAT formatted?)"));
    while (1) yield();
  }
  if (cal.loadCalibration()) {
    Serial.println(F("Loaded existing calibration (recalibrating one sensor keeps the others)."));
  } else {
    Serial.println(F("No saved calibration found. Starting from defaults."));
  }

  if (!init_sensors()) {
    Serial.println(F("Failed to find sensors"));
    while (1) delay(10);
  }
  setup_sensors();

  printCal();
  printHelp();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();

  switch (c) {
    case 'a': calibrateAccel(); break;
    case 'g': calibrateGyro();  break;
    case 'm': calibrateMag();   break;
    case 'p': printCal();       break;
    case 'z': resetCal();       break;
    case 'l':
      if (cal.loadCalibration()) {
        Serial.println(F("Loaded from flash."));
        printCal();
      } else {
        Serial.println(F("Nothing loaded (no saved calibration found)."));
      }
      break;
    case 's':
      if (cal.saveCalibration()) {
        Serial.println(F("Saved. Contents of flash:"));
        cal.printSavedCalibration();
      } else {
        Serial.println(F("**WARNING** couldn't save calibration"));
      }
      break;
    case '?': printHelp(); break;
    default:  break;   // ignore newlines and stray characters
  }
}
