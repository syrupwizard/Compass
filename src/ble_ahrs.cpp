// BLE UART stream "heading,qw,qx,qy,qz" + Device Information + Battery,
// using the calibrated AHRS module (ahrs.cpp / ahrs.h).
#include <bluefruit.h>
#include "ahrs.h"

#define SEND_EVERY_N_UPDATES  4           // 100 Hz filter / 4 = 25 Hz over BLE
#define BATTERY_INTERVAL_MS   30000

#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0 V / 4096
#define VBAT_DIVIDER_COMP (2.0F)          // 2:1 divider on the Feather Sense

BLEUart bleuart;
BLEDis  bledis;
BLEBas  blebas;

uint32_t last_batt_ms = 0;
uint8_t  send_count   = 0;

uint8_t readBatteryPercent()
{
  float mv = analogRead(PIN_VBAT) * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;
  if (mv < 3300) return 0;
  if (mv < 3600) return (uint8_t)((mv - 3300) / 30);
  float pct = 10 + (mv - 3600) * 0.15F;
  return pct > 100 ? 100 : (uint8_t)pct;
}

void startAdv()
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void setup()
{
  Serial.begin(115200);

  setupAHRS();   // loads saved calibration, inits sensors + filter

  // Battery ADC
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);
  delay(1);

  // BLE
  Bluefruit.configPrphConn(92, BLE_GAP_EVENT_LENGTH_MIN, 16, 16);  // before begin()
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Sense AHRS");

  char serial[17];
  snprintf(serial, sizeof(serial), "%08lX%08lX",
           (unsigned long)NRF_FICR->DEVICEID[1], (unsigned long)NRF_FICR->DEVICEID[0]);
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Feather nRF52840 Sense");
  bledis.setSerialNum(serial);
  bledis.setFirmwareRev("1.0.0");
  bledis.begin();

  blebas.begin();
  blebas.write(readBatteryPercent());

  bleuart.begin();
  startAdv();
}

void loop()
{
  if (updateAHRS()) {                       // true once per filter step (100 Hz)
    if (++send_count >= SEND_EVERY_N_UPDATES) {
      send_count = 0;

      float heading = getAHRSTrueHeading();   // degrees, 0-360, true north
      float qw, qx, qy, qz;
      getAHRSQuaternion(&qw, &qx, &qy, &qz);

      char line[48];
      int n = snprintf(line, sizeof(line), "%.1f,%.3f,%.3f,%.3f,%.3f\n",
                       heading, qw, qx, qy, qz);
      if (Bluefruit.connected() && bleuart.notifyEnabled()) {
        bleuart.write((uint8_t*)line, n);
      }
    }
  }

  if (millis() - last_batt_ms >= BATTERY_INTERVAL_MS) {
    last_batt_ms = millis();
    uint8_t pct = readBatteryPercent();
    blebas.write(pct);
    if (Bluefruit.connected()) blebas.notify(pct);
  }
}
