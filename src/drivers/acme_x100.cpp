#include <Arduino.h>
#include <math.h>

#include "modbus_common.h"
#include "drivers/meter_driver.h"

static void initRegisterMapAcme() {
  // Identify (2 chars per word, big-endian)
  setString_2charPerWord(0x9000, "ACME X100", 5);
  setString_2charPerWord(0x9010, "SN123456", 4);

  // Measurements (raw integers, no scaling unless noted)
  setU32(0x9100, 2300);     // Voltage (0.1 V units)
  setU32(0x9102, 150);      // Current (0.01 A units)
  setS32(0x9104, 500);      // Active Power (W)
  setU32(0x9106, 12345);    // Energy (Wh)
  setFloat32(0x9108, 24.5); // Temperature (C)
}

static void updateValuesTickAcme() {
  static uint32_t t0 = millis();
  double sec = (millis() - t0) / 1000.0;

  // Small, smooth variation for demo data
  setS32(0x9104, (int32_t)(500.0 + 120.0 * sin(sec * 0.3)));
  setU32(0x9102, (uint32_t)(150.0 + 30.0 * sin(sec * 0.6)));
  setFloat32(0x9108, (float)(24.5 + 1.5 * sin(sec * 0.2)));
}

static const meter_driver_t DRIVER = {
  "acme_x100",
  initRegisterMapAcme,
  updateValuesTickAcme,
};

const meter_driver_t* get_acme_driver() {
  return &DRIVER;
}
