#include <Arduino.h>
#include <math.h>

#include "modbus_common.h"
#include "drivers/meter_driver.h"

static void initRegisterMapSocomec() {
  // Identify (1 char per word)
  setU16(0xC350, 0x0053); // 'S'
  setU16(0xC351, 0x004F); // 'O'
  setU16(0xC352, 0x0043); // 'C'
  setU16(0xC353, 0x004F); // 'O'

  // Identify (2 chars per word, big-endian)
  setString_2charPerWord(0xC38A, "countis e14", 8);

  // Measurements (raw integers, no scaling)
  setU32(0xC558, 230); // Voltage
  setU32(0xC55E, 50);  // Frequency
  setU32(0xC560, 12);  // Current
  setS32(0xC568, 500); // Active Power
  setS32(0xC56A, 0);   // Reactive Power
  setU32(0xC702, 500); // Energy
  setU32(0xC708, 0);   // Energy  
}

static void updateValuesTickSocomec() {
  static uint32_t t0 = millis();
  double sec = (millis() - t0) / 1000.0;

  // Keep raw integers, only small variation
  double p_w = 500.0 + 100.0 * sin(sec * 0.4);
  setS32(0xC568, (int32_t)(p_w));
}

static const meter_driver_t DRIVER = {
  "socomec_e14",
  initRegisterMapSocomec,
  updateValuesTickSocomec,
};

const meter_driver_t* get_socomec_driver() {
  return &DRIVER;
}
