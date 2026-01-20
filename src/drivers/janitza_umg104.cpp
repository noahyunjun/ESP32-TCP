#include <Arduino.h>
#include <math.h>

#include "modbus_common.h"
#include "drivers/meter_driver.h"

static void initRegisterMapJanitza() {
  // Voltage L1/L2/L3
  setFloat32(1317, 230.5f);
  setFloat32(1319, 231.2f);
  setFloat32(1321, 229.8f);

  // Current L1/L2/L3
  setFloat32(1325, 5.1f);
  setFloat32(1327, 4.8f);
  setFloat32(1329, 5.4f);

  // Active power L1/L2/L3
  setFloat32(1333, 800.0f);
  setFloat32(1335, 750.0f);
  setFloat32(1337, 820.0f);

  // Reactive power L1/L2/L3
  setFloat32(1341, 120.0f);
  setFloat32(1343, 110.0f);
  setFloat32(1345, 130.0f);

  // Total active/reactive power
  setFloat32(1369, 2370.0f);
  setFloat32(1371, 360.0f);

  // Frequency, rotating field, internal temperature
  setFloat32(1439, 50.0f);
  setFloat32(1449, 0.0f);
  setFloat32(1461, 35.0f);

  // Energy (kWh, kVarh)
  setFloat32(9851, 12345.6f);
  setFloat32(9863, 2345.6f);
}

static void updateValuesTickJanitza() {
  static uint32_t t0 = millis();
  double sec = (millis() - t0) / 1000.0;

  // Change a few key values so OpenEMS can see live updates
  float p_total = 2000.0f + 500.0f * sin(sec * 0.6);
  float freq = 50.0f + 0.02f * sin(sec * 1.3);

  setFloat32(1369, p_total);
  setFloat32(1439, freq);
}

static const meter_driver_t DRIVER = {
  "janitza_umg104",
  initRegisterMapJanitza,
  updateValuesTickJanitza,
};

const meter_driver_t* get_janitza_driver() {
  return &DRIVER;
}
