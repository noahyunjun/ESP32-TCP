#include <Arduino.h>
#include <ETH.h>

#include "modbus_common.h"
#include "driver_registry.h"

void setup() {
  Serial.begin(115200);
  delay(300);

  eth_init_static_ip();
  modbus_init_fc03();

  driver_init();
  Serial.printf("[MAP] driver=%s\n", driver_name());
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    last = millis();
    driver_tick();

    Serial.printf("Status: link=%d clients=%u msg=%lu err=%lu driver=%s\n",
                  ETH.linkUp() ? 1 : 0,
                  modbus_active_clients(),
                  (unsigned long)modbus_message_count(),
                  (unsigned long)modbus_error_count(),
                  driver_name());
  }
}
