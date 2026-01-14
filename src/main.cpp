#include <Arduino.h>
#include <WiFi.h>
#include <ETH.h>
#include <math.h>

#include "ModbusServerETH.h"
#include "ModbusMessage.h"

// -------------------------
// OLIMEX ESP32-GATEWAY (LAN8720)
// -------------------------
#define ETH_ADDR        0
#define ETH_POWER_PIN   5
#define ETH_MDC_PIN     23
#define ETH_MDIO_PIN    18
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT
// 링크가 불안정하면 아래도 시험
// #define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN

// -------------------------
// 직결 고정 IP
// Mac(en6): 192.168.50.1 / ESP32: 192.168.50.2
// -------------------------
IPAddress ip  (192, 168, 50, 2);
IPAddress gw  (192, 168, 50, 1);
IPAddress mask(255, 255, 255, 0);
IPAddress dns (192, 168, 50, 1);

// -------------------------
// Modbus/TCP 서버
// -------------------------
static const uint8_t  UNIT_ID = 1;
static const uint16_t MODBUS_PORT = 1502;

ModbusServerEthernet mb;

// -------------------------
// Holding Register Map (sparse)
// -------------------------
#include <unordered_map>
static std::unordered_map<uint16_t, uint16_t> HR;

static inline void setU16(uint16_t addr, uint16_t v) { HR[addr] = v; }
static inline void setU32(uint16_t addr, uint32_t v) {
  HR[addr]   = (uint16_t)((v >> 16) & 0xFFFF); // MSW
  HR[addr+1] = (uint16_t)(v & 0xFFFF);         // LSW
}
static inline void setS32(uint16_t addr, int32_t v) { setU32(addr, (uint32_t)v); }

static inline int32_t getS32(uint16_t addr) {
  uint32_t hi = HR.count(addr)   ? HR[addr]   : 0;
  uint32_t lo = HR.count(addr+1) ? HR[addr+1] : 0;
  uint32_t u = (hi << 16) | lo;
  return (int32_t)u;
}

// STRING_NORM: word당 2바이트(2문자), big-endian ('S''O' => 0x534F)
static void setStringNorm(uint16_t startAddr, const char* s, uint16_t words) {
  for (uint16_t i = 0; i < words; i++) {
    char c1 = s[i * 2]     ? s[i * 2]     : ' ';
    char c2 = s[i * 2 + 1] ? s[i * 2 + 1] : ' ';
    uint16_t w = ((uint16_t)(uint8_t)c1 << 8) | (uint16_t)(uint8_t)c2;
    setU16(startAddr + i, w);
  }
}

static void initRegisterMap_CountisE14() {
  // 50000 (4 words): "SOCO"
  setStringNorm(50000, "SOCO", 4);

  // 50004: Product order ID (Countis=100)
  setU16(50004, 100);

  // 50005: Product ID (임의값; 실제 장비 값과 일치하면 더 좋음)
  setU16(50005, 1014);

  // 50006: JBUS Table Version (임의)
  setU16(50006, 101);

  // 50007: Product software version (임의)
  setU16(50007, 100);

  // 50042 (8 words): Vendor name "SOCOMEC"
  setStringNorm(50042, "SOCOMEC", 8);

  // 50050 (8 words): Product name "COUNTIS E14"
  setStringNorm(50050, "COUNTIS E14", 8);

  // 50058 (8 words): Extended name (여기도 "COUNTIS E14"로 맞춰서 시작)
  setStringNorm(50058, "COUNTIS E14", 8);

  // ---- Measurements (Countis E14 table 일부)
  // 50520 (U32, V/100): V1
  setU32(50520, (uint32_t)(230.00 * 100));

  // 50526 (U32, Hz/1000): F
  setU32(50526, (uint32_t)(50.000 * 1000));

  // 50528 (U32, A/1000): I1
  setU32(50528, (uint32_t)(1.234 * 1000));

  // 50536 (S32, W/0.1): Sum Active Power P
  setS32(50536, (int32_t)(500.0 * 10));

  // 50542 (S32, /1000): PF
  setS32(50542, (int32_t)(0.980 * 1000));
}

static void updateValuesTick() {
  static uint32_t t0 = millis();
  double sec = (millis() - t0) / 1000.0;

  double p_w = 500.0 + 300.0 * sin(sec * 0.4); // 200~800W
  double v = 230.0;
  double i = p_w / v;
  double pf = 0.98;

  setU32(50520, (uint32_t)(v * 100));
  setU32(50528, (uint32_t)(i * 1000));
  setS32(50536, (int32_t)(p_w * 10));
  setS32(50542, (int32_t)(pf * 1000));
}

// FC03: Read Holding Registers
ModbusMessage FC03(ModbusMessage request) {
  uint16_t startAddr = 0;
  uint16_t count = 0;
  request.get(2, startAddr);
  request.get(4, count);

  ModbusMessage response;
  response.add(request.getServerID());
  response.add((uint8_t)0x03);
  response.add((uint8_t)(count * 2));

  for (uint16_t i = 0; i < count; i++) {
    uint16_t addr = startAddr + i;
    uint16_t val = HR.count(addr) ? HR[addr] : 0; // 없는 주소는 0
    response.add(val);
  }
  return response;
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n--- ESP32 Modbus/TCP Slave (Socomec Countis E14 mock) ---");

  pinMode(ETH_POWER_PIN, OUTPUT);
  digitalWrite(ETH_POWER_PIN, HIGH);
  delay(100);

  bool ok = ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
  Serial.printf("ETH.begin() => %s\n", ok ? "true" : "false");

  ETH.config(ip, gw, mask, dns);
  delay(200);

  Serial.printf("IP: %s  linkUp=%d\n", ETH.localIP().toString().c_str(), ETH.linkUp() ? 1 : 0);

  initRegisterMap_CountisE14();

  mb.registerWorker(UNIT_ID, 0x03, FC03);

  bool started = mb.start(MODBUS_PORT, 4, 2000);
  Serial.printf("Modbus start %u => %s\n", MODBUS_PORT, started ? "true" : "false");
  Serial.println("Test from Mac:");
  Serial.println("  python3 -c \"from pymodbus.client import ModbusTcpClient as C; c=C('192.168.50.2',port=1502); c.connect(); r=c.read_holding_registers(50000,count=4,slave=1); print(r.registers); r=c.read_holding_registers(50042,count=8,slave=1); print(r.registers); c.close()\"");
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    last = millis();
    updateValuesTick();

    Serial.printf("linkUp=%d ip=%s clients=%u msg=%lu err=%lu P(50536)=%ld\n",
                  ETH.linkUp() ? 1 : 0,
                  ETH.localIP().toString().c_str(),
                  mb.activeClients(),
                  (unsigned long)mb.getMessageCount(),
                  (unsigned long)mb.getErrorCount(),
                  (long)getS32(50536));
  }
}