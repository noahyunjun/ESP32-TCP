#include <Arduino.h>
#include <ETH.h>
#include <math.h>
#include <unordered_map>

#include "ModbusServerETH.h"
#include "ModbusMessage.h"

// Compile-time word swap for float32 (some devices use LSW->MSW)
#ifndef JANITZA_WORD_SWAP
#define JANITZA_WORD_SWAP 0
#endif

// 1) Hardware (Olimex ESP32-GATEWAY, LAN8720)
#define ETH_ADDR        0
#define ETH_POWER_PIN   5
#define ETH_MDC_PIN     23
#define ETH_MDIO_PIN    18
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT
// 링크 문제 있으면 아래도 시험
// #define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN

// 2) Network (direct static IP)
// Mac: 192.168.50.1 / ESP32: 192.168.50.2
IPAddress ip  (192, 168, 50, 2);
IPAddress gw  (192, 168, 50, 1);
IPAddress mask(255, 255, 255, 0);
IPAddress dns (192, 168, 50, 1);

// 3) Modbus/TCP
static const uint8_t  UNIT_ID     = 1;     // OpenEMS 설정의 Unit-ID 와 반드시 일치
static const uint16_t MODBUS_PORT = 1502;

ModbusServerEthernet mb;
static std::unordered_map<uint16_t, uint16_t> HR;

// Helpers (16bit register map)
static inline void setU16(uint16_t addr, uint16_t v) { HR[addr] = v; }

// 32-bit (2 registers) MSW->LSW
static inline void setU32(uint16_t addr, uint32_t v) {
  HR[addr]   = (uint16_t)((v >> 16) & 0xFFFF); // MSW
  HR[addr+1] = (uint16_t)(v & 0xFFFF);         // LSW
}
static inline void setS32(uint16_t addr, int32_t v) { setU32(addr, (uint32_t)v); }

static inline int32_t getS32(uint16_t addr) {
  uint32_t hi = HR.count(addr)   ? HR[addr]   : 0;
  uint32_t lo = HR.count(addr+1) ? HR[addr+1] : 0;
  return (int32_t)((hi << 16) | lo);
}

static inline void setFloat32(uint16_t addr, float v) {
  // IEEE754 float32 -> 2x16-bit holding registers (word order selectable)
  union {
    float f;
    uint32_t u32;
  } conv;
  conv.f = v;
  uint16_t msw = (uint16_t)((conv.u32 >> 16) & 0xFFFF);
  uint16_t lsw = (uint16_t)(conv.u32 & 0xFFFF);
#if JANITZA_WORD_SWAP
  // Some devices swap word order (LSW->MSW), make it a compile-time toggle
  HR[addr]   = lsw;
  HR[addr+1] = msw;
#else
  HR[addr]   = msw;
  HR[addr+1] = lsw;
#endif
}

static inline float getFloat32(uint16_t addr) {
  // Helper for debug prints (mirrors word-swap config)
  uint16_t w0 = HR.count(addr)   ? HR[addr]   : 0;
  uint16_t w1 = HR.count(addr+1) ? HR[addr+1] : 0;
#if JANITZA_WORD_SWAP
  uint16_t msw = w1;
  uint16_t lsw = w0;
#else
  uint16_t msw = w0;
  uint16_t lsw = w1;
#endif
  union {
    uint32_t u32;
    float f;
  } conv;
  conv.u32 = ((uint32_t)msw << 16) | (uint32_t)lsw;
  return conv.f;
}

// StringWordElement(., words): 1 register = 1 char (0x00XX)
static void setString_1charPerWord(uint16_t startAddr, const char* s, uint16_t words) {
  size_t len = strlen(s);
  for (uint16_t i = 0; i < words; i++) {
    uint8_t c = (i < len) ? (uint8_t)s[i] : (uint8_t)'\0';
    setU16(startAddr + i, (uint16_t)c); // 0x00XX
  }
}

// “일반 Modbus 문자열” (1 register = 2 chars, big-endian)
static void setString_2charPerWord(uint16_t startAddr, const char* s, uint16_t words) {
  size_t len = strlen(s);
  for (uint16_t i = 0; i < words; i++) {
    size_t idx = i * 2;
    uint8_t c1 = (idx < len) ? (uint8_t)s[idx] : (uint8_t)' ';
    uint8_t c2 = (idx + 1 < len) ? (uint8_t)s[idx + 1] : (uint8_t)' ';
    setU16(startAddr + i, (uint16_t(c1) << 8) | c2);
  }
}

#if defined(MODE_SOCOMEC)
// 4) Socomec Singlephase Countis E14 “정답 레지스터맵”
static void initRegisterMapSocomec() {
  setU16(0xC350, 0x0053); // 'S'
  setU16(0xC351, 0x004F); // 'O'
  setU16(0xC352, 0x0043); // 'C'
  setU16(0xC353, 0x004F); // 'O'
  setString_2charPerWord(0xC38A, "countis e14", 8);

  // ---- Measurements (폴링되는 실제 주소들)
  setU32(0xC558, 2300); // Voltage
  setU32(0xC55E, 500); // Frequency
  setU32(0xC560, 12.3); // Current
  setS32(0xC568, 500); // Active Power
  setS32(0xC56A, 0); // Reactive Power
  setU32(0xC702, 1234.5); // Energy
  setU32(0xC708, 0); // Energy
}

// 5) Live update (optional)
static void updateValuesTickSocomec() {
  static uint32_t t0 = millis();
  double sec = (millis() - t0) / 1000.0;

  // 200~800W 흔들기 (가정: /10이면 raw는 *10)
  double p_w = 500.0 + 300.0 * sin(sec * 0.4);
  setS32(0xC568, (int32_t)(p_w));
}

#elif defined(MODE_JANITZA)
// Janitza UMG104 polling addresses (OpenEMS Edge driver)
// Each value is IEEE754 float32 split across 2 holding registers.

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

#else
#error "Define MODE_SOCOMEC or MODE_JANITZA in build flags."
#endif

// FC03: Read Holding Registers (shared)
ModbusMessage FC03(ModbusMessage request) {
  uint16_t startAddr = 0;
  uint16_t count = 0;

  request.get(2, startAddr);
  request.get(4, count);

  Serial.printf("[FC03] start=%u count=%u\n", startAddr, count);

  ModbusMessage response;
  response.add(request.getServerID());
  response.add((uint8_t)0x03);
  response.add((uint8_t)(count * 2));

  for (uint16_t i = 0; i < count; i++) {
    uint16_t addr = startAddr + i;
    response.add(HR.count(addr) ? HR[addr] : (uint16_t)0);
  }
  return response;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Ethernet power on
  pinMode(ETH_POWER_PIN, OUTPUT);
  digitalWrite(ETH_POWER_PIN, HIGH);
  delay(100);

  bool ethOk = ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
  ETH.config(ip, gw, mask, dns);
  delay(200);

  Serial.printf("ETH.begin=%d linkUp=%d ip=%s\n",
                ethOk ? 1 : 0,
                ETH.linkUp() ? 1 : 0,
                ETH.localIP().toString().c_str());

  // Modbus start
  mb.registerWorker(UNIT_ID, 0x03, FC03);
  bool started = mb.start(MODBUS_PORT, 4, 2000);
  Serial.printf("Modbus start port=%u => %d (UNIT_ID=%u)\n", MODBUS_PORT, started ? 1 : 0, UNIT_ID);

  // Register map init
#if defined(MODE_SOCOMEC)
  initRegisterMapSocomec();
  Serial.println("[MAP] Socomec Countis E14 loaded");
#elif defined(MODE_JANITZA)
  initRegisterMapJanitza();
  Serial.println("[MAP] Janitza UMG104 mock loaded");
#endif
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    last = millis();
#if defined(MODE_SOCOMEC)
    updateValuesTickSocomec();
#elif defined(MODE_JANITZA)
    updateValuesTickJanitza();
#endif

    Serial.printf("Status: link=%d clients=%u msg=%lu err=%lu",
                  ETH.linkUp() ? 1 : 0,
                  mb.activeClients(),
                  (unsigned long)mb.getMessageCount(),
                  (unsigned long)mb.getErrorCount());
#if defined(MODE_SOCOMEC)
    Serial.printf(" [SOCOMEC]=%ld\n", (long)getS32(0xC568));
#elif defined(MODE_JANITZA)
    Serial.printf(" [JANITZA] P_total=%.2f freq=%.2f\n",
                  getFloat32(1369),
                  getFloat32(1439));
#endif
  }
}
