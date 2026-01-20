#include <Arduino.h>
#include <ETH.h>
#include <math.h>
#include <unordered_map>

#include "ModbusServerETH.h"
#include "ModbusMessage.h"

// -------------------------
// 1) Hardware (Olimex ESP32-GATEWAY, LAN8720)
// -------------------------
#define ETH_ADDR        0
#define ETH_POWER_PIN   5
#define ETH_MDC_PIN     23
#define ETH_MDIO_PIN    18
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT
// 링크 문제 있으면 아래도 시험
// #define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN

// -------------------------
// 2) Network (direct static IP)
// Mac: 192.168.50.1 / ESP32: 192.168.50.2
// -------------------------
IPAddress ip  (192, 168, 50, 2);
IPAddress gw  (192, 168, 50, 1);
IPAddress mask(255, 255, 255, 0);
IPAddress dns (192, 168, 50, 1);

// -------------------------
// 3) Modbus/TCP
// -------------------------
static const uint8_t  UNIT_ID     = 1;     // OpenEMS 설정의 Unit-ID 와 반드시 일치
static const uint16_t MODBUS_PORT = 1502;

ModbusServerEthernet mb;
static std::unordered_map<uint16_t, uint16_t> HR;

// -------------------------
// Helpers (16bit register map)
// -------------------------
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
static void initRegisterMap() {
  setU16(0xC350, 0x0053); // 'S'
  setU16(0xC351, 0x004F); // 'O'
  setU16(0xC352, 0x0043); // 'C'
  setU16(0xC353, 0x004F); // 'O'
  setString_1charPerWord(0xC38A, "countis ", 8);

  // 2) 2char/word 방식(16 chars): "countis e14"를 16바이트 공간에 넣기
  //    (OpenEMS 구현이 2char/word로 해석하는 경우 대비)
  setString_2charPerWord(0xC38A, "countis e14", 8);

  setString_2charPerWord(0xC542, "SOCOMEC", 8);
  setString_2charPerWord(0xC550, "COUNTIS E14", 8);

  // ---- Measurements (폴링되는 실제 주소들)
  setU32(0xC558, 2300);
  setU32(0xC55E, 500);
  setU32(0xC560, 12.3);
  setS32(0xC568, 500);
  setS32(0xC56A, 0);
  setU32(0xC702, 1234.5);
  setU32(0xC708, 0);
}

// -------------------------
// 5) Live update (optional)
// -------------------------
static void updateValuesTick() {
  static uint32_t t0 = millis();
  double sec = (millis() - t0) / 1000.0;

  // 200~800W 흔들기 (가정: /10이면 raw는 *10)
  double p_w = 500.0 + 300.0 * sin(sec * 0.4);
  setS32(0xC568, (int32_t)(p_w * 10.0));  // /10 가정
}

// -------------------------
// FC03: Read Holding Registers (Socomec driver uses FC3)
// -------------------------
ModbusMessage FC03(ModbusMessage request) {
  uint16_t startAddr = 0;
  uint16_t count = 0;

  request.get(2, startAddr);
  request.get(4, count);

  Serial.printf("[FC03] start=0x%04X (%u) count=%u\n", startAddr, startAddr, count);

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

#elif defined(MODE_JANITZA)

static void initRegisterMap() {
  Serial.println("[MAP] Janitza stub (not implemented)");
}

static void updateValuesTick() {
  Serial.println("[JANITZA] updateValuesTick stub");
}

ModbusMessage FC03(ModbusMessage request) {
  (void)request;
  Serial.println("[JANITZA] FC03 stub (not implemented)");
  return ModbusMessage();
}

#else
#error "Define MODE_SOCOMEC or MODE_JANITZA in build flags."
#endif

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
  initRegisterMap();
#if defined(MODE_SOCOMEC)
  Serial.println("[MAP] Socomec Countis E14 loaded");
#elif defined(MODE_JANITZA)
  Serial.println("[MAP] Janitza stub selected");
#endif
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    last = millis();
    updateValuesTick();

    Serial.printf("Status: link=%d clients=%u msg=%lu err=%lu",
                  ETH.linkUp() ? 1 : 0,
                  mb.activeClients(),
                  (unsigned long)mb.getMessageCount(),
                  (unsigned long)mb.getErrorCount());
#if defined(MODE_SOCOMEC)
    Serial.printf(" [SOCOMEC]=%ld\n", (long)getS32(0xC568));
#elif defined(MODE_JANITZA)
    Serial.printf(" [JANITZA stub]\n");
#endif
  }
}
