#include <Arduino.h>
#include <WiFi.h>
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
static const uint8_t  UNIT_ID = 1;
static const uint16_t MODBUS_PORT = 1502;

ModbusServerEthernet mb;
static std::unordered_map<uint16_t, uint16_t> HR;

// -------------------------
// Helpers
// -------------------------
static inline void setU16(uint16_t addr, uint16_t v) { HR[addr] = v; }

static inline void setU32(uint16_t addr, uint32_t v) {
  HR[addr]   = (uint16_t)((v >> 16) & 0xFFFF); // MSW
  HR[addr+1] = (uint16_t)(v & 0xFFFF);         // LSW
}

static inline void setS32(uint16_t addr, int32_t v) {
  setU32(addr, (uint32_t)v);
}

static inline int32_t getS32(uint16_t addr) {
  uint32_t hi = HR.count(addr)   ? HR[addr]   : 0;
  uint32_t lo = HR.count(addr+1) ? HR[addr+1] : 0;
  uint32_t u = (hi << 16) | lo;
  return (int32_t)u;
}

// Modbus 문자열(워드당 2문자, big-endian: [c1][c2])
static void setStringNorm(uint16_t startAddr, const char* s, uint16_t words) {
  size_t len = strlen(s);
  for (uint16_t i = 0; i < words; i++) {
    size_t idx = i * 2;
    uint8_t c1 = (idx < len) ? (uint8_t)s[idx] : (uint8_t)' ';
    uint8_t c2 = (idx + 1 < len) ? (uint8_t)s[idx + 1] : (uint8_t)' ';
    setU16(startAddr + i, (uint16_t(c1) << 8) | c2);
  }
}

// -------------------------
// SOCOMEC Identify Map
// -------------------------
static void initRegisterMap_Socomec() {
  // Gate 1: 50000 (0xC350) 4 words
  // 가장 안전한 형태: 0x534F 0x434F 0x0000 0x0000  => "SOCO\0\0\0\0"
  setU16(50000, 0x534F); // 'S''O'
  setU16(50001, 0x434F); // 'C''O'
  setU16(50002, 0x0000);
  setU16(50003, 0x0000);

  // Gate 2: 50058 (0xC38A) 8 words
  // OpenEMS가 여기서 "countis e14"를 보는 케이스가 많았음(너 socat 캡처 기준)
  setStringNorm(50058, "countis e14", 8);

  // 참고용(있어도 손해 없음)
  setStringNorm(50042, "SOCOMEC", 8);
  setStringNorm(50050, "COUNTIS E14", 8);

  // Measurements (예시)
  setU32(50520, 23000);  // 230.00 V
  setU32(50526, 50000);  // 50.000 Hz
  setU32(50528, 1000);   // 1.000 A

  // 주의: 어떤 드라이버는 Active Power가 W/0.1 이거나 float32 등이라 스케일/형식을 맞춰야 함
  // 우선은 “정수 W”로 흔들리게만 두고, OpenEMS 코드(defineModbusProtocol) 보고 스케일 맞추자.
  setS32(50536, 500);    // 500 W
  setS32(50542, 1000);   // PF 1.000
}

// -------------------------
// JANITZA Placeholder
// -------------------------
static void initRegisterMap_Janitza() {
  // TODO: Janitza 장비의 identify/register map에 맞춰 채워야 함
  // 지금은 그냥 기본값만
  setS32(50536, 123);
}

// -------------------------
// 1s tick update
// -------------------------
static void updateValuesTick() {
  static uint32_t t0 = millis();
  double sec = (millis() - t0) / 1000.0;

  double p_w = 500.0 + 300.0 * sin(sec * 0.4); // 200~800W
  setS32(50536, (int32_t)p_w);
}

// -------------------------
// FC03: Read Holding Registers
// -------------------------
ModbusMessage FC03(ModbusMessage request) {
  uint16_t startAddr = 0;
  uint16_t count = 0;

  // 요청 파싱 실패 대비해서 0으로 초기화 + 로그
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

  pinMode(ETH_POWER_PIN, OUTPUT);
  digitalWrite(ETH_POWER_PIN, HIGH);
  delay(100);

  bool ethOk = ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
  ETH.config(ip, gw, mask, dns);
  delay(200);

  Serial.printf("ETH.begin=%d  linkUp=%d  ip=%s\n",
                ethOk ? 1 : 0,
                ETH.linkUp() ? 1 : 0,
                ETH.localIP().toString().c_str());

  // Modbus worker / start
  mb.registerWorker(UNIT_ID, 0x03, FC03);
  bool started = mb.start(MODBUS_PORT, 4, 2000);
  Serial.printf("Modbus start port=%u => %d (UNIT_ID=%u)\n", MODBUS_PORT, started ? 1 : 0, UNIT_ID);

  // Mode select
#if defined(MODE_SOCOMEC)
  initRegisterMap_Socomec();
  Serial.println("[MODE] SOCOMEC");
#elif defined(MODE_JANITZA)
  initRegisterMap_Janitza();
  Serial.println("[MODE] JANITZA");
#else
  Serial.println("[MODE] BASIC (no identify map)");
#endif
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    last = millis();
    updateValuesTick();

    Serial.printf("Status: link=%d ip=%s clients=%u msg=%lu err=%lu P=%ldW\n",
                  ETH.linkUp() ? 1 : 0,
                  ETH.localIP().toString().c_str(),
                  mb.activeClients(),
                  (unsigned long)mb.getMessageCount(),
                  (unsigned long)mb.getErrorCount(),
                  (long)getS32(50536));
  }
}