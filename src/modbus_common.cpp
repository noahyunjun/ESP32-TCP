#include "modbus_common.h"

// Hardware (Olimex ESP32-GATEWAY, LAN8720)
#define ETH_ADDR        0
#define ETH_POWER_PIN   5
#define ETH_MDC_PIN     23
#define ETH_MDIO_PIN    18
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT
// If link issues: try ETH_CLOCK_GPIO0_IN instead

// Network (direct static IP)
// Mac: 192.168.50.1 / ESP32: 192.168.50.2
static IPAddress ip  (192, 168, 50, 2);
static IPAddress gw  (192, 168, 50, 1);
static IPAddress mask(255, 255, 255, 0);
static IPAddress dns (192, 168, 50, 1);

static ModbusServerEthernet mb;
static std::unordered_map<uint16_t, uint16_t> HR;

void eth_init_static_ip() {
  // Power up PHY for ESP32-GATEWAY and set static IP
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
}

static ModbusMessage FC03(ModbusMessage request) {
  uint16_t startAddr = 0;
  uint16_t count = 0;

  request.get(2, startAddr);
  request.get(4, count);

  // Log every FC03 request to see actual polling addresses
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

void modbus_init_fc03() {
  mb.registerWorker(UNIT_ID, 0x03, FC03);
  bool started = mb.start(MODBUS_PORT, 4, 2000);
  Serial.printf("Modbus start port=%u => %d (UNIT_ID=%u)\n", MODBUS_PORT, started ? 1 : 0, UNIT_ID);
}

void setU16(uint16_t addr, uint16_t v) { HR[addr] = v; }

void setU32(uint16_t addr, uint32_t v) {
  HR[addr]   = (uint16_t)((v >> 16) & 0xFFFF); // MSW
  HR[addr+1] = (uint16_t)(v & 0xFFFF);         // LSW
}

void setS32(uint16_t addr, int32_t v) { setU32(addr, (uint32_t)v); }

int32_t getS32(uint16_t addr) {
  uint32_t hi = HR.count(addr)   ? HR[addr]   : 0;
  uint32_t lo = HR.count(addr+1) ? HR[addr+1] : 0;
  return (int32_t)((hi << 16) | lo);
}

void setFloat32(uint16_t addr, float v) {
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

float getFloat32(uint16_t addr) {
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

void setString_1charPerWord(uint16_t startAddr, const char* s, uint16_t words) {
  size_t len = strlen(s);
  for (uint16_t i = 0; i < words; i++) {
    uint8_t c = (i < len) ? (uint8_t)s[i] : (uint8_t)'\0';
    setU16(startAddr + i, (uint16_t)c); // 0x00XX
  }
}

void setString_2charPerWord(uint16_t startAddr, const char* s, uint16_t words) {
  size_t len = strlen(s);
  for (uint16_t i = 0; i < words; i++) {
    size_t idx = i * 2;
    uint8_t c1 = (idx < len) ? (uint8_t)s[idx] : (uint8_t)' ';
    uint8_t c2 = (idx + 1 < len) ? (uint8_t)s[idx + 1] : (uint8_t)' ';
    setU16(startAddr + i, (uint16_t(c1) << 8) | c2);
  }
}

uint32_t modbus_message_count() { return mb.getMessageCount(); }
uint32_t modbus_error_count() { return mb.getErrorCount(); }
uint16_t modbus_active_clients() { return mb.activeClients(); }
