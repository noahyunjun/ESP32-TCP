#include <Arduino.h>
#include <WiFi.h>
#include <ETH.h>
#include <math.h>
#include <unordered_map>

// eModbus 라이브러리
#include "ModbusServerETH.h"
#include "ModbusMessage.h"

// 1. 하드웨어 설정 (Olimex ESP32-GATEWAY)
#define ETH_ADDR        0
#define ETH_POWER_PIN   5
#define ETH_MDC_PIN     23
#define ETH_MDIO_PIN    18
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT

// 2. 네트워크 설정 (직결 고정 IP)
IPAddress ip  (192, 168, 50, 2);
IPAddress gw  (192, 168, 50, 1);
IPAddress mask(255, 255, 255, 0);
IPAddress dns (192, 168, 50, 1);

// 3. Modbus 설정
// [주의] OpenEMS Socomec App의 기본 Unit ID는 6입니다. 
// OpenEMS 설정과 일치시켜야 합니다.
static const uint8_t  UNIT_ID = 1; 
static const uint16_t MODBUS_PORT = 1502;

ModbusServerEthernet mb;
static std::unordered_map<uint16_t, uint16_t> HR;

// --- 헬퍼 함수 ---
static inline void setU16(uint16_t addr, uint16_t v) { HR[addr] = v; }
static inline void setU32(uint16_t addr, uint32_t v) {
  HR[addr]   = (uint16_t)((v >> 16) & 0xFFFF); // MSW
  HR[addr+1] = (uint16_t)(v & 0xFFFF);         // LSW
}
static inline void setS32(uint16_t addr, int32_t v) { setU32(addr, (uint32_t)v); }

// 문자열을 Modbus 표준(word당 2문자)으로 저장하는 함수
static void setStringNorm(uint16_t startAddr, const char* s, uint16_t words) {
  size_t len = strlen(s);
  for (uint16_t i = 0; i < words; i++) {
    size_t idx = i * 2;
    uint8_t c1 = (idx < len) ? (uint8_t)s[idx] : (uint8_t)' ';
    uint8_t c2 = (idx + 1 < len) ? (uint8_t)s[idx + 1] : (uint8_t)' ';
    setU16(startAddr + i, (uint16_t(c1) << 8) | c2);
  }
}
// 50536번과 50537번 두 칸을 읽어서 하나의 32비트 숫자로 합쳐주는 함수
static inline int32_t getS32(uint16_t addr) {
  // 만약 해당 주소에 데이터가 없으면 0을 반환
  uint32_t hi = HR.count(addr)   ? HR[addr]   : 0; // 상위 16비트
  uint32_t lo = HR.count(addr+1) ? HR[addr+1] : 0; // 하위 16비트
  
  // 비트 연산으로 합치기 (상위를 16칸 밀고 하위를 붙임)
  uint32_t u = (hi << 16) | lo;
  return (int32_t)u; // 부호 있는 32비트 정수로 변환
}

// --- 핵심: OpenEMS 전용 식별자 튜닝 ---
static void initRegisterMap_Socomec_Tuned() {
  /* [관문 1] 제조사 식별 (Address 50000 / 0xC350)
     OpenEMS 코드의 UnsignedQuadruplewordElement(0xC350) 대응
     기대값: 0x0053 004F 0043 004F (S O C O)
  */
  setU16(50000, 0x0053); // 'S'
  setU16(50001, 0x004F); // 'O'
  setU16(50002, 0x0043); // 'C'
  setU16(50003, 0x004F); // 'O'

  /* [관문 2] 모델 식별 (Address 50058 / 0xC38A)
     OpenEMS 코드의 StringWordElement(0xC38A, 8) 대응
     기대값: "countis e14" (소문자로 비교하므로 안전하게 세팅)
  */
  setStringNorm(50058, "countis e14", 8);

  // 50050에도 혹시 모르니 동일하게 기록
  setStringNorm(50050, "COUNTIS E14", 8);
  setStringNorm(50042, "SOCOMEC", 8);

  // --- 측정값 초기값 세팅 ---
  setU32(50520, 23000);  // 230.00 V
  setU32(50526, 50000);  // 50.000 Hz
  setU32(50528, 1000);   // 1.000 A
  setS32(50536, 5000);   // 500.0 W (Active Power)
  setS32(50542, 1000);   // PF 1.000
}

// 1초마다 데이터 변화 시뮬레이션
static void updateValuesTick() {
  static uint32_t t0 = millis();
  double sec = (millis() - t0) / 1000.0;
  
  // 전력값이 200W~800W 사이를 오르내림
  double p_w = 500.0 + 300.0 * sin(sec * 0.4); 
  setS32(50536, (int32_t)(p_w));
}

// FC03 응답 처리기
ModbusMessage FC03(ModbusMessage request) {
  uint16_t startAddr, count;
  request.get(2, startAddr);
  request.get(4, count);

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
  
  // 이더넷 전원 ON
  pinMode(ETH_POWER_PIN, OUTPUT);
  digitalWrite(ETH_POWER_PIN, HIGH);
  delay(100);

  // 이더넷 시작
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
  ETH.config(ip, gw, mask, dns);
  
  // 레지스터 초기화
  initRegisterMap_Socomec_Tuned();

  // Modbus 설정
  mb.registerWorker(UNIT_ID, 0x03, FC03);
  mb.start(MODBUS_PORT, 4, 2000);

  Serial.println("--- ESP32 Tuned for OpenEMS Identification ---");
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    last = millis();
    updateValuesTick();
    
    // HR[50536]만 읽지 말고, 50536과 50537을 합쳐서 읽는 getS32 사용
    int32_t powerRaw = getS32(50536); 
    float realPower = powerRaw;

    Serial.printf("Status: Link=%d, Clients=%u, P=%.1fW\n", 
                  ETH.linkUp(), mb.activeClients(), realPower);
  }
}