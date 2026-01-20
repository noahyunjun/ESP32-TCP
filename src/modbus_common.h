#ifndef MODBUS_COMMON_H
#define MODBUS_COMMON_H

#include <Arduino.h>
#include <ETH.h>
#include <unordered_map>

#include "ModbusMessage.h"
#include "ModbusServerETH.h"

// Compile-time word swap for float32 (some devices use LSW->MSW)
#ifndef JANITZA_WORD_SWAP
#define JANITZA_WORD_SWAP 0
#endif

static const uint8_t UNIT_ID = 1;
static const uint16_t MODBUS_PORT = 1502;

void eth_init_static_ip();
void modbus_init_fc03();

void setU16(uint16_t addr, uint16_t v);
void setU32(uint16_t addr, uint32_t v);
void setS32(uint16_t addr, int32_t v);
int32_t getS32(uint16_t addr);

void setFloat32(uint16_t addr, float v);
float getFloat32(uint16_t addr);

void setString_1charPerWord(uint16_t startAddr, const char* s, uint16_t words);
void setString_2charPerWord(uint16_t startAddr, const char* s, uint16_t words);

uint32_t modbus_message_count();
uint32_t modbus_error_count();
uint16_t modbus_active_clients();

#endif
