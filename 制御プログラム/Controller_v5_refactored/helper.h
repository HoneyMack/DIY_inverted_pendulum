#pragma once

#include <Wire.h>
#include <Arduino.h>

//I2c書き込み関数
void I2c_write(uint8_t device_address, int memory_address, byte value, bool is_continue=false);

//I2c読み取り関数
void I2c_read(uint8_t device_address, int memory_address, int read_length, byte *p, bool is_continue=false);
