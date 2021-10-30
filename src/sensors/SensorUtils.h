#pragma once

#include <stdint.h>

void writeTo(int handle, uint8_t address, uint8_t val);
uint8_t readFrom(int handle, uint8_t address);
