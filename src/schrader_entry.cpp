// schrader_entry.cpp
#include "schrader_entry.h"

// Decode the 3-byte ID into a 32-bit integer
uint32_t decode_id(const uint8_t id[3]) {
    return (id[0] << 16) | (id[1] << 8) | id[2];
}

// Decode the raw pressure byte into pressure in kPa
uint16_t decode_pressure(uint8_t pressure_raw) {
    return pressure_raw * 25; // 1 raw unit = 25 mbar
}

// Decode the raw temperature byte into temperature in Celsius
int8_t decode_temperature(uint8_t temperature_raw) {
    return temperature_raw - 50; // Offset by 50
}
