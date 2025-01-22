// schrader_entry.h
#ifndef SCHRADER_ENTRY_H
#define SCHRADER_ENTRY_H

#include <stdint.h>
#include <time.h>

// SchraderEntry structure matching the byte structure of the original entry
typedef struct {
    uint8_t flags[4];           // Flags (bytes 0-3)
    uint8_t id[3];              // Unique ID (bytes 4-6)
    uint8_t pressure_raw;       // Raw pressure byte (byte 7)
    uint8_t temperature_raw;    // Raw temperature byte (byte 8)
    uint8_t mic;                // MIC/Checksum (byte 9)
    time_t timestamp;           // Timestamp of when the entry was added/modified
    uint32_t retransmit_count;  // Number of times the entry has been retransmitted
} SchraderEntry;

// Helper functions to decode fields
uint32_t decode_id(const uint8_t id[3]);
uint16_t decode_pressure(uint8_t pressure_raw);
int8_t decode_temperature(uint8_t temperature_raw);

#endif // SCHRADER_ENTRY_H
