#pragma once

#include <AP_Logger/LogStructure.h>

struct PACKED log_SMAUD_VTX_PWR {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t power;
    int8_t zone;
    uint8_t mode;
};

#define LOG_STRUCTURE_FROM_SMAUD \
	{ LOG_SMAUD_VTX_PWR_MSG, sizeof(log_SMAUD_VTX_PWR), \
      "VTX", "QBbB", "TimeUS,Power,Zone,Mode", "s---", "F000" },
