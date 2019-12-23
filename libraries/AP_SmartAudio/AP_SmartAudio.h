#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define SA_IO_TIMEOUT_MS 200

#define SA_BUFFER_SIZE 16
#define DBM_ARRAY_SIZE 6

#define SA_SYNC_BYTE 0xAA
#define SA_HEADER_BYTE 0x55

#define SA_CMD_GET_SETTINGS 0x01
#define SA_CMD_SET_POWER 0x02

#define SA_PROTOCOL_v1  0
#define SA_PROTOCOL_v2  1
#define SA_PROTOCOL_v21 2

class AP_SmartAudio
{

public:
    AP_SmartAudio();

    /* Do not allow copies */
    AP_SmartAudio(const AP_SmartAudio &other) = delete;
    AP_SmartAudio &operator=(const AP_SmartAudio&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();

    void set_power_level(uint8_t level) { power_level_request = level; }
    uint8_t get_power_level() const { return power_level; }

    void refresh_settings() { get_settings_request = true; }
    
    static AP_SmartAudio *get_singleton(void) { return singleton; }

private:
    static AP_SmartAudio* singleton;
    
    AP_HAL::UARTDriver* port;
    
    const uint8_t header_size = 4; // header bytes number of the request and response
    uint8_t buffer[SA_BUFFER_SIZE];
    uint8_t buffer_len;
    uint32_t last_io_time;

    void send_command(uint8_t command, uint8_t* payload, uint8_t payload_size);
    void read_response();
    void parse_response(uint8_t command, uint8_t* payload, uint8_t payload_size);
    bool is_waiting_response() const { return AP_HAL::millis() - last_io_time < SA_IO_TIMEOUT_MS; }

    // desired values
    bool get_settings_request;
    int8_t power_level_request;

    // actual values
    uint8_t protocol;
    uint8_t channel;
    uint8_t power_level;
    uint8_t power_dBm;
    uint8_t power_levels_count;
    uint8_t power_dBm_array[DBM_ARRAY_SIZE];
    uint8_t operation_mode;
    uint8_t frequency;

    //AP_Int8 protocol_version;
    AP_Int8 enabled;
    AP_Int8 use_trailing_zero;
};