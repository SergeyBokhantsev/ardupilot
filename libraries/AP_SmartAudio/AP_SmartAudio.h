#pragma once

#include <DataFlash/DataFlash.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

#define SMARTAUDIO_V2_BAUDRATE 4800
#define SMARTAUDIO_RC_SPLIT_BAUDRATE 115200

#define SMARTAUDIO_V2_COMMAND_LOW 0x00
#define SMARTAUDIO_V2_COMMAND_SYNC 0xAA
#define SMARTAUDIO_V2_COMMAND_HEADER 0x55

#define SMARTAUDIO_V2_COMMAND_POWER_0 { 0x05, 0x01, 0x00, 0x6B }
#define SMARTAUDIO_V2_COMMAND_POWER_1 { 0x05, 0x01, 0x01, 0xBE }
#define SMARTAUDIO_V2_COMMAND_POWER_2 { 0x05, 0x01, 0x02, 0x14 }
#define SMARTAUDIO_V2_COMMAND_POWER_3 { 0x05, 0x01, 0x03, 0xC1 }

#define SMARTAUDIO_RC_SPLIT_COMMAND_HEADER 0x55
#define SMARTAUDIO_RC_SPLIT_COMMAND_TAILER 0xAA

#define SMARTAUDIO_RC_SPLIT_COMMAND_TOGGLE_WIFI { 0x01, 0x01, 0xd8 }
#define SMARTAUDIO_RC_SPLIT_COMMAND_TOGGLE_REC  { 0x01, 0x02, 0xf5 }
#define SMARTAUDIO_RC_SPLIT_COMMAND_TOGGLE_MODE { 0x01, 0x03, 0x01 }

#define SMARTAUDIO_PORT_MODE_NONE 0
#define SMARTAUDIO_PORT_MODE_SMAUDv2 1
#define SMARTAUDIO_PORT_MODE_RC_SPLIT 2

#define SMARTAUDIO_AUTOPOWER_HISTERESIS 5

// Power mode numbers matches Copter/switches.cpp
#define SMARTAUDIO_POWER_MODE_HI 2
#define SMARTAUDIO_POWER_MODE_AUTO 1
#define SMARTAUDIO_POWER_MODE_LO 0

class AP_SmartAudio {
public:
    AP_SmartAudio();

    // init - perform required initialisation
    void init(const AP_SerialManager &serial_manager, GCS *gcs);
    
    // update flight control mode. The control mode is vehicle type specific
    void set_power(int8_t value);
    void set_power_mode(uint8_t mode);
    void toggle_recording();
    void check_home_distance(const float meters);
    
    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];
	
private:
    AP_HAL::UARTDriver *_port;
    GCS *_gcs;
    
    bool _uart_exists;
    uint8_t _port_mode;
    int8_t _current_vtx_mode;
    int8_t _power_zone;
    
    uint8_t _power_mode;    

    AP_Int8 _power_lo;
    AP_Int8 _power_hi;
    
    AP_Int16 _auto_power_zone0;
    AP_Int16 _auto_power_zone1;
    AP_Int16 _auto_power_zone2;
    
    uint8_t command_power_0[4] SMARTAUDIO_V2_COMMAND_POWER_0;
    uint8_t command_power_1[4] SMARTAUDIO_V2_COMMAND_POWER_1;
    uint8_t command_power_2[4] SMARTAUDIO_V2_COMMAND_POWER_2;
    uint8_t command_power_3[4] SMARTAUDIO_V2_COMMAND_POWER_3;
    
    uint8_t command_toggle_wifi[3] SMARTAUDIO_RC_SPLIT_COMMAND_TOGGLE_WIFI;
    uint8_t command_toggle_rec[3] SMARTAUDIO_RC_SPLIT_COMMAND_TOGGLE_REC;
    uint8_t command_toggle_mode[3] SMARTAUDIO_RC_SPLIT_COMMAND_TOGGLE_MODE;

    bool is_auto_power_enabled() { return _power_mode == SMARTAUDIO_POWER_MODE_AUTO && _auto_power_zone0 > 0 && _auto_power_zone1 > 0 && _auto_power_zone2 > 0; }

    bool activate_port(uint8_t mode);
    void send_v2_command(uint8_t* data, uint8_t len);
    void send_rc_split_command(uint8_t* data, uint8_t len);
};
