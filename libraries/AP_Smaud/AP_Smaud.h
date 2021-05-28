#pragma once

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>

#define SMARTAUDIO_V2_BAUDRATE 4800
#define SMARTAUDIO_RC_SPLIT_BAUDRATE 115200

#define SMARTAUDIO_V2_COMMAND_LOW 0x00
#define SMARTAUDIO_V2_COMMAND_SYNC 0xAA
#define SMARTAUDIO_V2_COMMAND_HEADER 0x55

#define SMARTAUDIO_V2_COMMAND_LEN_MAX 10

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

class AP_Smaud {
public:
    AP_Smaud();

    // init - perform required initialisation
    void init(const AP_SerialManager &serial_manager, GCS *gcs);
    
    void set_power_mode(uint8_t mode) { _power_mode = mode; msg_pending = true; }
    void set_pit_mode(bool enabled);
    void set_channel(uint8_t chan) { _channel = (uint8_t)constrain_int16(chan, 0, 39); }// Band A Ch 1 = 0 | Band R Ch 8 = 39
    uint8_t get_channel() { return _channel; }
    void toggle_recording();
    void toggle_wifi();
    void update(const float home_dist_meters);
    
    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];
	
private:
    bool set_power(int8_t value);
    bool update_channel();
    void send_text();
	void write_SMAUD_VTX(uint8_t value, int8_t power_zone, uint8_t power_mode);

    AP_HAL::UARTDriver *_port;
    uint8_t _command_buffer[SMARTAUDIO_V2_COMMAND_LEN_MAX];
    
    GCS *_gcs;
    bool msg_pending;
    
    bool _uart_exists;
    int8_t _current_vtx_ch;
    uint8_t _port_mode;
    int8_t _current_vtx_pwr;
    int8_t _power_zone;
    
    uint8_t _power_mode;    

    AP_Int8 _power_lo;
    AP_Int8 _power_hi;
    
    AP_Int16 _auto_power_zone0;
    AP_Int16 _auto_power_zone1;
    AP_Int16 _auto_power_zone2;
    
    AP_Int8 _channel;
    
    uint8_t command_toggle_wifi[3] SMARTAUDIO_RC_SPLIT_COMMAND_TOGGLE_WIFI;
    uint8_t command_toggle_rec[3] SMARTAUDIO_RC_SPLIT_COMMAND_TOGGLE_REC;
    uint8_t command_toggle_mode[3] SMARTAUDIO_RC_SPLIT_COMMAND_TOGGLE_MODE;

    bool is_auto_power_enabled() { return _power_mode == SMARTAUDIO_POWER_MODE_AUTO && _auto_power_zone0 > 0 && _auto_power_zone1 > 0 && _auto_power_zone2 > 0; }

    bool activate_port(uint8_t mode);
    void send_v2_command(uint8_t* data, uint8_t len);
    void send_rc_split_command(uint8_t* data, uint8_t len);
    
    uint8_t crc8(const uint8_t* ptr, uint8_t len);
    
    const unsigned char crc8tab[256] = {
        0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
        0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
        0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
        0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
        0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
        0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
        0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
        0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
        0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
        0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
        0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
        0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
        0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
        0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
        0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
        0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};
};
