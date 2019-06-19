#include "AP_SmartAudio.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#define SMARTAUDIO_BAUDRATE 4800

const AP_Param::GroupInfo AP_SmartAudio::var_info[] = {

    // @Param: PROTOCOL
    // @DisplayName: SmartAudio enabled
    // @Description: 
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLED", 1, AP_SmartAudio, enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TRAILZERO
    // @DisplayName: Enable trailing zero
    // @Description: If enabled the trailing zero will be sent after SA frame. Some VTX requires that.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("TRAILZERO", 2, AP_SmartAudio, use_trailing_zero, 0),

    AP_GROUPEND
};

AP_SmartAudio::AP_SmartAudio()
{
    AP_Param::setup_object_defaults(this, var_info);
    singleton = this;
}

void AP_SmartAudio::init()
{
    port = AP_SerialManager::get_singleton()->find_serial(AP_SerialManager::SerialProtocol_SmartAudio, 0);

    if (port != nullptr) {        
        port->begin(SMARTAUDIO_BAUDRATE, 16, 16);
        port->set_stop_bits(2);
        port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        port->set_options(AP_HAL::UARTDriver::OPTION_HDPLEX);        
    }
        
        gcs().send_text(MAV_SEVERITY_WARNING, "PORT enabled");
}

void AP_SmartAudio::send_command(uint8_t command, uint8_t* payload, uint8_t payload_size)
{
    // make a header
    buffer[0] = SA_SYNC_BYTE;
    buffer[1] = SA_HEADER_BYTE;
    // sent to the VTX command byte needs to be shifted left by one bit and the LSB needs to be set
    buffer[2] = (command << 1) | 0x01; 
    buffer[3] = payload_size;
        
    buffer_len = header_size + payload_size;
    
    if (buffer_len > SA_BUFFER_SIZE) {
        buffer_len = 0;
        return;
    }
    
    // make payload
    if (payload_size > 0) {
        for (int i=0; i < payload_size; ++i) {
            buffer[i + header_size] = payload[i];
        }
    }
    
    // pull line low
    port->write((uint8_t)0);
    
    // write request
    for(int i = 0; i < buffer_len; ++i) {
	    port->write(buffer[i]);
    }
    
    // write crc
    port->write(crc_crc8_dvb_s2(buffer, buffer_len));
        
    // some VTX need this
    if (use_trailing_zero) {
        port->write((uint8_t)0);
    }
    
    buffer_len = 0;
    last_io_time = AP_HAL::millis();
    
    gcs().send_text(MAV_SEVERITY_WARNING, "Command sent");
}

void AP_SmartAudio::read_response()
{
    int16_t nbytes = port->available();
    
    if (nbytes < 1) {        
        return;
    }
    
    gcs().send_text(MAV_SEVERITY_WARNING, "Incoming bytes");
    
    for (int i = 0; i < nbytes; ++i) {
        uint8_t b = port->read();
        
        if ((buffer_len == 0 && b != SA_SYNC_BYTE) 
            || (buffer_len == 1 && b != SA_HEADER_BYTE))
        {
            buffer_len = 0;
        }
        else if (buffer_len < SA_BUFFER_SIZE) {
            buffer[buffer_len++] = b;
        }
    }
    
    last_io_time = AP_HAL::millis();
    
    if (buffer_len < header_size) {        
        return;
    }
    
    uint8_t payload_len = buffer[3];
    
    if (buffer_len < payload_len + header_size) {
        // Not all response bytes received yet
        return;
    }
    else {
        buffer_len = payload_len + header_size;
    }
    
    uint8_t crc = buffer[buffer_len-1];
    
    if (crc != crc_crc8_dvb_s2(buffer, buffer_len-1)) {
        // crc missmatch
        gcs().send_text(MAV_SEVERITY_WARNING, "CRC missmatch");
        return;
    }
    
    parse_response(buffer[2], &(buffer[header_size]), payload_len-1);
}

void AP_SmartAudio::parse_response(uint8_t command, uint8_t* payload, uint8_t payload_size)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "parsing");
    
    switch (command){
        case SA_CMD_GET_SETTINGS: // GET_SETTINGS SA v1
        case 0x09:                // GET_SETTINGS SA v2
            if (payload_size == 5) {
                protocol = command == SA_CMD_GET_SETTINGS ? SA_PROTOCOL_v1 : SA_PROTOCOL_v2;
                channel = payload[0];
                power_level = payload[1];
                operation_mode = payload[2];
                frequency = (payload[3] << 8) + payload[4];
                // NA
                power_dBm = 0;
                power_levels_count = 0;
                gcs().send_text(MAV_SEVERITY_WARNING, "SA v1 or v2");
            }
            break;
           
        case 0x11:               // GET_SETTINGS SA v2.1
            if (payload_size == 12) {
                protocol = SA_PROTOCOL_v21;
                channel = payload[0];
                power_level = payload[1];
                operation_mode = payload[2];
                frequency = (payload[3] << 8) + payload[4];
                power_dBm = payload[5];
                power_levels_count = payload[6];
                gcs().send_text(MAV_SEVERITY_WARNING, "SA v21");
            }
            break;
    }
}

void AP_SmartAudio::update()
{
    if (!enabled && port == nullptr){
        return;
    }
    
    if (is_waiting_response()) {
        read_response();
    }
    else {
        if (get_settings_request) {
            send_command(SA_CMD_GET_SETTINGS, nullptr, 0);
            get_settings_request = false;
            gcs().send_text(MAV_SEVERITY_WARNING, "SA Refresh");
        }
        else if (power_level_request > -1) {
            uint8_t pwr = (uint8_t)power_level_request;
            send_command(SA_CMD_SET_POWER, &pwr, 1);
            power_level_request = -1;
        }
    }
}

AP_SmartAudio *AP_SmartAudio::singleton;