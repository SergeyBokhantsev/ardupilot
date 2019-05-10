#include "AP_SmartAudio.h"
#include <AP_Notify/AP_Notify.h>
#include <AP_HAL/AP_HAL.h>

const AP_Param::GroupInfo AP_SmartAudio::var_info[] = {
    // @Param: PWRHI
    // @DisplayName: SmartAudio power high
    // @Description: SmartAudio power high used with switch function #47
    // @Units: point
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("PWRHI", 0, AP_SmartAudio, _power_hi, 1),

	// @Param: PWRLO
    // @DisplayName: SmartAudio power low
    // @Description: SmartAudio power low used with switch function #47
    // @Units: point
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("PWRLO", 1, AP_SmartAudio, _power_lo, 0),

    // @Param: APWRZ0
    // @DisplayName: Boundary of the power level #0 zone
    // @Description: Entering that zone power level #0 will be applied. Leaving that zone power level #1 will be applied
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("APWRZ0", 2, AP_SmartAudio, _auto_power_zone0, 0),
    
    // @Param: APWRZ1
    // @DisplayName: Additional boundary (to the #0 boundary) of the power level #1 zone
    // @Description: Entering that zone power level #1 will be applied. Leaving that zone power level #2 will be applied
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("APWRZ1", 3, AP_SmartAudio, _auto_power_zone1, 0),
    
    // @Param: APWRZ2
    // @DisplayName: Additional boundary (to the #0+1 boundary) of the power level #2 zone
    // @Description: Entering that zone power level #2 will be applied. Leaving that zone power level #3 will be applied
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("APWRZ2", 4, AP_SmartAudio, _auto_power_zone2, 0), 

    AP_GROUPINFO("CHAN", 5, AP_SmartAudio, _channel, 31),     
    
    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

AP_SmartAudio::AP_SmartAudio() : 
_uart_exists(false),
_port_mode(SMARTAUDIO_PORT_MODE_NONE),
_power_zone(-1),
_power_mode(SMARTAUDIO_POWER_MODE_LO),
_current_vtx_pwr(-1),
_current_vtx_ch(-1)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
 * init - perform required initialisation
 */
void AP_SmartAudio::init(const AP_SerialManager &serial_manager, GCS *gcs)
{
    _gcs = gcs;
    
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_SmartAudio, 0))) 
        _uart_exists = true;
}

bool AP_SmartAudio::activate_port(uint8_t mode)
{
    if (_port_mode == mode)
        return true;
    
    _port_mode = mode;
    
    if (_uart_exists)
    {
        switch(_port_mode)
        {
            case SMARTAUDIO_PORT_MODE_SMAUDv2:
                // http://www.team-blacksheep.com/tbs_smartaudio_rev08.pdf
                _port->begin(SMARTAUDIO_V2_BAUDRATE, 30, 30);
                _port->set_stop_bits(2);
                _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);                
                return true;
                    
            case SMARTAUDIO_PORT_MODE_RC_SPLIT:    
                // https://support.runcam.com/hc/en-us/articles/115011786628-RunCam-Split-Communication-protocol
                _port->begin(SMARTAUDIO_RC_SPLIT_BAUDRATE, 30, 30);
                _port->set_stop_bits(1);
                _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);                
                return true;
        }
    }
    
    _port_mode = SMARTAUDIO_PORT_MODE_NONE;
    return false;
}

void AP_SmartAudio::send_text()
{
    int mw = _current_vtx_pwr == 0 ? 25
           : _current_vtx_pwr == 1 ? 200
           : _current_vtx_pwr == 2 ? 600
           : 1200;
    
    int chan = 0;
    char band = 'U';
    
    // Band A
    if (_current_vtx_ch < 8) {
        chan = _current_vtx_ch + 1;
        band = 'A';
    } 
    // Band B
    else if (AP_Notify::flags.vtx_channel < 16) {
        chan = _current_vtx_ch - 7;
        band = 'B';
    }
    // Band E
    else if (AP_Notify::flags.vtx_channel < 24) {
        chan = _current_vtx_ch - 15;
        band = 'E';
    }
    // Band Airwave
    else if (AP_Notify::flags.vtx_channel < 32) {
        chan = _current_vtx_ch - 23;
        band = 'F';
    }
    // Band Race
    else if (AP_Notify::flags.vtx_channel < 40) {
        chan = _current_vtx_ch - 31;
        band = 'R';
    }   
    
    char mode = 'U';
    
    switch (_power_mode) {
        case SMARTAUDIO_POWER_MODE_HI:
            mode = 'H';
            break;
        case SMARTAUDIO_POWER_MODE_AUTO:
            mode = 'A';
            break;
        case SMARTAUDIO_POWER_MODE_LO:
            mode = 'L';
            break;
    }
    
    _gcs->send_text(MAV_SEVERITY_INFO, "VTX %c:%d %dMW %c", band, chan, mw, mode);
}

bool AP_SmartAudio::set_power(int8_t value)
{
    if (_current_vtx_pwr == value)
        return false;
        
    if (send_v2_command(SMARTAUDIO_V2_COMMAND_SET_POWER, (uint8_t*)&value, 1)
        && frame.meta.command == SMARTAUDIO_V2_COMMAND_SET_POWER
        && frame.meta.data_len == 1) {
        _current_vtx_pwr = frame.data[0];
        DataFlash_Class::instance()->Log_Write_SMAUD_VTX((uint8_t)_current_vtx_pwr, _power_zone, _power_mode);        
        AP_Notify::flags.vtx_power = _current_vtx_pwr;
        return true;
    }
}

void AP_SmartAudio::set_pit_mode(bool enabled)
{
    uint8_t value = enabled ? 0x01 : 0x04;
    if (send_v2_command(SMARTAUDIO_V2_COMMAND_SET_MODE, &value, 1))
    {
        
    }
}

bool AP_SmartAudio::update_channel()
{
    if (_current_vtx_ch == _channel)
        return false;
    
    _current_vtx_ch = _channel;
    
    send_v2_command(SMARTAUDIO_V2_COMMAND_SET_CHANNEL, (uint8_t*)(&_current_vtx_ch), 1);
    
    AP_Notify::flags.vtx_channel = _channel;
    
    return true;
}

void AP_SmartAudio::toggle_recording()
{    
    send_rc_split_command(command_toggle_rec, 3);
    _gcs->send_text(MAV_SEVERITY_INFO, "REC switch");
}

void AP_SmartAudio::update(float home_dist_meters)
{
    bool command_sent = false;
    
    command_sent = update_channel();
    
    if (home_dist_meters <= _auto_power_zone0)
        _power_zone = 0;
    else if (home_dist_meters <= _auto_power_zone0 + _auto_power_zone1)
        _power_zone = 1;
    else if (home_dist_meters <= _auto_power_zone0 + _auto_power_zone1 + _auto_power_zone2)
        _power_zone = 2;
    else
        _power_zone = 3;
    
    // If other command was sent then exit to make some pause before another possible command
    if (!command_sent)
    {
        int8_t desired_power;
        
        if (is_auto_power_enabled()){        
            desired_power = _power_zone;        
        }
        else if (_power_mode == SMARTAUDIO_POWER_MODE_HI){
            desired_power = _power_hi;
        }
        else {
            desired_power = _power_lo;
        }
        
        command_sent = set_power(desired_power);        
    }
    
    if (command_sent) {
        msg_pending = true;
    }
    else if (msg_pending){
        send_text();
        msg_pending = false;
    }
}

bool AP_SmartAudio::send_v2_command(uint8_t command, uint8_t* data, uint8_t len)
{
    if (len > SMARTAUDIO_V2_COMMAND_LEN_MAX)
        return false;
    
	if (activate_port(SMARTAUDIO_PORT_MODE_SMAUDv2))
	{
        frame.meta.command = (command << 1) | 0x01;
        frame.meta.data_len = len;
        
        for (int i=0; i<len; ++i) {
            frame.data[i] = data[i];
        }
        
        // Write        
        uint8_t size = frame.size();
        uint8_t crc = crc8();
        
		_port->write((uint8_t)0x00);      
		for(int i=0; i < size; ++i)
		{
			_port->write(frame.ptr()[i]);
		}        
        _port->write(crc);
        
        // Read
        int16_t elapsedMs = 0;
        int8_t delayMs = 20;
        uint8_t state = 0;
        uint8_t received = 0;
        
        if (_port->available()){
            int16_t b = _port->read();
            
            if (b < 0 || b > 255) return false;
            
            switch (state) {
                // Looking for SMARTAUDIO_V2_COMMAND_SYNC
                case 0:
                    if (b == SMARTAUDIO_V2_COMMAND_SYNC) state++; 
                    break;
                    // Looking for SMARTAUDIO_V2_COMMAND_HEADER
                case 1:
                    if (b == SMARTAUDIO_V2_COMMAND_HEADER) state++; 
                    break;
                    // COMMAND
                case 2:
                    if (b != frame.meta.command) {
                        frame.meta.command = b;
                        state++; 
                    }
                    else state = 0; //we've catch own echo, so repeat waiting for the VTX response
                    break;
                    // DATA LEN
                case 3:
                    frame.meta.data_len = b;
                    if (frame.meta.data_len > SMARTAUDIO_V2_COMMAND_LEN_MAX) {
                        return false;
                    }
                    state++;
                    break;
                    // READING DATA
                case 4:
                    if (received < frame.meta.data_len) {
                        frame.data[received++] = b;
                    }
                    else {
                        state++;
                    }
                    break;
                    // CRC
                case 5:
                    if (crc8() == b) {
                        return true;
                    }
                    else {
                        return false;
                    }
                    break;
            }
        }
        else if (elapsedMs < 1000) {            
            hal.scheduler->delay(delayMs);
            elapsedMs += delayMs;
        }
        else {
            return false;
        }
	}
}

uint8_t AP_SmartAudio::crc8()
{
    uint8_t crc = 0;
    uint8_t size = frame.size();
    for (uint8_t i=0; i<size; i++) {
        crc = crc8tab[crc ^ frame.ptr()[i]];
    }
    return crc;
}

void AP_SmartAudio::send_rc_split_command(uint8_t* data, uint8_t len)
{
	if (activate_port(SMARTAUDIO_PORT_MODE_RC_SPLIT))
	{
		_port->write((uint8_t)SMARTAUDIO_RC_SPLIT_COMMAND_HEADER);
		
		for(int i=0; i<len; ++i)
		{
			_port->write(*data++);
		}
        
        _port->write((uint8_t)SMARTAUDIO_RC_SPLIT_COMMAND_TAILER);
	}
}
