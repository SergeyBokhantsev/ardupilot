#include "AP_Smaud.h"
#include <AP_Notify/AP_Notify.h>
#include <AP_Logger/AP_Logger.h>

const AP_Param::GroupInfo AP_Smaud::var_info[] = {
    // @Param: PWRHI
    // @DisplayName: SmartAudio power high
    // @Description: SmartAudio power high used with switch function #47
    // @Units: point
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("PWRHI", 0, AP_Smaud, _power_hi, 1),

	// @Param: PWRLO
    // @DisplayName: SmartAudio power low
    // @Description: SmartAudio power low used with switch function #47
    // @Units: point
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("PWRLO", 1, AP_Smaud, _power_lo, 0),

    // @Param: APWRZ0
    // @DisplayName: Boundary of the power level #0 zone
    // @Description: Entering that zone power level #0 will be applied. Leaving that zone power level #1 will be applied
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("APWRZ0", 2, AP_Smaud, _auto_power_zone0, 0),
    
    // @Param: APWRZ1
    // @DisplayName: Additional boundary (to the #0 boundary) of the power level #1 zone
    // @Description: Entering that zone power level #1 will be applied. Leaving that zone power level #2 will be applied
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("APWRZ1", 3, AP_Smaud, _auto_power_zone1, 0),
    
    // @Param: APWRZ2
    // @DisplayName: Additional boundary (to the #0+1 boundary) of the power level #2 zone
    // @Description: Entering that zone power level #2 will be applied. Leaving that zone power level #3 will be applied
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("APWRZ2", 4, AP_Smaud, _auto_power_zone2, 0), 

    AP_GROUPINFO("CHAN", 5, AP_Smaud, _channel, 31),     
    
    AP_GROUPEND
};

AP_Smaud::AP_Smaud() : 
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
void AP_Smaud::init(const AP_SerialManager &serial_manager, GCS *gcs)
{
    _gcs = gcs;
    
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Smaud, 0))) 
        _uart_exists = true;
}

bool AP_Smaud::activate_port(uint8_t mode)
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

void AP_Smaud::send_text()
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

bool AP_Smaud::set_power(int8_t value)
{
    if (_current_vtx_pwr == value)
        return false;
    
    write_SMAUD_VTX((uint8_t)value, _power_zone, _power_mode);
    
    uint8_t CMD_SET_POWER[] = { 0x05, 0x01, 0x00 };
    CMD_SET_POWER[2] = (uint8_t)value;
    
    send_v2_command(CMD_SET_POWER, sizeof(CMD_SET_POWER));
    
    _current_vtx_pwr = value;
    
    AP_Notify::flags.vtx_power = value;
    
    return true;
}

void AP_Smaud::write_SMAUD_VTX(uint8_t value, int8_t power_zone, uint8_t power_mode)
{
	const struct log_SMAUD_VTX_PWR pkt {
    LOG_PACKET_HEADER_INIT(LOG_SMAUD_VTX_PWR_MSG),
    time_us     : AP_HAL::micros64(),
    power 		: value,
    zone		: power_zone,
    mode		: power_mode
	};
	AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void AP_Smaud::set_pit_mode(bool enabled)
{
    uint8_t CMD_SET_PIT_MODE[] = { 0x0B, 0x01, 0x00 };
    CMD_SET_PIT_MODE[2] = enabled ? 0x01 : 0x04;
    send_v2_command(CMD_SET_PIT_MODE, sizeof(CMD_SET_PIT_MODE));
}

bool AP_Smaud::update_channel()
{
    if (_current_vtx_ch == _channel)
        return false;
    
    uint8_t CMD_SET_CH[] = { 0x07, 0x01, 0x00 };
    CMD_SET_CH[2] = _channel;
    send_v2_command(CMD_SET_CH, sizeof(CMD_SET_CH));
    
    _current_vtx_ch = _channel;
    
    AP_Notify::flags.vtx_channel = _channel;
    
    return true;
}

void AP_Smaud::toggle_recording()
{    
    send_rc_split_command(command_toggle_rec, 3);
    _gcs->send_text(MAV_SEVERITY_INFO, "REC switch");
}

void AP_Smaud::toggle_wifi()
{    
    send_rc_split_command(command_toggle_wifi, 3);
    _gcs->send_text(MAV_SEVERITY_INFO, "REC toggle");
}

void AP_Smaud::update(float home_dist_meters)
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

void AP_Smaud::send_v2_command(uint8_t* data, uint8_t len)
{
    // Reserve 2 bytes for sync and header
    if (len > SMARTAUDIO_V2_COMMAND_LEN_MAX - 2) {		
        return;
	}
    
	if (activate_port(SMARTAUDIO_PORT_MODE_SMAUDv2))
	{
        uint8_t buflen = 2;
        _command_buffer[0] = (uint8_t)SMARTAUDIO_V2_COMMAND_SYNC;
        _command_buffer[1] = (uint8_t)SMARTAUDIO_V2_COMMAND_HEADER;
        
        for(int i=0; i<len; ++i)
		{
			_command_buffer[buflen++] = *data++;
		}
        
        uint8_t crc = crc8(_command_buffer, buflen);
        
		_port->write((uint8_t)SMARTAUDIO_V2_COMMAND_LOW);
        
		for(int i=0; i<buflen; ++i)
		{
			_port->write(_command_buffer[i]);
		}
        
        _port->write(crc);
        
        // TBS Unify Pro HV SE cant work without this
        _port->write((uint8_t)SMARTAUDIO_V2_COMMAND_LOW);
	}
}

uint8_t AP_Smaud::crc8(const uint8_t* ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i=0; i<len; i++) {
        crc = crc8tab[crc ^ *ptr++];
    }
    return crc;
}

void AP_Smaud::send_rc_split_command(uint8_t* data, uint8_t len)
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
