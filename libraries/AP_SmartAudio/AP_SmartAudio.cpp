#include "AP_SmartAudio.h"
//#include <GCS_MAVLink/GCS.h>

//extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_SmartAudio::var_info[] = {
    // @Param: POWER_HI
    // @DisplayName: SmartAudio power high
    // @Description: SmartAudio power high used with switch function #47
    // @Units: point
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("POWER_HI", 0, AP_SmartAudio, _power_hi, 1),

	// @Param: POWER_LO
    // @DisplayName: SmartAudio power low
    // @Description: SmartAudio power low used with switch function #47
    // @Units: point
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("POWER_LO", 0, AP_SmartAudio, _power_lo, 0),

    // @Param: AUTOPWR_ZONE0
    // @DisplayName: Boundary of the power level #0 zone
    // @Description: Entering that zone power level #0 will be applied. Leaving that zone power level #1 will be applied
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("AUTOPWR_ZONE0", 0, AP_SmartAudio, _auto_power_zone0, 0),
    
    // @Param: AUTOPWR_ZONE1
    // @DisplayName: Additional boundary (to the #0 boundary) of the power level #1 zone
    // @Description: Entering that zone power level #1 will be applied. Leaving that zone power level #2 will be applied
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("AUTOPWR_ZONE1", 0, AP_SmartAudio, _auto_power_zone1, 0),
    
    // @Param: AUTOPWR_ZONE2
    // @DisplayName: Additional boundary (to the #0+1 boundary) of the power level #2 zone
    // @Description: Entering that zone power level #2 will be applied. Leaving that zone power level #3 will be applied
    // @Units: meters
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("AUTOPWR_ZONE2", 0, AP_SmartAudio, _auto_power_zone2, 0),    
    
    AP_GROUPEND
};

AP_SmartAudio::AP_SmartAudio() : 
_uart_exists(false),
_port_mode(SMARTAUDIO_PORT_MODE_NONE),
_power_zone(-1)
{
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

void AP_SmartAudio::set_power_hi()
{
	set_power(_power_hi);
}

void AP_SmartAudio::set_power_lo()
{
	set_power(_power_lo);
}

void AP_SmartAudio::set_power(uint8_t value)
{
	switch(value)
	{
		case 0:	send_v2_command(command_power_0, 4); _gcs->send_text(MAV_SEVERITY_INFO, "VTX PWR 0"); break;
		case 1:	send_v2_command(command_power_1, 4); _gcs->send_text(MAV_SEVERITY_INFO, "VTX PWR 1"); break;
		case 2:	send_v2_command(command_power_2, 4); _gcs->send_text(MAV_SEVERITY_INFO, "VTX PWR 2"); break;
		case 3:	send_v2_command(command_power_3, 4); _gcs->send_text(MAV_SEVERITY_INFO, "VTX PWR 3"); break;
	}
}

void AP_SmartAudio::toggle_recording()
{
	send_rc_split_command(command_toggle_rec, 3);
    _gcs->send_text(MAV_SEVERITY_INFO, "REC switch");
}

void AP_SmartAudio::check_home_distance(float meters)
{
	if (_auto_power_zone0 > SMARTAUDIO_AUTOPOWER_HISTERESIS * 2
        && _auto_power_zone1 > SMARTAUDIO_AUTOPOWER_HISTERESIS * 2
        && _auto_power_zone2 > SMARTAUDIO_AUTOPOWER_HISTERESIS * 2)
    {
        int8_t actual_zone;
        
        if (meters < _auto_power_zone0)
            actual_zone = 0;
        else if (meters < _auto_power_zone0 + _auto_power_zone1)
            actual_zone = 1;
        else if (meters < _auto_power_zone0 + _auto_power_zone1 + _auto_power_zone2)
            actual_zone = 2;
        else
            actual_zone = 3;
        
        if (_power_zone != actual_zone)
        {
            _power_zone = actual_zone;
            set_power(_power_zone);
        }
    }
}

void AP_SmartAudio::send_v2_command(uint8_t* data, uint8_t len)
{
	if (activate_port(SMARTAUDIO_PORT_MODE_SMAUDv2))
	{
		_port->write((uint8_t)SMARTAUDIO_V2_COMMAND_LOW);
		_port->write((uint8_t)SMARTAUDIO_V2_COMMAND_SYNC);
		_port->write((uint8_t)SMARTAUDIO_V2_COMMAND_HEADER);
		
		for(int i=0; i<len; ++i)
		{
			_port->write(*data++);
		}
	}
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
