#include "AP_SmartAudio.h"
//#include <GCS_MAVLink/GCS.h>

//extern const AP_HAL::HAL& hal;

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
    
    AP_GROUPEND
};

AP_SmartAudio::AP_SmartAudio() : 
_uart_exists(false),
_port_mode(SMARTAUDIO_PORT_MODE_NONE),
_power_zone(-1),
_power_mode(SMARTAUDIO_POWER_MODE_LO),
_current_vtx_mode(-1)
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

void AP_SmartAudio::set_power_mode(uint8_t mode)
{
    _power_mode = mode;

    if (is_auto_power_enabled())
    {
        set_power(_power_zone);
    }
    else if (_power_mode == SMARTAUDIO_POWER_MODE_HI)
    {
        set_power(_power_hi);
    }
    else
    {
        set_power(_power_lo);
    }
}

void AP_SmartAudio::set_power(int8_t value)
{
    DataFlash_Class::instance()->Log_Write_SMAUD_VTX((uint8_t)value, _power_zone, _power_mode);
    
    switch(value)
    {
        case 0:	send_v2_command(command_power_0, 4); _gcs->send_text(MAV_SEVERITY_INFO, "VTX0"); break;                        
        case 1:	send_v2_command(command_power_1, 4); _gcs->send_text(MAV_SEVERITY_INFO, "VTX1"); break;
        case 2:	send_v2_command(command_power_2, 4); _gcs->send_text(MAV_SEVERITY_INFO, "VTX2"); break;
        case 3:	send_v2_command(command_power_3, 4); _gcs->send_text(MAV_SEVERITY_INFO, "VTX3"); break;
    }
    
    _current_vtx_mode = value;
}

void AP_SmartAudio::toggle_recording()
{    
    send_rc_split_command(command_toggle_rec, 3);
    _gcs->send_text(MAV_SEVERITY_INFO, "REC switch");
}

void AP_SmartAudio::check_home_distance(float meters)
{
    if (meters <= _auto_power_zone0)
        _power_zone = 0;
    else if (meters <= _auto_power_zone0 + _auto_power_zone1)
        _power_zone = 1;
    else if (meters <= _auto_power_zone0 + _auto_power_zone1 + _auto_power_zone2)
        _power_zone = 2;
    else
        _power_zone = 3;
    
    if (is_auto_power_enabled() && _current_vtx_mode != _power_zone)
        set_power(_power_zone);        
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
