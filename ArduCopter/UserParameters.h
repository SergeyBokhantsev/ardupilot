#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Smaud/AP_Smaud.h>

#define USR_SWITCH_NONE 0
#define USR_SWITCH_VTX_PITMODE 1
#define USR_SWITCH_VTX_AUTO_PRW_OR_FORCE_LOW 2
#define USR_SWITCH_VTX_CHANNEL_SELECT 3
#define USR_SWITCH_VTX_FORCELOW_OR_AUTO_OR_FORCEHIGH 4
#define USR_SWITCH_RUNCAM_SPLIT_TOGGLE_REC 5
#define USR_SWITCH_RUNCAM_SPLIT_TOGGLE_WIFI 6
#define USR_SWITCH_LED_CONTROL 7

#define AUXSWITCH_LOW 0
#define AUXSWITCH_MEDIUM 1
#define AUXSWITCH_HIGH 2

class UserParameters {

public:
    UserParameters() {}
    static const struct AP_Param::GroupInfo var_info[];
    
    // UserCode usage example: g2.user_parameters.get_int8Param()
    
    void doSwitch(uint8_t sw, uint8_t ch_flag, AP_Smaud& smart_audio);
    
    float getCruiseAmps() const { return _cruise_amps; }
    uint8_t getCruiseJerk() const { return _cruise_jerk; }

private:
    AP_Int8 _usr_sw1_func;
    AP_Int8 _usr_sw2_func;
    AP_Int8 _usr_sw3_func;    
    AP_Float _cruise_amps;
    AP_Int8 _cruise_jerk;
    
    void doSwitch(AP_Int8 _usr_sw_func, uint8_t ch_flag, AP_Smaud& smart_audio);
    void vtxPitmode(AP_Smaud& smart_audio, bool enabled);
    void vtxAutoOrForceLow(AP_Smaud& smart_audio, bool force_low);
    void vtxChannelSelect(AP_Smaud& smart_audio, uint8_t ch_flag);
    void vtxForceLowOrAutoOrForceHigh(AP_Smaud& smart_audio, uint8_t ch_flag);    
    void runcamToggleRecording(AP_Smaud& smart_audio, uint8_t ch_flag);
    void runcamToggleWifi(AP_Smaud& smart_audio, uint8_t ch_flag);    
};
