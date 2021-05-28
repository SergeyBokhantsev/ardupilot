#include "UserParameters.h"
#include "defines.h"
#include <RC_Channel/RC_Channel.h>
#include <AP_Notify/AP_Notify.h>

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    
    // USER SWITH #1 FUNCTION (CHx_OPT = 47)
    AP_GROUPINFO("_SW1_FUNC", 0, UserParameters, _usr_sw1_func, USR_SWITCH_NONE),
    // USER SWITH #2 FUNCTION (CHx_OPT = 48)
    AP_GROUPINFO("_SW2_FUNC", 1, UserParameters, _usr_sw2_func, USR_SWITCH_NONE),
    // USER SWITH #3 FUNCTION (CHx_OPT = 49)
    AP_GROUPINFO("_SW3_FUNC", 2, UserParameters, _usr_sw3_func, USR_SWITCH_NONE),
    
    // CRUISE CONTROL PARAMS
    AP_GROUPINFO("_CRZ_AMPS", 4, UserParameters, _cruise_amps, 0), // keep batt current below this value
    AP_GROUPINFO("_CRZ_JERK", 5, UserParameters, _cruise_jerk, 10), // batt current limitation sharpness (more - quicker)

    AP_GROUPEND
};

void UserParameters::doSwitch(uint8_t sw, uint8_t ch_flag, AP_Smaud& smart_audio)
{
    switch(sw) {
        case 1: doSwitch(_usr_sw1_func, ch_flag, smart_audio); break;
        case 2: doSwitch(_usr_sw2_func, ch_flag, smart_audio); break;
        case 3: doSwitch(_usr_sw3_func, ch_flag, smart_audio); break;
    }    
}

void UserParameters::doSwitch(AP_Int8 _usr_sw_func, uint8_t ch_flag, AP_Smaud& smart_audio)
{
    switch (_usr_sw_func) {
        case USR_SWITCH_VTX_PITMODE: 
            vtxPitmode(smart_audio, ch_flag > 0); 
            break;
        
        case USR_SWITCH_VTX_AUTO_PRW_OR_FORCE_LOW: 
            vtxAutoOrForceLow(smart_audio, ch_flag == AUXSWITCH_HIGH); 
            break;
            
        case USR_SWITCH_VTX_CHANNEL_SELECT:
            vtxChannelSelect(smart_audio, ch_flag);
            break;
        
        case USR_SWITCH_VTX_FORCELOW_OR_AUTO_OR_FORCEHIGH:
            vtxForceLowOrAutoOrForceHigh(smart_audio, ch_flag);
            break;            
            
        case USR_SWITCH_RUNCAM_SPLIT_TOGGLE_REC:
            runcamToggleRecording(smart_audio, ch_flag);
            break;
            
        case USR_SWITCH_RUNCAM_SPLIT_TOGGLE_WIFI:
            runcamToggleWifi(smart_audio, ch_flag);
            break;
            
        case USR_SWITCH_LED_CONTROL:            
            AP_Notify::flags.leds_disabled = ch_flag == AUXSWITCH_LOW;
            break;
    }
}

void UserParameters::vtxPitmode(AP_Smaud& smart_audio, bool enabled)
{
    smart_audio.set_pit_mode(enabled);
}

void UserParameters::vtxAutoOrForceLow(AP_Smaud& smart_audio, bool force_low)
{
    if (force_low){
        smart_audio.set_power_mode(SMARTAUDIO_POWER_MODE_LO);
    }
    else {
        smart_audio.set_power_mode(SMARTAUDIO_POWER_MODE_AUTO);
    }
}

void UserParameters::vtxChannelSelect(AP_Smaud& smart_audio, uint8_t ch_flag)
{
   uint8_t chan = smart_audio.get_channel();
    
    switch (ch_flag)
    {
        case AUXSWITCH_HIGH:
            chan = chan == 39 ? 0 : chan+1;
            break;
        case AUXSWITCH_LOW:
            chan = chan == 0 ? 39 : chan-1;
            break;
    }
    
    smart_audio.set_channel(chan); 
}

void UserParameters::vtxForceLowOrAutoOrForceHigh(AP_Smaud& smart_audio, uint8_t ch_flag)
{
    smart_audio.set_power_mode(ch_flag);
}

void UserParameters::runcamToggleRecording(AP_Smaud& smart_audio, uint8_t ch_flag)
{
    if (ch_flag == AUXSWITCH_HIGH)
        smart_audio.toggle_recording();
}

void UserParameters::runcamToggleWifi(AP_Smaud& smart_audio, uint8_t ch_flag)
{
    if (ch_flag == AUXSWITCH_HIGH)
        smart_audio.toggle_wifi();
}