#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    if (ahrs.get_likely_flying())
    {
        Location loc;
        if (ahrs.get_position(loc))
        {
            const Location &home_loc = ahrs.get_home();
            if (home_loc.lat != 0 || home_loc.lng != 0)
            {
                g2.smart_audio.check_home_distance(get_distance(home_loc, loc));
            }
        }
    }
    else
        g2.smart_audio.check_home_distance(0);
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // VTX FORCE POWER (CHx_OPT = 47)
#if SMARTAUDIO_ENABLED == ENABLED
    g2.smart_audio.set_power_mode(ch_flag);    
#endif
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // RunCam Split RECORD TOGGLE (CHx_OPT = 48)
#if SMARTAUDIO_ENABLED == ENABLED
    if (ch_flag == AUX_SWITCH_HIGH)
        g2.smart_audio.toggle_recording();
#endif
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif