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
    if (smaud_update_delay_sec == 0)
    {
        if (get_likely_flying())
        {
            Location loc;
            if (ahrs.get_position(loc))
            {
                const Location &home_loc = ahrs.get_home();
                if (home_loc.lat != 0 || home_loc.lng != 0)
                {
                    g2.smaud.update(home_loc.get_distance(loc));
                }
            }
        }
        else
            g2.smaud.update(0);
    }
    else
        smaud_update_delay_sec--;
}
#endif

#ifdef USERHOOK_AUXSWITCH

// (CHx_OPT = 47)
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    g2.user_parameters.doSwitch(1, ch_flag, g2.smaud);
}

// (CHx_OPT = 48)
void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    g2.user_parameters.doSwitch(2, ch_flag, g2.smaud);
}

// (CHx_OPT = 49)
void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{    
    g2.user_parameters.doSwitch(3, ch_flag, g2.smaud);
}
#endif
