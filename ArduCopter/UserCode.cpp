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
