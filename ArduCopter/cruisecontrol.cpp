
#include "cruisecontrol.h"
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Logger/AP_Logger.h>

CruiseControl::CruiseControl() :
 initialized(false),
 state(CC_STATE_DISENGAGED),
 error(CC_ERROR_NOTINIT)
{
}

void CruiseControl::init(RC_Channel* _channel_cruise, RC_Channel* _channel_pitch, RC_Channel* _channel_roll, float _cruise_amps, uint8_t _cruise_jerk)
{
    channel_cruise = _channel_cruise;
    channel_pitch = _channel_pitch;
    channel_roll = _channel_roll;
    cruise_amps = _cruise_amps;
    cruise_jerk = (float)_cruise_jerk;

    error = 0;

    if (channel_cruise == nullptr)
        error |= CC_ERROR_NOCRUISE_CHAN;

    if (channel_pitch == nullptr)
        error |= CC_ERROR_NOPITCH_CHAN;

    else if (channel_roll == nullptr)
        error |= CC_ERROR_NOROLL_CHAN;

    if (error == 0)
        initialized = true;
}

void CruiseControl::update(float battery_amps)
{
    float cruise_ratio = 0.0f;

    if (!initialized)
        return;

    error = 0;

    if (AP_Notify::flags.flying)
        error &= ~CC_ERROR_NOT_FLYING;
    else
        error |= CC_ERROR_NOT_FLYING;

    if (AP_Notify::flags.armed)
        error &= ~CC_ERROR_NOT_ARMED;
    else
        error |= CC_ERROR_NOT_ARMED;

    if (error == 0)
    {
        if (state == CC_STATE_DISENGAGED)
        {
            // Allow engage condition: cruise ch value is minimum and pitch/roll are centered
            if (channel_cruise->norm_input_dz() < -0.90f && channel_pitch->in_trim_dz() && channel_roll->in_trim_dz())
            {
                state = CC_STATE_READY_TO_ENGAGE;
            }
            else
                error = CC_ERROR_STICKS;
        }
        else
        {
            // keep cruise while pitch/roll not moving
            if (channel_pitch->in_trim_dz() && channel_roll->in_trim_dz())
            {
                cruise_ratio = (channel_cruise->norm_input() + 1.0f) / 2.0f;

                switch (state)
                {
                    case CC_STATE_READY_TO_ENGAGE:
                        if (cruise_ratio > 0.1f)
                            state = CC_STATE_ENGAGED;
                        break;

                    case CC_STATE_ENGAGED:
                        if (cruise_ratio > 0.1f)
                            update_pitch(cruise_ratio, battery_amps);
                        else
                            disengage();
                        break;
                }
            }
            else
            {
                disengage();
            }
        }
    }
    else
    {
        disengage();
    }

    AP::logger().Write_CRUISE_CONTROL(error, state, cruise_ratio);
}

void CruiseControl::update_pitch(float cruise_ratio, float battery_amps)
{
    if (cruise_amps > 0.0f && battery_amps > 0.0f)
    {
        float fade = constrain_float(abs(cruise_amps - battery_amps) / 3.0f, 0.0f, 1.0f);

        if (fade > 0.0f)
        {
            if (battery_amps > cruise_amps)
                cruise_static_ratio -=  cruise_jerk / (1000.0f * fade);
            else
                cruise_static_ratio +=  cruise_jerk / (1000.0f * fade);
        }
    }
    else
        cruise_static_ratio = 1.0f;
    
    cruise_static_ratio = constrain_float(cruise_static_ratio, 0.2f, 1.0f);

    float range = channel_pitch->get_reverse() 
                   ? (float)(channel_pitch->get_radio_max() - channel_pitch->get_radio_trim())
                   : (float)(channel_pitch->get_radio_trim() - channel_pitch->get_radio_min());

    if (range < 0.0f || range > channel_pitch->get_radio_max())
    {
        error = CC_ERROR_PITCH_RANGE;
        disengage();
        return;
    }

    float pitchx = range * cruise_ratio * cruise_static_ratio;

    channel_pitch->set_radio_in(channel_pitch->get_reverse() 
                                ? channel_pitch->get_radio_trim() + (uint16_t)pitchx
                                : channel_pitch->get_radio_trim() - (uint16_t)pitchx);

    channel_pitch->recompute_pwm_no_deadzone();
}
