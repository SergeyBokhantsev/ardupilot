
#include "cruisecontrol.h"
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>

CruiseControl::CruiseControl() :
 initialized(false),
 state(CC_STATE_DISENGAGED)
{
}

void CruiseControl::init(RC_Channel* _channel_cruise, RC_Channel* _channel_pitch, RC_Channel* _channel_roll, float _cruise_amps, uint8_t _cruise_jerk)
{
    channel_cruise = _channel_cruise;
    channel_pitch = _channel_pitch;
    channel_roll = _channel_roll;
    cruise_amps = _cruise_amps;
    cruise_jerk = (float)_cruise_jerk;

    if (channel_cruise != nullptr && channel_pitch != nullptr && channel_roll != nullptr)
        initialized = true;
}

void CruiseControl::update(float battery_amps)
{
    if (!initialized
        || !AP_Notify::flags.armed
        || !AP_Notify::flags.flying)
    {
        disengage();
        return;
    }

    if (state == CC_STATE_DISENGAGED)
    {
        // Allow engage condition: cruise ch value is minimum and pitch/roll are centered
        if (channel_cruise->norm_input_dz() < -0.90f && channel_pitch->in_trim_dz() && channel_roll->in_trim_dz())
        {
            state = CC_STATE_READY_TO_ENGAGE;
        }
    }
    else
    {
        // keep cruise while pitch/roll not moving
        if (channel_pitch->in_trim_dz() && channel_roll->in_trim_dz())
        {
            float cruise_ratio = (channel_cruise->norm_input() + 1.0f) / 2.0f;

            switch (state)
            {
                case CC_STATE_READY_TO_ENGAGE:
                    if (cruise_ratio > 0.1f)
                        state = CC_STATE_ENGAGED;
                    break;

                case CC_STATE_ENGAGED:
                    update_pitch(cruise_ratio, battery_amps);
                    break;
            }
        }
        else
        {
            disengage();
        }
    }
}

void CruiseControl::update_pitch(float cruise_ratio, float battery_amps)
{
    if (cruise_amps > 0.0f && battery_amps > 0.0f)
    {
        float fade = constrain_float(abs(cruise_amps - battery_amps) / 3.0f, 0.0f, 1.0f);

        if (battery_amps > cruise_amps)
            cruise_static_ratio -=  cruise_jerk / (1000.0f * fade);
        else
            cruise_static_ratio +=  cruise_jerk / (1000.0f * fade);
    }
    else
        cruise_static_ratio = 1.0f;
    
    cruise_static_ratio = constrain_float(cruise_static_ratio, 0.2f, 1.0f);
    uint16_t range = channel_pitch->get_radio_trim() - channel_pitch->get_radio_min();

    if (range < 0.0f)
        return;

    float pitchx = (float)range * cruise_ratio * cruise_static_ratio;
    channel_pitch->set_radio_in(channel_pitch->get_radio_trim() - (uint16_t)pitchx);
    channel_pitch->recompute_pwm_no_deadzone();
}
