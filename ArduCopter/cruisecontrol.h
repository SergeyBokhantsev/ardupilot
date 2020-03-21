#pragma once

#include <RC_Channel/RC_Channel.h>

#define CC_STATE_DISENGAGED 0
#define CC_STATE_READY_TO_ENGAGE 1
#define CC_STATE_ENGAGED 2

class CruiseControl
{
public:
    CruiseControl();

    void init(RC_Channel* _channel_cruise, RC_Channel* _channel_pitch, RC_Channel* _channel_roll, float _cruise_amps, uint8_t _cruise_jerk);

    void update(float battery_current);
    
    void disengage() { state = CC_STATE_DISENGAGED; cruise_static_ratio = 0.0f; }
    
    uint8_t get_state() { return state; }

private:
    RC_Channel* channel_cruise;
    RC_Channel* channel_pitch;
    RC_Channel* channel_roll;
    float cruise_amps;
    float cruise_jerk;
    bool initialized;

    uint8_t state;
    float cruise_static_ratio;

    void update_pitch(float cruise_ratio, float battery_current);
};