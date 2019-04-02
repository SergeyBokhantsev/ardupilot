#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_TfMiniI2C : public AP_RangeFinder_Backend
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void);

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // constructor
    AP_RangeFinder_TfMiniI2C(RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    void init();
    void timer();

    // get a reading
    bool get_reading(uint16_t &reading_cm, uint8_t &strength, uint8_t &mode);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    
    uint8_t read_response[7];
    uint8_t read_request[3] = { 1, 2, 7 };
};
