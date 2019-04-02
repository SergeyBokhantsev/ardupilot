#include "AP_RangeFinder_TfMiniI2C.h"

AP_RangeFinder_TfMiniI2C::AP_RangeFinder_TfMiniI2C(RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_RangeFinder_Backend(_state)
    , _dev(std::move(dev)) {}

AP_RangeFinder_Backend *AP_RangeFinder_TfMiniI2C::detect(RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_TfMiniI2C *sensor
        = new AP_RangeFinder_TfMiniI2C(_state, std::move(dev));

    if (!sensor) {
        delete sensor;
        return nullptr;
    }

   /* if (sensor->_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        uint16_t reading_cm;
        uint8_t strength, mode;
        if (!sensor->get_reading(reading_cm, strength, mode)) {
            sensor->_dev->get_semaphore()->give();
            delete sensor;
            return nullptr;
        }
        sensor->_dev->get_semaphore()->give();
    } */

    sensor->init();

    return sensor;
}

void AP_RangeFinder_TfMiniI2C::init()
{
    // call timer() at 50Hz
    //_dev->register_periodic_callback(20000,
    //      FUNCTOR_BIND_MEMBER(&AP_RangeFinder_TfMiniI2C::timer, void));
   
   // if (state.offset > 0.0f){
    //    _dev->set_split_transfers(true);
   // }
    
    // call timer() at 10Hz
    _dev->register_periodic_callback(100000,
         FUNCTOR_BIND_MEMBER(&AP_RangeFinder_TfMiniI2C::timer, void));
     
     //state.i2c_result = 0;
}

void AP_RangeFinder_TfMiniI2C::update()
{
    //state.i2c_result++;    
}

bool AP_RangeFinder_TfMiniI2C::get_reading(uint16_t &reading_cm, uint8_t &strength, uint8_t &mode)
{
    read_request[0] = 0xFF;
    read_request[1] = 2;
    read_request[2] = 7;
    
    bool read_ok = _dev->transfer(read_request, 3, read_response, 7);
    
   // state.i2c_result = read_response[0];
    
    //_dev->transfer(read_request, 3, nullptr, 0);
    //bool read_ok = _dev->transfer(nullptr, 0, read_response, 7);
    
    if (read_ok){
        reading_cm = (uint16_t)read_response[2]|((uint16_t)read_response[3]) << 8;
        strength = 10;
        mode = 4;
    }
    
    return read_ok;
}

void AP_RangeFinder_TfMiniI2C::timer()
{
    if (get_reading(state.distance_cm, state.strength, state.mode)) {
        //update_status();
        state.status = RangeFinder::RangeFinder_Good;
    }
    else {
        state.status = RangeFinder::RangeFinder_NoData;
    }
}