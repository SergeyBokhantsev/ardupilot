#include "Copter.h"
#include <cmath>

#if MODE_BRAKE_ENABLED == ENABLED

/*
 * Init and run calls for brake flight mode
 */

// brake_init - initialise brake controller
bool Copter::ModeBrake::init(bool ignore_checks)
{
    // set target to current position
    wp_nav->init_brake_target(BRAKE_MODE_DECEL_RATE);

    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z);
    pos_control->set_max_accel_z(BRAKE_MODE_DECEL_RATE);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    _timeout_ms = 0;

    _cancel_if_radio_link = false;
        
    return true;
}

// brake_run - runs the brake controller
// should be called at 100hz or more
void Copter::ModeBrake::run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        wp_nav->init_brake_target(BRAKE_MODE_DECEL_RATE);
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // relax stop target if we might be landed
    if (ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // run brake controller
    wp_nav->update_brake();

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), 0.0f);

    // update altitude target and call position controller
    // protects heli's from inflight motor interlock disable
    if (motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::GROUND_IDLE && !ap.land_complete) {
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
    } else {
        pos_control->set_alt_target_from_climb_rate_ff(0.0f, G_Dt, false);
    }
    pos_control->update_z_controller();

    if (_timeout_ms != 0 && millis()-_timeout_start >= _timeout_ms) {
        _timeout_ms = 0;
        if (!copter.set_mode(_timeout_mode, MODE_REASON_BRAKE_TIMEOUT)) {
            if (_timeout_mode_backup != _timeout_mode){
                copter.set_mode(_timeout_mode_backup, MODE_REASON_BRAKE_TIMEOUT);
            }
        }
    }
    
    if (_cancel_if_radio_link && !copter.failsafe.radio){
        copter.set_mode(_cancel_to_mode, MODE_REASON_SUPPRESS_BRAKE);
        _cancel_if_radio_link = false;
    }
}

void Copter::ModeBrake::timeout_to_mode_ms(uint32_t timeout_ms, control_mode_t mode, control_mode_t backup_mode)
{
    _timeout_start = millis();
    _timeout_ms = timeout_ms;
    _timeout_mode = mode;
    _timeout_mode_backup = backup_mode;
}

void Copter::ModeBrake::suppress_to_mode(control_mode_t mode)
{
    _cancel_if_radio_link = true;
    _cancel_to_mode = mode;
}

#endif

