#include "Sub.h"

// manual_init - initialise manual controller
bool Sub::manual_init() {
    // set target altitude to zero for reporting
    pos_control.set_pos_target_z_cm(0);

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    set_neutral_controls();

    sub.motors.disable_raw();

    return true;
}

// manual_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void Sub::manual_run() {
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    motors.set_roll(channel_roll->norm_input());
    motors.set_pitch(channel_pitch->norm_input());
    motors.set_yaw(channel_yaw->norm_input() * g.acro_yaw_p / ACRO_YAW_P);
    motors.set_throttle(channel_throttle->norm_input());
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}

// manual_init - initialise manual controller
bool Sub::raw_init() {
    // set target altitude to zero for reporting
    pos_control.set_alt_target_with_slew(0);

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    set_neutral_controls();

    // Set raw command neutral

    sub.motors.enable_raw();

    for (int i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        sub.motors.set_raw_command(i, 1500U, AP_HAL::millis());
    }

    return true;
}

// raw_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void Sub::raw_run() {
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0, true, g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        return;
    }

  motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

}
