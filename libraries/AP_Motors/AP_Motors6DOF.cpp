/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Motors6DOF.cpp - ArduSub motors library
 */

#include "AP_Motors6DOF.h"
#include "AP_Motors.h"
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// parameters for the motor class
const AP_Param::GroupInfo AP_Motors6DOF::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),
    // @Param: 1_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("1_DIRECTION", 1, AP_Motors6DOF, _motor_reverse[0], 1),

    // @Param: 2_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("2_DIRECTION", 2, AP_Motors6DOF, _motor_reverse[1], 1),

    // @Param: 3_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("3_DIRECTION", 3, AP_Motors6DOF, _motor_reverse[2], 1),

    // @Param: 4_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("4_DIRECTION", 4, AP_Motors6DOF, _motor_reverse[3], 1),

    // @Param: 5_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("5_DIRECTION", 5, AP_Motors6DOF, _motor_reverse[4], 1),

    // @Param: 6_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("6_DIRECTION", 6, AP_Motors6DOF, _motor_reverse[5], 1),

    // @Param: 7_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("7_DIRECTION", 7, AP_Motors6DOF, _motor_reverse[6], 1),

    // @Param: 8_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("8_DIRECTION", 8, AP_Motors6DOF, _motor_reverse[7], 1),

    // @Param: FV_CPLNG_K
    // @DisplayName: Forward/vertical to pitch decoupling factor
    // @Description: Used to decouple pitch from forward/vertical motion. 0 to disable, 1.2 normal
    // @Range: 0.0 1.5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("FV_CPLNG_K", 9, AP_Motors6DOF, _forwardVerticalCouplingFactor, 1.0),

    // @Param: 9_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("9_DIRECTION", 10, AP_Motors6DOF, _motor_reverse[8], 1),

    // @Param: 10_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("10_DIRECTION", 11, AP_Motors6DOF, _motor_reverse[9], 1),

    // @Param: 11_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("11_DIRECTION", 12, AP_Motors6DOF, _motor_reverse[10], 1),

    // @Param: 12_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("12_DIRECTION", 13, AP_Motors6DOF, _motor_reverse[11], 1),

    // @Param: WR_GAIN_0
    // @DisplayName: Wrench gain
    // @Description: Scale input
    // @Range: -100.0 100.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("WR_GAIN_0", 14, AP_Motors6DOF, _wrench_gains[0], 1.13f),

    // @Param: WR_GAIN_1
    // @DisplayName: Wrench gain
    // @Description: Scale input
    // @Range: -100.0 100.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("WR_GAIN_1", 16, AP_Motors6DOF, _wrench_gains[1], 1.251f),

    // @Param: WR_GAIN_2
    // @DisplayName: Wrench gain
    // @Description: Scale input
    // @Range: -100.0 100.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("WR_GAIN_2", 17, AP_Motors6DOF, _wrench_gains[2], 0.433f),

    // @Param: WR_GAIN_3
    // @DisplayName: Wrench gain
    // @Description: Scale input
    // @Range: -100.0 100.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("WR_GAIN_3", 18, AP_Motors6DOF, _wrench_gains[3], 5.683),

    // @Param: WR_GAIN_4
    // @DisplayName: Wrench gain
    // @Description: Scale input
    // @Range: -100.0 100.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("WR_GAIN_4", 19, AP_Motors6DOF, _wrench_gains[4], 5.449f),

    // @Param: WR_GAIN_5
    // @DisplayName: Wrench gain
    // @Description: Scale input
    // @Range: -100.0 100.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("WR_GAIN_5", 20, AP_Motors6DOF, _wrench_gains[5], 1.425f),

    // @Param: MAX_THR
    // @DisplayName: Max thrust
    // @Description: Maximum thrust of thruster in N
    // @Range: 1.0 60.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("MAX_THR", 21, AP_Motors6DOF, _maximum_thrust, 60.0f),

    // @Param: PWM_DELAY_MS
    // @DisplayName: PWM delay ms
    // @Description: Delay the pwm output to the motors by ms
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PWM_DELAY_MS", 22, AP_Motors6DOF, _pwm_delay_ms, 0),

    AP_GROUPEND
};

void AP_Motors6DOF::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remove existing motors
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }

    // hard coded config for supported frames
    switch ((sub_frame_t)frame_class) {
        //                 Motor #              Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor  Testing Order
    case SUB_FRAME_BLUEROV1:
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              -1.0f,          0,                  1.0f,               0,              1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     -0.5f,          0.5f,           0,              0.45f,              0,                  0,              3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     0.5f,           0.5f,           0,              0.45f,              0,                  0,              4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     0,              -1.0f,          0,              1.0f,               0,                  0,              5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6,     -0.25f,         0,              0,              0,                  0,                  1.0f,           6);
        break;

    case SUB_FRAME_VECTORED_6DOF_90DEG:
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     1.0f,           1.0f,           0,              1.0f,               0,                  0,              1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     1.0f,           -1.0f,          0,              1.0f,               0,                  0,              3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     0,              0,              0,              0,                  0,                  1.0f,           4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     0,              0,              0,              0,                  0,                  1.0f,           5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6,     -1.0f,          1.0f,           0,              1.0f,               0,                  0,              6);
        add_motor_raw_6dof(AP_MOTORS_MOT_7,     0,              0,              -1.0f,          0,                  1.0f,               0,              7);
        add_motor_raw_6dof(AP_MOTORS_MOT_8,     -1.0f,          -1.0f,          0,              1.0f,               0,                  0,              8);
        break;

    case SUB_FRAME_VECTORED_6DOF:
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              1.0f,           0,                  -1.0f,              1.0f,           1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              -1.0f,          0,                  -1.0f,              -1.0f,          2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     0,              0,              -1.0f,          0,                  1.0f,               1.0f,           3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     0,              0,              1.0f,           0,                  1.0f,               -1.0f,          4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     1.0f,           -1.0f,          0,              -1.0f,              0,                  0,              5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6,     -1.0f,          -1.0f,          0,              -1.0f,              0,                  0,              6);
        add_motor_raw_6dof(AP_MOTORS_MOT_7,     1.0f,           1.0f,           0,              -1.0f,              0,                  0,              7);
        add_motor_raw_6dof(AP_MOTORS_MOT_8,     -1.0f,          1.0f,           0,              -1.0f,              0,                  0,              8);
        break;

    case SUB_FRAME_VECTORED:
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              1.0f,           0,                  -1.0f,              1.0f,           1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              -1.0f,          0,                  -1.0f,              -1.0f,          2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     0,              0,              -1.0f,          0,                  1.0f,               1.0f,           3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     0,              0,              1.0f,           0,                  1.0f,               -1.0f,          4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     1.0f,           0,              0,              -1.0f,              0,                  0,              5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6,     -1.0f,          0,              0,              -1.0f,              0,                  0,              6);
        break;

    case SUB_FRAME_CUSTOM:
        // Those motors are just to add them to the list, numbers are useless.
        // The actual inverse thrusters mapping lives is AP_Motors6DOF::output_armed_stabilizing_custom() function
        // Motor #                              Roll(rx)        Pitch(ry)       Yaw(rz)         Throttle(fz)    Forward(fx)         Lateral(fy)         Testing Order
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     +0.637f,        +0.730f,        -0.248f,        -0.178f,        -0.710f,            +0.681f,            1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     +0.661f,        -0.732f,        +0.162f,        -0.178f,        +0.710f,            +0.681f,            2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     -0.661f,        +0.732f,        +0.162f,        -0.178f,        -0.710f,            -0.681f,            3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     -0.637f,        -0.730f,        -0.248f,        -0.178f,        +0.710f,            -0.681f,            4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     -0.637f,        -0.730f,        -0.248f,        +0.178f,        -0.710f,            +0.681f,            5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6,     -0.661f,        +0.732f,        +0.162f,        +0.178f,        +0.710f,            +0.681f,            6);
        add_motor_raw_6dof(AP_MOTORS_MOT_7,     +0.661f,        -0.732f,        +0.162f,        +0.178f,        -0.710f,            -0.681f,            7);
        add_motor_raw_6dof(AP_MOTORS_MOT_8,     +0.637f,        +0.730f,        -0.248f,        +0.178f,        +0.710f,            -0.681f,            8);
        break;

    case SUB_FRAME_SIMPLEROV_3:
    case SUB_FRAME_SIMPLEROV_4:
    case SUB_FRAME_SIMPLEROV_5:
    default:
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              -1.0f,          0,                  1.0f,               0,              1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     0,              0,              0,              -1.0f,              0,                  0,              3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     0,              0,              0,              -1.0f,              0,                  0,              4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     0,              0,              0,              0,                  0,                  1.0f,           5);
        break;
    }
}

void AP_Motors6DOF::add_motor_raw_6dof(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float throttle_fac, float forward_fac, float lat_fac, uint8_t testing_order)
{
    //Parent takes care of enabling output and setting up masks
    add_motor_raw(motor_num, roll_fac, pitch_fac, yaw_fac, testing_order);

    //These are additional parameters for an ROV
    _throttle_factor[motor_num] = throttle_fac;
    _forward_factor[motor_num] = forward_fac;
    _lateral_factor[motor_num] = lat_fac;
}

// output_min - sends minimum values out to the motors
void AP_Motors6DOF::output_min()
{
    int8_t i;

    // set limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // fill the motor_out[] array for HIL use and send minimum value to each motor
    // ToDo find a field to store the minimum pwm instead of hard coding 1500
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, 1500);
        }
    }
}

int16_t AP_Motors6DOF::calc_thrust_to_pwm(float thrust_in) const
{

  int16_t pwm = 0U;
  int16_t mid_pwm = 1500U;

  float x = fabsf(thrust_in * _maximum_thrust);

  if (0.0 <= x && x <= 0.01) {  // motor dead zone
    return mid_pwm;
  }

  if (thrust_in > 0.0) {
    pwm = static_cast<int16_t>(safe_sqrt(x*x*19.328913748011416 + x*946.8280079018037 + -1.1617525031661196));
  } else {
    pwm = -1 * static_cast<int16_t>(safe_sqrt(x*x*20.56590334031135 + x*1262.4149340710521 + 110.90447379277136));
  }

  return constrain_int16(pwm + mid_pwm, _throttle_radio_min /*1100*/, _throttle_radio_max /*1900*/);
//  return constrain_int16(1500 + thrust_in * 400, _throttle_radio_min, _throttle_radio_max);
}

void AP_Motors6DOF::output_to_motors()
{
    int8_t i;
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final pwm values sent to the motor

    // Initialize the delay queues if delay > 0
    if (!delay_applied)
    {

        if (_pwm_delay_ms > 0)
        {
            delay_pwm = true;
            
            // Update rate of motor function is 400Hz -> 2.5ms
            int queue_sz = static_cast<int>(static_cast<float>(_pwm_delay_ms) / 2.5);

            // Initialize the thruster queues
            for (int i = 0 ; i < AP_MOTORS_MAX_NUM_MOTORS ; i++)
            {
                std::queue<int> tmp;
                for (int j = 0; j < queue_sz ; j++)
                    tmp.push(1500);
                thruster_queues.push_back(tmp);
            }

        }
        else
        {
            delay_pwm = false;
        }

        delay_applied = true;
    }

    switch (_spool_mode) {
    case SHUT_DOWN:
        // sends minimum values out to the motors
        // set motor output based on thrust requests
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = 1500;
            }
        }
        break;
    case GROUND_IDLE:
        // sends output to motors when armed but not flying
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = 1500;
            }
        }
        break;
    case SPOOL_UP:
    case THROTTLE_UNLIMITED:
    case SPOOL_DOWN:
        // set motor output based on thrust requests
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                if (is_raw()) {
                    motor_out[i] = raw_command.pwm[i];
                } else {
                    motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i]);
                    if (motor_out[i] > 999U) {
                        raw_command.pwm[i] = motor_out[i];
                    } else {
                      raw_command.pwm[i] = 1500U;
                    }
                }
            }
        }
        break;
    }
//
//    if (m_debug_counter > 10) {
    //  gcs().send_text(MAV_SEVERITY_DEBUG, "%u|%u|%u|%u|%u",
    //                  motor_out[0], thruster_queues[0].front(), motor_out[1], thruster_queues[1].front(), delay_pwm);
    //      m_debug_counter = 0;
//    } else {
//      m_debug_counter += 1;
//    }

    // send output to each motor
    if (delay_pwm)
    {
       for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++)
       {
            if (motor_enabled[i]) 
            {
                rc_write(i, thruster_queues[i].front());
                thruster_queues[i].pop();
                thruster_queues[i].push(motor_out[i]);
            }
       } 
    }
    else
    {
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++)
        {
            if (motor_enabled[i]) 
                rc_write(i, motor_out[i]);
        } 
    }
}

float AP_Motors6DOF::get_current_limit_max_throttle()
{
    return 1.0f;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
// ToDo calculate headroom for rpy to be added for stabilization during full throttle/forward/lateral commands
void AP_Motors6DOF::output_armed_stabilizing()
{
    if ((sub_frame_t)_last_frame_class == SUB_FRAME_VECTORED) {
        output_armed_stabilizing_vectored();
    } else if ((sub_frame_t)_last_frame_class == SUB_FRAME_VECTORED_6DOF) {
        output_armed_stabilizing_vectored_6dof();
    } else if ((sub_frame_t)_last_frame_class == SUB_FRAME_CUSTOM) {
        output_armed_stabilizing_custom();
    } else {
        uint8_t i;                          // general purpose counter
        float   roll_thrust;                // roll thrust input value, +/- 1.0
        float   pitch_thrust;               // pitch thrust input value, +/- 1.0
        float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
        float   throttle_thrust;            // throttle thrust input value, +/- 1.0
        float   forward_thrust;             // forward thrust input value, +/- 1.0
        float   lateral_thrust;             // lateral thrust input value, +/- 1.0

        roll_thrust = _roll_in;
        pitch_thrust = _pitch_in;
        yaw_thrust = _yaw_in;
        throttle_thrust = get_throttle_bidirectional();
        forward_thrust = _forward_in;
        lateral_thrust = _lateral_in;

        float rpy_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
        float linear_out[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor

        // initialize limits flags
        limit.roll_pitch = false;
        limit.yaw = false;
        limit.throttle_lower = false;
        limit.throttle_upper = false;

        // sanity check throttle is above zero and below current limited throttle
        if (throttle_thrust <= -_throttle_thrust_max) {
            throttle_thrust = -_throttle_thrust_max;
            limit.throttle_lower = true;
        }
        if (throttle_thrust >= _throttle_thrust_max) {
            throttle_thrust = _throttle_thrust_max;
            limit.throttle_upper = true;
        }

        // calculate roll, pitch and yaw for each motor
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                rpy_out[i] = roll_thrust * _roll_factor[i] +
                             pitch_thrust * _pitch_factor[i] +
                             yaw_thrust * _yaw_factor[i];

            }
        }

        // calculate linear command for each motor
        // linear factors should be 0.0 or 1.0 for now
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                linear_out[i] = throttle_thrust * _throttle_factor[i] +
                                forward_thrust * _forward_factor[i] +
                                lateral_thrust * _lateral_factor[i];
            }
        }

        // Calculate final output for each motor
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                _thrust_rpyt_out[i] = constrain_float(_motor_reverse[i]*(rpy_out[i] + linear_out[i]),-1.0f,1.0f);
            }
        }
    }

    const AP_BattMonitor &battery = AP::battery();

	// Current limiting
    if (_batt_current_max <= 0.0f || !battery.has_current()) {
        return;
    }

    float _batt_current = battery.current_amps();

    float _batt_current_delta = _batt_current - _batt_current_last;

    float loop_interval = 1.0f/_loop_rate;

    float _current_change_rate = _batt_current_delta / loop_interval;

    float predicted_current = _batt_current + (_current_change_rate * loop_interval * 5);

    float batt_current_ratio = _batt_current/_batt_current_max;

    float predicted_current_ratio = predicted_current/_batt_current_max;
    _batt_current_last = _batt_current;

    if (predicted_current > _batt_current_max * 1.5f) {
        batt_current_ratio = 2.5f;
    } else if (_batt_current < _batt_current_max && predicted_current > _batt_current_max) {
        batt_current_ratio = predicted_current_ratio;
    }
    _output_limited += (loop_interval/(loop_interval+_batt_current_time_constant)) * (1 - batt_current_ratio);

    _output_limited = constrain_float(_output_limited, 0.0f, 1.0f);

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] *= _output_limited;
        }
    }
}

void AP_Motors6DOF::output_armed_stabilizing_custom() {

  uint8_t i;                          // general purpose counter
  float   roll_thrust;                // roll thrust input value, +/- 1.0
  float   pitch_thrust;               // pitch thrust input value, +/- 1.0
  float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
  float   throttle_thrust;            // throttle thrust input value, +/- 1.0
  float   forward_thrust;             // forward thrust input value, +/- 1.0
  float   lateral_thrust;             // lateral thrust input value, +/- 1.0

  const uint8_t conf_rows = AP_MOTORS_MAX_NUM_MOTORS;
  const uint8_t conf_cols = 6;

  float thrusters_forces[conf_rows] = {0.0f};
  float thrusters_forces_abs_max = 0.0f;
  float thrusters_forces_scaler = 1.0f;

  const float conf_inv[conf_rows][conf_cols] =
      {
          {-0.88536058, -0.79975776,  2.31465619,  0.17598475, -0.18355167, -0.7018588 },
          {-0.57627255,  0.50341193, -2.31465619, -0.17598475, -0.18355167, -0.7018588 },
          { 0.57627255, -0.50341193, -2.31465619,  0.17598475,  0.18355167, -0.7018588 },
          { 0.88536058,  0.79975776,  2.31465619, -0.17598475,  0.18355167, -0.7018588 },
          { 0.88536058,  0.79975776,  2.31465619,  0.17598475, -0.18355167,  0.7018588 },
          { 0.57627255, -0.50341193, -2.31465619, -0.17598475, -0.18355167,  0.7018588 },
          {-0.57627255,  0.50341193, -2.31465619,  0.17598475,  0.18355167,  0.7018588 },
          {-0.88536058, -0.79975776,  2.31465619, -0.17598475,  0.18355167,  0.7018588 }
      };

  roll_thrust = _roll_in * _wrench_gains[0];
  pitch_thrust = _pitch_in * _wrench_gains[1];
  yaw_thrust = _yaw_in * _wrench_gains[2];
  forward_thrust = _forward_in * _wrench_gains[3];
  lateral_thrust = _lateral_in * _wrench_gains[4];
  throttle_thrust = -1.0f * _wrench_gains[5] * get_throttle_bidirectional();

  // Joystick mode 3 , manual mode
//  roll_thrust = 0 Ok
//  pitch_thrust = 0 Ok
//  yaw_thrust right stick right+ left- Ok
//  forward_thrust left stick up+ down- Ok
//  lateral_thrust left stick right+ left- Ok
//  throttle_thrust right stick up+ down- NOT Ok, was reverted above.

  float desired_wrench[conf_cols] = {roll_thrust, pitch_thrust, yaw_thrust, forward_thrust, lateral_thrust, throttle_thrust};

  for (int row = 0; row < conf_rows; row++) {
    for (int col = 0; col < conf_cols; col++) {
      thrusters_forces[row] += conf_inv[row][col] * desired_wrench[col];
    }
  }

  // initialize limits flags
  limit.roll_pitch = false;
  limit.yaw = false;
  limit.throttle_lower = false;
  limit.throttle_upper = false;
  limit.throttle_upper = false;
  limit.throttle_lower = false;

//  if (m_debug_counter > 100) {
//    gcs().send_text(MAV_SEVERITY_DEBUG, "%.2f|%.2f|%.2f|%.2f|%.2f|%.2f",
//                    roll_thrust, pitch_thrust, yaw_thrust, forward_thrust, lateral_thrust, throttle_thrust);
//    gcs().send_text(MAV_SEVERITY_DEBUG, "%+.2f|%+.2f|%+.2f|%+.2f|%+.2f|%+.2f|%+.2f|%+.2f",
//                    thrusters_forces[0], thrusters_forces[1], thrusters_forces[2], thrusters_forces[3],
//                    thrusters_forces[4], thrusters_forces[5], thrusters_forces[6], thrusters_forces[7]);
//    m_debug_counter = 0;
//  } else {
//    m_debug_counter += 1;
//  }

  for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
    float abs_force = fabsf(thrusters_forces[i]);
    if (abs_force > thrusters_forces_abs_max) {
      thrusters_forces_abs_max = abs_force;
    }
  }

  if (thrusters_forces_abs_max > 1.0f) {
    thrusters_forces_scaler = 1.0f/thrusters_forces_abs_max;
  }

  for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
    if (motor_enabled[i]) {
      _thrust_rpyt_out[i] = constrain_float(static_cast<float>(_motor_reverse[i])*thrusters_forces[i]*thrusters_forces_scaler, -1.0f, 1.0f);  // TODO, OLSLO, do we need constraint here?
//      _thrust_rpyt_out[i] = 0.0f;
    }
  }

}

// output_armed - sends commands to the motors
// includes new scaling stability patch
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
// ToDo calculate headroom for rpy to be added for stabilization during full throttle/forward/lateral commands
void AP_Motors6DOF::output_armed_stabilizing_vectored()
{
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, +/- 1.0
    float   forward_thrust;             // forward thrust input value, +/- 1.0
    float   lateral_thrust;             // lateral thrust input value, +/- 1.0

    roll_thrust = _roll_in;
    pitch_thrust = _pitch_in;
    yaw_thrust = _yaw_in;
    throttle_thrust = get_throttle_bidirectional();
    forward_thrust = _forward_in;
    lateral_thrust = _lateral_in;

    float rpy_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
    float linear_out[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor

    // initialize limits flags
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= -_throttle_thrust_max) {
        throttle_thrust = -_throttle_thrust_max;
        limit.throttle_lower = true;
    }

    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // calculate roll, pitch and yaw for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rpy_out[i] = roll_thrust * _roll_factor[i] +
                         pitch_thrust * _pitch_factor[i] +
                         yaw_thrust * _yaw_factor[i];
        }
    }

    float forward_coupling_limit = 1-_forwardVerticalCouplingFactor*float(fabsf(throttle_thrust));
    if (forward_coupling_limit < 0) {
        forward_coupling_limit = 0;
    }
    int8_t forward_coupling_direction[] = {-1,-1,1,1,0,0,0,0,0,0,0,0};

    // calculate linear command for each motor
    // linear factors should be 0.0 or 1.0 for now
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {

            float forward_thrust_limited = forward_thrust;

            // The following statements decouple forward/vertical hydrodynamic coupling on
            // vectored ROVs. This is done by limiting the maximum output of the "rear" vectored
            // thruster (where "rear" depends on direction of travel).
            if (!is_zero(forward_thrust_limited)) {
                if ((forward_thrust < 0) == (forward_coupling_direction[i] < 0) && forward_coupling_direction[i] != 0) {
                    forward_thrust_limited = constrain_float(forward_thrust, -forward_coupling_limit, forward_coupling_limit);
                }
            }

            linear_out[i] = throttle_thrust * _throttle_factor[i] +
                            forward_thrust_limited * _forward_factor[i] +
                            lateral_thrust * _lateral_factor[i];
        }
    }

    // Calculate final output for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_motor_reverse[i]*(rpy_out[i] + linear_out[i]), -1.0f, 1.0f);
        }
    }
}

// Band Aid fix for motor normalization issues.
// TODO: find a global solution for managing saturation that works for all vehicles
void AP_Motors6DOF::output_armed_stabilizing_vectored_6dof()
{
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, +/- 1.0
    float   forward_thrust;             // forward thrust input value, +/- 1.0
    float   lateral_thrust;             // lateral thrust input value, +/- 1.0

    roll_thrust = _roll_in;
    pitch_thrust = _pitch_in;
    yaw_thrust = _yaw_in;
    throttle_thrust = get_throttle_bidirectional();
    forward_thrust = _forward_in;
    lateral_thrust = _lateral_in;

    float rpt_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
    float yfl_out[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor
    float rpt_max;
    float yfl_max;

    // initialize limits flags
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= -_throttle_thrust_max) {
        throttle_thrust = -_throttle_thrust_max;
        limit.throttle_lower = true;
    }

    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // calculate roll, pitch and Throttle for each motor (only used by vertical thrusters)
    rpt_max = 1; //Initialized to 1 so that normalization will only occur if value is saturated
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rpt_out[i] = roll_thrust * _roll_factor[i] +
                         pitch_thrust * _pitch_factor[i] +
                         throttle_thrust * _throttle_factor[i];
            if (fabsf(rpt_out[i]) > rpt_max) {
                rpt_max = fabsf(rpt_out[i]);
            }
        }
    }

    // calculate linear/yaw command for each motor (only used for translational thrusters)
    // linear factors should be 0.0 or 1.0 for now
    yfl_max = 1; //Initialized to 1 so that normalization will only occur if value is saturated
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            yfl_out[i] = yaw_thrust * _yaw_factor[i] +
                         forward_thrust * _forward_factor[i] +
                         lateral_thrust * _lateral_factor[i];
            if (fabsf(yfl_out[i]) > yfl_max) {
                yfl_max = fabsf(yfl_out[i]);
            }
        }
    }

    // Calculate final output for each motor and normalize if necessary
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_motor_reverse[i]*(rpt_out[i]/rpt_max + yfl_out[i]/yfl_max),-1.0f,1.0f);
        }
    }
}

void AP_Motors6DOF::set_raw_command(uint8_t chan, uint16_t pwm, uint32_t timestamp)
{
    raw_command.pwm[chan] = pwm;
    raw_command.last_message_ms = timestamp;
}

uint16_t AP_Motors6DOF::get_raw_command(uint8_t chan)
{
    return raw_command.pwm[chan];
}
