#include "carla_shenlan_pid_controller/pid_controller.h"

#include <assert.h>
#include <iostream>
namespace shenlan {
namespace control {

PIDController::PIDController(const double kp, const double ki, const double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
}

// /**to-do**/ 实现PID控制
double PIDController::Control(const double error, const double dt) {
    assert(dt > 0 && "dt must be positive!!!");

    proportional_part = kp_ * error;
    integral_ += error * dt;
    integral_part = ki_ * integral_;

    derivative_part = 0.0;
    if (!first_hit_) {
        derivative_part = kd_ * (error - previous_error_) / dt;
    }

    current_output = proportional_part + integral_part + derivative_part;
    previous_error_ = error;
    previous_output_ = current_output;
    first_hit_ = false;
    return current_output;
}

// /**to-do**/ 重置PID参数
void PIDController::Reset() {
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    proportional_part = 0.0;
    integral_part = 0.0;
    derivative_part = 0.0;
    current_output = 0.0;
    first_hit_ = true;
}

}    // namespace control
}    // namespace shenlan
