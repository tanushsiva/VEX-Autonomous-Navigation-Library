#include <cmath>

#include "AlchLib/chassis/chassis.hpp"
#include "AlchLib/motionProfiling/motionProfile2D.hpp"
#include "config.hpp"

void alchlib::Chassis::followProfile(alchlib::MotionProfile2D profile, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([&]() { followProfile(profile, false); });
        this->endMotion();
        pros::delay(10); 
        return;
    }

    this->left_ff.reset();
    this->right_ff.reset();
    this->left_velocity_pid.reset();
    this->right_velocity_pid.reset();

    distTraveled = 0;
    double time_taken = 0;
    const double delta_time = 10 / 1000.0;
    const double duration = profile.getDuration();

    while (time_taken <= duration) {
        MotionCommand desired_cmd = profile.get(time_taken);

        Pose pose = this->getPose(true);
        auto [v, w] = ramsete.calculate(pose, desired_cmd);

        chassis.bezier_running = desired_cmd.bezier_index + 1;

        double v_left = v - w * 11.875 / 2;
        double v_right = v + w * 11.875 / 2;

        double left_voltage = this->left_ff.update(v_left) + this->left_velocity_pid.update(v_left - this->getLeftWheelVelocity());
        double right_voltage = this->right_ff.update(v_right) + this->right_velocity_pid.update(v_right - this->getRightWheelVelocity());

        double ratio = std::max(std::fabs(left_voltage), std::fabs(right_voltage)) / 127;
        if (ratio > 1) {
            left_voltage /= ratio;
            right_voltage /= ratio;
        }

        this->drivetrain.left_motors->move(left_voltage);
        this->drivetrain.right_motors->move(right_voltage);

        time_taken += (delta_time);
        
        pros::delay(10);
    }
    this->drivetrain.left_motors->move(0);
    this->drivetrain.right_motors->move(0);

    distTraveled = -1;
    this->endMotion();
}