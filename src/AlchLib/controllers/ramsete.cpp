#include <cmath>

#include "AlchLib/controllers/ramsete.hpp"
#include "AlchLib/util/math.hpp"

alchlib::RamseteController::RamseteController(double beta, double zeta) {
	this->beta = beta;
	this->zeta = zeta;
}

std::pair<double, double> alchlib::RamseteController::calculate(const Pose& actual, const MotionCommand& command) const {
    // Get the desired pose and command values, and convert the desired linear velocity to meters per second
    Pose desired_pose = command.pose;
    const double v_d = command.vel / 39.3701; // Convert desired velocity from inches to meters
    const double w_d = command.angularVel; // Desired angular velocity (in radians per second)

    // Calculate the error in position (in meters)
    const double delta_x = (desired_pose.x - actual.x) / 39.3701; // Difference in x-coordinate (in meters)
    const double delta_y = (desired_pose.y - actual.y) / 39.3701; // Difference in y-coordinate (in meters)

    // Calculate the error components in the robot's frame of reference (e_x, e_y)
    const double e_x = delta_x * std::cos(actual.theta) + delta_y * std::sin(actual.theta);
    const double e_y = delta_x * -std::sin(actual.theta) + delta_y * std::cos(actual.theta);

    // Calculate the angular error, making sure it's non-zero
    double e_theta = Math::angleError(desired_pose.theta, actual.theta, true);
    if (e_theta == 0) {
        e_theta = 0.0001; // Avoid division by zero
    }

    // Calculate the control gains based on the desired angular velocity (w_d), damping factor (zeta), and robot dynamics
    const double k = 2 * this->zeta * std::sqrt((w_d * w_d) + this->beta * (v_d * v_d));

    // Calculate the desired linear and angular velocities (v_cmd, w_cmd) using the Ramsete controller equations
    const double v_cmd = v_d * cos(e_theta) + k * e_x; // Linear velocity command
    const double w_cmd = w_d + k * e_theta + (this->beta * v_d * std::sin(e_theta) * e_y) / e_theta; // Angular velocity command

    // Return the computed velocities, converted back to inches per second for linear velocity
    return std::make_pair(v_cmd * 39.3701, w_cmd);
}
