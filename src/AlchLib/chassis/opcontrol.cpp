#include "AlchLib/chassis/chassis.hpp"
#include "AlchLib/util/math.hpp"

// Arcade control function for driving the robot with a single joystick.
// Takes in a throttle value for forward/backward movement, a turn value for rotation, 
// and a desaturation bias to balance power distribution.
void alchlib::Chassis::arcade(int throttle, int turn, float desaturateBias) {
    
    // Check if the sum of absolute throttle and turn values exceeds the motor power limit (127).
    if (std::abs(throttle) + std::abs(turn) > 127) {
        int old_throttle = throttle;
        int old_turn = turn;

        // Scale down throttle and turn to prevent exceeding the maximum motor output.
        // The desaturateBias determines how the scaling is distributed between throttle and turn.
        throttle *= (1 - desaturateBias * std::abs(old_turn / 127.0));
        turn *= (1 - (1 - desaturateBias) * std::abs(old_throttle / 127.0));

        // If after scaling, the combined power is exactly 126, adjust slightly to reach 127.
        if (std::abs(turn) + std::abs(throttle) == 126) {
            if (desaturateBias < 0.5) 
                throttle += Math::sgn(throttle); // Adjust throttle slightly
            else 
                turn += Math::sgn(turn); // Adjust turn slightly
        }
    }

    // Compute final motor power values for left and right sides.
    int left_power = throttle + turn;
    int right_power = throttle - turn;

    // Apply power to the drivetrain motors.
    drivetrain.left_motors->move(left_power);
    drivetrain.right_motors->move(right_power);
}
