#pragma once

#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <memory>

namespace alchlib {
class LadyBrown {
public:
    LadyBrown(int motor_port, int rotation_port);
    
    void nextState();

    void moveAt(int volts);

    void setState(int state_index);

    void init();
private:
    std::shared_ptr<pros::Motor> arm_motor;
    std::shared_ptr<pros::Rotation> rotation_sensor;
    std::vector<int> states = {0, 2900, 14500};
    int curr_state = 0;
    int target = states.at(0);
};
}