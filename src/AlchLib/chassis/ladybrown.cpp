#include "AlchLib/chassis/ladybrown.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <memory>
#include <cmath>

alchlib::LadyBrown::LadyBrown(int motor_port, int rotation_port) {
    this->arm_motor = std::make_shared<pros::Motor>(motor_port, pros::MotorGears::red);
    this->arm_motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    this->rotation_sensor = std::make_shared<pros::Rotation>(rotation_port);
    this->rotation_sensor->reset_position();
}

void alchlib::LadyBrown::nextState() {
    this->curr_state++;
    if (this->curr_state == this->states.size()) {
        this->curr_state = 0;
    }
    this->target = this->states.at(curr_state);
}

void alchlib::LadyBrown::setState(int state_index) {
    this->target = this->states.at(state_index);
}

void alchlib::LadyBrown::init() {
    pros::Task liftControlTask([=, this] {
        double kP = 0;
        double kD = 0;
        double prev_error = 0;
        bool settling = false;
        while (true) {
            double error = this->target - this->rotation_sensor->get_position();
            double derivative = error - prev_error;

            if (curr_state == 2) {
                kP = 0.03;
                kD = 0.072;
                printf("%f\n", fabs(error));
                settling = fabs(error) < 800;
            }
            else {
                kP = 0.019;
                kD = 0.075;
            }

            if (!settling) {
                this->arm_motor->move(kP * error + kD * derivative);
            }
            else {
                this->arm_motor->brake();
            }

            prev_error = error;
            settling = false;
            pros::delay(10);
        }
    });
}

void alchlib::LadyBrown::moveAt(int volts) {
    this->arm_motor->move(volts);
}