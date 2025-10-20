#include "AlchLib/controllers/pid.hpp"

alchlib::PID::PID(double kP, double kI, double kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
}

double alchlib::PID::update(double error) {
    this->integral += error;
    const float derivative = error - this->prev_error;
    this->prev_error = error;

    return this->kP * error + this->kI * integral + this->kD * derivative;
}
    
void alchlib::PID::reset() {
    this->integral = 0;
    prev_error = 0;
}