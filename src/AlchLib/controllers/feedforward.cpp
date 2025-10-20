#include "Alchlib/controllers/feedforward.hpp"
#include "AlchLib/util/math.hpp"

alchlib::Feedforward::Feedforward(double kS, double kV, double kA) {
    this->kS = kS;
    this->kV = kV;
    this->kA = kA;
}

double alchlib::Feedforward::update(double vel) {
    float accel_term = vel - this->prevVel;
    this->prevVel = vel;

    return this->kS * Math::sgn(vel) + this->kV * vel + this->kA * accel_term;
}
    
void alchlib::Feedforward::reset() {
    this->prevVel = 0;
}