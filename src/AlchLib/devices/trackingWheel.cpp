#include "AlchLib/devices/trackingWheel.hpp"
#include "AlchLib/util/math.hpp"

alchlib::TrackingWheel::TrackingWheel() {
    this->rotation_sensor = nullptr;
    this->wheel_diameter = 0;
    this->offset = 0;
}

alchlib::TrackingWheel::TrackingWheel(pros::Rotation* rotation_sensor, float wheel_diameter, float offset) {
    this->rotation_sensor = rotation_sensor;
    this->wheel_diameter = wheel_diameter;
    this->offset = offset;
}

void alchlib::TrackingWheel::reset() {
    this->rotation_sensor->reset_position();
}

float alchlib::TrackingWheel::getOffset() {
    return this->offset;
}

double alchlib::TrackingWheel::getDistanceTravelled() {
    return double(this->rotation_sensor->get_position()) * this->wheel_diameter * Math::PI / 36000;
}