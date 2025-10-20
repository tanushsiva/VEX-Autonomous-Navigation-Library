#pragma once

#include "pros/rotation.hpp" 

namespace alchlib {

class TrackingWheel {
public:
    TrackingWheel();
    TrackingWheel(pros::Rotation* rotation_sensor, float wheel_diameter, float offset);
    void reset();
    float getOffset();
    double getDistanceTravelled();

private:
    pros::Rotation* rotation_sensor;
    float wheel_diameter;
    float offset;
};
}