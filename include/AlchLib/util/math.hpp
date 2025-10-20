#pragma once

#include <vector>

#include "AlchLib/chassis/chassis.hpp"

namespace alchlib {

namespace Math {
    constexpr double PI = 3.1415926535897932384626433832795028841971693993751058209;
    constexpr double PI_HALF = PI / 2;
    constexpr double TWO_PI = 2 * PI;

    double degToRad(double degrees);

    double radToDeg(double radians);

    double normalize180(double angle, bool radians = false);

    double normalize360(double angle, bool radians = false);

    double angleError(float angle1, float angle2, bool radians = false);

    constexpr float sanitizeAngle(float angle, bool radians);

    float lemlib_angle_error(float target, float position, bool radians = true, AngularDirection direction = AngularDirection::AUTO);

    template <typename T>
    int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    float slew(float target, float current, float maxChange);

    double interpolate(const std::vector<double>& keys, const std::vector<double>& values, double queryKey);
}
}