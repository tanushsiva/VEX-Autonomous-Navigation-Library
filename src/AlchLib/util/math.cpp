// Adapted from LemLib (https://github.com/LemLib/LemLib), MIT License.
// See LICENSE-LemLib.

#include <cmath>
#include <stdexcept>

#include "AlchLib/util/math.hpp"

double alchlib::Math::degToRad(double degrees) {
    return degrees * PI / 180;
}

double alchlib::Math::radToDeg(double radians) {
    return radians * 180 / PI;
}

double alchlib::Math::normalize180(double angle, bool radians) {
    float max = radians ? 2 * Math::PI : 360;
    float half = radians ? Math::PI : 180;

    if (angle > half) {
        return angle - max;
    }
    else if (angle < -half) {
        return angle + max;
    }
    return angle;
}

double alchlib::Math::normalize360(double angle, bool radians) {
    float max = radians ? 2 * PI : 360;

    if (angle > max) {
        return angle - max;
    }
    else if (angle < 0) {
        return angle + max;
    }
    return angle;
}

double alchlib::Math::angleError(float angle1, float angle2, bool radians) {
    float max = radians ? 2 * Math::PI : 360;
    angle1 = fmod(angle1, max);
    angle2 = fmod(angle2, max);
    return normalize180(angle1 - angle2, radians);
}

constexpr float alchlib::Math::sanitizeAngle(float angle, bool radians) {
    if (radians) return std::fmod(std::fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    else return std::fmod(std::fmod(angle, 360) + 360, 360);
}

float alchlib::Math::lemlib_angle_error(float target, float position, bool radians, AngularDirection direction) {
    target = sanitizeAngle(target, radians);
    target = sanitizeAngle(target, radians);
    const float max = radians ? 2 * M_PI : 360;
    const float rawError = target - position;
    switch (direction) {
        case AngularDirection::CW_CLOCKWISE: 
            return rawError < 0 ? rawError + max : rawError; 
        case AngularDirection::CCW_COUNTERCLOCKWISE: 
            return rawError > 0 ? rawError - max : rawError; 
        default: // choose the shortest path
            return std::remainder(rawError, max);
    }
}

float alchlib::Math::slew(float target, float current, float maxChange) {
    float change = target - current;
    if (maxChange == 0) return target;
    if (change > maxChange) change = maxChange;
    else if (change < -maxChange) change = -maxChange;
    return current + change;
}

double alchlib::Math::interpolate(const std::vector<double> &keys, const std::vector<double> &values, double queryKey) {
    if (keys.size() != values.size() || keys.empty()) {
        throw std::invalid_argument("Keys and Values must have the same non-zero size!");
    }

    if (queryKey <= keys.front()) {
        return values.front();
    }
    if (queryKey >= keys.back()) {
        return values.back();
    }

    auto it = std::lower_bound(keys.begin(), keys.end(), queryKey);
    int lower_index = std::distance(keys.begin(), it) - 1;

    double t = (queryKey - keys[lower_index]) / (keys[lower_index + 1] - keys[lower_index]);
    return values[lower_index] + t * (values[lower_index + 1] - values[lower_index]);
}