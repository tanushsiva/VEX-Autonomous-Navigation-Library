#include <cmath>

#include "AlchLib/util/pose.hpp"

double alchlib::Pose::distanceTo(const Pose& other) const {
    double delta_x = this->x - other.x;
	double delta_y = this->y - other.y;
	return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}

double alchlib::Pose::angleTo(const Pose& other) const {
    return std::atan2(other.y - this->y, other.x - this->x);
}