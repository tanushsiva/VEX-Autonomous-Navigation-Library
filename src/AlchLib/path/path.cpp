#include <cmath>

#include "AlchLib/path/path.hpp"

alchlib::Path::Path(const std::vector<PathSegment>& segments, double start_vel, double end_vel) {
	this->segments = segments;
	this->start_vel = start_vel;
	this->end_vel = end_vel;
}

double alchlib::Path::getLength() const {
	double length = 0;

	for (PathSegment segment : this->segments) {
		length += segment.bezier.getLength();
	}

	return length;
}

int alchlib::Path::getNumSegments() const {
	return this->segments.size();
}

alchlib::PathSegment alchlib::Path::getSegment(int index) const {
	return this->segments.at(index);
}

alchlib::CubicBezier alchlib::Path::getBezier(int index) const {
	return this->segments.at(index).bezier;
}

alchlib::CubicBezier alchlib::Path::getBezierFromT(double t_global) const {
	return this->getBezier(std::min(static_cast<int>(std::floor(t_global)), static_cast<int>(this->segments.size() - 1)));
}

double alchlib::Path::getLocalT(double t_global) const {
	if (std::floor(t_global) == this->getNumSegments()) {
		return 1.0;
	}
	return std::fmod(t_global, 1.0);
}