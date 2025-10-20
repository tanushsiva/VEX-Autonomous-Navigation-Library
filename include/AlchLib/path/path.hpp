#pragma once

#include "AlchLib/path/cubicBezier.hpp"

namespace alchlib {

struct Constraints {
	double max_vel;

	double max_accel;

    double max_decel;
};

struct PathSegment {
	CubicBezier bezier;

	Constraints constraints;
};

class Path {
public:
	Path(const std::vector<PathSegment>& segments, double start_vel, double end_vel);

	double getLength() const; 

	int getNumSegments() const;

	PathSegment getSegment(int index) const;

	CubicBezier getBezier(int index) const;

	CubicBezier getBezierFromT(double t_global) const;

	double getLocalT(double t_global) const;
private:
	std::vector<PathSegment> segments;

	double start_vel;

	double end_vel;

	friend class MotionProfile2D;
};
}