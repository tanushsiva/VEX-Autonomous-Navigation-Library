#pragma once

#include <vector>

#include "AlchLib/util/pose.hpp"
#include "AlchLib/path/path.hpp"

namespace alchlib {

struct MotionAttribute {
	double dist;
	double vel;
	double accel;
    double decel;
};

struct TimeToVelocity {	
	std::vector<double> time;

	std::vector<double> vel;

	std::vector<double> t;

	void push(double time, double vel, double t);
};

struct MotionCommand {
	double vel;

	double angularVel;

	Pose pose;

    int bezier_index;
};

class MotionProfile2D {
public:
	MotionProfile2D(const Path& path, bool inverted, double trackwidth, double friction_coefficient);

	TimeToVelocity getTimeToVelocity() const;

	double getDuration() const;

	MotionCommand get(double time) const;

	double limitSpeed(double vel, double curvature, double trackwidth, double friction_coefficient) const;
private:
	Path path;
	
	bool inverted;

	TimeToVelocity time_to_velocity;

	template <class MAIter>
	std::vector<double> pass(MAIter begin, MAIter end, bool pass_type) const;

	void computeProfile(double start_vel, double end_vel, double trackwidth, double friction_coefficient);
};
}