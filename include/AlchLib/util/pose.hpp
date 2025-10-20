#pragma once

namespace alchlib {
struct Pose {
	double x;
	
	double y;

	double theta;

	double distanceTo(const Pose& other) const;

    double angleTo(const Pose& other) const;
};
}