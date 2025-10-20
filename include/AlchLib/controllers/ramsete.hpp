#pragma once

#include <utility>

#include "AlchLib/motionProfiling/motionProfile2D.hpp"

namespace alchlib {

class RamseteController {
public:
	RamseteController(double beta, double zeta);
	std::pair<double, double> calculate(const Pose& actual, const MotionCommand& desired) const;

private:
	double zeta;
	double beta;
};
}