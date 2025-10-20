#include <cmath>
#include <algorithm>

#include "AlchLib/motionProfiling/motionProfile2D.hpp"
#include "AlchLib/util/math.hpp"

alchlib::MotionProfile2D::MotionProfile2D(const Path& path, bool inverted, double trackwidth, double friction_coefficient)
    : path(path), 
      inverted(inverted) {
    computeProfile(this->path.start_vel, this->path.end_vel, trackwidth, friction_coefficient);
}

double alchlib::MotionProfile2D::limitSpeed(double vel, double curvature, double trackwidth, double friction_coefficient) const {
	double max_turn_speed = (2 * vel) / (2 + std::fabs(curvature) * trackwidth);
	if (curvature == 0) {
		return max_turn_speed;
	}
	double max_slip_speed = std::sqrt(friction_coefficient * (1 / std::fabs(curvature)) * 9.81 * 39.3701);
	return std::min(max_turn_speed, max_slip_speed);
}

template <class MAIter>
std::vector<double> alchlib::MotionProfile2D::pass(MAIter begin, MAIter end, bool pass_type) const {
	std::vector<double> vels = { begin->vel };

    double accel = 0;
	if (pass_type) {
		accel = begin->accel;
	}
	else {
		accel = begin->decel;
	}

	int i = 1;
	for (MAIter iter = begin + 1; iter != end; ++iter, ++i) {
		double delta_dist = std::fabs(iter->dist - (iter - 1)->dist);
		double vel = vels.at(i - 1);
		vels.push_back(std::min(std::sqrt(vel * vel + 2 * accel * delta_dist), iter->vel)); //vf = sqrt(vi^2 + 2 * a * change_d)
	}

	return vels;
}

void alchlib::MotionProfile2D::computeProfile(double start_vel, double end_vel, double trackwidth, double friction_coefficient) {
	std::vector<MotionAttribute> unlimited_speed = {
		{
			0.0,
			std::min(start_vel, this->limitSpeed(start_vel, this->path.getSegment(0).bezier.getCurvature(0), trackwidth, friction_coefficient)),
			this->path.getSegment(0).constraints.max_accel,
            this->path.getSegment(0).constraints.max_decel
		}
	};

	double distance = 0;
	this->time_to_velocity.t.push_back(0);

	for (int index = 0; index < path.getNumSegments(); index++) {
		PathSegment segment = path.getSegment(index); 
		CubicBezier bezier = path.getBezier(index);

		double length = bezier.getLength();
		int count = static_cast<int>(std::max(5.0, length / 0.02));

		for (int j = 1; j <= count; j++) {
			double t = (double) j / count;
			this->time_to_velocity.t.push_back(index + t);

			double curvature = bezier.getCurvature(t);

			unlimited_speed.push_back(
				{
					distance + bezier.getDistanceFromT(t),
					this->limitSpeed(segment.constraints.max_vel, curvature, trackwidth, friction_coefficient),
					segment.constraints.max_accel,
                    segment.constraints.max_decel
				}
			);
		}
		distance += length;
	}

	unlimited_speed.back().vel = std::min(end_vel, unlimited_speed.back().vel);

	std::vector<double> left_pass = this->pass(unlimited_speed.begin(), unlimited_speed.end(), true);	
	std::vector<double> right_pass = this->pass(unlimited_speed.rbegin(), unlimited_speed.rend(), false);
	std::reverse(right_pass.begin(), right_pass.end());

	for (int i = 0; i < unlimited_speed.size(); i++) {
		unlimited_speed.at(i).vel = std::min(left_pass.at(i), right_pass.at(i));
	}

	double time = 0;

	this->time_to_velocity.time.push_back(time);
	this->time_to_velocity.vel.push_back(unlimited_speed.at(0).vel);

	for (int i = 1; i < unlimited_speed.size(); i++) {
		double delta_dist = unlimited_speed.at(i).dist - unlimited_speed.at(i - 1).dist;
		double change_in_v = unlimited_speed.at(i).vel * unlimited_speed.at(i).vel - unlimited_speed.at(i - 1).vel * unlimited_speed.at(i - 1).vel;
		double accel = change_in_v / (2 * delta_dist); //a = (vf^2 - vi^2) / (2 * Δd)

		if (std::fabs(accel) > 0.01) {
			time += (unlimited_speed.at(i).vel - unlimited_speed.at(i - 1).vel) / (accel); //t = change_v / a
		}
		else {
			time += delta_dist / unlimited_speed.at(i).vel; //t = change_d / v
		}
		this->time_to_velocity.vel.push_back(unlimited_speed.at(i).vel);
		this->time_to_velocity.time.push_back(time);
	}
}

alchlib::TimeToVelocity alchlib::MotionProfile2D::getTimeToVelocity() const {
	return this->time_to_velocity;
}

double alchlib::MotionProfile2D::getDuration() const {
	return this->time_to_velocity.time.back();
}
 
alchlib::MotionCommand alchlib::MotionProfile2D::get(double time) const {
	if (time > this->getDuration()) {
		return { 0, 0, {0, 0, 0}, 0 };
	}
	double t_global = Math::interpolate(this->time_to_velocity.time, this->time_to_velocity.t, time);
	CubicBezier bezier = this->path.getBezierFromT(t_global);
	double t_local = this->path.getLocalT(t_global);

	int inverted_multiplier = this->inverted ? -1 : 1;

	double v_d = Math::interpolate(this->time_to_velocity.time, this->time_to_velocity.vel, time);

	double curvature = bezier.getCurvature(t_local);
	Pose pose = bezier.getPose(t_local);
	pose.theta = this->inverted ? pose.theta + Math::PI : pose.theta;

	return {
		v_d * inverted_multiplier,
		v_d * curvature,
		pose,
        std::min(static_cast<int>(std::floor(t_global)), static_cast<int>(this->path.segments.size() - 1))
	};
}