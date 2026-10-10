// Adapted from LemLib (https://github.com/LemLib/LemLib), MIT License.
// See THIRD_PARTY_NOTICES.

#include <cmath>

#include "AlchLib/chassis/chassis.hpp"
#include "AlchLib/util/math.hpp"
#include "AlchLib/util/timer.hpp"

void alchlib::Chassis::moveRelative(float dist, int timeout, MoveRelativeParams params, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;

    if (async) {
        pros::Task task([&]() { moveRelative(dist, timeout, params, false); });
        this->endMotion();
        pros::delay(10); 
        return;
    }

    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();
    distTraveled = 0;
    Timer timer(timeout);

    float distTarget = params.forwards ? dist : -dist;
    float currDist = this->getDistanceTravelled();
    double headingTarget = getPose(true).theta;
    
    while (!timer.isDone() && ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit())) && this->motionRunning) {
        Pose pose = getPose(true);

        distTraveled = this->getDistanceTravelled() - currDist;
        const float lateralError = distTarget - distTraveled;

        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        float lateralOut = lateralPID.update(lateralError);
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

        const float angularError = Math::normalize180(headingTarget - pose.theta, true);

        float angularOut = angularPID.update(angularError);
        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);

        printf("%f %f\n", angularError, angularOut);

        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        this->drivetrain.left_motors->move(leftPower);
        this->drivetrain.right_motors->move(rightPower);

        pros::delay(10);
    }
    drivetrain.left_motors->move(0);
    drivetrain.right_motors->move(0);

    distTraveled = -1;
    this->endMotion();
    printf("\n");
}