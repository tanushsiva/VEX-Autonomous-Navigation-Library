// Adapted from LemLib (https://github.com/LemLib/LemLib), MIT License.
// See THIRD_PARTY_NOTICES.

#include <cmath>

#include "AlchLib/chassis/chassis.hpp"
#include "AlchLib/util/timer.hpp"
#include "AlchLib/util/math.hpp"

void alchlib::Chassis::moveToPoint(float x, float y, int timeout, MoveToPointParams params, bool async) {
    params.earlyExitRange = std::fabs(params.earlyExitRange);
    this->requestMotionStart();

    if (!this->motionRunning) return;

    if (async) {
        pros::Task task([&]() { moveToPoint(x, y, timeout, params, false); });
        this->endMotion();
        pros::delay(10); 
        return;
    }

    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();

    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    float prevLateralOut = 0; 
    float prevAngularOut = 0; 
    std::optional<bool> prevSide = std::nullopt;

    Pose target(x, y);
    target.theta = lastPose.angleTo(target);

    // main loop
    while (!timer.isDone() && ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) || !close) &&
           this->motionRunning) {

        Pose pose = this->getPose();
        pose.theta = Math::degToRad(pose.theta);

        distTraveled += pose.distanceTo(lastPose);
        lastPose = pose;

        const float distTarget = pose.distanceTo(target);

        if (distTarget < 7.5 && close == false) {
            close = true;
            params.maxSpeed = fmax(fabs(prevLateralOut), 60);
        }

        const bool side =
            (pose.y - target.y) * -sin(target.theta) <= (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        if (prevSide == std::nullopt) prevSide = side;
        const bool sameSide = side == prevSide;

        if (!sameSide && params.minSpeed != 0) break;
        prevSide = side;

        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularError = Math::lemlib_angle_error(adjustedRobotTheta, pose.angleTo(target));
        float lateralError = pose.distanceTo(target) * cos(Math::lemlib_angle_error(pose.theta, pose.angleTo(target)));

        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        float lateralOut = lateralPID.update(lateralError);
        float angularOut = angularPID.update(Math::radToDeg(angularError));
        if (close) angularOut = 0;

        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);
        angularOut = Math::slew(angularOut, prevAngularOut, angular_settings.slew);

        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

        if (!close) lateralOut = Math::slew(lateralOut, prevLateralOut, lateral_settings.slew);

        if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

        if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0) lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

        prevAngularOut = angularOut;
        prevLateralOut = lateralOut;

        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        // move the drivetrain
        this->drivetrain.left_motors->move(leftPower);
        this->drivetrain.right_motors->move(rightPower);

        // delay to save resources
        pros::delay(10);
    }

    drivetrain.left_motors->move(0);
    drivetrain.right_motors->move(0);

    distTraveled = -1;
    this->endMotion();
}