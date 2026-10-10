// Adapted from LemLib (https://github.com/LemLib/LemLib), MIT License.
// See LICENSE-LemLib.

#include <cmath>
#include "AlchLib/chassis/chassis.hpp"
#include "AlchLib/util/timer.hpp"
#include "AlchLib/util/math.hpp"
#include "pros/misc.hpp"

void alchlib::Chassis::turnToHeading(float theta, int timeout, TurnToHeadingParams params, bool async) {
    params.minSpeed = std::abs(params.minSpeed);
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { turnToHeading(theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    float targetTheta;
    float deltaTheta;
    float motorPower;
    float prevMotorPower = 0;
    float startTheta = getPose().theta;
    bool settling = false;
    std::optional<float> prevRawDeltaTheta = std::nullopt;
    std::optional<float> prevDeltaTheta = std::nullopt;
    std::uint8_t compState = pros::competition::get_status();
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();

    while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit() && this->motionRunning) {
        Pose pose = getPose();

        distTraveled = fabs(Math::lemlib_angle_error(pose.theta, startTheta, false));

        targetTheta = theta;

        const float rawDeltaTheta = -Math::lemlib_angle_error(targetTheta, pose.theta, false);
        if (prevRawDeltaTheta == std::nullopt) prevRawDeltaTheta = rawDeltaTheta;
        if (Math::sgn(rawDeltaTheta) != Math::sgn(prevRawDeltaTheta)) settling = true;
        prevRawDeltaTheta = rawDeltaTheta;

        if (settling) deltaTheta = -Math::lemlib_angle_error(targetTheta, pose.theta, false);
        else deltaTheta = -Math::lemlib_angle_error(targetTheta, pose.theta, false, params.direction);
        if (prevDeltaTheta == std::nullopt) prevDeltaTheta = deltaTheta;

        if (params.minSpeed != 0 && fabs(deltaTheta) < params.earlyExitRange) break;
        if (params.minSpeed != 0 && Math::sgn(deltaTheta) != Math::sgn(prevDeltaTheta)) break;

        // calculate the speed
        motorPower = angularPID.update(deltaTheta);
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // cap the speed
        if (motorPower > params.maxSpeed) motorPower = params.maxSpeed;
        else if (motorPower < -params.maxSpeed) motorPower = -params.maxSpeed;
        if (fabs(deltaTheta) > 20) motorPower = Math::slew(motorPower, prevMotorPower, angular_settings.slew);
        if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        else if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
        prevMotorPower = motorPower;

        // move the drivetrain
        drivetrain.left_motors->move(motorPower);
        drivetrain.right_motors->move(-motorPower);

        pros::delay(10);
    }

    // stop the drivetrain
    drivetrain.left_motors->move(0);
    drivetrain.right_motors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}