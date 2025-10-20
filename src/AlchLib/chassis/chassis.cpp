#include "AlchLib/chassis/chassis.hpp"
#include "Alchlib/util/math.hpp"
#include "config.hpp"

alchlib::Chassis::Chassis(const Drivetrain& drivetrain, const OdometrySensors& odom_sensors, 
                          const PIDSettings& lateral_settings, const PIDSettings& angular_settings,
                          const PIDSettings& velocity_pid_settings, const FeedforwardSettings& ff_settings)
    : drivetrain(drivetrain),
      odom_sensors(odom_sensors),
      lateral_settings(lateral_settings),
      angular_settings(angular_settings),
      left_ff(ff_settings.kS, ff_settings.kV, ff_settings.kA),
      right_ff(ff_settings.kS, ff_settings.kV, ff_settings.kA),
      left_velocity_pid(velocity_pid_settings.kP, velocity_pid_settings.kI, velocity_pid_settings.kD),
      right_velocity_pid(velocity_pid_settings.kP, velocity_pid_settings.kI, velocity_pid_settings.kD),
      lateralPID(lateral_settings.kP, lateral_settings.kI, lateral_settings.kD),
      angularPID(angular_settings.kP, angular_settings.kI, angular_settings.kD),
      lateralLargeExit(lateral_settings.large_error, lateral_settings.large_error_timeout),
      lateralSmallExit(lateral_settings.small_error, lateral_settings.small_error_timeout),
      angularLargeExit(angular_settings.large_error, angular_settings.large_error_timeout),
      angularSmallExit(angular_settings.small_error, angular_settings.large_error_timeout) {}

void alchlib::Chassis::calibrate() {
    this->odom_sensors.imu->reset(true);
    this->odom_sensors.verticalWheel->reset();
    this->odom_sensors.horizontalWheel->reset();
    this->drivetrain.left_motors->tare_position_all();
    this->drivetrain.right_motors->tare_position_all();
    this->initOdomTask();
    chassis.setPose({0, 0, 0});
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ". . .");
}

void alchlib::Chassis::intake() {
    intake_motor.move(-127);
}

void alchlib::Chassis::outtake() {
    intake_motor.move(127);
}

void alchlib::Chassis::stop_intake() {
    intake_motor.move(0);
}

double alchlib::Chassis::getDistanceTravelled() {
    double avg_pos_left = 0;
    double avg_pos_right = 0;
    std::vector<double> pos_left = this->drivetrain.left_motors->get_position_all();
    std::vector<double> pos_right = this->drivetrain.right_motors->get_position_all();

    for (int i = 0; i < 3; i++) {
        avg_pos_left += pos_left.at(i);
        avg_pos_right += pos_right.at(i);
    }

    avg_pos_left /= 3.0;  
    avg_pos_right /= 3.0; 
    double avg_pos = (avg_pos_left + avg_pos_right) / 2.0;

    return (avg_pos / 360) * 2.75 * Math::PI;
} 

void alchlib::Chassis::move_voltage(int left_volts, int right_volts) {
    this->drivetrain.left_motors->move(left_volts);
    this->drivetrain.right_motors->move(right_volts);
}

double alchlib::Chassis::getVelocity() {
    double avg_vel_left = 0;
    double avg_vel_right = 0;
    std::vector<double> rpm_left = this->drivetrain.left_motors->get_actual_velocity_all();
    std::vector<double> rpm_right = this->drivetrain.right_motors->get_actual_velocity_all();

    for (int i = 0; i < 3; i++) {
        avg_vel_left += rpm_left.at(i);
        avg_vel_right += rpm_right.at(i);
    }

    avg_vel_left /= 3.0;  
    avg_vel_right /= 3.0; 
    double avg_vel = (avg_vel_left + avg_vel_right) / 2.0;

    return ((avg_vel) / 60) * 2.75 * alchlib::Math::PI;
}

double alchlib::Chassis::getLeftWheelVelocity() {
    double avg_vel_left = 0;
    std::vector<double> rpm_left = this->drivetrain.left_motors->get_actual_velocity_all();

    for (int i = 0; i < 3; i++) {
        avg_vel_left += rpm_left.at(i);
    }

    avg_vel_left /= 3.0;  
    return ((avg_vel_left) / 60) * 2.75 * alchlib::Math::PI;
}

double alchlib::Chassis::getRightWheelVelocity() {
    double avg_vel_right = 0;
    std::vector<double> rpm_right = this->drivetrain.right_motors->get_actual_velocity_all();

    for (int i = 0; i < 3; i++) {
        avg_vel_right += rpm_right.at(i);
    }

    avg_vel_right /= 3.0;  
    return ((avg_vel_right) / 60) * 2.75 * alchlib::Math::PI;
}

void alchlib::Chassis::waitUntil(float dist) {
    do pros::delay(10);
    while (distTraveled <= dist && distTraveled != -1);
}

void alchlib::Chassis::waitUntilDone() {
    do pros::delay(10);
    while (distTraveled != -1);
}

void alchlib::Chassis::requestMotionStart() {
    if (this->isInMotion()) this->motionQueued = true; 
    else this->motionRunning = true; 

    this->mutex.take(TIMEOUT_MAX);
}

void alchlib::Chassis::endMotion() {
    this->motionRunning = this->motionQueued;
    this->motionQueued = false;

    this->mutex.give();
}

void alchlib::Chassis::cancelMotion() {
    this->motionRunning = false;
    pros::delay(10); 
}

void alchlib::Chassis::cancelAllMotions() {
    this->motionRunning = false;
    this->motionQueued = false;
    pros::delay(10); 
}

bool alchlib::Chassis::isInMotion() const { 
    return this->motionRunning; 
}

