#include <cmath>

#include "AlchLib/chassis/chassis.hpp"
#include "AlchLib//util/math.hpp"

pros::Task* tracking_task = nullptr;

double prev_vertical = 0;
double prev_horizontal = 0;
double prev_imu = 0;

double vert_offset = 0.75;

double imu_scalar = 360.0 / 361.87;

void alchlib::Chassis::updatePose() {
    double vertical_raw = this->getDistanceTravelled();
    double horizontal_raw = this->odom_sensors.horizontalWheel->getDistanceTravelled();

    double imu_raw = Math::degToRad(360 - this->odom_sensors.imu->get_heading());
    double delta_imu = (imu_raw - prev_imu);
    prev_imu = imu_raw;

    double heading = Math::normalize360(this->pose.theta + delta_imu, true);
    double delta_heading = heading - this->pose.theta;
    double average_heading = this->pose.theta + delta_heading / 2;

    double delta_x = vertical_raw - prev_vertical;
    double delta_y = horizontal_raw - prev_horizontal;
    prev_vertical = vertical_raw, prev_horizontal = horizontal_raw;

    double local_x = 0;
    double local_y = 0;

    if (delta_heading == 0) {
        local_x = delta_x;
        local_y = delta_y;
    }
    else {
        local_x = 2 * std::sin(delta_heading / 2) * (delta_x / delta_heading + vert_offset);
        local_y = 2 * std::sin(delta_heading / 2) * (delta_y / delta_heading + this->odom_sensors.horizontalWheel->getOffset());
    }
    this->pose.x += local_x * std::cos(average_heading) - local_y * std::sin(average_heading);
    this->pose.y += local_x * std::sin(average_heading) + local_y * std::cos(average_heading);
    this->pose.theta = heading;
}

void alchlib::Chassis::initOdomTask() {
    if (tracking_task == nullptr) {
        tracking_task =  new pros::Task {[=, this] {
            while (true) {
                updatePose();
                pros::delay(10);
            }
        }};
    }
}

void alchlib::Chassis::setPose(const alchlib::Pose& pose, bool radians) {
    if (!radians) {
        this->pose.x = pose.x;
        this->pose.y = pose.y;
        this->pose.theta = Math::degToRad(pose.theta);
    }
    else {
        this->pose.x = pose.x;
        this->pose.y = pose.y;
        this->pose.theta = pose.theta;
    }
}

alchlib::Pose alchlib::Chassis::getPose(bool radians) {
    if (!radians) {
        return {this->pose.x, this->pose.y, Math::radToDeg(this->pose.theta)};
    }
    else {
        return this->pose;
    }
}