#include "AlchLib/chassis/chassis.hpp"
#include "AlchLib/devices/trackingWheel.hpp"
#include "AlchLib/chassis/ladybrown.hpp"

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/distance.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({1, -2, 3}, pros::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
pros::MotorGroup right_motors({-4, 5, -7}, pros::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

pros::Motor intake_motor(6, pros::MotorGears::blue);

alchlib::LadyBrown lb(8,11);

pros::Rotation vertical_tracker(1);
pros::Rotation horizontal_tracker( 13);
pros::IMU imu(9);

pros::adi::Pneumatics clamp('a', false);
pros::adi::Pneumatics doinker('b', false);

pros::Optical color_sensor(12);

pros::Distance front_dist_sensor(14);
pros::Distance left_dist_sensor(16);
pros::Distance back_dist_sensor(15);
pros::Distance right_dist_sensor(17);

alchlib::TrackingWheel vertical_wheel(&vertical_tracker, 2.75, 0);
alchlib::TrackingWheel horizontal_wheel(&horizontal_tracker, 2, 1.5);

alchlib::OdometrySensors odom_sensors(&vertical_wheel, &horizontal_wheel, &imu);
alchlib::Drivetrain drivetrain(&left_motors, &right_motors);

alchlib::PIDSettings lateral_settings(
    20, 
    0, 
    76.5, 
    1, 
    100, 
    3, 
    500, 
    100
);

alchlib::PIDSettings angular_settings(
    5, 
    0, 
    32, 
    1, 
    100, 
    3, 
    500, 
    0
);

alchlib::PIDSettings velocity_pid_settings(0.775474025496, 0.0361895234068, 0);
alchlib::FeedforwardSettings ff_settings(8.79485774432, 1.30063749024 , 33.233925601);

alchlib::Chassis chassis(drivetrain, odom_sensors, lateral_settings, angular_settings, velocity_pid_settings, ff_settings);