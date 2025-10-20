#include "AlchLib/chassis/chassis.hpp"
#include "AlchLib/chassis/ladybrown.hpp"

#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"

extern alchlib::Chassis chassis;

extern alchlib::LadyBrown lb;

extern pros::Motor intake_motor;

extern pros::adi::Pneumatics clamp;
extern pros::adi::Pneumatics doinker;

extern pros::Optical color_sensor;

extern pros::Distance front_dist_sensor;
extern pros::Distance left_dist_sensor;
extern pros::Distance back_dist_sensor;
extern pros::Distance right_dist_sensor;

extern pros::Controller master;