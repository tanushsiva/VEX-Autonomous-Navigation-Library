#include "main.h"
#include "config.hpp"
	
using namespace alchlib;

void initialize() {
	pros::lcd::initialize();

    chassis.calibrate();

    pros::Task screen_task([] {
        while (true) {
            alchlib::Pose pose = chassis.getPose(); 
            pros::lcd::print(0, "x: %f", pose.x); 
            pros::lcd::print(1, "y: %f", pose.y); 
            pros::lcd::print(2, "heading: %f", pose.theta);
            pros::delay(10);
        }
    });

    color_sensor.set_integration_time(5);
    color_sensor.set_led_pwm(35);

    lb.init();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

}

void opcontrol() {
	while (true) {
		pros::delay(10);      
    }
}