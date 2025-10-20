#pragma once

#include "AlchLib/controllers/pid.hpp"
#include "AlchLib/devices/trackingWheel.hpp"
#include "AlchLib/motionProfiling/motionProfile2D.hpp"
#include "AlchLib/controllers/feedforward.hpp"
#include "AlchLib/controllers/ramsete.hpp"
#include "AlchLib/util/exitConditions.hpp"

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

namespace alchlib {

struct OdometrySensors {
    TrackingWheel* verticalWheel;
    TrackingWheel* horizontalWheel;
    pros::IMU* imu;
};

struct PIDSettings {
    float kP;
    float kI;
    float kD;
    float small_error;
    float small_error_timeout;
    float large_error;
    float large_error_timeout;
    float slew;
};

struct FeedforwardSettings {
    float kS;
    float kV;
    float kA;
};

enum class AngularDirection {
    CW_CLOCKWISE, 
    CCW_COUNTERCLOCKWISE, 
    AUTO
};

struct TurnToPointParams {
    bool forwards = true;

    AngularDirection direction = AngularDirection::AUTO;

    int maxSpeed = 127;
       
    int minSpeed = 0;

    float earlyExitRange = 0;
};

struct TurnToHeadingParams {
    AngularDirection direction = AngularDirection::AUTO;

    int maxSpeed = 127;

    int minSpeed = 0;
    
    float earlyExitRange = 0;
};

struct MoveToPointParams {
    bool forwards = true;
    
    float maxSpeed = 127;

    float minSpeed = 0;

    float earlyExitRange = 0;
};

struct MoveRelativeParams {
    bool forwards = true;

    float maxSpeed = 127;

    float minSpeed = 0;

    float earlyExitRange = 0;
};

struct Drivetrain {
    pros::MotorGroup* left_motors;
    pros::MotorGroup* right_motors;
};

class Chassis {
public:
    Chassis(const Drivetrain& drivetrain, const OdometrySensors& odom_sensors, const PIDSettings& lateral_settings, 
            const PIDSettings& angular_settings, const PIDSettings& velocity_pid_settings,
            const FeedforwardSettings& ff_settings);

    void calibrate();

    void setPose(const Pose& pose, bool radians = false);
    Pose getPose(bool radians = false);

    void waitUntil(float dist);
    void waitUntilDone();
    void cancelMotion();
    void cancelAllMotions();
    bool isInMotion() const;

    void turnToHeading(float theta, int timeout, TurnToHeadingParams params = {}, bool async = true);
    void moveToPoint(float x, float y, int timeout, MoveToPointParams params = {}, bool async = true);
    void moveRelative(float dist, int timeout, MoveRelativeParams params = {}, bool async = true);
    void followProfile(MotionProfile2D profile, bool async = true);

    void arcade(int throttle, int turn, float desaturateBias);
    void intake();
    void outtake();
    void stop_intake();
    void move_voltage(int left_volts, int right_volts);

    double getVelocity();
    double getLeftWheelVelocity();
    double getRightWheelVelocity();
    int bezier_running;

    Drivetrain drivetrain;
private:
    void updatePose();
    void initOdomTask();
    double getDistanceTravelled();

    void requestMotionStart();
    void endMotion();

    Pose pose;
    
    OdometrySensors odom_sensors;

    PIDSettings lateral_settings;
    PIDSettings angular_settings;

    Feedforward left_ff;
    Feedforward right_ff;

    PID left_velocity_pid;
    PID right_velocity_pid;

    PID lateralPID;
    PID angularPID;

    ExitCondition lateralLargeExit;
    ExitCondition lateralSmallExit;
    ExitCondition angularLargeExit;
    ExitCondition angularSmallExit;

    RamseteController ramsete = {2.0, 0.7};

    bool motionRunning = false;
    bool motionQueued = false;  
    float distTraveled = 0;

    pros::Mutex mutex;
};
}