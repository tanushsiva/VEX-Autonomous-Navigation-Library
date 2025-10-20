#pragma once

namespace alchlib {
class PID {
public:
    PID(double kP, double kI, double kD);

    double update(double error);
    
    void reset();
private:
    double kP;
    double kI;
    double kD;

    double integral = 0;
    double prev_error = 0;
};
}