#pragma once

namespace alchlib {
class Feedforward {
public:
    Feedforward(double kS, double kV, double kA);

    double update(double vel);
    
    void reset();
private:
    double kS;
    double kV;
    double kA;

    double prevVel = 0;
};
}