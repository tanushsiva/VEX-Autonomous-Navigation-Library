// Adapted from LemLib (https://github.com/LemLib/LemLib), MIT License.
// See THIRD_PARTY_NOTICES.

#include <cmath>
#include "pros/rtos.hpp"
#include "AlchLib/util/exitConditions.hpp"

alchlib::ExitCondition::ExitCondition(const float range, const int time)
    : range(range),
      time(time) {}

bool alchlib::ExitCondition::getExit() { return done; }

bool alchlib::ExitCondition::update(const float input) {
    const int curTime = pros::millis();
    if (std::fabs(input) > range) startTime = -1;
    else if (startTime == -1) startTime = curTime;
    else if (curTime >= startTime + time) done = true;
    return done;
}

void alchlib::ExitCondition::reset() {
    startTime = -1;
    done = false;
}