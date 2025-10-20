#include "pros/rtos.hpp"
#include "AlchLib/util/timer.hpp"

alchlib::Timer::Timer(uint32_t time)
    : period(time) {
    lastTime = pros::millis();
}

uint32_t alchlib::Timer::getTimeSet() {
    const uint32_t time = pros::millis(); 
    if (!paused) timeWaited += time - lastTime; 
    lastTime = time; 
    return period;
}

uint32_t alchlib::Timer::getTimeLeft() {
    const uint32_t time = pros::millis(); 
    if (!paused) timeWaited += time - lastTime; 
    lastTime = time; 
    const int delta = period - timeWaited; 
    return (delta > 0) ? delta : 0; 
}

uint32_t alchlib::Timer::getTimePassed() {
    const uint32_t time = pros::millis(); 
    if (!paused) timeWaited += time - lastTime; 
    lastTime = time; 
    return timeWaited;
}

bool alchlib::Timer::isDone() {
    const uint32_t time = pros::millis(); 
    if (!paused) timeWaited += time - lastTime; 
    lastTime = time; 
    const int delta = period - timeWaited; 
    return delta <= 0;
}

bool alchlib::Timer::isPaused() {
    const uint32_t time = pros::millis(); 
    if (!paused) timeWaited += time - lastTime; 
    return paused;
}

void alchlib::Timer::set(uint32_t time) {
    period = time; 
    reset();
}

void alchlib::Timer::reset() {
    timeWaited = 0;
    lastTime = pros::millis();
}

void alchlib::Timer::pause() {
    if (!paused) lastTime = pros::millis();
    paused = true;
}

void alchlib::Timer::resume() {
    if (paused) lastTime = pros::millis();
    paused = false;
}

void alchlib::Timer::waitUntilDone() {
    do pros::delay(5);
    while (!this->isDone());
}