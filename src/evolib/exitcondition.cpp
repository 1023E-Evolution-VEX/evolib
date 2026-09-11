#include "evolib/exitcondition.hpp"

#include <cmath>

#include "pros/rtos.hpp"
namespace evolib {
ExitCondition::ExitCondition(const float range, const int time)
    : range(range), time(time) {}

bool ExitCondition::getExit() { return done; }
bool ExitCondition::update(const float input) {
    int currentTime = pros::millis();
    if (std::fabs(input) > range) startTime = -1;
    else if (startTime == -1) startTime = currentTime;
    else if (currentTime - startTime > time) done = true;
    return done;
}
void ExitCondition::reset() {
    done = false;
    startTime = -1;
}
}                     