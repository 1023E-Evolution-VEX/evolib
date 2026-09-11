#pragma once

#include "evolib/trackingWheel.hpp"

#include <math.h>

evolib::TrackingWheel::TrackingWheel(pros::Rotation* rotSensor, float offset,
                                     float diameter) {
    this->rotSensor = rotSensor;
    this->offset = offset;
    this->diameter = diameter;
}

void evolib::TrackingWheel::reset() { this->rotSensor->reset_position(); }

float evolib::TrackingWheel::getDistanceMoved() {
    return (float(this->rotSensor->get_position()) * this->diameter * M_PI /
            36000);
}

float evolib::TrackingWheel::getOffset() { return this->offset; }
float evolib::TrackingWheel::getDiameter() { return this->diameter; }