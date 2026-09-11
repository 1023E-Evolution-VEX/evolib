#pragma once

#include "evolib/pose.hpp"
#include "evolib/robot.hpp"

namespace evolib {

void setSensors(evolib::OdomSensors sensors, evolib::Drivetrain drivetrain);

Pose getPose(bool radians = false);

void setPose(Pose pose, bool radians = false);
void setOdomX(float x);
void setOdomY(float y);

Pose getSpeed(bool radians = false);

Pose getLocalSpeed(bool radians = false);

Pose estimatePose(float time, bool radians = false);

void update();

void init();
}  // namespace evolib
