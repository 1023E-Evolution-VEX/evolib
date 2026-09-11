#pragma once

#include <cmath>
#include <vector>

#include "evolib/pose.hpp"
#include "evolib/robot.hpp"

namespace evolib {

float slew(float target, float current, float maxChange);

constexpr float radToDeg(float rad) { return rad * 180 / M_PI; }

constexpr float degToRad(float deg) { return deg * M_PI / 180; }

constexpr float sanitizeAngle(float angle, bool radians = true);

float angleError(float target, float position, bool radians = true,
                 AngularDirection direction = AngularDirection::AUTO);

template <typename T>
constexpr T sgn(T value) {
    return value < 0 ? -1 : 1;
}

float avg(std::vector<float> values);

float ema(float current, float previous, float smooth);

float getCurvature(Pose pose, Pose other);
}  // namespace evolib