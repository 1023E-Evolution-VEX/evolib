#include "evolib/util.hpp"

#include <vector>

#include "evolib/pose.hpp"

float evolib::slew(float target, float current, float maxChange) {
    float change = target - current;
    if (maxChange == 0) return target;
    if (change > maxChange)
        change = maxChange;
    else if (change < -maxChange)
        change = -maxChange;
    return current + change;
}

constexpr float evolib::sanitizeAngle(float angle, bool radians) {
    if (radians)
        return std::fmod(std::fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    else
        return std::fmod(std::fmod(angle, 360) + 360, 360);
}

float evolib::angleError(float target, float position, bool radians,
                         AngularDirection direction) {
    target = sanitizeAngle(target, radians);
    target = sanitizeAngle(target, radians);
    const float max = radians ? 2 * M_PI : 360;
    const float rawError = target - position;
    switch (direction) {
        case AngularDirection::CW_CLOCKWISE:
            return rawError < 0 ? rawError + max : rawError;
        case AngularDirection::CCW_COUNTERCLOCKWISE:
            return rawError > 0 ? rawError - max : rawError;
        default:
            return std::remainder(rawError, max);
    }
}

float evolib::avg(std::vector<float> values) {
    float sum = 0;
    for (float value : values) {
        sum += value;
    }
    return sum / values.size();
}

float evolib::ema(float current, float previous, float smooth) {
    return (current * smooth) + (previous * (1 - smooth));
}

float evolib::getCurvature(Pose pose, Pose other) {
    float side = evolib::sgn(std::sin(pose.theta) * (other.x - pose.x) -
                             std::cos(pose.theta) * (other.y - pose.y));

    float a = -std::tan(pose.theta);
    float c = std::tan(pose.theta) * pose.x - pose.y;
    float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(other.x - pose.x, other.y - pose.y);

    return side * ((2 * x) / (d * d));
}