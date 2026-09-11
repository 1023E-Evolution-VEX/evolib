#pragma once

#include <string>

namespace evolib {
class Pose {
   public:
    float x;

    float y;

    float theta;

    Pose(float x, float y, float theta = 0);

    Pose operator+(const Pose& other);

    Pose operator-(const Pose& other);

    float operator*(const Pose& other);

    Pose operator*(const float& other);

    Pose operator/(const float& other);

    Pose lerp(Pose other, float t);

    float distance(Pose other) const;

    float angle(Pose other) const;

    Pose rotate(float angle);
};

std::string format_as(const Pose& pose);
}  // namespace evolib
