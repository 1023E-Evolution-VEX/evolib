

#define FMT_HEADER_ONLY
#include "evolib/pose.hpp"

#include "fmt/core.h"

evolib::Pose::Pose(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

evolib::Pose evolib::Pose::operator+(const evolib::Pose& other) {
    return evolib::Pose(this->x + other.x, this->y + other.y, this->theta);
}

evolib::Pose evolib::Pose::operator-(const evolib::Pose& other) {
    return evolib::Pose(this->x - other.x, this->y - other.y, this->theta);
}

float evolib::Pose::operator*(const evolib::Pose& other) {
    return this->x * other.x + this->y * other.y;
}

evolib::Pose evolib::Pose::operator*(const float& other) {
    return evolib::Pose(this->x * other, this->y * other, this->theta);
}

evolib::Pose evolib::Pose::operator/(const float& other) {
    return evolib::Pose(this->x / other, this->y / other, this->theta);
}

evolib::Pose evolib::Pose::lerp(evolib::Pose other, float t) {
    return evolib::Pose(this->x + (other.x - this->x) * t,
                        this->y + (other.y - this->y) * t, this->theta);
}

float evolib::Pose::distance(evolib::Pose other) const {
    return std::hypot(this->x - other.x, this->y - other.y);
}

float evolib::Pose::angle(evolib::Pose other) const {
    return std::atan2(other.y - this->y, other.x - this->x);
}

evolib::Pose evolib::Pose::rotate(float angle) {
    return evolib::Pose(this->x * std::cos(angle) - this->y * std::sin(angle),
                        this->x * std::sin(angle) + this->y * std::cos(angle),
                        this->theta);
}

std::string evolib::format_as(const evolib::Pose& pose) {
    return fmt::format("Pose {{ x: {}, y: {}, theta: {} }}", pose.x, pose.y,
                       pose.theta);
}