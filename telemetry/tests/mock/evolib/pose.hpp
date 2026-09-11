#pragma once
namespace evolib {
struct Pose {
    float x, y, theta;
    Pose(float x = 0, float y = 0, float theta = 0) : x(x), y(y), theta(theta) {}
};
}
