#pragma once

#include <cstdint>
#include <vector>

#include "evolib/pose.hpp"
#include "pros/distance.hpp"

namespace evolib {

struct RclSensor {
    pros::Distance* sensor;
    float right = 0;
    float forward = 0;
    float heading = 0;
};

struct RclConfig {
    float fieldHalfSize = 72;
    float angleTolerance = 10;
    float minDistanceMm = 20;
    float maxDistanceMm = 2000;
    int minConfidence = 60;
    float minCorrection = .05f;
    float maxCorrection = 4;
    float maxSyncPerSecond = 3;
    std::uint32_t periodMs = 40;
    std::uint32_t samples = 1;
};

struct RclStatus {
    bool enabled = false;
    std::uint32_t accepted = 0;
    std::uint32_t rejected = 0;
    float correctionX = 0;
    float correctionY = 0;
};

bool configureRcl(std::vector<RclSensor> sensors, RclConfig config = {});
void disableRcl();
RclStatus getRclStatus();
bool addRclLine(float x1, float y1, float x2, float y2,
                std::uint32_t lifetimeMs = 0);
bool addRclCircle(float x, float y, float radius, std::uint32_t lifetimeMs = 0);
void clearRclObstacles();

namespace detail {
Pose correctRcl(Pose pose, std::uint32_t now);
void resetRcl();
}  // namespace detail
}  // namespace evolib
