#pragma once

#include "pros/rotation.hpp"
namespace evolib {
class TrackingWheel {
   public:
    TrackingWheel(pros::Rotation* rotSensor, float offset, float diameter);

    void reset();

    float getDistanceMoved();

    float getOffset();
    float getDiameter();

   private:
    float offset;
    float diameter;
    pros::Rotation* rotSensor = nullptr;
};
}  // namespace evolib