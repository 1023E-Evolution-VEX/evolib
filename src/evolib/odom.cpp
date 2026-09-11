
#include "evolib/odom.hpp"

#include <math.h>

#include "evolib/rcl.hpp"
#include "evolib/robot.hpp"
#include "evolib/trackingWheel.hpp"
#include "evolib/util.hpp"
#include "pros/rtos.hpp"
pros::Task* odomTask = nullptr;
pros::Mutex odomMutex;

evolib::OdomSensors odomSensors(nullptr, nullptr, nullptr);
evolib::Drivetrain dt(nullptr, nullptr, 0);
evolib::Pose odomPose(0, 0, 0);
float prevVertical = 0;
float prevHorizontal = 0;
float prevImu = 0;

void evolib::setSensors(evolib::OdomSensors sensors,
                        evolib::Drivetrain drivetrain) {
    dt = drivetrain;
    odomSensors = sensors;
}
void evolib::update() {
    odomMutex.take();
    float verticalRaw = 0;
    float horizontalRaw = 0;
    float imuRaw = 0;
    verticalRaw = odomSensors.vertical->getDistanceMoved();
    horizontalRaw = odomSensors.horizontal->getDistanceMoved();
    imuRaw = degToRad(odomSensors.imu->get_rotation());

    float deltaVertical = verticalRaw - prevVertical;
    float deltaHorizontal = horizontalRaw - prevHorizontal;
    float deltaImu = imuRaw - prevImu;

    prevImu = imuRaw;

    float heading = odomPose.theta;

    heading = imuRaw;
    float deltaHeading = heading - odomPose.theta;
    float avgHeading = odomPose.theta + deltaHeading / 2;

    float horizontalOffset = 0;
    float verticalOffset = 0;
    verticalOffset = odomSensors.vertical->getOffset();
    horizontalOffset = odomSensors.horizontal->getOffset();

    float deltaX = 0;
    float deltaY = 0;
    deltaY = verticalRaw - prevVertical;
    deltaX = horizontalRaw - prevHorizontal;
    prevVertical = verticalRaw;
    prevHorizontal = horizontalRaw;
    if (horizontalOffset == 100) {
        deltaX = 0;
    }
    deltaX = 0;

    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0) {
        localX = deltaX;
        localY = deltaY;
    } else {
        if (horizontalOffset == 100) {
            localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + 0);
        } else {
            localX = 2 * sin(deltaHeading / 2) *
                     (deltaX / deltaHeading + horizontalOffset);
        }
        localY = 2 * sin(deltaHeading / 2) *
                 (deltaY / deltaHeading + verticalOffset);
    }

    evolib::Pose prevPose = odomPose;

    odomPose.x += localY * sin(avgHeading);
    odomPose.y += localY * cos(avgHeading);
    odomPose.x += localX * -cos(avgHeading);
    odomPose.y += localX * sin(avgHeading);
    odomPose.theta = heading;
    auto corrected = detail::correctRcl(
        {odomPose.x, odomPose.y, radToDeg(odomPose.theta)}, pros::millis());
    odomPose.x = corrected.x;
    odomPose.y = corrected.y;
    odomMutex.give();
}
evolib::Pose evolib::getPose(bool radians) {
    odomMutex.take();

    evolib::Pose temp = odomPose;

    odomMutex.give();

    return radians ? temp : evolib::Pose(temp.x, temp.y, radToDeg(temp.theta));
}

void evolib::setPose(Pose pose, bool radians) {
    odomMutex.take();
    detail::resetRcl();
    odomSensors.imu->set_rotation(radians ? radToDeg(pose.theta) : pose.theta);
    odomPose =
        radians ? pose : evolib::Pose(pose.x, pose.y, degToRad(pose.theta));
    odomMutex.give();
}

void evolib::setOdomX(float x) {
    odomMutex.take();
    detail::resetRcl();
    odomPose.x = x;
    odomMutex.give();
}

void evolib::setOdomY(float y) {
    odomMutex.take();
    detail::resetRcl();
    odomPose.y = y;
    odomMutex.give();
}

void evolib::init() {
    if (odomTask == nullptr) {
        odomTask = new pros::Task{[] {
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
}
