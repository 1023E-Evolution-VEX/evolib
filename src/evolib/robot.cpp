
#include "evolib/robot.hpp"

#include <math.h>

#include "evolib/odom.hpp"
#include "evolib/trackingWheel.hpp"
#include "evolib/util.hpp"
#include "pros/imu.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
evolib::OdomSensors::OdomSensors(TrackingWheel* vertical,
                                 TrackingWheel* horizontal, pros::Imu* imu)
    : vertical(vertical), horizontal(horizontal), imu(imu) {}

evolib::Drivetrain::Drivetrain(pros::MotorGroup* lefts,
                               pros::MotorGroup* rights, float horizontalDrift)
    : leftMotors(lefts),
      rightMotors(rights),
      horizontalDrift(horizontalDrift) {}

evolib::Robot::Robot(Drivetrain dt, ControllerSettings linearSettings,
                     ControllerSettings angularSettings, OdomSensors sensors)
    : dt(dt),
      lateralSettings(linearSettings),
      angularSettings(angularSettings),
      sensors(sensors),
      lateralPID(linearSettings.kP, linearSettings.kI, linearSettings.kD,
                 linearSettings.windupRange, true),
      angularPID(angularSettings.kP, angularSettings.kI, angularSettings.kD,
                 angularSettings.windupRange, true),
      lateralLargeExit(lateralSettings.largeError,
                       lateralSettings.largeErrorTimeout),
      lateralSmallExit(lateralSettings.smallError,
                       lateralSettings.smallErrorTimeout),
      angularLargeExit(angularSettings.largeError,
                       angularSettings.largeErrorTimeout),
      angularSmallExit(angularSettings.smallError,
                       angularSettings.smallErrorTimeout) {}
void loadingAnimation(int iter) {
    int boarder = 50;

    pros::screen::set_pen(pros::c::COLOR_WHITE);
    for (int i = 1; i < 3; i++) {
        pros::screen::draw_rect(boarder + i, boarder + i, 480 - boarder - i,
                                240 - boarder - i);
    }

    if (iter < 2000) {
        static int last_x1 = boarder;
        pros::screen::set_pen(0x0000889C);
        int x1 = (iter * ((480 - (boarder * 2)) / 2000.0)) + boarder;
        pros::screen::fill_rect(last_x1, boarder, x1, 240 - boarder);
        last_x1 = x1;
    }

    else {
        static int last_x1 = boarder;
        pros::screen::set_pen(pros::c::COLOR_RED);
        int x1 = ((iter - 2000) * ((480 - (boarder * 2)) / 1000.0)) + boarder;
        pros::screen::fill_rect(last_x1, boarder, x1, 240 - boarder);
        last_x1 = x1;
    }
}

void calibrateIMU(evolib::OdomSensors& sensors) {
    int attempt = 1;
    bool calibrated = false;

    while (attempt <= 5) {
        sensors.imu->reset();

        do pros::delay(10);
        while (sensors.imu->get_status() != pros::ImuStatus::error &&
               sensors.imu->is_calibrating());

        if (!isnanf(sensors.imu->get_heading()) &&
            !isinf(sensors.imu->get_heading())) {
            calibrated = true;
            break;
        }

        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        attempt++;
    }

    if (attempt > 5) {
        sensors.imu = nullptr;
    }
}
void evolib::Robot::calibrate() {
    calibrateIMU(sensors);

    sensors.vertical->reset();
    sensors.horizontal->reset();
    setSensors(sensors, dt);
    init();
}

void evolib::Robot::setPose(float x, float y, float theta) {
    evolib::setPose(evolib::Pose(x, y, theta), false);
}

void evolib::Robot::setPose(Pose pose) { evolib::setPose(pose, false); }

evolib::Pose evolib::Robot::getPose(bool radians, bool standardPos) {
    Pose pose = evolib::getPose(true);
    if (standardPos) pose.theta = M_PI_2 - pose.theta;
    if (!radians) pose.theta = radToDeg(pose.theta);
    return pose;
}
void evolib::Robot::setX(float x) { evolib::setOdomX(x); }
void evolib::Robot::setY(float y) { evolib::setOdomY(y); }
void evolib::Robot::waitUntil(float dist) {
    do pros::delay(10);
    while (distTraveled <= dist && distTraveled != -1);
}

void evolib::Robot::waitUntilDone() {
    do pros::delay(10);
    while (distTraveled != -1);
}

void evolib::Robot::requestMotionStart() {
    if (this->isInMotion())
        this->motionQueued = true;
    else
        this->motionRunning = true;

    this->mutex.take(TIMEOUT_MAX);
}

void evolib::Robot::endMotion() {
    this->motionRunning = this->motionQueued;
    this->motionQueued = false;

    this->mutex.give();
}

void evolib::Robot::cancelMotion() {
    this->motionRunning = false;
    pros::delay(10);
}

void evolib::Robot::cancelAllMotions() {
    this->motionRunning = false;
    this->motionQueued = false;
    pros::delay(10);
}

bool evolib::Robot::isInMotion() const { return this->motionRunning; }

void evolib::Robot::resetLocalPosition() {
    float theta = this->getPose().theta;
    evolib::setPose(evolib::Pose(0, 0, theta), false);
}

void evolib::Robot::setBrakeType(pros::motor_brake_mode_e_t mode) {
    dt.leftMotors->set_brake_mode_all(mode);
    dt.rightMotors->set_brake_mode_all(mode);
}
