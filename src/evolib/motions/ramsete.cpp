#include "ramsete.h"

#include <cmath>
#include <iostream>
struct DriveSpeeds {
    QVelocity linearVelocity = 0.0;
    QAngularVelocity angularVelocity = 0.0;
};
void setDriveSpeeds(DriveSpeeds lastSpeeds,
                    DriveSpeeds nextDriveSpeeds = {0.0, 0.0}) {
    const QAngularAcceleration angularAcceleration =
        (nextDriveSpeeds.angularVelocity - lastSpeeds.angularVelocity) / 10_ms;
    const QAcceleration linearAcceleration =
        (nextDriveSpeeds.linearVelocity - lastSpeeds.linearVelocity) / 10_ms;

    const double uLinear =
        (CONFIG::DRIVETRAIN_LINEAR_VELOCITY_FF_NO_GOAL)*Eigen::Vector2d(
            nextDriveSpeeds.linearVelocity.getValue(),
            linearAcceleration.getValue());
    const double uAngular =
        (CONFIG::DRIVETRAIN_ANGULAR_VELOCITY_FF_NO_GOAL)*Eigen::Vector2d(
            nextDriveSpeeds.angularVelocity.getValue(),
            angularAcceleration.getValue());

    double left = uLinear - uAngular;
    double right = uLinear + uAngular;

    left += signnum_c(left) * CONFIG::K_s;
    right += signnum_c(right) * CONFIG::K_s;

    this->setPct(left, right);
}

Ramsete::Ramsete(MotionProfile* motion_profile, const float zeta,
                 const float beta)
    : drivetrain(drivetrain),
      zeta(zeta),
      beta(beta),
      motionProfile(motion_profile) {
    startTime = 0.0;
}

void Ramsete::initialize() { startTime = pros::millis() * millisecond; }

void Ramsete::execute() {
    if (const auto command =
            motionProfile->get(pros::millis() * millisecond - startTime);
        command.has_value()) {
        Eigen::Vector3f currentPose = drivetrain->getPose();
        Eigen::Vector3f desiredPose = command->desiredPose.cast<float>();

        Eigen::Vector2f error = Eigen::Rotation2Df(-currentPose.z()) *
                                (desiredPose - currentPose).head<2>();

        Angle errorAngle =
            angleDifference(desiredPose.z(), currentPose.z()).getValue();

        const auto k =
            2.0f * this->zeta *
            std::sqrt(Qsq(command->desiredAngularVelocity).getValue() +
                      this->beta * Qsq(command->desiredVelocity).getValue());

        const auto velocity_commanded =
            std::cos(errorAngle) * command->desiredVelocity +
            k * error.x() * metre / second;
        const auto angular_wheel_velocity_commanded =
            (command->desiredAngularVelocity.getValue() +
             k * errorAngle.getValue() +
             this->beta * command->desiredVelocity.getValue() *
                 sinc(errorAngle) * error.y());

        drivetrain->setDriveSpeeds(
            lastSpeeds, {velocity_commanded,
                         angular_wheel_velocity_commanded * radian / second});
        lastSpeeds = {velocity_commanded, angular_wheel_velocity_commanded};
    }
}

void Ramsete::end(bool interrupted) { std::cout << "DONE" << std::endl; }

bool Ramsete::isFinished() {
    return motionProfile->getDuration() <
           pros::millis() * millisecond - startTime;
}

                                                                              
