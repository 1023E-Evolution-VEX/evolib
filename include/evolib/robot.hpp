#pragma once
#include "evolib/asset.hpp"
#include "evolib/exitcondition.hpp"
#include "evolib/pid.hpp"
#include "evolib/trackingWheel.hpp"
#include "pose.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
namespace evolib {

class OdomSensors {
   public:
    OdomSensors(TrackingWheel* vertical, TrackingWheel* horizontal,
                pros::Imu* imu);
    TrackingWheel* vertical;
    TrackingWheel* horizontal;
    pros::Imu* imu;
};

class ControllerSettings {
   public:
    ControllerSettings(float kP, float kI, float kD, float windupRange,
                       float smallError, float smallErrorTimeout,
                       float largeError, float largeErrorTimeout, float slew)
        : kP(kP),
          kI(kI),
          kD(kD),
          windupRange(windupRange),
          smallError(smallError),
          smallErrorTimeout(smallErrorTimeout),
          largeError(largeError),
          largeErrorTimeout(largeErrorTimeout),
          slew(slew) {}

    float kP;
    float kI;
    float kD;
    float windupRange;
    float smallError;
    float smallErrorTimeout;
    float largeError;
    float largeErrorTimeout;
    float slew;
};

enum class AngularDirection { CW_CLOCKWISE, CCW_COUNTERCLOCKWISE, AUTO };
struct TurnToPointParams {
    bool forwards = true;
    AngularDirection direction = AngularDirection::AUTO;
    int maxSpeed = 127;
    int minSpeed = 0;
    float earlyExitRange = 0;
};
struct TurnToHeadingParams {
    AngularDirection direction = AngularDirection::AUTO;
    int maxSpeed = 127;
    int minSpeed = 0;
    float earlyExitRange = 0;
};
enum class DriveSide { LEFT, RIGHT };
struct SwingToPointParams {
    bool forwards = true;
    AngularDirection direction = AngularDirection::AUTO;
    float maxSpeed = 127;
    float minSpeed = 0;
    float earlyExitRange = 0;
};
struct SwingToHeadingParams {
    AngularDirection direction = AngularDirection::AUTO;
    float maxSpeed = 127;
    float minSpeed = 0;
    float earlyExitRange = 0;
};
struct BoomerangParams {
    bool forwards = true;
    float horizontalDrift = 0;
    float lead = 0.6;
    float maxSpeed = 127;
    float minSpeed = 0;
    float earlyExitRange = 0;
};
struct MoveToPointParams {
    bool forwards = true;
    float maxSpeed = 127;
    float minSpeed = 0;
    float earlyExitRange = 0;
};
struct MoveToPoint2Params {
    bool forwards = true;
    float turnPower = 11;
    float settleFactor = 4;
    float maxSpeed = 127;
    float minSpeed = 0;
    float earlyExitRange = 0;
};
struct MoveForDistanceParams {
    bool forwards = true;
    float maxSpeed = 127;
    float minSpeed = 0;
    bool chain = false;
};
class Drivetrain {
   public:
    Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors,
               float horizontalDrift);
    pros::MotorGroup* leftMotors;
    pros::MotorGroup* rightMotors;
    float horizontalDrift;
};

class Robot {
   public:
    Robot(Drivetrain dt, ControllerSettings linearSettings,
          ControllerSettings angularSettings, OdomSensors sensors);

    void calibrate();
    void setPose(float x, float y, float theta);
    void setPose(Pose pose);
    void setX(float x);
    void setY(float y);
    Pose getPose(bool radians = false, bool standardPos = false);

    void setBrakeType(pros::motor_brake_mode_e_t mode);
    void turnToPoint(float x, float y, int timeout,
                     TurnToPointParams params = {}, bool async = true);
    void turnToHeading(float theta, int timeout,
                       TurnToHeadingParams params = {}, bool async = true);

    void swingToHeading(float theta, DriveSide lockedSide, int timeout,
                        SwingToHeadingParams params = {}, bool async = true);
    void swingToPoint(float x, float y, DriveSide lockedSide, int timeout,
                      SwingToPointParams params = {}, bool async = true);
    void boomerang(float x, float y, float theta, int timeout,
                   BoomerangParams params = {}, bool async = true);
    void moveToPoint(float x, float y, int timeout,
                     MoveToPointParams params = {}, bool async = true);
    void moveToPoint2(float x, float y, int timeout,
                      MoveToPoint2Params params = {}, bool async = true);
    void follow(const asset& path, float lookahead, int timeout,
                bool forwards = true, bool async = true);
    void moveForDistance(float dist, int timeout, MoveForDistanceParams params,
                         bool async = true);
    void cancelMotion();
    void cancelAllMotions();
    bool isInMotion() const;
    void resetLocalPosition();
    void waitUntil(float dist);
    void waitUntilDone();

   protected:
    void requestMotionStart();
    void endMotion();
    ControllerSettings lateralSettings;
    ControllerSettings angularSettings;
    OdomSensors sensors;
    Drivetrain dt;
    PID lateralPID;
    PID angularPID;
    ExitCondition lateralLargeExit;
    ExitCondition lateralSmallExit;
    ExitCondition angularLargeExit;
    ExitCondition angularSmallExit;
    bool motionRunning = false;
    bool motionQueued = false;
    float distTraveled = 0;

   private:
    pros::Mutex mutex;
};
}  // namespace evolib