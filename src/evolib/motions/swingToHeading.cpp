#include <cmath>
#include "evolib/robot.hpp"
#include "evolib/timer.hpp"
#include "evolib/util.hpp"
#include "pros/misc.hpp"

void evolib::Robot::swingToHeading(float theta, DriveSide lockedSide, int timeout, SwingToHeadingParams params,
                                     bool async) {
    params.minSpeed = fabs(params.minSpeed);
    this->requestMotionStart();
                                  
    if (!this->motionRunning) return;
                                                     
    if (async) {
        pros::Task task([&]() { swingToHeading(theta, lockedSide, timeout, params, false); });
        this->endMotion();
        pros::delay(10);                                        
        return;
    }
    float targetTheta;
    float deltaTheta;
    float motorPower;
    float prevMotorPower = 0;
    float startTheta = getPose().theta;
    bool settling = false;
    std::optional<float> prevRawDeltaTheta = std::nullopt;
    std::optional<float> prevDeltaTheta = std::nullopt;
    std::uint8_t compState = pros::competition::get_status();
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();
                                                                                                          
    pros::MotorBrake brakeMode = (lockedSide == DriveSide::LEFT)
                                     ? this->dt.leftMotors->get_brake_mode_all().at(0)
                                     : this->dt.rightMotors->get_brake_mode_all().at(0);
                                                
    if (lockedSide == DriveSide::LEFT) this->dt.leftMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    else this->dt.rightMotors->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

                
    while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit() && this->motionRunning) {
                           
        Pose pose = getPose();
        pose.theta = fmod(pose.theta, 360);

                                 
        distTraveled = fabs(angleError(pose.theta, startTheta, false));
        targetTheta = theta;

                            
        const float rawDeltaTheta = angleError(targetTheta, pose.theta, false);
        if (prevRawDeltaTheta == std::nullopt) prevRawDeltaTheta = rawDeltaTheta;
        if (sgn(rawDeltaTheta) != sgn(prevRawDeltaTheta)) settling = true;
        prevRawDeltaTheta = rawDeltaTheta;

                               
        if (settling) deltaTheta = angleError(targetTheta, pose.theta, false);
        else deltaTheta = angleError(targetTheta, pose.theta, false, params.direction);
        if (prevDeltaTheta == std::nullopt) prevDeltaTheta = deltaTheta;

                          
        if (params.minSpeed != 0 && fabs(deltaTheta) < params.earlyExitRange) break;
        if (params.minSpeed != 0 && sgn(deltaTheta) != sgn(prevDeltaTheta)) break;

                              
        motorPower = angularPID.update(deltaTheta);
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

                        
        if (motorPower > params.maxSpeed) motorPower = params.maxSpeed;
        else if (motorPower < -params.maxSpeed) motorPower = -params.maxSpeed;
        if (fabs(deltaTheta) > 20) motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
        if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        else if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
        prevMotorPower = motorPower;

                      
        if (lockedSide == DriveSide::LEFT) {
            dt.rightMotors->move(-motorPower);
            dt.leftMotors->brake();
        } else {
            dt.leftMotors->move(motorPower);
            dt.rightMotors->brake();
        }

                                  
        pros::delay(10);
    }

                                                             
                     
    if (lockedSide == DriveSide::LEFT) this->dt.leftMotors->set_brake_mode_all(brakeMode);
    else this->dt.rightMotors->set_brake_mode_all(brakeMode);
                  
    dt.leftMotors->move(0);
    dt.rightMotors->move(0);
                                                                        
    distTraveled = -1;
    this->endMotion();
}