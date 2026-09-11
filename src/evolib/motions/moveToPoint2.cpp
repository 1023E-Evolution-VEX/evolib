#include <cmath>
#include "evolib/robot.hpp"
#include "evolib/timer.hpp"
#include "evolib/util.hpp"
#include "pros/misc.hpp"

void evolib::Robot::moveToPoint2(float x, float y, int timeout, MoveToPoint2Params params, bool async) {
    params.earlyExitRange = fabs(params.earlyExitRange);
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([&]() { moveToPoint2(x, y, timeout, params, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();

    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    float prevLateralOut = 0;                          
    float prevAngularOut = 0;                          
    const int compState = pros::competition::get_status();
    std::optional<bool> prevSide = std::nullopt;

                                             
    Pose target(x, y);
    target.theta = lastPose.angle(target);
    float initDistTarget = lastPose.distance(target);
    
                
    while (!timer.isDone() && ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) || !close) &&
           this->motionRunning) {
                          
        const Pose pose = getPose(true, true);

                                   
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

                                                 
        const float distTarget = pose.distance(target);
                                                                             
        float lastAngle;
        if (distTarget < (initDistTarget / params.settleFactor) && close == false) {
            close = true;
            params.maxSpeed = fmax(fabs(prevLateralOut), 60);
                                      
        }

                          
        const bool side =
            (pose.y - target.y) * -sin(target.theta) <= (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        if (prevSide == std::nullopt) prevSide = side;
        const bool sameSide = side == prevSide;
                        
        if (!sameSide && params.minSpeed != 0) break;
        prevSide = side;

                          
        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularError = angleError(adjustedRobotTheta, pose.angle(target));
                              
                                                                                  
                        
                                                                                                      
                   
                                                                                                      
            
        float lateralError = pose.distance(target) * powf(cos(angularError), params.turnPower);
                                                                                                  
        
                                                                                                                                 

                                 
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

                               
        float lateralOut = lateralPID.update(lateralError);
        if (!params.forwards) lateralOut *= -1;
        float angularOut = angularPID.update(radToDeg(angularError));
        if (close) angularOut = 0;
        printf("%.2f: %.2f, %.2f, %.2f\n", distTarget, lateralOut, angularOut, angularError);
                                              
        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);
        angularOut = slew(angularOut, prevAngularOut, angularSettings.slew);

                                              
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);
                                                
                                                                             
        if (!close) lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

                                                
        if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

                                                        
        if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0) lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

                                 
        if (close) angularOut = 0;
        prevAngularOut = angularOut;
        prevLateralOut = lateralOut;

                                                    
        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

                      
        dt.leftMotors->move(leftPower);
        dt.rightMotors->move(rightPower);

                                  
        pros::delay(10);
    }

                  
    dt.leftMotors->brake();
    dt.rightMotors->brake();
                                                                        
    distTraveled = -1;
    this->endMotion();
}