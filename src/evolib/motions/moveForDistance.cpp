#include <cmath>

#include "evolib/robot.hpp"
#include "evolib/timer.hpp"
#include "evolib/util.hpp"
#include "pros/misc.hpp"

void evolib::Robot::moveForDistance(float dist, int timeout,
                                    MoveForDistanceParams params, bool async) {
    this->requestMotionStart();
                                  
    if (!this->motionRunning) return;
                                                     
    if (async) {
        pros::Task task(
            [&]() { moveForDistance(dist, timeout, params, false); });
        this->endMotion();
        pros::delay(10);                                         
        return;
    }

    float prevLateralOut = 0;                           
    float prevAngularOut = 0;                           
    bool close = false;

                                     
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();

                                              
    distTraveled = 0;
    Timer timer(timeout);
    const int compState = pros::competition::get_status();

                                             
    Pose initPose = getPose();

                
    while (!timer.isDone() &&
           (((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()))) &&
           this->motionRunning) {
                          
        const Pose pose = getPose(false);

                                   
        distTraveled = pose.distance(initPose);

                                                 
        float distTarget = fabs(dist) - pose.distance(initPose);
        if (!params.forwards) distTarget *= -1;
        if (fabs(distTarget) < 4 && close == false) {
            close = true;
            params.maxSpeed = fmax(fabs(prevLateralOut), 60);
        }
        if (distTarget < 3 && params.chain) {
            break;
        }
        if (distTraveled > fabs(dist) && params.minSpeed != 0) break;
                          
        float lateralError = distTarget;
                                 
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

                                                   

                               
        float lateralOut = lateralPID.update(lateralError);
        float angularOut =
            angularPID.update(angleError(initPose.theta, pose.theta, false));

                                              
        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);
        angularOut = slew(angularOut, prevAngularOut, angularSettings.slew);

                                              
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);
                                                
                                                                             

        lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

                                                
        if (params.forwards && !close)
            lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close)
            lateralOut = std::fmin(lateralOut, 0);

                                                        
        if (params.forwards && lateralOut < fabs(params.minSpeed) &&
            lateralOut > 0)
            lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) &&
            lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

                                 
        prevAngularOut = angularOut;
        prevLateralOut = lateralOut;

                                                    
        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float ratio =
            std::max(std::fabs(leftPower), std::fabs(rightPower)) /
            params.maxSpeed;
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