#include <cmath>
#include "evolib/robot.hpp"
#include "evolib/timer.hpp"
#include "evolib/util.hpp"
#include "pros/misc.hpp"

void evolib::Robot::boomerang(float x, float y, float theta, int timeout, BoomerangParams params, bool async) {
                     
    this->requestMotionStart();
                                  
    if (!this->motionRunning) return;
                                                     
    if (async) {
        pros::Task task([&]() { boomerang(x, y, theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10);                                        
        return;
    }

                                     
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();
    angularLargeExit.reset();
    angularSmallExit.reset();

                                             
    Pose target(x, y, M_PI_2 - degToRad(theta));
    if (!params.forwards) target.theta = fmod(target.theta + M_PI, 2 * M_PI);                      

                                                         
                                                                                            

                                              
    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    bool lateralSettled = false;
    bool prevSameSide = false;
    float prevLateralOut = 0;                          
    float prevAngularOut = 0;                          
    const int compState = pros::competition::get_status();

                
    while (!timer.isDone() &&
           ((!lateralSettled || (!angularLargeExit.getExit() && !angularSmallExit.getExit())) || !close) &&
           this->motionRunning) {
                          
        const Pose pose = getPose(true, true);

                                   
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

                                                 
        const float distTarget = pose.distance(target);

                                                                             
        if (distTarget < 7.5 && close == false) {
            close = true;
            params.maxSpeed = fmax(fabs(prevLateralOut), 60);
        }

                                                      
        if (lateralLargeExit.getExit() && lateralSmallExit.getExit()) lateralSettled = true;

                                     
        Pose carrot = target - Pose(cos(target.theta), sin(target.theta)) * params.lead * distTarget;
        if (close) carrot = target;                     

                                                                         
        const bool robotSide =
            (pose.y - target.y) * -sin(target.theta) <= (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        const bool carrotSide = (carrot.y - target.y) * -sin(target.theta) <=
                                (carrot.x - target.x) * cos(target.theta) + params.earlyExitRange;
        const bool sameSide = robotSide == carrotSide;
                        
        if (!sameSide && prevSameSide && close && params.minSpeed != 0) break;
        prevSameSide = sameSide;

                          
        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularError =
            close ? angleError(adjustedRobotTheta, target.theta) : angleError(adjustedRobotTheta, pose.angle(carrot));
        float lateralError = pose.distance(carrot);
                                     
                                                     
                                                
        if (close) lateralError *= cos(angleError(pose.theta, pose.angle(carrot)));
        else lateralError *= sgn(cos(angleError(pose.theta, pose.angle(carrot))));

                                 
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);
        angularSmallExit.update(radToDeg(angularError));
        angularLargeExit.update(radToDeg(angularError));

                               
        float lateralOut = lateralPID.update(lateralError);
        float angularOut = angularPID.update(radToDeg(angularError));

                                              
        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);

                                              
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

                                                
        if (!close) lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

                                                                             
                   
        const float radius = 1 / fabs(getCurvature(pose, carrot));
        const float maxSlipSpeed(sqrt(dt.horizontalDrift * radius * 9.8));
        lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
                                                            
        const float overturn = fabs(angularOut) + fabs(lateralOut) - params.maxSpeed;
        if (overturn > 0) lateralOut -= lateralOut > 0 ? overturn : -overturn;

                                                
        if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

                                                        
        if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0) lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

                                 
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

                          
    dt.leftMotors->move(0);
    dt.rightMotors->move(0);
                                                                        
    distTraveled = -1;
    this->endMotion();
}