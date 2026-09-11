                                                  
                                 
                                          
                                                                                                           

#include <cmath>
#include <vector>
#include <string>
#include "pros/misc.hpp"
#include "evolib/robot.hpp"
#include "evolib/util.hpp"
#include "evolib/asset.hpp"
   
                                                                                 
  
                              
                                                              
                                                                        
   
std::vector<std::string> readElement(const std::string& input, const std::string& delimiter) {
    std::string token;
    std::string s = input;
    std::vector<std::string> output;
    size_t pos = 0;

                
    while ((pos = s.find(delimiter)) != std::string::npos) {                                                  
        token = s.substr(0, pos);                       
        output.push_back(token);
        s.erase(0, pos + delimiter.length());                             
    }

    output.push_back(s);                                               

    return output;
}

   
                                 
  
                                     
                                         
   
std::string stringToHex(const std::string& input) {
    static const char hex_digits[] = "0123456789ABCDEF";

    std::string output;
    output.reserve(input.length() * 2);
    for (unsigned char c : input) {
        output.push_back(hex_digits[c >> 4]);
        output.push_back(hex_digits[c & 15]);
    }
    return output;
}

   
                                     
  
                                        
                                                                 
   
std::vector<evolib::Pose> getData(const asset& path) {
    std::vector<evolib::Pose> robotPath;

                                 
    const std::string data(reinterpret_cast<char*>(path.buf), path.size);
    const std::vector<std::string> dataLines = readElement(data, "\n");

                                              
    for (std::string line : dataLines) {
        if (line == "endData" || line == "endData\r") break;
        const std::vector<std::string> pointInput = readElement(line, ", ");              
                                               
        if (pointInput.size() != 3) {
            break;
        }
        evolib::Pose pathPoint(0, 0);
        pathPoint.x = std::stof(pointInput.at(0));              
        pathPoint.y = std::stof(pointInput.at(1));              
        pathPoint.theta = std::stof(pointInput.at(2));            
        robotPath.push_back(pathPoint);             
    }

    return robotPath;
}

   
                                                         
  
                                            
                                 
                                         
   
int findClosest(evolib::Pose pose, std::vector<evolib::Pose> path) {
    int closestPoint;
    float closestDist = infinity();

                                   
    for (int i = 0; i < path.size(); i++) {
        const float dist = pose.distance(path.at(i));
        if (dist < closestDist) {                     
            closestDist = dist;
            closestPoint = i;
        }
    }

    return closestPoint;
}

   
                                                                                
  
                                    
                                  
                                   
                                 
                                           
   
float circleIntersect(evolib::Pose p1, evolib::Pose p2, evolib::Pose pose, float lookaheadDist) {
                   
                                                                  
    evolib::Pose d = p2 - p1;
    evolib::Pose f = p1 - pose;
    float a = d * d;
    float b = 2 * (f * d);
    float c = (f * f) - lookaheadDist * lookaheadDist;
    float discriminant = b * b - 4 * a * c;

                                           
    if (discriminant >= 0) {
        discriminant = sqrt(discriminant);
        float t1 = (-b - discriminant) / (2 * a);
        float t2 = (-b + discriminant) / (2 * a);

                                           
        if (t2 >= 0 && t2 <= 1) return t2;
        else if (t1 >= 0 && t1 <= 1) return t1;
    }

                            
    return -1;
}

   
                                     
  
                                                  
                                                  
                                   
                                                               
                                                                 
   
evolib::Pose lookaheadPoint(evolib::Pose lastLookahead, evolib::Pose pose, std::vector<evolib::Pose> path, int closest,
                            float lookaheadDist) {
                             
                                                                                                
                   
                                                                                          
                      
    const int start = std::max(closest, int(lastLookahead.theta));
    for (int i = start; i < path.size() - 1; i++) {
        evolib::Pose lastPathPose = path.at(i);
        evolib::Pose currentPathPose = path.at(i + 1);

        float t = circleIntersect(lastPathPose, currentPathPose, pose, lookaheadDist);

        if (t != -1) {
            evolib::Pose lookahead = lastPathPose.lerp(currentPathPose, t);
            lookahead.theta = i;
            return lookahead;
        }
    }

                                                         
    return lastLookahead;
}

   
                                                                                         
  
                                       
                                          
                                       
                          
   
float findLookaheadCurvature(evolib::Pose pose, float heading, evolib::Pose lookahead) {
                                                                             
    float side = evolib::sgn(std::sin(heading) * (lookahead.x - pose.x) - std::cos(heading) * (lookahead.y - pose.y));
                                        
    float a = -std::tan(heading);
    float c = std::tan(heading) * pose.x - pose.y;
    float x = std::fabs(a * lookahead.x + lookahead.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(lookahead.x - pose.x, lookahead.y - pose.y);

                       
    return side * ((2 * x) / (d * d));
}


void evolib::Robot::follow(const asset& path, float lookahead, int timeout, bool forwards, bool async) {
    this->requestMotionStart();
                                  
    if (!this->motionRunning) return;
                                                     
    if (async) {
        pros::Task task([&]() { follow(path, lookahead, timeout, forwards, false); });
        this->endMotion();
        pros::delay(10);                                        
        return;
    }

    std::vector<evolib::Pose> pathPoints = getData(path);                           
    if (pathPoints.size() == 0) {
                                                                            
        distTraveled = -1;
                              
        this->endMotion();
        return;
    }
    Pose pose = this->getPose(true);
    Pose lastPose = pose;
    Pose lookaheadPose(0, 0, 0);
    Pose lastLookahead = pathPoints.at(0);
    lastLookahead.theta = 0;
    float curvature;
    float targetVel;
    float prevLeftVel = 0;
    float prevRightVel = 0;
    int closestPoint;
    float leftInput = 0;
    float rightInput = 0;
    float prevVel = 0;
    int compState = pros::competition::get_status();
    distTraveled = 0;

                                                       
    for (int i = 0; i < timeout / 10 && pros::competition::get_status() == compState && this->motionRunning; i++) {
                                                
        pose = this->getPose(true);
        if (!forwards) pose.theta -= M_PI;

                                 
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

                                                          
        closestPoint = findClosest(pose, pathPoints);
                                                            
        if (pathPoints.at(closestPoint).theta == 0) break;

                                   
        lookaheadPose = lookaheadPoint(lastLookahead, pose, pathPoints, closestPoint, lookahead);
        lastLookahead = lookaheadPose;                                  

                                                                                 
        float curvatureHeading = M_PI / 2 - pose.theta;
        curvature = findLookaheadCurvature(pose, curvatureHeading, lookaheadPose);

                                               
        targetVel = pathPoints.at(closestPoint).theta;
        targetVel = slew(targetVel, prevVel, lateralSettings.slew);
        prevVel = targetVel;

                                                     
        float targetLeftVel = targetVel * (2 + curvature * 10.75) / 2;
        float targetRightVel = targetVel * (2 - curvature * 10.75) / 2;

                                                    
        float ratio = std::max(std::fabs(targetLeftVel), std::fabs(targetRightVel)) / 127;
        if (ratio > 1) {
            targetLeftVel /= ratio;
            targetRightVel /= ratio;
        }

                                     
        prevLeftVel = targetLeftVel;
        prevRightVel = targetRightVel;

                      
        if (forwards) {
            dt.leftMotors->move(targetLeftVel);
            dt.rightMotors->move(targetRightVel);
        } else {
            dt.leftMotors->move(-targetRightVel);
            dt.rightMotors->move(-targetLeftVel);
        }

        pros::delay(10);
    }

                     
    dt.leftMotors->move(0);
    dt.rightMotors->move(0);
                                                                        
    distTraveled = -1;
                          
    this->endMotion();
}