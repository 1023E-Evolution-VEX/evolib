#pragma once

#include <cmath>
#include "motionProfiling/motionProfile.h"
#include "units/units.hpp"
#include "utils/utils.h"
#include "evolib/robot.hpp"
#include "evolib/timer.hpp"
#include "evolib/util.hpp"
#include "pros/misc.hpp"

   
                                        
   
class Ramsete {
private:
    DrivetrainSubsystem *drivetrain;

    float zeta;
    float beta;

    QTime startTime;

    MotionProfile *motionProfile;

    QVelocity lastLeft = 0.0, lastRight = 0.0;

    DriveSpeeds lastSpeeds{0.0, 0.0};

public:
    Ramsete(MotionProfile *motion_profile, const float zeta = CONFIG::RAMSETE_ZETA,
            const float beta = CONFIG::RAMSETE_BETA);

    void initialize();

    void execute();

    void end(bool interrupted);

    bool isFinished();

                                                  
};