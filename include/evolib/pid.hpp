#pragma once

namespace evolib {
class PID {
public:
  PID(float kP, float kI, float kD, float windupRange, bool signFlipReset);
  float update(const float error);
  void reset();

protected:
          
  const float kP;
  const float kI;
  const float kD;

                  
  const float windupRange;
  const bool signFlipReset;

  float integral = 0;
  float prevError = 0;
};
};                    