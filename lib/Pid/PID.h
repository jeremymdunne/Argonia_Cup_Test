#ifndef _PID_H
#define _PID_H
#include <Math.h>

struct PID_SETTINGS {
  float kP = 0;
  float kI = 0;
  float kD = 0;
  float maxI = 0;
  float maxError = 0;
  long interval = 0;
};

class PID{


public:
  PID(float kP, float kI, float kD, long sampleTime);
  PID(float kP, float kI, float kD, float maxI, float maxError, long sampleTime);
  PID(PID_SETTINGS settings);
  float calculate(float currentValue);
  void setSP(float setPoint);
private:
  long millisInterval;
  float lastValue;
  long lastUpdateTime = 0;
  float _kP, _kI, _kD,_SP;
  float _max_I = 0, _max_error = 0;
  float error = 0, pError = 0, iError = 0, dError = 0, totalError = 0;
  float prev_error = 0; 
};



#endif
