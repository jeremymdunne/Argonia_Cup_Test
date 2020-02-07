#include "PID.h"
#include <Arduino.h>


PID::PID(float kP, float kI, float kD, long sampleTime){
  _kP = kP;
  _kI = kI;
  _kD = kD;
  millisInterval = sampleTime;
}

PID::PID(float kP, float kI, float kD, float maxI, float maxError, long sampleTime){
  _kP = kP;
  _kI = kI;
  _kD = kD;
  _max_I = maxI;
  _max_error = maxError;
  millisInterval = sampleTime;
}

PID::PID(PID_SETTINGS settings){
  _kP = settings.kP;
  _kI = settings.kI;
  _kD = settings.kD;
  _max_I = settings.maxI;
  _max_error = settings.maxError;
}

void PID::setSP(float setPoint){
  _SP = setPoint;
}

float PID::calculate(float currentValue){
  long currentTime = millis();
  long elapsedMillis = currentTime - lastUpdateTime;
  if(millisInterval == 0 || elapsedMillis > millisInterval){
    error = _SP - currentValue;
    if(_kP != 0){
      pError =  error* _kP;
      Serial.println("P-Comp: " + String(pError));
    }
    if(_kI != 0){
      iError += error * _kI * elapsedMillis/1000.0;
      if(_max_I != 0)
      if(fabs(iError) > _max_I) iError = iError/fabs(iError) * _max_I;
      Serial.println("I-Comp: " + String(iError));
    }
    if(_kD != 0){
      dError = _kD * (prev_error  - error)/(elapsedMillis/1000.0);
      prev_error = error; 
    }
    totalError = pError + iError + dError;
    if(_max_error != 0)
    if(fabs(totalError) > _max_error) totalError = totalError/fabs(totalError) * _max_error;
    lastUpdateTime = currentTime;
    lastValue = totalError;
    return totalError;
  }
  return lastValue;
}
