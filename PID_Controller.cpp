#include "PID_Controller.hpp"

float PID_Controller::calculateController(float processVar, std::time_t time) 
{
    // calculate P (the magnitude of current error)
    float error = _setpoint - processVar;

    // calculate I (the sum of past error)
    if(_prevTime == 0)
    {
        auto now = std::chrono::system_clock::now().time_since_epoch();
        _prevTime = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
    }
    float integratedError = ((error + _prevError) / 2) * (time - _prevTime);
    _totError += integratedError;

    // calculate D (the predicted future change)
    float derivedError = ((error - _prevError) / (time - _prevTime));    

    float P = _Kp * error;
    float I = _Ki * _totError;
    float D = _Kd * derivedError * (-1);

    return (P + I + D);
}

