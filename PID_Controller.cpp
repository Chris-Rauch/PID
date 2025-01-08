#include "PID_Controller.hpp"
#include <iostream>

using namespace std;
float PID_Controller::calculateController(float processVar, std::size_t time_now) 
{
    // calculate P (the magnitude of current error)
    float error = _setpoint - processVar;
    float P = Kp * error;

    // if prevTime is 0 then this is the first iteration
    // if this is the first iteration, return without calculating I or D
    if(_prev_time == 0)
    {
        _prev_time = time_now;
        return P;
    }

    // protect against divide by 0
    float delta_t = time_now - _prev_time;
    _prev_time = time_now;
    if(delta_t == 0)
    {
        delta_t = 0.001;
    }

    // calculate I (sum of all error so far)
    float integratedError = ((error + _prev_error) / 2) * delta_t;
    float I = Ki * (_total_error + integratedError);

    // calculate D (the predicted future error)
    //cout << "Error: " << error << " Previous Error: " << _prev_error << endl;
    //cout << "Delta t: " << delta_t << endl;
    float derivedError = ((error - _prev_error) / delta_t);
    _prev_error = error;
    float D = Kd * derivedError * (-1);

    //cout << "P: " << P << " I: " << I << " D: " << D << endl;   
    return (P + I + D);
}

