#define Kp 1.0f
#define Ki 0.5f
#define Kd 100.0f

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <chrono>

class PID_Controller {
public:

    PID_Controller(float setpoint) : _setpoint(setpoint), _controller(0) {};

    float calculateController(const float processVar, const std::size_t time);


private:
    float _setpoint; // input
    float _controller; // output


    // stuff for integrals
    float _total_error = 0;
    float _prev_error = 0;
    std::size_t _prev_time = 0;
};
#endif // PID_CONTROLLER_H
