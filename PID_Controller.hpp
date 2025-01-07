#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ctime>

class PID_Controller {
public:

    PID_Controller(float sp) : _setpoint(sp), _controller(0) {};

    float calculateController(float processVar);

private:
    float _setpoint; // input
    float _controller; // output

    // gain factors
    constexpr float _Kp = 1.5;
    constexpr float _Ki = 0.5;
    constexpr float _Kd = 0.5;

    // stuff for integrals
    float _totalError = 0;
    float _prevError = 0;
    std::size_t _prevTime = 0;
}
#endif // PID_CONTROLLER_H
