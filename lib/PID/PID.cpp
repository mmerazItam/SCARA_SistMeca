#include "PID.h"

static inline float clampPID(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

PID::PID()
{
}

void PID::setup(float gains[3], float dt, float saturation)
{
    _kp = gains[0];
    _ki = gains[1];
    _kd = gains[2];
    _dt = dt;
    _saturation = saturation;
    _prev_error = 0.0f;
    _integral = 0.0f;
}

float PID::calculate(float error)
{
    if (_dt <= 0.0f)
        return clampPID(_kp * error, -_saturation, _saturation);

    _integral += _dt * (error + _prev_error) / 2.0f;
    float u = _kp * error;
    u += _ki * _integral;
    u += _kd * (error - _prev_error) / _dt;
    _prev_error = error;
    return clampPID(u, -_saturation, _saturation);
}
