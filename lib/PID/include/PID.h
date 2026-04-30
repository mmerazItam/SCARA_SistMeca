#ifndef __PID_H__
#define __PID_H__

class PID
{
public:
    PID();
    void setup(float gains[3], float dt, float saturation);
    float calculate(float error);

private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;
    float _dt = 0.0f;
    float _saturation = 100.0f;
    float _prev_error = 0.0f;
    float _integral = 0.0f;
};

#endif
