#include "HBridge.h"

static inline float clampHBridge(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

HBridge::HBridge()
{
}

void HBridge::setup(uint8_t *pins, uint8_t *channels, TimerConfig *timer, float start_threshold)
{
    _start_threshold = start_threshold;
    _in1.setup(pins[0], channels[0], timer);
    _in2.setup(pins[1], channels[1], timer);
    setStop(false);
}

void HBridge::setDuty(float pwm)
{
    pwm = clampHBridge(pwm, -100.0f, 100.0f);

    if (pwm == 0.0f)
    {
        setStop(false);
        return;
    }

    float sign = (pwm < 0.0f) ? -1.0f : 1.0f;
    float mag = (pwm < 0.0f) ? -pwm : pwm;

    mag = _start_threshold + mag * (100.0f - _start_threshold) / 100.0f;
    mag = clampHBridge(mag, 0.0f, 100.0f);

    if (sign > 0.0f)
    {
        _in1.setDuty(0.0f);
        _in2.setDuty(mag);
    }
    else
    {
        _in1.setDuty(mag);
        _in2.setDuty(0.0f);
    }
}

void HBridge::setStop(bool brake)
{
    if (brake)
    {
        _in1.setDuty(100.0f);
        _in2.setDuty(100.0f);
    }
    else
    {
        _in1.setDuty(0.0f);
        _in2.setDuty(0.0f);
    }
}
