#ifndef __HBRIDGE_H__
#define __HBRIDGE_H__

#include <cstdint>
#include <SimplePWM.h>

class HBridge
{
public:
    HBridge();
    void setup(uint8_t *pins, uint8_t *channels, TimerConfig *timer, float start_threshold = 10.0f);
    void setDuty(float pwm);
    void setStop(bool brake);

private:
    float _start_threshold = 10.0f;
    SimplePWM _in1;
    SimplePWM _in2;
};

#endif
