#ifndef SCARA_KINEMATICS_H
#define SCARA_KINEMATICS_H

#include <cstdint>

enum ScaraElbow
{
    SCARA_ELBOW_DOWN = 0,
    SCARA_ELBOW_UP = 1,
};

struct ScaraPose
{
    float x_mm;
    float y_mm;
    float z_mm;
    float phi_deg;
};

struct ScaraJointAngles
{
    float theta1_deg;
    float theta2_deg;
    float theta3_deg;
    float z_mm;
};

struct ScaraJointSteps
{
    int32_t theta1_steps;
    int32_t theta2_steps;
    int32_t theta3_steps;
    int32_t z_steps;
};

struct ScaraSolution
{
    bool reachable;
    ScaraJointAngles angles;
    ScaraJointSteps steps;
};

class ScaraKinematics
{
public:
    ScaraKinematics();
    ScaraKinematics(float link1_mm, float link2_mm);

    void setup(float link1_mm, float link2_mm);
    void setStepScale(float theta1_steps_per_deg,
                      float theta2_steps_per_deg,
                      float theta3_steps_per_deg,
                      float z_steps_per_mm);

    ScaraSolution inverse(float x_mm,
                          float y_mm,
                          float z_mm,
                          float phi_deg,
                          ScaraElbow elbow = SCARA_ELBOW_DOWN) const;
    ScaraSolution inverse(const ScaraPose &pose,
                          ScaraElbow elbow = SCARA_ELBOW_DOWN) const;

    ScaraJointSteps anglesToSteps(const ScaraJointAngles &angles) const;

private:
    float _link1_mm;
    float _link2_mm;
    float _theta1_steps_per_deg;
    float _theta2_steps_per_deg;
    float _theta3_steps_per_deg;
    float _z_steps_per_mm;
};

#endif
