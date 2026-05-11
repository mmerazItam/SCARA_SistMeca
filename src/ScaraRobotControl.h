#ifndef SCARA_ROBOT_CONTROL_H
#define SCARA_ROBOT_CONTROL_H

#include <ScaraKinematics.h>

struct ScaraMotorCalibration
{
    float theta1_direction;
    float theta2_direction;
    float z_direction;
    float theta4_direction;
    float theta1_offset_deg;
    float theta2_offset_deg;
    float z_offset_units;
    float z_units_per_mm;
    float theta4_offset_deg;
};

struct ScaraMotorTargets
{
    bool reachable;
    float stepper_target[3];
    float bldc_target;
    ScaraSolution solution;
};

class ScaraRobotControl
{
public:
    ScaraRobotControl();

    void setup(float link1_mm, float link2_mm);
    void setCalibration(const ScaraMotorCalibration &calibration);
    ScaraMotorCalibration getCalibration() const;

    ScaraMotorTargets solve(float x_mm,
                            float y_mm,
                            float z_mm,
                            float phi_deg,
                            ScaraElbow elbow = SCARA_ELBOW_DOWN) const;

    bool apply(float x_mm,
               float y_mm,
               float z_mm,
               float phi_deg,
               ScaraElbow elbow = SCARA_ELBOW_DOWN) const;

private:
    static float normalizeAngle(float angle_deg);

    ScaraKinematics _kinematics;
    ScaraMotorCalibration _calibration;
};

extern ScaraRobotControl scara_robot;

bool setScaraTarget(float x_mm,
                    float y_mm,
                    float z_mm,
                    float phi_deg,
                    ScaraElbow elbow = SCARA_ELBOW_DOWN);

#endif
