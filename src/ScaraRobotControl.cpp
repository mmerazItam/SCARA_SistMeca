#include "ScaraRobotControl.h"

#include <cmath>
#include <cstdio>
#include <cstdint>

extern void setMotorTarget(uint8_t motor, float target);
extern void setBldcTarget(uint8_t motor, float reference);

ScaraRobotControl scara_robot;

ScaraRobotControl::ScaraRobotControl()
{
    _calibration.theta1_direction = 1.0f;
    _calibration.theta2_direction = 1.0f;
    _calibration.theta3_direction = 1.0f;
    _calibration.z_direction = 1.0f;
    _calibration.theta1_offset_deg = 0.0f;
    _calibration.theta2_offset_deg = 0.0f;
    _calibration.theta3_offset_deg = 0.0f;
    _calibration.z_offset_units = 0.0f;
    _calibration.z_units_per_mm = 100.0f;
}

void ScaraRobotControl::setup(float link1_mm, float link2_mm)
{
    _kinematics.setup(link1_mm, link2_mm);
}

void ScaraRobotControl::setCalibration(const ScaraMotorCalibration &calibration)
{
    _calibration = calibration;
}

ScaraMotorCalibration ScaraRobotControl::getCalibration() const
{
    return _calibration;
}

ScaraMotorTargets ScaraRobotControl::solve(float x_mm,
                                           float y_mm,
                                           float z_mm,
                                           float phi_deg,
                                           ScaraElbow elbow) const
{
    ScaraMotorTargets targets = {};
    targets.solution = _kinematics.inverse(x_mm, y_mm, z_mm, phi_deg, elbow);
    targets.reachable = targets.solution.reachable;

    if (!targets.reachable)
    {
        printf("[SCARA] unreachable x=%.2f y=%.2f z=%.2f phi=%.2f elbow=%d\n",
               x_mm, y_mm, z_mm, phi_deg, elbow);
        return targets;
    }

    const ScaraJointAngles &joints = targets.solution.angles;

    targets.stepper_target[0] = _calibration.theta1_offset_deg +
                                _calibration.theta1_direction * joints.theta1_deg;
    targets.stepper_target[1] = _calibration.theta2_offset_deg +
                                _calibration.theta2_direction * joints.theta2_deg;
    targets.bldc_target = normalizeAngle(_calibration.theta3_offset_deg +
                                         _calibration.theta3_direction * joints.theta3_deg);
    targets.stepper_target[2] = _calibration.z_offset_units +
                                _calibration.z_direction * joints.z_mm * _calibration.z_units_per_mm;

    printf("[SCARA] pose x=%.2f y=%.2f z=%.2f phi=%.2f elbow=%d\n",
           x_mm, y_mm, z_mm, phi_deg, elbow);
    printf("[SCARA] joints theta1=%.2f theta2=%.2f theta3=%.2f z=%.2f\n",
           joints.theta1_deg, joints.theta2_deg, joints.theta3_deg, joints.z_mm);
    printf("[SCARA] targets step1=%.2f step2=%.2f bldc=%.2f step3=%.2f\n",
           targets.stepper_target[0], targets.stepper_target[1],
           targets.bldc_target, targets.stepper_target[2]);

    return targets;
}

bool ScaraRobotControl::apply(float x_mm,
                              float y_mm,
                              float z_mm,
                              float phi_deg,
                              ScaraElbow elbow) const
{
    ScaraMotorTargets targets = solve(x_mm, y_mm, z_mm, phi_deg, elbow);
    if (!targets.reachable)
    {
        printf("[SCARA] apply skipped\n");
        return false;
    }

    setMotorTarget(0, targets.stepper_target[0]);
    setMotorTarget(1, targets.stepper_target[1]);
    setBldcTarget(0, targets.bldc_target);
    setMotorTarget(2, targets.stepper_target[2]);
    printf("[SCARA] apply ok\n");
    return true;
}

float ScaraRobotControl::normalizeAngle(float angle_deg)
{
    float angle = std::fmod(angle_deg, 360.0f);
    if (angle < 0.0f)
        angle += 360.0f;
    return angle;
}

bool setScaraTarget(float x_mm,
                    float y_mm,
                    float z_mm,
                    float phi_deg,
                    ScaraElbow elbow)
{
    return scara_robot.apply(x_mm, y_mm, z_mm, phi_deg, elbow);
}
