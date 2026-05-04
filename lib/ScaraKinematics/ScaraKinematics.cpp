#include "ScaraKinematics.h"

#include <cmath>

static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kRadToDeg = 180.0f / kPi;

static float clampScara(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static int32_t roundToSteps(float value)
{
    if (value >= 0.0f)
        return (int32_t)(value + 0.5f);
    return (int32_t)(value - 0.5f);
}

ScaraKinematics::ScaraKinematics()
    : _link1_mm(228.0f),
      _link2_mm(136.5f),
      _theta1_steps_per_deg(44.444444f),
      _theta2_steps_per_deg(35.555555f),
      _theta3_steps_per_deg(10.0f),
      _z_steps_per_mm(100.0f)
{
}

ScaraKinematics::ScaraKinematics(float link1_mm, float link2_mm)
    : _link1_mm(228.0f),
      _link2_mm(136.5f),
      _theta1_steps_per_deg(44.444444f),
      _theta2_steps_per_deg(35.555555f),
      _theta3_steps_per_deg(10.0f),
      _z_steps_per_mm(100.0f)
{
    setup(link1_mm, link2_mm);
}

void ScaraKinematics::setup(float link1_mm, float link2_mm)
{
    _link1_mm = link1_mm;
    _link2_mm = link2_mm;
}

void ScaraKinematics::setStepScale(float theta1_steps_per_deg,
                                   float theta2_steps_per_deg,
                                   float theta3_steps_per_deg,
                                   float z_steps_per_mm)
{
    _theta1_steps_per_deg = theta1_steps_per_deg;
    _theta2_steps_per_deg = theta2_steps_per_deg;
    _theta3_steps_per_deg = theta3_steps_per_deg;
    _z_steps_per_mm = z_steps_per_mm;
}

ScaraSolution ScaraKinematics::inverse(float x_mm,
                                       float y_mm,
                                       float z_mm,
                                       float phi_deg,
                                       ScaraElbow elbow) const
{
    ScaraSolution solution = {};

    const float r2 = x_mm * x_mm + y_mm * y_mm;
    const float l1 = _link1_mm;
    const float l2 = _link2_mm;
    const float cos_theta2 = (r2 - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);

    if (cos_theta2 < -1.0f || cos_theta2 > 1.0f)
        return solution;

    float theta2_rad = std::acos(clampScara(cos_theta2, -1.0f, 1.0f));
    if (elbow == SCARA_ELBOW_UP)
        theta2_rad = -theta2_rad;

    const float k1 = l1 + l2 * std::cos(theta2_rad);
    const float k2 = l2 * std::sin(theta2_rad);
    const float theta1_rad = std::atan2(y_mm, x_mm) - std::atan2(k2, k1);

    const float theta1_deg = theta1_rad * kRadToDeg;
    const float theta2_deg = theta2_rad * kRadToDeg;

    solution.reachable = true;
    solution.angles.theta1_deg = theta1_deg;
    solution.angles.theta2_deg = theta2_deg;
    solution.angles.theta3_deg = phi_deg - theta1_deg - theta2_deg;
    solution.angles.z_mm = z_mm;
    solution.steps = anglesToSteps(solution.angles);
    return solution;
}

ScaraSolution ScaraKinematics::inverse(const ScaraPose &pose, ScaraElbow elbow) const
{
    return inverse(pose.x_mm, pose.y_mm, pose.z_mm, pose.phi_deg, elbow);
}

ScaraJointSteps ScaraKinematics::anglesToSteps(const ScaraJointAngles &angles) const
{
    ScaraJointSteps steps = {};
    steps.theta1_steps = roundToSteps(angles.theta1_deg * _theta1_steps_per_deg);
    steps.theta2_steps = roundToSteps(angles.theta2_deg * _theta2_steps_per_deg);
    steps.theta3_steps = roundToSteps(angles.theta3_deg * _theta3_steps_per_deg);
    steps.z_steps = roundToSteps(angles.z_mm * _z_steps_per_mm);
    return steps;
}
