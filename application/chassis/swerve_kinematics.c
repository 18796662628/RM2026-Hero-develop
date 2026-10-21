#include "swerve_kinematics.h"

#include <math.h>

#define SWERVE_PI 3.14159265358979323846f
#define SWERVE_TWO_PI (2.0f * SWERVE_PI)
#define SWERVE_HALF_PI (0.5f * SWERVE_PI)

float SwerveWrapAngle(float angle_rad)
{
    while (angle_rad > SWERVE_PI)
        angle_rad -= SWERVE_TWO_PI;
    while (angle_rad <= -SWERVE_PI)
        angle_rad += SWERVE_TWO_PI;
    return angle_rad;
}

void SwerveCalculate(const Swerve_Module_Position_s positions[4],
                     float vx,
                     float vy,
                     float wz,
                     Swerve_Module_State_s states[4])
{
    for (uint8_t index = 0U; index < 4U; index++)
    {
        const float wheel_vx = vx - wz * positions[index].y;
        const float wheel_vy = vy + wz * positions[index].x;

        states[index].steering_angle_rad = atan2f(wheel_vy, wheel_vx);
        states[index].drive_speed = sqrtf(wheel_vx * wheel_vx + wheel_vy * wheel_vy);
    }
}

void SwerveOptimizeState(Swerve_Module_State_s *target, float current_angle_rad)
{
    float error = SwerveWrapAngle(target->steering_angle_rad - current_angle_rad);

    if (error > SWERVE_HALF_PI)
    {
        target->steering_angle_rad = SwerveWrapAngle(target->steering_angle_rad - SWERVE_PI);
        target->drive_speed = -target->drive_speed;
    }
    else if (error < -SWERVE_HALF_PI)
    {
        target->steering_angle_rad = SwerveWrapAngle(target->steering_angle_rad + SWERVE_PI);
        target->drive_speed = -target->drive_speed;
    }
}
