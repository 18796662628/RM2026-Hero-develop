#ifndef SENTINEL_SWERVE_KINEMATICS_H
#define SENTINEL_SWERVE_KINEMATICS_H

#include <stdint.h>

/* Chassis frame: x forward, y left, positive yaw rate counter-clockwise. */
typedef struct
{
    float x;
    float y;
} Swerve_Module_Position_s;

typedef struct
{
    float steering_angle_rad;
    float drive_speed;
} Swerve_Module_State_s;

/*
 * Converts chassis velocity to each wheel's steering angle and rolling speed.
 * Positions and vx/vy/wz must use one coherent unit system, normally m, m/s,
 * and rad/s. Wheel ordering is owned by the caller.
 */
void SwerveCalculate(const Swerve_Module_Position_s positions[4],
                     float vx,
                     float vy,
                     float wz,
                     Swerve_Module_State_s states[4]);

/* Select the equivalent wheel angle requiring at most a quarter turn. */
void SwerveOptimizeState(Swerve_Module_State_s *target, float current_angle_rad);

float SwerveWrapAngle(float angle_rad);

#endif
