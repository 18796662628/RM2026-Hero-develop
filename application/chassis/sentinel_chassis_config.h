#ifndef SENTINEL_CHASSIS_CONFIG_H
#define SENTINEL_CHASSIS_CONFIG_H

/*
 * Chassis +x is the CAN2 wheel-pair side. +y is left while facing +x.
 * The chassis is square; all dimensions below are SI units.
 */
#define SENTINEL_WHEEL_MODULE_SPACING_M 0.342f
#define SENTINEL_HALF_MODULE_SPACING_M (SENTINEL_WHEEL_MODULE_SPACING_M * 0.5f)
#define SENTINEL_WHEEL_RADIUS_M 0.050f

/* M3508 output shaft -> wheel axle: 17-tooth pinion drives 268-tooth gear. */
#define SENTINEL_DRIVE_OUTPUT_TO_WHEEL_RATIO (268.0f / 17.0f)

/* First-drive limits: deliberately low until directions and zero positions are verified. */
#define SENTINEL_TEST_MAX_LINEAR_SPEED_MPS 0.10f
#define SENTINEL_STATIONARY_SPEED_MPS 0.005f

/* Update these signs after the one-wheel low-speed direction check. */
#define SENTINEL_LF_STEER_DIRECTION 1.0f
#define SENTINEL_RF_STEER_DIRECTION 1.0f
#define SENTINEL_RB_STEER_DIRECTION 1.0f
#define SENTINEL_LB_STEER_DIRECTION 1.0f

#define SENTINEL_LF_DRIVE_DIRECTION 1.0f
#define SENTINEL_RF_DRIVE_DIRECTION 1.0f
#define SENTINEL_RB_DRIVE_DIRECTION 1.0f
#define SENTINEL_LB_DRIVE_DIRECTION 1.0f

#endif
