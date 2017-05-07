#ifndef TILT_MOTOR_CONTROL_H
#define TILT_MOTOR_CONTROL_H

#define TILT_TWO_PI 6.28318530718f

#define TILT_MOTOR_GEAR_RATIO   30.0f
#define TILT_MOTOR_LPR          16
#define TILT_MOTOR_QCPR         (4 * TILT_MOTOR_LPR)

#define TILT_ZERO_POSITION_QC      0x7FFF
#define TILT_CCW_FLAG_POSITION_QC (TILT_ZERO_POSITION + TILT_FLAG_WIDTH)
#define TILT_CW_FLAG_POSITION_QC  (TILT_ZERO_POSITION - TILT_FLAG_WIDTH)

#define TILT_HALF_ANGLE_RAD        1.57079632679f
#define TILT_MIN_ANGLE_RAD         (-TILT_HALF_ANGLE_RAD)
#define TILT_MAX_ANGLE_RAD         (TILT_HALF_ANGLE_RAD)

void tilt_motor_init_flag(void);

void tilt_motor_get_angle(float *tilt_angle);

#endif
