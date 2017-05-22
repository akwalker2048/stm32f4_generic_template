#ifndef TILT_MOTOR_CONTROL_H
#define TILT_MOTOR_CONTROL_H

#define TILT_TWO_PI 6.28318530718f

#define TILT_MOTOR_GEAR_RATIO   (24.0f * 74.0f / 16.0f)
#define TILT_MOTOR_LPR          25
#define TILT_MOTOR_QCPR         (4 * TILT_MOTOR_LPR)

#define TILT_ZERO_POSITION_QC      0x7FFF
#define TILT_CCW_FLAG_POSITION_QC (TILT_ZERO_POSITION + TILT_FLAG_WIDTH)
#define TILT_CW_FLAG_POSITION_QC  (TILT_ZERO_POSITION - TILT_FLAG_WIDTH)

//#define TILT_HALF_ANGLE_RAD        1.658f
#define TILT_HALF_ANGLE_RAD (TILT_TWO_PI / 4.0f)
#define TILT_MIN_ANGLE_RAD         (-TILT_HALF_ANGLE_RAD)
#define TILT_MAX_ANGLE_RAD         (TILT_HALF_ANGLE_RAD)

typedef enum {TILT_INITIALIZE, TILT_FIND_HOME, TILT_CW, TILT_CCW, TILT_HORIZONTAL_HOLD, TILT_DISABLE, TILT_TABLE, TILT_ERROR} tilt_motor_states;

typedef enum {TILT_HOME_INITIALIZE, TILT_HOME_MOVE_CW, TILT_HOME_MOVE_CCW} tilt_home_states;

void tilt_motor_init(void);
void tilt_motor_init_flag(void);
void tilt_motor_init_state_machine(void);
void tilt_motor_get_angle(float *tilt_angle);
void tilt_motor_angle_to_counts(float tilt_angle_rad, uint32_t *quad_counts);
uint8_t tilt_motor_set_pid_gains(float p, float i, float d);
uint8_t tilt_motor_query_pid_gains(float *p, float *i, float *d);
uint8_t tilt_motor_start(void);
uint8_t tilt_motor_stop(void);

#endif
