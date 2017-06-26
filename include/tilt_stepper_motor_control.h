/**
 * @file tilt_stepper_motor_control.h
 * @author Andrew K. Walker
 * @date 26 JUN 2017
 * @brief Stepper control of rotating LIDAR unit.
 *
 * Handles step timing, flag sensor for homing, and high level state machine.
 * Lower level motor control is handled in TMC260.c.
 */
#ifndef TILT_STEPPER_MOTOR_CONTROL_H
#define TILT_STEPPER_MOTOR_CONTROL_H

#define DEFAULT_STEP_FREQ_HZ  200

#define TILT_STEPPER_STATE_MACHINE_HZ 1000

typedef enum {TILT_STEPPER_INITIALIZE,
              TILT_STEPPER_HOME,
              TILT_STEPPER_TILT_TABLE,
              TILT_STEPPER_ERROR} tilt_stepper_states;



/**
 * @fn void tilt_stepper_motor_init(void)
 * @brief Initialize all of the tilt stepper motor control.
 * @param None
 * @return None
 *
 * This function will reload the ARR value each time such that the next step
 * will be taken at the appropriate time.  It will do nothing if we aren't
 * in a moving state.
 */
void tilt_stepper_motor_init(void);


#endif
