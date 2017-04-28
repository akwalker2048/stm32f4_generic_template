#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

typedef enum {MOTOR_CONTROL_POSITION, MOTOR_CONTROL_VELOCITY} motor_control_variable_e;

typedef struct {
   motor_control_variable_e control_var;
   float p;
   float i;
   float d;
   float pos;
   float vel;
   float cmd;
   float err;
   float ierr;
} motor_control_pid_t;

uint8_t motor_control_run_pid(motor_control_pid_t mcp, float actual_);

#endif
