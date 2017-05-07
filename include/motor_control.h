#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

typedef struct {
   float p_upper_limit;
   float p_lower_limit;
   float i_upper_limit;
   float i_lower_limit;
   float d_upper_limit;
   float d_lower_limit;
} motor_control_pid_limits_t;

typedef struct {
   float p;
   float i;
   float d;
   motor_control_pid_limits_t pid_limits;
   float cmd;
   float msr;
   float dt;
   float err;
   float ierr;
   float out;
} motor_control_pid_t;

uint8_t motor_control_init_pid(motor_control_pid_t *mcp);
uint8_t motor_control_run_pid(motor_control_pid_t *mcp);
uint8_t motor_control_seed_integrator(motor_control_pid_t *mcp, float iseed);

#endif
