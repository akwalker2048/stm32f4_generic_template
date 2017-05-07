#include "motor_control.h"

uint8_t motor_control_init_pid(motor_control_pid_t *mcp)
{

   mcp->p = 1.0f;
   mcp->pid_limits.p_upper_limit = 24.0f;
   mcp->pid_limits.p_lower_limit = -24.0f;

   mcp->i = 0.1f;
   mcp->pid_limits.i_upper_limit = 24.0f;
   mcp->pid_limits.i_lower_limit = -24.0f;

   mcp->d = 0.0f;
   mcp->pid_limits.d_upper_limit = 0.0f;
   mcp->pid_limits.d_lower_limit = -0.0f;

   mcp->cmd = 0.0f;
   mcp->msr = 0.0f;
   mcp->dt  = 0.001f;

   mcp->err = 0.0f;
   mcp->ierr = 0.0f;
   mcp->out = 0.0f;

   return 0;

}

uint8_t motor_control_run_pid(motor_control_pid_t *mcp)
{
   float pout, iout, dout;
   float prev_err;

   prev_err = mcp->err;

   mcp->err = mcp->cmd - mcp->msr;
   pout = mcp->err * mcp->p;

   if(mcp->d > 0.0001f)
   {
      dout = ((mcp->err - prev_err) / mcp->dt) * mcp->d;
      if(dout >= mcp->pid_limits.d_upper_limit)
      {
         dout = mcp->pid_limits.d_upper_limit;
      }
      if(dout <= mcp->pid_limits.d_lower_limit)
      {
         dout = mcp->pid_limits.d_lower_limit;
      }
   }
   else
   {
      dout = 0.0f;
   }

   iout = mcp->err * mcp->i + mcp->ierr;
   if(iout >= mcp->pid_limits.i_upper_limit)
   {
      iout = mcp->pid_limits.i_upper_limit;
   }
   if(iout <= mcp->pid_limits.i_lower_limit)
   {
      iout = mcp->pid_limits.i_lower_limit;
   }
   mcp->ierr = iout;

   mcp->out = pout + iout + dout;
   if(mcp->out >= mcp->pid_limits.p_upper_limit)
   {
      mcp->out = mcp->pid_limits.p_upper_limit;
   }
   if(mcp->out <= mcp->pid_limits.p_lower_limit)
   {
      mcp->out = mcp->pid_limits.p_lower_limit;
   }


   return 0;

}


uint8_t motor_control_seed_integrator(motor_control_pid_t *mcp, float iseed)
{

   mcp->ierr = iseed;
   if(mcp->ierr >= mcp->pid_limits.i_upper_limit)
   {
      mcp->ierr = mcp->pid_limits.i_upper_limit;
   }
   if(mcp->ierr <= mcp->pid_limits.i_lower_limit)
   {
      mcp->ierr = mcp->pid_limits.i_lower_limit;
   }

   return 0;

}
