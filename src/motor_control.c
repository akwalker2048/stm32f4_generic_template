#include "motor_control.h"

uint8_t motor_control_init_pid(motor_control_pid_t *mcp)
{

   mcp->p = 40.00f;
   mcp->pid_limits.p_upper_limit = 1.00f;
   mcp->pid_limits.p_lower_limit = -1.00f;

   mcp->i = 0.020f;
   mcp->pid_limits.i_upper_limit = 1.00f;
   mcp->pid_limits.i_lower_limit = -1.00f;

   mcp->d = 0.025f;
   mcp->pid_limits.d_upper_limit = 1.00f;
   mcp->pid_limits.d_lower_limit = -1.00f;

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
   static float prev_err;
   static float deriv_err_avg = 0.0f;

   prev_err = mcp->err;

   pout = (mcp->cmd - mcp->msr) * mcp->p;
   mcp->err = pout;

   if(mcp->d > 0.0001f)
   {
      dout = ((mcp->err - prev_err) / mcp->dt) * mcp->d;
      deriv_err_avg = (dout + (deriv_err_avg*MOVING_DERIVATIVE_AVG_PTS)) / (MOVING_DERIVATIVE_AVG_PTS + 1.0f);
      if(deriv_err_avg >= mcp->pid_limits.d_upper_limit)
      {
         deriv_err_avg = mcp->pid_limits.d_upper_limit;
      }
      if(deriv_err_avg <= mcp->pid_limits.d_lower_limit)
      {
         deriv_err_avg = mcp->pid_limits.d_lower_limit;
      }
   }
   else
   {
      deriv_err_avg = 0.0f;
   }
   mcp->derr = deriv_err_avg;

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

   mcp->out = pout + iout + deriv_err_avg;
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


uint8_t motor_control_set_pid_gains(motor_control_pid_t *mcp, float p, float i, float d)
{
   /* Need any bounds checking here? */
   mcp->p = p;
   mcp->i = i;
   mcp->d = d;

   return 0;
}
