/**
 * @file tilt_motor_control_stepper.c
 * @author Andrew K. Walker
 * @date 26 JUN 2017
 * @brief Stepper control of rotating LIDAR unit.
 *
 * Handles step timing, flag sensor for homing, and high level state machine.
 * Lower level motor control is handled in TMC260.c.
 */
#include "stm32f4xx_conf.h"
#include "tilt_stepper_motor_control.h"

#include "debug.h"

#include "full_duplex_usart_dma.h"

#include "generic_packet.h"
#include "gp_proj_motor.h"

#include "TMC260.h"

/* Note that this can only bee included one time from one file. */
#include "tilt_stepper_motor_profile.h"

uint32_t ts_state_timer = 0;
tilt_stepper_states ts_state = TILT_STEPPER_INITIALIZE;

tmc260_status_struct stat_struct;

uint8_t last_dir = 0;

volatile uint32_t tilt_index = 0;

GenericPacket gp_pos_rad;
float pos_rad = 0.0f;

/* Private functions. */
void tilt_stepper_motor_init_state_machine(void);
void tilt_stepper_motor_init_step_timer(void);
void tilt_stepper_motor_state_change(tilt_stepper_states new_state, uint8_t reset_timer);

/* Public function.  Doxygen documentation is in the header file. */
void tilt_stepper_motor_init(void)
{

   tilt_stepper_motor_init_state_machine();
   tilt_stepper_motor_init_step_timer();
   /* Set initial state and stuch... */


}


/**
 * @fn void tilt_stepper_motor_init_state_machine(void)
 * @brief Initialize the state machine timer
 * @param None
 * @return None
 *
 */
void tilt_stepper_motor_init_state_machine(void)
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;

   uint32_t TimerPeriod = 0;
   uint16_t pscale = 0;

   /* Turn the timer clock on! */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);

   /* TIM11 on APB2 runs at SystemCoreClock. */
   pscale = 1;
   TimerPeriod = (SystemCoreClock / (TILT_STEPPER_STATE_MACHINE_HZ * (pscale+1))) - 1;

   /* Time Base configuration */
   TIM_TimeBaseStructure.TIM_Prescaler = pscale;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

   TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

   /* Set up interrupt. */
   NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM11_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   TIM_ITConfig(TIM11, TIM_IT_Update, ENABLE);

   TIM_Cmd(TIM11, ENABLE);

}

/**
 * @fn void tilt_stepper_motor_init_step_timer(void)
 * @brief Initialize the step timer.
 * @param None
 * @return None
 *
 * This function will reload the ARR value each time such that the next step
 * will be taken at the appropriate time.  It will do nothing if we aren't
 * in a moving state.
 */
void tilt_stepper_motor_init_step_timer(void)
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;

   uint32_t TimerPeriod = 0;
   uint16_t pscale = 0;

   /* Turn the timer clock on! */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

   /* Start with a 1ms timer for the state machine.  TIM2 is a 32bit counter!
    * And APB1 is counting at 84 MHz...SystemCoreClock is 168 MHz so the
    * factor of 2 in the denominator. Assumes the Prescaler is 0 and  that
    * TimerPeriod wouldn't roll a 32 bit number.
    */
   pscale = 0;
   TimerPeriod = (SystemCoreClock / (DEFAULT_STEP_FREQ_HZ * 2 * (pscale + 1))) - 1;

   /* Time Base configuration */
   TIM_TimeBaseStructure.TIM_Prescaler = pscale;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

   TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

   /* Set up interrupt. */
   NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

   TIM_Cmd(TIM5, ENABLE);
}

/**
 * @fn void TIM5_IRQHandler(void)
 * @brief Tilt stepper step timer.
 * @param None
 * @return None
 *
 * This function will reload the ARR value each time such that the next step
 * will be taken at the appropriate time.  It will do nothing if we aren't
 * in a moving state.
 */
void TIM5_IRQHandler(void)
{
   if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
   {

      debug_output_toggle(DEBUG_LED_GREEN);

      /* if(rotating)
       *   {step through the table...flip dir at end...keep stepping until state changes...}
       * else if(homing)
       *   {move until we see the flag...dir determined by current state of flag}
       * else
       *   {do nothing}
       */

      if((ts_state == TILT_STEPPER_TEST_CW)||(ts_state == TILT_STEPPER_TEST_CCW))
      {
         TMC260_step();
      }

      TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
   }
}


/**
 * @fn void TIM1_TRG_COM_TIM11_IRQHandler(void)
 * @brief Tilt stepper motor state machine interrupt handler.
 * @param None
 * @return None
 *
 */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
   if(TIM_GetITStatus(TIM11, TIM_IT_Update) != RESET)
   {
      debug_output_toggle(DEBUG_LED_BLUE);

      ts_state_timer++;

      switch(ts_state)
      {
         case TILT_STEPPER_INITIALIZE:
            /* debug_output_set(DEBUG_LED_RED); */
            TMC260_initialize();

            if(last_dir)
            {
               last_dir = 0;
               tilt_stepper_motor_state_change(TILT_STEPPER_TEST_CW, 1);
            }
            else
            {
               last_dir = 1;
               tilt_stepper_motor_state_change(TILT_STEPPER_TEST_CCW, 1);
            }

            break;
         case TILT_STEPPER_HOME:
            break;
         case TILT_STEPPER_TILT_TABLE:
            break;
         case TILT_STEPPER_TEST_CW:
            if(ts_state_timer == 1)
            {
               /* debug_output_clear(DEBUG_LED_RED); */
               TMC260_dir_CW();
               TMC260_enable();
            }

            pos_rad += 0.005f;
            if(pos_rad >= 3.14f)
            {
               pos_rad = 3.14f;
            }
            /**
             * @todo Need to add the callback for the outgoing queue.
             */
            create_motor_resp_position(&gp_pos_rad, pos_rad);
            full_duplex_usart_dma_add_to_queue(&gp_pos_rad, NULL, 0);

            if(ts_state_timer > 1000)
            {
               TMC260_disable();
               tilt_stepper_motor_state_change(TILT_STEPPER_TEST_DELAY, 1);
            }
            break;
         case TILT_STEPPER_TEST_CCW:
            if(ts_state_timer == 1)
            {
               TMC260_dir_CCW();
               TMC260_enable();
            }

            pos_rad -= 0.005f;
            if(pos_rad < 0.0f)
            {
               pos_rad = 0.0f;
            }

            /**
             * @todo Need to add the callback for the outgoing queue.
             */
            create_motor_resp_position(&gp_pos_rad, pos_rad);
            full_duplex_usart_dma_add_to_queue(&gp_pos_rad, NULL, 0);

            if(ts_state_timer > 1000)
            {
               TMC260_disable();
               tilt_stepper_motor_state_change(TILT_STEPPER_TEST_DELAY, 1);
            }

            break;
         case TILT_STEPPER_TEST_DELAY:
            if(ts_state_timer == 1)
            {
               TMC260_status(&stat_struct, 1);
            }

            if(ts_state_timer > 1000)
            {
               tilt_stepper_motor_state_change(TILT_STEPPER_INITIALIZE, 1);
            }
            break;
         case TILT_STEPPER_ERROR:
            break;
         default:
            break;
      }

      TIM_ClearITPendingBit(TIM11, TIM_IT_Update);
   }
}


/**
 * @fn void tilt_stepper_motor_state_change(tilt_stepper_states new_state, uint8_t reset_timer)
 * @brief Used to change the state of the tilt stepper motor control.
 * @param tilt_stepper_states new_state -> Must be of type tilt_stepper_states.
 * @param uint8_t reset_timer -> Anything other than 0 will result in the timer being reset.
 * @return None
 *
 */
void tilt_stepper_motor_state_change(tilt_stepper_states new_state, uint8_t reset_timer)
{

   if(reset_timer)
   {
      ts_state_timer = 0;
   }

   ts_state = new_state;

}
