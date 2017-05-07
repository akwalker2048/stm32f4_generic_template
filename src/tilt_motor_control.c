#include "stm32f4xx_conf.h"
#include "tilt_motor_control.h"
#include "quad_encoder.h"

void tilt_motor_init(void)
{

}


void tilt_motor_init_state_machine(void)
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

   uint32_t TimerPeriod = 0;

   /* Turn the timer clock on! */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

   /* Start with a 1ms timer for the state machine. */
   TimerPeriod = (SystemCoreClock / 1000) - 1;

   /* Time Base configuration */
   TIM_TimeBaseStructure.TIM_Prescaler = 0;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

   TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);


}

void tilt_motor_init_flag(void)
{

   EXTI_InitTypeDef   EXTI_InitStructure;
   GPIO_InitTypeDef   GPIO_InitStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;

   /* Enable GPIOA clock */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
   /* Enable SYSCFG clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

   /* Configure PA0 pin as input floating */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   /* Connect EXTI Line15 to PA15 pin */
   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource15);

   /* Configure EXTI Line15 */
   EXTI_InitStructure.EXTI_Line = EXTI_Line15;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   /* Enable and set EXTI Line0 Interrupt to the lowest priority */
   NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

}

/* Tilt Flag Interrupt Handler */
void EXTI15_10_IRQHandler(void)
{

  if(EXTI_GetITStatus(EXTI_Line15) != RESET)
  {

     /* Do this for now...but need to be smarter later! */
     quad_encoder_set_position(TILT_ZERO_POSITION_QC);
     /* Algorithm should be something like this:
        if(direction == CCW)
            quad_encoder_set_position(TILT_CCW_FLAG_POSITION)
        else
            quad_encoder_set_position(TILT_CW_FLAG_POSITION)
     */

     /* Clear the EXTI line 15 pending bit */
     EXTI_ClearITPendingBit(EXTI_Line15);
  }

}


void tilt_motor_get_angle(float *tilt_angle_rad)
{
   uint32_t quad_counts;

   quad_encoder_read_position(&quad_counts);

   *tilt_angle_rad = (((float)quad_counts - (float)TILT_ZERO_POSITION_QC) * TILT_TWO_PI) / ((float)TILT_MOTOR_QCPR * TILT_MOTOR_GEAR_RATIO);

}
