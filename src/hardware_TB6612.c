#include "stm32f4xx_conf.h"
#include "hardware_TB6612.h"

uint8_t TB6612_initialized = 0;
uint16_t TIM1_Period = 0;

void TB6612_initialize(void)
{

   TB6612_init_gpio();
   TB6612_init_complimentary_pwm();
   TB6612_set_duty(0.0f);
   TB6612_enable();
   TB6612_initialized = 1;

}

void TB6612_init_gpio(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;

   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

   /* GPIOA Configuration:
    *   A1 -> PWM
    *   A2 -> Enable
    */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

}

void TB6612_brake(void)
{

   /* Put us in short brake mode. */
   GPIO_ResetBits(GPIOA, TB6612_PWM_PIN);

}

void TB6612_enable(void)
{

   /* Enable the driver! */
   GPIO_SetBits(GPIOA, TB6612_PWM_PIN);
   GPIO_SetBits(GPIOA, TB6612_ENABLE_PIN);

}

void TB6612_disable(void)
{

   /* Turn the driver off! */
   GPIO_ResetBits(GPIOA, TB6612_ENABLE_PIN);

}

void TB6612_set_duty(float duty)
{
   /* duty = from -1.0 to 1.0
    *        -1.0 = full CCW PWM
    *         1.0 = full CW PWM
    */
   uint16_t ccr_val;

   if(duty > 1.0f)
   {
      duty = 1.0f;
   }
   if(duty < -1.0f)
   {
      duty = -1.0f;
   }

   duty = ((duty + 1.0f) / 2.0f);
   ccr_val = (uint16_t)(duty*TIM1_Period + 0.5);

   TIM1->CCR1 = ccr_val;
}

void TB6612_init_complimentary_pwm(void)
{

   GPIO_InitTypeDef GPIO_InitStructure;
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   TIM_OCInitTypeDef  TIM_OCInitStructure;
   TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
   uint16_t TimerPeriod = 0;
   uint16_t Channel1Pulse = 0;

   /* GPIOA and GPIOB clocks enable */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE, ENABLE);

   /* TIM1 clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

   /* GPIOA Configuration: TIM1_CH1 -> A8 */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   /* GPIOE Configuration: TIM1_CH1N -> E8 */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
   GPIO_Init(GPIOE, &GPIO_InitStructure);

   /* Connect TIM pins to AF1 */
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
   GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_TIM1);

   /* Compute the value to be set in ARR register to generate signal frequency at 24.0 Khz */
   TimerPeriod = (SystemCoreClock / 24000) - 1;
   TIM1_Period = TimerPeriod;

   /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 */
   Channel1Pulse = (uint16_t) (((uint32_t) 50 * (TimerPeriod - 1)) / 100);

   /* Time Base configuration */
   TIM_TimeBaseStructure.TIM_Prescaler = 0;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

   TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

   /* Channel 1, 2 and 3 Configuration in PWM mode */
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
   TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
   TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
   TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
   TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
   TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
   TIM_OC1Init(TIM1, &TIM_OCInitStructure);

   /* Automatic Output enable, Break, dead time and lock configuration*/
   TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
   TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
   TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
   TIM_BDTRInitStructure.TIM_DeadTime = 0;
   TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
   TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
   TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

   TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

   /* TIM1 counter enable */
   TIM_Cmd(TIM1, ENABLE);

   /* Main Output Enable */
   TIM_CtrlPWMOutputs(TIM1, ENABLE);

}
