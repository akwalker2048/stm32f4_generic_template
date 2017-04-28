#include <stdint.h>

/* awalker - Should these files be named hardware_quadrature_encoder.c and
 *           hardware_quadrature_encoder.h since they actually set pins and
 *           choose the timer????
 */

#include "stm32f4xx_conf.h"
#include "quad_encoder.h"


uint8_t quad_encoder_initialized = 0;

void quad_encoder_init(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;

   // turn on the clocks for each of the ports needed
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

   // now configure the pins themselves
   // they are all going to be inputs with pullups
   GPIO_StructInit (&GPIO_InitStructure);
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
   GPIO_Init (GPIOB, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
   GPIO_Init (GPIOB, &GPIO_InitStructure);

   // Connect the pins to their Alternate Functions
   GPIO_PinAFConfig (GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
   GPIO_PinAFConfig (GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

   // Timer peripheral clock enable
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

   // set them up as encoder inputs
   // set both inputs to rising polarity to let it use both edges
   TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
   TIM_SetAutoreload (TIM3, 0xFFFF);

   // turn on the timer/counters
   TIM_Cmd (TIM3, ENABLE);

   quad_encoder_initialized = 1;

}

void quad_encoder_read_position(uint32_t *position_counts)
{

   *position_counts = 0x0000;
   if(quad_encoder_initialized == 1)
   {
      *position_counts = TIM_GetCounter(TIM3);
   }

}

void quad_encoder_set_position(uint32_t position_counts)
{

   if(quad_encoder_initialized == 1)
   {
      if(position_counts > 0xFFFF)
      {
         position_counts = 0xFFFF;
      }

      TIM_SetCounter(TIM3, position_counts);
   }

}
