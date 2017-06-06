/**
 * @file    hardware_STM32F407G_DISC1.c
 * @author  Andrew K. Walker
 * @date    13MAR2017
 * @brief   Main program body
 * @attention
 * <h2><center>&copy; COPYRIGHT 2017 Badger Technologies, LLC</center></h2>
 */

/**
 *  @todo Move all of these functions to files that are function specific.  The
 *  original plan was to have a single file with all functions using specific
 *  hardware in a single file in order to track what resources were being used.
 *  However, this project is made up of many different sub projects which may
 *  be used as smaller parts of other projects.  As such, it is better to have
 *  them be part of self contained units.  In the future, this file will
 *  eventually be removed.
 */


#include "stm32f4xx_conf.h"
#include "hardware_STM32F407G_DISC1.h"
#include "hardware_TB6612.h"
#include "tilt_motor_control.h"
#include "quad_encoder.h"


/* Global variable for handling USART so that whole packets are sent at a
 * a time.  In the future, I need to just add a function that is in this
 * file that controls all packet sending.  Maybe implement a circular
 * packet buffer for sending.
 */

volatile uint8_t grab_frame = 0;
volatile uint8_t send_code_version = 0;


volatile uint8_t sonar_index = 0;
volatile uint8_t sonar_data[4];



/* Variables Global Within This File */
uint8_t gpio_initialized = 0;
uint8_t systick_initialized = 0;
/* uint8_t usart_one_initialized = 0; */
/* uint8_t usart_one_dma_initialized = 0; */

uint8_t pushbutton_initialized = 0;
uint8_t analog_input_initialized = 0;
uint8_t adc_initialized = 0;
uint8_t tia_initialized = 0;


/* Free Running Coutner */
volatile uint32_t ms_counter = 0;
/* Resettable Counter */
volatile uint32_t ms_count_r = 0;

/* ****************************************************************** */
/* GPIO Initialization                                                */
/* ****************************************************************** */
/* Only purely GPIO should be initialized here.  Other pins should be *
 * initialized in their respective init functions as needed.          *
 * ****************************************************************** */
void init_gpio(void)
{
   GPIO_InitTypeDef  GPIO_InitStructure;

   /* Init LEDs */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
   GPIO_InitStructure.GPIO_Pin = LED_PIN_RED | LED_PIN_GREEN | LED_PIN_BLUE | LED_PIN_ORANGE;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOD, &GPIO_InitStructure);

   gpio_initialized = 1;
}

/* ****************************************************************** */
/* TIM Interrupt Initialization                                       */
/* ****************************************************************** */
void init_systick(void)
{
   /* Make sure we are using the correct value for SystemCoreClock! */
   SystemCoreClockUpdate();
   /* Set SysTick to expire every ms. */
   if (SysTick_Config(SystemCoreClock / 1000))
   {
      /* Capture error */
      while (1);
   }

   systick_initialized = 1;
}





/* ****************************************************************** */
/* ADC  Initialization                                         */
/* ****************************************************************** */
void init_adc(void)
{
   GPIO_InitTypeDef GPIO_InitStructure; //Variable used to setup the GPIO pins
   DMA_InitTypeDef DMA_InitStructure; //Variable used to setup the DMA
   ADC_InitTypeDef ADC_InitStructure; //Variable used to setup the ADC
   ADC_CommonInitTypeDef ADC_CommonInitStructure;

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

   GPIO_StructInit(&GPIO_InitStructure);
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   /* I think you can also hook up the battery voltage internally to the ADC. I
      need to look into that and add the capability here.
   */

   /* ADC Common Init */
   ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
   ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
   ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
   ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_12Cycles;
   ADC_CommonInit(&ADC_CommonInitStructure);

   /* Init ADC1 Specific Stuff */
   ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
   ADC_InitStructure.ADC_ScanConvMode = DISABLE;
   ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
   ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
   ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO; /* Actually...a don't care... */
   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
   ADC_InitStructure.ADC_NbrOfConversion = 1;
   ADC_Init(ADC1, &ADC_InitStructure); //Initialise ADC1

   ADC_Cmd(ADC1, ENABLE); //Enable ADC1

   /* Need to calibrate the ADC??? */

   adc_initialized = 1;

}


/* ****************************************************************** */
/* Pushbutton  Initialization                                         */
/* ****************************************************************** */
void init_pushbutton(void)
{

   GPIO_InitTypeDef  GPIO_InitStructure;
   EXTI_InitTypeDef   EXTI_InitStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;

   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOD, &GPIO_InitStructure);

   /* Connect EXTI Line0 to PA0 pin */
   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

   /* Configure EXTI Line0 */
   EXTI_InitStructure.EXTI_Line = EXTI_Line0;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   /* Enable and set EXTI Line0 Interrupt to the lowest priority */
   NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);



   pushbutton_initialized = 1;

}

/* /\* ****************************************************************** *\/ */
/* /\* Analog Input Initialization                                        *\/ */
/* /\* ****************************************************************** *\/ */
/* void init_analog_input(void) */
/* { */


/*    analog_input_initialized = 1; */

/* } */

/* ****************************************************************** */
/* TIA Initialization                                                 */
/* ****************************************************************** */
void init_tia(void)
{

   /* Init TIM5 Pins for Capture/Compare...TIA Application */
   /* TIM5_CH1-> A0  !!CH1 and A0 not used...using the pushbutton feature!!
    * TIM5_CH2-> A1
    * TIM5_CH3-> A2
    * TIM5_CH4-> A3
    */

   tia_initialized = 1;

}








/* ****************************************************************** */
/* SysTick Handler  (moved from stm32f4xx_it.c)                       */
/* ****************************************************************** */
void SysTick_Handler(void)
{
   static uint32_t ii = 0;

   static uint8_t up = 1;

   uint8_t retval;

   uint8_t temp_head;



   static float duty = -1.0f;

   float current_tilt_position;

   ms_counter++;
   ms_count_r++;

   ii++;







   /* tilt_motor_get_angle(&current_tilt_position); */
   /* if(current_tilt_position > TILT_MAX_ANGLE_RAD) */
   /* { */
   /*    TB6612_set_duty(-0.20); */
   /* } */

   /* if(current_tilt_position < TILT_MIN_ANGLE_RAD) */
   /* { */
   /*    TB6612_set_duty(0.20); */
   /* } */

   /* if(ms_counter%100 == 0) */
   /* { */

   /*    if(up == 1) */
   /*    { */
   /*       duty = duty + 0.05f; */
   /*       if(duty >= 0.50f) */
   /*       { */
   /*          up = 0; */
   /*       } */
   /*    } */
   /*    else */
   /*    { */
   /*       duty = duty - 0.05f; */
   /*       if(duty <= 0.05f) */
   /*       { */
   /*          up = 1; */
   /*       } */
   /*    } */
   /*    TB6612_set_duty(duty); */

   /* } */





   if(ms_counter%50 == 0)
   {

      send_code_version = 1;

      if(GPIO_ReadInputDataBit(GPIOD, LED_PIN_GREEN) == Bit_SET)
      {
         GPIO_ResetBits(GPIOD, LED_PIN_GREEN);
         /* GPIO_SetBits(GPIOD, LED_PIN_BLUE); */

      }
      else
      {
         GPIO_SetBits(GPIOD, LED_PIN_GREEN);
         /* GPIO_ResetBits(GPIOD, LED_PIN_BLUE); */

      }


   }


}





/* ****************************************************************** */
/* Pushbutton Handler                                                 */
/* ****************************************************************** */
void EXTI0_IRQHandler(void)
{
   if(EXTI_GetITStatus(EXTI_Line0) != RESET)
   {
      /* /\* Toggle LED1 *\/ */
      /* if(GPIO_ReadInputDataBit(GPIOD, LED_PIN_ORANGE) == Bit_SET) */
      /* { */
      /*    GPIO_ResetBits(GPIOD, LED_PIN_ORANGE); */
      /* } */
      /* else */
      /* { */
      /*    GPIO_SetBits(GPIOD, LED_PIN_ORANGE); */
      /* } */

      /* Zero the tilt motor here for now.  Eventually, we need to tie that to
         the pin that will actually have the flag piped in. */
      /* quad_encoder_set_position(TILT_ZERO_POSITION_QC); */
      /* TB6612_set_duty(0.25); */

      /* Clear the EXTI line 0 pending bit */
      EXTI_ClearITPendingBit(EXTI_Line0);
   }
}



/* ****************************************************************** */
/* Exported Functions                                                 */
/* ****************************************************************** */





void blocking_wait_ms(uint32_t delay_ms)
{
   ms_count_r = 0;
   while(ms_count_r < delay_ms);
}


void non_blocking_wait_ms(uint32_t delay_ms)
{
   /* Need to implement this! */
   /* ms_count_r = 0; */
   /* while(ms_count_r < delay_ms); */
}














uint8_t read_adc(float *vc14, float *vc15)
{
   uint16_t c14, c15;

   /* VDD = 2.935 Volts on discovery card. */

   if(adc_initialized)
   {
      ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_28Cycles);
      // Start the conversion
      ADC_SoftwareStartConv(ADC1);
      // Wait until conversion completion
      while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
      // Get the conversion value
      c14 = ADC_GetConversionValue(ADC1);

      ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_28Cycles);
      // Start the conversion
      ADC_SoftwareStartConv(ADC1);
      // Wait until conversion completion
      while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
      // Get the conversion value
      c15 = ADC_GetConversionValue(ADC1);

      *vc14 = ((float)c14 / (float)4096) * 2.935f;
      *vc15 = ((float)c15 / (float)4096) * 2.935f;
   }
   else
   {
      *vc14 = -9.99;
      *vc15 = -9.99;
   }

   return 0;
}

