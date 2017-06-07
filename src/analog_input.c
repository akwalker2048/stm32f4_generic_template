/**
 * @file analog_input.c
 * @author Andrew K. Walker
 * @date 07 JUN 2017
 * @brief Analog input functions.
 */
#include "analog_input.h"

#include "debug.h"

uint8_t analog_input_initialized = 0;


/* Public Function - Doxygen documentation is in the header. */
void analog_input_init(void)
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

   analog_input_initialized = 1;

}


/* Public Function - Doxygen documentation included in the header. */
uint8_t analog_input_read(float *vc14, float *vc15)
{
   uint16_t c14, c15;

   /* VDD = 2.935 Volts on discovery card. */

   if(analog_input_initialized)
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

