/**
 * @file debug.c
 * @author Andrew K. Walker
 * @date 07 JUN 2017
 * @brief Convenience library for uC debug.
 */
#include "debug.h"

uint8_t debug_initialized = 0;

volatile uint8_t send_code_version;

/* Public Function - Doxygen documentation is included in the header. */
void debug_init(void)
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

   debug_initialized = 1;
}

