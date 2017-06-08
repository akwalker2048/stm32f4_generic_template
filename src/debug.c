/**
 * @file debug.c
 * @author Andrew K. Walker
 * @date 07 JUN 2017
 * @brief Convenience library for uC debug.
 */
#include "debug.h"

debug_struct dbg_outputs[NUM_DEBUG];

uint8_t debug_initialized = 0;

volatile uint8_t send_code_version;

/* Public Function - Doxygen documentation is included in the header. */
void debug_init(void)
{
   GPIO_InitTypeDef  GPIO_InitStructure;

   /* Init LEDs */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOD, &GPIO_InitStructure);

   /* Fill debug_struct manually for now. */
   dbg_outputs[DEBUG_LED_GREEN].name = DEBUG_LED_GREEN;
   dbg_outputs[DEBUG_LED_GREEN].state = DEBUG_STATE_CLEAR;
   dbg_outputs[DEBUG_LED_GREEN].blink = DEBUG_BLINK_NONE;
   dbg_outputs[DEBUG_LED_GREEN].initialized = 1;
   dbg_outputs[DEBUG_LED_GREEN].port = GPIOD;
   dbg_outputs[DEBUG_LED_GREEN].pin = GPIO_Pin_12;

   dbg_outputs[DEBUG_LED_ORANGE].name = DEBUG_LED_ORANGE;
   dbg_outputs[DEBUG_LED_ORANGE].initialized = 1;
   dbg_outputs[DEBUG_LED_ORANGE].state = DEBUG_STATE_CLEAR;
   dbg_outputs[DEBUG_LED_ORANGE].blink = DEBUG_BLINK_NONE;
   dbg_outputs[DEBUG_LED_ORANGE].port = GPIOD;
   dbg_outputs[DEBUG_LED_ORANGE].pin = GPIO_Pin_13;

   dbg_outputs[DEBUG_LED_RED].name = DEBUG_LED_RED;
   dbg_outputs[DEBUG_LED_RED].initialized = 1;
   dbg_outputs[DEBUG_LED_RED].state = DEBUG_STATE_CLEAR;
   dbg_outputs[DEBUG_LED_RED].blink = DEBUG_BLINK_NONE;
   dbg_outputs[DEBUG_LED_RED].port = GPIOD;
   dbg_outputs[DEBUG_LED_RED].pin = GPIO_Pin_14;

   dbg_outputs[DEBUG_LED_BLUE].name = DEBUG_LED_BLUE;
   dbg_outputs[DEBUG_LED_BLUE].state = DEBUG_STATE_CLEAR;
   dbg_outputs[DEBUG_LED_BLUE].blink = DEBUG_BLINK_NONE;
   dbg_outputs[DEBUG_LED_BLUE].initialized = 1;
   dbg_outputs[DEBUG_LED_BLUE].port = GPIOD;
   dbg_outputs[DEBUG_LED_BLUE].pin = GPIO_Pin_15;

   debug_initialized = 1;
}

void debug_output_set(debug_outputs out)
{
   if(dbg_outputs[out].initialized)
   {
      GPIO_SetBits(dbg_outputs[out].port, dbg_outputs[out].pin);
      dbg_outputs[out].state = DEBUG_STATE_SET;
   }
}

void debug_output_clear(debug_outputs out)
{
   if(dbg_outputs[out].initialized)
   {
      GPIO_ResetBits(dbg_outputs[out].port, dbg_outputs[out].pin);
      dbg_outputs[out].state = DEBUG_STATE_CLEAR;
   }
}

void debug_output_toggle(debug_outputs out)
{
   if(dbg_outputs[out].initialized)
   {
      if(GPIO_ReadInputDataBit(dbg_outputs[out].port, dbg_outputs[out].pin) == Bit_SET)
      {
         debug_output_clear(out);
      }
      else
      {
         debug_output_set(out);
      }
   }
}

void debug_output_blink(debug_outputs out, debug_blink_rate rate)
{
   /** @todo There currently isn't a timer interrupt in the debug code.  So
    *  setting the blink rate will currently do nothing.  This needs to be
    *  implemented.  For now, we will just set the approprate states in the
    *  debug structure that would be checked in the timer interrupt to see
    *  if it was time to blink.
    */
   if(dbg_outputs[out].initialized)
   {
      dbg_outputs[out].blink = rate;
      dbg_outputs[out].state = DEBUG_STATE_BLINK;
   }
}
