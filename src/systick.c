/**
 * @file systick.c
 * @author Andrew K. Walker
 * @date 06 JUN 2017
 * @brief Basic systick timer function
 *
 * Even if it's just for a heartbeat, to send a watchdog message, or to keep
 * time, it's good to have a systick.
 */
#include "systick.h"

uint8_t systick_initialized = 0;

/* Free Running Coutner */
volatile uint32_t ms_counter = 0;
/* Resettable Counter */
volatile uint32_t ms_counter_r = 0;


/* Public Function - Doxygen documentation is in the header file. */
void systick_init(void)
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
/* SysTick Handler  (moved from stm32f4xx_it.c)                       */
/* ****************************************************************** */
/**
 *
 * @fn void SysTick_Handler(void)
 * @brief Handles the SysTick interrupt.
 *
 * Was originally in stm32f4xx_it.c.  But from the organization of my code, you
 * can tell that I don't like ST's architecture of putting all of the interrupt
 * handlers in one spot.
 *
 * @param None
 * @return None
 *
 */
void SysTick_Handler(void)
{
   ms_counter++;
   ms_counter_r++;

   if(ms_counter%50 == 0)
   {

      /** @todo Need to get the debug code set up and use that to toggle the
       *  heartbeat LED here.
       */
      /* if(GPIO_ReadInputDataBit(GPIOD, LED_PIN_GREEN) == Bit_SET) */
      /* { */
      /*    GPIO_ResetBits(GPIOD, LED_PIN_GREEN); */
      /*    /\* GPIO_SetBits(GPIOD, LED_PIN_BLUE); *\/ */

      /* } */
      /* else */
      /* { */
      /*    GPIO_SetBits(GPIOD, LED_PIN_GREEN); */
      /*    /\* GPIO_ResetBits(GPIOD, LED_PIN_BLUE); *\/ */

      /* } */


   }


}


/* Public Function - Doxygen documentation is in the header file. */
void systick_delay_ms(uint32_t delay_ms)
{
   /** @todo I don't believe this is thread safe.  Be careful where this is
    *  called from.
    */
   ms_counter_r = 0;
   while(ms_counter_r < delay_ms);
}
