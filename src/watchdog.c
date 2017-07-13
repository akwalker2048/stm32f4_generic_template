/**
 * @file watchdog.c
 * @author Andrew K. Walker
 * @date 13 JUL 2017
 * @brief Watchdog implementation in case our micro goes out to lunch...
 */
#include "watchdog.h"

volatile uint8_t watchdog_enabled = 0;

void watchdog_init(void)
{

   /* (#) Enable WWDG clock using RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE) function */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

   /*    (#) Configure the WWDG prescaler using WWDG_SetPrescaler() function */
   /* WWDG_Prescaler_1 will count at 20507.8125 Hz */
   WWDG_SetPrescaler(WWDG_Prescaler_1);

   /*    (#) Configure the WWDG refresh window using WWDG_SetWindowValue() function */
   WWDG_SetWindowValue(WATCHDOG_WINDOW_COUNT);

   /*    (#) Set the WWDG counter value and start it using WWDG_Enable() function. */
   /*    When the WWDG is enabled the counter value should be configured to  */
   /*    a value greater than 0x40 to prevent generating an immediate reset.   */
   WWDG_Enable(WATCHDOG_RESET_COUNT);

   watchdog_enabled = 1;

}



void watchdog_tickle(void)
{
   uint8_t current_count;

   if(watchdog_enabled)
   {
      current_count = WWDG->CR & 0x7F;
      if(current_count < WATCHDOG_WINDOW_COUNT)
      {
         WWDG_SetCounter(WATCHDOG_RESET_COUNT);
      }
   }
}
