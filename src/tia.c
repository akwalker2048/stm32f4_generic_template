/**
 * @file tia.c
 * @author Andrew K. Walker
 * @date 06 JUN 2017
 * @brief Time Interval Analysis
 *
 * Measuring the time between edges is useful in many different areas.  This tool
 * provides the ability to record timestamps at edges of digital signals.  It can
 * operate continually for low speed signals...or with a RAM buffer for short
 * bursts of fast signals.
 *
 * ...at least it will once I finish rewriting the code...
 */
#include "tia.h"

uint8_t tia_initialized = 0;

/* Public Function - Doxygen documentation is in the header file. */
void tia_init(void)
{

   /* Init TIM5 Pins for Capture/Compare...TIA Application */
   /* TIM5_CH1-> A0  !!CH1 and A0 not used...using the pushbutton feature!!
    * TIM5_CH2-> A1
    * TIM5_CH3-> A2
    * TIM5_CH4-> A3
    */

   tia_initialized = 1;

}

