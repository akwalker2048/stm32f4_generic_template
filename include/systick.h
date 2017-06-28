/**
 * @file systick.h
 * @author Andrew K. Walker
 * @date 06 JUN 2017
 * @brief Basic systick timer function
 *
 * Even if it's just for a heartbeat, to send a watchdog message, or to keep
 * time, it's good to have a systick.
 */

#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>
#include "stm32f4xx_conf.h"


/**
 *
 * @fn void systick_init(void)
 * @brief Set up the systick timer.
 *
 * @param None
 * @return None
 *
 */
void systick_init(void);

/**
 *
 * @fn void systick_delay_ms(uint32_t delay_ms)
 * @brief A simple delay function.
 *
 * @param None
 * @return None
 *
 */
void systick_delay_ms(uint32_t delay_ms);

void Delay(__IO uint32_t nCount);

#endif
