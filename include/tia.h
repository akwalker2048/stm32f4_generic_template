/**
 * @file tia.h
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
#ifndef TIA_H
#define TIA_H

#include <stdint.h>
#include "stm32f4xx_conf.h"
#include "generic_packet.h"

/**
 *
 * @fn void tia_init(void)
 * @brief Initialize 32 bit timer and capture compare hardware.
 *
 * TIM5 is used because it is 32 bit and can run at 84 MHz.
 * DMA buffers are set up to handle data.
 *
 * @param None
 * @return None
 *
 */
void tia_init(void);

#endif
