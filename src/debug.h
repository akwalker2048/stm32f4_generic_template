/**
 * @file debug.h
 * @author Andrew K. Walker
 * @date 07 JUN 2017
 * @brief Convenience library for uC debug.
 */

#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>
#include "stm32f4xx_conf.h"

#ifndef GIT_REVISION
#define GIT_REVISION "generic-stm32f4-DEADBEEF"
#endif

/* Green  - D12 */
#define LED_PIN_GREEN    GPIO_Pin_12
/* Orange - D13 */
#define LED_PIN_ORANGE   GPIO_Pin_13
/* Red    - D14 */
#define LED_PIN_RED      GPIO_Pin_14
/* Blue   - D15 */
#define LED_PIN_BLUE     GPIO_Pin_15


/**
 * @fn void debug_init(void)
 * @brief Initializes GPIO used for LEDs and other debug.
 *
 * @param None
 * @return None
 */
void debug_init(void);

#endif
