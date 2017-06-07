/**
 * @file pushbutton.h
 * @author Andrew K. Walker
 * @date 07 JUN 2017
 * @brief Headers for the discovery board pushbutton function.
 */

#ifndef PUSHBUTTON_H
#define PUSHBUTTON_H

#include <stdint.h>
#include "stm32f4xx_conf.h"

/**
 * @fn void pushbutton_init(void)
 * @brief Initializes the external interrupt hardware for the user pushbutton.
 *
 * @param None
 * @return None
 */
void pushbutton_init(void);

#endif
