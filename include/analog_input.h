/**
 * @file analog_input.h
 * @author Andrew K. Walker
 * @date 07 JUN 2017
 * @brief Convenience library for uC analog_input.
 */

#ifndef ANALOG_INPUT_H
#define ANALOG_INPUT_H

#include <stdint.h>
#include "stm32f4xx_conf.h"

/**
 * @fn void analog_input_init(void)
 * @brief Initializes analog input for this project.
 *
 * @param None
 * @return None
 */
void analog_input_init(void);

/**
 * @fn void analog_input_read(float *vc14, float *vc15);
 * @brief Returns the values on ADC channels.
 *
 * @param None
 * @return None
 */
uint8_t analog_input_read(float *vc14, float *vc15);

#endif
