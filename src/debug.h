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

typedef enum {DEBUG_LED_GREEN = 0,
              DEBUG_LED_ORANGE,
              DEBUG_LED_RED,
              DEBUG_LED_BLUE,
              NUM_DEBUG} debug_outputs;

typedef enum {DEBUG_STATE_CLEAR = 0,
              DEBUG_STATE_SET,
              DEBUG_STATE_BLINK} debug_states;


typedef enum {DEBUG_BLINK_NONE = 0,
              DEBUG_BLINK_SLOW,
              DEBUG_BLINK_MEDIUM,
              DEBUG_BLINK_FAST,
              DEBUG_BLINK_ERROR} debug_blink_rate;

typedef struct {
   debug_outputs name;
   debug_states state;
   debug_blink_rate blink;
   uint8_t initialized;
   GPIO_TypeDef *port;
   uint16_t pin;
} debug_struct;


/**
 * @fn void debug_init(void)
 * @brief Initializes GPIO used for LEDs and other debug.
 *
 * @param None
 * @return None
 */
void debug_init(void);

/**
 * @fn void debug_output_set(debug_outputs out)
 * @brief Set this debug pin high...
 *
 * @param None
 * @return None
 */
void debug_output_set(debug_outputs out);

/**
 * @fn void debug_output_clear(debug_outputs out)
 * @brief Set this debug pin low...
 *
 * @param None
 * @return None
 */
void debug_output_clear(debug_outputs out);

/**
 * @fn void debug_output_toggle(debug_outputs out)
 * @brief Set this debug pin the opposite of it's current state...
 *
 * @param None
 * @return None
 */
void debug_output_toggle(debug_outputs out);

/**
 * @fn void debug_output_blink(debug_outputs out, debug_blink_rate rate)
 * @brief Make this pin toggle indefinitely at a fixed rate.
 *
 * @param None
 * @return None
 */
void debug_output_blink(debug_outputs out, debug_blink_rate rate);

#endif
