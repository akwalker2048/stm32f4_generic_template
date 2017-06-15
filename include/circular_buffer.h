/**
 * @file circular_buffer.h
 * @author Andrew K. Walker
 * @date 23 MAY 2017
 * @brief Header for circular buffer library.
 *
 */

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>

#define CB_MIN_BUFFER_SIZE 4


/* ************************************************************* */
/* * Circular Buffer Return Codes                              * */
/* ************************************************************* */
#define CB_SUCCESS                 0x00
#define CB_ERROR_TAIL_CAUGHT_HEAD  0x01
#define CB_ERROR_HEAD_CAUGHT_TAIL  0x02
#define CB_INIT_FAIL_SIZE          0x03
#define CB_INIT_FAIL_NULL          0x04
#define CB_ERROR_DMA_HEAD_OOR      0x05

/* ************************************************************* */
/* * Circular Buffer Struct for uC Communications              * */
/* ************************************************************* */
typedef struct {
   uint8_t *cb_data;
   uint16_t cb_size;
   uint16_t cb_head;
   uint16_t cb_head_temp;
   uint16_t cb_tail;
} circular_buffer_t;

/* ************************************************************* */
/* * Circular Buffer Functions                                 * */
/* ************************************************************* */
/* External Calls */

/**
 *
 * @fn uint8_t cb_init(circular_buffer_t *cb, uint8_t *data, uint16_t size)
 * @brief Used to initialize a circular buffer structure.
 * @param *cb Pointer to the circular_buffer_t data structure.
 * @param *data Pointer to uint8_t data array.
 * @param size The number of circular buffer eleme=nts in data.
 * @return uint8_t Circular buffer return code.
 *
 */
uint8_t cb_init(circular_buffer_t *cb, uint8_t *data, uint16_t size);

/**
 *
 * @fn uint8_t cb_add_byte(circular_buffer_t *cb, uint8_t byte);
 * @brief Adds a byte at the next position in the circular buffer.
 * @param *cb Pointer to the circular_buffer_t data structure.
 * @param byte Data to be added at the head.
 * @return uint8_t Circular buffer return code.
 *
 */
uint8_t cb_add_byte(circular_buffer_t *cb, uint8_t byte);

/**
 *
 * @fn uint8_t cb_get_byte(circular_buffer_t *cb, uint8_t *byte);
 * @brief Retrieves a byte at the current tail location.
 * @param *cb Pointer to the circular_buffer_t data structure.
 * @param *byte Retrieved data.
 * @return uint8_t Circular buffer return code.
 *
 */
uint8_t cb_get_byte(circular_buffer_t *cb, uint8_t *byte);

/**
 *
 * @fn uint8_t cb_set_head_dma(circular_buffer_t *cb, uint16_t head_dma);
 * @brief Unique function when using a hardware DMA controller.
 * @param *cb Pointer to the circular_buffer_t data structure.
 * @param head_dma The new circular buffer head location..
 * @return uint8_t Circular buffer return code.
 *
 * This function is unique to when a hardware DMA controller is being used to
 * load data into the circular buffer.  In order to use the other convenience
 * functions, we need to make sure that the head being tracked in software
 * matches the head location in hardware.
 *
 */
uint8_t cb_set_head_dma(circular_buffer_t *cb, uint16_t head_dma);

#endif
