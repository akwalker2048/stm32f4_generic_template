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
uint8_t cb_init(circular_buffer_t *cb, uint8_t *data, uint16_t size);
uint8_t cb_add_byte(circular_buffer_t *cb, uint8_t byte);
uint8_t cb_get_byte(circular_buffer_t *cb, uint8_t *byte);
/* Used Internally */
uint8_t cb_increment_temp_head(circular_buffer_t *cb);
uint8_t cb_increment_head(circular_buffer_t *cb);
uint8_t cb_increment_tail(circular_buffer_t *cb);
/* Used when hardware has control over the head pointer... */
uint8_t cb_set_head_dma(circular_buffer_t *cb, uint16_t head_dma);

#endif
