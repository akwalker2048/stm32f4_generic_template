/**
 * @file circular_buffer.c
 * @author Andrew K. Walker
 * @date 23 MAY 2017
 * @brief Circular buffer library.
 *
 * This includes code for working with a ciruclar buffer made up of uint8_t's.
 * It's main purpose is to handle data coming in on a DMA circular buffer and
 * subsequently handled in a RAM buffer that can be handled in a lower priority
 * function.
 */


#include "circular_buffer.h"
#include <stdlib.h>

/* Used Internally */
uint8_t cb_increment_temp_head(circular_buffer_t *cb);
uint8_t cb_increment_head(circular_buffer_t *cb);
uint8_t cb_increment_tail(circular_buffer_t *cb);

/* Public Function - Doxygen documentation is in the header file. */
uint8_t cb_init(circular_buffer_t *cb, uint8_t *data, uint16_t size)
{

   if(data != NULL)
   {
      cb->cb_data = data;
   }
   else
   {
      return CB_INIT_FAIL_NULL;
   }

   if(size > CB_MIN_BUFFER_SIZE)
   {
      cb->cb_size = size;
   }
   else
   {
      return CB_INIT_FAIL_SIZE;
   }

   cb->cb_head = 0;
   cb->cb_head_temp = 0;
   cb->cb_tail = 0;

   return CB_SUCCESS;

}

/* Public Function - Doxygen documentation is in the header file. */
uint8_t cb_add_byte(circular_buffer_t *cb, uint8_t byte)
{
   uint8_t retval;

   cb->cb_data[cb->cb_head_temp] = byte;
   retval = cb_increment_temp_head(cb);
   if(retval == CB_SUCCESS)
   {
      retval = cb_increment_head(cb);
   }

   return retval;
}

/* Public Function - Doxygen documentation is in the header file. */
uint8_t cb_get_byte(circular_buffer_t *cb, uint8_t *byte)
{
   uint8_t retval, retval_inc_tail;

   if(cb->cb_head != cb->cb_tail)
   {
         *byte = cb->cb_data[cb->cb_tail];
         retval_inc_tail = cb_increment_tail(cb);
         retval = CB_SUCCESS;
   }
   else
   {
      retval = CB_ERROR_TAIL_CAUGHT_HEAD;
   }

   return retval;
}

/* Public Function - Doxygen documentation is in the header file. */
uint8_t cb_set_head_dma(circular_buffer_t *cb, uint16_t head_dma)
{

   if(head_dma <= cb->cb_size)
   {
      cb->cb_head = head_dma;
   }
   else
   {
      return CB_ERROR_DMA_HEAD_OOR;
   }

   return CB_SUCCESS;
}


/**
 *
 * @fn uint8_t cb_increment_temp_head(circular_buffer_t *cb)
 * @brief Moves the temp head forward one.
 * @param *cb Pointer to the circular_buffer_t data structure.
 * @return uint8_t Circular buffer return code.
 *
 * The temp head points to the location currently being written.  The actual
 * head isn't incremented until the write is complete and the data is ready
 * to be read.
 *
 */
uint8_t cb_increment_temp_head(circular_buffer_t *cb)
{
   uint16_t temp_temp_head;

   /* Store this in the event that we cannot increment head_temp. */
   temp_temp_head = cb->cb_head_temp;

   /* Increment head_temp. */
   cb->cb_head_temp++;
   if(cb->cb_head_temp >= cb->cb_size)
   {
      cb->cb_head_temp = 0;
   }

   /* Check to make sure we didn't catch the tail. */
   if(cb->cb_head_temp == cb->cb_tail)
   {
      /* If we did...don't increment...and let the caller know. */
      cb->cb_head_temp = temp_temp_head;
      return CB_ERROR_HEAD_CAUGHT_TAIL;
   }

   return CB_SUCCESS;


}

/**
 *
 * @fn uint8_t cb_increment_head(circular_buffer_t *cb)
 * @brief Moves the head forward one once the data has been written.
 * @param *cb Pointer to the circular_buffer_t data structure.
 * @return uint8_t Circular buffer return code.
 *
 * @todo Do we need to add an asm("DSB") or the like to make sure that the
 * address really is ready to be read????
 *
 */
uint8_t cb_increment_head(circular_buffer_t *cb)
{
   /* This should always be safe...because we didn't increment head_temp unless
    * it was OK to do so...and buffer rollover was handled there as well...
    */
   cb->cb_head = cb->cb_head_temp;

   return CB_SUCCESS;

}

/**
 *
 * @fn uint8_t cb_increment_tail(circular_buffer_t *cb)
 * @brief Moves the tail ahead one after the next byte has been read.
 * @param *cb Pointer to the circular_buffer_t data structure.
 * @return uint8_t Circular buffer return code.
 *
 */
uint8_t cb_increment_tail(circular_buffer_t *cb)
{
   if(cb->cb_tail != cb->cb_head)
   {
      cb->cb_tail++;
      if(cb->cb_tail >= cb->cb_size)
      {
         cb->cb_tail = 0;
      }
   }
   else
   {
      return CB_ERROR_TAIL_CAUGHT_HEAD;
   }

   return CB_SUCCESS;

}

