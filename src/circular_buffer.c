#include "circular_buffer.h"
#include <stdlib.h>

/* External Calls */
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

uint8_t cb_get_byte(circular_buffer_t *cb, uint8_t *byte)
{
   uint8_t retval;

   if(cb->cb_head != cb->cb_tail)
   {
         *byte = cb->cb_data[cb->cb_tail];
         retval = cb_increment_tail(cb);
   }
   else
   {
      retval = CB_ERROR_TAIL_CAUGHT_HEAD;
   }

   return retval;
}

/* Used Internally */
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


uint8_t cb_increment_head(circular_buffer_t *cb)
{
   /* This should always be safe...because we didn't increment head_temp unless
    * it was OK to do so...and buffer rollover was handled there as well...
    */
   cb->cb_head = cb->cb_head_temp;

   return CB_SUCCESS;

}

uint8_t cb_increment_tail(circular_buffer_t *cb)
{
   uint32_t temp_tail;

   /* Store this in the event that we caught the head. */
   temp_tail = cb->cb_tail;

   cb->cb_tail++;
   if(cb->cb_tail >= cb->cb_size)
   {
      cb->cb_tail = 0;
   }

   /* Did we catch the head?  Check here is greater than because we can read the
    * current head value because we don't increment it until it is ready to
    * read.  Notify the caller that we need to hang tight until the head is
    * incremented.
    */
   if(cb->cb_tail > cb->cb_head)
   {
      cb->cb_tail = temp_tail;
      return CB_ERROR_TAIL_CAUGHT_HEAD;
   }

   return CB_SUCCESS;

}

/* This function is used when the DMA hardware is in control of the head
 * pointer.
 */
uint8_t cb_set_head_dma(circular_buffer_t *cb, uint16_t head_dma)
{

   if(head_dma < cb->cb_size)
   {
      cb->cb_head = head_dma;
   }
   else
   {
      return CB_ERROR_DMA_HEAD_OOR;
   }

   return CB_SUCCESS;
}
