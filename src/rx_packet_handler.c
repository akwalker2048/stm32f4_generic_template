#include "rx_packet_handler.h"

GenericPacketCircularBuffer gpcbs_rx_gp_queue;
GenericPacket rx_gp_queue[RX_PACKET_HANDLER_GP_QUEUE_SIZE];

uint8_t rx_packet_handler_initialized = 0;
FDUD_TxQueueCallback gpcbs_rx_gp_queue_callback = NULL;

uint8_t new_tail_callback_failed  = 0;

/* rx_packet_handler_init
 *
 * Notes:
 *  +This function should be called before rx_packet_handler
 */
void rx_packet_handler_init(void)
{
   uint8_t retval_gpcb;


   gpcbs_rx_gp_queue_callback = &rx_packet_handler_packet_send_callback;

   retval_gpcb = gpcb_initialize(&gpcbs_rx_gp_queue, rx_gp_queue, RX_PACKET_HANDLER_GP_QUEUE_SIZE);
   if(retval_gpcb == GP_CIRC_BUFFER_SUCCESS)
   {
      rx_packet_handler_initialized = 1;
   }

}

/* rx_packet_handler
 *
 * Notes:
 *  +This function is of the format GenericPacketCallback
 *  +It should only be called when a full packet has been received and the
 *   checksum matched
 *  +
 */
void rx_packet_handler(GenericPacket *gp_ptr)
{
   uint8_t retval_gpcb;
   uint8_t retval_fdud;
   uint8_t retval;
   float proportional, integral, derivative;

   if(rx_packet_handler_initialized)
   {
      switch(gp_ptr->gp[GP_LOC_PROJ_ID])
      {
         case GP_PROJ_UNIVERSAL:
            {
               switch(gp_ptr->gp[GP_LOC_PROJ_SPEC])
               {
                  default:
                     break;
               }
            } /* GP_PROJ_UNIVERSAL */
            break;
         case GP_PROJ_MOTOR:
            {
               switch(gp_ptr->gp[GP_LOC_PROJ_SPEC])
               {
                  case MOTOR_SET_PID:
                     {
                        /* GPIO_SetBits(GPIOD, LED_PIN_RED); */

                        /* Extract the new values. */
                        extract_motor_set_pid(gp_ptr, &proportional, &integral, &derivative);
                        /* Call a function here to set the gains. */
                        retval = tilt_motor_set_pid_gains(proportional, integral, derivative);
                        if(retval == 0)
                        {
                           /* Query the new gains from the motor driver. */
                           proportional = 0.0f;
                           integral = 0.0f;
                           derivative = 0.0f;
                           retval = tilt_motor_query_pid_gains(&proportional, &integral, &derivative);
                           if(retval == 0)
                           {
                              /* Respond with the new gains. */
                              retval_gpcb = gpcb_increment_temp_head(&gpcbs_rx_gp_queue);
                              if(retval_gpcb == GP_CIRC_BUFFER_SUCCESS)
                              {
                                 create_motor_resp_pid(&(gpcbs_rx_gp_queue.gpcb[gpcbs_rx_gp_queue.gpcb_head_temp]), proportional, integral, derivative);
                                 retval_gpcb = gpcb_increment_head(&gpcbs_rx_gp_queue);
                                 if(retval_gpcb == GP_CIRC_BUFFER_SUCCESS)
                                 {
                                    /* Add to full_duplex_outgoing_queueu....... */
                                    /* And don't forget to write that callback function... */
                                    retval_fdud = full_duplex_usart_dma_add_to_queue(&(gpcbs_rx_gp_queue.gpcb[gpcbs_rx_gp_queue.gpcb_head]), gpcbs_rx_gp_queue_callback, gpcbs_rx_gp_queue.gpcb_head);
                                    if(retval_fdud == FDUD_SUCCESS)
                                    {

                                    }
                                 }
                              }
                              else
                              {
                                 /* Someone else was already writing the next outgoing
                                    gp.  We need to handle that somehow.
                                 */
                              }
                           }
                        }
                        /* GPIO_ResetBits(GPIOD, LED_PIN_RED); */
                     } /* MOTOR_SET_PID */
                     break;
                  case MOTOR_START:
                     {
                        retval = tilt_motor_start();
                        if(retval == 0)
                        {
                           /* AWALKER - Replace with new full_duplex_usart_dma */
                           /* retval = get_next_outgoing_gp_head(&next_outgoing_head); */
                           /* if(retval == 0) */
                           /* { */
                           /*       create_motor_start(&(outgoing_circ_buffer[next_outgoing_head])); */
                           /*       increment_outgoing_gp_head(); */
                           /* } */
                           /* else */
                           /* { */
                           /*    /\* Someone else was already writing the next outgoing */
                           /*       gp.  We need to handle that somehow. */
                           /*    *\/ */
                           /* } */
                        }
                     } /* MOTOR_START */
                     break;
                  case MOTOR_STOP:
                     {
                        retval = tilt_motor_stop();
                        if(retval == 0)
                        {
                           /* AWALKER - Replace with new full_duplex_usart_dma */
                           /* retval = get_next_outgoing_gp_head(&next_outgoing_head); */
                           /* if(retval == 0) */
                           /* { */
                           /*    create_motor_stop(&(outgoing_circ_buffer[next_outgoing_head])); */
                           /*    increment_outgoing_gp_head(); */
                           /* } */
                           /* else */
                           /* { */
                           /*    /\* Someone else was already writing the next outgoing */
                           /*       gp.  We need to handle that somehow. */
                           /*    *\/ */
                           /* } */
                        }
                     } /* MOTOR_STOP */
                  default:
                     break;
               }
               break;

            } /* GP_PROJ_MOTOR */

         default:
            break;
      }
   } /* if(rx_packet_handler_initialized) */

}



void rx_packet_handler_packet_send_callback(uint32_t new_tail)
{
   /* Hopefully...this "new_tail" is the same tail we would get just by calling
    * the gpcb_increment_tail function.  We'll check...but then just set it and
    * hope for the best.  Maybe notify someone that we skipped a packet that
    * hasn't gone out...but the way the outgoing queue works...they should go in
    * order...so this shouldn't happen.
    */
   gpcb_increment_tail(&gpcbs_rx_gp_queue);

   if(gpcbs_rx_gp_queue.gpcb_tail != new_tail)
   {
      new_tail_callback_failed = 1;
      /* Notify myself somehow?  Maybe by putting another packet in the queue
       * that isn't working so well???
       */

      /* Well, just set it anyway!!!! */
      gpcbs_rx_gp_queue.gpcb_tail = new_tail;
   }

}
