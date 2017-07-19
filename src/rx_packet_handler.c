/**
 * @file rx_packet_handler.c
 * @author Andrew K. Walker
 * @date 26 MAY 2017
 * @brief Handles incoming GenericPackets.
 *
 * Regardless of what interface the data is coming in...completed packets are
 * handed off here to be dealt with.
 */


#include "rx_packet_handler.h"
#include "TMC260.h"

#include "debug.h"

#include "tilt_stepper_motor_control.h"

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

   tmc260_status_struct stat_struct;

   uint8_t intpol, dedge, mres;
   uint8_t tbl, chm, rndtf, hdec, hend, hstrt, toff;
   uint8_t seimin, sedn, semax, seup, semin;
   uint8_t sfilt, sgt, cs;
   uint8_t tst, slph, slpl, diss2g, ts2g, sdoff, vsense, rdsel;

   float pos;

   uint8_t stat_type;

   float multiplier;

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
                        /* /\* GPIO_SetBits(GPIOD, LED_PIN_RED); *\/ */

                        /* /\* Extract the new values. *\/ */
                        /* extract_motor_set_pid(gp_ptr, &proportional, &integral, &derivative); */
                        /* /\* Call a function here to set the gains. *\/ */
                        /* retval = tilt_motor_set_pid_gains(proportional, integral, derivative); */
                        /* if(retval == 0) */
                        /* { */
                        /*    /\* Query the new gains from the motor driver. *\/ */
                        /*    proportional = 0.0f; */
                        /*    integral = 0.0f; */
                        /*    derivative = 0.0f; */
                        /*    retval = tilt_motor_query_pid_gains(&proportional, &integral, &derivative); */
                        /*    if(retval == 0) */
                        /*    { */
                        /*       /\* Respond with the new gains. *\/ */
                        /*       retval_gpcb = gpcb_increment_temp_head(&gpcbs_rx_gp_queue); */
                        /*       if(retval_gpcb == GP_CIRC_BUFFER_SUCCESS) */
                        /*       { */
                        /*          create_motor_resp_pid(&(gpcbs_rx_gp_queue.gpcb[gpcbs_rx_gp_queue.gpcb_head_temp]), proportional, integral, derivative); */
                        /*          retval_gpcb = gpcb_increment_head(&gpcbs_rx_gp_queue); */
                        /*          if(retval_gpcb == GP_CIRC_BUFFER_SUCCESS) */
                        /*          { */
                        /*             /\* Add to full_duplex_outgoing_queueu....... *\/ */
                        /*             /\* And don't forget to write that callback function... *\/ */
                        /*             retval_fdud = full_duplex_usart_dma_add_to_queue(&(gpcbs_rx_gp_queue.gpcb[gpcbs_rx_gp_queue.gpcb_head]), gpcbs_rx_gp_queue_callback, gpcbs_rx_gp_queue.gpcb_head); */
                        /*             if(retval_fdud == FDUD_SUCCESS) */
                        /*             { */

                        /*             } */
                        /*          } */
                        /*       } */
                        /*       else */
                        /*       { */
                        /*          /\* Someone else was already writing the next outgoing */
                        /*             gp.  We need to handle that somehow. */
                        /*          *\/ */
                        /*       } */
                        /*    } */
                        /* } */
                        /* GPIO_ResetBits(GPIOD, LED_PIN_RED); */
                     } /* MOTOR_SET_PID */
                     break;
                  case MOTOR_START:
                     {
                        /* retval = tilt_motor_start(); */

                        tilt_stepper_motor_tilt();

                     } /* MOTOR_START */
                     break;
                  case MOTOR_STOP:
                     {
                        /* retval = tilt_motor_stop(); */

                        tilt_stepper_motor_stop();

                     } /* MOTOR_STOP */
                     break;
                  case MOTOR_HOME:
                     {
                        tilt_stepper_motor_home();
                     } /* MOTOR_HOME */
                     break;
                  case MOTOR_SET_POSITION:
                     {
                        extract_motor_set_position(gp_ptr, &pos);
                        tilt_stepper_motor_go_to_pos(pos);
                     }
                     break;
                  case MOTOR_SET_TILT_MULTIPLIER:
                     {
                        tilt_stepper_motor_stop();
                        extract_motor_set_tilt_multiplier(gp_ptr, &multiplier);
                        tilt_stepper_motor_set_profile_multiplier(multiplier);
                        tilt_stepper_motor_tilt();
                     }
                     break;
                  case MOTOR_TMC260_QUERY_STATUS:
                     {
                        extract_motor_tmc260_query_status(gp_ptr, &stat_type);

                        TMC260_status(stat_type, &stat_struct, 0);
                        /* Respond with the status. */
                        retval_gpcb = gpcb_increment_temp_head(&gpcbs_rx_gp_queue);
                        if(retval_gpcb == GP_CIRC_BUFFER_SUCCESS)
                        {
                           create_motor_tmc260_resp_status(&(gpcbs_rx_gp_queue.gpcb[gpcbs_rx_gp_queue.gpcb_head_temp]), stat_struct.position, stat_struct.stall_guard, stat_struct.current, stat_struct.status_byte);
                           retval_gpcb = gpcb_increment_head(&gpcbs_rx_gp_queue);
                           if(retval_gpcb == GP_CIRC_BUFFER_SUCCESS)
                           {
                              /* Add to full_duplex_outgoing_queueu....... */
                              /* And don't forget to write that callback function... */
                              retval_fdud = full_duplex_usart_dma_add_to_queue(&(gpcbs_rx_gp_queue.gpcb[gpcbs_rx_gp_queue.gpcb_head]), gpcbs_rx_gp_queue_callback, gpcbs_rx_gp_queue.gpcb_head);
                           }
                        }
                     }
                     break; /* MOTOR_TMC260_QUERY_STATUS */
                  case MOTOR_TMC260_SET_DRVCTRL_SDON:
                     {
                        tilt_stepper_motor_stop();
                        extract_motor_tmc260_set_drvctrl_sdon(gp_ptr, &intpol, &dedge, &mres);
                        TMC260_send_drvctrl_sdon(intpol, dedge, mres);
                        tilt_stepper_motor_tilt();
                        /* Respond with ACK? */

                     }
                     break; /* MOTOR_TMC260_SET_DRVCTRL_SDON */
                  case MOTOR_TMC260_QUERY_DRVCTRL_SDON:
                     {

                     }
                     break; /* MOTOR_TMC260_QUERY_DRVCTRL_SDON */
                  case MOTOR_TMC260_SET_CHOPCONF:
                     {
                        tilt_stepper_motor_stop();
                        extract_motor_tmc260_set_chopconf(gp_ptr, &tbl, &chm, &rndtf, &hdec, &hend, &hstrt, &toff);
                        TMC260_send_chopconf(tbl, chm, rndtf, hdec, hend, hstrt, toff);
                        tilt_stepper_motor_tilt();
                        /* Respond with ACK? */

                     }
                     break; /* MOTOR_TMC260_SET_CHOPCONF */
                  case MOTOR_TMC260_QUERY_CHOPCONF:
                     {

                     }
                     break; /* MOTOR_TMC260_QUERY_CHOPCONF */
                  case MOTOR_TMC260_SET_SMARTEN:
                     {
                        tilt_stepper_motor_stop();
                        extract_motor_tmc260_set_smarten(gp_ptr, &seimin, &sedn, &semax, &seup, &semin);
                        TMC260_send_smarten(seimin, sedn, semax, seup, semin);
                        tilt_stepper_motor_tilt();
                        /* Respond with ACK? */
                     }
                     break; /* MOTOR_TMC260_SET_SMARTEN */
                  case MOTOR_TMC260_QUERY_SMARTEN:
                     {
                        /* MOTOR_TMC260_RESP_SMARTEN */
                     }
                     break; /* MOTOR_TMC260_QUERY_SMARTEN */
                  case MOTOR_TMC260_SET_DRVCONF:
                     {
                        tilt_stepper_motor_stop();
                        extract_motor_tmc260_set_drvconf(gp_ptr, &tst, &slph, &slpl, &diss2g, &ts2g, &sdoff, &vsense, &rdsel);
                        TMC260_send_drvconf(tst, slph, slpl, diss2g, ts2g, sdoff, vsense, rdsel);
                        tilt_stepper_motor_tilt();
                        /* Respond with ACK? */
                     }
                     break; /* MOTOR_TMC260_SET_DRVCONF */
                  case MOTOR_TMC260_QUERY_DRVCONF:
                     {

                     }
                     break; /* MOTOR_TMC260_QUERY_DRVCONF */
                  case MOTOR_TMC260_SET_SGCSCONF:
                     {
                        tilt_stepper_motor_stop();
                        extract_motor_tmc260_set_sgcsconf(gp_ptr, &sfilt, &sgt, &cs);
                        TMC260_send_sgcsconf(sfilt, sgt, cs);
                        tilt_stepper_motor_tilt();
                        /* Respond with ACK? or the MOTOR_TMC260_RESP_SGCSCONF packet?*/
                     }
                     break; /* MOTOR_TMC260_SET_SGCSCONF */
                  case MOTOR_TMC260_QUERY_SGCSCONF:
                     {

                     }
                     break; /* MOTOR_TMC260_QUERY_SGCSCONF */
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
