/**
 * @file    main.c
 * @author  Andrew K. Walker
 * @version V0.0.1
 * @date    01MAR2017
 * @brief   Main program body
 * @attention
 * <h2><center>&copy; COPYRIGHT 2017 Badger Technologies, LLC</center></h2>
 */

/* Includes ------------------------------------------------------------------*/
#include <math.h>

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#include "main.h"
#include "systick.h"
#include "pushbutton.h"
#include "debug.h"
#include "analog_input.h"

/** @todo All references to the hardware_STM32F407G_DISC1.h header should go
 *  away soon.
 */
/* #include "hardware_STM32F407G_DISC1.h" */

#include "full_duplex_usart_dma.h"
#include "rx_packet_handler.h"

#include "tilt_stepper_motor_control.h"
/* #include "tilt_motor_control.h" */
/* #include "lepton_functions.h" */

#include "quad_encoder.h"

#include "generic_packet.h"
#include "gp_proj_motor.h"
#include "gp_proj_analog.h"
#include "gp_proj_universal.h"

#include "rs485_sensor_bus.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

extern volatile uint8_t grab_frame;
extern volatile uint8_t send_code_version;


extern volatile motor_feedback_t mf;
extern volatile uint8_t send_motor_feedback;

extern volatile uint8_t tilt_stepper_motor_send_angle;

volatile uint8_t cts_pos_packet = 1;
#define POS_PACKET_CALLBACK_NUM 0x04

void main_packet_send_callback(uint32_t packet_num);
FDUD_TxQueueCallback gpcbs_main_queue_callback = &main_packet_send_callback;


void main_packet_send_callback(uint32_t packet_num)
{
   if(packet_num == POS_PACKET_CALLBACK_NUM)
   {
      cts_pos_packet = 1;
   }

}

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
   /*!< At this stage the microcontroller clock setting is already configured,
     this is done through SystemInit() function which is called from startup
     files (startup_stm32f40_41xxx.s/startup_stm32f427_437xx.s/startup_stm32f429_439xx.s)
     before to branch to application main.
     To reconfigure the default setting of SystemInit() function, refer to
     system_stm32f4xx.c file
   */


   GenericPacketCallback rx_packet_handler_ptr = &rx_packet_handler;


   float vc14, vc15;
   GenericPacket gp_porrst, gp_sftrst, gp_pinrst, gp_wwdgrst;
   GenericPacket gp_pos_rad;

   uint32_t pos_count, pos_ts;
   float pos_rad, prev_pos_rad;

   /* SystemCoreClockUpdate(); */

   debug_init();
   /* init_usart_one(); */
   /* init_usart_one_dma(); */

   /* rx_packet_handler_init() must be called before full_duplex_usart_dma_init()
    * or else we won't be ready to get packet handler callbacks...
    */
   rx_packet_handler_init();
   full_duplex_usart_dma_init(rx_packet_handler_ptr);

   /* init_usart_three(); */
   /* pushbutton_init(); */

   /* init_spi(); */
   /* init_i2c(); */

   systick_init();

   analog_input_init();

   /* Cannot RS485 and Tilt!!!! Pin A2 */
   /* rs485_sensor_bus_init_slave(); */
   /* rs485_sensor_bus_init_master(); */

   tilt_stepper_motor_init();

   /* Cannot RS485 and Tilt!!!! Pin A2 */
   /* tilt_motor_init(); */


   /* tilt_motor_get_angle(&pos_rad); */
   prev_pos_rad = pos_rad;

   grab_frame = 3;



   /* Check why we came out of reset? */
   if(RCC_GetFlagStatus(RCC_FLAG_WWDGRST) == SET)
   {
      create_universal_timestamp(&gp_wwdgrst, 0x1111);
      full_duplex_usart_dma_add_to_queue(&gp_wwdgrst, NULL, 0);
   }

   if(RCC_GetFlagStatus(RCC_FLAG_SFTRST) == SET)
   {
      create_universal_timestamp(&gp_sftrst, 0x2222);
      full_duplex_usart_dma_add_to_queue(&gp_sftrst, NULL, 0);
   }

   if(RCC_GetFlagStatus(RCC_FLAG_PORRST) == SET)
   {
      create_universal_timestamp(&gp_porrst, 0x3333);
      full_duplex_usart_dma_add_to_queue(&gp_porrst, NULL, 0);
   }

   if(RCC_GetFlagStatus(RCC_FLAG_PINRST) == SET)
   {
      /* This will always be true as best I can tell...so why check it? */
      create_universal_timestamp(&gp_pinrst, 0x4444);
      full_duplex_usart_dma_add_to_queue(&gp_pinrst, NULL, 0);
   }


   /* Now clear all of the RCC flags.  Otherwise, they will continue to be set. */
   RCC_ClearFlag();

   while(1)
   {

      /* if(grab_frame == 1) */
      /* { */
      /*    lepton_transfer(); */
      /*    grab_frame = 3; */
      /* } */

      /* process_rx_buffer(); */

      /* write_timestamps(); */


      /* handle_incoming_packets(); */
      /* write_outgoing(); */

      /* process_usart3_buffer(); */



      /* rs485_master_spin(); */
      /* rs485_slave_spin(); */
      full_duplex_usart_dma_spin();

      debug_output_toggle(DEBUG_LED_GREEN);


      if(tilt_stepper_motor_send_angle)
      {
         if(cts_pos_packet)
         {
            tilt_stepper_motor_pos(&pos_rad, &pos_ts);
            /* create_motor_resp_position(&gp_pos_rad, pos_rad); */
            create_motor_resp_position_ts(&gp_pos_rad, pos_rad, pos_ts);
            /**
             * @todo Need to set up the callback function so that we don't
             *       overwrite this packet with the next one.
             */
            cts_pos_packet = 0;
            full_duplex_usart_dma_add_to_queue(&gp_pos_rad, gpcbs_main_queue_callback, POS_PACKET_CALLBACK_NUM);
            tilt_stepper_motor_send_angle = 0;
         }
      }

      /* tilt_motor_get_angle(&pos_rad); */
      /* if(fabs(pos_rad - prev_pos_rad) > 0.03) */
      /* { */
      /*    create_motor_resp_position(&gp_pos_rad, pos_rad); */
      /*    usart_write_dma(gp_pos_rad.gp, gp_pos_rad.packet_length); */
      /*    prev_pos_rad = pos_rad; */
      /* } */

      /* if(send_motor_feedback) */
      /* { */
      /*    create_motor_feedback(&gp_mf, mf); */
      /*    usart_write_dma(gp_mf.gp, gp_mf.packet_length); */
      /*    send_motor_feedback = 0; */
      /* } */



      /* Just a test.  This will need to be a response to a request in the future. */
      if(send_code_version == 1)
      {

         /* /\* write_code_version(); *\/ */
         /* send_code_version = 0; */

         /* read_adc(&vc14, &vc15); */
         /* /\* create_analog_voltage(&gp, ANALOG_VOLTAGE, vc14); *\/ */
         /* /\* usart_write_dma(gp.gp, gp.packet_length); *\/ */
         /* create_analog_voltage(&gp_two, ANALOG_BATTERY_VOLTAGE, vc15); */
         /* usart_write_dma(gp_two.gp, gp_two.packet_length); */

         /* /\* quad_encoder_read_position(&pos_count); *\/ */
         /* /\* pos_rad = 6.28318530718f * (float)pos_count / (64.0f * 4.0f); *\/ */
         /* /\* create_universal_word(&gp_pos, pos_count); *\/ */
         /* /\* usart_write_dma(gp_pos.gp, gp_pos.packet_length); *\/ */

         /* /\* create_motor_resp_position(&gp_pos_rad, pos_rad); *\/ */
         /* /\* usart_write_dma(gp_pos_rad.gp, gp_pos_rad.packet_length); *\/ */

      }

      /* send_gp_packets(); */
   }

}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
   /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */


   /* Infinite loop */
   while (1)
   {

   }
}
#endif
