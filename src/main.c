/**
******************************************************************************
* @file    stm32f4_generic_template/main.c
* @author  Andrew Walker
* @version V0.0.1
* @date    01MAR2017
* @brief   Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2017 Badger Technologies, LLC</center></h2>
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <math.h>

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#include "main.h"
#include "hardware_STM32F407G_DISC1.h"

#include "full_duplex_usart_dma.h"
#include "rx_packet_handler.h"

#include "tilt_motor_control.h"
#include "lepton_functions.h"

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
   GenericPacket gp, gp_two, gp_pos, gp_pos_rad, gp_mf;

   uint32_t pos_count;
   float pos_rad, prev_pos_rad;

   /* SystemCoreClockUpdate(); */

   init_gpio();
   /* init_usart_one(); */
   /* init_usart_one_dma(); */

   /* rx_packet_handler_init() must be called before full_duplex_usart_dma_init()
    * or else we won't be ready to get packet handler callbacks...
    */
   rx_packet_handler_init();
   full_duplex_usart_dma_init(rx_packet_handler_ptr);

   /* init_usart_three(); */
   init_pushbutton();

   /* init_spi(); */
   /* init_i2c(); */

   init_systick();
   init_adc();

   /* Cannot RS485 and Tilt!!!! Pin A2 */
   rs485_sensor_bus_init_master();
   rs485_sensor_bus_init_slave();


   /* Cannot RS485 and Tilt!!!! Pin A2 */
   /* tilt_motor_init(); */


   /* tilt_motor_get_angle(&pos_rad); */
   prev_pos_rad = pos_rad;

   grab_frame = 3;

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


      rs485_master_process_rx_ram();
      rs485_master_handle_packets();
      /* /\* rs485_write_outgoing_master(); *\/ */

      rs485_slave_process_rx_ram();
      rs485_slave_handle_packets();
      /* /\* rs485_write_outgoing_slave(); *\/ */

      /* At least figure out if we got here... */
      if(GPIO_ReadInputDataBit(GPIOD, LED_PIN_ORANGE) == Bit_SET)
      {
         GPIO_ResetBits(GPIOD, LED_PIN_ORANGE);
      }
      else
      {
         GPIO_SetBits(GPIOD, LED_PIN_ORANGE);
      }


      /* full_duplex_usart_dma_service(); */


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
