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
#include "main.h"
#include "hardware_STM32F407G_DISC1.h"
#include "hardware_TB6612.h"
#include "lepton_functions.h"

#include "generic_packet.h"
#include "gp_proj_analog.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

extern volatile uint8_t grab_frame;
extern volatile uint8_t send_code_version;

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

   float vc14, vc15;
   GenericPacket gp, gp_two;

   init_gpio();
   /* init_usart_one(); */
   init_usart_one_dma();
   init_usart_three();
   init_pushbutton();
   init_spi();
   init_i2c();
   init_systick();
   init_adc();

   TB6612_initialize();

   GPIO_SetBits(GPIOD, LED_PIN_RED);
   blocking_wait_ms(1000);
   GPIO_ResetBits(GPIOD, LED_PIN_RED);
   blocking_wait_ms(1000);
   /* GPIO_SetBits(GPIOD, LED_PIN_RED); */
   /* blocking_wait_ms(1000); */
   /* GPIO_ResetBits(GPIOD, LED_PIN_RED); */
   /* blocking_wait_ms(1000); */
   /* GPIO_SetBits(GPIOD, LED_PIN_RED); */
   /* blocking_wait_ms(1000); */
   /* GPIO_ResetBits(GPIOD, LED_PIN_RED); */



   non_blocking_wait_ms(185);

   grab_frame = 3;

   while(1)
   {

      /* if(grab_frame == 1) */
      /* { */
      /*    lepton_transfer(); */
      /*    grab_frame = 3; */
      /* } */

      process_rx_buffer();

      /* write_timestamps(); */

      write_outgoing();

      process_usart3_buffer();

      /* Just a test.  This will need to be a response to a request in the future. */
      if(send_code_version == 1)
      {
         /* write_code_version(); */
         send_code_version = 0;

         read_adc(&vc14, &vc15);
         /* create_analog_voltage(&gp, ANALOG_VOLTAGE, vc14); */
         /* usart_write_dma(gp.gp, gp.packet_length); */
         create_analog_voltage(&gp_two, ANALOG_BATTERY_VOLTAGE, vc15);
         usart_write_dma(gp_two.gp, gp_two.packet_length);

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
