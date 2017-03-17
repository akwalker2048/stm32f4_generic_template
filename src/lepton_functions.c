#include "stm32f4xx_conf.h"
#include <stdint.h>
#include "hardware_STM32F407G_DISC1.h"
#include "lepton_functions.h"

/* Serial pc(SERIAL_TX, SERIAL_RX); */
/* SPI lepton_spi(SPI_MOSI, SPI_MISO, SPI_SCK); */
/* DigitalOut spi_cs(SPI_CS); */

#define VOSPI_NUM_FRAMES_IN_IMAGE (60)
#define VOSPI_FRAME_SIZE    (164)
#define VOSPI_RESET_TIME_MS (185)

#define VOSPI_ALL_IMAGE_FRAME_BYTES (VOSPI_NUM_FRAMES_IN_IMAGE*VOSPI_FRAME_SIZE)

uint8_t lepton_mega_packet[VOSPI_ALL_IMAGE_FRAME_BYTES];
uint8_t lepton_frame_packet[VOSPI_FRAME_SIZE];
int lepton_image[80][80];

int print_image_binary_state =-1;
int print_image_binary_i;
int print_image_binary_j;

void lepton_print_image_binary_background(void)
{


   if( print_image_binary_state == -1)
   {
      return;
   }
   else if( print_image_binary_state == 0)
   {
      usart_write_byte(0xDE);
      print_image_binary_state++;
   }
   else if( print_image_binary_state == 1)
   {
      usart_write_byte(0xAD);
      print_image_binary_state++;
   }
   else if( print_image_binary_state == 2)
   {
      usart_write_byte(0xBE);
      print_image_binary_state++;
   }
   else if( print_image_binary_state == 3)
   {
      usart_write_byte(0xEF);
      print_image_binary_state++;
      print_image_binary_i = 0;
      print_image_binary_j = 0;
   }
   else if( print_image_binary_state == 4)
   {
      usart_write_byte((lepton_image[print_image_binary_i][print_image_binary_j]>>8)&0xff);
      usart_write_byte(lepton_image[print_image_binary_i][print_image_binary_j]&0xff);

      print_image_binary_j++;
      if(print_image_binary_j>=80)
      {
         print_image_binary_j=0;
         print_image_binary_i++;
         if(print_image_binary_i>=60)
         {
            print_image_binary_state = -1;
         }
      }
   }
}



int lost_frame_counter = 0;
int last_frame_number;
int frame_complete = 0;
int start_image = 0;
int need_resync = 0;
int last_crc;
int new_frame = 0;
int frame_counter = 0;

void lepton_transfer(void)
{
   int i;
   int frame_number;

   GPIO_SetBits(GPIOD, LED_PIN_ORANGE);

   spi_cs_enable();
   /* for(i=0;i<VOSPI_FRAME_SIZE;i++) */
   /* { */
   /*    lepton_frame_packet[i] = spi_read_byte(); */
   /* } */

   for(i=0;i<VOSPI_ALL_IMAGE_FRAME_BYTES;i++)
   {
      lepton_mega_packet[i] = spi_read_byte();
   }
   spi_cs_disable();


   usart_write_byte(0xDE);
   usart_write_byte(0xAD);
   usart_write_byte(0xBE);
   usart_write_byte(0xEF);
   usart_write_byte(0xDE);
   usart_write_byte(0xAD);
   usart_write_byte(0xBE);
   usart_write_byte(0xEF);
   for(i=0;i<VOSPI_ALL_IMAGE_FRAME_BYTES;i++)
   {
      usart_write_byte(lepton_mega_packet[i]);
   }
   usart_write_byte(0xDE);
   usart_write_byte(0xAD);
   usart_write_byte(0xBE);
   usart_write_byte(0xEF);
   usart_write_byte(0xDE);
   usart_write_byte(0xAD);
   usart_write_byte(0xBE);
   usart_write_byte(0xEF);

   blocking_wait_ms(185);


   if(((lepton_frame_packet[0]&0xf) != 0x0f))
   {
      if(lepton_frame_packet[1] == 0  )
      {
         if(last_crc != (lepton_frame_packet[3]<<8 | lepton_frame_packet[4]))
         {
            new_frame = 1;
         }
         last_crc = lepton_frame_packet[3]<<8 | lepton_frame_packet[4];
      }
      frame_number = lepton_frame_packet[1];

      if(frame_number < 60 )
      {
         lost_frame_counter = 0;
         if(print_image_binary_state == -1)
         {
            for(i=0;i<80;i++)
            {
               lepton_image[frame_number][i] = (lepton_frame_packet[2*i+4] << 8 | lepton_frame_packet[2*i+5]);
            }
         }
      }
      else
      {
         lost_frame_counter++;
      }
      if( frame_number == 59)
      {
         frame_complete = 1;
         last_frame_number = 0;
      }
   }
   else
   {
      if(last_frame_number ==0)
      {
      }
   }

   lost_frame_counter++;
   if(lost_frame_counter>100)
   {
      need_resync = 1;
      lost_frame_counter = 0;

   }

   if(need_resync)
   {

      blocking_wait_ms(185);
      need_resync = 0;

   }


   if(frame_complete)
   {
      if(new_frame)
      {
         frame_counter++;
         {
            if(frame_counter%18 ==0)
            {
               print_image_binary_state = 0;
            }
         }
         new_frame = 0;
      }
      frame_complete = 0;
   }


   GPIO_ResetBits(GPIOD, LED_PIN_ORANGE);

}

