#include "stm32f4xx_conf.h"
#include <stdint.h>
#include "hardware_STM32F407G_DISC1.h"
#include "lepton_functions.h"

/* Serial pc(SERIAL_TX, SERIAL_RX); */
/* SPI lepton_spi(SPI_MOSI, SPI_MISO, SPI_SCK); */
/* DigitalOut spi_cs(SPI_CS); */

#define VOSPI_NUM_FRAMES_IN_IMAGE (60)
/* #define VOSPI_NUM_FRAMES_IN_IMAGE (180) */

#define VOSPI_FRAME_SIZE    (164)
#define VOSPI_RESET_TIME_MS (185)

#define VOSPI_ALL_IMAGE_FRAME_BYTES (VOSPI_NUM_FRAMES_IN_IMAGE*VOSPI_FRAME_SIZE)

/* uint8_t lepton_mega_packet[VOSPI_ALL_IMAGE_FRAME_BYTES]; */
/* uint8_t lepton_frame_packet[VOSPI_FRAME_SIZE]; */
/* int lepton_image[80][80]; */

int print_image_binary_state =-1;
int print_image_binary_i;
int print_image_binary_j;

uint8_t lepton_phase_offset = 0;
uint8_t lepton_image_timeout = 0;

extern uint32_t ms_counter;

void lepton_print_image_binary_background(void)
{


   /* if( print_image_binary_state == -1) */
   /* { */
   /*    return; */
   /* } */
   /* else if( print_image_binary_state == 0) */
   /* { */
   /*    usart_write_byte(0xDE); */
   /*    print_image_binary_state++; */
   /* } */
   /* else if( print_image_binary_state == 1) */
   /* { */
   /*    usart_write_byte(0xAD); */
   /*    print_image_binary_state++; */
   /* } */
   /* else if( print_image_binary_state == 2) */
   /* { */
   /*    usart_write_byte(0xBE); */
   /*    print_image_binary_state++; */
   /* } */
   /* else if( print_image_binary_state == 3) */
   /* { */
   /*    usart_write_byte(0xEF); */
   /*    print_image_binary_state++; */
   /*    print_image_binary_i = 0; */
   /*    print_image_binary_j = 0; */
   /* } */
   /* else if( print_image_binary_state == 4) */
   /* { */
   /*    usart_write_byte((lepton_image[print_image_binary_i][print_image_binary_j]>>8)&0xff); */
   /*    usart_write_byte(lepton_image[print_image_binary_i][print_image_binary_j]&0xff); */

   /*    print_image_binary_j++; */
   /*    if(print_image_binary_j>=80) */
   /*    { */
   /*       print_image_binary_j=0; */
   /*       print_image_binary_i++; */
   /*       if(print_image_binary_i>=60) */
   /*       { */
   /*          print_image_binary_state = -1; */
   /*       } */
   /*    } */
   /* } */
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
   int ii, jj;
   int last_frame_number;
   int resets;

   GenericPacket *vospi_ptr;
   VOSPIFrame frame[VOSPI_NUM_FRAMES_IN_IMAGE];

   uint8_t retval;

   GenericPacket thermal_packet;

   static uint16_t image_num = 0;

   GPIO_SetBits(GPIOD, LED_PIN_ORANGE);

   last_frame_number = -1;
   resets = 0;


   spi_cs_enable();
   for(ii=0; ii<VOSPI_NUM_FRAMES_IN_IMAGE; ii++)
   {


      for(jj=0; jj<VOSPI_FRAME_SIZE; jj++)
      {
         frame[ii].data[jj] = spi_read_byte();
      }

      /* Check if the frame is a discard frame and chuck. */
      if((frame[ii].data[0] & 0x0F) == 0x0F)
      {
         ii--;
      }
      else
      {
         last_frame_number = frame[ii].data[1];
         if(last_frame_number != ii)
         {
            /* We aren't at the start of an image yet. */
            ii=-1;
            resets++;
            blocking_wait_ms(1);
         }
      }

      if(resets == 750)
      {
         ii = VOSPI_NUM_FRAMES_IN_IMAGE + 1;
         lepton_phase_offset++;
         lepton_image_timeout = 1;
      }

   }
   spi_cs_disable();

   if(lepton_image_timeout == 0)
   {
      create_thermal_begin_lepton_image(&thermal_packet, image_num, ms_counter);
      usart_write_dma(thermal_packet.gp, thermal_packet.packet_length);
      for(ii=0; ii<VOSPI_NUM_FRAMES_IN_IMAGE; ii++)
      {
         vospi_ptr = get_next_vospi_ptr();
         retval =  create_thermal_lepton_frame(vospi_ptr, &(frame[ii]));
         increment_vospi_head();
         write_vospi();
      }
      create_thermal_end_lepton_image(&thermal_packet);
      usart_write_dma(thermal_packet.gp, thermal_packet.packet_length);
      image_num++;
   }
   else
   {
      create_thermal_image_timeout(&thermal_packet);
      usart_write_dma(thermal_packet.gp, thermal_packet.packet_length);
      lepton_image_timeout = 0;
   }




   GPIO_ResetBits(GPIOD, LED_PIN_ORANGE);

}

