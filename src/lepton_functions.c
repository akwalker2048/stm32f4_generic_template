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


/** @todo This function should use the new circular_buffer.c code that has
 *  been added and debugged when I get time.
 */
#define VOSPI_CIRC_BUFFER_SIZE 128
volatile GenericPacket vospi_circ_buffer[VOSPI_CIRC_BUFFER_SIZE];
volatile uint32_t vospi_circ_buffer_head = 0;
volatile uint32_t vospi_circ_buffer_tail = 0;

uint8_t spi_initialized = 0;
uint8_t i2c_initialized = 0;


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

      /** @todo This function needs to be rewritten to use the new full_duplex_usart_dma
       *  functions when I have time...
       */
      /* usart_write_dma(thermal_packet.gp, thermal_packet.packet_length); */
      for(ii=0; ii<VOSPI_NUM_FRAMES_IN_IMAGE; ii++)
      {
         vospi_ptr = get_next_vospi_ptr();
         retval =  create_thermal_lepton_frame(vospi_ptr, &(frame[ii]));
         increment_vospi_head();
         write_vospi();
      }
      create_thermal_end_lepton_image(&thermal_packet);

      /** @todo This function needs to be rewritten to use the new full_duplex_usart_dma
       *  functions when I have time...
       */
      /* usart_write_dma(thermal_packet.gp, thermal_packet.packet_length); */
      image_num++;
   }
   else
   {
      create_thermal_image_timeout(&thermal_packet);
      /** @todo This function needs to be rewritten to use the new full_duplex_usart_dma
       *  functions when I have time...
       */
      /* usart_write_dma(thermal_packet.gp, thermal_packet.packet_length); */
      lepton_image_timeout = 0;
   }




   GPIO_ResetBits(GPIOD, LED_PIN_ORANGE);

}





/** @todo Get all of these hardware related functions moved over here. And
 *  also get them working.  These have been moved from the old hardware_*
 *  file and have not yet been tested or debugged.  That is because there
 *  is currently no pressure to get this working.
 *  - Timer interrupt to control when to grab frames
 *  - SPI configuration/init
 *  - SPI communications
 */

/* This is a snippet that used to be in the SysTick handler. */
/* extern uint8_t lepton_phase_offset; */
/* /\* if(ms_counter%(2000) == 0) *\/ */
/* if((ms_counter + lepton_phase_offset)%(185) == 0) */
/* { */
/*    /\* Kick off one frame grab. *\/ */
/*    if(grab_frame == 2) */
/*    { */
/*       grab_frame = 1; */
/*    } */

/*    if(grab_frame == 3) */
/*    { */
/*       grab_frame = 2; */
/*    } */

/* } */

void spi_cs_enable(void)
{
   GPIO_ResetBits(GPIOB, SPI_PIN_CS_AL);
   blocking_wait_ms(1);
}

void spi_cs_disable(void)
{
   GPIO_SetBits(GPIOB, SPI_PIN_CS_AL);
}

uint8_t spi_read_byte(void)
{
   uint8_t retval;
   uint16_t tmpval;

   while(SPI_I2S_GetFlagStatus(SPI3, SPI_FLAG_TXE) == RESET);
   SPI_I2S_SendData(SPI3, (uint16_t)0x00);
   while(SPI_I2S_GetFlagStatus(SPI3, SPI_FLAG_RXNE) == RESET);
   tmpval = SPI_I2S_ReceiveData(SPI3);
   retval = (uint8_t)(tmpval & 0xFF);

   return retval;

}


void write_vospi(void)
{
   uint8_t ii;

   /** @todo This function needs to be rewritten to use the new full_duplex_usart_dma
    *  functions when I have time...
    */

   /* while(vospi_circ_buffer_tail != vospi_circ_buffer_head) */
   /* { */
   /*    vospi_circ_buffer_tail = vospi_circ_buffer_tail + 1; */
   /*    if(vospi_circ_buffer_tail >= VOSPI_CIRC_BUFFER_SIZE) */
   /*    { */
   /*       vospi_circ_buffer_tail = 0; */
   /*    } */

   /*    if(usart_one_initialized) */
   /*    { */
   /*       for(ii=0; ii<vospi_circ_buffer[vospi_circ_buffer_tail].packet_length; ii++) */
   /*       { */
   /*          usart_write_byte(vospi_circ_buffer[vospi_circ_buffer_tail].gp[ii]); */
   /*       } */
   /*    } */
   /*    else if(usart_one_dma_initialized) */
   /*    { */
   /*       usart_write_dma((uint8_t *)vospi_circ_buffer[vospi_circ_buffer_tail].gp, vospi_circ_buffer[vospi_circ_buffer_tail].packet_length); */
   /*    } */
   /* } */
}


GenericPacket * get_next_vospi_ptr(void)
{
   GenericPacket *ptr;
   uint32_t temp_head;

   temp_head = vospi_circ_buffer_head + 1;
   if(temp_head >= VOSPI_CIRC_BUFFER_SIZE)
   {
      temp_head = 0;
   }
   ptr = (GenericPacket *)&(vospi_circ_buffer[temp_head]);

   return ptr;

}

void increment_vospi_head(void)
{
   vospi_circ_buffer_head = vospi_circ_buffer_head + 1;
   if(vospi_circ_buffer_head >= VOSPI_CIRC_BUFFER_SIZE)
   {
      vospi_circ_buffer_head = 0;
   }
}


/* ****************************************************************** */
/* SPI Initialization                                                 */
/* ****************************************************************** */
void init_spi(void)
{

   SPI_InitTypeDef  SPI_InitStructure;
   GPIO_InitTypeDef GPIO_InitStructure;
   /* NVIC_InitTypeDef NVIC_InitStructure; */

   /* Peripheral Clock Enable -------------------------------------------------*/
   /* Enable the SPI clock */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

   /* Enable GPIO clocks */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

   /* SPI GPIO Configuration --------------------------------------------------*/
   /* GPIO Deinitialisation */  /* No...because other GPIO on Port B may be already configured. */
   /* GPIO_DeInit(GPIOB); */


   /* Connect SPI pins to AF5 */
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);

   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

   /* SPI SCK pin configuration */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* SPI  MISO pin configuration */
   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* SPI  MOSI pin configuration */
   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* SPI  Chip Select Configuration */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
   GPIO_InitStructure.GPIO_Pin = SPI_PIN_CS_AL;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   GPIO_SetBits(GPIOB, SPI_PIN_CS_AL);

   /* SPI configuration -------------------------------------------------------*/
   SPI_I2S_DeInit(SPI3);
   SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
   SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
   SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
   SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
   SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
   SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
   SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
   SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
   SPI_InitStructure.SPI_CRCPolynomial = 7;
   SPI_Init(SPI3, &SPI_InitStructure);

   /* Not sure we need to operate this in interrupt mode... */
   /* /\* Configure the Priority Group to 1 bit *\/ */
   /* NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); */
   /* /\* Configure the SPI interrupt priority *\/ */
   /* NVIC_InitStructure.NVIC_IRQChannel = SPI3_IRQn; */
   /* NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; */
   /* NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; */
   /* NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; */
   /* NVIC_Init(&NVIC_InitStructure); */

   /* Enable the SPI peripheral */
   SPI_Cmd(SPI3, ENABLE);

   spi_initialized = 1;

}


/* ****************************************************************** */
/* I2C Initialization                                                 */
/* ****************************************************************** */
void init_i2c(void)
{

   GPIO_InitTypeDef GPIO_InitStructure;
   I2C_InitTypeDef I2C_InitStructure;

   /* I2C1:  SCL->PB8, SDA->PB9 */
   /* RCC Configuration */
   /*I2C Peripheral clock enable */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

   /* Reset I2Cx IP */
   RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);

   /* Release reset signal of I2Cx IP */
   RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

   /* GPIO Configuration */
   /*Configure I2C SCL pin */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /*Configure I2C SDA pin */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Connect PXx to I2C_SCL */
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);

   /* Connect PXx to I2C_SDA */
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

   /* Configure I2C Filters */
   I2C_AnalogFilterCmd(I2C1, ENABLE);
   I2C_DigitalFilterConfig(I2C1, 0x0F);

   /* I2C Struct Initialize */
   I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
   I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
   I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
   I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
   I2C_InitStructure.I2C_ClockSpeed = 100000;
   I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

   /* I2C Initialize */
   I2C_Init(I2C1, &I2C_InitStructure);

   i2c_initialized = 1;

}

