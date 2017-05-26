#include "full_duplex_usart_interrupt.h"

/* This file contains code that is being migrated from
 * hardware_STM32F407G_DISC1.c... Several things need to be done before it is
 * ready...Including:
 *  1) Rename things to include "full_duplex_usart_interrupt_****"
 *  2) Make all of it more self contained...like...
 *     full_duplex_usart_dma.c...which is ahead of this one in development
 *     and debugging...
 *
 */
#define USART3_CIRC_BUFFER_SIZE 256
volatile uint8_t usart3_circ_buffer[USART3_CIRC_BUFFER_SIZE];
volatile uint8_t usart3_circ_buffer_head = 0;
volatile uint8_t usart3_circ_buffer_tail = 0;

uint8_t full_duplex_usart_initialized = 0;

/* ****************************************************************** */
/* USART Initialization                                               */
/* ****************************************************************** */
void init_usart_three(void)
{
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
   GPIO_InitTypeDef  GPIO_InitStructure;

   /* Init USART Pins */
   /* USART3 has three pinouts on this card:
    *   TX -> B10 | C10 | D8
    *   RX -> B11 | C11 | D9
    *
    *   I'll start with D8 and D9 because those pins aren't mapped to many
    *   other functions.
    */

   /* Enable GPIO clock */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
   /* Connect PXx to USARTx_Tx*/
   GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
   /* Connect PXx to USARTx_Rx */
   GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
   /* Configure USART Tx as alternate function  */
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
   GPIO_Init(GPIOD, &GPIO_InitStructure);
   /* Configure USART Rx as alternate function  */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_Init(GPIOD, &GPIO_InitStructure);

   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

   /* USART_InitStructure.USART_BaudRate = 115200; */
   USART_InitStructure.USART_BaudRate = 9600;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   /* USART_InitStructure.USART_Mode = USART_Mode_Rx; */

   USART_OverSampling8Cmd(USART3, ENABLE);

   /* USART configuration */
   USART_Init(USART3, &USART_InitStructure);

   /* Enable the USARTx Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   /* Enable the interrupt for Receive Not Empty */
   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

   /* Enable USART */
   USART_Cmd(USART3, ENABLE);

   usart_three_initialized = 1;

   /* /\* AT+BAUD8 -> Change to 115200 *\/ */
   /* usart_three_write_byte('A'); */
   /* usart_three_write_byte('T'); */
   /* usart_three_write_byte('+'); */
   /* usart_three_write_byte('B'); */
   /* usart_three_write_byte('A'); */
   /* usart_three_write_byte('U'); */
   /* usart_three_write_byte('D'); */
   /* usart_three_write_byte('8'); */

   /*  /\* /\\* AT+BAUD8 -> Change to 115200 *\\/ *\/ */
   /* usart_three_write_byte('A'); */
   /* usart_three_write_byte('T'); */
   /* usart_three_write_byte('+'); */
   /* usart_three_write_byte('B'); */
   /* usart_three_write_byte('A'); */
   /* usart_three_write_byte('U'); */
   /* usart_three_write_byte('D'); */
   /* usart_three_write_byte('8'); */

   /* /\* Reconfigure things! *\/ */
   /* usart_three_initialized = 0; */
   /* USART_Cmd(USART3, DISABLE); */
   /* USART_InitStructure.USART_BaudRate = 115200; */
   /* USART_Init(USART3, &USART_InitStructure); */
   /* USART_Cmd(USART3, ENABLE); */
   /* usart_three_initialized = 1; */

}



/* ****************************************************************** */
/* USART3 Handler                                                     */
/* ****************************************************************** */
void USART3_IRQHandler(void)
{
   uint16_t tchar;
   uint8_t temp_head;

   if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
   {
      /* Read one byte from the receive data register */
      tchar = (USART_ReceiveData(USART3) & 0x7F);

      /* Just drop the byte in a circular buffer and deal with it later. */
      temp_head = usart3_circ_buffer_head + 1;
      if(temp_head >= USART3_CIRC_BUFFER_SIZE)
      {
         temp_head = 0;
      }
      usart3_circ_buffer[temp_head] = (uint8_t)tchar;
      /* usart3_circ_buffer[temp_head] = ~usart3_circ_buffer[temp_head]; */
      usart3_circ_buffer_head = temp_head;


      /* Echo what we just got! */
      /* USART_SendData(USART3, tchar); */
      /* usart_write_byte((uint8_t)tchar); */
   }

}



uint8_t usart_three_write_byte(uint8_t data)
{

   if(usart_three_initialized)
   {
      /* Make sure any previous write has completed. */
      while( !(USART3->SR & 0x00000040) );
      /* Send the data. */
      /* if((0x20 <= data)&&(data <= 0x7E)) */
      /* { */
      USART_SendData(USART3, (uint16_t)data);
      /* } */
   }

   return 0;
}





void process_usart3_buffer(void)
{
   GenericPacket packet;
   char tchar[2];

   /* Debug Sonar Packet! */
   /* sonar_data[0] = 0x31; */
   /* sonar_data[1] = 0x32; */
   /* sonar_data[2] = 0x33; */
   /* sonar_data[3] = 0; */
   /* create_sonar_maxbot_serial(&packet, (char *)sonar_data); */
   /* usart_write_dma(packet.gp, packet.packet_length); */

   while(usart3_circ_buffer_tail != usart3_circ_buffer_head)
   {
      /* if(GPIO_ReadInputDataBit(GPIOD, LED_PIN_RED) == Bit_SET) */
      /* { */
      /*    GPIO_ResetBits(GPIOD, LED_PIN_RED); */
      /* } */
      /* else */
      /* { */
      /*    GPIO_SetBits(GPIOD, LED_PIN_RED); */
      /* } */

      usart3_circ_buffer_tail++;
      if(usart3_circ_buffer_tail >= USART3_CIRC_BUFFER_SIZE)
      {
         usart3_circ_buffer_tail = 0;
      }

      /* Temp Debug */
      tchar[0] = usart3_circ_buffer[usart3_circ_buffer_tail];
      tchar[1] = 0;
      create_universal_str(&packet, tchar);
      usart_write_dma(packet.gp, packet.packet_length);

      switch(usart3_circ_buffer[usart3_circ_buffer_tail])
      {
         case 0x52:
            /* This is an R!  The next byte will be sonar data. */
            sonar_index = 0;
            break;
         case 0x0D:
            /* This is a Carriage Return!  This ends this set of sonar data. */
            sonar_index = 3;
            sonar_data[sonar_index] = 0;
            create_sonar_maxbot_serial(&packet, (char *)sonar_data);
            usart_write_dma(packet.gp, packet.packet_length);
            sonar_index = 0;
            break;
         default:
            /* Add characters to the next packet! */
            if(sonar_index < 4)
            {
               sonar_data[sonar_index] = usart3_circ_buffer[usart3_circ_buffer_tail];
               sonar_index++;
            }
            else
            {
               /* sonar_data[3] = 0; */
               /* create_sonar_maxbot_serial(&packet, (char *)sonar_data); */
               /* usart_write_dma(packet.gp, packet.packet_length); */
            }
            break;
      }

   }

}
