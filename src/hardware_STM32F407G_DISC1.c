#include "stm32f4xx_conf.h"
#include "hardware_STM32F407G_DISC1.h"

/* Global variable for handling USART so that whole packets are sent at a
 * a time.  In the future, I need to just add a function that is in this
 * file that controls all packet sending.  Maybe implement a circular
 * packet buffer for sending.
 */
volatile uint8_t packet_send_mutex = 0;

volatile uint8_t grab_frame = 0;
volatile uint8_t send_code_version = 0;

#define TS_CIRC_BUFFER_SIZE 8
volatile GenericPacket ts_circ_buffer[TS_CIRC_BUFFER_SIZE];
volatile uint32_t ts_circ_buffer_head = 0;
volatile uint32_t ts_circ_buffer_tail = 0;

#define VOSPI_CIRC_BUFFER_SIZE 128
volatile GenericPacket vospi_circ_buffer[VOSPI_CIRC_BUFFER_SIZE];
volatile uint32_t vospi_circ_buffer_head = 0;
volatile uint32_t vospi_circ_buffer_tail = 0;


/* Variables Global Within This File */
uint8_t gpio_initialized = 0;
uint8_t systick_initialized = 0;
uint8_t usart_one_initialized = 0;
uint8_t usart_one_dma_initialized = 0;
uint8_t usart_three_initialized = 0;
uint8_t pushbutton_initialized = 0;
uint8_t analog_input_initialized = 0;
uint8_t tia_initialized = 0;
uint8_t spi_initialized = 0;
uint8_t i2c_initialized = 0;

/* Circular Receive Buffer */
#define RX_BUFFER_SIZE (GP_MAX_PACKET_LENGTH * 4)
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint16_t rx_buffer_head = 0;
uint16_t rx_buffer_tail = 0;

/* USART DMA Buffers */
uint8_t usart_dma_tx_buffer[GP_MAX_PACKET_LENGTH];
uint8_t usart_dma_rx_buffer[GP_MAX_PACKET_LENGTH];

/* Free Running Coutner */
volatile uint32_t ms_counter = 0;
/* Resettable Counter */
volatile uint32_t ms_count_r = 0;

/* ****************************************************************** */
/* GPIO Initialization                                                */
/* ****************************************************************** */
/* Only purely GPIO should be initialized here.  Other pins should be *
 * initialized in their respective init functions as needed.          *
 * ****************************************************************** */
void init_gpio(void)
{
   GPIO_InitTypeDef  GPIO_InitStructure;

   /* Init LEDs */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
   GPIO_InitStructure.GPIO_Pin = LED_PIN_RED | LED_PIN_GREEN | LED_PIN_BLUE | LED_PIN_ORANGE;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOD, &GPIO_InitStructure);

   gpio_initialized = 1;
}

/* ****************************************************************** */
/* TIM Interrupt Initialization                                       */
/* ****************************************************************** */
void init_systick(void)
{
   /* Make sure we are using the correct value for SystemCoreClock! */
   SystemCoreClockUpdate();
   /* Set SysTick to expire every ms. */
   if (SysTick_Config(SystemCoreClock / 1000))
   {
      /* Capture error */
      while (1);
   }

   systick_initialized = 1;
}

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
   USART_InitStructure.USART_BaudRate = 115200;
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

}

void init_usart_one(void)
{
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
   GPIO_InitTypeDef  GPIO_InitStructure;

   /* Init USART Pins */
   /* USART1:
    *   TX -> B6
    *   RX -> B7
    */

   /* Enable the USART Clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

   /* Enable GPIO clock */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
   /* Connect PXx to USARTx_Tx*/
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
   /* Connect PXx to USARTx_Rx*/
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
   /* Configure USART Tx as alternate function */
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   /* Configure USART Rx as alternate function  */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* USART_InitStructure.USART_BaudRate = 115200; */
   USART_InitStructure.USART_BaudRate = 3000000;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   /* USART_InitStructure.USART_Mode = USART_Mode_Tx; */

   USART_OverSampling8Cmd(USART1, ENABLE);

   /* USART configuration */
   USART_Init(USART1, &USART_InitStructure);

   /* Enable the USARTx Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   /* Enable the interrupt for Receive Not Empty */
   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

   /* Enable USART */
   USART_Cmd(USART1, ENABLE);

   usart_one_initialized = 1;
   usart_one_dma_initialized = 0;

}



void init_usart_one_dma(void)
{
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
   GPIO_InitTypeDef  GPIO_InitStructure;
   DMA_InitTypeDef  DMA_InitStructure;

   /* Init USART Pins */
   /* USART1:
    *   TX -> B6
    *   RX -> B7
    */

   /* Enable DMA Clock */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
   /* Enable the USART Clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

   /* Enable GPIO clock */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
   /* Connect PXx to USARTx_Tx*/
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
   /* Connect PXx to USARTx_Rx*/
   GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
   /* Configure USART Tx as alternate function */
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   /* Configure USART Rx as alternate function  */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_Init(GPIOB, &GPIO_InitStructure);


   /* USART_InitStructure.USART_BaudRate = 115200; */
   USART_InitStructure.USART_BaudRate = 3000000;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   /* USART_InitStructure.USART_Mode = USART_Mode_Tx; */

   USART_OverSampling8Cmd(USART1, ENABLE);

   /* USART configuration */
   USART_Init(USART1, &USART_InitStructure);

   /* /\* Enable the USARTx Interrupt *\/ */
   /* NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; */
   /* NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; */
   /* NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; */
   /* NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; */
   /* NVIC_Init(&NVIC_InitStructure); */

   /* /\* Enable the interrupt for Receive Not Empty *\/ */
   /* USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); */


   /* Set up DMA Here!!!! */
   DMA_InitStructure.DMA_BufferSize = GP_MAX_PACKET_LENGTH;
   DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
   DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
   DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(USART1->DR));
   DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
   /* Configure TX DMA */
   DMA_InitStructure.DMA_Channel = DMA_Channel_4;
   DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart_dma_tx_buffer;
   DMA_Init(DMA2_Stream7, &DMA_InitStructure);
   /* Configure RX DMA */
   DMA_InitStructure.DMA_Channel = DMA_Channel_4;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart_dma_rx_buffer;
   DMA_Init(DMA2_Stream5, &DMA_InitStructure);

   /* Enable USART */
   USART_Cmd(USART1, ENABLE);

   usart_one_dma_initialized = 1;
   usart_one_initialized = 0;
}






/* ****************************************************************** */
/* Pushbutton  Initialization                                         */
/* ****************************************************************** */
void init_pushbutton(void)
{

   GPIO_InitTypeDef  GPIO_InitStructure;
   EXTI_InitTypeDef   EXTI_InitStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;

   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOD, &GPIO_InitStructure);

   /* Connect EXTI Line0 to PA0 pin */
   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

   /* Configure EXTI Line0 */
   EXTI_InitStructure.EXTI_Line = EXTI_Line0;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   /* Enable and set EXTI Line0 Interrupt to the lowest priority */
   NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);



   pushbutton_initialized = 1;

}

/* ****************************************************************** */
/* Analog Input Initialization                                        */
/* ****************************************************************** */
void init_analog_input(void)
{


   analog_input_initialized = 1;

}

/* ****************************************************************** */
/* TIA Initialization                                                 */
/* ****************************************************************** */
void init_tia(void)
{

   /* Init TIM5 Pins for Capture/Compare...TIA Application */
   /* TIM5_CH1-> A0  !!CH1 and A0 not used...using the pushbutton feature!!
    * TIM5_CH2-> A1
    * TIM5_CH3-> A2
    * TIM5_CH4-> A3
    */

   tia_initialized = 1;

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


/* ****************************************************************** */
/* ????  Initialization                                               */
/* ****************************************************************** */





/* ****************************************************************** */
/* SysTick Handler  (moved from stm32f4xx_it.c)                       */
/* ****************************************************************** */
void SysTick_Handler(void)
{
   static uint32_t ii = 0;

   uint8_t temp_head;

   ms_counter++;
   ms_count_r++;

   ii++;

   /* if(ms_counter%(6*37) == 0) */
   if(ms_counter%(185) == 0)
   {
      /* Kick off one frame grab. */
      if(grab_frame == 2)
      {
         grab_frame = 1;
      }

      if(grab_frame == 3)
      {
         grab_frame = 2;
      }

   }

   if(ms_counter%100 == 0)
   {

      temp_head = ts_circ_buffer_head + 1;
      if(temp_head >= TS_CIRC_BUFFER_SIZE)
      {
         temp_head = 0;
      }
      create_universal_timestamp((GenericPacket *)&ts_circ_buffer[temp_head], ms_counter);
      ts_circ_buffer_head = temp_head;

      /* for(ii=0; ii<packet.packet_length; ii++) */
      /* { */
      /*    usart_write_byte(packet.gp[ii]); */
      /* } */
      /* packet_send_mutex = 0; */

   }



   if(ms_counter%500 == 0)
   {

      send_code_version = 1;

      if(GPIO_ReadInputDataBit(GPIOD, LED_PIN_GREEN) == Bit_SET)
      {
         GPIO_ResetBits(GPIOD, LED_PIN_GREEN);
         GPIO_SetBits(GPIOD, LED_PIN_BLUE);

      }
      else
      {
         GPIO_SetBits(GPIOD, LED_PIN_GREEN);
         GPIO_ResetBits(GPIOD, LED_PIN_BLUE);

      }


   }


}


/* ****************************************************************** */
/* USART3 Handler                                                     */
/* ****************************************************************** */
void USART3_IRQHandler(void)
{
   uint16_t tchar;

   if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
   {
      /* Read one byte from the receive data register */
      tchar = (USART_ReceiveData(USART3) & 0x7F);

      /* Echo what we just got! */
      /* USART_SendData(USART3, tchar); */
      usart_write_byte((uint8_t)tchar);
   }

}

void USART1_IRQHandler(void)
{
   uint16_t tchar;

   if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
   {
      /* Read one byte from the receive data register */
      rx_buffer[rx_buffer_head] = (uint8_t)(USART_ReceiveData(USART1) & 0x7F);
      rx_buffer_head++;
      if(rx_buffer_head >= RX_BUFFER_SIZE)
      {
         rx_buffer_head = 0;
      }
      /* Echo what we just got! */
      /* USART_SendData(USART3, tchar); */
      /* usart_write_byte((uint8_t)tchar); */
   }

}



/* ****************************************************************** */
/* Pushbutton Handler                                                 */
/* ****************************************************************** */
void EXTI0_IRQHandler(void)
{
   if(EXTI_GetITStatus(EXTI_Line0) != RESET)
   {
      /* Toggle LED1 */
      if(GPIO_ReadInputDataBit(GPIOD, LED_PIN_ORANGE) == Bit_SET)
      {
         GPIO_ResetBits(GPIOD, LED_PIN_ORANGE);
      }
      else
      {
         GPIO_SetBits(GPIOD, LED_PIN_ORANGE);
      }

      /* Clear the EXTI line 0 pending bit */
      EXTI_ClearITPendingBit(EXTI_Line0);
   }
}



/* ****************************************************************** */
/* Exported Functions                                                 */
/* ****************************************************************** */
uint8_t usart_write_dma(uint8_t *data_ptr, uint32_t data_len)
{

   if(usart_one_dma_initialized)
   {

      /* Enable USART DMA TX Requsts */
      USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
      /* Enable the DMA */
      DMA_Cmd(DMA2_Stream7, ENABLE);


      /* /\* Wait for any previous transfer to complete. *\/ */
      while (USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);
      while (DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7)==RESET);

      /* Enable the DMA */
      DMA_Cmd(DMA2_Stream7, DISABLE);
      /* Disable USART DMA TX Requsts */
      USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);

      /* Clear DMA Transfer Complete Flags */
      DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
      /* Clear USART Transfer Complete Flags */
      USART_ClearFlag(USART1, USART_FLAG_TC);

      /* Set the length of data to transmit. */
      DMA2_Stream7->NDTR = data_len;
      /* Set the pointer to the data. */
      DMA2_Stream7->M0AR = (uint32_t)data_ptr;

      /* Enable USART DMA TX Requsts */
      USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
      /* Enable the DMA */
      DMA_Cmd(DMA2_Stream7, ENABLE);

   }

}

uint8_t usart_write_byte(uint8_t data)
{
   /* if(usart_three_initialized) */
   /* { */
   /*    /\* Make sure any previous write has completed. *\/ */
   /*    while( !(USART3->SR & 0x00000040) ); */
   /*    /\* Send the data. *\/ */
   /*    /\* if((0x20 <= data)&&(data <= 0x7E)) *\/ */
   /*    /\* { *\/ */
   /*       USART_SendData(USART3, (uint16_t)data); */
   /*    /\* } *\/ */
   /* } */

   if(usart_one_initialized)
   {
      /* Make sure any previous write has completed. */
      while( !(USART1->SR & 0x00000040) );
      /* Send the data. */
      /* if((0x20 <= data)&&(data <= 0x7E)) */
      /* { */
      USART_SendData(USART1, (uint16_t)data);
      /* } */
   }

   return 0;
}


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


void blocking_wait_ms(uint32_t delay_ms)
{
   ms_count_r = 0;
   while(ms_count_r < delay_ms);
}


void non_blocking_wait_ms(uint32_t delay_ms)
{
   /* Need to implement this! */
   /* ms_count_r = 0; */
   /* while(ms_count_r < delay_ms); */
}

void process_rx_buffer(void)
{
   while(rx_buffer_head != rx_buffer_tail)
   {
      /* usart_write_byte(rx_buffer[rx_buffer_tail]); */
      rx_buffer_tail++;
      if(rx_buffer_tail >= RX_BUFFER_SIZE)
      {
         rx_buffer_tail = 0;
      }
   }
}


/* uint8_t add_gp_to_circ_buffer(GenericPacket packet) */
/* { */
/*    uint8_t retval; */
/*    uint8_t temp_head; */
/*    uint8_t ii; */

/*    uint32_t prim; */

/*    /\* Read PRIMASK register, check interrupt status before you disable them *\/ */
/*    /\* Returns 0 if they are enabled, or non-zero if disabled *\/ */
/*    prim = __get_PRIMASK(); */

/*    /\* Disable interrupts *\/ */
/*    __disable_irq(); */

/*    while(packet_send_mutex != 0); */
/*    packet_send_mutex = 1; */
/*    temp_head = gp_circ_buffer_head + 1; */
/*    if(temp_head >= GP_CIRC_BUFFER_SIZE) */
/*    { */
/*       temp_head = 0; */
/*    } */

/*    gp_circ_buffer[temp_head].data_index = 0; */
/*    gp_circ_buffer[temp_head].packet_length = packet.packet_length; */
/*    gp_circ_buffer[temp_head].packet_error = packet.packet_error; */
/*    gp_circ_buffer[temp_head].gp_state = packet.gp_state; */
/*    for(ii=0; ii<packet.packet_length; ii++) */
/*    { */
/*       gp_circ_buffer[temp_head].gp[ii] = packet.gp[ii]; */
/*    } */


/*    /\* retval = gp_copy_packet(packet, &(gp_circ_buffer[temp_head])); *\/ */
/*    /\* if(retval != GP_SUCCESS) *\/ */
/*    /\* { *\/ */
/*    /\*    GPIO_SetBits(GPIOD, LED_PIN_RED); *\/ */
/*    /\*    while(1); *\/ */
/*    /\*    return retval; *\/ */
/*    /\* } *\/ */
/*    gp_circ_buffer_head = temp_head; */
/*    packet_send_mutex = 0; */

/*    /\* Enable interrupts back *\/ */
/*    if (!prim) { */
/*       __enable_irq(); */
/*    } */


/*    return GP_SUCCESS; */

/* } */


/* uint8_t send_gp_packets(void) */
/* { */
/*    GenericPacket *gp_ptr; */
/*    uint8_t ii; */

/*    while(gp_circ_buffer_head != gp_circ_buffer_tail) */
/*    { */
/*       gp_circ_buffer_tail = gp_circ_buffer_tail + 1; */
/*       if(gp_circ_buffer_tail >= GP_CIRC_BUFFER_SIZE) */
/*       { */
/*          gp_circ_buffer_tail = 0; */
/*       } */
/*       gp_ptr = &(gp_circ_buffer[gp_circ_buffer_tail]); */

/*       /\* while(packet_send_mutex != 0); *\/ */
/*       /\* packet_send_mutex = 1; *\/ */
/*       /\* for(ii=0; ii<gp_ptr->packet_length; ii++) *\/ */
/*       /\* { *\/ */
/*       /\*    usart_write_byte(gp_ptr->gp[ii]); *\/ */
/*       /\* } *\/ */
/*       /\* packet_send_mutex = 0; *\/ */

/*    } */
/* } */

void write_timestamps(void)
{
   uint8_t ii;

   while(ts_circ_buffer_tail != ts_circ_buffer_head)
   {
      ts_circ_buffer_tail = ts_circ_buffer_tail + 1;
      if(ts_circ_buffer_tail >= TS_CIRC_BUFFER_SIZE)
      {
         ts_circ_buffer_tail = 0;
      }

      if(usart_one_initialized)
      {
         for(ii=0; ii<ts_circ_buffer[ts_circ_buffer_tail].packet_length; ii++)
         {
            usart_write_byte(ts_circ_buffer[ts_circ_buffer_tail].gp[ii]);
         }
      }
      else if(usart_one_dma_initialized)
      {
         usart_write_dma((uint8_t *)ts_circ_buffer[ts_circ_buffer_tail].gp, ts_circ_buffer[ts_circ_buffer_tail].packet_length);
      }
   }
}



void write_vospi(void)
{
   uint8_t ii;

   while(vospi_circ_buffer_tail != vospi_circ_buffer_head)
   {
      vospi_circ_buffer_tail = vospi_circ_buffer_tail + 1;
      if(vospi_circ_buffer_tail >= VOSPI_CIRC_BUFFER_SIZE)
      {
         vospi_circ_buffer_tail = 0;
      }

      if(usart_one_initialized)
      {
         for(ii=0; ii<vospi_circ_buffer[vospi_circ_buffer_tail].packet_length; ii++)
         {
            usart_write_byte(vospi_circ_buffer[vospi_circ_buffer_tail].gp[ii]);
         }
      }
      else if(usart_one_dma_initialized)
      {
         usart_write_dma((uint8_t *)vospi_circ_buffer[vospi_circ_buffer_tail].gp, vospi_circ_buffer[vospi_circ_buffer_tail].packet_length);
      }
   }
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


void write_code_version(void)
{
   GenericPacket packet;
   uint8_t ii;

   if(create_universal_code_ver(&packet, GIT_REVISION) == GP_SUCCESS)
   {

      if(usart_one_initialized)
      {
         for(ii=0; ii<packet.packet_length; ii++)
         {
            usart_write_byte(packet.gp[ii]);
         }
      }
      else if(usart_one_dma_initialized)
      {
         usart_write_dma((uint8_t *)packet.gp, packet.packet_length);
      }
   }
}
