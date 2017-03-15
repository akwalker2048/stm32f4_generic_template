#include "stm32f4xx_conf.h"
#include "hardware_STM32F407G_DISC1.h"

/* Variables Global Within This File */
uint8_t gpio_initialized = 0;
uint8_t systick_initialized = 0;
uint8_t usart_initialized = 0;
uint8_t pushbutton_initialized = 0;
uint8_t analog_input_initialized = 0;
uint8_t tia_initialized = 0;
uint8_t spi_initialized = 0;
uint8_t i2c_initialized = 0;

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
void init_usart(void)
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
   RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
   /* Connect PXx to USARTx_Tx*/
   GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
   /* Connect PXx to USARTx_Rx*/
   GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
   /* Configure USART Tx as alternate function  */
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_Init(GPIOD, &GPIO_InitStructure);
   /* Configure USART Rx as alternate function  */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_Init(GPIOD, &GPIO_InitStructure);

   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

   USART_InitStructure.USART_BaudRate = 115200;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

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

   usart_initialized = 1;

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
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
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
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
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
   SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
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
   uint16_t txbuf[2];

   ms_counter++;
   ms_count_r++;

   txbuf[0] = 'a';
   txbuf[1] = 'b';

   ii++;
   if(ii%500 == 0)
   {
      if(GPIO_ReadInputDataBit(GPIOD, LED_PIN_GREEN) == Bit_SET)
      {
         GPIO_ResetBits(GPIOD, LED_PIN_GREEN);
         GPIO_SetBits(GPIOD, LED_PIN_BLUE);
         /* USART_SendData(USART3, txbuf[0]); */
      }
      else
      {
         GPIO_SetBits(GPIOD, LED_PIN_GREEN);
         GPIO_ResetBits(GPIOD, LED_PIN_BLUE);
         /* USART_SendData(USART3, txbuf[1]); */
      }


   }

   /* if(ii%250 == 0) */
   /* { */
   /*    if(GPIO_ReadInputDataBit(GPIOD, LED_PIN_ORANGE) == Bit_SET) */
   /*    { */
   /*       GPIO_ResetBits(GPIOD, LED_PIN_ORANGE); */
   /*       GPIO_SetBits(GPIOD, LED_PIN_RED); */
   /*    } */
   /*    else */
   /*    { */
   /*       GPIO_SetBits(GPIOD, LED_PIN_ORANGE); */
   /*       GPIO_ResetBits(GPIOD, LED_PIN_RED); */
   /*    } */
   /* } */

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
uint8_t usart_write_byte(uint8_t data)
{
   if(usart_initialized)
   {
      /* Make sure any previous write has completed. */
      while( !(USART3->SR & 0x00000040) );
      /* Send the data. */
      if((0x20 <= data)&&(data <= 0x7E))
      {
         USART_SendData(USART3, (uint16_t)data);
      }
   }

   return 0;
}


void spi_cs_enable(void)
{
   GPIO_ResetBits(GPIOB, SPI_PIN_CS_AL);
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
