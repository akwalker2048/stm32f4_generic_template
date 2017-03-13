#include "stm32f4xx_conf.h"
#include "hardware_STM32F407G_DISC1.h"

/* Variables Global Within This File */
uint8_t gpio_initialized = 0;
uint8_t usart_initialized = 0;

/* ****************************************************************** */
/* GPIO Initialization                                                */
/* ****************************************************************** */
/* All GPIO for all other peripherals should be intialized here,
   including alternate function initialization.  That way, there is a
   single place where one can determine all used pins.
   /* ****************************************************************** */
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

   /* Init TIM5 Pins for Capture/Compare...TIA Application */
   /* TIM5_CH1-> A0
    * TIM5_CH2-> A1
    * TIM5_CH3-> A2
    * TIM5_CH4-> A3
    */


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
}

/* ****************************************************************** */
/* USART Initialization                                               */
/* ****************************************************************** */
void init_usart(void)
{
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;

   if(gpio_initialized == 1)
   {

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
/* USART3                                                             */
/* ****************************************************************** */
void USART3_IRQHandler(void)
{
   char tchar;

   GPIO_SetBits(GPIOD, LED_PIN_RED);

   if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
   {
      /* Read one byte from the receive data register */
      tchar = (USART_ReceiveData(USART3) & 0x7F);

      /* Echo what we just got! */
      USART_SendData(USART3, tchar);
   }

}
