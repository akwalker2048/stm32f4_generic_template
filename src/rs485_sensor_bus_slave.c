#include "rs485_sensor_bus.h"
#include "circular_buffer.h"

#include "hardware_STM32F407G_DISC1.h"

/* Buffers for raw data dma send and receive. */
uint8_t rs485_slave_dma_tx_buffer[GP_MAX_PACKET_LENGTH];

uint8_t rs485_slave_dma_rx_buffer[DMA_RX_BUFFER_SIZE];
circular_buffer_t cb_slave_dma_rx;
uint8_t rs485_slave_ram_rx_buffer[RAM_RX_BUFFER_SIZE];
circular_buffer_t cb_slave_ram_rx;

/* Buffers for incoming and outgoing GenericPacket data. */
GenericPacketCircularBuffer gpcbs_slave_tx;
GenericPacketCircularBuffer gpcbs_slave_rx;

/* Let us know when we're up! */
volatile uint8_t rs485_slave_initialized = 0;

rs485_master_states slave_state = RS485_SLAVE_INIT;
uint32_t rs485_slave_state_timer = 0;

/* Sensor Bus Slave State Machine
 *
 * Do I need the __attribute__("bank_switch") since I have a function call in here?
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
   GenericPacket packet_query;
   uint8_t retval;

   if(TIM_GetITStatus(TIM10, TIM_IT_Update) != RESET)
   {
      rs485_slave_state_timer++;

      /* GPIO_SetBits(GPIOD, LED_PIN_RED); */

      /* Always move received data out of the dma buffer to be processed outside
       * of the interrupt.  We don't want to take too long in here.
       */
      rs485_slave_process_rx_dma();

      switch(slave_state)
      {
         case RS485_SLAVE_INIT:
            break;
         default:
            break;
      } /* switch(slave_state) */

      /* GPIO_ResetBits(GPIOD, LED_PIN_RED); */

      TIM_ClearITPendingBit(TIM10, TIM_IT_Update);
   }

}


uint8_t rs485_sensor_bus_init_slave(void)
{

   uint8_t fail = 0;
   uint8_t retval;

   /* Initialize our packet circular buffers. */
   retval = gpcb_initialize(&gpcbs_slave_tx);
   if(retval != GP_CIRC_BUFFER_SUCCESS)
   {
      fail = 1;
   }

   retval = gpcb_initialize(&gpcbs_slave_rx);
   if(retval != GP_CIRC_BUFFER_SUCCESS)
   {
      fail = 1;
   }

   retval = cb_init(&cb_slave_dma_rx, rs485_slave_dma_rx_buffer, DMA_RX_BUFFER_SIZE);
   if(retval != CB_SUCCESS)
   {
      fail = 1;
   }

   retval = cb_init(&cb_slave_ram_rx, rs485_slave_ram_rx_buffer, RAM_RX_BUFFER_SIZE);
   if(retval != CB_SUCCESS)
   {
      fail = 1;
   }


   if(fail)
   {
      return RS485_SB_INIT_FAIL;
   }
   else
   {

      /* Finish up! */
      rs485_sensor_bus_init_slave_communications();
      rs485_sensor_bus_init_slave_state_machine();

      /* Everyone else should hold tight until this is set! */
      rs485_slave_initialized = 1;

      return RS485_SB_SUCCESS;

   }

}


void rs485_sensor_bus_init_slave_state_machine(void)
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;

   uint32_t TimerPeriod = 0;

   /* Turn the timer clock on! */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

   /* TIM9 on APB2 runs at SystemCoreClock.  The factor of 2 in the denominator
    * in this case is because we are dividing the clock to get a longer period.
    */
   TimerPeriod = (SystemCoreClock / (RS485_SENSOR_BUS_SM_HZ * 2)) - 1;

   /* Time Base configuration */
   TIM_TimeBaseStructure.TIM_Prescaler = 0;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

   TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);

   /* Set up interrupt. */
   NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE);

   TIM_Cmd(TIM10, ENABLE);

}


void rs485_sensor_bus_init_slave_communications(void)
{
   /* Slave RS485 is going to use:
    * USART6
    * Tx  -> C6, DMA2 - Channel 5 - Stream 6
    * Rx  -> C7, DMA2 - Channel 5 - Stream 1
    * T/R -> C8
    */
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
   GPIO_InitTypeDef  GPIO_InitStructure;
   DMA_InitTypeDef  DMA_InitStructure;

   uint8_t retval;

   /* Enable DMA Clock */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
   /* Enable the USART Clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

   /* Enable GPIO clock */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

   /* Set up GPIO for T/R line. */
   GPIO_StructInit(&GPIO_InitStructure);
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   /* Connect PXx to USARTx_Tx*/
   GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
   /* Connect PXx to USARTx_Rx*/
   GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
   /* Configure USART Tx as alternate function */
   GPIO_StructInit(&GPIO_InitStructure);
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   /* Configure USART Rx as alternate function  */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_Init(GPIOC, &GPIO_InitStructure);


   /* USART_InitStructure.USART_BaudRate = 115200; */
   USART_InitStructure.USART_BaudRate = RS485_SENSOR_BUS_BAUD;
   /* USART_InitStructure.USART_BaudRate = 1500000; */
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

   USART_OverSampling8Cmd(USART6, ENABLE);

   /* USART configuration */
   USART_Init(USART6, &USART_InitStructure);

   /* Set up DMA Here!!!! */
   /* Configure TX DMA */
   DMA_DeInit(DMA2_Stream6);
   DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
   DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
   DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(USART6->DR));
   DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
   DMA_InitStructure.DMA_BufferSize = (uint16_t)sizeof(rs485_slave_dma_tx_buffer);;
   DMA_InitStructure.DMA_Channel = DMA_Channel_5;
   DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rs485_slave_dma_tx_buffer;
   DMA_Init(DMA2_Stream6, &DMA_InitStructure);
   /* Configure RX DMA */
   DMA_DeInit(DMA2_Stream1);
   DMA_InitStructure.DMA_Channel = DMA_Channel_5;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rs485_slave_dma_rx_buffer;
   DMA_InitStructure.DMA_BufferSize = (uint16_t)sizeof(rs485_slave_dma_rx_buffer);
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(USART6->DR));
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
   DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
   DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
   DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
   DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
   DMA_Init(DMA2_Stream1, &DMA_InitStructure);
   /* Enable the USART Rx DMA request */
   USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
   /* Enable the DMA RX Stream */
   DMA_Cmd(DMA2_Stream1, ENABLE);

   /* Enable USART */
   USART_Cmd(USART6, ENABLE);

   /* Use DMA interrupt to flip the R/T line for RS485. */
   NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_Init(&NVIC_InitStructure);

   DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, DISABLE);


}

void DMA2_Stream6_IRQHandler(void)
{
   if(DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6) != RESET)
   {
      /* DMA is Done...we still need to wait for the last byte to exit the USART */
      while (USART_GetFlagStatus(USART6, USART_FLAG_TC)==RESET);
      while (DMA_GetFlagStatus(DMA2_Stream6, DMA_FLAG_TCIF6)==RESET);

      /* Now put us in receive mode. */
      rs485_sensor_bus_slave_rx();

      /* Disable the DMA */
      DMA_Cmd(DMA2_Stream6, DISABLE);
      /* Disable USART DMA TX Requsts */
      USART_DMACmd(USART6, USART_DMAReq_Tx, DISABLE);

      DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6);
   }
}


void rs485_sensor_bus_slave_tx(void)
{
   if(rs485_slave_initialized)
   {
      GPIO_SetBits(GPIOC, GPIO_Pin_8);
   }
}

void rs485_sensor_bus_slave_rx(void)
{
   if(rs485_slave_initialized)
   {
      GPIO_ResetBits(GPIOC, GPIO_Pin_8);
   }
}

void rs485_slave_state_change(rs485_slave_states new_state, uint8_t reset_timer)
{
   if(reset_timer)
   {
      rs485_slave_state_timer = 0;
   }

   slave_state = new_state;
}

void rs485_slave_write_dma(uint8_t *data, uint32_t length)
{
   if(rs485_slave_initialized)
   {

      /* /\* Enable USART DMA TX Requsts *\/ */
      /* USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE); */
      /* /\* Enable the DMA *\/ */
      /* DMA_Cmd(DMA2_Stream6, ENABLE); */


      /* /\* /\\* Wait for any previous transfer to complete. *\\/ *\/ */
      /* while (USART_GetFlagStatus(USART6, USART_FLAG_TC)==RESET); */
      /* while (DMA_GetFlagStatus(DMA2_Stream6, DMA_FLAG_TCIF6)==RESET); */

      /* Disable the DMA */
      DMA_Cmd(DMA2_Stream6, DISABLE);
      /* Disable USART DMA TX Requsts */
      USART_DMACmd(USART6, USART_DMAReq_Tx, DISABLE);

      /* Clear DMA Transfer Complete Flags */
      DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6);
      /* Clear USART Transfer Complete Flags */
      USART_ClearFlag(USART6, USART_FLAG_TC);

      /* Make sure we are in transmit mode. */
      rs485_sensor_bus_slave_tx();

      /* Set the length of data to transmit. */
      DMA2_Stream6->NDTR = length;
      /* Set the pointer to the data. */
      DMA2_Stream6->M0AR = (uint32_t)data;

      /* Enable USART DMA TX Requsts */
      USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
      /* Enable the DMA */
      DMA_Cmd(DMA2_Stream6, ENABLE);

   }
}

void rs485_slave_process_rx_dma(void)
{
   uint8_t retval;
   uint16_t dma_head;
   uint8_t rx_byte;

   dma_head = (cb_slave_dma_rx.cb_size - DMA2_Stream1->NDTR);
   retval = cb_set_head_dma(&cb_slave_dma_rx, dma_head);
   if(retval == CB_SUCCESS)
   {
      do{
         retval = cb_get_byte(&cb_slave_dma_rx, &rx_byte);
         if(retval == CB_SUCCESS)
         {
            retval = cb_add_byte(&cb_slave_ram_rx, rx_byte);
         }
      }while(retval == CB_SUCCESS);
   }

}


void rs485_slave_process_rx_ram(void)
{
   uint8_t retval, retval_gpcb;
   uint8_t rx_byte;

   uint8_t retval_debug;
   GenericPacket gp_debug;

   do{
      retval = cb_get_byte(&cb_slave_ram_rx, &rx_byte);
      if(retval == CB_SUCCESS)
      {
         /* GPIO_SetBits(GPIOD, LED_PIN_RED); */

         retval_gpcb = gpcb_receive_byte(rx_byte, &gpcbs_slave_rx);

         /* for debug */
         asm("DMB"); /* Data Memory Barrier */
         asm("DSB"); /* Data Synchronization Barrier */
         asm("ISB"); /* Instruction Synchronization Barrier. */
         /* retval_debug = create_universal_byte(&gp_debug, 0x08); */
         create_universal_byte(&gp_debug, retval_gpcb);
         if(retval_debug == GP_SUCCESS)
         {
            asm("DMB"); /* Data Memory Barrier */
            asm("DSB"); /* Data Synchronization Barrier */
            asm("ISB"); /* Instruction Synchronization Barrier. */
            usart_write_dma(gp_debug.gp, gp_debug.packet_length);
         }

         /* if(retval_gpcb == GP_CHECKSUM_MATCH) */
         /* { */
         /*    /\* GPIO_SetBits(GPIOD, LED_PIN_RED); *\/ */
         /* } */

         /* if(retval_gpcb == GP_ERROR_CHECKSUM_MISMATCH) */
         /* { */
         /*    /\* GPIO_SetBits(GPIOD, LED_PIN_RED); *\/ */
         /* } */

         /* GPIO_ResetBits(GPIOD, LED_PIN_RED); */

      }
   }while((retval == CB_SUCCESS)&&(retval_gpcb = GP_CIRC_BUFFER_SUCCESS));

}

void rs485_slave_handle_packets(void)
{
   uint8_t address, sensor_type;
   uint8_t retval_tail, retval;
   GenericPacket *gp_ptr;
   GenericPacket gp_sensor_info;
   PoseIsh p;


   do{
      retval_tail = gpcb_increment_tail(&gpcbs_slave_rx);
      if(retval_tail == GP_CIRC_BUFFER_SUCCESS)
      {



         gp_ptr = &(gpcbs_slave_rx.gpcb[gpcbs_slave_rx.gpcb_tail]);
         switch(gp_ptr->gp[GP_LOC_PROJ_ID])
         {
            case GP_PROJ_RS485_SB:
               {
                  switch(gp_ptr->gp[GP_LOC_PROJ_SPEC])
                  {
                     case RS485_QUERY_SENSOR_INFO:
                        {
                           retval = extract_rs485_query_sensor_info(gp_ptr, &address);
                           if((retval == GP_SUCCESS)&&(address == SLAVE_ADDRESS))
                           {
                              /* Send the response packet to the master. */
                              p.x = 1.1f;
                              p.y = 2.2f;
                              p.z = 3.3f;
                              p.roll = 4.4f;
                              p.pitch = 5.5f;
                              p.yaw = 6.6f;
                              retval = create_rs485_resp_sensor_info(&gp_sensor_info, RS485_ADDRESS_MASTER, RS485_SB_TYPE_PROXIMITY_SONAR, p);
                              if(retval == GP_SUCCESS)
                              {
                                 rs485_slave_write_dma(gp_sensor_info.gp, gp_sensor_info.packet_length);
                              }
                           }
                        } /* RS485_QUERY_SENSOR_INFO */
                        break;
                     case RS485_RESP_SENSOR_INFO:
                        {
                           retval = extract_rs485_resp_sensor_info(gp_ptr, &address, &sensor_type, &p);
                           if((retval == GP_SUCCESS)&&(address == SLAVE_ADDRESS))
                           {
                              /* As a slave, we should be sending this...not receiving. */
                           }
                           break;
                        } /* RS485_RESP_SENSOR_INFO */
                  } /* switch(gp_ptr->gp[GP_LOC_PROJ_SPEC]) */
                  break;
               } /* GP_PROJ_RS485_SB */
            default:
               break;
         } /* switch(gp_ptr->gp[GP_LOC_PROJ_ID]) */




      } /* if(GP_CIRC_BUFFER_SUCCESS) */


   }while(retval_tail == GP_CIRC_BUFFER_SUCCESS);



}
