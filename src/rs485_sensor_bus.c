#include "rs485_sensor_bus.h"
#include "circular_buffer.h"

#include "hardware_STM32F407G_DISC1.h"

/* Buffers for raw data dma send and receive. */
/* This one is really just a place holder for initialization.  Probably don't
 * need to make it this big.
 */
uint8_t rs485_master_dma_tx_buffer[GP_MAX_PACKET_LENGTH];

uint8_t rs485_master_dma_rx_buffer[DMA_RX_BUFFER_SIZE];
circular_buffer_t cb_master_dma_rx;
uint8_t rs485_master_ram_rx_buffer[RAM_RX_BUFFER_SIZE];
circular_buffer_t cb_master_ram_rx;

/* Buffers for incoming and outgoing GenericPacket data. */
GenericPacketCircularBuffer gpcbs_master_tx;
GenericPacketCircularBuffer gpcbs_master_rx;

/* Let us know when we're up! */
volatile uint8_t rs485_master_initialized = 0;

rs485_master_states master_state = RS485_MASTER_INIT;
uint32_t rs485_master_state_timer = 0;

volatile uint8_t response_received = 0;

/* Sensor Bus Master State Machine
 *
 * Do I need the __attribute__("bank_switch") since I have a function call in here?
 */
void TIM1_BRK_TIM9_IRQHandler(void)
{
   GenericPacket packet_query;
   uint8_t retval;

   if(TIM_GetITStatus(TIM9, TIM_IT_Update) != RESET)
   {

      GPIO_SetBits(GPIOD, LED_PIN_ORANGE);

      rs485_master_state_timer++;

      /* Always move received data out of the dma buffer to be processed outside
       * of the interrupt.  We don't want to take too long in here.
       */
      rs485_master_process_rx_dma();

      switch(master_state)
      {
         case RS485_MASTER_INIT:
            {
               rs485_master_state_change(RS485_MASTER_DELAY, 1);
            } /* RS485_MASTER_INIT */
            break;
         case RS485_MASTER_FIND_ATTACHED_DEVICES:
            {

            } /* RS485_MASTER_FIND_ATTACHED_DEVICES */
            break;
         case RS485_MASTER_QUERY_DEVICE:
            {
               retval = create_rs485_query_sensor_info(&packet_query, SLAVE_ADDRESS);
               if(retval == GP_SUCCESS)
               {
                  response_received = 0;
                  rs485_master_write_dma(packet_query.gp, packet_query.packet_length);
                  rs485_master_state_change(RS485_MASTER_AWAIT_RESPONSE, 1);
               }
               else
               {
                  rs485_master_state_change(RS485_MASTER_DELAY, 1);
               }
            } /* RS485_MASTER_QUERY_DEVICE */
            break;
         case RS485_MASTER_AWAIT_RESPONSE:
            {
               if(rs485_master_state_timer >= RS485_MASTER_RESPONSE_TIMEOUT_TICKS)
               {
                  rs485_master_state_change(RS485_MASTER_DELAY, 1);
               }

               if(response_received)
               {
                  rs485_master_state_change(RS485_MASTER_DELAY, 1);
               }

            } /* RS485_MASTER_AWAIT_RESPONSE */
            break;
         case RS485_MASTER_DELAY:
            {
               if(rs485_master_state_timer >= RS485_MASTER_DELAY_TICKS)
               {
                  rs485_master_state_change(RS485_MASTER_QUERY_DEVICE, 1);
               }
            } /* RS485_MASTER_DELAY */
            break;
         case RS485_MASTER_IDLE:
            {
            } /* RS_485_MASTER_IDLE */
            break;
         case RS485_MASTER_ERROR:
            {
            } /* RS_485_MASTER_ERROR */
            break;
         default:
            {
            } /* default */
            break;
      } /* switch(master_state) */

      GPIO_ResetBits(GPIOD, LED_PIN_ORANGE);

      TIM_ClearITPendingBit(TIM9, TIM_IT_Update);
   }

}


void rs485_master_state_change(rs485_master_states new_state, uint8_t reset_timer)
{
   if(reset_timer)
   {
      rs485_master_state_timer = 0;
   }

   master_state = new_state;
}



uint8_t rs485_sensor_bus_init_master(void)
{

   uint8_t fail = 0;
   uint8_t retval;

   /* Initialize our packet circular buffers. */
   retval = gpcb_initialize(&gpcbs_master_tx);
   if(retval != GP_CIRC_BUFFER_SUCCESS)
   {
      fail = 1;
   }

   retval = gpcb_initialize(&gpcbs_master_rx);
   if(retval != GP_CIRC_BUFFER_SUCCESS)
   {
      fail = 1;
   }

   retval = cb_init(&cb_master_dma_rx, rs485_master_dma_rx_buffer, DMA_RX_BUFFER_SIZE);
   if(retval != CB_SUCCESS)
   {
      fail = 1;
   }

   retval = cb_init(&cb_master_ram_rx, rs485_master_ram_rx_buffer, RAM_RX_BUFFER_SIZE);
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
      rs485_sensor_bus_init_master_communications();
      rs485_sensor_bus_init_master_state_machine();

      /* /\* Everyone else should hold tight until this is set! *\/ */
      rs485_master_initialized = 1;

      return RS485_SB_SUCCESS;

   }

}


void rs485_sensor_bus_init_master_state_machine(void)
{

   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;

   uint32_t TimerPeriod = 0;

   /* Turn the timer clock on! */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

   /* TIM9 on APB2 runs at SystemCoreClock.  The factor of 2 in the denominator
    * in this case is because we are dividing the clock to get a longer period.
    */
   TimerPeriod = (SystemCoreClock / (RS485_SENSOR_BUS_SM_HZ)) - 1;

   /* Time Base configuration */
   TIM_TimeBaseStructure.TIM_Prescaler = 0;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

   TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

   /* Set up interrupt. */
   NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   TIM_ITConfig(TIM9, TIM_IT_Update, ENABLE);

   TIM_Cmd(TIM9, ENABLE);

}

void rs485_sensor_bus_init_master_communications(void)
{
   /* Master RS485 is going to use:
    *  USART2
    *  Tx  -> D5, DMA1 - Channel 4 - Stream 6    (Or A2 to avoid RED LED)
    *  Rx  -> D6, DMA1 - Channel 4 - Stream 5
    *  T/R -> D7
    */
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
   GPIO_InitTypeDef  GPIO_InitStructure;
   DMA_InitTypeDef  DMA_InitStructure;

   uint8_t retval;

   /* Enable DMA Clock */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
   /* Enable the USART Clock */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

   /* Enable GPIO clock */
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

   /* Set up GPIO for T/R line. */
   GPIO_StructInit(&GPIO_InitStructure);
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_Init(GPIOD, &GPIO_InitStructure);

   /* Connect PXx to USARTx_Tx*/
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
   /* Connect PXx to USARTx_Rx*/
   GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
   /* Configure USART Tx as alternate function */
   GPIO_StructInit(&GPIO_InitStructure);
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   /* Configure USART Rx as alternate function  */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   GPIO_Init(GPIOD, &GPIO_InitStructure);


   /* USART_InitStructure.USART_BaudRate = 115200; */
   USART_InitStructure.USART_BaudRate = RS485_SENSOR_BUS_BAUD;
   /* USART_InitStructure.USART_BaudRate = 1500000; */
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

   USART_OverSampling8Cmd(USART2, ENABLE);

   /* USART configuration */
   USART_Init(USART2, &USART_InitStructure);

   /* Set up DMA Here!!!! */
   /* Configure TX DMA */
   DMA_DeInit(DMA1_Stream6);
   DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
   DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
   DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(USART2->DR));
   DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
   DMA_InitStructure.DMA_BufferSize = (uint16_t)sizeof(rs485_master_dma_tx_buffer);;
   DMA_InitStructure.DMA_Channel = DMA_Channel_4;
   DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rs485_master_dma_tx_buffer;
   DMA_Init(DMA1_Stream6, &DMA_InitStructure);
   /* Configure RX DMA */
   DMA_DeInit(DMA1_Stream5);
   DMA_InitStructure.DMA_Channel = DMA_Channel_4;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rs485_master_dma_rx_buffer;
   DMA_InitStructure.DMA_BufferSize = (uint16_t)sizeof(rs485_master_dma_rx_buffer);
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(USART2->DR));
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
   DMA_Init(DMA1_Stream5, &DMA_InitStructure);
   /* Enable the USART Rx DMA request */
   USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
   /* Enable the DMA RX Stream */
   DMA_Cmd(DMA1_Stream5, ENABLE);

   /* Enable USART */
   USART_Cmd(USART2, ENABLE);

   /* Use DMA interrupt to flip the R/T line for RS485. */
   NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_Init(&NVIC_InitStructure);

   DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, DISABLE);

}


void DMA1_Stream6_IRQHandler(void)
{
   if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) != RESET)
   {
      /* DMA is Done...we still need to wait for the last byte to exit the USART */
      while (USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
      while (DMA_GetFlagStatus(DMA1_Stream6, DMA_FLAG_TCIF6)==RESET);

      /* Now put us in receive mode. */
      rs485_sensor_bus_master_rx();

      /* Disable the DMA */
      DMA_Cmd(DMA1_Stream6, DISABLE);
      /* Disable USART DMA TX Requsts */
      USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);


      DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
   }
}


void rs485_sensor_bus_master_tx(void)
{
   if(rs485_master_initialized)
   {
      GPIO_SetBits(GPIOD, GPIO_Pin_7);
   }
}

void rs485_sensor_bus_master_rx(void)
{
   if(rs485_master_initialized)
   {
      GPIO_ResetBits(GPIOD, GPIO_Pin_7);
   }
}

void rs485_master_write_dma(uint8_t *data, uint32_t length)
{
   if(rs485_master_initialized)
   {

      /* /\* Enable USART DMA TX Requsts *\/ */
      /* USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE); */
      /* /\* Enable the DMA *\/ */
      /* DMA_Cmd(DMA1_Stream6, ENABLE); */


      /* /\* /\\* Wait for any previous transfer to complete. *\\/ *\/ */
      /* while (USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET); */
      /* while (DMA_GetFlagStatus(DMA1_Stream6, DMA_FLAG_TCIF6)==RESET); */

      /* Disable the DMA */
      DMA_Cmd(DMA1_Stream6, DISABLE);
      /* Disable USART DMA TX Requsts */
      USART_DMACmd(USART2, USART_DMAReq_Tx, DISABLE);

      /* Clear DMA Transfer Complete Flags */
      DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
      /* Clear USART Transfer Complete Flags */
      USART_ClearFlag(USART2, USART_FLAG_TC);

      /* Make sure we are in transmit mode. */
      rs485_sensor_bus_master_tx();


      /* Enable the interrupt for RS485 R/T handling. */
      DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);

      /* Set the length of data to transmit. */
      DMA1_Stream6->NDTR = length;
      /* Set the pointer to the data. */
      DMA1_Stream6->M0AR = (uint32_t)data;

      /* Enable USART DMA TX Requsts */
      USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
      /* Enable the DMA */
      DMA_Cmd(DMA1_Stream6, ENABLE);

   }
}


void rs485_master_process_rx_dma(void)
{
   uint8_t retval;
   uint16_t dma_head;
   uint8_t rx_byte;

   dma_head = (cb_master_dma_rx.cb_size - DMA1_Stream5->NDTR);
   retval = cb_set_head_dma(&cb_master_dma_rx, dma_head);
   if(retval == CB_SUCCESS)
   {
      do{
         retval = cb_get_byte(&cb_master_dma_rx, &rx_byte);
         if(retval == CB_SUCCESS)
         {
            retval = cb_add_byte(&cb_master_ram_rx, rx_byte);
         }
      }while(retval == CB_SUCCESS);
   }

}

void rs485_master_process_rx_ram(void)
{
   uint8_t retval, retval_gpcb;
   uint8_t rx_byte;

   do{
      retval = cb_get_byte(&cb_master_ram_rx, &rx_byte);
      if(retval == CB_SUCCESS)
      {
         retval_gpcb = gpcb_receive_byte(rx_byte, &gpcbs_master_rx);
      }
   }while((retval == CB_SUCCESS)&&(retval_gpcb = GP_CIRC_BUFFER_SUCCESS));

}

void rs485_master_handle_packets(void)
{
   uint8_t address, sensor_type;
   uint8_t retval_tail, retval;
   GenericPacket *gp_ptr;
   GenericPacket gp_sensor_info;
   PoseIsh p;

   do{
      retval_tail = gpcb_increment_tail(&gpcbs_master_rx);
      if(retval_tail == GP_CIRC_BUFFER_SUCCESS)
      {
         gp_ptr = &(gpcbs_master_rx.gpcb[gpcbs_master_rx.gpcb_tail]);
         switch(gp_ptr->gp[GP_LOC_PROJ_ID])
         {
            case GP_PROJ_RS485_SB:
               switch(gp_ptr->gp[GP_LOC_PROJ_SPEC])
               {
                  case RS485_QUERY_SENSOR_INFO:
                     retval = extract_rs485_query_sensor_info(gp_ptr, &address);
                     if((retval == GP_SUCCESS)&&(address == RS485_ADDRESS_MASTER))
                     {
                        /* We actually shouldn't get this packet as a master. */
                     }
                     break;
                  case RS485_RESP_SENSOR_INFO:
                     retval = extract_rs485_resp_sensor_info(gp_ptr, &address, &sensor_type, &p);
                     if((retval == GP_SUCCESS)&&(address == RS485_ADDRESS_MASTER))
                     {

                        GPIO_SetBits(GPIOD, LED_PIN_RED);

                        response_received = 1;
                        /* Need to pass this data along to the ROS system. */
                        retval = create_rs485_resp_sensor_info(&gp_sensor_info, RS485_ADDRESS_BROADCAST, sensor_type, p);
                        if(retval == GP_SUCCESS)
                        {
                           usart_write_dma(gp_sensor_info.gp, gp_sensor_info.packet_length);
                        }
                     }
                     break;
               } /* switch(gp_ptr->gp[GP_LOC_PROJ_SPEC]) */
               break;
            default:
               break;
         } /* switch(gp_ptr->gp[GP_LOC_PROJ_ID]) */
      }
   }while(retval_tail == GP_CIRC_BUFFER_SUCCESS);
}
