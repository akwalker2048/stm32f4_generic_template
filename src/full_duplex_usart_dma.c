/* FULL_DUPLEX_USART_DMA
 *
 * Notes:
 *   +This USART is intended to be high speed and high performance.
 *   +This USART is configured using the following hardware.  If you choose to
 *    change any of these hardware selections...also change this note so that
 *    it is easy to find/read the resources that are consumed here and that
 *    might produce conflicts with other hardware.
 *     -> USART1
 *     -> TX -> B6, DMA2_Stream7, DMA_Channel_4
 *     -> RX -> B7, DMA2_Stream5, DMA_Channel_4
 *     -> DMA2_Stream7_IRQn, DMA_IT_TC, DMA_IT_TCIF7
 *     -> 3 MBaud, 8N1, No hardware flow control...
 *   +This USART is intended to TX/RX Generic_Packets as defined in
 *    generic_packet.h.  I have found this to be very useful in many
 *    applications.  If you wish to have a raw usart that you are pushing
 *    characters or less structured bytes...or more structured packets...
 *    write a different utility and link to that.
 */

#define INIT_VARIABLES
#include "full_duplex_usart_dma.h"

/* GP_CIRC_BUFFER_SIZE defaults to 16 within the generic packet code if you
 * don't override it.
 */
#define GP_CIRC_BUFFER_SIZE FDUD_RX_QUEUE_SIZE
#include "gp_circular_buffer.h"



/* Private Defines */

/* Private Variables */
uint8_t full_duplex_usart_dma_initialized = 0;

uint8_t full_duplex_usart_dma_tx_buffer[FDUD_TX_DMA_SIZE];
uint8_t full_duplex_usart_dma_rx_buffer[FDUD_RX_DMA_SIZE];
uint16_t full_duplex_usart_dma_rx_buffer_tail = 0;

FDUD_TxQueue_Struct fdud_txqs[FDUD_TX_QUEUE_SIZE];
FDUD_TxQueue_CB_Struct fdud_txq_cb;

GenericPacketCircularBuffer fdud_rx_gpcb;

/* Private Functions */
void full_duplex_usart_dma_communications_init(void);
void full_duplex_usart_dma_service_rx(void);
void full_duplex_usart_dma_service_tx(void);


/* PUBLIC full_duplex_usart_up
 *
 * Notes:
 *  +This function is used from the main program to determine resource
 *   availability.
 *  +This function returns 1 if the USART is up and running.
 *  +This function returns 0 if the USART has not been initialized.
 */
uint8_t full_duplex_usart_dma_up(void)
{
   return full_duplex_usart_dma_initialized;
}


/* PUBLIC init_full_duplex_usart_dma
 *
 * Notes:
 *
 */
uint8_t full_duplex_usart_dma_init(void)
{
   uint8_t fail = 0;
   uint8_t retval;

   /* Init Circular Buffers and Memory */
   /* Tx Queue */
   fdud_txq_cb.fdud_txqs_ptr = fdud_txqs;
   fdud_txq_cb.size = FDUD_TX_QUEUE_SIZE;
   fdud_txq_cb.head = 0;
   fdud_txq_cb.tail = 0;

   /* Rx GenericPacket Circular Buffer */
   retval = gpcb_initialize(&fdud_rx_gpcb);
   if(retval != GP_CIRC_BUFFER_SUCCESS)
   {
      fail = 1;
   }

   /* Init Hardware */
   full_duplex_usart_dma_communications_init();

   if(fail != 0)
   {
      return FDUD_FAIL_NOT_INITIALIZED;
   }

   /* We're Up */
   full_duplex_usart_dma_initialized = 1;

   return FDUD_SUCCESS;
}

/* PUBLIC full_duplex_usart_dma_service(void)
 *
 * Notes:
 *  +This function must be called by the main program frequently enough to
 *   a. Pull bytes from the receive DMA buffer before the head catches the tail
 *   b. Send outgoing packets before the head catches the tail
 */
void full_duplex_usart_dma_service(void)
{
   full_duplex_usart_dma_service_rx();
   full_duplex_usart_dma_service_tx();
}

/* PUBLIC full_duplex_usart_dma_add_to_queue
 *
 * Notes:
 *  +This function allows a user to add a GenericPacket to the queue to be sent
 *   out in the order that it came in.
 *  +This function will return FDUD_SUCCESS if the circular queue buffer was
 *   not full.
 *  +This function will return FDUD_FAIL if the circular queue buffer was full
 *   and the calling program needs to resubmit the packet at a later time.
 */
uint8_t full_duplex_usart_dma_add_to_queue(GenericPacket *gp_ptr, FDUD_TxQueueCallback callback_func, uint32_t callback_data)
{
   uint32_t temp_head;

   if(full_duplex_usart_dma_initialized)
   {
      temp_head = fdud_txq_cb.head + 1;
      if(temp_head >= fdud_txq_cb.size)
      {
         temp_head = 0;
      }

      if(temp_head == fdud_txq_cb.tail)
      {
         /* We cannot add to the queue because we have caught the tail. */
         return FDUD_FAIL;
      }

      /* Add the next packet to the transmit queue. */
      fdud_txq_cb.fdud_txqs_ptr[temp_head].gp_ptr = gp_ptr;
      fdud_txq_cb.fdud_txqs_ptr[temp_head].cb = callback_func;
      fdud_txq_cb.fdud_txqs_ptr[temp_head].cb_data = callback_data;
      /* Make sure all memory has been written... */
      asm("DSB");
      /* Now as the very last thing...increment head... */
      fdud_txq_cb.head = temp_head;

      return FDUD_SUCCESS;
   }
   else
   {
      return FDUD_FAIL_NOT_INITIALIZED;
   }
}

/* PUBLIC full_duplex_usart_dma_service_rx
 *
 * Notes:
 *  +This function must be called from the main program frequently enough that
 *   the circular DMA buffer never wraps before we pull the data out.
 *  +I used to have a timer interrupt that just moved DMA bytes into a RAM
 *   buffer and then subsequently packetized it.  Maybe this can be a single
 *   step?
 */
void full_duplex_usart_dma_service_rx(void)
{
   uint16_t dma_head;
   uint8_t retval_gpcb;

   if(full_duplex_usart_dma_initialized)
   {
      retval_gpcb = GP_CIRC_BUFFER_SUCCESS;
      dma_head = (FDUD_RX_DMA_SIZE - DMA2_Stream5->NDTR);
      while((full_duplex_usart_dma_rx_buffer_tail != dma_head)&&(retval_gpcb == GP_CIRC_BUFFER_SUCCESS))
      {
         retval_gpcb = gpcb_receive_byte((full_duplex_usart_dma_rx_buffer[full_duplex_usart_dma_rx_buffer_tail]), &fdud_rx_gpcb);

         if(retval_gpcb == GP_CIRC_BUFFER_SUCCESS)
         {
            full_duplex_usart_dma_rx_buffer_tail++;
            if(full_duplex_usart_dma_rx_buffer_tail >= FDUD_RX_DMA_SIZE)
            {
               full_duplex_usart_dma_rx_buffer_tail = 0;
            }
         }
      }
   }

}

/* PUBLIC full_duplex_usart_dma_service_tx
 *
 * Notes:
 *  +This function sends out all packets that are currently in the queue.
 */
void full_duplex_usart_dma_service_tx(void)
{
   if(full_duplex_usart_dma_initialized)
   {

   }
}



/* PRIVATE init_full_duplex_usart_dma_communications
 *
 * Notes:
 *   +All hardware initialization is to be done here.  We should be up and
 *    operational after this.
 *   +This function must be called from the main program before the resource
 *    can be used.
 */
void full_duplex_usart_dma_communications_init(void)
{
   USART_InitTypeDef USART_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
   GPIO_InitTypeDef  GPIO_InitStructure;
   DMA_InitTypeDef  DMA_InitStructure;

   uint8_t retval;

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

   USART_InitStructure.USART_BaudRate = 3000000;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

   USART_OverSampling8Cmd(USART1, ENABLE);

   /* USART configuration */
   USART_Init(USART1, &USART_InitStructure);

   /* Set up DMA Here!!!! */
   /* Configure TX DMA */
   DMA_DeInit(DMA2_Stream7);
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
   DMA_InitStructure.DMA_BufferSize = (uint16_t)sizeof(full_duplex_usart_dma_tx_buffer);
   DMA_InitStructure.DMA_Channel = DMA_Channel_4;
   DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)full_duplex_usart_dma_tx_buffer;
   DMA_Init(DMA2_Stream7, &DMA_InitStructure);
   /* Configure RX DMA */
   DMA_DeInit(DMA2_Stream5);
   DMA_InitStructure.DMA_Channel = DMA_Channel_4;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)full_duplex_usart_dma_rx_buffer;
   DMA_InitStructure.DMA_BufferSize = (uint16_t)sizeof(full_duplex_usart_dma_rx_buffer);
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(USART1->DR));
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
   DMA_Init(DMA2_Stream5, &DMA_InitStructure);
   /* Enable the USART Rx DMA request */
   USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
   /* Enable the DMA RX Stream */
   DMA_Cmd(DMA2_Stream5, ENABLE);

   /* Enable USART */
   USART_Cmd(USART1, ENABLE);

   /* Use DMA TC interrupt to know when a packet has been sent. This way we can
    * notify the source of the packet that the memory can be free'd for use
    * in building another packet.
    */
   NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_Init(&NVIC_InitStructure);

   /* We won't enable this until we transmit a packet. */
   DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, DISABLE);

}


/* DAM2_Stream7_IRQHandler
 *
 * Notes:
 *  +This handler is used to catch the Transmit Complete event so that we can
 *   fire off the callback to the originator fo the packet so that they know
 *   that the memory resources can be free'd or used to form another packet.
 */
void DMA2_Stream7_IRQHandler(void)
{
   if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)
   {
      /* I believe this condition should already be met...or we woudln't
       * be here. */
      while (DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF6)==RESET);
      /* DMA has done it's job...but the last byte may not have been sent via
       * the USART hardware.  We might not need to poll for it here...but it
       * is quick and keeps us on the safe side.
       */
      while (USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);

      /* Put code to notify the orignator that the packet has been sent here! */


      /* Disable everything, we will turn it back on when we are ready to send
       * the next packet.
       */
      /* Disable the DMA */
      DMA_Cmd(DMA2_Stream7, DISABLE);
      /* Disable USART DMA TX Requsts */
      USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);

      /* Not sure whether this should go at the beginning or the end.  In this
       * case we will not have another interrupt being generated while we are in
       * here...so I don't think it matters.
       */
      DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
   }
}


void full_duplex_usart_dma_write(void)
{
   if(full_duplex_usart_dma_initialized)
   {

   }
}
