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

#include "gp_circular_buffer.h"

#include "circular_buffer.h"

#include "hardware_STM32F407G_DISC1.h"



/* Private Defines */

/* Private Variables */
uint8_t full_duplex_usart_dma_initialized = 0;

GenericPacketCallback fdud_gp_handler = NULL;

uint8_t full_duplex_usart_dma_tx_buffer[FDUD_TX_DMA_SIZE];

uint8_t full_duplex_usart_dma_rx_buffer[FDUD_RX_DMA_SIZE];
circular_buffer_t cb_fdud_dma_rx;
uint8_t full_duplex_usart_ram_rx_buffer[FDUD_RX_RAM_SIZE];
circular_buffer_t cb_fdud_ram_rx;


FDUD_TxQueue_Struct fdud_txqs[FDUD_TX_QUEUE_SIZE];
FDUD_TxQueue_CB_Struct fdud_txq_cb;
volatile uint8_t fdud_txq_cb_mutex = 0;


GenericPacket fdud_rx[FDUD_RX_QUEUE_SIZE];
GenericPacketCircularBuffer fdud_rx_gpcb;


GenericPacket gp_debug;
GenericPacket gp_debug_out;
GenericPacket gp_debug_two;

GenericPacket gp_received_bytes;
uint8_t received_bytes[0xFB];
uint8_t received_bytes_index = 0;
volatile uint8_t received_bytes_sending = 0;

/* Private Functions */
void full_duplex_usart_dma_communications_init(void);
void full_duplex_usart_dma_write(void);
void full_duplex_usart_dma_init_state_machine(void);
void reset_received_bytes_sending(uint32_t cb_data);

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
uint8_t full_duplex_usart_dma_init(GenericPacketCallback gp_handler)
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
   retval = gpcb_initialize(&fdud_rx_gpcb, fdud_rx, FDUD_RX_QUEUE_SIZE);
   if(retval != GP_CIRC_BUFFER_SUCCESS)
   {
      fail = 1;
   }

   retval = gp_receive_byte(0x00, GP_CONTROL_INITIALIZE, &gp_debug);

   retval = cb_init(&cb_fdud_dma_rx, full_duplex_usart_dma_rx_buffer, FDUD_RX_DMA_SIZE);
   if(retval != CB_SUCCESS)
   {
      fail = 1;
   }

   retval = cb_init(&cb_fdud_ram_rx, full_duplex_usart_ram_rx_buffer, FDUD_RX_RAM_SIZE);
   if(retval != CB_SUCCESS)
   {
      fail = 1;
   }


   /* Set the Callback Function Pointer */
   fdud_gp_handler = gp_handler;

   /* Init Hardware */
   full_duplex_usart_dma_communications_init();
   full_duplex_usart_dma_init_state_machine();

   if(fail != 0)
   {
      return FDUD_FAIL_NOT_INITIALIZED;
   }

   /* We're Up */
   /* GPIO_SetBits(GPIOD, LED_PIN_RED); */
   full_duplex_usart_dma_initialized = 1;

   /* Put something in the outgoing queue to see if it is working. */
   create_universal_ack(&gp_debug_out);
   full_duplex_usart_dma_add_to_queue(&gp_debug_out, NULL, 0);
   create_universal_timestamp(&gp_debug_two, 0x1234);
   full_duplex_usart_dma_add_to_queue(&gp_debug_two, NULL, 0);

   return FDUD_SUCCESS;
}


void TIM8_BRK_TIM12_IRQHandler(void)
{
   GenericPacket packet_query;
   uint8_t retval;

   if(TIM_GetITStatus(TIM12, TIM_IT_Update) != RESET)
   {
      /* See if we can service the circular buffer in here!
       *
       * May decide to make this a more involved state machine later...but for
       * now...well just service the single state of "handle the DMA buffer".
       */
      full_duplex_usart_dma_service();

      TIM_ClearITPendingBit(TIM12, TIM_IT_Update);
   }

}


void full_duplex_usart_dma_init_state_machine(void)
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;

   uint32_t TimerPeriod = 0;

   /* Turn the timer clock on! */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

   /* TIM12 on APB1 runs at SystemCoreClock/2.  The factor of 2 in the denominator
    * in this case is because APB1 runs at half of SystemCoreClock.
    */
   TimerPeriod = (SystemCoreClock / (FULL_DUPLEX_USART_SM_HZ * 2)) - 1;

   /* Time Base configuration */
   TIM_TimeBaseStructure.TIM_Prescaler = 0;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

   TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);

   /* Set up interrupt. */
   NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   TIM_ITConfig(TIM12, TIM_IT_Update, ENABLE);

   TIM_Cmd(TIM12, ENABLE);

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
   uint8_t retval;
   uint16_t dma_head;
   uint8_t rx_byte;


   dma_head = (cb_fdud_dma_rx.cb_size - DMA2_Stream5->NDTR);
   retval = cb_set_head_dma(&cb_fdud_dma_rx, dma_head);
   if(retval == CB_SUCCESS)
   {
      do{
         retval = cb_get_byte(&cb_fdud_dma_rx, &rx_byte);
         if(retval == CB_SUCCESS)
         {
            retval = cb_add_byte(&cb_fdud_ram_rx, rx_byte);

            /* if(GPIO_ReadInputDataBit(GPIOD, LED_PIN_BLUE) == Bit_SET) */
            /* { */
            /*    GPIO_ResetBits(GPIOD, LED_PIN_BLUE); */
            /* } */
            /* else */
            /* { */
            /*    GPIO_SetBits(GPIOD, LED_PIN_BLUE); */
            /* } */


         }
      }while(retval == CB_SUCCESS);
   }

}


void reset_received_bytes_sending(uint32_t cb_data)
{
   received_bytes_sending = 0;
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
   uint8_t retval;
   uint8_t rx_byte;

   if(full_duplex_usart_dma_initialized)
   {

      do{
         retval = cb_get_byte(&cb_fdud_ram_rx, &rx_byte);
         if(retval == CB_SUCCESS)
         {
            retval_gpcb = gpcb_receive_byte(rx_byte, &fdud_rx_gpcb);

            if(GPIO_ReadInputDataBit(GPIOD, LED_PIN_BLUE) == Bit_SET)
            {
               GPIO_ResetBits(GPIOD, LED_PIN_BLUE);
            }
            else
            {
               GPIO_SetBits(GPIOD, LED_PIN_BLUE);
            }


            if(retval_gpcb == GP_CHECKSUM_MATCH)
            {
               if(GPIO_ReadInputDataBit(GPIOD, LED_PIN_BLUE) == Bit_SET)
               {
                  GPIO_ResetBits(GPIOD, LED_PIN_BLUE);
               }
               else
               {
                  GPIO_SetBits(GPIOD, LED_PIN_BLUE);
               }

            }

         }
      }while ((retval == CB_SUCCESS)&&((retval_gpcb == GP_CIRC_BUFFER_SUCCESS)||(retval_gpcb == GP_ERROR_CHECKSUM_MISMATCH)||(retval_gpcb == GP_CHECKSUM_MATCH)));


   }

}

/* PUBLIC full_duplex_usart_dma_service_tx
 *
 * Notes:
 *  +This function sends out all packets that are currently in the queue.
 *  +This is currently being called from main...outside of an interrupt...
 *   That has some advantages but some drawbacks.  I can think of two
 *   alternatives:
 *   a) Have a timer interrupt in here that will kick off subsequent
 *      transfers so that this function kicks off one transfer and
 *      doesn't block.
 *   b) Have the DMAx_Streamy TC interrupt kick off the next transfer
 *      if one is available.  Then it is only the beginning of the
 *      first transfer that is kicked off from the main loop polling.
 *
 *  Maybe this isn't a big deal, since in most cases there will only
 *  be a packet or two waiting to be transmitted...But for now...this
 *  function will spin until everything has been sent.  That will burn
 *  up clock cycles that could be taking care of other things.
 */
void full_duplex_usart_dma_service_tx(void)
{
   if(full_duplex_usart_dma_initialized)
   {
      /* We can't kick this off from here again until the previous chain of
       * transmits is complete.
       */
      if(!fdud_txq_cb_mutex)
      {
         if(fdud_txq_cb.head != fdud_txq_cb.tail)
         {
            fdud_txq_cb_mutex = 1;
            fdud_txq_cb.tail++;
            if(fdud_txq_cb.tail >= fdud_txq_cb.size)
            {
               fdud_txq_cb.tail = 0;
            }
            /* This function will use the current tail. */
            full_duplex_usart_dma_write();
         }
      }
   }
}

/* PUBLIC full_duplex_usart_dma_get_rx_packet
 *
 * Notes:
 *  +Not 100% sure how to implement this yet...copying GenericPackets has been
 *   a little difficult on the micro (bandwidth/resources)...however, if I just
 *   hand out a pointer here, I need to figure out when it's done being used
 *   so that the tail can be incremented.  Maybe I just increment the tail and
 *   assume that by the time the buffer gets back around to here that it's ok
 *   to overwrite?  Seems like a bad idea... But so does expected the caller
 *   to let me know when they're done...
 *  +Anyway, the main program needs to be able to retrieve the next incoming
 *   GenericPacket that has been received...Unless it's going to be handled
 *   in here?  It would be OK for now...but then when I want to share the code
 *   later...it's not as flexible.  Maybe I should have the caller send a
 *   pointer to the handler function at init time...and I can call that
 *   function with the next valid packet when I find it?
 */
uint8_t full_duplex_usart_dma_get_rx_packet(void)
{
   uint8_t retval;

   if(full_duplex_usart_dma_initialized)
   {
      /* Or... while(gpcb_increment_tail(&fdud_rx_gpcb) == GP_CIRC_BUFFER_SUCCESS)
       * or...maybe not...need to post increment here...
       */
      while(gpcb_increment_tail(&fdud_rx_gpcb) == GP_CIRC_BUFFER_SUCCESS)
      {
         fdud_gp_handler(&(fdud_rx_gpcb.gpcb[fdud_rx_gpcb.gpcb_tail]));
      }
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
   /* DMA_InitStructure.DMA_BufferSize = (uint16_t)sizeof(full_duplex_usart_dma_rx_buffer); */
   DMA_InitStructure.DMA_BufferSize = (uint16_t)FDUD_RX_DMA_SIZE;
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
      while (DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7)==RESET);
      /* DMA has done it's job...but the last byte may not have been sent via
       * the USART hardware.  We might not need to poll for it here...but it
       * is quick and keeps us on the safe side.
       */
      while (USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);

      /* Not sure whether this should go at the beginning or the end.  In this
       * case we will not have another interrupt being generated while we are in
       * here...so I don't think it matters.
       *
       * If we're goinig to call full_duplex_usart_dma_write() from within this
       * function, we need to have already cleared this bit.  Calling that
       * function will result in a new TC interrupt being set.
       */
      DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);

      /* Disable everything, we will turn it back on when we are ready to send
       * the next packet.
       */
      /* Disable the DMA */
      DMA_Cmd(DMA2_Stream7, DISABLE);
      /* Disable USART DMA TX Requsts */
      USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);

      /* Put code to notify the orignator that the packet has been sent here! */
      if(fdud_txq_cb.fdud_txqs_ptr[fdud_txq_cb.tail].cb != NULL)
      {
         fdud_txq_cb.fdud_txqs_ptr[fdud_txq_cb.tail].cb(fdud_txq_cb.fdud_txqs_ptr[fdud_txq_cb.tail].cb_data);
      }
      /* Increment the tail */
      if(fdud_txq_cb.head != fdud_txq_cb.tail)
      {
         fdud_txq_cb.tail++;
         if(fdud_txq_cb.tail >= fdud_txq_cb.size)
         {
            fdud_txq_cb.tail = 0;
         }
         /* Call for the next packet to be sent if there are more waiting. */
         /* This function uses the current tail. */
         /* Is it really OK to put this here.  Once the next write happens, it
          * will trigger this TC Interrupt...maybe before we're done??? */
         full_duplex_usart_dma_write();
      }
      else
      {
         /* It is now safe for the main program to kick off another transfer. */
         fdud_txq_cb_mutex = 0;
      }


   }
}


void full_duplex_usart_dma_write(void)
{
   if(full_duplex_usart_dma_initialized)
   {
      /* Under the current version of the code...we shouldn't get back here
       * until all previous transmits are complete...  If we do...I guess
       * we'll chop off the last part of that packet.
       */
      /* /\* If a transmit was already underway, you wouldn't need to do this. */
      /*  * However, if one isn't, we would hang checking the flags below... */
      /*  * So...just do this to be safe. */
      /*  *\/ */
      /* /\* Enable USART DMA TX Requsts *\/ */
      /* USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE); */
      /* /\* Enable the DMA *\/ */
      /* DMA_Cmd(DMA2_Stream7, ENABLE); */

      /* /\* /\\* Wait for any previous transfer to complete. *\\/ *\/ */
      /* while (USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET); */
      /* while (DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7)==RESET); */

      /* Disable the DMA */
      DMA_Cmd(DMA2_Stream7, DISABLE);
      /* Disable USART DMA TX Requsts */
      USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);

      /* Clear DMA Transfer Complete Flags */
      DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
      /* Clear USART Transfer Complete Flags */
      USART_ClearFlag(USART1, USART_FLAG_TC);

      /* Set the length of data to transmit. */
      DMA2_Stream7->NDTR = fdud_txq_cb.fdud_txqs_ptr[fdud_txq_cb.tail].gp_ptr->packet_length;
      /* Set the pointer to the data. */
      DMA2_Stream7->M0AR = (uint32_t)(fdud_txq_cb.fdud_txqs_ptr[fdud_txq_cb.tail].gp_ptr->gp);

      /* Enable Transmit Complete Interrupt */
      DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);


      /* Enable USART DMA TX Requsts */
      USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
      /* Enable the DMA */
      DMA_Cmd(DMA2_Stream7, ENABLE);
   }
}
