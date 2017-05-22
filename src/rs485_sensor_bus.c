#include "rs485_sensor_bus.h"

/* Both master and slave will use the same size buffers. */
#define DMA_RX_BUFFER_SIZE (GP_MAX_PACKET_LENGTH * 4)


/* Buffers for raw data dma send and receive. */
uint8_t rs485_master_dma_tx_buffer[GP_MAX_PACKET_LENGTH];
uint8_t rs485_master_dma_rx_buffer[DMA_RX_BUFFER_SIZE];
/* Buffers for incoming and outgoing GenericPacket data. */
GenericPacketCircularBuffer gpcbs_master_tx;
GenericPacketCircularBuffer gpcbs_master_rx;
/* Let us know when we're up! */
uint8_t rs485_master_initialized = 0;


/* Buffers for raw data dma send and receive. */
uint8_t rs485_master_dma_tx_buffer[GP_MAX_PACKET_LENGTH];
uint8_t rs485_master_dma_rx_buffer[DMA_RX_BUFFER_SIZE];
/* Buffers for incoming and outgoing GenericPacket data. */
GenericPacketCircularBuffer gpcbs_master_tx;
GenericPacketCircularBuffer gpcbs_master_rx;
/* Let us know when we're up! */
uint8_t rs485_slave_initialized = 0;


void rs485_sensor_bus_init_master(void)
{
   /* Master RS485 is going to use:
    *  USART2
    *  Tx  -> D5, DMA1 - Channel 4 - Stream 6
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
   /* Connect PXx to USARTx_Tx*/
   GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
   /* Connect PXx to USARTx_Rx*/
   GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
   /* Configure USART Tx as alternate function */
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   /* Configure USART Rx as alternate function  */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
   GPIO_Init(GPIOB, &GPIO_InitStructure);


   /* USART_InitStructure.USART_BaudRate = 115200; */
   USART_InitStructure.USART_BaudRate = 3000000;
   /* USART_InitStructure.USART_BaudRate = 1500000; */
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

   USART_OverSampling8Cmd(USART1, ENABLE);

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
   DMA_InitStructure.DMA_BufferSize = GP_MAX_PACKET_LENGTH;
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
   DMA_Init(DMA2_Stream5, &DMA_InitStructure);
   /* Enable the USART Rx DMA request */
   USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
   /* Enable the DMA RX Stream */
   DMA_Cmd(DMA2_Stream5, ENABLE);

   /* Initialize our packet circular buffers. */
   retval = gpcb_initialize(&gpcbs_master_tx);
   retval = gpcb_initialize(&gpcbs_master_rx);

   /* Enable USART */
   USART_Cmd(USART1, ENABLE);

   /* Everyone else should hold tight until this is set! */
   rs485_master_initialized = 1;

}

void rs485_sensor_bus_init_slave(void)
{
   /* Slave RS485 is going to use:
    * USART6
    * Tx  -> C6, DMA2 - Channel 5 - Stream 6
    * Rx  -> C7, DMA2 - Channel 5 - Stream 1
    * T/R -> C8
    */


}
