#ifndef FULL_DUPLEX_USART_DMA_H
#define FULL_DUPLEX_USART_DMA_H

/* Hardware related defines for this processor. */
#include "stm32f4xx_conf.h"

/* High level packet information as we will be sending packets. */
#include "generic_packet.h"

/* Global Variables */
#ifdef INIT_VARIABLES
#define GLOBAL_VAR_FDUD
#else
#define GLOBAL_VAR_FDUD extern
#endif


/* Public Defines */
#define FDUD_SUCCESS              0x00
#define FDUD_FAIL                 0x01
#define FDUD_FAIL_NOT_INITIALIZED 0x02

/* DMA size is in bytes */
#define FDUD_TX_DMA_SIZE (GP_MAX_PACKET_LENGTH)
/* Queue size is in # of GenericPackets */
#define FDUD_TX_QUEUE_SIZE 32

/* DMA size is in bytes */
#define FDUD_RX_DMA_SIZE (GP_MAX_PACKET_LENGTH * 8)
/* Queue size is in # of GenericPackets */
#define FDUD_RX_QUEUE_SIZE 32


/* Typedefs for Tx Queue Callback */
typedef void (*FDUD_TxQueueCallback)(uint32_t callback_data);

typedef struct {
   GenericPacket *gp_ptr;
   FDUD_TxQueueCallback cb;
   uint32_t cb_data;
} FDUD_TxQueue_Struct;

typedef struct {
   FDUD_TxQueue_Struct *fdud_txqs_ptr;
   uint32_t size;
   uint32_t head;
   uint32_t tail;
} FDUD_TxQueue_CB_Struct;

/* Public Functions */
uint8_t full_duplex_usart_dma_up(void);
uint8_t full_duplex_usart_dma_init(void);
void full_duplex_usart_dma_service(void);
uint8_t full_duplex_usart_dma_add_to_queue(GenericPacket *gp_ptr, FDUD_TxQueueCallback callback_func, uint32_t callback_data);

uint8_t full_duplex_usart_dma_get_rx_packet(GenericPacket *gp_ptr);


#endif
