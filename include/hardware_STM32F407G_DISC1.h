#ifndef HARDWARE_STM32F407G_DISC1_H
#define HARDWARE_STM32F407G_DISC1_H

#include "generic_packet.h"
#include "gp_receive.h"
#include "gp_proj_thermal.h"
#include "gp_proj_universal.h"

#ifndef GIT_REVISION
#define GIT_REVISION "generic-stm32f4-DEADBEEF"
#endif


/* Green  - D12 */
#define LED_PIN_GREEN    GPIO_Pin_12
/* Orange - D13 */
#define LED_PIN_ORANGE   GPIO_Pin_13
/* Red    - D14 */
#define LED_PIN_RED      GPIO_Pin_14
/* Blue   - D15 */
#define LED_PIN_BLUE     GPIO_Pin_15

/* SPI - Software Chip Select - Active Low - B2 */
#define SPI_PIN_CS_AL    GPIO_Pin_2


void init_gpio(void);
void init_systick(void);
void init_usart_three(void);
void init_usart_one(void);
void init_usart_one_dma(void);
void init_adc(void);

void init_pushbutton(void);
void init_analog_input(void);
void init_tia(void);
void init_spi(void);
void init_i2c(void);

uint8_t usart_write_byte(uint8_t data);
uint8_t usart_write_dma(uint8_t *data_ptr, uint32_t data_len);

void process_rx_buffer(void);

void spi_cs_enable(void);
void spi_cs_disable(void);
uint8_t spi_read_byte(void);

void blocking_wait_ms(uint32_t delay_ms);
void non_blocking_wait_ms(uint32_t delay_ms);

/* uint8_t add_gp_to_circ_buffer(GenericPacket packet); */
/* uint8_t send_gp_packets(void); */

void write_timestamps(void);
void write_vospi(void);

void write_code_version(void);

GenericPacket * get_next_vospi_ptr(void);
void increment_vospi_head(void);

uint8_t add_gp_to_outgoing(GenericPacket packet);
void write_outgoing(void);

uint8_t read_adc(float *vc14, float *vc15);

#endif
