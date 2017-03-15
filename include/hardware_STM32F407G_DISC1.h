#ifndef HARDWARE_STM32F407G_DISC1_H
#define HARDWARE_STM32F407G_DISC1_H


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
void init_usart(void);
void init_pushbutton(void);
void init_analog_input(void);
void init_tia(void);
void init_spi(void);
void init_i2c(void);

uint8_t usart_write_byte(uint8_t data);

void spi_cs_enable(void);
void spi_cs_disable(void);
uint8_t spi_read_byte(void);

void blocking_wait_ms(uint32_t delay_ms);

#endif
