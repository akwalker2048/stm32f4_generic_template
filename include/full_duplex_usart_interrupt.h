#ifndef FULL_DUPLEX_USART_INTERRUPT_H
#define FULL_DUPLEX_USART_INTERRUPT_H

void init_usart_three(void);
uint8_t usart_three_write_byte(uint8_t data);
void process_usart3_buffer(void);


#endif

