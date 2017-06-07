#ifndef LEPTON_FUNCTIONS_H
#define LEPTON_FUNCTIONS_H

#include "generic_packet.h"
#include "gp_proj_universal.h"
#include "gp_proj_thermal.h"

/* SPI - Software Chip Select - Active Low - B2 */
#define SPI_PIN_CS_AL    GPIO_Pin_2


void write_vospi(void);

/** @todo Move this code over to the new circular_buffer code. */
GenericPacket * get_next_vospi_ptr(void);
void increment_vospi_head(void);


void spi_cs_enable(void);
void spi_cs_disable(void);
uint8_t spi_read_byte(void);


void lepton_print_image_binary_background(void);
void lepton_transfer(void);

#endif
