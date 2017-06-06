#ifndef LEPTON_FUNCTIONS_H
#define LEPTON_FUNCTIONS_H

#include "generic_packet.h"
#include "gp_proj_universal.h"
#include "gp_proj_thermal.h"

void write_vospi(void);

/** @todo Move this code over to the new circular_buffer code. */
GenericPacket * get_next_vospi_ptr(void);
void increment_vospi_head(void);


void lepton_print_image_binary_background(void);
void lepton_transfer(void);

#endif
