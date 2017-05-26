#ifndef RX_PACKET_HANDLER_H
#define RX_PACKET_HANDLER_H

#include <stdlib.h>
#include <stdint.h>

#include "generic_packet.h"
#include "gp_proj_thermal.h"
#include "gp_proj_universal.h"
#include "gp_proj_sonar.h"
#include "gp_proj_motor.h"
#include "gp_proj_rs485_sb.h"

#include "full_duplex_usart_dma.h"

#include "tilt_motor_control.h"

#define RX_PACKET_HANDLER_GP_QUEUE_SIZE 16

void rx_packet_handler(GenericPacket *gp_ptr);
void rx_packet_handler_packet_send_callback(uint32_t new_tail);

#endif
