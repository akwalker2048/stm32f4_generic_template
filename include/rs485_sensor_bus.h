#ifndef RS485_SENSOR_BUS_H
#define RS485_SENSOR_BUS_H

#include "stm32f4xx_conf.h"

#include "generic_packet.h"
#include "gp_proj_rs485_sb.h"

/* GP_CIRC_BUFFER_SIZE defaults to 16 within the generic packet code if you
 * don't override it.
 */
#define GP_CIRC_BUFFER_SIZE 8
#include "gp_circular_buffer.h"

#endif
