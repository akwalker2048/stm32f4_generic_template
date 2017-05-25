#ifndef RS485_SENSOR_BUS_H
#define RS485_SENSOR_BUS_H

#include <stdint.h>

#include "stm32f4xx_conf.h"

#include "generic_packet.h"
#include "gp_proj_rs485_sb.h"
#include "gp_proj_universal.h"

/* Both master and slave will use the same size buffers. */
#define DMA_RX_BUFFER_SIZE (GP_MAX_PACKET_LENGTH * 4)
#define RAM_RX_BUFFER_SIZE (4 * DMA_RX_BUFFER_SIZE)


#define RS485_SENSOR_BUS_BAUD   1500000
#define RS485_SENSOR_BUS_SM_HZ     2000

#define RS485_MASTER_DELAY_TIME_MSEC  2000
#define RS485_MASTER_DELAY_TICKS   ((RS485_SENSOR_BUS_SM_HZ * RS485_MASTER_DELAY_TIME_MSEC)/1000)
#if(RS485_MASTER_DELAY_TICKS <= 1)
#error "RS485_MASTER_DELAY_TICKS too small!"
#endif

#define RS485_MASTER_RESPONSE_TIMEOUT_MSEC  500
#define RS485_MASTER_RESPONSE_TIMEOUT_TICKS ((RS485_SENSOR_BUS_SM_HZ * RS485_MASTER_RESPONSE_TIMEOUT_MSEC)/1000)
#if (RS485_MASTER_RESPONSE_TIMEOUT_TICKS <= 1)
#error "RS485_MASTER_RESPONSE_TIMEOUT_TICS too small!"
#endif


/* Sensor Bus Return Codes */
#define RS485_SB_SUCCESS      0x00
#define RS485_SB_INIT_FAIL    0x01

/* Hard code the slave address for now.  In the future, this will need to be
 * stored in flash memory.  Functions will be provided to allow the master to
 * set the address.  This could be used at manufacturing to configure sensors
 * for the robot.
 */
#define SLAVE_ADDRESS 0x01

typedef enum {RS485_MASTER_INIT,
              RS485_MASTER_FIND_ATTACHED_DEVICES,
              RS485_MASTER_QUERY_DEVICE,
              RS485_MASTER_AWAIT_RESPONSE,
              RS485_MASTER_DELAY,
              RS485_MASTER_IDLE,
              RS485_MASTER_ERROR} rs485_master_states;
uint8_t rs485_sensor_bus_init_master(void);
void rs485_sensor_bus_init_master_state_machine(void);
void rs485_sensor_bus_init_master_communications(void);
void rs485_sensor_bus_master_tx(void);
void rs485_sensor_bus_master_rx(void);
void rs485_master_state_change(rs485_master_states new_state, uint8_t reset_timer);
void rs485_master_write_dma(uint8_t *data, uint32_t length);
void rs485_master_process_rx_dma(void);
void rs485_master_process_rx_ram(void);
void rs485_master_handle_packets(void);



typedef enum {RS485_SLAVE_INIT,
              RS485_SLAVE_WAIT_FOR_QUERY,
              RS485_SLAVE_SEND_DATA,
              RS485_SLAVE_IDLE,
              RS485_SLAVE_ERROR} rs485_slave_states;
uint8_t rs485_sensor_bus_init_slave(void);
void rs485_sensor_bus_init_slave_state_machine(void);
void rs485_sensor_bus_init_slave_communications(void);
void rs485_sensor_bus_slave_tx(void);
void rs485_sensor_bus_slave_rx(void);
void rs485_slave_state_change(rs485_slave_states new_state, uint8_t reset_timer);
void rs485_slave_write_dma(uint8_t *data, uint32_t length);
void rs485_slave_process_rx_dma(void);
void rs485_slave_process_rx_ram(void);
void rs485_slave_handle_packets(void);




#endif
