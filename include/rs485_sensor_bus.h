#ifndef RS485_SENSOR_BUS_H
#define RS485_SENSOR_BUS_H

/**
 * @file rs485_sensor_bus.h
 * @author Andrew K. Walker
 * @date 22 MAY 2017
 * @brief Header file for RS485 sensor bus (master and slave).
 *
 * This includes all of the defines and function prototypes for  the RS485
 * sensor bus.
 */


#include <stdint.h>

#include "stm32f4xx_conf.h"

#include "generic_packet.h"
#include "gp_proj_rs485_sb.h"
#include "gp_proj_universal.h"

/* Both master and slave will use the same size buffers. */
#define DMA_RX_BUFFER_SIZE (GP_MAX_PACKET_LENGTH * 4)
#define RAM_RX_BUFFER_SIZE (4 * DMA_RX_BUFFER_SIZE)

#define GP_CIRC_BUFFER_SIZE_TX  16
#define GP_CIRC_BUFFER_SIZE_RX  16

#define RS485_SENSOR_BUS_BAUD   3000000
#define RS485_SENSOR_BUS_SM_HZ     8000

#define RS485_MASTER_DELAY_TIME_USEC  500
#define RS485_MASTER_DELAY_TICKS   ((RS485_SENSOR_BUS_SM_HZ * RS485_MASTER_DELAY_TIME_USEC)/1000000)
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
#define SLAVE_ADDRESS 0x02

/** \enum rs485_master_states
 *
 * Describes the states in the RS485 master state machine.
 *
 */
typedef enum {RS485_MASTER_INIT,
              RS485_MASTER_FIND_ATTACHED_DEVICES,
              RS485_MASTER_QUERY_DEVICE,
              RS485_MASTER_AWAIT_RESPONSE,
              RS485_MASTER_DELAY,
              RS485_MASTER_IDLE,
              RS485_MASTER_ERROR} rs485_master_states;

/**
 *
 * \fn uint8_t rs485_sensor_bus_init_master(void);
 *
 * \brief Initialization of RS485 master function.
 *
 * Should be called one time after the device is initialized, clocks are
 * initialized, and SystemCoreClock is set.  Packet loss can occur if the
 * master device is initialized before the slave device.
 *
 * This function initializes the USART hardware used for the RS485 master
 * device.  Additionally, a GPIO is configured to be used to select between
 * receive and transmit mode.  A transmission complete interrupt is configured
 * to allow quick transition of the receive/transmit GPIO.  Finally, a timer
 * is configured to run a state machine that controls the flow of communication
 * between the master and slave devices.
 *
 * \todo Optimize the state machine timing relative to time required to process
 * packets.
 * \todo Finish documenting the rest of the functions.
 * \todo Improve the recovery when we lose our place parsing packets.  This may
 * involve using the state machine to know that enough time has elapsed between
 * packets that we should re-initialize the GenericPacket structure.
 *
 * \note Brings up the RS485 master communications.
 *
 * \param   None
 * \return  None
 *
 */
uint8_t rs485_sensor_bus_init_master(void);

/**
 *
 * \fn uint8_t rs485_master_spin(void);
 *
 * \brief Public fucntion for polled RS485 master functions.
 *
 * Functions that can, and should be polled outside of an interrupt.  This
 * includes parsing the packet data from the circular RAM buffer (the DMA
 * circular buffer is handled in a hard time interrupt).  This is because
 * parsing certain bytes can take longer than the timer interrupt.  For
 * example when the last byte of the packet is received and the checksum
 * must be calculated.
 *
 * \todo Add memory protection via callback for memory used for GenericPackets
 * that are transmitted.  We need to make sure that the memory isn't used again
 * until the transmission is complete.
 *
 * \param   None
 * \return  None
 *
 */
void rs485_master_spin(void);

/** \enum rs485_slave_states
 *
 * Describes the states in the RS485 slave state machine.
 *
 */
typedef enum {RS485_SLAVE_INIT,
              RS485_SLAVE_WAIT_FOR_QUERY,
              RS485_SLAVE_SEND_DATA,
              RS485_SLAVE_IDLE,
              RS485_SLAVE_ERROR} rs485_slave_states;


/**
 *
 * \fn uint8_t rs485_sensor_bus_init_slave(void);
 *
 * \brief Initialization of RS485 slave function.
 *
 * Should be called one time after the device is initialized, clocks are
 * initialized, and SystemCoreClock is set.  Packet loss can occur if the
 * master device is initialized before the slave device.
 *
 * This function initializes the USART hardware used for the RS485 master
 * device.  Additionally, a GPIO is configured to be used to select between
 * receive and transmit mode.  A transmission complete interrupt is configured
 * to allow quick transition of the receive/transmit GPIO.  Finally, a timer
 * is configured to run a state machine that controls the flow of communication
 * between the master and slave devices.
 *
 * \todo Optimize the state machine timing relative to time required to process
 * packets.
 * \todo Finish documenting the rest of the functions.
 * \todo Improve the recovery when we lose our place parsing packets.  This may
 * involve using the state machine to know that enough time has elapsed between
 * packets that we should re-initialize the GenericPacket structure.
 *
 * \note Brings up the RS485 master communications.
 *
 * \param   None
 * \return  None
 *
 */
uint8_t rs485_sensor_bus_init_slave(void);



/**
 *
 * \fn uint8_t rs485_slave_spin(void);
 *
 * \brief Public fucntion for polled RS485 slave functions.
 *
 * Functions that can, and should be polled outside of an interrupt.  This
 * includes parsing the packet data from the circular RAM buffer (the DMA
 * circular buffer is handled in a hard time interrupt).  This is because
 * parsing certain bytes can take longer than the timer interrupt.  For
 * example when the last byte of the packet is received and the checksum
 * must be calculated.
 *
 * \todo Add memory protection via callback for memory used for GenericPackets
 * that are transmitted.  We need to make sure that the memory isn't used again
 * until the transmission is complete.
 *
 * \param   None
 * \return  None
 *
 */
void rs485_slave_spin(void);

#endif
