/**
 * @file TMC260.h
 * @author Andrew K. Walker
 * @date 14 JUN 2017
 * @brief Firmware for Trinamic TMC260-PA stepper motor driver.
 *
 * Set of low level functions for controlling the the Trinamic TMC260-PA stepper
 * motor driver.  Higher level functions...like commanding a particular motor
 * profile should be done in another layer.
 */

#ifndef TMC260_H
#define TMC260_H

#include <stdint.h>
#include "stm32f4xx_conf.h"

typedef enum {MICROSTEP_CONFIG_1 = 0,
              MICROSTEP_CONFIG_2,
              MICROSTEP_CONFIG_4,
              MICROSTEP_CONFIG_16,
              MICROSTEP_CONFIG_32,
              MICROSTEP_CONFIG_64,
              MICROSTEP_CONFIG_128,
              MICROSTEP_CONFIG_256} microstep_config;


/* TMC260 Function Return Codes */
#define TMC260_SUCCESS 0
#define TMC260_ERROR_INVALID_INPUT 1


/** @todo Keep in mind that all registers will need to be shifted left 12 bits
 *  and transferred starting with the highest byte to lowest byte.  This will
 *  result in 12 extra bits being sent (or 4...if we only send the first 3
 *  bytes).  This is OK as long as there is only one TMC260 connected.  If
 *  more than one are connected, it would screw things up as the additional
 *  bytes would be shifted on out to the next device on the chain.
 *
 *  This is to do with the fact that the ST Micro peripheral works with either
 *  8bit or 16bit SPI frames.  Can't send exactly 20bits unless we bit bang.
 *  And we don't like to bit bang.
 */

/* DRVCTRL Register Shifts and Masks if SDOFF = 1 (SPI Mode) */
#define TMC260_DRVCTRL_SDOFF_PHA_DIR_SHIFT 17
#define TMC260_DRVCTRL_SDOFF_PHA_DIR_MASK  0x00020000
#define TMC260_DRVCTRL_SDOFF_PHA_CUR_SHIFT 9
#define TMC260_DRVCTRL_SDOFF_PHA_CUR_MASK  0x0001FE00
#define TMC260_DRVCTRL_SDOFF_PHB_DIR_SHIFT 8
#define TMC260_DRVCTRL_SDOFF_PHB_DIR_MASK  0x00000100
#define TMC260_DRVCTRL_SDOFF_PHB_CUR_SHIFT 0
#define TMC260_DRVCTRL_SDOFF_PHB_CUR_MASK  0x000000FF

/* DRVCTRL Register Shifts and Masks if SDOFF = 0 (Step/Dir Mode) */
#define TMC260_DRVCTRL_SDON_INTPOL_SHIFT 9
#define TMC260_DRVCTRL_SDON_INTPOL_MASK 0x00000200
#define TMC260_DRVCTRL_SDON_DEDGE_SHIFT 8
#define TMC260_DRVCTRL_SDON_DEDGE_MASK  0x00000100
#define TMC260_DRVCTRL_SDON_MRES_SHIFT 0
#define TMC260_DRVCTRL_SDON_MRES_MASK  0x0000000F

/* CHOPCONF Register Shifts and Masks */
#define TMC260_CHOPCONF_TBL_SHIFT 15
#define TMC260_CHOPCONF_TBL_MASK


/* SMARTEN Register Shifts and Masks */

/* SGCSCONF Register Shifts and Masks */

/* DRVCONF Register Shifts and Masks */


typedef struct {

} tmc260_status;

typedef struct {

} tmc260_config;


/**
 *
 * @fn void TMC260_initialize(void)
 * @brief Initializes the hardware and data structures for the TMC260.
 * @param None
 * @return None
 *
 * Calles private initialization functions to initialize the GPIO and SPI ports.
 *
 */
void TMC260_initialize(void);


#endif
