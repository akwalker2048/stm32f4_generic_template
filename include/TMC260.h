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

typedef enum {MICROSTEP_CONFIG_256 = 0,
              MICROSTEP_CONFIG_128,
              MICROSTEP_CONFIG_64,
              MICROSTEP_CONFIG_32,
              MICROSTEP_CONFIG_16,
              MICROSTEP_CONFIG_8,
              MICROSTEP_CONFIG_4,
              MICROSTEP_CONFIG_2,
              MCIROSTEP_CONFIG_1} microstep_config;


/* TMC260 Function Return Codes */
#define TMC260_SUCCESS 0
#define TMC260_ERROR_INVALID_INPUT 1


#define TMC260_SPI_DELAY_COUNT 0x000001FF

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
#define TMC260_DRVCTRL_SDOFF_INIT          0x00000000
#define TMC260_DRVCTRL_SDOFF_PHA_DIR_SHIFT 17
#define TMC260_DRVCTRL_SDOFF_PHA_DIR_MASK  0x00020000
#define TMC260_DRVCTRL_SDOFF_PHA_CUR_SHIFT 9
#define TMC260_DRVCTRL_SDOFF_PHA_CUR_MASK  0x0001FE00
#define TMC260_DRVCTRL_SDOFF_PHB_DIR_SHIFT 8
#define TMC260_DRVCTRL_SDOFF_PHB_DIR_MASK  0x00000100
#define TMC260_DRVCTRL_SDOFF_PHB_CUR_SHIFT 0
#define TMC260_DRVCTRL_SDOFF_PHB_CUR_MASK  0x000000FF

/* DRVCTRL Register Shifts and Masks if SDOFF = 0 (Step/Dir Mode) */
#define TMC260_DRVCTRL_SDON_INIT         0x00000000
#define TMC260_DRVCTRL_SDON_INTPOL_SHIFT 9
#define TMC260_DRVCTRL_SDON_INTPOL_MASK  0x00000200
#define TMC260_DRVCTRL_SDON_DEDGE_SHIFT  8
#define TMC260_DRVCTRL_SDON_DEDGE_MASK   0x00000100
#define TMC260_DRVCTRL_SDON_MRES_SHIFT   0
#define TMC260_DRVCTRL_SDON_MRES_MASK    0x0000000F

/* DRVCONF Register Shifts and Masks */
#define TMC260_DRVCONF_INIT         0x000E0000
#define TMC260_DRVCONF_TST_SHIFT    16
#define TMC260_DRVCONF_TST_MASK     0x00010000
#define TMC260_DRVCONF_SLPH_SHIFT   14
#define TMC260_DRVCONF_SLPH_MASK    0x0000C000
#define TMC260_DRVCONF_SLPL_SHIFT   12
#define TMC260_DRVCONF_SLPL_MASK    0x00003000
#define TMC260_DRVCONF_DISS2G_SHIFT 10
#define TMC260_DRVCONF_DISS2G_MASK  0x00000400
#define TMC260_DRVCONF_TS2G_SHIFT   8
#define TMC260_DRVCONF_TS2G_MASK    0x00000300
#define TMC260_DRVCONF_SDOFF_SHIFT  7
#define TMC260_DRVCONF_SDOFF_MASK   0x00000080
#define TMC260_DRVCONF_VSENSE_SHIFT 6
#define TMC260_DRVCONF_VSENSE_MASK  0x00000040
#define TMC260_DRVCONF_RDSEL_SHIFT  4
#define TMC260_DRVCONF_RDSEL_MASK   0x00000030

/* CHOPCONF Register Shifts and Masks */
#define TMC260_CHOPCONF_INIT        0x00080000
#define TMC260_CHOPCONF_TBL_SHIFT   15
#define TMC260_CHOPCONF_TBL_MASK    0x00018000
#define TMC260_CHOPCONF_CHM_SHIFT   14
#define TMC260_CHOPCONF_CHM_MASK    0x00014000
#define TMC260_CHOPCONF_RNDTF_SHIFT 13
#define TMC260_CHOPCONF_RNDTF_MASK  0x00012000
#define TMC260_CHOPCONF_HDEC_SHIFT  11
#define TMC260_CHOPCONF_HDEC_MASK   0x00001800
#define TMC260_CHOPCONF_HEND_SHIFT  7
#define TMC260_CHOPCONF_HEND_MASK   0x00000780
#define TMC260_CHOPCONF_HSTRT_SHIFT 4
#define TMC260_CHOPCONF_HSTRT_MASK  0x00000070
#define TMC260_CHOPCONF_TOFF_SHIFT  0
#define TMC260_CHOPCONF_TOFF_MASK   0x0000000F

/* SMARTEN Register Shifts and Masks */
#define TMC260_SMARTEN_INIT         0x000A0000
#define TMC260_SMARTEN_SEIMIN_SHIFT 15
#define TMC260_SMARTEN_SEIMIN_MASK  0x00008000
#define TMC260_SMARTEN_SEDN_SHIFT   13
#define TMC260_SMARTEN_SEDN_MASK    0x00006000
#define TMC260_SMARTEN_SEMAX_SHIFT  8
#define TMC260_SMARTEN_SEMAX_MASK   0x00000F00
#define TMC260_SMARTEN_SEUP_SHIFT   5
#define TMC260_SMARTEN_SEUP_MASK    0x00000060
#define TMC260_SMARTEN_SEMIN_SHIFT  0
#define TMC260_SMARTEN_SEMIN_MASK   0x0000000F

/* SGCSCONF Register Shifts and Masks */
#define TMC260_SGCSCONF_INIT        0x000C0000
#define TMC260_SGCSCONF_SFILT_SHIFT 16
#define TMC260_SGCSCONF_SFILT_MASK  0x00010000
#define TMC260_SGCSCONF_SGT_SHIFT   8
#define TMC260_SGCSCONF_SGT_MASK    0x00007F00
#define TMC260_SGCSCONF_CS_SHIFT    0
#define TMC260_SGCSCONF_CS_MASK     0x0000001F

/* Read Response / Status Shifts and Masks */
/* For TMC260_STATUS_POSITION */
#define TMC260_STATUS_MSTEP_SHIFT  10
#define TMC260_STATUS_MSTEP_MASK   0x000FFC00
/* For TMC260_STATUS_STALLGUARD */
#define TMC260_STATUS_STALLGUARD_SHIFT  10
#define TMC260_STATUS_STALLGUARD_MASK   0x000FFC00
/* For TMC260_STATUS_CURRENT */
#define TMC260_STATUS_CUR_SG_SHIFT  15
#define TMC260_STATUS_CUR_SG_MASK   0x000F8000
#define TMC260_STATUS_CUR_SE_SHIFT  10
#define TMC260_STATUS_CUR_SE_MASK   0x00007C00
/* For All TMC_260_STATUS_* */
#define TMC260_STATUS_STST_SHIFT  7
#define TMC260_STATUS_STST_MASK   0x00000080
#define TMC260_STATUS_OLB_SHIFT   6
#define TMC260_STATUS_OLB_MASK    0x00000040
#define TMC260_STATUS_OLA_SHIFT   5
#define TMC260_STATUS_OLA_MASK    0x00000020
#define TMC260_STATUS_S2GB_SHIFT  4
#define TMC260_STATUS_S2GB_MASK   0x00000010
#define TMC260_STATUS_S2GA_SHIFT  3
#define TMC260_STATUS_S2GA_MASK   0x00000008
#define TMC260_STATUS_OTPW_SHIFT  2
#define TMC260_STATUS_OTPW_MASK   0x00000004
#define TMC260_STATUS_OT_SHIFT    1
#define TMC260_STATUS_OT_MASK     0x00000002
#define TMC260_STATUS_SG_SHIFT    0
#define TMC260_STATUS_SG_MASK     0x00000001

typedef enum {TMC260_STATUS_POSITION = 0,
              TMC260_STATUS_STALLGUARD,
              TMC260_STATUS_CURRENT} tmc260_status_types;


typedef struct {
   tmc260_status_types status_type;
   uint16_t position;
   uint16_t stall_guard;
   uint16_t current;
   uint8_t status_byte;
   uint8_t STST:1;
   uint8_t OLB:1;
   uint8_t OLA:1;
   uint8_t S2GB:1;
   uint8_t S2GA:1;
   uint8_t OTPW:1;
   uint8_t OT:1;
   uint8_t SG:1;
} tmc260_status_struct;


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

/**
 *
 * @fn void TMC260_enable(void)
 * @brief Enables the driver output.
 * @param None
 * @return None
 *
 */
void TMC260_enable(void);

/**
 *
 * @fn void TMC260_disable(void)
 * @brief Disables the driver output.
 * @param None
 * @return None
 *
 */
void TMC260_disable(void);

/**
 *
 * @fn void TMC260_dir_CW(void)
 * @brief Sets the state of the dir pin for CW shaft rotation.
 * @param None
 * @return None
 *
 */
void TMC260_dir_CW(void);

/**
 *
 * @fn void TMC260_dir_CCW(void)
 * @brief Sets the state of the dir pin for CCW shaft rotation.
 * @param None
 * @return None
 *
 */
void TMC260_dir_CCW(void);

/**
 *
 * @fn void TMC260_step(void)
 * @brief Take a step in the current direction.
 * @param None
 * @return None
 *
 * The motor will take a step at it's current microstep setting.  So, if the
 * driver is configured for 1/128 microsteps.  It will turn 1/128'th of a full
 * step.
 *
 */
void TMC260_step(void);

/**
 *
 * @fn void TMC260_status(void)
 * @brief Report the status of the driver.
 * @param tmc260_status_struct *status -> filled with all current status info
 * @param uint8_t send_packet -> 0=does nothing, !0=sends GenericPacket
 * @return None
 *
 * The returned status will be different based on the RSEL bits.
 *
 */
void TMC260_status(tmc260_status_struct *status, uint8_t send_packet);


/**
 * @todo Add functions to set TMC260 registers from outside the hardware
 *       driver.
 *
 *       Add functions to report the current register values to give visibility
 *       outside the driver.
 */


#endif
