/**
 *  @file i2c_driver.h
 *  @author SurgeExperiments
 *
 *  @brief  This file contains prototypes for the i2c functions, function prototypes
 *          and defines used with i2c.
 *
 *  TODO:  I've set an arbitrary # of i2c errors before i2c functions return an err code.
 *         (TIMEOUT_THRESHOLD). This number needs to be adjusted to something less arbitrary
 *         once you add another IMU to get redundancy. The current value would crash the entire
 *         quad into the ground before an IMU switch would occur (haha).
 *         : try to calculate a reasonable upper bound on # of i2c errors given a dual IMU HW config
 *           and use that as TIMEOUT_THRESHOLD b4 an IMU switch?
 */

#ifndef I2C_DRIVER_
#define I2C_DRIVER_

#include "stm32f4xx.h"

/** Max number of failed attempts before an i2c function returns
 * NOTE: arbitrary number, needs to be tuned.
 */
#define TIMEOUT_THRESHOLD ((uint32_t)40000)

/*********************************
 *        DATA STRUCTURES        *
 *                               *
 ********************************/

typedef struct
{
    uint32_t I2C_ClockSpeed; /*!< Specifies the clock frequency.
                                  This parameter must be set to a value lower than 400kHz */

    uint16_t I2C_Mode; /*!< Specifies the I2C mode.
                            This parameter can be a value of @ref I2C_mode */

    uint16_t I2C_DutyCycle; /*!< Specifies the I2C fast mode duty cycle.
                                 This parameter can be a value of @ref I2C_duty_cycle_in_fast_mode */

    uint16_t I2C_OwnAddress1; /*!< Specifies the first device own address.
                                   This parameter can be a 7-bit or 10-bit address. */

    uint16_t I2C_Ack; /*!< Enables or disables the acknowledgement.
                           This parameter can be a value of @ref I2C_acknowledgement */

    uint16_t I2C_AcknowledgedAddress; /*!< Specifies if 7-bit or 10-bit address is acknowledged.
                                           This parameter can be a value of @ref I2C_acknowledged_address */
} I2C_InitTypeDef;

uint8_t i2c_initialize(I2C_TypeDef *I2Cx);
void i2c_setupRegisters(I2C_TypeDef *I2Cx, I2C_InitTypeDef *I2C_InitStruct);
void i2c_load_init_struct_fc(I2C_InitTypeDef *myStruct);
uint8_t i2c_check_connectivity(I2C_TypeDef *I2Cx, uint8_t address);
uint8_t i2c_stop(I2C_TypeDef *I2Cx);
uint8_t i2c_start(I2C_TypeDef *I2Cx, uint8_t targetAddress, uint8_t amIreceiver);
uint8_t i2c_write_byte(I2C_TypeDef *I2Cx, uint8_t myByte);
uint8_t i2c_read_ack(I2C_TypeDef *I2Cx, uint8_t *storeHere);
uint8_t i2c_read_nak(I2C_TypeDef *I2Cx, uint8_t *storeHere);

/*
 * Useful defines and notes!
 */

#define CR1_CLEAR_MASK ((uint16_t)0xFBF5) /*<! I2C registers Masks */
#define FLAG_MASK ((uint32_t)0x00FFFFFF)  /*<! I2C FLAG mask */
#define ITEN_MASK ((uint32_t)0x07000000)  /*<! I2C Interrupt Enable mask */

#define I2C_AcknowledgedAddress_7bit ((uint16_t)0x4000)
#define I2C_Mode_I2C ((uint16_t)0x0000)
#define I2C_Ack_Enable ((uint16_t)0x0400)
#define I2C_Ack_Disable ((uint16_t)0x0000)
#define I2C_DutyCycle_16_9 ((uint16_t)0x4000) /*!< I2C fast mode Tlow/Thigh = 16/9 */
#define I2C_DutyCycle_2 ((uint16_t)0xBFFF)    /*!< I2C fast mode Tlow/Thigh = 2 */

/* Exported constants --------------------------------------------------------*/

#define CR1_CLEAR_MASK ((uint16_t)0xFBF5) /*<! I2C registers Masks */
#define FLAG_MASK ((uint32_t)0x00FFFFFF)  /*<! I2C FLAG mask */
#define ITEN_MASK ((uint32_t)0x07000000)  /*<! I2C Interrupt Enable mask */

/** @defgroup I2C_Exported_Constants
 * @{
 */
#define IS_I2C_ALL_PERIPH(PERIPH) (((PERIPH) == I2C1) || \
                                   ((PERIPH) == I2C2) || \
                                   ((PERIPH) == I2C3))
/** @defgroup I2C_Digital_Filter
 * @{
 */

#define IS_I2C_DIGITAL_FILTER(FILTER) ((FILTER) <= 0x0000000F)
/**
 * @}
 */

/** @defgroup I2C_mode
 * @{
 */

/** @defgroup I2C_mode
 * @{
 */

#define I2C_Mode_I2C ((uint16_t)0x0000)
#define I2C_Mode_SMBusDevice ((uint16_t)0x0002)
#define I2C_Mode_SMBusHost ((uint16_t)0x000A)
#define IS_I2C_MODE(MODE) (((MODE) == I2C_Mode_I2C) ||         \
                           ((MODE) == I2C_Mode_SMBusDevice) || \
                           ((MODE) == I2C_Mode_SMBusHost))
/**
 * @}
 */

/** @defgroup I2C_duty_cycle_in_fast_mode
 * @{
 */

#define I2C_DutyCycle_16_9 ((uint16_t)0x4000) /*!< I2C fast mode Tlow/Thigh = 16/9 */
#define I2C_DutyCycle_2 ((uint16_t)0xBFFF)    /*!< I2C fast mode Tlow/Thigh = 2 */
#define IS_I2C_DUTY_CYCLE(CYCLE) (((CYCLE) == I2C_DutyCycle_16_9) || \
                                  ((CYCLE) == I2C_DutyCycle_2))
/**
 * @}
 */

/** @defgroup I2C_acknowledgement
 * @{
 */

#define I2C_Ack_Enable ((uint16_t)0x0400)
#define I2C_Ack_Disable ((uint16_t)0x0000)
#define IS_I2C_ACK_STATE(STATE) (((STATE) == I2C_Ack_Enable) || \
                                 ((STATE) == I2C_Ack_Disable))
/**
 * @}
 */

/** @defgroup I2C_transfer_direction
 * @{
 */

#define I2C_Direction_Transmitter ((uint8_t)0x00)
#define I2C_Direction_Receiver ((uint8_t)0x01)
#define IS_I2C_DIRECTION(DIRECTION) (((DIRECTION) == I2C_Direction_Transmitter) || \
                                     ((DIRECTION) == I2C_Direction_Receiver))
/**
 * @}
 */

/** @defgroup I2C_acknowledged_address
 * @{
 */

#define I2C_AcknowledgedAddress_7bit ((uint16_t)0x4000)
#define I2C_AcknowledgedAddress_10bit ((uint16_t)0xC000)
#define IS_I2C_ACKNOWLEDGE_ADDRESS(ADDRESS) (((ADDRESS) == I2C_AcknowledgedAddress_7bit) || \
                                             ((ADDRESS) == I2C_AcknowledgedAddress_10bit))
/**
 * @}
 */

/** @defgroup I2C_registers
 * @{
 */

#define I2C_Register_CR1 ((uint8_t)0x00)
#define I2C_Register_CR2 ((uint8_t)0x04)
#define I2C_Register_OAR1 ((uint8_t)0x08)
#define I2C_Register_OAR2 ((uint8_t)0x0C)
#define I2C_Register_DR ((uint8_t)0x10)
#define I2C_Register_SR1 ((uint8_t)0x14)
#define I2C_Register_SR2 ((uint8_t)0x18)
#define I2C_Register_CCR ((uint8_t)0x1C)
#define I2C_Register_TRISE ((uint8_t)0x20)
#define IS_I2C_REGISTER(REGISTER) (((REGISTER) == I2C_Register_CR1) ||  \
                                   ((REGISTER) == I2C_Register_CR2) ||  \
                                   ((REGISTER) == I2C_Register_OAR1) || \
                                   ((REGISTER) == I2C_Register_OAR2) || \
                                   ((REGISTER) == I2C_Register_DR) ||   \
                                   ((REGISTER) == I2C_Register_SR1) ||  \
                                   ((REGISTER) == I2C_Register_SR2) ||  \
                                   ((REGISTER) == I2C_Register_CCR) ||  \
                                   ((REGISTER) == I2C_Register_TRISE))
/**
 * @}
 */

/** @defgroup I2C_NACK_position
 * @{
 */

#define I2C_NACKPosition_Next ((uint16_t)0x0800)
#define I2C_NACKPosition_Current ((uint16_t)0xF7FF)
#define IS_I2C_NACK_POSITION(POSITION) (((POSITION) == I2C_NACKPosition_Next) || \
                                        ((POSITION) == I2C_NACKPosition_Current))
/**
 * @}
 */

/** @defgroup I2C_SMBus_alert_pin_level
 * @{
 */

#define I2C_SMBusAlert_Low ((uint16_t)0x2000)
#define I2C_SMBusAlert_High ((uint16_t)0xDFFF)
#define IS_I2C_SMBUS_ALERT(ALERT) (((ALERT) == I2C_SMBusAlert_Low) || \
                                   ((ALERT) == I2C_SMBusAlert_High))
/**
 * @}
 */

/** @defgroup I2C_PEC_position
 * @{
 */

#define I2C_PECPosition_Next ((uint16_t)0x0800)
#define I2C_PECPosition_Current ((uint16_t)0xF7FF)
#define IS_I2C_PEC_POSITION(POSITION) (((POSITION) == I2C_PECPosition_Next) || \
                                       ((POSITION) == I2C_PECPosition_Current))
/**
 * @}
 */

/** @defgroup I2C_interrupts_definition
 * @{
 */

#define I2C_IT_BUF ((uint16_t)0x0400)
#define I2C_IT_EVT ((uint16_t)0x0200)
#define I2C_IT_ERR ((uint16_t)0x0100)
#define IS_I2C_CONFIG_IT(IT) ((((IT) & (uint16_t)0xF8FF) == 0x00) && ((IT) != 0x00))
/**
 * @}
 */

/** @defgroup I2C_interrupts_definition
 * @{
 */

#define I2C_IT_SMBALERT ((uint32_t)0x01008000)
#define I2C_IT_TIMEOUT ((uint32_t)0x01004000)
#define I2C_IT_PECERR ((uint32_t)0x01001000)
#define I2C_IT_OVR ((uint32_t)0x01000800)
#define I2C_IT_AF ((uint32_t)0x01000400)
#define I2C_IT_ARLO ((uint32_t)0x01000200)
#define I2C_IT_BERR ((uint32_t)0x01000100)
#define I2C_IT_TXE ((uint32_t)0x06000080)
#define I2C_IT_RXNE ((uint32_t)0x06000040)
#define I2C_IT_STOPF ((uint32_t)0x02000010)
#define I2C_IT_ADD10 ((uint32_t)0x02000008)
#define I2C_IT_BTF ((uint32_t)0x02000004)
#define I2C_IT_ADDR ((uint32_t)0x02000002)
#define I2C_IT_SB ((uint32_t)0x02000001)

#define IS_I2C_CLEAR_IT(IT) ((((IT) & (uint16_t)0x20FF) == 0x00) && ((IT) != (uint16_t)0x00))

#define IS_I2C_GET_IT(IT) (((IT) == I2C_IT_SMBALERT) || ((IT) == I2C_IT_TIMEOUT) || \
                           ((IT) == I2C_IT_PECERR) || ((IT) == I2C_IT_OVR) ||       \
                           ((IT) == I2C_IT_AF) || ((IT) == I2C_IT_ARLO) ||          \
                           ((IT) == I2C_IT_BERR) || ((IT) == I2C_IT_TXE) ||         \
                           ((IT) == I2C_IT_RXNE) || ((IT) == I2C_IT_STOPF) ||       \
                           ((IT) == I2C_IT_ADD10) || ((IT) == I2C_IT_BTF) ||        \
                           ((IT) == I2C_IT_ADDR) || ((IT) == I2C_IT_SB))
/**
 * @}
 */

/** @defgroup I2C_flags_definition
 * @{
 */

/**
 * @brief  SR2 register flags
 */

#define I2C_FLAG_DUALF ((uint32_t)0x00800000)
#define I2C_FLAG_SMBHOST ((uint32_t)0x00400000)
#define I2C_FLAG_SMBDEFAULT ((uint32_t)0x00200000)
#define I2C_FLAG_GENCALL ((uint32_t)0x00100000)
#define I2C_FLAG_TRA ((uint32_t)0x00040000)
#define I2C_FLAG_BUSY ((uint32_t)0x00020000)
#define I2C_FLAG_MSL ((uint32_t)0x00010000)

/**
 * @brief  SR1 register flags
 */

#define I2C_FLAG_SMBALERT ((uint32_t)0x10008000)
#define I2C_FLAG_TIMEOUT ((uint32_t)0x10004000)
#define I2C_FLAG_PECERR ((uint32_t)0x10001000)
#define I2C_FLAG_OVR ((uint32_t)0x10000800)
#define I2C_FLAG_AF ((uint32_t)0x10000400)
#define I2C_FLAG_ARLO ((uint32_t)0x10000200)
#define I2C_FLAG_BERR ((uint32_t)0x10000100)
#define I2C_FLAG_TXE ((uint32_t)0x10000080)
#define I2C_FLAG_RXNE ((uint32_t)0x10000040)
#define I2C_FLAG_STOPF ((uint32_t)0x10000010)
#define I2C_FLAG_ADD10 ((uint32_t)0x10000008)
#define I2C_FLAG_BTF ((uint32_t)0x10000004)
#define I2C_FLAG_ADDR ((uint32_t)0x10000002)
#define I2C_FLAG_SB ((uint32_t)0x10000001)

#define IS_I2C_CLEAR_FLAG(FLAG) ((((FLAG) & (uint16_t)0x20FF) == 0x00) && ((FLAG) != (uint16_t)0x00))

#define IS_I2C_GET_FLAG(FLAG) (((FLAG) == I2C_FLAG_DUALF) || ((FLAG) == I2C_FLAG_SMBHOST) ||      \
                               ((FLAG) == I2C_FLAG_SMBDEFAULT) || ((FLAG) == I2C_FLAG_GENCALL) || \
                               ((FLAG) == I2C_FLAG_TRA) || ((FLAG) == I2C_FLAG_BUSY) ||           \
                               ((FLAG) == I2C_FLAG_MSL) || ((FLAG) == I2C_FLAG_SMBALERT) ||       \
                               ((FLAG) == I2C_FLAG_TIMEOUT) || ((FLAG) == I2C_FLAG_PECERR) ||     \
                               ((FLAG) == I2C_FLAG_OVR) || ((FLAG) == I2C_FLAG_AF) ||             \
                               ((FLAG) == I2C_FLAG_ARLO) || ((FLAG) == I2C_FLAG_BERR) ||          \
                               ((FLAG) == I2C_FLAG_TXE) || ((FLAG) == I2C_FLAG_RXNE) ||           \
                               ((FLAG) == I2C_FLAG_STOPF) || ((FLAG) == I2C_FLAG_ADD10) ||        \
                               ((FLAG) == I2C_FLAG_BTF) || ((FLAG) == I2C_FLAG_ADDR) ||           \
                               ((FLAG) == I2C_FLAG_SB))
/**
 * @}
 */

/** @defgroup I2C_Events
 * @{
 */

/**
 ===============================================================================
               I2C Master Events (Events grouped in order of communication)
 ===============================================================================
 */

/**
 * @brief  Communication start
 *
 * After sending the START condition (I2C_GenerateSTART() function) the master
 * has to wait for this event. It means that the Start condition has been correctly
 * released on the I2C bus (the bus is free, no other devices is communicating).
 *
 */
/* --EV5 */
#define I2C_EVENT_MASTER_MODE_SELECT ((uint32_t)0x00030001) /* BUSY, MSL and SB flag */

/**
 * @brief  Address Acknowledge
 *
 * After checking on EV5 (start condition correctly released on the bus), the
 * master sends the address of the slave(s) with which it will communicate
 * (I2C_Send7bitAddress() function, it also determines the direction of the communication:
 * Master transmitter or Receiver). Then the master has to wait that a slave acknowledges
 * his address. If an acknowledge is sent on the bus, one of the following events will
 * be set:
 *
 *  1) In case of Master Receiver (7-bit addressing): the I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED
 *     event is set.
 *
 *  2) In case of Master Transmitter (7-bit addressing): the I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED
 *     is set
 *
 *  3) In case of 10-Bit addressing mode, the master (just after generating the START
 *  and checking on EV5) has to send the header of 10-bit addressing mode (I2C_SendData()
 *  function). Then master should wait on EV9. It means that the 10-bit addressing
 *  header has been correctly sent on the bus. Then master should send the second part of
 *  the 10-bit address (LSB) using the function I2C_Send7bitAddress(). Then master
 *  should wait for event EV6.
 *
 */

/* --EV6 */
#define I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ((uint32_t)0x00070082) /* BUSY, MSL, ADDR, TXE and TRA flags */
#define I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ((uint32_t)0x00030002)    /* BUSY, MSL and ADDR flags */
/* --EV9 */
#define I2C_EVENT_MASTER_MODE_ADDRESS10 ((uint32_t)0x00030008) /* BUSY, MSL and ADD10 flags */

/**
 * @brief Communication events
 *
 * If a communication is established (START condition generated and slave address
 * acknowledged) then the master has to check on one of the following events for
 * communication procedures:
 *
 * 1) Master Receiver mode: The master has to wait on the event EV7 then to read
 *    the data received from the slave (I2C_ReceiveData() function).
 *
 * 2) Master Transmitter mode: The master has to send data (I2C_SendData()
 *    function) then to wait on event EV8 or EV8_2.
 *    These two events are similar:
 *     - EV8 means that the data has been written in the data register and is
 *       being shifted out.
 *     - EV8_2 means that the data has been physically shifted out and output
 *       on the bus.
 *     In most cases, using EV8 is sufficient for the application.
 *     Using EV8_2 leads to a slower communication but ensure more reliable test.
 *     EV8_2 is also more suitable than EV8 for testing on the last data transmission
 *     (before Stop condition generation).
 *
 *  @note In case the  user software does not guarantee that this event EV7 is
 *        managed before the current byte end of transfer, then user may check on EV7
 *        and BTF flag at the same time (ie. (I2C_EVENT_MASTER_BYTE_RECEIVED | I2C_FLAG_BTF)).
 *        In this case the communication may be slower.
 *
 */

/* Master RECEIVER mode -----------------------------*/
/* --EV7 */
#define I2C_EVENT_MASTER_BYTE_RECEIVED ((uint32_t)0x00030040) /* BUSY, MSL and RXNE flags */

/* Master TRANSMITTER mode --------------------------*/
/* --EV8 */
#define I2C_EVENT_MASTER_BYTE_TRANSMITTING ((uint32_t)0x00070080) /* TRA, BUSY, MSL, TXE flags */
/* --EV8_2 */
#define I2C_EVENT_MASTER_BYTE_TRANSMITTED ((uint32_t)0x00070084) /* TRA, BUSY, MSL, TXE and BTF flags */
// #define I2C_FLAG_AF                     ((uint32_t)0x400)

/**
 ===============================================================================
               I2C Slave Events (Events grouped in order of communication)
 ===============================================================================
 */

/**
 * @brief  Communication start events
 *
 * Wait on one of these events at the start of the communication. It means that
 * the I2C peripheral detected a Start condition on the bus (generated by master
 * device) followed by the peripheral address. The peripheral generates an ACK
 * condition on the bus (if the acknowledge feature is enabled through function
 * I2C_AcknowledgeConfig()) and the events listed above are set :
 *
 * 1) In normal case (only one address managed by the slave), when the address
 *   sent by the master matches the own address of the peripheral (configured by
 *   I2C_OwnAddress1 field) the I2C_EVENT_SLAVE_XXX_ADDRESS_MATCHED event is set
 *   (where XXX could be TRANSMITTER or RECEIVER).
 *
 * 2) In case the address sent by the master matches the second address of the
 *   peripheral (configured by the function I2C_OwnAddress2Config() and enabled
 *   by the function I2C_DualAddressCmd()) the events I2C_EVENT_SLAVE_XXX_SECONDADDRESS_MATCHED
 *   (where XXX could be TRANSMITTER or RECEIVER) are set.
 *
 * 3) In case the address sent by the master is General Call (address 0x00) and
 *   if the General Call is enabled for the peripheral (using function I2C_GeneralCallCmd())
 *   the following event is set I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED.
 *
 */

/* --EV1  (all the events below are variants of EV1) */
/* 1) Case of One Single Address managed by the slave */
#define I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED ((uint32_t)0x00020002)    /* BUSY and ADDR flags */
#define I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED ((uint32_t)0x00060082) /* TRA, BUSY, TXE and ADDR flags */

/* 2) Case of Dual address managed by the slave */
#define I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED ((uint32_t)0x00820000)    /* DUALF and BUSY flags */
#define I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED ((uint32_t)0x00860080) /* DUALF, TRA, BUSY and TXE flags */

/* 3) Case of General Call enabled for the slave */
#define I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED ((uint32_t)0x00120000) /* GENCALL and BUSY flags */

/**
 * @brief  Communication events
 *
 * Wait on one of these events when EV1 has already been checked and:
 *
 * - Slave RECEIVER mode:
 *     - EV2: When the application is expecting a data byte to be received.
 *     - EV4: When the application is expecting the end of the communication: master
 *       sends a stop condition and data transmission is stopped.
 *
 * - Slave Transmitter mode:
 *    - EV3: When a byte has been transmitted by the slave and the application is expecting
 *      the end of the byte transmission. The two events I2C_EVENT_SLAVE_BYTE_TRANSMITTED and
 *      I2C_EVENT_SLAVE_BYTE_TRANSMITTING are similar. The second one can optionally be
 *      used when the user software doesn't guarantee the EV3 is managed before the
 *      current byte end of transfer.
 *    - EV3_2: When the master sends a NACK in order to tell slave that data transmission
 *      shall end (before sending the STOP condition). In this case slave has to stop sending
 *      data bytes and expect a Stop condition on the bus.
 *
 *  @note In case the  user software does not guarantee that the event EV2 is
 *        managed before the current byte end of transfer, then user may check on EV2
 *        and BTF flag at the same time (ie. (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_BTF)).
 *        In this case the communication may be slower.
 *
 */

/* Slave RECEIVER mode --------------------------*/
/* --EV2 */
#define I2C_EVENT_SLAVE_BYTE_RECEIVED ((uint32_t)0x00020040) /* BUSY and RXNE flags */
/* --EV4  */
#define I2C_EVENT_SLAVE_STOP_DETECTED ((uint32_t)0x00000010) /* STOPF flag */

/* Slave TRANSMITTER mode -----------------------*/
/* --EV3 */
#define I2C_EVENT_SLAVE_BYTE_TRANSMITTED ((uint32_t)0x00060084)  /* TRA, BUSY, TXE and BTF flags */
#define I2C_EVENT_SLAVE_BYTE_TRANSMITTING ((uint32_t)0x00060080) /* TRA, BUSY and TXE flags */
/* --EV3_2 */
#define I2C_EVENT_SLAVE_ACK_FAILURE ((uint32_t)0x00000400) /* AF flag */

/*
 ===============================================================================
                          End of Events Description
 ===============================================================================
 */

#define IS_I2C_EVENT(EVENT) (((EVENT) == I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED) ||           \
                             ((EVENT) == I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED) ||              \
                             ((EVENT) == I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED) ||     \
                             ((EVENT) == I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED) ||        \
                             ((EVENT) == I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED) ||            \
                             ((EVENT) == I2C_EVENT_SLAVE_BYTE_RECEIVED) ||                         \
                             ((EVENT) == (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_DUALF)) ||      \
                             ((EVENT) == (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_GENCALL)) ||    \
                             ((EVENT) == I2C_EVENT_SLAVE_BYTE_TRANSMITTED) ||                      \
                             ((EVENT) == (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_DUALF)) ||   \
                             ((EVENT) == (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_GENCALL)) || \
                             ((EVENT) == I2C_EVENT_SLAVE_STOP_DETECTED) ||                         \
                             ((EVENT) == I2C_EVENT_MASTER_MODE_SELECT) ||                          \
                             ((EVENT) == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) ||            \
                             ((EVENT) == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) ||               \
                             ((EVENT) == I2C_EVENT_MASTER_BYTE_RECEIVED) ||                        \
                             ((EVENT) == I2C_EVENT_MASTER_BYTE_TRANSMITTED) ||                     \
                             ((EVENT) == I2C_EVENT_MASTER_BYTE_TRANSMITTING) ||                    \
                             ((EVENT) == I2C_EVENT_MASTER_MODE_ADDRESS10) ||                       \
                             ((EVENT) == I2C_EVENT_SLAVE_ACK_FAILURE))

#define IS_I2C_OWN_ADDRESS1(ADDRESS1) ((ADDRESS1) <= 0x3FF)

#define IS_I2C_CLOCK_SPEED(SPEED) (((SPEED) >= 0x1) && ((SPEED) <= 400000))

/*
The interface can operate in one of the four following modes:
- Slave transmitter
- Slave receiver
- Master transmitter
- Master receive

By default, it operates in slave mode. The interface automatically switches from slave to master,
after it generates a START condition and from master to slave, if an arbitration loss or a Stop generation occurs,
allowing multimaster capability

In Master mode, the I2C interface initiates a data transfer and generates the clock signal.
A serial data transfer always begins with a start condition and ends with a stop condition.
Both start and stop conditions are generated in master mode by software.

Data and addresses are transferred as 8-bit bytes, MSB first.
The first byte(s) following the start condition contain the address (one in 7-bit mode, two in 10-bit mode).
The address is always transmitted in Master mode.

A 9th clock pulse follows the 8 clock cycles of a byte transfer,
during which the receiver must send an acknowledge bit to the transmitter

Acknowledge may be enabled or disabled by software.
The I2C interface addresses (dual addressing 7-bit/ 10-bit and/or general call address) can be selected by software.

The peripheral input clock must be programmed in the I2C_CR2 register in order to generate correct timings.
The peripheral input clock frequency must be at least:
- 2 MHz in Sm mode
- 4 MHz in Fm mode

In 10-bit mode, after receiving the address sequence the slave is always in Receiver mode.
It will enter Transmitter mode on receiving a repeated Start condition followed by the header sequence with matching address
bits and the least significant bit set (11110xx1). The TRA bit indicates whether the slave is in Receiver or Transmitter mode.


I2C master mode
In Master mode, the I2C interface initiates a data transfer and generates the clock signal.
A serial data transfer always begins with a Start condition and ends with a Stop condition.
Master mode is selected as soon as the Start condition is generated on the bus with a START bit.
The following is the required sequence in master mode:
- Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
- Configure the clock control registers
- Configure the rise time register
- Program the I2C_CR1 register to enable the peripheral
- Set the START bit in the I2C_CR1 register to generate a Start condition The peripheral input clock frequency must be at least:
- 2 MHz in Sm mode � 4 MHz in Fm mode

SCL master clock generation
The CCR bits are used to generate the high and low level of the SCL clock,
starting from the generation of the rising and falling edge (respectively).
As a slave may stretch the SCL line, the peripheral checks the SCL input
from the bus at the end of the time programmed in TRISE bits after rising edge generation.
- If the SCL line is low, it means that a slave is stretching the bus, and the high level counter stops until the SCL line is detected high.
  This allows to guarantee the minimum HIGH period of the SCL clock parameter.
- If the SCL line is high, the high level counter keeps on counting. Indeed, the feedback loop from the SCL rising
  edge generation by the peripheral to the SCL rising edge detection by the peripheral takes time even if no slave stretches the clock.
  This loopback duration is linked to the SCL rising time (impacting SCL VIH input detection),
  plus delay due to the noise filter present on the SCL input path, plus delay due to internal SCL input synchronization with APB clock.
  The maximum time used by the feedback loop is programmed in the TRISE bits, so that the SCL frequency remains stable whatever the SCL rising time.

Start condition
Setting the START bit causes the interface to generate a Start condition and to switch to Master mode (MSL bit set) when the BUSY bit is cleared.
Note: In master mode, setting the START bit causes the interface to generate a ReStart condition at the end of the current byte transfer.
Once the Start condition is sent:
- The SB bit is set by hardware and an interrupt is generated if the ITEVFEN bit is set. Then the master waits for a read of the
  SR1 register followed by a write in the DR register with the Slave address

Slave address transmission
Then the slave address is sent to the SDA line via the internal shift register.
- In 7-bit addressing mode, one address byte is sent. As soon as the address byte is sent,
- The ADDR bit is set by hardware and an interrupt is generated if the ITEVFEN bit is set.
Then the master waits for a read of the SR1 register followed by a read of the SR2 register

The master can decide to enter Transmitter or Receiver mode depending on the LSB of the slave address sent.
- In 7-bit addressing mode,
- To enter Transmitter mode, a master sends the slave address with LSB reset.
- To enter Receiver mode, a master sends the slave address with LSB set.

Master transmitter
Following the address transmission and after clearing ADDR, the master sends bytes from the DR register to the SDA line via the internal shift register.
The master waits until the first data byte is written into I2C_DR.
When the acknowledge pulse is received, the TxE bit is set by hardware and an interrupt is generated if the ITEVFEN and ITBUFEN bits are set.
If TxE is set and a data byte was not written in the DR register before the end of the last data transmission, BTF is set and the interface
waits until BTF is cleared by a write to I2C_DR, stretching SCL low.
: Closing the communication After the last byte is written to the DR register,
the STOP bit is set by software to generate a Stop condition.
The interface automatically goes back to slave mode (MSL bit cleared).
Note: Stop condition should be programmed during EV8_2 event, when either TxE or BTF is set.


Master receiver
Following the address transmission and after clearing ADDR, the I2C interface enters Master Receiver mode.
In this mode the interface receives bytes from the SDA line into the DR register via the internal shift register.
After each byte the interface generates in sequence: 1. An acknowledge pulse if the ACK bit is set 2. The RxNE bit
is set and an interrupt is generated if the ITEVFEN and ITBUFEN bits are set (see Figure 165 Transfer sequencing EV7).
If the RxNE bit is set and the data in the DR register is not read before the end of the last data reception, the BTF bit
is set by hardware and the interface waits until BTF is cleared by a read in the DR register, stretching SCL low.
: Closing the communication
The master sends a NACK for the last byte received from the slave.
After receiving this NACK, the slave releases the control of the SCL and SDA lines.
Then the master can send a Stop/Restart condition.
1. To generate the nonacknowledge pulse after the last received data byte, the ACK bit must be cleared just after reading
the second last data byte (after second last RxNE event).
2. In order to generate the Stop/Restart condition, software must set the STOP/START bit after reading the second last data byte
(after the second last RxNE event).
3. In case a single byte has to be received, the Acknowledge disable
is made during EV6 (before ADDR flag is cleared) and the STOP condition generation is made after EV6.
After the Stop condition generation, the interface goes automatically back to slave mode (MSL bit cleared).


The procedures described below are recommended if the EV7-1 software sequence is not completed before the ACK pulse of the current byte transfer.
These procedures must be followed to make sure:
- The ACK bit is set low on time before the end of the last data reception
- The STOP bit is set high after the last data reception without reception of supplementary data.

For 2-byte reception:
- Wait until ADDR = 1 (SCL stretched low until the ADDR flag is cleared)
- Set ACK low, set POS high
- Clear ADDR flag
- Wait until BTF = 1 (Data 1 in DR, Data2 in shift register, SCL stretched low until a data 1 is read)
- Set STOP high
- Read data 1 and 2

For N >2 -byte reception, from N-2 data reception
- Wait until BTF = 1 (data N-2 in DR, data N-1 in shift register, SCL stretched low until data N-2 is read)
- Set ACK low
- Read data N-2
- Wait until BTF = 1 (data N-1 in DR, data N in shift register, SCL stretched low until a data N-1 is read)
- Set STOP high
- Read data N-1 and N

ERRORS:

Bus error (BERR)
This error occurs when the I2C interface detects an external Stop or Start condition during an address or a data transfer.
In this case:
- the BERR bit is set and an interrupt is generated if the ITERREN bit is set
- in Slave mode: data are discarded and the lines are released by hardware:
- in case of a misplaced Start, the slave considers it is a restart and waits for an address, or a Stop condition
- in case of a misplaced Stop, the slave behaves like for a Stop condition and the lines are released by hardware
- In Master mode: the lines are not released and the state of the current transmission is not affected.
It is up to the software to abort or not the current transmission Acknowledge failure (AF)
This error occurs when the interface detects a nonacknowledge bit.
In this case:
- the AF bit is set and an interrupt is generated if the ITERREN bit is set
- a transmitter which receives a NACK must reset the communication:
- If Slave: lines are released by hardware
- If Master: a Stop or repeated Start condition must be generated by software Arbitration lost (ARLO)
This error occurs when the I2C interface detects an arbitration lost condition.
In this case,
- the ARLO bit is set by hardware (and an interrupt is generated if the ITERREN bit is set)
- the I2C Interface goes automatically back to slave mode (the MSL bit is cleared). When the I2C loses the arbitration,
it is not able to acknowledge its slave address in the same transfer, but it can acknowledge it after a repeated Start from the winning master.
- lines are released by hardware

Overrun/underrun error (OVR)
An overrun error can occur in slave mode when clock stretching is disabled and the I2C interface is receiving data.
The interface has received a byte (RxNE=1) and the data in DR has not been read, before the next byte is received by the interface.
In this case,
- The last received byte is lost.
- In case of Overrun error, software should clear the RxNE bit and the transmitter should re-transmit the last received byte.
  Underrun error can occur in slave mode when clock stretching is disabled and the I2C interface is transmitting data.
  The interface has not updated the DR with the next byte (TxE=1), before the clock comes for the next byte. In this case,
  - The same byte in the DR register will be sent again
  - The user should make sure that data received on the receiver side during an underrun error are discarded and that the next bytes are written within the
  clock low time specified in the I2C bus standard. For the first byte to be transmitted, the DR must be written after ADDR
  is cleared and before the first SCL rising edge. If not possible, the receiver must discard the first data.


SDA/SCL line control
- If clock stretching is enabled:
- Transmitter mode: If TxE=1 and BTF=1: the interface holds the clock line low before transmission to wait for the microcontroller to write the byte in the Data
  Register (both buffer and shift register are empty).
- Receiver mode: If RxNE=1 and BTF=1: the interface holds the clock line low after reception to wait for
  the microcontroller to read the byte in the Data Register (both buffer and shift register are full).
- If clock stretching is disabled in Slave mode:
- Overrun Error in case of RxNE=1 and no read of DR has been done before the next byte is received. The last received byte is lost.
- Underrun Error in case TxE=1 and no write into DR has been done before the next byte must be transmitted. The same byte will be sent again.
- Write Collision not managed.

DMA requests
DMA requests (when enabled) are generated only for data transfer.
DMA requests are generated by Data Register becoming empty in transmission and Data Register becoming full in reception.
The DMA must be initialized and enabled before the I2C data transfer. The DMAEN bit must be set in the I2C_CR2 register before the ADDR event.
In master mode or in slave mode when clock stretching is enabled, the DMAEN bit can also be set during the ADDR event, before clearing the ADDR flag.
The DMA request must be served before the end of the current byte transfer. When the number of data transfers which has been programmed for the corresponding
DMA stream is reached, the DMA controller sends an End of Transfer EOT signal to the I2C interface and generates a Transfer Complete interrupt if enabled:
- Master transmitter: In the interrupt routine after the EOT interrupt, disable DMA requests then wait for a BTF event before programming the Stop condition.
- Master receiver � When the number of bytes to be received is equal to or greater than two, the DMA controller sends a hardware signal, EOT_1, corresponding to the
  last but one data byte (number_of_bytes  1). If, in the I2C_CR2 register, the LAST bit is set, I2C automatically sends a NACK after the next byte
  following EOT_1.

  The user can generate a Stop condition in the DMA Transfer Complete interrupt routine if enabled.
  - When a single byte must be received: the NACK must be programmed during EV6 event, i.e. program ACK=0 when ADDR=1, before clearing ADDR flag.
    Then the user can program the STOP condition either after clearing ADDR flag, or in the DMA Transfer Complete interrupt routine. Transmission
    using DMA DMA mode can be enabled for transmission by setting the DMAEN bit in the I2C_CR2 register.
    Data will be loaded from a Memory area configured using the DMA peripheral (refer to the DMA specification) to the I2C_DR register whenever the TxE bit is set.
    To map a DMA stream x for I2C transmission (where x is the stream number), perform the following sequence:
    1. Set the I2C_DR register address in the DMA_SxPAR register. The data will be moved to this address from the memory after each TxE event.
    2. Set the memory address in the DMA_SxMA0R register (and in DMA_SxMA1R register in the case of a bouble buffer mode). The data will be loaded into I2C_DR from this memory after each TxE event.
    3. Configure the total number of bytes to be transferred in the DMA_SxNDTR register. After each TxE event, this value will be decremented.
    4. Configure the DMA stream priority using the PL[0:1] bits in the DMA_SxCR register
    5. Set the DIR bit in the DMA_SxCR register and configure interrupts after half transfer or full transfer depending on application requirements.
    6. Activate the stream by setting the EN bit in the DMA_SxCR register. When the number of data transfers which has been programmed in the DMA
    Controller registers is reached, the DMA controller sends an End of Transfer EOT/ EOT_1 signal to the I2C interface and the DMA generates an interrupt,
    if enabled, on the DMA stream interrupt vector.

    Note: Do not enable the ITBUFEN bit in the I2C_CR2 register if DMA is used for transmission.

Reception using DMA
DMA mode can be enabled for reception by setting the DMAEN bit in the I2C_CR2 register.
Data will be loaded from the I2C_DR register to a Memory area configured using the DMA peripheral (refer to the DMA specification)
whenever a data byte is received. To map a DMA stream x for I2C reception (where x is the stream number), perform the following sequence:
1. Set the I2C_DR register address in DMA_SxPAR register. The data will be moved from this address to the memory after each RxNE event.
2. Set the memory address in the DMA_SxMA0R register (and in DMA_SxMA1R register in the case of a bouble buffer mode). The data will be loaded from the I2C_DR register to this memory area after each RxNE event.
3. Configure the total number of bytes to be transferred in the DMA_SxNDTR register. After each RxNE event, this value will be decremented.
4. Configure the stream priority using the PL[0:1] bits in the DMA_SxCR register
5. Reset the DIR bit and configure interrupts in the DMA_SxCR register after half transfer or full transfer depending on application requirements.
6. Activate the stream by setting the EN bit in the DMA_SxCR register. When the number of data transfers which has been programmed in the DMA Controller registers is reached,
   the DMA controller sends an End of Transfer EOT/ EOT_1 signal to the I2C interface and DMA generates an interrupt, if enabled, on the DMA stream interrupt vector.

   Note: Do not enable the ITBUFEN bit in the I2C_CR2 register if DMA is used for reception.

i2c interrupts:
See page 490

I2C debug mode
When the microcontroller enters the debug mode (Cortex�-M4 with FPU core halted), the SMBUS timeout either continues to work normally or stops,
depending on the DBG_I2Cx_SMBUS_TIMEOUT configuration bits in the DBG module

i2c registers:
See page 492

: I2C Control register 1 (I2C_CR1) : bit 10 ack enable: supress when receiving only 1 byte (as a nack is needed)
: I2C Control register 2 (I2C_CR2) : bit 11 DMAEN
: I2C Control register 2 (I2C_CR2) : bit Bit 8 ITERREN: Error interrupt enable
: I2C Control register 2 (I2C_CR2) : Bits 5:0 FREQ[5:0]: Peripheral clock frequency The FREQ bits must be configured with the APB clock frequency value (I2C peripheral connected to APB).
  The FREQ field is used by the peripheral to generate data setup and hold times compliant with the I2C specifications.
  The minimum allowed frequency is 2 MHz, the maximum frequency is limited by the maximum APB1 frequency and cannot exceed 50 MHz (peripheral intrinsic maximum limit).
  0b000000: Not allowed 0b000001: Not allowed 0b000010: 2 MHz ... 0b110010: 50 MHz Higher than 0b100100: Not allowed
: I2C Own address register 1 (I2C_OAR1)
: I2C Data register (I2C_DR)
: I2C Status register 1 (I2C_SR1): and reg 2: see
: I2C Clock control register (I2C_CCR) : Bits 11:0 CCR[11:0]: Clock control register in Fm/Sm mode (Master mode)
  : get these right
: I2C TRISE register (I2C_TRISE) : set up values
: I2C FLTR register (I2C_FLTR): analog filter on by default, digital filter off by default

*/

#endif /* I2C_DRIVER_ */
