/**
 * @file pca9534.h
 * @brief PCA9534 I2C GPIO expander interface for XM125 satellite control.
 *
 * Provides XM125-specific control plane setters/getters using PCA9534 I/O expander.
 * All functions are non-blocking and integrate with state machines.
 *
 * @see PCA9534_I2C_Interface.md for device protocol
 * @see Hardware.md for system wiring
 * @see xm125_async.c for state machine integration
 */

#ifndef PCA9534_H
#define PCA9534_H

#include <stdint.h>
#include <stdbool.h>
#include "vmt_i2c_async.h"

/* ===== Register Addresses ===== */

/** PCA9534 register addresses per PCA9534_I2C_Interface.md Table "Register Map" */
#define PCA9534_REG_INPUT    (0x00u) ///< Input Port: Read-only, reflects pin logic levels
#define PCA9534_REG_OUTPUT   (0x01u) ///< Output Port: R/W, drives output pins (default 0xFF)
#define PCA9534_REG_POLARITY (0x02u) ///< Polarity Inversion: R/W, inverts input data (default 0x00)
#define PCA9534_REG_CONFIG   (0x03u) ///< Configuration: R/W, 1=input 0=output (default 0xFF)

/* ===== Pin Bit Masks ===== */

/** Generic pin bit mask generator */
#define PCA9534_PIN(n) ((uint8_t)(1u << (n))) ///< Bit mask for pin n (0..7)

/** Individual pin masks */
#define PCA9534_PIN0 PCA9534_PIN(0) ///< Pin 0 bit mask
#define PCA9534_PIN1 PCA9534_PIN(1) ///< Pin 1 bit mask
#define PCA9534_PIN2 PCA9534_PIN(2) ///< Pin 2 bit mask
#define PCA9534_PIN3 PCA9534_PIN(3) ///< Pin 3 bit mask
#define PCA9534_PIN4 PCA9534_PIN(4) ///< Pin 4 bit mask
#define PCA9534_PIN5 PCA9534_PIN(5) ///< Pin 5 bit mask
#define PCA9534_PIN6 PCA9534_PIN(6) ///< Pin 6 bit mask
#define PCA9534_PIN7 PCA9534_PIN(7) ///< Pin 7 bit mask

/* ===== XM125 Control Pin Mapping ===== */

/**
 * XM125 control pins per Hardware.md "PCA9534 to XM125 Mapping (Authoritative GPIO Table)"
 */
#define PCA9534_PIN_WAKE_UP PCA9534_PIN5 ///< IO5 = WAKE_UP (active HIGH)
#define PCA9534_PIN_NRESET  PCA9534_PIN7 ///< IO7 = NRESET (active LOW)
#define PCA9534_PIN_MCU_INT PCA9534_PIN6 ///< IO6 = MCU_INT (active HIGH, input)

/* ===== XM125 Control Constants ===== */

/* Configuration Register value for XM125 control.
 * IO7 and IO5 = outputs (0), IO6 = input (1), others inputs = 1
 * Binary: 0b01011111 = 0x5F
 */
#define PCA_CONFIG_XM125_DEFAULT 0x5Fu

/**
 * Output port values for XM125 state control.
 * Only bits 0-1 are significant.
 */
#define PCA_OUTPUT_RUN          0xA0u ///< WAKE_UP=1 (bit5), NRESET=1 (bit7)
#define PCA_OUTPUT_SLEEP        0x80u ///< WAKE_UP=0, NRESET=1 (bit7)
#define PCA_OUTPUT_RESET_ASSERT 0x00u ///< WAKE_UP=0, NRESET=0
/**
 * @brief Extract MCU_INT state from PCA9534 Input Port byte.
 * @retval true  MCU_INT is HIGH (XM125 ready)
 * @retval false MCU_INT is LOW (XM125 busy/asleep)
 */
#define PCA9534_MCU_INT_IS_HIGH(input_byte) (((input_byte) & PCA9534_PIN_MCU_INT) != 0)

/**
 * @brief Extract MCU_INT state as boolean.
 * @retval 1 MCU_INT is HIGH
 * @retval 0 MCU_INT is LOW
 */
#define PCA9534_GET_MCU_INT(input_byte) (((input_byte) & PCA9534_PIN_MCU_INT) ? 1u : 0u)

/**
 * @brief Validate PCA9534 7-bit I2C address.
 * @retval true  address in [0x20..0x27]
 * @retval false otherwise
 */
#define PCA9534_ADDR_VALID(addr7) ((addr7) >= 0x20u && (addr7) <= 0x27u)

/* ===== XM125 Control Setters/Getters ===== */

/** @name XM125 control API
 *  @{
 */

/**
 * @brief Initialize PCA9534 for XM125 control.
 * @param bank  I2C bank identifier.
 * @param addr7 7-bit address [0x20-0x27].
 * @retval HAL_OK    Initiated
 * @retval HAL_BUSY  Bank busy
 * @retval HAL_ERROR Invalid address
 * @note Non-blocking; check completion with vmt_i2c_async_is_idle().
 */
HAL_StatusTypeDef pca9534_init_for_xm125(vmt_i2c_bank_t bank, uint8_t addr7);

/**
 * @brief Set XM125 to RUN mode (WAKE_UP=1, NRESET=1).
 * @param bank  I2C bank identifier.
 * @param addr7 7-bit address [0x20-0x27].
 * @retval HAL_OK    Initiated
 * @retval HAL_BUSY  Bank busy
 * @retval HAL_ERROR Invalid address
 * @note Non-blocking; check completion with vmt_i2c_async_is_idle().
 */
HAL_StatusTypeDef pca9534_xm125_run(vmt_i2c_bank_t bank, uint8_t addr7);

/**
 * @brief Set XM125 to SLEEP mode (WAKE_UP=0, NRESET=1).
 * @param bank  I2C bank identifier.
 * @param addr7 7-bit address [0x20-0x27].
 * @retval HAL_OK    Initiated
 * @retval HAL_BUSY  Bank busy
 * @retval HAL_ERROR Invalid address
 * @note Non-blocking; check completion with vmt_i2c_async_is_idle().
 */
HAL_StatusTypeDef pca9534_xm125_sleep(vmt_i2c_bank_t bank, uint8_t addr7);

/**
 * @brief Assert XM125 hardware reset (WAKE_UP=0, NRESET=0).
 * @param bank  I2C bank identifier.
 * @param addr7 7-bit address [0x20-0x27].
 * @retval HAL_OK    Initiated
 * @retval HAL_BUSY  Bank busy
 * @retval HAL_ERROR Invalid address
 * @note Non-blocking; check completion with vmt_i2c_async_is_idle().
 */
HAL_StatusTypeDef pca9534_xm125_reset_assert(vmt_i2c_bank_t bank, uint8_t addr7);

/**
 * @brief Release XM125 from hardware reset (WAKE_UP=1, NRESET=1).
 * @param bank  I2C bank identifier.
 * @param addr7 7-bit address [0x20-0x27].
 * @retval HAL_OK    Initiated
 * @retval HAL_BUSY  Bank busy
 * @retval HAL_ERROR Invalid address
 * @note Non-blocking; check completion with vmt_i2c_async_is_idle().
 */
HAL_StatusTypeDef pca9534_xm125_reset_release(vmt_i2c_bank_t bank, uint8_t addr7);

/**
 * @brief Read PCA9534 Input Port to get MCU_INT state.
 * @param bank       I2C bank identifier.
 * @param addr7      7-bit address [0x20-0x27].
 * @param input_byte [out] Pointer to store 8-bit Input Port value.
 * @retval HAL_OK    Initiated
 * @retval HAL_BUSY  Bank busy
 * @retval HAL_ERROR Invalid params
 * @note Non-blocking; check completion with vmt_i2c_async_is_idle().
 */
HAL_StatusTypeDef pca9534_read_mcu_int(vmt_i2c_bank_t bank, uint8_t addr7, uint8_t *input_byte);

/**
 * @brief Check if XM125 is ready for I2C transactions.
 * @param input_byte Value read from PCA9534 Input Port register.
 * @retval true  MCU_INT is HIGH (XM125 ready)
 * @retval false MCU_INT is LOW (busy/asleep)
 */
static inline bool pca9534_is_xm125_ready(uint8_t input_byte)
{
    return PCA9534_MCU_INT_IS_HIGH(input_byte);
}

// Add function prototypes for XM125 helpers:
HAL_StatusTypeDef pca9534_xm125_wake(vmt_i2c_bank_t bank, uint8_t addr7);

/** @} */

#endif /* PCA9534_H */
