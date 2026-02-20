/**
 * @file vmt_i2c_async.h
 * @brief Asynchronous I2C helper API for STM32F7 (HAL).
 *
 * Provides non-blocking memory and combined transactions, basic state tracking,
 * and convenience helpers for PCA9534 and XM125 peripherals.
 *
 * @note All API functions are non-blocking and require periodic polling via
 *       vmt_i2c_async_process(). Only one transaction per bank at a time.
 */

#ifndef VMT_I2C_ASYNC_H
#define VMT_I2C_ASYNC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "stm32f7xx_hal.h"

/**
 * @def VMT_I2C_ASYNC_TIMEOUT_MS
 * @brief Default timeout in milliseconds for async helpers if not overridden by caller.
 * @details Defines the maximum time a transaction may take before being aborted by supervision.
 */
#ifndef VMT_I2C_ASYNC_TIMEOUT_MS
#define VMT_I2C_ASYNC_TIMEOUT_MS 50u
#endif

/**
 * @name Satellite addressing (SPEC154)
 * @brief Helpers to derive 7-bit I2C addresses from satellite index [1..3].
 * @details Per Hardware.md "Addressing Contract (Authoritative)"
 *          XM125: 0x51-0x53 per bus, PCA9534: 0x21-0x23 per bus
 * @{
 */
/** Compute XM125 7-bit address for satellite index (1->0x51..3->0x53). */
#define VMT_XM125_ADDR_FOR_SAT(idx) ((uint8_t)(0x50u + (uint8_t)(idx)))
/** Compute PCA9534 7-bit address for satellite index (1->0x21..3->0x23). */
#define VMT_PCA9534_ADDR_FOR_SAT(idx) ((uint8_t)(0x20u + (uint8_t)(idx)))
/** Validate satellite ID is in range [1..6]. */
#define VMT_SAT_ID_VALID(id) ((id) >= 1u && (id) <= 6u)
/** @} */

/**
 * @brief I2C peripheral bank abstraction.
 */
typedef enum
{
    VMT_I2C_BANK1 = 1, /**< Use I2C1 peripheral. */
    VMT_I2C_BANK2 = 2, /**< Use I2C2 peripheral. */
} vmt_i2c_bank_t;

/**
 * @brief High-level async state of a bank.
 */
typedef enum
{
    VMT_I2C_ASYNC_IDLE = 0,  /**< No transaction pending. */
    VMT_I2C_ASYNC_BUSY_TX,   /**< Transmit in progress. */
    VMT_I2C_ASYNC_BUSY_STOP, /**< Waiting for STOP condition (XM125 protocol). */
    VMT_I2C_ASYNC_BUSY_RX,   /**< Receive in progress. */
    VMT_I2C_ASYNC_ERROR,     /**< Error latched; see last_status/last_error. */
} vmt_i2c_async_state_t;

/**
 * @brief Currently scheduled operation kind.
 */
typedef enum
{
    VMT_I2C_OP_NONE = 0,        /**< No operation. */
    VMT_I2C_OP_MEM_WRITE,       /**< Memory write (with register). */
    VMT_I2C_OP_MEM_READ,        /**< Memory read (with register). */
    VMT_I2C_OP_WRITE_THEN_READ, /**< STOP-separated write then read. */
} vmt_i2c_op_t;

/**
 * @brief Runtime context for an async I2C bank.
 */
typedef struct
{
    vmt_i2c_async_state_t state;               /**< Current state. */
    vmt_i2c_op_t          op;                  /**< Operation being executed. */
    vmt_i2c_bank_t        bank;                /**< Bank identifier. */
    uint8_t               addr7;               /**< 7-bit target address. */
    uint16_t              reg;                 /**< Register for memory ops. */
    uint16_t              reg_size;            /**< Register size in bytes (1 or 2). */
    uint8_t              *buf;                 /**< Data buffer for memory ops. */
    uint16_t              len;                 /**< Data length for memory ops. */
    const uint8_t        *tx;                  /**< TX buffer for combined op. */
    uint16_t              tx_len;              /**< TX length for combined op. */
    uint8_t              *rx;                  /**< RX buffer for combined op. */
    uint16_t              rx_len;              /**< RX length for combined op. */
    uint8_t               scratch_tx[2];       /**< Small scratch for 1/2-byte regs. */
    uint32_t              start_tick;          /**< Start tick for timeout bookkeeping. */
    uint32_t              stop_wait_start;     /**< Tick when STOP wait began. */
    HAL_StatusTypeDef     last_status;         /**< Last HAL status. */
    uint32_t              last_error;          /**< Last HAL error bitfield. */
    uint8_t               current_sat_id;      /**< Satellite ID for error reporting (1-6). */
    uint8_t               pca_output_shadow;   /**< Shadow register for PCA9534 outputs. */
    bool                  rmw_in_progress;     /**< Read-modify-write sequence active. */
    uint8_t               rmw_target_reg;      /**< Target register for RMW (CONFIG or OUTPUT). */
    uint8_t               rmw_mask;            /**< Mask for current RMW operation. */
    uint8_t               rmw_values;          /**< Values for current RMW operation. */
    uint8_t               rmw_read_buf;        /**< Temporary buffer for RMW read phase. */
    uint8_t               internal_tx_buf[96]; /**< DMA-safe TX buffer (XM125 config = 84 bytes). */
} vmt_i2c_async_ctx_t;

/* ===== Public API ===== */

/**
 * @brief Initialize the async I2C module and clear internal state.
 */
void vmt_i2c_async_init(void);

/**
 * @brief Returns true if the given bank has no active transfer.
 * @param bank I2C bank to query.
 * @retval true  No active transfer.
 * @retval false Transfer in progress or invalid bank.
 */
bool vmt_i2c_async_is_idle(vmt_i2c_bank_t bank);

/**
 * @brief Progress engine to be called from the main loop or scheduler.
 * @details Supervises timeouts and aborts stalled transfers.
 */
void vmt_i2c_async_process(void);

/**
 * @brief Get current async state for a bank.
 * @param bank I2C bank to query.
 * @return Current async state.
 */
vmt_i2c_async_state_t vmt_i2c_async_get_state(vmt_i2c_bank_t bank);

/**
 * @brief Get last HAL status observed for a bank.
 * @param bank I2C bank to query.
 * @return Last HAL status.
 */
HAL_StatusTypeDef vmt_i2c_async_get_last_status(vmt_i2c_bank_t bank);

/**
 * @brief Get last HAL error code observed for a bank.
 * @param bank I2C bank to query.
 * @return Last HAL error code.
 */
uint32_t vmt_i2c_async_get_last_error(vmt_i2c_bank_t bank);

/**
 * @brief Submit an asynchronous memory read transaction.
 * @param bank      [in] Target I2C bank.
 * @param addr7     [in] 7-bit slave address.
 * @param reg       [in] Register value (1 or 2 bytes depending on reg_size).
 * @param reg_size  [in] Register size in bytes (1 or 2).
 * @param buf       [out] Destination buffer.
 * @param len       [in] Number of bytes to read.
 * @param timeout_ms[in] Overall timeout for the transaction.
 * @return HAL status at submit time (final result reflected via getters/callbacks).
 */
HAL_StatusTypeDef vmt_i2c_async_mem_read(vmt_i2c_bank_t bank, uint8_t addr7, uint16_t reg, uint16_t reg_size, uint8_t *buf, uint16_t len, uint32_t timeout_ms);

/**
 * @brief Submit an asynchronous memory write transaction.
 * @param bank      [in] Target I2C bank.
 * @param addr7     [in] 7-bit slave address.
 * @param reg       [in] Register value (1 or 2 bytes depending on reg_size).
 * @param reg_size  [in] Register size in bytes (1 or 2).
 * @param buf       [in] Source buffer.
 * @param len       [in] Number of bytes to write.
 * @param timeout_ms[in] Overall timeout for the transaction.
 * @return HAL status at submit time.
 */
HAL_StatusTypeDef vmt_i2c_async_mem_write(vmt_i2c_bank_t bank, uint8_t addr7, uint16_t reg, uint16_t reg_size, const uint8_t *buf, uint16_t len, uint32_t timeout_ms);

/**
 * @brief Submit a STOP-separated write-then-read (combined) transaction.
 * @param bank      [in] Target I2C bank.
 * @param addr7     [in] 7-bit slave address.
 * @param tx        [in] Pointer to write buffer.
 * @param tx_len    [in] Number of bytes to write.
 * @param rx        [out] Pointer to read buffer.
 * @param rx_len    [in] Number of bytes to read.
 * @param timeout_ms[in] Overall timeout for the transaction.
 * @return HAL status at submit time.
 */
HAL_StatusTypeDef
vmt_i2c_async_write_then_read(vmt_i2c_bank_t bank, uint8_t addr7, const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len, uint32_t timeout_ms);

/* ===== PCA9534 GPIO Expander APIs ===== */

/**
 * @brief Read a PCA9534 register (1-byte register, 1-byte data).
 * @param bank  [in] I2C bank.
 * @param addr7 [in] PCA9534 7-bit address.
 * @param reg   [in] Register address.
 * @param val   [out] Out pointer for value.
 * @return HAL status at submit time.
 */
HAL_StatusTypeDef pca9534_read_reg_async(vmt_i2c_bank_t bank, uint8_t addr7, uint8_t reg, uint8_t *val);

/**
 * @brief Read PCA9534 Input Port register (convenience wrapper for MCU_INT polling).
 * @details Per Hardware.md "MCU_INT Semantics": MCU_INT HIGH = XM125 ready for I2C.
 * @param bank  [in] I2C bank.
 * @param addr7 [in] PCA9534 7-bit address.
 * @param val   [out] Input port byte (bit 6 = MCU_INT status).
 * @return HAL status at submit time.
 */
HAL_StatusTypeDef pca9534_read_input_async(vmt_i2c_bank_t bank, uint8_t addr7, uint8_t *val);

/**
 * @brief Write a PCA9534 register (1-byte register, 1-byte data).
 * @param bank  [in] I2C bank.
 * @param addr7 [in] PCA9534 7-bit address.
 * @param reg   [in] Register address.
 * @param val   [in] Value to write.
 * @return HAL status at submit time.
 */
HAL_StatusTypeDef pca9534_write_reg_async(vmt_i2c_bank_t bank, uint8_t addr7, uint8_t reg, uint8_t val);

/**
 * @brief Configure PCA9534 pin directions (masked, read-modify-write).
 * @details Per PCA9534_I2C_Interface.md Section 3: requires RMW to preserve non-target bits.
 * @param bank   [in] I2C bank.
 * @param addr7  [in] PCA9534 7-bit address.
 * @param mask   [in] Bit mask of pins to affect (1 = modify, 0 = preserve).
 * @param output [in] If true, set masked pins as outputs; otherwise inputs.
 * @return HAL status at submit time.
 * @warning Not atomic - caller must ensure no concurrent access to same device.
 */
HAL_StatusTypeDef pca9534_set_direction_async(vmt_i2c_bank_t bank, uint8_t addr7, uint8_t mask, bool output);

/**
 * @brief Write PCA9534 output values with mask (read-modify-write).
 * @details Uses shadow register for zero-latency modification.
 *          Per PCA9534_I2C_Interface.md: Output Port reads return latch value.
 * @param bank   [in] I2C bank.
 * @param addr7  [in] PCA9534 7-bit address.
 * @param mask   [in] Pins to update (1 = modify, 0 = preserve).
 * @param values [in] New values for masked bits.
 * @return HAL status at submit time.
 * @warning Not atomic - caller must ensure no concurrent access to same device.
 */
HAL_StatusTypeDef pca9534_write_outputs_masked_async(vmt_i2c_bank_t bank, uint8_t addr7, uint8_t mask, uint8_t values);

/* ===== HAL callback hooks (forwarded by HAL) ===== */

/**
 * @brief HAL completion callback for memory RX; forwarded to async engine.
 * @param hi2c [in] HAL I2C handle.
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief HAL completion callback for memory TX.
 * @param hi2c [in] HAL I2C handle.
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief HAL completion callback for master TX.
 * @param hi2c [in] HAL I2C handle.
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief HAL completion callback for master RX.
 * @param hi2c [in] HAL I2C handle.
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief HAL error callback.
 * @param hi2c [in] HAL I2C handle.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief HAL abort completion callback.
 * @param hi2c [in] HAL I2C handle.
 */
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);

/* ===== Satellite Addressing & Mapping Helpers ===== */

/**
 * @name Satellite mapping helpers (shared)
 * @brief Normalize logical satellite IDs (1..6) to bank and per-bank addresses.
 * @{
 */
static inline uint8_t vmt_sat_physical_index(uint8_t sat_id)
{
    return (uint8_t)(((sat_id - 1u) % 3u) + 1u); // 4->1, 5->2, 6->3
}

static inline vmt_i2c_bank_t vmt_sat_bank_for_id(uint8_t sat_id)
{
    return (sat_id >= 1u && sat_id <= 3u) ? VMT_I2C_BANK1 : (sat_id >= 4u && sat_id <= 6u) ? VMT_I2C_BANK2 : (vmt_i2c_bank_t)0;
}

static inline uint8_t vmt_sat_xm125_addr_for_id(uint8_t sat_id)
{
    return VMT_XM125_ADDR_FOR_SAT(vmt_sat_physical_index(sat_id));
}

static inline uint8_t vmt_sat_pca9534_addr_for_id(uint8_t sat_id)
{
    return VMT_PCA9534_ADDR_FOR_SAT(vmt_sat_physical_index(sat_id));
}
/** @} */

/* ===== XM125 Register Access APIs ===== */

/**
 * @brief Read 32-bit big-endian register from XM125 (reg16, data32).
 * @note Implemented using STOP-separated write-then-read per XM125_I2C_Distance_Detector_SPEC.md.
 * @param bank   [in] I2C bank.
 * @param addr7  [in] XM125 7-bit address.
 * @param reg_be [in] Register address in big-endian order.
 * @param rx4    [out] Pointer to 4-byte buffer for result.
 * @return HAL status at submit time.
 */
HAL_StatusTypeDef xm125_reg_read_u32_async(vmt_i2c_bank_t bank, uint8_t addr7, uint16_t reg_be, uint8_t *rx4);

/**
 * @brief Read consecutive XM125 registers in a single burst (optimized for measurement data).
 * @details Per XM125 spec Section 2.3.3: DISTANCE_RESULT + peaks = 28 bytes.
 * @param bank      [in] I2C bank.
 * @param addr7     [in] XM125 7-bit address.
 * @param start_reg_be [in] Starting register address (big-endian).
 * @param rx        [out] Buffer for (count * 4) bytes.
 * @param count     [in] Number of consecutive 32-bit registers to read.
 * @return HAL status at submit time.
 */
HAL_StatusTypeDef xm125_reg_read_burst_async(vmt_i2c_bank_t bank, uint8_t addr7, uint16_t start_reg_be, uint8_t *rx, uint8_t count);

/**
 * @brief Write 32-bit big-endian register to XM125 (reg16, data32).
 * @param bank   [in] I2C bank.
 * @param addr7  [in] XM125 7-bit address.
 * @param reg_be [in] Register address in big-endian order.
 * @param tx4    [in] Pointer to 4-byte data buffer.
 * @return HAL status at submit time.
 */
HAL_StatusTypeDef xm125_reg_write_u32_async(vmt_i2c_bank_t bank, uint8_t addr7, uint16_t reg_be, const uint8_t *tx4);

/**
 * @brief Recover I2C async engine from error state
 * @details Resets a bank from ERROR state to IDLE, aborting any pending transactions.
 * @param bank Bank to recover (VMT_I2C_BANK1 or VMT_I2C_BANK2)
 */
void vmt_i2c_async_recover_error(vmt_i2c_bank_t bank);

#endif /* VMT_I2C_ASYNC_H */
