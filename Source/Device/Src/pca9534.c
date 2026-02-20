/**
 * @file pca9534.c
 * @brief XM125 control plane setters/getters using PCA9534 I/O expander.
 *
 * Implements XM125-specific control functions that wrap vmt_i2c_async primitives.
 * All functions use constants from pca9534.h and perform parameter validation.
 *
 * @see pca9534.h for API documentation
 */

#include "pca9534.h"
#include "vmt_i2c_async.h"

/**
 * @copydoc pca9534_init_for_xm125
 */
HAL_StatusTypeDef pca9534_init_for_xm125(vmt_i2c_bank_t bank, uint8_t addr7)
{
    if (!PCA9534_ADDR_VALID(addr7))
    {
        return HAL_ERROR;
    }
    return pca9534_write_reg_async(bank, addr7, PCA9534_REG_CONFIG, PCA_CONFIG_XM125_DEFAULT);
}

/**
 * @copydoc pca9534_xm125_run
 */
HAL_StatusTypeDef pca9534_xm125_run(vmt_i2c_bank_t bank, uint8_t addr7)
{
    if (!PCA9534_ADDR_VALID(addr7))
    {
        return HAL_ERROR;
    }
    return pca9534_write_reg_async(bank, addr7, PCA9534_REG_OUTPUT, PCA_OUTPUT_RUN);
}

/**
 * @copydoc pca9534_xm125_sleep
 */
HAL_StatusTypeDef pca9534_xm125_sleep(vmt_i2c_bank_t bank, uint8_t addr7)
{
    if (!PCA9534_ADDR_VALID(addr7))
    {
        return HAL_ERROR;
    }
    return pca9534_write_outputs_masked_async(bank, addr7, PCA9534_PIN_WAKE_UP, 0);
}

/**
 * @copydoc pca9534_xm125_reset_assert
 */
HAL_StatusTypeDef pca9534_xm125_reset_assert(vmt_i2c_bank_t bank, uint8_t addr7)
{
    if (!PCA9534_ADDR_VALID(addr7))
    {
        return HAL_ERROR;
    }
    return pca9534_write_outputs_masked_async(bank, addr7, PCA9534_PIN_NRESET, 0);
}

/**
 * @copydoc pca9534_xm125_reset_release
 */
HAL_StatusTypeDef pca9534_xm125_reset_release(vmt_i2c_bank_t bank, uint8_t addr7)
{
    if (!PCA9534_ADDR_VALID(addr7))
    {
        return HAL_ERROR;
    }
    return pca9534_write_outputs_masked_async(bank, addr7, PCA9534_PIN_NRESET, PCA9534_PIN_NRESET);
}

/**
 * @copydoc pca9534_read_mcu_int
 */
HAL_StatusTypeDef pca9534_read_mcu_int(vmt_i2c_bank_t bank, uint8_t addr7, uint8_t *input_byte)
{
    if (!PCA9534_ADDR_VALID(addr7) || input_byte == NULL)
    {
        return HAL_ERROR;
    }
    return pca9534_read_input_async(bank, addr7, input_byte);
}

/**
 * @copydoc pca9534_xm125_wake
 */
HAL_StatusTypeDef pca9534_xm125_wake(vmt_i2c_bank_t bank, uint8_t addr7)
{
    // Set WAKE_UP HIGH (bit 0)
    return pca9534_write_outputs_masked_async(bank, addr7, PCA9534_PIN_WAKE_UP, PCA9534_PIN_WAKE_UP);
}
