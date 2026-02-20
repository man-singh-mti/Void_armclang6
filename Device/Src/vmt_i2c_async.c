/**
 * @file vmt_i2c_async.c
 * @brief Asynchronous I2C transaction engine for STM32F7xx HAL DMA.
 *
 * Implements non-blocking, stateful I2C memory and combined transactions
 * for multiple I2C peripherals, with progress supervision, error handling,
 * and device-specific helpers for PCA9534 and XM125 peripherals.
 *
 * Assumptions:
 * - Only one transaction per I2C bank at a time.
 * - All API calls are non-blocking; completion is signaled via HAL callbacks.
 * - vmt_i2c_async_process() must be called periodically to supervise timeouts.
 *
 * Side effects:
 * - Uses static context for each I2C bank.
 * - Aborts and resets transactions on timeout or error.
 *
 * See vmt_i2c_async.h for API documentation.
 */

#include "vmt_i2c_async.h"
#include "i2c.h"
#include "vmt_uart.h"
#include "pca9534.h"
#include "xm125_regmap.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* Static context for each I2C bank */
static vmt_i2c_async_ctx_t ctx1;
static vmt_i2c_async_ctx_t ctx2;

/* Debug names for logging */
static const char *bank_names[2] = { "BANK1", "BANK2" };

/* --- Static helpers --- */

/**
 * Print I2C configuration for debugging.
 * Called during initialization for both banks.
 */
static void i2c_print_cfg(void)
{
    uint32_t           pclk1    = HAL_RCC_GetPCLK1Freq();
    I2C_HandleTypeDef *buses[2] = { &hi2c1, &hi2c2 };
    const char        *names[2] = { "I2C1", "I2C2" };

    for (int i = 0; i < 2; i++)
    {
        uint32_t t     = buses[i]->Init.Timing;
        uint32_t presc = (t >> 28) & 0xF;
        uint32_t sclh  = (t >> 8) & 0xFF;
        uint32_t scll  = (t >> 0) & 0xFF;

        uint32_t denom  = (presc + 1U) * ((sclh + 1U) + (scll + 1U));
        uint32_t scl_hz = denom ? (pclk1 / denom) : 0;

        uint32_t pclk1_mhz = pclk1 / 1000000U;
        uint32_t scl_khz   = scl_hz / 1000U;
        debug_send("> i2c:bus,%s,pclk1,%luMHz,scl,%lukHz", names[i], pclk1_mhz, scl_khz);

        // Validate SCL is within I2C Fast Mode spec (100-400 kHz)
        if (scl_khz < 90 || scl_khz > 410)
        {
            debug_send("> i2c:WARNING,bus,%s,scl_out_of_spec,%lukHz", names[i], scl_khz);
        }
    }
}

/**
 * Get async context pointer for a given bank.
 * Returns NULL if bank is invalid.
 */
static vmt_i2c_async_ctx_t *get_ctx_for_bank(vmt_i2c_bank_t bank)
{
    switch (bank)
    {
    case VMT_I2C_BANK1:
        return &ctx1;
    case VMT_I2C_BANK2:
        return &ctx2;
    default:
        return NULL;
    }
}

/**
 * Map HAL I2C handle to bank identifier.
 * Returns 0 if not recognized.
 */
static vmt_i2c_bank_t bank_for_handle(I2C_HandleTypeDef *h)
{
    if (h == &hi2c1)
    {
        return VMT_I2C_BANK1;
    }
    if (h == &hi2c2)
    {
        return VMT_I2C_BANK2;
    }
    return (vmt_i2c_bank_t)0;
}

/**
 * Convert 7-bit address to HAL 8-bit address format.
 */
static inline uint16_t vmt_hal_addr(uint8_t addr7)
{
    return (uint16_t)((uint16_t)addr7 << 1);
}

/**
 * Obtain HAL I2C handle for a bank.
 * Returns NULL if bank is invalid.
 */
static I2C_HandleTypeDef *handle_for_bank(vmt_i2c_bank_t bank)
{
    switch (bank)
    {
    case VMT_I2C_BANK1:
        return &hi2c1;
    case VMT_I2C_BANK2:
        return &hi2c2;
    default:
        return NULL;
    }
}

/* --- Public API implementation --- */

void vmt_i2c_async_init(void)
{
    // Clear all context state
    memset(&ctx1, 0, sizeof(ctx1));
    memset(&ctx2, 0, sizeof(ctx2));

    ctx1.state             = VMT_I2C_ASYNC_IDLE;
    ctx1.op                = VMT_I2C_OP_NONE;
    ctx1.last_status       = HAL_OK;
    ctx1.last_error        = 0u;
    ctx1.pca_output_shadow = 0xFF; // PCA9534 power-on default per spec

    ctx2.state             = VMT_I2C_ASYNC_IDLE;
    ctx2.op                = VMT_I2C_OP_NONE;
    ctx2.last_status       = HAL_OK;
    ctx2.last_error        = 0u;
    ctx2.pca_output_shadow = 0xFF; // PCA9534 power-on default per spec

    i2c_print_cfg();
}

bool vmt_i2c_async_is_idle(vmt_i2c_bank_t bank)
{
    vmt_i2c_async_ctx_t *ctx = get_ctx_for_bank(bank);
    return (ctx != NULL) && (ctx->state == VMT_I2C_ASYNC_IDLE);
}

HAL_StatusTypeDef vmt_i2c_async_mem_read(vmt_i2c_bank_t bank, uint8_t addr7, uint16_t reg, uint16_t reg_size, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    vmt_i2c_async_ctx_t *ctx = get_ctx_for_bank(bank);
    if ((ctx == NULL) || (ctx->state != VMT_I2C_ASYNC_IDLE))
    {
        return HAL_BUSY;
    }
    if (!buf || (len == 0u))
    {
        return HAL_ERROR;
    }
    if (!((reg_size == 1u) || (reg_size == 2u)))
    {
        return HAL_ERROR;
    }

    // No DMA alignment check needed - I2C DMA configured with MemDataAlignment = BYTE
    // (see Core/Src/i2c.c), so byte-aligned buffers are valid

    ctx->bank        = bank;
    ctx->addr7       = addr7;
    ctx->reg         = reg;
    ctx->reg_size    = reg_size;
    ctx->buf         = buf;
    ctx->len         = len;
    ctx->op          = VMT_I2C_OP_MEM_READ;
    ctx->start_tick  = HAL_GetTick();
    ctx->last_status = HAL_OK;
    ctx->last_error  = 0u;
    ctx->state       = VMT_I2C_ASYNC_BUSY_RX;

    (void)timeout_ms;
    I2C_HandleTypeDef *h          = (bank == VMT_I2C_BANK1) ? &hi2c1 : &hi2c2;
    uint16_t           memaddsize = (reg_size == 1u) ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;

    // HAL_I2C_Mem_Read_DMA with I2C_MEMADD_SIZE_16BIT automatically sends MSB first
    // No manual byte swap needed - HAL handles wire format per I2C spec
    return HAL_I2C_Mem_Read_DMA(h, vmt_hal_addr(addr7), reg, memaddsize, buf, len);
}

HAL_StatusTypeDef vmt_i2c_async_mem_write(vmt_i2c_bank_t bank, uint8_t addr7, uint16_t reg, uint16_t reg_size, const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    vmt_i2c_async_ctx_t *ctx = get_ctx_for_bank(bank);
    if ((ctx == NULL) || (ctx->state != VMT_I2C_ASYNC_IDLE))
    {
        return HAL_BUSY;
    }
    if (!buf || (len == 0u))
    {
        return HAL_ERROR;
    }
    if (!((reg_size == 1u) || (reg_size == 2u)))
    {
        return HAL_ERROR;
    }

    // No DMA alignment check needed - I2C DMA configured with MemDataAlignment = BYTE
    // (see Core/Src/i2c.c), so byte-aligned buffers are valid

    ctx->bank     = bank;
    ctx->addr7    = addr7;
    ctx->reg      = reg;
    ctx->reg_size = reg_size;
    // Copy into persistent buffer when small enough
    if (len <= sizeof(ctx->internal_tx_buf))
    {
        memcpy(ctx->internal_tx_buf, buf, len);
        ctx->buf = ctx->internal_tx_buf;
    }
    else
    {
        ctx->buf = (uint8_t *)buf;
    }
    ctx->len         = len;
    ctx->op          = VMT_I2C_OP_MEM_WRITE;
    ctx->start_tick  = HAL_GetTick();
    ctx->last_status = HAL_OK;
    ctx->last_error  = 0u;
    ctx->state       = VMT_I2C_ASYNC_BUSY_TX;

    (void)timeout_ms;
    I2C_HandleTypeDef *h          = (bank == VMT_I2C_BANK1) ? &hi2c1 : &hi2c2;
    uint16_t           memaddsize = (reg_size == 1u) ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;

    // HAL handles wire format - 16-bit addresses transmitted MSB first automatically
    return HAL_I2C_Mem_Write_DMA(h, vmt_hal_addr(addr7), reg, memaddsize, ctx->buf, len);
}

HAL_StatusTypeDef
vmt_i2c_async_write_then_read(vmt_i2c_bank_t bank, uint8_t addr7, const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len, uint32_t timeout_ms)
{
    vmt_i2c_async_ctx_t *ctx = get_ctx_for_bank(bank);
    if ((ctx == NULL) || (ctx->state != VMT_I2C_ASYNC_IDLE))
    {
        return HAL_BUSY;
    }
    if (!tx || !rx || (tx_len == 0u) || (rx_len == 0u))
    {
        return HAL_ERROR;
    }

    // No DMA alignment check needed - I2C DMA configured with MemDataAlignment = BYTE
    // (see Core/Src/i2c.c), so byte-aligned buffers are valid

    ctx->bank  = bank;
    ctx->addr7 = addr7;
    // Persist TX for DMA safety
    if (tx_len <= sizeof(ctx->internal_tx_buf))
    {
        memcpy(ctx->internal_tx_buf, tx, tx_len);
        ctx->tx = ctx->internal_tx_buf;
    }
    else
    {
        ctx->tx = tx;
    }
    ctx->tx_len      = tx_len;
    ctx->rx          = rx;
    ctx->rx_len      = rx_len;
    ctx->op          = VMT_I2C_OP_WRITE_THEN_READ;
    ctx->start_tick  = HAL_GetTick();
    ctx->last_status = HAL_OK;
    ctx->last_error  = 0u;
    ctx->state       = VMT_I2C_ASYNC_BUSY_TX;

    (void)timeout_ms;
    I2C_HandleTypeDef *h = (bank == VMT_I2C_BANK1) ? &hi2c1 : &hi2c2;
    return HAL_I2C_Master_Transmit_DMA(h, (uint16_t)((uint16_t)addr7 << 1), (uint8_t *)ctx->tx, tx_len);
}

vmt_i2c_async_state_t vmt_i2c_async_get_state(vmt_i2c_bank_t bank)
{
    vmt_i2c_async_ctx_t *ctx = get_ctx_for_bank(bank);
    return ctx ? ctx->state : VMT_I2C_ASYNC_ERROR;
}

HAL_StatusTypeDef vmt_i2c_async_get_last_status(vmt_i2c_bank_t bank)
{
    vmt_i2c_async_ctx_t *ctx = get_ctx_for_bank(bank);
    return ctx ? ctx->last_status : HAL_ERROR;
}

uint32_t vmt_i2c_async_get_last_error(vmt_i2c_bank_t bank)
{
    vmt_i2c_async_ctx_t *ctx = get_ctx_for_bank(bank);
    return ctx ? ctx->last_error : 0u;
}

void vmt_i2c_async_process(void)
{
    vmt_i2c_async_ctx_t *banks[2] = { &ctx1, &ctx2 };
    for (size_t i = 0; i < 2; ++i)
    {
        vmt_i2c_async_ctx_t *ctx = banks[i];
        if (!ctx)
        {
            continue;
        }

        // Handle STOP wait and initiate read phase (XM125 protocol requires STOP between W/R)
        if (ctx->state == VMT_I2C_ASYNC_BUSY_STOP)
        {
            I2C_HandleTypeDef *h = handle_for_bank(ctx->bank);
            if (h)
            {
                // Check if STOP condition completed (wait for bus idle)
                // Per STM32 RM: STOPF is cleared by software, BUSY indicates ongoing operation
                if ((h->Instance->ISR & I2C_ISR_BUSY) == 0)
                {
                    // Bus is idle - safe to start read phase with fresh START condition
                    ctx->state               = VMT_I2C_ASYNC_BUSY_RX;
                    HAL_StatusTypeDef status = HAL_I2C_Master_Receive_DMA(h, vmt_hal_addr(ctx->addr7), ctx->rx, ctx->rx_len);
                    if (status != HAL_OK)
                    {
                        ctx->last_status = status;
                        ctx->last_error  = h->ErrorCode;
                        ctx->state       = VMT_I2C_ASYNC_ERROR;
                        debug_send("> i2c:ERROR,%s,read_start_failed,status,%d", bank_names[i], status);
                    }
                }
                else
                {
                    // Check timeout - STOP should complete within 1ms at 400kHz
                    uint32_t stop_elapsed = HAL_GetTick() - ctx->stop_wait_start;
                    if (stop_elapsed > 5u) // 5ms max for bus idle (generous for 400kHz)
                    {
                        ctx->last_status = HAL_ERROR;
                        ctx->last_error  = HAL_I2C_ERROR_TIMEOUT;
                        ctx->state       = VMT_I2C_ASYNC_ERROR;
                        debug_send("> i2c:ERROR,%s,stop_timeout,isr,0x%08lX", bank_names[i], h->Instance->ISR);
                    }
                }
            }
        }

        // Handle transaction timeout
        if (ctx->state == VMT_I2C_ASYNC_BUSY_TX || ctx->state == VMT_I2C_ASYNC_BUSY_RX)
        {
            uint32_t elapsed = HAL_GetTick() - ctx->start_tick;
            if (elapsed > VMT_I2C_ASYNC_TIMEOUT_MS)
            {
                I2C_HandleTypeDef *h = handle_for_bank(ctx->bank);
                if (h)
                {
                    (void)HAL_I2C_Master_Abort_IT(h, vmt_hal_addr(ctx->addr7));
                }
                ctx->last_status = HAL_ERROR;
                ctx->last_error  = HAL_I2C_ERROR_TIMEOUT;
                ctx->state       = VMT_I2C_ASYNC_ERROR;
                debug_send("> i2c:ERROR,%s,transaction_timeout,addr,0x%02X", bank_names[i], ctx->addr7);
            }
        }
    }
}

/* --- Device helper APIs --- */

HAL_StatusTypeDef pca9534_read_reg_async(vmt_i2c_bank_t bank, uint8_t addr7, uint8_t reg, uint8_t *val)
{
    if (!val)
    {
        return HAL_ERROR;
    }
    return vmt_i2c_async_mem_read(bank, addr7, reg, 1u, val, 1u, VMT_I2C_ASYNC_TIMEOUT_MS);
}

HAL_StatusTypeDef pca9534_write_reg_async(vmt_i2c_bank_t bank, uint8_t addr7, uint8_t reg, uint8_t val)
{
    return vmt_i2c_async_mem_write(bank, addr7, reg, 1u, &val, 1u, VMT_I2C_ASYNC_TIMEOUT_MS);
}

HAL_StatusTypeDef pca9534_read_input_async(vmt_i2c_bank_t bank, uint8_t addr7, uint8_t *val)
{
    if (!val)
    {
        return HAL_ERROR;
    }
    return vmt_i2c_async_mem_read(bank, addr7, PCA9534_REG_INPUT, 1u, val, 1u, VMT_I2C_ASYNC_TIMEOUT_MS);
}

HAL_StatusTypeDef pca9534_set_direction_async(vmt_i2c_bank_t bank, uint8_t addr7, uint8_t mask, bool output)
{
    vmt_i2c_async_ctx_t *ctx = get_ctx_for_bank(bank);
    if (!ctx || ctx->state != VMT_I2C_ASYNC_IDLE)
    {
        return HAL_BUSY;
    }

    // PCA9534 Configuration Register: 0 = output, 1 = input
    // Start RMW sequence: read current config
    ctx->rmw_in_progress = true;
    ctx->rmw_target_reg  = PCA9534_REG_CONFIG;
    ctx->rmw_mask        = mask;
    ctx->rmw_values      = output ? 0x00 : 0xFF; // Set masked bits to 0 (output) or 1 (input)

    return pca9534_read_reg_async(bank, addr7, PCA9534_REG_CONFIG, &ctx->rmw_read_buf);
}

HAL_StatusTypeDef pca9534_write_outputs_masked_async(vmt_i2c_bank_t bank, uint8_t addr7, uint8_t mask, uint8_t values)
{
    vmt_i2c_async_ctx_t *ctx = get_ctx_for_bank(bank);
    if (!ctx || ctx->state != VMT_I2C_ASYNC_IDLE)
    {
        return HAL_BUSY;
    }

    // Use shadow register to avoid read latency
    // Per PCA9534_I2C_Interface.md: Output Port reads return latch value
    uint8_t new_val        = (ctx->pca_output_shadow & ~mask) | (values & mask);
    ctx->pca_output_shadow = new_val;

    return pca9534_write_reg_async(bank, addr7, PCA9534_REG_OUTPUT, new_val);
}

HAL_StatusTypeDef xm125_reg_read_u32_async(vmt_i2c_bank_t bank, uint8_t addr7, uint16_t reg_be, uint8_t *rx4)
{
    if (!rx4)
    {
        return HAL_ERROR;
    }
    uint8_t reg2[2] = { (uint8_t)(reg_be >> 8), (uint8_t)(reg_be & 0xFF) };
    return vmt_i2c_async_write_then_read(bank, addr7, reg2, 2u, rx4, 4u, VMT_I2C_ASYNC_TIMEOUT_MS);
}

HAL_StatusTypeDef xm125_reg_write_u32_async(vmt_i2c_bank_t bank, uint8_t addr7, uint16_t reg_be, const uint8_t *tx4)
{
    if (!tx4)
    {
        return HAL_ERROR;
    }
    return vmt_i2c_async_mem_write(bank, addr7, reg_be, 2u, tx4, 4u, VMT_I2C_ASYNC_TIMEOUT_MS);
}

HAL_StatusTypeDef xm125_reg_read_burst_async(vmt_i2c_bank_t bank, uint8_t addr7, uint16_t start_reg_be, uint8_t *rx, uint8_t count)
{
    if (!rx || count == 0 || count > 32)
    {
        return HAL_ERROR;
    }

    uint16_t byte_count = (uint16_t)count * 4u;
    uint8_t  reg2[2]    = { (uint8_t)(start_reg_be >> 8), (uint8_t)(start_reg_be & 0xFF) };

    return vmt_i2c_async_write_then_read(bank, addr7, reg2, 2u, rx, byte_count, VMT_I2C_ASYNC_TIMEOUT_MS);
}

/* --- HAL callback hooks --- */

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    vmt_i2c_bank_t       bank = bank_for_handle(hi2c);
    vmt_i2c_async_ctx_t *ctx  = get_ctx_for_bank(bank);
    if (ctx && ctx->op == VMT_I2C_OP_WRITE_THEN_READ && ctx->state == VMT_I2C_ASYNC_BUSY_TX)
    {
        // Per XM125_I2C_Distance_Detector_SPEC.md Section 3.2.2:
        // MUST have STOP condition between write and read phases
        // HAL_I2C_Master_Transmit_DMA followed by Receive_DMA generates repeated START
        // which violates XM125 protocol - force STOP here

        // Generate STOP condition
        SET_BIT(hi2c->Instance->CR2, I2C_CR2_STOP);

        // Transition to STOP wait state
        ctx->state           = VMT_I2C_ASYNC_BUSY_STOP;
        ctx->stop_wait_start = HAL_GetTick();

        // STOP completion will be handled in process loop, then start read phase
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    vmt_i2c_bank_t       bank = bank_for_handle(hi2c);
    vmt_i2c_async_ctx_t *ctx  = get_ctx_for_bank(bank);
    if (ctx && ctx->op == VMT_I2C_OP_WRITE_THEN_READ && ctx->state == VMT_I2C_ASYNC_BUSY_RX)
    {
        ctx->last_status = HAL_OK;
        ctx->state       = VMT_I2C_ASYNC_IDLE;
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    vmt_i2c_bank_t       bank = bank_for_handle(hi2c);
    vmt_i2c_async_ctx_t *ctx  = get_ctx_for_bank(bank);
    if (ctx != NULL && ctx->state == VMT_I2C_ASYNC_BUSY_RX)
    {
        // Check if this completes an RMW read phase (for PCA9534 masked operations)
        if (ctx->rmw_in_progress)
        {
            // Modify: apply mask and values
            uint8_t new_val = (ctx->rmw_read_buf & ~ctx->rmw_mask) | (ctx->rmw_values & ctx->rmw_mask);

            // Update shadow if modifying OUTPUT register
            if (ctx->rmw_target_reg == PCA9534_REG_OUTPUT)
            {
                ctx->pca_output_shadow = new_val;
            }

            // Write back (this will complete in MemTxCpltCallback)
            ctx->rmw_in_progress     = false; // Clear before write to avoid recursion
            HAL_StatusTypeDef status = pca9534_write_reg_async(bank, ctx->addr7, ctx->rmw_target_reg, new_val);

            if (status != HAL_OK)
            {
                ctx->last_status = status;
                ctx->state       = VMT_I2C_ASYNC_ERROR;
            }
            // Note: state will be set to IDLE by subsequent MemTxCpltCallback
        }
        else
        {
            // Normal read completion
            ctx->last_status = HAL_OK;
            ctx->state       = VMT_I2C_ASYNC_IDLE;
        }
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    vmt_i2c_bank_t       bank = bank_for_handle(hi2c);
    vmt_i2c_async_ctx_t *ctx  = get_ctx_for_bank(bank);
    if (ctx != NULL && ctx->state == VMT_I2C_ASYNC_BUSY_TX)
    {
        ctx->last_status = HAL_OK;
        ctx->state       = VMT_I2C_ASYNC_IDLE;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    vmt_i2c_bank_t       bank = bank_for_handle(hi2c);
    vmt_i2c_async_ctx_t *ctx  = get_ctx_for_bank(bank);
    if (ctx != NULL)
    {
        ctx->last_status = HAL_ERROR;
        ctx->last_error  = hi2c->ErrorCode;
        ctx->state       = VMT_I2C_ASYNC_ERROR;
    }
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    vmt_i2c_bank_t       bank = bank_for_handle(hi2c);
    vmt_i2c_async_ctx_t *ctx  = get_ctx_for_bank(bank);
    if (ctx != NULL)
    {
        // Abort complete - safe to return to idle if we're in error state
        // This allows recovery after timeout abort
        if (ctx->state == VMT_I2C_ASYNC_ERROR)
        {
            ctx->state           = VMT_I2C_ASYNC_IDLE;
            ctx->rmw_in_progress = false; // Clear any pending RMW
        }
    }
}

/**
 * @brief Recover I2C async engine from error state
 * @param bank Bank to recover (BANK1 or BANK2)
 */
void vmt_i2c_async_recover_error(vmt_i2c_bank_t bank)
{
    vmt_i2c_async_ctx_t *ctx = get_ctx_for_bank(bank);
    if (ctx == NULL)
    {
        return;
    }

    if (ctx->state == VMT_I2C_ASYNC_ERROR)
    {
        // Get HAL handle for this bank
        I2C_HandleTypeDef *h = handle_for_bank(bank);

        // Abort any ongoing HAL operation
        if (h != NULL && ctx->addr7 != 0)
        {
            (void)HAL_I2C_Master_Abort_IT(h, vmt_hal_addr(ctx->addr7));
        }

        // Reset error state to idle
        ctx->state           = VMT_I2C_ASYNC_IDLE;
        ctx->last_status     = HAL_OK;
        ctx->last_error      = 0u;
        ctx->rmw_in_progress = false; // Clear any pending RMW
        ctx->op              = VMT_I2C_OP_NONE;

        debug_send("> i2c:RECOVER,%s,error_state_cleared\r\n", bank == VMT_I2C_BANK1 ? "BANK1" : "BANK2");
    }
}
