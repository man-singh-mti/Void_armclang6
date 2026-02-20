/**
 * @file xm125_async.c
 * @brief Implementation of per-satellite state machine for XM125 radar cluster.
 *
 * @details
 * Core state machine driver for XM125 satellites with independent per-satellite states.
 * Each satellite progresses through: INIT_PCA → WAKE → CONFIG → IDLE → MEASURE → ...
 * Banks operate independently; no global blocking.
 *
 * Key functions:
 * - xm125_sat_process(): Per-satellite state machine advancement
 * - xm125_cluster_process(): Main loop that calls xm125_sat_process() for each satellite
 *
 * @see xm125_async.h for API documentation
 */

#include "xm125_async.h"
#include "pca9534.h"
#include "xm125_regmap.h"
#include "xm125_profiles.h"
#include "vmt_uart.h"
#include "stm32f7xx_hal.h"
#include <string.h>

// =============================================================================
// STATIC GLOBALS
// =============================================================================

/** @brief Cluster context (all satellites). */
static xm_cluster_ctx_t xm_ctx;

/** @brief RAM copy of configuration to be written. */
static detector_config_t ram_cfg;

/** @brief Configuration dirty flag (triggers config write on next cycle). */
static bool config_dirty = true;

/** @brief Last successfully confirmed configuration. */
static detector_config_t last_confirmed_cfg;

/** @brief Global configuration status. */
static xm125_config_status_t config_status = XM125_CONFIG_PENDING;

/** @brief Global configuration error flags. */
static uint32_t config_error = 0;

/** @brief Debug flag for initialization logging. */
static bool xm_debug_init = true;

// =============================================================================
// INTERNAL HELPER PROTOTYPES
// =============================================================================

/**
 * @brief Per-satellite state machine advancement.
 * @param sat Pointer to satellite structure.
 */
static void xm125_sat_process(xm125_satellite_t *sat);

/**
 * @brief Helper to write a single 32-bit register (MSB first).
 * @param bank I2C bank.
 * @param addr7 7-bit XM125 address.
 * @param reg 16-bit register address.
 * @param val 32-bit value to write.
 * @return HAL status.
 */
static inline HAL_StatusTypeDef xm_write_u32(vmt_i2c_bank_t bank, uint8_t addr7, uint16_t reg, uint32_t val);

/**
 * @brief Helper to read a single 32-bit register (MSB first, converts to host endian).
 * @param sat Pointer to satellite structure.
 * @param reg 16-bit register address.
 * @return 32-bit value (host endian).
 */
static inline uint32_t xm_parse_u32_be(const uint8_t *buf);

/**
 * @brief Check if state has timed out.
 * @param sat Pointer to satellite structure.
 * @param timeout_ms Timeout in milliseconds.
 * @return true if timeout exceeded.
 */
static inline bool xm_state_timeout(xm125_satellite_t *sat, uint32_t timeout_ms);

/**
 * @brief Transition satellite to ERROR state.
 * @param sat Pointer to satellite structure.
 */
static void xm_error_handler(xm125_satellite_t *sat);

// =============================================================================
// INLINE HELPERS
// =============================================================================

static inline HAL_StatusTypeDef xm_write_u32(vmt_i2c_bank_t bank, uint8_t addr7, uint16_t reg, uint32_t val)
{
    uint8_t tx[4] = { (uint8_t)(val >> 24), (uint8_t)(val >> 16), (uint8_t)(val >> 8), (uint8_t)val };
    return xm125_reg_write_u32_async(bank, addr7, reg, tx);
}

static inline uint32_t xm_parse_u32_be(const uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | ((uint32_t)buf[3]);
}

static inline bool xm_state_timeout(xm125_satellite_t *sat, uint32_t timeout_ms)
{
    return (HAL_GetTick() - sat->state_start_tick) >= timeout_ms;
}

// =============================================================================
// CONFIGURATION API
// =============================================================================

void xm125_set_profile(xm125_profile_id_t id)
{
    switch (id)
    {
    case XM_PROFILE_BALANCED:
        ram_cfg = config_balanced_mvp;
        break;
    case XM_PROFILE_HIGH_SPEED:
        ram_cfg = config_high_speed;
        break;
    case XM_PROFILE_HIGH_RES:
        ram_cfg = config_high_resolution;
        break;
    case XM_PROFILE_CLOSE_RANGE:
        ram_cfg = config_close_range_calibrated;
        break;
    default:
        ram_cfg = config_balanced_mvp;
        break;
    }
    config_dirty  = true;
    config_status = XM125_CONFIG_PENDING;
}

xm125_config_status_t xm125_get_config_status(void)
{
    return config_status;
}

uint32_t xm125_get_config_error(void)
{
    return config_error;
}

const detector_config_t *xm125_get_active_config(void)
{
    return &last_confirmed_cfg;
}

// =============================================================================
// CLUSTER CONTROL API
// =============================================================================

void xm125_cluster_hw_check(uint8_t banks_mask)
{
    xm_ctx.active_count = 0;

    // Initialize only satellites requested via banks_mask
    for (uint8_t i = 0; i < XM125_MAX_SATELLITES; i++)
    {
        xm125_satellite_t *sat       = &xm_ctx.sats[i];
        uint8_t            bank_flag = (sat->bank == VMT_I2C_BANK1) ? 0x01 : 0x02;

        if (banks_mask & bank_flag)
        {
            sat->state              = XM125_SAT_INIT_PCA;
            sat->cfg_phase          = 0;
            sat->consecutive_errors = 0;
            xm_ctx.active_count++;
        }
        else
        {
            sat->state = XM125_SAT_OFFLINE;
        }
    }

    if (xm_debug_init)
    {
        debug_send("> xm125_hw_check: Starting hardware presence check for %u satellites\r\n", xm_ctx.active_count);
    }
}

void xm125_cluster_init(uint8_t banks_mask)
{
    // Clear cluster context
    memset(&xm_ctx, 0, sizeof(xm_ctx));
    xm_ctx.active_count        = 0;
    xm_ctx.running             = false;
    xm_ctx.config_applied_once = false;

    // Set default configuration
    ram_cfg       = config_balanced_mvp;
    config_dirty  = true;
    config_status = XM125_CONFIG_PENDING;
    config_error  = 0;

    // Initialize satellites based on banks_mask
    // Bank 1 (satellites 1-3)
    if (banks_mask & 0x01u)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            xm125_satellite_t *sat  = &xm_ctx.sats[xm_ctx.active_count];
            sat->id                 = i + 1;
            sat->addr7              = 0x51 + i;
            sat->addr_pca           = 0x21 + i;
            sat->bank               = VMT_I2C_BANK1;
            sat->state              = XM125_SAT_INIT_PCA;
            sat->state_start_tick   = 0;
            sat->cfg_phase          = 0;
            sat->consecutive_errors = 0;
            sat->version_read       = false;
            sat->config_applied     = false;
            sat->config_verified    = false;
            sat->data_ready         = false;
            xm_ctx.active_count++;
        }
    }

    // Bank 2 (satellites 4-6)
    if (banks_mask & 0x02u)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            xm125_satellite_t *sat  = &xm_ctx.sats[xm_ctx.active_count];
            sat->id                 = i + 4;
            sat->addr7              = 0x51 + i; // Same addresses, different bank
            sat->addr_pca           = 0x21 + i;
            sat->bank               = VMT_I2C_BANK2;
            sat->state              = XM125_SAT_INIT_PCA;
            sat->state_start_tick   = 0;
            sat->cfg_phase          = 0;
            sat->consecutive_errors = 0;
            sat->version_read       = false;
            sat->config_applied     = false;
            sat->config_verified    = false;
            sat->data_ready         = false;
            xm_ctx.active_count++;
        }
    }
}

void xm125_cluster_start(void)
{
    if (!xm_ctx.running)
    {
        xm_ctx.running = true;
        // All satellites in IDLE will automatically start measuring
    }
}

void xm125_cluster_stop(void)
{
    xm_ctx.running = false;
    // Satellites will complete current measurement and return to IDLE
}

void xm125_cluster_wait_ready(uint32_t timeout_ms, uint8_t *ready_count, uint8_t *error_count)
{
    if (ready_count == NULL || error_count == NULL)
    {
        return;
    }

    *ready_count = 0;
    *error_count = 0;

    uint32_t init_start = HAL_GetTick();

    while ((HAL_GetTick() - init_start) < timeout_ms)
    {
        // Advance state machines
        vmt_i2c_async_process();
        xm125_cluster_process();

        // Count satellite states
        uint8_t ready = 0;
        uint8_t error = 0;

        for (uint8_t i = 0; i < xm_ctx.active_count; i++)
        {
            xm125_sat_state_t state = xm_ctx.sats[i].state;

            if (state >= XM125_SAT_IDLE)
            {
                ready++;
            }
            else if (state == XM125_SAT_OFFLINE || state == XM125_SAT_ERROR)
            {
                error++;
            }
        }

        // All accounted for?
        if ((ready + error) == xm_ctx.active_count)
        {
            *ready_count = ready;
            *error_count = error;
            debug_send("> xm125 init complete: %u ready, %u failed", ready, error);
            return;
        }

        HAL_Delay(10); // Poll every 10ms
    }

    // Timeout - report current counts
    for (uint8_t i = 0; i < xm_ctx.active_count; i++)
    {
        xm125_sat_state_t state = xm_ctx.sats[i].state;
        if (state >= XM125_SAT_IDLE)
        {
            (*ready_count)++;
        }
        else if (state == XM125_SAT_OFFLINE || state == XM125_SAT_ERROR)
        {
            (*error_count)++;
        }
    }

    debug_send("@db,XM125 init timeout: %u ready, %u failed", *ready_count, *error_count);
}

bool xm125_hw_presence_check(bool *all_accounted, uint8_t *present, uint8_t *failed)
{
    uint8_t active_sats = xm125_get_active_count();
    *present            = 0;
    *failed             = 0;

    if (active_sats == 0)
    {
        *all_accounted = true;
        *present       = 0;
        *failed        = 0;
        return false; // No satellites configured
    }

    for (uint8_t i = 0; i < active_sats; i++)
    {
        xm125_sat_state_t state = xm125_get_satellite_state(i);
        if (state >= XM125_SAT_WAKE_POLL)
        {
            (*present)++;
        }
        else if (state == XM125_SAT_OFFLINE || state == XM125_SAT_ERROR)
        {
            (*failed)++;
        }
    }

    *all_accounted = ((*present + *failed) == active_sats);
    return true;
}

void xm125_cluster_process(void)
{
    // Process each satellite independently
    for (uint8_t i = 0; i < xm_ctx.active_count; i++)
    {
        xm125_satellite_t *sat = &xm_ctx.sats[i];

        // Skip offline satellites
        if (sat->state == XM125_SAT_OFFLINE)
            continue;

        // Skip if bank is busy (another transaction in progress)
        if (!vmt_i2c_async_is_idle(sat->bank))
            continue;

        // Advance this satellite's state machine
        xm125_sat_process(sat);
    }
}

void xm125_cluster_reset_all(void)
{
    for (uint8_t i = 0; i < xm_ctx.active_count; i++)
    {
        xm125_satellite_reset(i);
    }
}

void xm125_satellite_reset(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return;

    xm125_satellite_t *sat = &xm_ctx.sats[sat_idx];

    // Issue RESET_MODULE command
    xm_write_u32(sat->bank, sat->addr7, XM125_REG_COMMAND_ADDR, XM125_COMMAND_RESET_MODULE);

    // Clear configuration flags
    sat->config_applied        = false;
    sat->config_verified       = false;
    xm_ctx.config_applied_once = false;

    // Restart from INIT_PCA
    sat->state              = XM125_SAT_INIT_PCA;
    sat->state_start_tick   = HAL_GetTick();
    sat->consecutive_errors = 0;
}

// =============================================================================
// GETTER FUNCTIONS - VERSION & STATUS
// =============================================================================

bool xm125_get_version(uint8_t sat_idx, xm125_version_t *ver)
{
    if (sat_idx >= xm_ctx.active_count || ver == NULL)
        return false;

    xm125_satellite_t *sat = &xm_ctx.sats[sat_idx];
    if (!sat->version_read)
        return false;

    *ver = sat->version;
    return true;
}

uint32_t xm125_get_detector_status(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return 0;
    return xm_ctx.sats[sat_idx].last_detector_status;
}

uint32_t xm125_get_protocol_status(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return 0;
    return xm_ctx.sats[sat_idx].last_protocol_status;
}

bool xm125_is_config_verified(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return false;
    return xm_ctx.sats[sat_idx].config_verified;
}

xm125_sat_state_t xm125_get_satellite_state(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return XM125_SAT_OFFLINE;
    return xm_ctx.sats[sat_idx].state;
}

uint8_t xm125_get_consecutive_errors(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return 0;
    return xm_ctx.sats[sat_idx].consecutive_errors;
}

uint8_t xm125_get_active_count(void)
{
    return xm_ctx.active_count;
}

// =============================================================================
// GETTER FUNCTIONS - MEASUREMENT DATA
// =============================================================================

bool xm125_measurement_ready(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return false;
    return xm_ctx.sats[sat_idx].data_ready;
}

bool xm125_get_measurement(uint8_t sat_idx, uint32_t *dists, uint16_t *strengths, uint8_t *num_peaks)
{
    if (sat_idx >= xm_ctx.active_count || dists == NULL || strengths == NULL || num_peaks == NULL)
        return false;

    xm125_satellite_t *sat = &xm_ctx.sats[sat_idx];
    if (!sat->data_ready)
        return false;

    // Copy data
    *num_peaks = sat->num_peaks_actual;
    for (uint8_t i = 0; i < sat->num_peaks_actual && i < XM125_MAX_PEAKS; i++)
    {
        dists[i]     = sat->peak_dist_mm[i];
        strengths[i] = sat->peak_strength[i];
    }

    // Clear data_ready flag after successful read
    sat->data_ready = false;
    return true;
}

uint8_t xm125_get_num_peaks(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return 0;
    return xm_ctx.sats[sat_idx].num_peaks_actual;
}

uint32_t xm125_get_measure_counter(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return 0;
    return xm_ctx.sats[sat_idx].measure_counter_last;
}

int16_t xm125_get_temperature(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return 0;
    return xm_ctx.sats[sat_idx].temperature_c;
}

bool xm125_is_near_start_edge(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return false;
    return xm_ctx.sats[sat_idx].near_start_edge;
}

bool xm125_needs_recalibration(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return false;
    return xm_ctx.sats[sat_idx].calibration_needed;
}

uint32_t xm125_get_measure_gap_count(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return 0;
    return xm_ctx.sats[sat_idx].measure_gap_count;
}

// =============================================================================
// ERROR HANDLER
// =============================================================================

static void xm_error_handler(xm125_satellite_t *sat)
{
    sat->consecutive_errors++;

    // Read error registers for logging
    // (This is non-blocking; we'll read on next call if bank is busy)
    if (vmt_i2c_async_is_idle(sat->bank))
    {
        // Try to read DETECTOR_STATUS and PROTOCOL_STATUS
        xm125_reg_read_u32_async(sat->bank, sat->addr7, XM125_REG_DETECTOR_STATUS_ADDR, sat->rx_scratch);
        // Will be parsed on next state entry
    }

    // Recovery strategy
    if (sat->consecutive_errors < 3)
    {
        // Try simple re-init
        sat->state            = XM125_SAT_INIT_PCA;
        sat->state_start_tick = HAL_GetTick();
    }
    else if (sat->consecutive_errors < XM125_MAX_CONSECUTIVE_ERRORS)
    {
        // Issue RESET_MODULE
        if (vmt_i2c_async_is_idle(sat->bank))
        {
            xm_write_u32(sat->bank, sat->addr7, XM125_REG_COMMAND_ADDR, XM125_COMMAND_RESET_MODULE);
            sat->config_applied   = false;
            sat->config_verified  = false;
            sat->state            = XM125_SAT_INIT_PCA;
            sat->state_start_tick = HAL_GetTick();
        }
    }
    else
    {
        // Mark permanently OFFLINE after max errors
        sat->state = XM125_SAT_OFFLINE;
    }
}

// =============================================================================
// PER-SATELLITE STATE MACHINE
// =============================================================================

static void xm125_sat_process(xm125_satellite_t *sat)
{
    HAL_StatusTypeDef status;
    uint32_t          reg_val;

    switch (sat->state)
    {
    // -------------------------------------------------------------------------
    case XM125_SAT_INIT_PCA:
        // Initialize PCA9534: Write CONFIG register (phase 0), then OUTPUT register (phase 1)
        if (sat->cfg_phase == 0)
        {
            // Phase 0: Write CONFIG register (set IO directions)
            status = pca9534_init_for_xm125(sat->bank, sat->addr_pca);
            if (status == HAL_OK)
            {
                sat->cfg_phase = 1;
            }
            else if (status != HAL_BUSY)
            {
                xm_error_handler(sat);
            }
        }
        else if (sat->cfg_phase == 1)
        {
            // Phase 1: Write OUTPUT register (WAKE_UP=1, NRESET=1)
            status = pca9534_xm125_run(sat->bank, sat->addr_pca);
            if (status == HAL_OK)
            {
                sat->cfg_phase        = 0; // Reset for future use
                sat->state            = XM125_SAT_WAKE_POLL;
                sat->state_start_tick = HAL_GetTick();
            }
            else if (status != HAL_BUSY)
            {
                xm_error_handler(sat);
            }
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_WAKE_ASSERT:
        // NOTE: This state is now skipped - WAKE_UP already asserted in INIT_PCA
        // Keeping for potential future use (re-wake after sleep)
        status = pca9534_xm125_run(sat->bank, sat->addr_pca);
        if (status == HAL_OK)
        {
            sat->state            = XM125_SAT_WAKE_POLL;
            sat->state_start_tick = HAL_GetTick();
        }
        else if (status != HAL_BUSY)
        {
            xm_error_handler(sat);
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_WAKE_POLL:
        // Poll MCU_INT until HIGH
        status = pca9534_read_mcu_int(sat->bank, sat->addr_pca, &sat->rx_scratch[0]);
        if (status == HAL_OK)
        {
            if (pca9534_is_xm125_ready(sat->rx_scratch[0]))
            {
                // MCU_INT is HIGH - XM125 hardware present and booted
                // Advance to version read to start full initialization in background
                if (xm_debug_init)
                {
                    uint8_t sat_idx = (uint8_t)(sat - &xm_ctx.sats[0]);
                    debug_send("> xm125[%u]: MCU_INT HIGH, advancing to version read\r\n", sat_idx);
                }
                sat->state            = XM125_SAT_READ_VERSION;
                sat->state_start_tick = HAL_GetTick();
            }
            else if (xm_state_timeout(sat, XM125_WAKE_TIMEOUT_MS))
            {
                // Timeout waiting for MCU_INT
                xm_error_handler(sat);
            }
            // else: keep polling
        }
        else if (status != HAL_BUSY && xm_state_timeout(sat, XM125_WAKE_TIMEOUT_MS))
        {
            xm_error_handler(sat);
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_READ_VERSION:
        // Read VERSION register
        status = xm125_reg_read_u32_async(sat->bank, sat->addr7, XM125_REG_VERSION_ADDR, sat->rx_scratch);
        if (status == HAL_OK)
        {
            reg_val            = xm_parse_u32_be(sat->rx_scratch);
            sat->version.major = (uint16_t)((reg_val >> 16) & 0xFFFF);
            sat->version.minor = (uint8_t)((reg_val >> 8) & 0xFF);
            sat->version.patch = (uint8_t)(reg_val & 0xFF);
            sat->version_read  = true;

            // Move to config phase
            sat->state            = XM125_SAT_CONFIG_WRITE;
            sat->cfg_phase        = 0;
            sat->state_start_tick = HAL_GetTick();
        }
        else if (status != HAL_BUSY)
        {
            xm_error_handler(sat);
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_CONFIG_WRITE:
        // Check if we need to write config
        if (xm_ctx.config_applied_once && !config_dirty)
        {
            // Config already applied and not dirty - skip to IDLE
            sat->state = XM125_SAT_IDLE;
            break;
        }

        // Write 13 configuration registers sequentially
        // cfg_phase tracks which register [0..12]
        if (sat->cfg_phase == 0 && vmt_i2c_async_is_idle(sat->bank))
        {
            status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_START_ADDR, ram_cfg.start_mm);
        }
        else if (sat->cfg_phase == 1 && vmt_i2c_async_is_idle(sat->bank))
        {
            status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_END_ADDR, ram_cfg.end_mm);
        }
        else if (sat->cfg_phase == 2 && vmt_i2c_async_is_idle(sat->bank))
        {
            status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_MAX_STEP_LENGTH_ADDR, ram_cfg.max_step_length);
        }
        else if (sat->cfg_phase == 3 && vmt_i2c_async_is_idle(sat->bank))
        {
            status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_CLOSE_RANGE_LLC_ADDR, ram_cfg.close_range_leakage);
        }
        else if (sat->cfg_phase == 4 && vmt_i2c_async_is_idle(sat->bank))
        {
            status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_SIGNAL_QUALITY_ADDR, (uint32_t)ram_cfg.signal_quality);
        }
        else if (sat->cfg_phase == 5 && vmt_i2c_async_is_idle(sat->bank))
        {
            status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_MAX_PROFILE_ADDR, ram_cfg.max_profile);
        }
        else if (sat->cfg_phase == 6 && vmt_i2c_async_is_idle(sat->bank))
        {
            status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_THRESHOLD_METHOD_ADDR, ram_cfg.threshold_method);
        }
        else if (sat->cfg_phase == 7 && vmt_i2c_async_is_idle(sat->bank))
        {
            status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_PEAK_SORTING_ADDR, ram_cfg.peak_sorting_method);
        }
        else if (sat->cfg_phase == 8 && vmt_i2c_async_is_idle(sat->bank))
        {
            status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_NUM_FRAMES_RECORDED_ADDR, ram_cfg.num_frames_rec_thresh);
        }
        else if (sat->cfg_phase == 9 && vmt_i2c_async_is_idle(sat->bank))
        {
            status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_FIXED_AMP_THRESH_ADDR, ram_cfg.fixed_amp_thresh_val);
        }
        else if (sat->cfg_phase == 10 && vmt_i2c_async_is_idle(sat->bank))
        {
            status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_THRESH_SENS_ADDR, ram_cfg.threshold_sensitivity);
        }
        else if (sat->cfg_phase == 11 && vmt_i2c_async_is_idle(sat->bank))
        {
            status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_REFLECTOR_SHAPE_ADDR, ram_cfg.reflector_shape);
        }
        else if (sat->cfg_phase == 12 && vmt_i2c_async_is_idle(sat->bank))
        {
            status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_FIXED_STRENGTH_THRESH_ADDR, (uint32_t)ram_cfg.fixed_str_thresh_val);
        }
        else
        {
            break; // Wait for I2C idle
        }

        if (status == HAL_OK)
        {
            sat->cfg_phase++;
            if (sat->cfg_phase >= 13)
            {
                // All config registers written, move to APPLY
                sat->state            = XM125_SAT_CONFIG_APPLY;
                sat->state_start_tick = HAL_GetTick();
            }
        }
        else if (status != HAL_BUSY)
        {
            xm_error_handler(sat);
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_CONFIG_APPLY:
        // Execute APPLY_CONFIG_AND_CALIBRATE command
        status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_COMMAND_ADDR, XM125_COMMAND_APPLY_CONFIG_AND_CALIBRATE);
        if (status == HAL_OK)
        {
            sat->state            = XM125_SAT_CONFIG_POLL;
            sat->state_start_tick = HAL_GetTick();
        }
        else if (status != HAL_BUSY)
        {
            xm_error_handler(sat);
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_CONFIG_POLL:
        // Poll DETECTOR_STATUS until BUSY bit clears
        status = xm125_reg_read_u32_async(sat->bank, sat->addr7, XM125_REG_DETECTOR_STATUS_ADDR, sat->rx_scratch);
        if (status == HAL_OK)
        {
            reg_val                   = xm_parse_u32_be(sat->rx_scratch);
            sat->last_detector_status = reg_val;

            if ((reg_val & XM125_DETECTOR_STATUS_BUSY_MASK) == 0)
            {
                // BUSY cleared, move to verify
                sat->state = XM125_SAT_CONFIG_VERIFY;
            }
            else if (xm_state_timeout(sat, XM125_CONFIG_TIMEOUT_MS))
            {
                // Timeout waiting for config/calibration
                xm_error_handler(sat);
            }
            // else: keep polling
        }
        else if (status != HAL_BUSY && xm_state_timeout(sat, XM125_CONFIG_TIMEOUT_MS))
        {
            xm_error_handler(sat);
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_CONFIG_VERIFY:
        // Verify all 10 OK bits per XM125 spec section 5.2 embedded host example
        // Manufacturer reference: configuration_ok() checks ALL_OK_MASK
        {
            uint32_t status = sat->last_detector_status;
            if ((status & XM125_DETECTOR_STATUS_ALL_OK_MASK) == XM125_DETECTOR_STATUS_ALL_OK_MASK)
            {
                // Configuration and calibration successful
                sat->config_applied        = true;
                sat->config_verified       = true;
                xm_ctx.config_applied_once = true;
                config_dirty               = false;
                last_confirmed_cfg         = ram_cfg;
                config_status              = XM125_CONFIG_OK;
                sat->consecutive_errors    = 0; // Reset error count on success

                sat->state = XM125_SAT_IDLE;
            }
            else
            {
                // Configuration or calibration failed
                config_error  = status;
                config_status = XM125_CONFIG_ERROR;
                xm_error_handler(sat);
            }
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_IDLE:
        // Ready for measurement
        if (xm_ctx.running)
        {
            // Start measurement automatically
            sat->state            = XM125_SAT_MEASURE_CMD;
            sat->state_start_tick = HAL_GetTick();
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_MEASURE_CMD:
        // Execute MEASURE_DISTANCE command
        status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_COMMAND_ADDR, XM125_COMMAND_MEASURE_DISTANCE);
        if (status == HAL_OK)
        {
            sat->state            = XM125_SAT_MEASURE_POLL;
            sat->state_start_tick = HAL_GetTick();
        }
        else if (status != HAL_BUSY)
        {
            xm_error_handler(sat);
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_MEASURE_POLL:
        // Poll DETECTOR_STATUS until measurement completes
        status = xm125_reg_read_u32_async(sat->bank, sat->addr7, XM125_REG_DETECTOR_STATUS_ADDR, sat->rx_scratch);
        if (status == HAL_OK)
        {
            reg_val                   = xm_parse_u32_be(sat->rx_scratch);
            sat->last_detector_status = reg_val;

            if ((reg_val & XM125_DETECTOR_STATUS_BUSY_MASK) == 0)
            {
                // Measurement complete - read counter for gap detection
                sat->state = XM125_SAT_READ_MEASURE_COUNTER;
            }
            else if (xm_state_timeout(sat, XM125_MEASURE_TIMEOUT_MS))
            {
                xm_error_handler(sat);
            }
        }
        else if (status != HAL_BUSY && xm_state_timeout(sat, XM125_MEASURE_TIMEOUT_MS))
        {
            xm_error_handler(sat);
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_READ_MEASURE_COUNTER:
        // Read MEASURE_COUNTER for gap detection
        status = xm125_reg_read_u32_async(sat->bank, sat->addr7, XM125_REG_MEASURE_COUNTER_ADDR, sat->rx_scratch);
        if (status == HAL_OK)
        {
            uint32_t measure_counter = xm_parse_u32_be(sat->rx_scratch);

            // Track measurement counter gaps
            if (sat->measure_counter_last != 0) // Skip first reading
            {
                if (measure_counter < sat->measure_counter_last)
                {
                    // Module restarted (counter wrapped to 0)
                    sat->measure_gap_count++;
                }
                else if (measure_counter > sat->measure_counter_last + 1)
                {
                    // Missed measurements (counter jumped)
                    sat->measure_gap_count += (measure_counter - sat->measure_counter_last - 1);
                }
            }
            sat->measure_counter_last = measure_counter;

            sat->state = XM125_SAT_READ_RESULT;
        }
        else if (status != HAL_BUSY)
        {
            xm_error_handler(sat);
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_READ_RESULT:
        // Single burst read: DISTANCE_RESULT + all 10 peak distances + all 10 peak strengths
        // Registers 0x0010-0x0024 (21 registers = 84 bytes)
        // 0x0010: DISTANCE_RESULT
        // 0x0011-0x001A: Peak0-9 distances
        // 0x001B-0x0024: Peak0-9 strengths
        status = xm125_reg_read_burst_async(sat->bank, sat->addr7, XM125_REG_DISTANCE_RESULT_ADDR, sat->rx_scratch, 21);
        if (status == HAL_OK)
        {
            // Parse DISTANCE_RESULT (first 4 bytes)
            reg_val                 = xm_parse_u32_be(&sat->rx_scratch[0]);
            sat->num_peaks_actual   = (uint8_t)(reg_val & XM125_DISTANCE_RESULT_NUM_DISTANCES_MASK);
            sat->near_start_edge    = (reg_val & XM125_DISTANCE_RESULT_NEAR_START_EDGE_MASK) != 0;
            sat->calibration_needed = (reg_val & XM125_DISTANCE_RESULT_CALIBRATION_NEEDED_MASK) != 0;
            sat->measure_error      = (reg_val & XM125_DISTANCE_RESULT_MEASURE_ERROR_MASK) != 0;
            sat->temperature_c      = (int16_t)((reg_val >> 16) & 0xFFFF);

            // Parse all 10 peak distances (bytes 4-43)
            for (uint8_t i = 0; i < XM125_MAX_PEAKS; i++)
            {
                sat->peak_dist_mm[i] = xm_parse_u32_be(&sat->rx_scratch[4 + (i * 4)]);
            }

            // Parse all 10 peak strengths (bytes 44-83, upper 16 bits)
            for (uint8_t i = 0; i < XM125_MAX_PEAKS; i++)
            {
                uint32_t val          = xm_parse_u32_be(&sat->rx_scratch[44 + (i * 4)]);
                sat->peak_strength[i] = (uint16_t)((val >> 16) & 0xFFFF);
            }

            // Determine next state
            if (sat->measure_error)
            {
                // Measurement failed
                sat->data_ready = false;
                sat->state      = XM125_SAT_IDLE;
            }
            else if (sat->calibration_needed)
            {
                // Need recalibration
                sat->state = XM125_SAT_RECAL_CMD;
            }
            else
            {
                // Measurement successful (even if num_peaks = 0)
                sat->data_ready         = true;
                sat->consecutive_errors = 0; // Reset error count on success
                sat->state              = XM125_SAT_IDLE;
            }
        }
        else if (status != HAL_BUSY)
        {
            xm_error_handler(sat);
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_RECAL_CMD:
        // Execute RECALIBRATE command
        status = xm_write_u32(sat->bank, sat->addr7, XM125_REG_COMMAND_ADDR, XM125_COMMAND_RECALIBRATE);
        if (status == HAL_OK)
        {
            sat->state            = XM125_SAT_RECAL_POLL;
            sat->state_start_tick = HAL_GetTick();
        }
        else if (status != HAL_BUSY)
        {
            xm_error_handler(sat);
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_RECAL_POLL:
        // Poll DETECTOR_STATUS until recalibration completes
        status = xm125_reg_read_u32_async(sat->bank, sat->addr7, XM125_REG_DETECTOR_STATUS_ADDR, sat->rx_scratch);
        if (status == HAL_OK)
        {
            reg_val                   = xm_parse_u32_be(sat->rx_scratch);
            sat->last_detector_status = reg_val;

            if ((reg_val & XM125_DETECTOR_STATUS_BUSY_MASK) == 0)
            {
                // Check if recalibration succeeded
                if ((reg_val & XM125_DETECTOR_STATUS_ALL_OK_MASK) == XM125_DETECTOR_STATUS_ALL_OK_MASK)
                {
                    sat->calibration_needed = false;
                    sat->state              = XM125_SAT_IDLE;
                }
                else
                {
                    // Recalibration failed
                    xm_error_handler(sat);
                }
            }
            else if (xm_state_timeout(sat, XM125_CONFIG_TIMEOUT_MS))
            {
                xm_error_handler(sat);
            }
        }
        else if (status != HAL_BUSY && xm_state_timeout(sat, XM125_CONFIG_TIMEOUT_MS))
        {
            xm_error_handler(sat);
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_SLEEP:
        // Put satellite to sleep (WAKE_UP LOW)
        status = pca9534_xm125_sleep(sat->bank, sat->addr_pca);
        if (status == HAL_OK)
        {
            sat->state = XM125_SAT_IDLE; // Or stay in SLEEP state
        }
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_ERROR:
        // Error state - handled by xm_error_handler
        // State machine will be restarted or marked OFFLINE
        break;

    // -------------------------------------------------------------------------
    case XM125_SAT_OFFLINE:
        // Permanently offline - do nothing
        break;

    default:
        // Unknown state - go to error
        xm_error_handler(sat);
        break;
    }
}

// =============================================================================
// DEBUG HELPERS (CONDITIONAL COMPILATION)
// =============================================================================

#ifdef XM125_DEBUG_ERRORS

void xm125_decode_detector_errors(uint32_t status, char *buf, size_t len)
{
    if (buf == NULL || len == 0)
        return;

    buf[0]     = '\0'; // Start empty
    size_t pos = 0;

    // Check error bits [16-28]
    if (status & XM125_DETECTOR_STATUS_RSS_REGISTER_ERR_MASK)
        pos += snprintf(buf + pos, len - pos, "RSS_REGISTER_ERR ");
    if (status & XM125_DETECTOR_STATUS_CONFIG_CREATE_ERR_MASK)
        pos += snprintf(buf + pos, len - pos, "CONFIG_CREATE_ERR ");
    if (status & XM125_DETECTOR_STATUS_SENSOR_CREATE_ERR_MASK)
        pos += snprintf(buf + pos, len - pos, "SENSOR_CREATE_ERR ");
    if (status & XM125_DETECTOR_STATUS_DETECTOR_CREATE_ERR_MASK)
        pos += snprintf(buf + pos, len - pos, "DETECTOR_CREATE_ERR ");
    if (status & XM125_DETECTOR_STATUS_DETECTOR_BUFFER_ERR_MASK)
        pos += snprintf(buf + pos, len - pos, "DETECTOR_BUFFER_ERR ");
    if (status & XM125_DETECTOR_STATUS_SENSOR_BUFFER_ERR_MASK)
        pos += snprintf(buf + pos, len - pos, "SENSOR_BUFFER_ERR ");
    if (status & XM125_DETECTOR_STATUS_CALIB_BUFFER_ERR_MASK)
        pos += snprintf(buf + pos, len - pos, "CALIB_BUFFER_ERR ");
    if (status & XM125_DETECTOR_STATUS_CONFIG_APPLY_ERR_MASK)
        pos += snprintf(buf + pos, len - pos, "CONFIG_APPLY_ERR ");
    if (status & XM125_DETECTOR_STATUS_SENSOR_CAL_ERR_MASK)
        pos += snprintf(buf + pos, len - pos, "SENSOR_CAL_ERR ");
    if (status & XM125_DETECTOR_STATUS_DETECTOR_CAL_ERR_MASK)
        pos += snprintf(buf + pos, len - pos, "DETECTOR_CAL_ERR ");
    if (status & XM125_DETECTOR_STATUS_DETECTOR_ERROR_MASK)
        pos += snprintf(buf + pos, len - pos, "DETECTOR_ERROR ");

    if (pos == 0)
        snprintf(buf, len, "NO_ERRORS");
}

void xm125_decode_protocol_errors(uint32_t status, char *buf, size_t len)
{
    if (buf == NULL || len == 0)
        return;

    buf[0]     = '\0';
    size_t pos = 0;

    if (status & XM125_PROTOCOL_STATUS_PROTOCOL_STATE_ERROR_MASK)
        pos += snprintf(buf + pos, len - pos, "PROTOCOL_STATE_ERROR ");
    if (status & XM125_PROTOCOL_STATUS_PACKET_LENGTH_ERROR_MASK)
        pos += snprintf(buf + pos, len - pos, "PACKET_LENGTH_ERROR ");
    if (status & XM125_PROTOCOL_STATUS_ADDRESS_ERROR_MASK)
        pos += snprintf(buf + pos, len - pos, "ADDRESS_ERROR ");
    if (status & XM125_PROTOCOL_STATUS_WRITE_FAILED_MASK)
        pos += snprintf(buf + pos, len - pos, "WRITE_FAILED ");
    if (status & XM125_PROTOCOL_STATUS_WRITE_TO_RO_MASK)
        pos += snprintf(buf + pos, len - pos, "WRITE_TO_RO ");

    if (pos == 0)
        snprintf(buf, len, "NO_ERRORS");
}

void xm125_enable_uart_logs(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return;

    xm125_satellite_t *sat = &xm_ctx.sats[sat_idx];
    xm_write_u32(sat->bank, sat->addr7, XM125_REG_COMMAND_ADDR, XM125_COMMAND_ENABLE_UART_LOGS);
}

void xm125_log_configuration(uint8_t sat_idx)
{
    if (sat_idx >= xm_ctx.active_count)
        return;

    xm125_satellite_t *sat = &xm_ctx.sats[sat_idx];
    xm_write_u32(sat->bank, sat->addr7, XM125_REG_COMMAND_ADDR, XM125_COMMAND_LOG_CONFIGURATION);
}

#endif // XM125_DEBUG_ERRORS
