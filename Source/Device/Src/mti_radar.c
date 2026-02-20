/**
 * @file mti_radar.c
 * @brief Implementation of high-level business logic for XM125 radar cluster.
 *
 * @details
 * Wraps xm125_async HAL driver with application-level abstractions:
 * - Automatic measurement data collection
 * - Health monitoring and statistics
 * - Event callbacks
 * - Target tracking and history
 *
 * @see mti_radar.h for API documentation
 */

#include "mti_radar.h"
#include "xm125_async.h"
#include "xm125_profiles.h"
#include "stm32f7xx_hal.h"
#include <string.h>

// =============================================================================
// STATIC STATE
// =============================================================================

/**
 * @brief Per-satellite runtime context.
 */
typedef struct
{
    mti_radar_measurement_t last_measurement;   /**< Latest measurement. */
    uint32_t                total_measurements; /**< Total successful measurements. */
    uint32_t                total_errors;       /**< Total errors. */
    bool                    measurement_ready;  /**< New data available. */
} sat_context_t;

/** @brief Satellite contexts. */
static sat_context_t sat_ctx[MTI_RADAR_MAX_SATELLITES];

/** @brief Active satellite count. */
static uint8_t active_count = 0;

/** @brief Measurement callback. */
static mti_radar_callback_t measurement_callback = NULL;

/** @brief Callback user data. */
static void *callback_user_data = NULL;

/** @brief Cluster running flag. */
static bool cluster_running = false;

/** @brief Global minimum strength filter (0 = disabled). */
static uint16_t global_min_strength = 0;

/** @brief Per-satellite tracking: had valid peaks last frame (for strength hysteresis). */
static bool had_valid_peaks_last_frame[MTI_RADAR_MAX_SATELLITES] = { false };

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

/**
 * @brief Convert satellite ID (1-6) to internal index (0-5).
 * @param sat_id Satellite ID.
 * @return Internal index, or 0xFF if invalid.
 */
static inline uint8_t sat_id_to_idx(uint8_t sat_id)
{
    if (sat_id < 1 || sat_id > MTI_RADAR_MAX_SATELLITES)
        return 0xFF;
    return sat_id - 1;
}

/**
 * @brief Determine health status from satellite state and error count.
 * @param state Current state.
 * @param consecutive_errors Error count.
 * @return Health status.
 */
static mti_radar_health_t determine_health(xm125_sat_state_t state, uint8_t consecutive_errors)
{
    if (state == XM125_SAT_OFFLINE)
        return MTI_RADAR_HEALTH_OFFLINE;

    if (state == XM125_SAT_ERROR)
        return MTI_RADAR_HEALTH_ERROR;

    if (consecutive_errors > 0)
        return MTI_RADAR_HEALTH_DEGRADED;

    if (state >= XM125_SAT_IDLE)
        return MTI_RADAR_HEALTH_OK;

    return MTI_RADAR_HEALTH_UNKNOWN;
}

/**
 * @brief Poll satellite for new measurements and invoke callback if ready.
 * @param sat_idx Satellite index (0-5).
 */
static void poll_satellite_measurement(uint8_t sat_idx)
{
    if (sat_idx >= active_count)
        return;

    // Check if new measurement is ready
    if (!xm125_measurement_ready(sat_idx))
        return;

    // Retrieve measurement data
    uint32_t dists[XM125_MAX_PEAKS];
    uint16_t strengths[XM125_MAX_PEAKS];
    uint8_t  num_peaks = 0;

    if (!xm125_get_measurement(sat_idx, dists, strengths, &num_peaks))
        return;

    // Build measurement structure
    sat_context_t           *ctx  = &sat_ctx[sat_idx];
    mti_radar_measurement_t *meas = &ctx->last_measurement;

    meas->sat_id             = sat_idx + 1;
    meas->temperature_c      = xm125_get_temperature(sat_idx);
    meas->measure_counter    = xm125_get_measure_counter(sat_idx);
    meas->timestamp_ms       = HAL_GetTick();
    meas->near_start_edge    = xm125_is_near_start_edge(sat_idx);
    meas->calibration_needed = xm125_needs_recalibration(sat_idx);

    // Calculate strength threshold with ±3% hysteresis
    uint16_t strength_threshold = global_min_strength;
    if (had_valid_peaks_last_frame[sat_idx] && global_min_strength > 0)
    {
        // Apply -3% hysteresis for exit (lower threshold)
        uint16_t hysteresis = (global_min_strength * 3u) / 100u;
        strength_threshold  = (global_min_strength > hysteresis) ? (global_min_strength - hysteresis) : 0;
    }

    // Filter peaks by strength and close-range (limit to 4 peaks, XM125 already did range filtering)
    uint8_t valid_count = 0;
    for (uint8_t i = 0; i < num_peaks && valid_count < 4; i++)
    {
        uint32_t dist = dists[i];
        uint16_t str  = strengths[i];

        // Apply strength filter with hysteresis
        bool strength_ok = (global_min_strength == 0) || (str >= strength_threshold);

        // Filter out probe body reflections and mud on sensor (< 20mm)
        bool distance_ok = (dist >= 20);

        // Store if both filters pass
        if (strength_ok && distance_ok)
        {
            meas->targets[valid_count].distance_mm = dist;
            meas->targets[valid_count].strength    = str;
            meas->targets[valid_count].valid       = true;
            valid_count++;
        }
    }

    meas->num_targets                   = valid_count;
    had_valid_peaks_last_frame[sat_idx] = (valid_count > 0);

    // Set validity flags (true if at least one valid target)
    meas->strength_valid = (valid_count > 0);
    meas->range_valid    = (valid_count > 0);

    // Clear unused targets
    for (uint8_t i = meas->num_targets; i < MTI_RADAR_MAX_TARGETS_PER_SAT; i++)
    {
        meas->targets[i].valid = false;
    }

    // Update health status
    xm125_sat_state_t state              = xm125_get_satellite_state(sat_idx);
    uint8_t           consecutive_errors = xm125_get_consecutive_errors(sat_idx);
    meas->health                         = determine_health(state, consecutive_errors);

    // Update statistics
    ctx->total_measurements++;
    ctx->measurement_ready = true;

    // Invoke callback if registered
    if (measurement_callback != NULL)
    {
        measurement_callback(meas, callback_user_data);
    }
}

// =============================================================================
// INITIALIZATION & CONFIGURATION
// =============================================================================

void mti_radar_init(uint8_t banks_mask)
{
    // Clear state
    memset(sat_ctx, 0, sizeof(sat_ctx));
    active_count         = 0;
    measurement_callback = NULL;
    callback_user_data   = NULL;
    cluster_running      = false;

    // Initialize XM125 cluster
    xm125_cluster_init(banks_mask);

    // Determine active count based on banks_mask
    if (banks_mask & 0x01)
        active_count += 3;
    if (banks_mask & 0x02)
        active_count += 3;

    // Initialize measurement structures
    for (uint8_t i = 0; i < active_count; i++)
    {
        sat_ctx[i].last_measurement.sat_id = i + 1;
        sat_ctx[i].last_measurement.health = MTI_RADAR_HEALTH_UNKNOWN;
    }
}

void mti_radar_set_profile(xm125_profile_id_t profile)
{
    xm125_set_profile(profile);
}

xm125_config_status_t mti_radar_get_config_status(void)
{
    return xm125_get_config_status();
}

void mti_radar_register_callback(mti_radar_callback_t callback, void *user_data)
{
    measurement_callback = callback;
    callback_user_data   = user_data;
}

void mti_radar_set_min_strength(uint16_t min_strength)
{
    if (min_strength > 100)
    {
        min_strength = 100;
    }
    global_min_strength = min_strength;
}

uint16_t mti_radar_get_min_strength(void)
{
    return global_min_strength;
}

// =============================================================================
// CONTROL
// =============================================================================

void mti_radar_start(void)
{
    if (!cluster_running)
    {
        cluster_running = true;
        xm125_cluster_start();
    }
}

void mti_radar_stop(void)
{
    if (cluster_running)
    {
        cluster_running = false;
        xm125_cluster_stop();
    }
}

void mti_radar_process(void)
{
    // Process HAL driver state machines
    xm125_cluster_process();

    // Poll all satellites for new measurements
    for (uint8_t i = 0; i < active_count; i++)
    {
        poll_satellite_measurement(i);
    }
}

void mti_radar_reset_all(void)
{
    xm125_cluster_reset_all();

    // Clear statistics
    for (uint8_t i = 0; i < active_count; i++)
    {
        sat_ctx[i].total_measurements = 0;
        sat_ctx[i].total_errors       = 0;
        sat_ctx[i].measurement_ready  = false;
    }
}

void mti_radar_reset_satellite(uint8_t sat_id)
{
    uint8_t idx = sat_id_to_idx(sat_id);
    if (idx == 0xFF)
        return;

    xm125_satellite_reset(idx);

    // Clear statistics for this satellite
    sat_ctx[idx].total_measurements = 0;
    sat_ctx[idx].total_errors       = 0;
    sat_ctx[idx].measurement_ready  = false;
}

// =============================================================================
// DATA RETRIEVAL
// =============================================================================

bool mti_radar_get_measurement(uint8_t sat_id, mti_radar_measurement_t *measurement)
{
    if (measurement == NULL)
        return false;

    uint8_t idx = sat_id_to_idx(sat_id);
    if (idx == 0xFF || idx >= active_count)
        return false;

    sat_context_t *ctx = &sat_ctx[idx];
    if (!ctx->measurement_ready)
        return false;

    // Copy measurement data
    *measurement = ctx->last_measurement;

    // Clear flag to prevent duplicate processing
    ctx->measurement_ready = false;

    return true;
}

bool mti_radar_get_all_measurements(mti_radar_measurement_t measurements[MTI_RADAR_MAX_SATELLITES], uint8_t *count)
{
    if (measurements == NULL || count == NULL)
        return false;

    *count = 0;

    for (uint8_t i = 0; i < active_count; i++)
    {
        if (sat_ctx[i].measurement_ready)
        {
            measurements[*count] = sat_ctx[i].last_measurement;
            (*count)++;
        }
    }

    return (*count > 0);
}

// =============================================================================
// DIAGNOSTICS & MONITORING
// =============================================================================

bool mti_radar_get_diagnostics(uint8_t sat_id, mti_radar_diagnostics_t *diag)
{
    if (diag == NULL)
        return false;

    uint8_t idx = sat_id_to_idx(sat_id);
    if (idx == 0xFF || idx >= active_count)
        return false;

    sat_context_t *ctx = &sat_ctx[idx];

    // Fill diagnostics structure
    diag->sat_id = sat_id;

    // Get version
    if (!xm125_get_version(idx, &diag->version))
    {
        diag->version.major = 0;
        diag->version.minor = 0;
        diag->version.patch = 0;
    }

    // Get state and status
    diag->state              = xm125_get_satellite_state(idx);
    diag->consecutive_errors = xm125_get_consecutive_errors(idx);
    diag->health             = determine_health(diag->state, diag->consecutive_errors);

    // Get statistics
    diag->total_measurements = ctx->total_measurements;
    diag->total_errors       = ctx->total_errors;
    diag->measure_gap_count  = xm125_get_measure_gap_count(idx);

    // Get status registers
    diag->last_detector_status = xm125_get_detector_status(idx);
    diag->last_protocol_status = xm125_get_protocol_status(idx);

    // Get config status
    diag->config_verified = xm125_is_config_verified(idx);

    return true;
}

void mti_radar_get_cluster_stats(mti_radar_cluster_stats_t *stats)
{
    if (stats == NULL)
        return;

    stats->active_satellites    = active_count;
    stats->healthy_satellites   = 0;
    stats->satellites_measuring = 0;
    stats->total_measurements   = 0;
    stats->total_errors         = 0;

    for (uint8_t i = 0; i < active_count; i++)
    {
        xm125_sat_state_t  state              = xm125_get_satellite_state(i);
        uint8_t            consecutive_errors = xm125_get_consecutive_errors(i);
        mti_radar_health_t health             = determine_health(state, consecutive_errors);

        // Count healthy satellites
        if (health == MTI_RADAR_HEALTH_OK || health == MTI_RADAR_HEALTH_DEGRADED)
            stats->healthy_satellites++;

        // Count measuring satellites
        if (state == XM125_SAT_MEASURE_CMD || state == XM125_SAT_MEASURE_POLL || state == XM125_SAT_READ_MEASURE_COUNTER || state == XM125_SAT_READ_RESULT)
            stats->satellites_measuring++;

        // Accumulate statistics
        stats->total_measurements += sat_ctx[i].total_measurements;
        stats->total_errors += sat_ctx[i].total_errors;
    }

    stats->config_status = xm125_get_config_status();
}

mti_radar_health_t mti_radar_get_health(uint8_t sat_id)
{
    uint8_t idx = sat_id_to_idx(sat_id);
    if (idx == 0xFF || idx >= active_count)
        return MTI_RADAR_HEALTH_OFFLINE;

    xm125_sat_state_t state              = xm125_get_satellite_state(idx);
    uint8_t           consecutive_errors = xm125_get_consecutive_errors(idx);
    return determine_health(state, consecutive_errors);
}

bool mti_radar_is_ready(uint8_t sat_id)
{
    uint8_t idx = sat_id_to_idx(sat_id);
    if (idx == 0xFF || idx >= active_count)
        return false;

    xm125_sat_state_t state           = xm125_get_satellite_state(idx);
    bool              config_verified = xm125_is_config_verified(idx);

    return (state == XM125_SAT_IDLE && config_verified);
}

uint8_t mti_radar_get_active_count(void)
{
    return active_count;
}

// =============================================================================
// DEBUG HELPERS
// =============================================================================

#ifdef MTI_RADAR_DEBUG

#include <stdio.h>

static const char *state_names[] = { "OFFLINE",      "INIT_PCA",    "WAKE_ASSERT",   "WAKE_POLL",  "READ_VERSION", "CONFIG_WRITE",
                                     "CONFIG_APPLY", "CONFIG_POLL", "CONFIG_VERIFY", "IDLE",       "MEASURE_CMD",  "MEASURE_POLL",
                                     "READ_RESULT",  "READ_PEAKS",  "RECAL_CMD",     "RECAL_POLL", "SLEEP",        "ERROR" };

static const char *health_names[] = { "UNKNOWN", "OK", "DEGRADED", "ERROR", "OFFLINE" };

void mti_radar_debug_print_satellite(uint8_t sat_id)
{
    mti_radar_diagnostics_t diag;
    if (!mti_radar_get_diagnostics(sat_id, &diag))
    {
        debug_send("Satellite %u: INVALID\r\n", sat_id);
        return;
    }

    debug_send("=== Satellite %u ===\r\n", sat_id);
    debug_send("  Version: %u.%u.%u\r\n", diag.version.major, diag.version.minor, diag.version.patch);
    debug_send("  State: %s\r\n", state_names[diag.state]);
    debug_send("  Health: %s\r\n", health_names[diag.health]);
    debug_send("  Errors: %u consecutive, %lu total\r\n", diag.consecutive_errors, diag.total_errors);
    debug_send("  Measurements: %lu total, %lu gaps\r\n", diag.total_measurements, diag.measure_gap_count);
    debug_send("  Detector Status: 0x%08lX\r\n", diag.last_detector_status);
    debug_send("  Protocol Status: 0x%08lX\r\n", diag.last_protocol_status);
    debug_send("  Config Verified: %s\r\n", diag.config_verified ? "YES" : "NO");

    // Print latest measurement if available
    mti_radar_measurement_t meas;
    if (mti_radar_get_measurement(sat_id, &meas))
    {
        debug_send("  Latest Measurement:\r\n");
        debug_send("    Targets: %u\r\n", meas.num_targets);
        for (uint8_t i = 0; i < meas.num_targets; i++)
        {
            debug_send("      [%u] %lu mm (strength %u)\r\n", i, meas.targets[i].distance_mm, meas.targets[i].strength);
        }
        debug_send("    Temperature: %d C\r\n", meas.temperature_c);
        debug_send("    Counter: %lu\r\n", meas.measure_counter);
        debug_send("    Flags: near_edge=%u cal_needed=%u\r\n", meas.near_start_edge, meas.calibration_needed);
    }
    debug_send("\r\n");
}

void mti_radar_debug_print_cluster(void)
{
    mti_radar_cluster_stats_t stats;
    mti_radar_get_cluster_stats(&stats);

    debug_send("=== Radar Cluster ===\r\n");
    debug_send("  Active Satellites: %u\r\n", stats.active_satellites);
    debug_send("  Healthy: %u\r\n", stats.healthy_satellites);
    debug_send("  Measuring: %u\r\n", stats.satellites_measuring);
    debug_send("  Total Measurements: %lu\r\n", stats.total_measurements);
    debug_send("  Total Errors: %lu\r\n", stats.total_errors);
    debug_send("  Config Status: %u\r\n", stats.config_status);
    debug_send("\r\n");

    // Print brief status of each satellite
    for (uint8_t i = 1; i <= stats.active_satellites; i++)
    {
        mti_radar_diagnostics_t diag;
        if (mti_radar_get_diagnostics(i, &diag))
        {
            debug_send("  Sat %u: %s (%s)\r\n", i, state_names[diag.state], health_names[diag.health]);
        }
    }
    debug_send("\r\n");
}

#endif // MTI_RADAR_DEBUG
