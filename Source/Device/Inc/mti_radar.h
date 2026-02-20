/**
 * @file mti_radar.h
 * @brief High-level business logic API for XM125 radar cluster.
 *
 * @details
 * This module provides application-level abstractions over the xm125_async HAL driver.
 * Use these functions for normal radar operations; xm125_async.h is for low-level control only.
 *
 * Features:
 * - Simplified initialization and configuration
 * - Automatic satellite management
 * - Multi-target tracking with history
 * - Health monitoring and diagnostics
 * - Configurable measurement callbacks
 *
 * Typical Usage:
 * 1. mti_radar_init()
 * 2. mti_radar_set_profile()
 * 3. mti_radar_start()
 * 4. Call mti_radar_process() at 1-2ms rate in main loop
 * 5. Use mti_radar_get_targets() or register callback for measurements
 *
 * @see xm125_async.h for HAL driver
 */

#ifndef MTI_RADAR_H
#define MTI_RADAR_H

#include <stdint.h>
#include <stdbool.h>
#include "xm125_async.h"

// =============================================================================
// CONFIGURATION
// =============================================================================

/** @brief Maximum targets to track per satellite. */
#define MTI_RADAR_MAX_TARGETS_PER_SAT 10u

/** @brief Maximum satellites in cluster. */
#define MTI_RADAR_MAX_SATELLITES XM125_MAX_SATELLITES

/** @brief Measurement history depth (for rate calculation, filtering, etc.). */
#define MTI_RADAR_HISTORY_DEPTH 4u

// =============================================================================
// TYPES
// =============================================================================

/**
 * @brief Single radar target with distance and strength.
 */
typedef struct
{
    uint32_t distance_mm; /**< Distance in millimeters. */
    uint16_t strength;    /**< Signal strength (relative). */
    bool     valid;       /**< True if target is valid. */
} mti_radar_target_t;

/**
 * @brief Satellite health status.
 */
typedef enum
{
    MTI_RADAR_HEALTH_UNKNOWN = 0, /**< Status not yet determined. */
    MTI_RADAR_HEALTH_OK,          /**< Satellite operating normally. */
    MTI_RADAR_HEALTH_DEGRADED,    /**< Satellite has minor issues (retries, gaps). */
    MTI_RADAR_HEALTH_ERROR,       /**< Satellite in error state. */
    MTI_RADAR_HEALTH_OFFLINE      /**< Satellite permanently offline. */
} mti_radar_health_t;

/**
 * @brief Satellite measurement data (latest frame).
 */
typedef struct
{
    uint8_t            sat_id;                                 /**< Satellite ID (1-6). */
    mti_radar_target_t targets[MTI_RADAR_MAX_TARGETS_PER_SAT]; /**< Detected targets. */
    uint8_t            num_targets;                            /**< Number of valid targets. */
    int16_t            temperature_c;                          /**< Temperature in Celsius. */
    uint32_t           measure_counter;                        /**< Measurement counter from XM125. */
    uint32_t           timestamp_ms;                           /**< HAL_GetTick() at measurement. */
    bool               near_start_edge;                        /**< True if target too close to range start. */
    bool               calibration_needed;                     /**< True if recalibration required. */
    bool               strength_valid;                         /**< True if strength passed global filter. */
    bool               range_valid;                            /**< True if distance within active config range. */
    mti_radar_health_t health;                                 /**< Satellite health status. */
} mti_radar_measurement_t;

/**
 * @brief Satellite diagnostics and statistics.
 */
typedef struct
{
    uint8_t            sat_id;               /**< Satellite ID (1-6). */
    xm125_version_t    version;              /**< Firmware version. */
    xm125_sat_state_t  state;                /**< Current state machine state. */
    mti_radar_health_t health;               /**< Health status. */
    uint8_t            consecutive_errors;   /**< Consecutive error count. */
    uint32_t           total_measurements;   /**< Total successful measurements. */
    uint32_t           total_errors;         /**< Total errors encountered. */
    uint32_t           measure_gap_count;    /**< Measurement counter gaps detected. */
    uint32_t           last_detector_status; /**< Last DETECTOR_STATUS register value. */
    uint32_t           last_protocol_status; /**< Last PROTOCOL_STATUS register value. */
    bool               config_verified;      /**< True if configuration verified. */
} mti_radar_diagnostics_t;

/**
 * @brief Cluster-wide statistics.
 */
typedef struct
{
    uint8_t               active_satellites;    /**< Number of active satellites. */
    uint8_t               healthy_satellites;   /**< Number of healthy satellites. */
    uint8_t               satellites_measuring; /**< Number currently measuring. */
    uint32_t              total_measurements;   /**< Total measurements across all satellites. */
    uint32_t              total_errors;         /**< Total errors across all satellites. */
    xm125_config_status_t config_status;        /**< Configuration status. */
} mti_radar_cluster_stats_t;

/**
 * @brief Measurement callback function type.
 *
 * Called when a new measurement is available from any satellite.
 * Callback is called from mti_radar_process() context.
 *
 * @param measurement Pointer to measurement data.
 * @param user_data User-provided context pointer.
 */
typedef void (*mti_radar_callback_t)(const mti_radar_measurement_t *measurement, void *user_data);

// =============================================================================
// INITIALIZATION & CONFIGURATION
// =============================================================================

/**
 * @brief Initialize radar cluster.
 *
 * Initializes all satellites on specified I2C banks.
 * Call this once during system startup.
 *
 * @param banks_mask Bitmask: bit 0 = Bank1 (sats 1-3), bit 1 = Bank2 (sats 4-6).
 *                   Example: 0x03 enables all 6 satellites.
 */
void mti_radar_init(uint8_t banks_mask);

/**
 * @brief Set radar configuration profile.
 *
 * Applies a predefined configuration profile to all satellites.
 * Configuration is applied asynchronously; check status with mti_radar_get_config_status().
 *
 * @param profile Profile ID (see xm125_profiles.h).
 */
void mti_radar_set_profile(xm125_profile_id_t profile);

/**
 * @brief Get current configuration status.
 *
 * @return Configuration status.
 */
xm125_config_status_t mti_radar_get_config_status(void);

/**
 * @brief Register measurement callback.
 *
 * Callback is invoked when new measurement data is available from any satellite.
 * Set to NULL to disable callback.
 *
 * @param callback Function pointer to callback.
 * @param user_data User context pointer (passed to callback).
 */
void mti_radar_register_callback(mti_radar_callback_t callback, void *user_data);

/**
 * @brief Set global minimum strength filter.
 *
 * Peaks with strength below this threshold are rejected during measurement processing.
 * Applies to all satellites. Default is 0 (no filtering).
 *
 * @param min_strength Minimum strength value (0-100, 0 = disabled).
 */
void mti_radar_set_min_strength(uint16_t min_strength);

/**
 * @brief Get current minimum strength filter value.
 * @return Current minimum strength threshold (0-100).
 */
uint16_t mti_radar_get_min_strength(void);

// =============================================================================
// CONTROL
// =============================================================================

/**
 * @brief Start continuous measurements on all satellites.
 *
 * Satellites will begin measuring continuously once configuration is verified.
 */
void mti_radar_start(void);

/**
 * @brief Stop measurements on all satellites.
 *
 * Satellites will complete current measurement and return to IDLE.
 */
void mti_radar_stop(void);

/**
 * @brief Process radar state machines (call at 1-2ms rate).
 *
 * This function must be called periodically from the main loop.
 * It advances all satellite state machines and invokes measurement callbacks.
 */
void mti_radar_process(void);

/**
 * @brief Reset all satellites.
 *
 * Issues RESET_MODULE command to all satellites and restarts initialization.
 * Use for recovery from error conditions.
 */
void mti_radar_reset_all(void);

/**
 * @brief Reset specific satellite.
 *
 * Issues RESET_MODULE command to one satellite and restarts its initialization.
 *
 * @param sat_id Satellite ID (1-6).
 */
void mti_radar_reset_satellite(uint8_t sat_id);

// =============================================================================
// DATA RETRIEVAL
// =============================================================================

/**
 * @brief Get latest measurement from specific satellite.
 *
 * Returns the most recent measurement data. Does not clear the data_ready flag.
 * Use callback for event-driven processing.
 *
 * @param sat_id Satellite ID (1-6).
 * @param measurement Pointer to measurement structure to fill.
 * @return true if measurement data is available, false otherwise.
 */
bool mti_radar_get_measurement(uint8_t sat_id, mti_radar_measurement_t *measurement);

/**
 * @brief Get all current targets from all satellites.
 *
 * Retrieves targets from all active satellites in a single call.
 * Useful for centralized processing or visualization.
 *
 * @param measurements Array to fill with measurements (size MTI_RADAR_MAX_SATELLITES).
 * @param count Pointer to receive number of satellites with data.
 * @return true if any data is available, false otherwise.
 */
bool mti_radar_get_all_measurements(mti_radar_measurement_t measurements[MTI_RADAR_MAX_SATELLITES], uint8_t *count);

// =============================================================================
// DIAGNOSTICS & MONITORING
// =============================================================================

/**
 * @brief Get diagnostics for specific satellite.
 *
 * @param sat_id Satellite ID (1-6).
 * @param diag Pointer to diagnostics structure to fill.
 * @return true if satellite exists, false otherwise.
 */
bool mti_radar_get_diagnostics(uint8_t sat_id, mti_radar_diagnostics_t *diag);

/**
 * @brief Get cluster-wide statistics.
 *
 * @param stats Pointer to statistics structure to fill.
 */
void mti_radar_get_cluster_stats(mti_radar_cluster_stats_t *stats);

/**
 * @brief Get health status of specific satellite.
 *
 * @param sat_id Satellite ID (1-6).
 * @return Health status.
 */
mti_radar_health_t mti_radar_get_health(uint8_t sat_id);

/**
 * @brief Check if specific satellite is ready for measurements.
 *
 * Satellite is ready if it's in IDLE state and configuration is verified.
 *
 * @param sat_id Satellite ID (1-6).
 * @return true if ready, false otherwise.
 */
bool mti_radar_is_ready(uint8_t sat_id);

/**
 * @brief Get number of active satellites.
 *
 * @return Number of satellites initialized and not offline.
 */
uint8_t mti_radar_get_active_count(void);

// =============================================================================
// DEBUG HELPERS (CONDITIONAL)
// =============================================================================

#ifdef MTI_RADAR_DEBUG

/**
 * @brief Print satellite state and diagnostics to debug console.
 *
 * @param sat_id Satellite ID (1-6).
 */
void mti_radar_debug_print_satellite(uint8_t sat_id);

/**
 * @brief Print cluster-wide status to debug console.
 */
void mti_radar_debug_print_cluster(void);

#endif // MTI_RADAR_DEBUG

#endif // MTI_RADAR_H
