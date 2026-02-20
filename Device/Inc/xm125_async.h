/**
 * @file xm125_async.h
 * @brief Asynchronous driver for XM125 radar satellite cluster with per-satellite state machines.
 *
 * @details
 * Architecture:
 * - Per-satellite state tracking enables true multi-bank concurrency.
 * - State machine handles: PCA9534 init, wake/MCU_INT polling, configuration, measurement, recalibration.
 * - Banks operate independently; Bank1 satellite doesn't block Bank2 satellite.
 * - All functions non-blocking; call xm125_cluster_process() at 1-2ms rate.
 * - Business logic should use mti_radar.h API; this file provides HAL-level driver primitives.
 *
 * State Flow:
 * OFFLINE → INIT_PCA → WAKE_ASSERT → WAKE_POLL → READ_VERSION → CONFIG_WRITE →
 * CONFIG_APPLY → CONFIG_POLL → CONFIG_VERIFY → IDLE → MEASURE_CMD → MEASURE_POLL →
 * READ_RESULT → READ_PEAKS → IDLE (or RECAL_CMD if calibration needed)
 *
 * @see xm125_regmap.h for register definitions
 * @see vmt_i2c_async.h for I2C primitives
 * @see pca9534.h for control plane
 * @see mti_radar.h for business logic API
 */

#ifndef XM125_ASYNC_H
#define XM125_ASYNC_H

#include <stdint.h>
#include <stdbool.h>
#include "vmt_i2c_async.h"
#include "xm125_regmap.h"

// --- Configuration Macros ---

/** @brief Maximum number of supported satellites in a cluster. */
#define XM125_MAX_SATELLITES 6u

/** @brief Maximum number of peaks XM125 can report (per spec). */
#define XM125_MAX_PEAKS 10u

/** @brief Wake timeout: MCU_INT must go HIGH within this time after WAKE_UP assertion. */
#define XM125_WAKE_TIMEOUT_MS 100u

/** @brief Config/calibration timeout: BUSY bit must clear within this time. */
#define XM125_CONFIG_TIMEOUT_MS 500u

/** @brief Measurement timeout: BUSY bit must clear within this time. */
#define XM125_MEASURE_TIMEOUT_MS 100u

/** @brief Maximum consecutive errors before marking satellite OFFLINE. */
#define XM125_MAX_CONSECUTIVE_ERRORS 10u

// --- Public Type Definitions ---

/**
 * @brief XM125 firmware version structure.
 */
typedef struct
{
    uint16_t major; /**< Major version (bits [31:16] of VERSION register). */
    uint8_t  minor; /**< Minor version (bits [15:8]). */
    uint8_t  patch; /**< Patch version (bits [7:0]). */
} xm125_version_t;

/**
 * @brief Per-satellite state machine states.
 *
 * Each satellite independently progresses through these states.
 * Multiple satellites can be in different states simultaneously.
 */
typedef enum
{
    XM125_SAT_OFFLINE = 0,          /**< Satellite failed or not initialized. */
    XM125_SAT_INIT_PCA,             /**< Initializing PCA9534 configuration register. */
    XM125_SAT_WAKE_ASSERT,          /**< Asserting WAKE_UP via PCA9534. */
    XM125_SAT_WAKE_POLL,            /**< Polling MCU_INT until HIGH (XM125 ready). */
    XM125_SAT_READ_VERSION,         /**< Reading VERSION register. */
    XM125_SAT_CONFIG_WRITE,         /**< Writing 13 configuration registers. */
    XM125_SAT_CONFIG_APPLY,         /**< Executing APPLY_CONFIG_AND_CALIBRATE command. */
    XM125_SAT_CONFIG_POLL,          /**< Polling DETECTOR_STATUS until BUSY clears. */
    XM125_SAT_CONFIG_VERIFY,        /**< Verifying ALL_OK_MASK (all 10 OK bits 0-9) per XM125 embedded host example. */
    XM125_SAT_IDLE,                 /**< Ready for measurement commands. */
    XM125_SAT_MEASURE_CMD,          /**< Executing MEASURE_DISTANCE command. */
    XM125_SAT_MEASURE_POLL,         /**< Polling DETECTOR_STATUS until measurement done. */
    XM125_SAT_READ_MEASURE_COUNTER, /**< Reading MEASURE_COUNTER for gap detection. */
    XM125_SAT_READ_RESULT,          /**< Burst reading DISTANCE_RESULT + all 10 peaks + strengths (84 bytes). */
    XM125_SAT_RECAL_CMD,            /**< Executing RECALIBRATE command. */
    XM125_SAT_RECAL_POLL,           /**< Polling DETECTOR_STATUS until recalibration done. */
    XM125_SAT_SLEEP,                /**< Putting satellite to sleep (WAKE_UP LOW). */
    XM125_SAT_ERROR                 /**< Error recovery state. */
} xm125_sat_state_t;

/**
 * @brief Configuration operation status.
 */
typedef enum
{
    XM125_CONFIG_OK      = 0, /**< Configuration succeeded and verified. */
    XM125_CONFIG_PENDING = 1, /**< Configuration in progress. */
    XM125_CONFIG_ERROR   = -1 /**< Configuration failed; check error flags. */
} xm125_config_status_t;

/**
 * @brief Individual satellite data structure.
 *
 * Contains all runtime state for a single XM125 satellite.
 * State machine operates independently per satellite.
 */
typedef struct
{
    // --- Addressing & Bank ---
    uint8_t        id;       /**< Logical satellite ID [1..6]. */
    uint8_t        addr7;    /**< XM125 7-bit I2C address (0x51-0x53). */
    uint8_t        addr_pca; /**< PCA9534 7-bit I2C address (0x21-0x23). */
    vmt_i2c_bank_t bank;     /**< I2C bank (BANK1 or BANK2). */

    // --- State Machine ---
    xm125_sat_state_t state;            /**< Current state. */
    uint32_t          state_start_tick; /**< Tick when state entered (for timeout). */
    uint8_t           cfg_phase;        /**< Config write phase [0..12] for sequential register writes. */

    // --- Error Tracking ---
    uint8_t  consecutive_errors;   /**< Consecutive error count (resets on success). */
    uint32_t last_detector_status; /**< Last DETECTOR_STATUS register value. */
    uint32_t last_protocol_status; /**< Last PROTOCOL_STATUS register value. */

    // --- Version & Configuration ---
    xm125_version_t version;         /**< Firmware version (read once at boot). */
    bool            version_read;    /**< True if version has been read. */
    bool            config_applied;  /**< True if configuration has been applied. */
    bool            config_verified; /**< True if ALL_OK_MASK verified (all 10 OK bits 0-9). */

    // --- Measurement Data ---
    bool     data_ready;                     /**< True if new measurement is available. */
    uint8_t  num_peaks_actual;               /**< Actual number of peaks from DISTANCE_RESULT [0..10]. */
    uint32_t peak_dist_mm[XM125_MAX_PEAKS];  /**< Peak distances [mm]. */
    uint16_t peak_strength[XM125_MAX_PEAKS]; /**< Peak strengths (relative units). */
    bool     near_start_edge;                /**< DISTANCE_RESULT bit[8]: object near start range. */
    bool     calibration_needed;             /**< DISTANCE_RESULT bit[9]: recalibration required. */
    bool     measure_error;                  /**< DISTANCE_RESULT bit[10]: measurement failed. */
    int16_t  temperature_c;                  /**< Temperature from DISTANCE_RESULT [31:16] (deg C, relative). */

    // --- Counters & Diagnostics ---
    uint32_t measure_counter_last; /**< Last MEASURE_COUNTER value (for gap detection). */
    uint32_t measure_gap_count;    /**< Count of detected MEASURE_COUNTER gaps. */

    // --- Scratch Buffer ---
    uint8_t rx_scratch[128]; /**< DMA-safe buffer for I2C reads (extended burst: 25 regs × 4 bytes = 100 bytes). */
} xm125_satellite_t;

/**
 * @brief Cluster context structure.
 *
 * Holds all satellites and global cluster state.
 * No global state machine; each satellite has independent state.
 */
typedef struct
{
    xm125_satellite_t sats[XM125_MAX_SATELLITES]; /**< Satellite array. */
    uint8_t           active_count;               /**< Number of active satellites (3 or 6). */
    bool              running;                    /**< True if continuous measurement loop is active. */
    bool              config_applied_once;        /**< True if config applied; prevents re-config after sleep/wake. */
} xm_cluster_ctx_t;

/**
 * @brief Detector configuration parameters.
 *
 * All fields match XM125 register map (0x0040-0x004C).
 * Units and scaling documented per field.
 */
typedef struct
{
    uint32_t start_mm;              /**< Start of measured interval [mm]. */
    uint32_t end_mm;                /**< End of measured interval [mm]. */
    uint16_t max_step_length;       /**< Max step length (0 = auto). */
    uint16_t max_profile;           /**< Max profile (1-5, see xm125_max_profile_t). */
    uint16_t close_range_leakage;   /**< Close range leakage cancellation (0=false, 1=true). */
    int32_t  signal_quality;        /**< Signal quality (×1000 scaling). */
    uint16_t threshold_method;      /**< Threshold method (see xm125_threshold_method_t). */
    uint16_t peak_sorting_method;   /**< Peak sorting (see xm125_peak_sorting_t). */
    uint16_t reflector_shape;       /**< Reflector shape (see xm125_reflector_shape_t). */
    uint16_t num_frames_rec_thresh; /**< Number of frames for recorded threshold. */
    uint32_t fixed_amp_thresh_val;  /**< Fixed amplitude threshold (×1000 scaling). */
    int32_t  fixed_str_thresh_val;  /**< Fixed strength threshold (×1000 scaling). */
    uint32_t threshold_sensitivity; /**< Threshold sensitivity (×1000 scaling). */
} detector_config_t;

/**
 * @brief Predefined profile IDs for XM125.
 */
typedef enum
{
    XM_PROFILE_BALANCED        = 1,
    XM_PROFILE_HIGH_SPEED      = 2,
    XM_PROFILE_HIGH_RES        = 3,
    XM_PROFILE_FIXED_THRESHOLD = 4,
    XM_PROFILE_CLOSE_RANGE     = 5
} xm125_profile_id_t;

// =============================================================================
// PUBLIC API - Configuration Management
// =============================================================================

/**
 * @brief Set the active detector profile for all satellites.
 * @param id Profile ID to apply.
 * @note Marks configuration as dirty; applies on next config cycle or after RESET_MODULE.
 */
void xm125_set_profile(xm125_profile_id_t id);

/**
 * @brief Get the global configuration status.
 * @return XM125_CONFIG_OK if all satellites configured, PENDING if in progress, ERROR if failed.
 */
xm125_config_status_t xm125_get_config_status(void);

/**
 * @brief Get the global configuration error flags.
 * @return Bitmask of DETECTOR_STATUS error bits from last failed configuration.
 */
uint32_t xm125_get_config_error(void);

/**
 * @brief Get pointer to the last successfully applied configuration.
 * @return Pointer to RAM copy of active configuration.
 */
const detector_config_t *xm125_get_active_config(void);

// =============================================================================
// PUBLIC API - Cluster Control
// =============================================================================

/**
 * @brief Hardware presence check only (INIT_PCA → WAKE_POLL).
 * @param banks_mask Bitmask: 0x01=Bank1, 0x02=Bank2, 0x03=Both.
 * @note Verifies XM125 hardware presence by checking MCU_INT goes high.
 *       Full initialization happens later in module_init().
 */
void xm125_cluster_hw_check(uint8_t banks_mask);

/**
 * @brief Initialize the satellite cluster.
 * @param banks_mask Bitmask: 0x01=Bank1, 0x02=Bank2, 0x03=Both.
 * @note Must be called before any other API. Initializes satellites based on bank mask.
 */
void xm125_cluster_init(uint8_t banks_mask);

/**
 * @brief Check presence of XM125 satellites and count present/failed.
 * @param all_accounted Pointer to bool set true if all satellites accounted for.
 * @param present Pointer to uint8_t to receive count of present satellites.
 * @param failed Pointer to uint8_t to receive count of failed satellites.
 * @return true if at least one satellite present, false otherwise.
 */
bool xm125_hw_presence_check(bool *all_accounted, uint8_t *present, uint8_t *failed);

/**
 * @brief Wait for satellites to reach IDLE or ERROR/OFFLINE state after initialization.
 * @param timeout_ms Maximum time to wait in milliseconds.
 * @param ready_count Output parameter: number of satellites that reached IDLE state.
 * @param error_count Output parameter: number of satellites that reached ERROR/OFFLINE state.
 * @note This function polls vmt_i2c_async_process() and xm125_cluster_process() internally.
 *       Call after xm125_cluster_init() during device initialization.
 */
void xm125_cluster_wait_ready(uint32_t timeout_ms, uint8_t *ready_count, uint8_t *error_count);

/**
 * @brief Start continuous measurement loop.
 * @note Non-blocking. State machine will cycle through measurements automatically.
 */
void xm125_cluster_start(void);

/**
 * @brief Stop continuous measurement loop.
 * @note Satellites will complete current measurement and return to IDLE.
 */
void xm125_cluster_stop(void);

/**
 * @brief Advance all satellite state machines.
 * @note MUST be called at 1-2ms rate from main loop. Processes each satellite independently.
 *       Also call vmt_i2c_async_process() in same loop for I2C transaction completion.
 */
void xm125_cluster_process(void);

/**
 * @brief Issue RESET_MODULE command to all satellites.
 * @note Clears configuration; satellites will re-initialize and re-configure.
 */
void xm125_cluster_reset_all(void);

/**
 * @brief Issue RESET_MODULE command to a single satellite.
 * @param sat_idx Satellite index [0..active_count-1].
 * @note Satellite will re-initialize and re-configure.
 */
void xm125_satellite_reset(uint8_t sat_idx);

// =============================================================================
// PUBLIC API - Per-Satellite Getters (Version & Status)
// =============================================================================

/**
 * @brief Get XM125 firmware version for a satellite.
 * @param sat_idx Satellite index [0..active_count-1].
 * @param ver Output structure for version info.
 * @return true if version has been read, false if not yet available or invalid index.
 */
bool xm125_get_version(uint8_t sat_idx, xm125_version_t *ver);

/**
 * @brief Get last DETECTOR_STATUS register value for a satellite.
 * @param sat_idx Satellite index [0..active_count-1].
 * @return Last DETECTOR_STATUS value, or 0 if invalid index.
 */
uint32_t xm125_get_detector_status(uint8_t sat_idx);

/**
 * @brief Get last PROTOCOL_STATUS register value for a satellite.
 * @param sat_idx Satellite index [0..active_count-1].
 * @return Last PROTOCOL_STATUS value, or 0 if invalid index.
 */
uint32_t xm125_get_protocol_status(uint8_t sat_idx);

/**
 * @brief Check if satellite configuration has been verified.
 * @param sat_idx Satellite index [0..active_count-1].
 * @return true if CONFIG_APPLY_OK, SENSOR_CAL_OK, DETECTOR_CAL_OK verified, false otherwise.
 */
bool xm125_is_config_verified(uint8_t sat_idx);

/**
 * @brief Get current state machine state for a satellite.
 * @param sat_idx Satellite index [0..active_count-1].
 * @return Current state, or XM125_SAT_OFFLINE if invalid index.
 */
xm125_sat_state_t xm125_get_satellite_state(uint8_t sat_idx);

/**
 * @brief Get consecutive error count for a satellite.
 * @param sat_idx Satellite index [0..active_count-1].
 * @return Error count [0..XM125_MAX_CONSECUTIVE_ERRORS].
 */
uint8_t xm125_get_consecutive_errors(uint8_t sat_idx);

/**
 * @brief Get the number of active satellites in the cluster.
 * @return Number of satellites initialized (0-6).
 */
uint8_t xm125_get_active_count(void);

// =============================================================================
// PUBLIC API - Per-Satellite Getters (Measurement Data)
// =============================================================================

/**
 * @brief Check if satellite has new measurement data available.
 * @param sat_idx Satellite index [0..active_count-1].
 * @return true if data_ready flag is set.
 */
bool xm125_measurement_ready(uint8_t sat_idx);

/**
 * @brief Get measurement data from a satellite.
 * @param sat_idx Satellite index [0..active_count-1].
 * @param dists Output array for peak distances (must be length XM125_MAX_PEAKS).
 * @param strengths Output array for peak strengths (must be length XM125_MAX_PEAKS).
 * @param num_peaks Output: actual number of valid peaks [0..10].
 * @return true if data retrieved and copied, false if no data or invalid params.
 * @note Clears data_ready flag after successful read.
 */
bool xm125_get_measurement(uint8_t sat_idx, uint32_t *dists, uint16_t *strengths, uint8_t *num_peaks);

/**
 * @brief Get number of peaks from last measurement.
 * @param sat_idx Satellite index [0..active_count-1].
 * @return Number of peaks [0..10], or 0 if no data or invalid index.
 */
uint8_t xm125_get_num_peaks(uint8_t sat_idx);

/**
 * @brief Get last MEASURE_COUNTER value.
 * @param sat_idx Satellite index [0..active_count-1].
 * @return Last MEASURE_COUNTER value.
 */
uint32_t xm125_get_measure_counter(uint8_t sat_idx);

/**
 * @brief Get temperature from last measurement.
 * @param sat_idx Satellite index [0..active_count-1].
 * @return Temperature in degrees C (relative, not absolute).
 */
int16_t xm125_get_temperature(uint8_t sat_idx);

/**
 * @brief Check if last measurement reported near-start-edge flag.
 * @param sat_idx Satellite index [0..active_count-1].
 * @return true if object detected near start of measurement range.
 */
bool xm125_is_near_start_edge(uint8_t sat_idx);

/**
 * @brief Check if satellite needs recalibration.
 * @param sat_idx Satellite index [0..active_count-1].
 * @return true if CALIBRATION_NEEDED flag set in DISTANCE_RESULT.
 */
bool xm125_needs_recalibration(uint8_t sat_idx);

/**
 * @brief Get count of MEASURE_COUNTER gaps detected.
 * @param sat_idx Satellite index [0..active_count-1].
 * @return Number of times MEASURE_COUNTER jumped by more than 1.
 */
uint32_t xm125_get_measure_gap_count(uint8_t sat_idx);

// =============================================================================
// PUBLIC API - Debug Helpers (Conditional Compilation)
// =============================================================================

#ifdef XM125_DEBUG_ERRORS

/**
 * @brief Decode DETECTOR_STATUS error bits into human-readable string.
 * @param status DETECTOR_STATUS register value.
 * @param buf Output buffer.
 * @param len Buffer length.
 * @note Decodes all 22 error/status bits.
 */
void xm125_decode_detector_errors(uint32_t status, char *buf, size_t len);

/**
 * @brief Decode PROTOCOL_STATUS error bits into human-readable string.
 * @param status PROTOCOL_STATUS register value.
 * @param buf Output buffer.
 * @param len Buffer length.
 */
void xm125_decode_protocol_errors(uint32_t status, char *buf, size_t len);

/**
 * @brief Enable XM125 UART debug logs.
 * @param sat_idx Satellite index [0..active_count-1].
 * @note Sends ENABLE_UART_LOGS command to XM125.
 */
void xm125_enable_uart_logs(uint8_t sat_idx);

/**
 * @brief Request XM125 to log its configuration to UART.
 * @param sat_idx Satellite index [0..active_count-1].
 * @note Sends LOG_CONFIGURATION command to XM125.
 */
void xm125_log_configuration(uint8_t sat_idx);

#endif // XM125_DEBUG_ERRORS

#endif // XM125_ASYNC_H
