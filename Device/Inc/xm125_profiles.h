/**
 * @file xm125_profiles.h
 * @brief Externally defined detector configuration profiles for XM125 radar.
 *
 * Provides statically allocated, application-selectable configuration profiles.
 * All profiles guarantee valid parameter ranges for XM125 operation.
 * Pointers remain valid for program duration; no ownership or lifetime management required.
 */

#ifndef XM125_PROFILES_H
#define XM125_PROFILES_H

#include <stdint.h>
#include <stdbool.h>
#include "xm125_async.h"

// =============================================================================
// PUBLIC API - Profile Management
// =============================================================================

/**
 * @brief Set the active detector profile using enum.
 * @param profile Profile ID from xm125_profile_id_t enum.
 * @return true if profile was set, false if invalid.
 *
 * @note This function directly wraps xm125_set_profile() from the HAL driver.
 *       Configuration is applied asynchronously; check status with
 *       xm125_profile_get_config_status().
 *
 * @example
 *   xm125_profile_set(XM_PROFILE_BALANCED);
 *   while (xm125_profile_get_config_status() != XM125_CONFIG_OK) {
 *       xm125_cluster_process();
 *   }
 */
void xm125_profile_set(xm125_profile_id_t profile);

/**
 * @brief Get the currently active profile.
 * @return Profile ID enum, or 0 if not set.
 */
xm125_profile_id_t xm125_profile_get(void);

/**
 * @brief Get profile name as string (for debugging/logging).
 * @param profile Profile ID enum.
 * @return Pointer to constant string name, or "UNKNOWN" if invalid.
 */
const char *xm125_profile_get_name(xm125_profile_id_t profile);

/**
 * @brief Get pointer to profile configuration structure.
 * @param profile Profile ID enum.
 * @return Pointer to constant configuration, or NULL if invalid.
 */
const detector_config_t *xm125_profile_get_config(xm125_profile_id_t profile);

/**
 * @brief Get the last configuration status from the driver.
 * @return Status code: OK, PENDING, or ERROR.
 */
xm125_config_status_t xm125_profile_get_config_status(void);

/**
 * @brief Get the last configuration error code from the driver.
 * @return Bitmask of error flags from DETECTOR_STATUS register.
 */
uint32_t xm125_profile_get_config_error(void);

/**
 * @brief Get a pointer to the last confirmed configuration.
 * @return Pointer to RAM copy of active configuration.
 */
const detector_config_t *xm125_profile_get_active_config(void);

// --- Externally defined profiles ---

/**
 * @brief Balanced profile for general-purpose MVP detection.
 * @details Suitable for most environments; optimized for signal quality and thresholding.
 */
extern const detector_config_t config_balanced_mvp;

/**
 * @brief High-speed profile for rapid detection scenarios.
 * @details Reduces signal quality threshold for faster response; may increase noise sensitivity.
 */
extern const detector_config_t config_high_speed;

/**
 * @brief High-resolution profile for precise detection.
 * @details Maximizes signal quality for best accuracy; may reduce speed.
 */
extern const detector_config_t config_high_resolution;

/**
 * @brief Calibrated profile for close-range detection.
 * @details Enables close-range leakage compensation; recommended for short distance applications.
 */
extern const detector_config_t config_close_range_calibrated;

// =============================================================================
// DEBUG HELPERS (CONDITIONAL)
// =============================================================================

#ifdef XM125_PROFILES_DEBUG

/**
 * @brief Print profile configuration to debug console.
 * @param profile Profile ID to print.
 *
 * @note Requires stdio.h and printf support.
 */
void xm125_profile_debug_print(xm125_profile_id_t profile);

#endif // XM125_PROFILES_DEBUG

#endif // XM125_PROFILES_H
