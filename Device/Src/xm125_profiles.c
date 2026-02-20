/**
 * @file xm125_profiles.c
 * @brief Statically allocated detector configuration profiles for XM125 radar.
 * @details
 *   - Provides externally visible constant profiles for runtime selection.
 *   - No ownership or lifetime management required; pointers remain valid.
 *   - All parameters guarantee valid ranges for XM125 operation.
 *   - Clean wrapper API over xm125_async HAL driver.
 */
#include "xm125_async.h"
#include "xm125_profiles.h"
#include <stddef.h>

// =============================================================================
// STATIC STATE
// =============================================================================

/** @brief Currently active profile (0 = not set). */
static xm125_profile_id_t current_profile = (xm125_profile_id_t)0;

// =============================================================================
// PROFILE NAME LOOKUP TABLE
// =============================================================================

/** @brief Profile name strings for debugging. */
static const char *profile_names[] = { "NONE", "BALANCED", "HIGH_SPEED", "HIGH_RESOLUTION", "CLOSE_RANGE" };

// =============================================================================
// PUBLIC API IMPLEMENTATION
// =============================================================================

void xm125_profile_set(xm125_profile_id_t profile)
{
    // Validate profile ID
    if (profile < XM_PROFILE_BALANCED || profile > XM_PROFILE_CLOSE_RANGE)
    {
        return;
    }

    // Store current profile
    current_profile = profile;

    // Apply to HAL driver
    xm125_set_profile(profile);
}

xm125_profile_id_t xm125_profile_get(void)
{
    return current_profile;
}

const char *xm125_profile_get_name(xm125_profile_id_t profile)
{
    if (profile < XM_PROFILE_BALANCED || profile > XM_PROFILE_CLOSE_RANGE)
    {
        return "UNKNOWN";
    }

    return profile_names[profile];
}

const detector_config_t *xm125_profile_get_config(xm125_profile_id_t profile)
{
    switch (profile)
    {
    case XM_PROFILE_BALANCED:
        return &config_balanced_mvp;
    case XM_PROFILE_HIGH_SPEED:
        return &config_high_speed;
    case XM_PROFILE_HIGH_RES:
        return &config_high_resolution;
    case XM_PROFILE_CLOSE_RANGE:
        return &config_close_range_calibrated;
    default:
        return NULL;
    }
}

xm125_config_status_t xm125_profile_get_config_status(void)
{
    return xm125_get_config_status();
}

uint32_t xm125_profile_get_config_error(void)
{
    return xm125_get_config_error();
}

const detector_config_t *xm125_profile_get_active_config(void)
{
    return xm125_get_active_config();
}

// =============================================================================
// PROFILE DEFINITIONS
// =============================================================================

/**
 * @brief Balanced MVP Profile
 *
 * Range: 100-1500mm
 * Profile: 2 (medium step, good balance)
 * Signal Quality: 10.0 (moderate filtering)
 * Method: CFAR (adaptive thresholding)
 * Sorting: Closest target first
 *
 * Best for: General-purpose void detection in most environments.
 */
const detector_config_t config_balanced_mvp = { .start_mm              = 100,
                                                .end_mm                = 1500,
                                                .max_step_length       = 0, // Auto-select
                                                .max_profile           = XM125_MAX_PROFILE_PROFILE2,
                                                .close_range_leakage   = 0, // Disabled
                                                .signal_quality        = 10000,
                                                .threshold_method      = XM125_THRESHOLD_METHOD_CFAR,
                                                .peak_sorting_method   = XM125_PEAK_SORTING_CLOSEST,
                                                .reflector_shape       = XM125_REFLECTOR_SHAPE_GENERIC,
                                                .num_frames_rec_thresh = 100,
                                                .fixed_amp_thresh_val  = 100000,
                                                .fixed_str_thresh_val  = 0,
                                                .threshold_sensitivity = 500 };

/**
 * @brief High-Speed Profile
 *
 * Range: 100-1500mm
 * Profile: 3 (larger steps, faster)
 * Signal Quality: 5.0 (less filtering)
 * Method: CFAR (adaptive thresholding)
 * Sorting: Closest target first
 *
 * Best for: Fast-moving drill string, rapid measurements needed.
 * Trade-off: May increase noise sensitivity.
 */
const detector_config_t config_high_speed = { .start_mm              = 100,
                                              .end_mm                = 1500,
                                              .max_step_length       = 0, // Auto-select
                                              .max_profile           = XM125_MAX_PROFILE_PROFILE3,
                                              .close_range_leakage   = 0,    // Disabled
                                              .signal_quality        = 5000, // Reduced for speed
                                              .threshold_method      = XM125_THRESHOLD_METHOD_CFAR,
                                              .peak_sorting_method   = XM125_PEAK_SORTING_CLOSEST,
                                              .reflector_shape       = XM125_REFLECTOR_SHAPE_GENERIC,
                                              .num_frames_rec_thresh = 100,
                                              .fixed_amp_thresh_val  = 100000,
                                              .fixed_str_thresh_val  = 0,
                                              .threshold_sensitivity = 500 };

/**
 * @brief High-Resolution Profile
 *
 * Range: 100-1500mm
 * Profile: 1 (smallest steps, highest resolution)
 * Signal Quality: 15.0 (maximum filtering)
 * Method: CFAR (adaptive thresholding)
 * Sorting: Closest target first
 *
 * Best for: Precise void boundary detection, slow/stationary drill string.
 * Trade-off: Slower measurement rate.
 */
const detector_config_t config_high_resolution = { .start_mm              = 100,
                                                   .end_mm                = 1500,
                                                   .max_step_length       = 0, // Auto-select
                                                   .max_profile           = XM125_MAX_PROFILE_PROFILE1,
                                                   .close_range_leakage   = 0,     // Disabled
                                                   .signal_quality        = 15000, // Maximized for accuracy
                                                   .threshold_method      = XM125_THRESHOLD_METHOD_CFAR,
                                                   .peak_sorting_method   = XM125_PEAK_SORTING_CLOSEST,
                                                   .reflector_shape       = XM125_REFLECTOR_SHAPE_GENERIC,
                                                   .num_frames_rec_thresh = 100,
                                                   .fixed_amp_thresh_val  = 100000,
                                                   .fixed_str_thresh_val  = 0,
                                                   .threshold_sensitivity = 500 };

/**
 * @brief Close-Range Calibrated Profile
 *
 * Range: 50-1500mm (extended down to 50mm)
 * Profile: 2 (medium step)
 * Signal Quality: 10.0 (moderate filtering)
 * Method: CFAR (adaptive thresholding)
 * Sorting: Closest target first
 * Close-Range Leakage: ENABLED
 *
 * Best for: Detecting voids very close to sensor (50-100mm).
 * Note: Requires close-range leakage calibration.
 */
const detector_config_t config_close_range_calibrated = { .start_mm              = 50, // Extended close range
                                                          .end_mm                = 1500,
                                                          .max_step_length       = 0, // Auto-select
                                                          .max_profile           = XM125_MAX_PROFILE_PROFILE2,
                                                          .close_range_leakage   = 1, // ENABLED for close detection
                                                          .signal_quality        = 10000,
                                                          .threshold_method      = XM125_THRESHOLD_METHOD_CFAR,
                                                          .peak_sorting_method   = XM125_PEAK_SORTING_CLOSEST,
                                                          .reflector_shape       = XM125_REFLECTOR_SHAPE_GENERIC,
                                                          .num_frames_rec_thresh = 100,
                                                          .fixed_amp_thresh_val  = 100000,
                                                          .fixed_str_thresh_val  = 0,
                                                          .threshold_sensitivity = 500 };

// =============================================================================
// DEBUG HELPERS (CONDITIONAL)
// =============================================================================

#ifdef XM125_PROFILES_DEBUG

#include <stdio.h>

void xm125_profile_debug_print(xm125_profile_id_t profile)
{
    const detector_config_t *cfg = xm125_profile_get_config(profile);
    if (cfg == NULL)
    {
        printf("Profile %d: INVALID\r\n", profile);
        return;
    }

    printf("=== Profile: %s ===\r\n", xm125_profile_get_name(profile));
    printf("  Range: %lu - %lu mm\r\n", cfg->start_mm, cfg->end_mm);
    printf("  Max Step Length: %lu\r\n", cfg->max_step_length);
    printf("  Max Profile: %lu\r\n", cfg->max_profile);
    printf("  Close Range Leakage: %s\r\n", cfg->close_range_leakage ? "Enabled" : "Disabled");
    printf("  Signal Quality: %lu\r\n", cfg->signal_quality);
    printf("  Threshold Method: %lu\r\n", cfg->threshold_method);
    printf("  Peak Sorting: %lu\r\n", cfg->peak_sorting_method);
    printf("  Reflector Shape: %lu\r\n", cfg->reflector_shape);
    printf("  Num Frames Recorded: %lu\r\n", cfg->num_frames_rec_thresh);
    printf("  Fixed Amplitude Threshold: %lu\r\n", cfg->fixed_amp_thresh_val);
    printf("  Fixed Strength Threshold: %u\r\n", cfg->fixed_str_thresh_val);
    printf("  Threshold Sensitivity: %lu\r\n", cfg->threshold_sensitivity);
    printf("\r\n");
}

#endif // XM125_PROFILES_DEBUG
