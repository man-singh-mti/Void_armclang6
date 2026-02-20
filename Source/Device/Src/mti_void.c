#include "mti_void.h"
#include "mti_radar.h"
#include "vmt_uart.h"

static uint32_t cfg_threshold_mm   = MTI_VOID_DEFAULT_THRESHOLD_MM;
static uint32_t cfg_hole_radius_mm = MTI_VOID_DEFAULT_HOLE_DIA_MM / 2; /* Store as radius */
static uint8_t  cfg_hysteresis_pct = MTI_VOID_DEFAULT_HYSTERESIS_PCT;
static uint8_t  cfg_debounce_enter = 3u;
static uint8_t  cfg_debounce_exit  = 5u;

#define PROBE_RADIUS_MM (MTI_VOID_PROBE_DIA_MM / 2) /* 30mm probe radius */

static bool    global_void_active                        = false;
static bool    sat_void_state[MTI_RADAR_MAX_SATELLITES]  = { 0 };
static uint8_t sat_enter_count[MTI_RADAR_MAX_SATELLITES] = { 0 };
static uint8_t sat_exit_count[MTI_RADAR_MAX_SATELLITES]  = { 0 };

static bool apply_hysteresis(bool current_state, uint32_t distance_mm)
{
    if (!current_state)
    {
        if (distance_mm > cfg_threshold_mm)
        {
            return true;
        }
    }
    else
    {
        uint32_t exit_threshold = cfg_threshold_mm - ((cfg_threshold_mm * cfg_hysteresis_pct) / 100u);
        if (distance_mm < exit_threshold)
        {
            return false;
        }
    }
    return current_state;
}

void mti_void_init(void)
{
    global_void_active = false;
    for (uint8_t i = 0; i < MTI_RADAR_MAX_SATELLITES; i++)
    {
        sat_void_state[i]  = false;
        sat_enter_count[i] = 0u;
        sat_exit_count[i]  = 0u;
    }
}

void mti_void_set_threshold_mm(uint32_t threshold_mm)
{
    cfg_threshold_mm = threshold_mm;
}

uint32_t mti_void_get_threshold_mm(void)
{
    return cfg_threshold_mm;
}

bool mti_void_is_detected(void)
{
    return global_void_active;
}

void mti_void_set_hysteresis_pct(uint8_t pct)
{
    if (pct > 100u)
        pct = 100u;
    cfg_hysteresis_pct = pct;
}

uint8_t mti_void_get_hysteresis_pct(void)
{
    return cfg_hysteresis_pct;
}

void mti_void_set_debounce_frames(uint8_t enter_frames, uint8_t exit_frames)
{
    cfg_debounce_enter = enter_frames;
    cfg_debounce_exit  = exit_frames;
}

uint8_t mti_void_get_debounce_enter_frames(void)
{
    return cfg_debounce_enter;
}

uint8_t mti_void_get_debounce_exit_frames(void)
{
    return cfg_debounce_exit;
}

bool mti_void_get_satellite_state(uint8_t sat_id)
{
    if (sat_id < 1 || sat_id > MTI_RADAR_MAX_SATELLITES)
    {
        return false;
    }
    return sat_void_state[sat_id - 1];
}

void mti_void_set_hole_diameter_mm(uint32_t diameter_mm)
{
    cfg_hole_radius_mm = diameter_mm / 2;
}

uint32_t mti_void_get_hole_diameter_mm(void)
{
    return cfg_hole_radius_mm * 2;
}

void mti_void_process(void)
{
    for (uint8_t sat_id = 1; sat_id <= MTI_RADAR_MAX_SATELLITES; sat_id++)
    {
        mti_radar_measurement_t meas;
        if (!mti_radar_get_measurement(sat_id, &meas))
        {
            continue;
        }

        uint8_t  sat_idx     = sat_id - 1;
        uint32_t calc_radius = 0;

        // Use the FIRST valid target (already sorted by XM125 as closest/strongest)
        // XM125 profiles use PEAK_SORTING_CLOSEST, so first target is nearest valid reflection
        if (meas.num_targets > 0 && meas.targets[0].valid)
        {
            uint32_t d = meas.targets[0].distance_mm;

            // FAIL-SAFE CALCULATION:
            // Assume probe is at NOMINAL WALL position (worst case for detection).
            // If probe is at +150mm (nominal) and measures 400mm, void is at 150+400=550mm from center.
            // This handles the critical case: probe at nominal wall, void on same side.
            // Formula: void_radius = hole_radius + measured_distance
            calc_radius = cfg_hole_radius_mm + d;
        }

        // Apply hysteresis to the calculated worst-case radius
        bool target = apply_hysteresis(sat_void_state[sat_idx], calc_radius);

        if (target)
        {
            sat_enter_count[sat_idx]++;
            sat_exit_count[sat_idx] = 0u;
            if (!sat_void_state[sat_idx] && sat_enter_count[sat_idx] >= cfg_debounce_enter)
            {
                sat_void_state[sat_idx]  = true;
                sat_enter_count[sat_idx] = 0u;
                sat_exit_count[sat_idx]  = 0u;
            }
        }
        else
        {
            sat_exit_count[sat_idx]  = (uint8_t)(sat_exit_count[sat_idx] + 1u);
            sat_enter_count[sat_idx] = 0u;
            if (sat_void_state[sat_idx] && sat_exit_count[sat_idx] >= cfg_debounce_exit)
            {
                sat_void_state[sat_idx]  = false;
                sat_enter_count[sat_idx] = 0u;
                sat_exit_count[sat_idx]  = 0u;
            }
        }
    }

    bool agg = false;
    for (uint8_t i = 0; i < MTI_RADAR_MAX_SATELLITES; i++)
    {
        if (sat_void_state[i])
        {
            agg = true;
            break;
        }
    }

    if (agg != global_void_active)
    {
        global_void_active = agg;
        uphole_send("!void,%d", global_void_active ? 1 : 0);
        debug_send("@db,Void State Changed: %d", global_void_active);
    }
}
