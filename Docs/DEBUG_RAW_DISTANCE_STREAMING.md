# Implementation Guide: Radar Raw Distance Streaming

## 1. Executive Summary

**Objective:** Output real-time radar target data (distance and strength) with per-satellite and global void detection states via the UART debug console for diagnostics.

---

## 2. Context & Reasoning

The device makes decisions based on radar data (e.g., "Is there a void?"). To debug why the device made a specific decision, we need visibility into the raw data the algorithm is seeing.

---

## 3. Architecture Overview

- **Control:** The `h_dev_debug` union in [`Device/Inc/vmt_device.h`](../Device/Inc/vmt_device.h) holds bit-flags (e.g., `b_void_sample`) that enable different debug streams.
- **Process:** The function `dev_debug_process()` in [`Device/Src/vmt_device.c`](../Device/Src/vmt_device.c) runs in the main loop.
- **Throttle:** Output is rate-limited to 1Hz using `HAL_GetTick()`.

---

## 4. Implementation Details

### Control Flag

```c
typedef union h_dev_debug_
{
    uint8_t byte;
    struct
    {
        bool b_init           : 1;
        bool b_spi_init       : 1;
        bool b_imu_sample_set : 1;
        bool b_adc_sample     : 1;
        bool b_imu_sample     : 1;
        bool b_void_sample    : 1; // Bit 5 (0x20)
    };
} h_dev_debug_t;
```

### Streaming Logic

- In `dev_debug_process()`, if `h_dev_debug.b_void_sample` is set, the system prints a line every second with:
    - Global void state (`Vd:Y` or `Vd:N`)
    - Per-satellite void state (`V` or `N`)
    - Per-satellite filtered radar targets as `[distance@strength,...]`
- The code currently iterates over satellites 1–3 (not 1–6).

**Example code:**
```c
if (h_dev_debug.b_void_sample)
{
    static uint32_t void_log_time = 0;
    uint32_t time_now = HAL_GetTick();

    if (time_now - void_log_time >= 1000)
    {
        void_log_time = time_now;
        printf("> Vd:%s", mti_void_is_detected() ? "Y" : "N");

        for (uint8_t sat_id = 1; sat_id <= 3; sat_id++)
        {
            bool sat_void = mti_void_get_satellite_state(sat_id);
            mti_radar_measurement_t meas;
            bool has_data = mti_radar_get_measurement(sat_id, &meas);
            printf(" %u:%s", sat_id, sat_void ? "V" : "N");

            if (has_data && meas.num_targets > 0)
            {
                printf("[");
                for (uint8_t i = 0; i < meas.num_targets; i++)
                {
                    if (i > 0)
                        printf(",");
                    printf("%lu@%u", meas.targets[i].distance_mm, meas.targets[i].strength);
                }
                printf("]");
            }
        }
        printf("\r\n");
    }
}
```

---

## 5. Usage & Verification

### Enabling the Feature

- Use the UART command: `@cmd,debug,dev,20` to enable (`b_void_sample` = 0x20).
- Use `@cmd,debug,dev,00` to disable.

### Output Format

```
> Vd:Y 1:V[1500@110,2200@65] 2:N[800@95] 3:N
```

- `Vd:Y` or `Vd:N` = Global void detection state
- `1:V` = Satellite 1 is detecting void
- `[1500@110,2200@65]` = List of targets (distance in mm @ strength)
- If no targets: just `N` or `V` with no brackets

---

## 6. Integration Notes

- Only satellites 1–3 are printed in the current implementation.
- Data is updated once per second.
- The debug output shows the data that led to the current void state (may be slightly behind real-time).

---

## 7. Debugging Scenarios

- **False Positive:** Check for long-range, low-strength targets.
- **False Negative:** All satellites show only close-range targets.
- **Intermittent:** Distances hover near the threshold; increase hysteresis or debounce.

---

## 8. Summary

- Real-time visibility into void detection logic
- Per-satellite diagnostics
- Minimal CPU overhead
- No breaking changes to existing functionality
