# XM125 Radar Driver - Future Enhancement Ideas

This document captures potential enhancements and alternative implementations for the XM125 radar driver. These features are **not required for MVP** but may be valuable for future iterations, optimizations, or specific use cases.

---

## Table of Contents

1. [Automatic Hardware Detection](#1-automatic-hardware-detection)
2. [Measurement Strategy Options](#2-measurement-strategy-options)
3. [Power Optimization](#3-power-optimization)
4. [Advanced Error Handling](#4-advanced-error-handling)
5. [Performance Profiling](#5-performance-profiling)
6. [Configuration Management](#6-configuration-management)
7. [Multi-Target Tracking](#7-multi-target-tracking)
8. [Data Filtering & Processing](#8-data-filtering--processing)
9. [Future Hardware Variations](#9-future-hardware-variations)
10. [Integration Enhancements](#10-integration-enhancements)

---

## 1. Automatic Hardware Detection

### Current Implementation

- Manual configuration via `banks_mask` parameter
- User specifies which banks are connected (0x01=Bank1, 0x02=Bank2, 0x03=Both)
- Simple, deterministic, fast initialization

### Enhancement Idea: Auto-Detection

**Objective:** Automatically detect which I²C banks have connected satellites.

**Implementation Approach:**

```c
/**
 * @brief Scan I2C banks and detect connected satellites.
 * @return Bitmask of detected banks (0x01=Bank1, 0x02=Bank2, 0x03=Both).
 */
uint8_t xm125_detect_connected_banks(void)
{
    uint8_t detected = 0;

    // Scan Bank1 (try PCA9534 addresses 0x21-0x23)
    for (uint8_t i = 0; i < 3; i++)
    {
        uint8_t addr = 0x21 + i;
        uint8_t dummy;

        if (pca9534_read_reg_async(VMT_I2C_BANK1, addr, PCA9534_REG_INPUT, &dummy) == HAL_OK)
        {
            // Wait for completion
            while (!vmt_i2c_async_is_idle(VMT_I2C_BANK1))
                vmt_i2c_async_process();

            if (vmt_i2c_async_get_last_status(VMT_I2C_BANK1) == HAL_OK)
            {
                detected |= 0x01;
                break;
            }
        }
    }

    // Scan Bank2 (same logic)
    for (uint8_t i = 0; i < 3; i++)
    {
        uint8_t addr = 0x21 + i;
        uint8_t dummy;

        if (pca9534_read_reg_async(VMT_I2C_BANK2, addr, PCA9534_REG_INPUT, &dummy) == HAL_OK)
        {
            while (!vmt_i2c_async_is_idle(VMT_I2C_BANK2))
                vmt_i2c_async_process();

            if (vmt_i2c_async_get_last_status(VMT_I2C_BANK2) == HAL_OK)
            {
                detected |= 0x02;
                break;
            }
        }
    }

    return detected;
}
```

**Usage Example:**

```c
// Auto-detect and initialize
uint8_t banks = xm125_detect_connected_banks();
if (banks == 0)
{
    printf("ERROR: No satellites detected!\n");
    return;
}

mti_radar_init(banks);
printf("Detected banks: 0x%02X\n", banks);
```

**Benefits:**

- ✅ Robust to partial failures (one bank disconnected)
- ✅ Simplifies field deployment (no manual configuration)
- ✅ Useful for diagnostics (verify hardware connectivity)
- ✅ Handles hardware variations automatically

**Drawbacks:**

- ❌ Adds ~50-100ms startup latency (I²C probing time)
- ❌ Could false-positive if bus has shorts/noise
- ❌ More complex initialization sequence

**Priority:** Low (manual config works fine for MVP)

**When to Implement:**

- Field deployments where configuration is uncertain
- Systems with hot-swappable sensor modules
- Diagnostic/test modes requiring hardware verification

---

## 2. Measurement Strategy Options

### Current Implementation: Continuous Polling

**How it Works:**

1. Keep XM125 awake (WAKE_UP pin HIGH)
2. Issue `MEASURE_DISTANCE` command via I²C
3. Poll `DETECTOR_STATUS` register until `BUSY` bit clears
4. Read measurement results
5. Repeat

**State Machine Flow:**

```ascii
IDLE → MEASURE_CMD → MEASURE_POLL → READ_RESULT → READ_PEAKS → IDLE
```

**Performance:**

- Measurement Rate: ~100Hz per satellite
- Power Consumption: ~100mW per satellite (always awake)
- I²C Transactions per Measurement: 5-7
- Latency (trigger→data): <10ms

### Enhancement Idea: Measure-on-Wakeup Mode

**How it Works:**

1. Enable `MEASURE_ON_WAKEUP` register in XM125
2. Sequence: WAKE_UP LOW → WAKE_UP HIGH → Wait MCU_INT → Read Results → WAKE_UP LOW
3. XM125 automatically measures on wake (no explicit command needed)

**Hardware Reference:**
> From Hardware.md: "When Measure on Wake Up is enabled, the module will perform a distance measurement every time it is woken by the WAKE UP pin."

**State Machine Flow:**

```ascii
IDLE → SLEEP → WAKE_ASSERT → WAKE_POLL (MCU_INT) → READ_RESULT → READ_PEAKS → IDLE
```

**Implementation Approach:**

```c
typedef enum {
    XM125_MODE_CONTINUOUS = 0,  /**< Keep awake, fast polling (current) */
    XM125_MODE_LOW_POWER  = 1,  /**< Sleep/wake, measure-on-wakeup */
} xm125_measurement_mode_t;

void xm125_set_measurement_mode(xm125_measurement_mode_t mode)
{
    if (mode == XM125_MODE_LOW_POWER)
    {
        // Enable measure-on-wakeup for all satellites
        for (uint8_t i = 0; i < xm_ctx.active_count; i++)
        {
            xm125_satellite_t *sat = &xm_ctx.sats[i];
            xm_write_u32(sat->bank, sat->addr7,
                         XM125_REG_MEASURE_ON_WAKEUP_ADDR, 1);
        }
        xm_ctx.measurement_mode = XM125_MODE_LOW_POWER;
    }
    else
    {
        // Disable measure-on-wakeup
        for (uint8_t i = 0; i < xm_ctx.active_count; i++)
        {
            xm125_satellite_t *sat = &xm_ctx.sats[i];
            xm_write_u32(sat->bank, sat->addr7,
                         XM125_REG_MEASURE_ON_WAKEUP_ADDR, 0);
        }
        xm_ctx.measurement_mode = XM125_MODE_CONTINUOUS;
    }
}
```

**State Machine Modifications:**

```c
case XM125_SAT_IDLE:
    if (xm_ctx.measurement_mode == XM125_MODE_LOW_POWER)
    {
        // Put to sleep
        status = pca9534_xm125_sleep(sat->bank, sat->addr_pca);
        if (status == HAL_OK)
        {
            sat->state = XM125_SAT_SLEEP;
            sat->state_start_tick = HAL_GetTick();
        }
    }
    else if (xm_ctx.running)
    {
        // Continuous mode: issue measurement command
        sat->state = XM125_SAT_MEASURE_CMD;
        sat->state_start_tick = HAL_GetTick();
    }
    break;

case XM125_SAT_SLEEP:
    // Wait for wake trigger (timer, interrupt, external event)
    if (wake_trigger_asserted)
    {
        // Wake XM125 (will auto-measure)
        status = pca9534_xm125_run(sat->bank, sat->addr_pca);
        if (status == HAL_OK)
        {
            sat->state = XM125_SAT_WAKE_POLL;
            sat->state_start_tick = HAL_GetTick();
        }
    }
    break;
```

### Performance Comparison

| Metric                       | Continuous Polling (Current) | Measure-on-Wakeup   |
|------------------------------|------------------------------|---------------------|
| **Measurement Rate**         | ~100Hz                       | ~50Hz               |
| **Power Consumption**        | ~100mW/satellite             | ~10mW/satellite     |
| **I²C Transactions/Meas**    | 5-7                          | 2-3                 |
| **Latency (trigger→data)**   | <10ms                        | 20-30ms             |
| **State Machine Complexity** | Medium                       | Low                 |
| **Responsiveness**           | High (always ready)          | Medium (wake delay) |
| **Burst Measurements**       | Easy (no power cycling)      | Requires wake/sleep |

### Recommendation

**For Void Probe MVP:**

- ✅ **Use Continuous Polling** (current implementation)
  - Justification: Faster response, better for real-time void detection
  - Power is less critical (wired system, not battery-powered)
  - Measurement rate important for catching fast-moving voids

**Future Use Cases for Measure-on-Wakeup:**

- Battery-powered deployments
- Low-traffic monitoring (infrequent measurements)
- Power-sensitive applications
- Scheduled/event-driven measurements

**Priority:** Medium (wait for hardware testing data)

**When to Implement:**

- After validating power consumption on hardware
- If battery operation becomes a requirement
- For variants with infrequent measurement needs

---

## 3. Power Optimization

### Potential Enhancements

#### 3.1 Dynamic Clock Scaling

- Reduce I²C clock speed during idle periods
- Increase clock during burst operations

#### 3.2 Selective Bank Activation

- Power down unused banks dynamically
- Wake only when measurement needed

#### 3.3 Adaptive Measurement Rate

- Reduce rate when no targets detected
- Increase rate when targets present

#### 3.4 Sleep/Wake Scheduling

- Time-based measurement windows
- Event-triggered wake-up

**Priority:** Low (power not critical for MVP)

---

## 4. Advanced Error Handling

### Current Error Recovery Strategy

- 3-tier recovery: Re-init → RESET_MODULE → OFFLINE
- Consecutive error counter
- Automatic recovery attempts

### Enhancement Ideas

#### 4.1 Intelligent Error Classification

```c
typedef enum {
    XM125_ERROR_I2C_TIMEOUT,
    XM125_ERROR_I2C_NACK,
    XM125_ERROR_CONFIG_FAILED,
    XM125_ERROR_CALIBRATION_FAILED,
    XM125_ERROR_MEASUREMENT_FAILED,
    XM125_ERROR_PROTOCOL_ERROR,
} xm125_error_type_t;

void xm125_record_error(uint8_t sat_idx, xm125_error_type_t error);
```

#### 4.2 Error Statistics & Logging

- Track error types and frequencies
- Export error history for diagnostics
- Predictive maintenance alerts

#### 4.3 Graceful Degradation

- Continue operation with reduced satellite count
- Fallback to redundant satellites

#### 4.4 Watchdog Integration

- Hardware watchdog for critical failures
- Automatic system reset on total failure

**Priority:** Medium (useful for production systems)

---

## 5. Performance Profiling

### Profiling & Instrumentation Ideas

#### 5.1 Timing Metrics

```c
typedef struct {
    uint32_t state_machine_us;      /**< Time spent in state machine */
    uint32_t i2c_transaction_us;    /**< I²C transaction time */
    uint32_t measurement_cycle_us;  /**< Full measurement cycle time */
    uint32_t max_jitter_us;         /**< Maximum timing jitter */
} xm125_performance_stats_t;
```

#### 5.2 Bandwidth Monitoring

- Track I²C bytes/second per bank
- Identify bottlenecks

#### 5.3 State Machine Profiling

- Time spent in each state
- Identify slow states

**Priority:** Low (useful for optimization)

---

## 6. Configuration Management

### Configuration Enhancement Ideas

#### 6.1 Runtime Configuration Changes

- Modify configuration without full restart
- Apply config to subset of satellites

#### 6.2 Configuration Profiles Library

- Expand profile set for specific use cases
- User-defined custom profiles

#### 6.3 Configuration Validation

- Pre-validate settings before applying
- Warn about invalid combinations

#### 6.4 A/B Configuration Testing

- Run different configs on different satellites
- Compare performance metrics

**Priority:** Low (current profiles sufficient)

---

## 7. Multi-Target Tracking

### Tracking Enhancement Ideas

#### 7.1 Target ID Assignment

- Track individual targets across measurements
- Assign persistent IDs to moving targets

#### 7.2 Velocity Estimation

- Calculate target velocity from position history
- Predict future positions

#### 7.3 Target Filtering

- Remove noise/false targets
- Apply Kalman filtering for smoothing

#### 7.4 Multi-Satellite Fusion

- Combine measurements from multiple satellites
- Improve accuracy through sensor fusion

**Priority:** Medium (useful for advanced applications)

---

## 8. Data Filtering & Processing

### Filtering & Processing Ideas

#### 8.1 Moving Average Filter

```c
typedef struct {
    uint32_t history[MTI_RADAR_HISTORY_DEPTH];
    uint8_t index;
    uint8_t count;
} moving_average_t;

uint32_t apply_moving_average(moving_average_t *ma, uint32_t new_value);
```

#### 8.2 Outlier Rejection

- Detect and remove spurious measurements
- Statistical outlier detection

#### 8.3 Signal Quality Metrics

- SNR estimation
- Measurement confidence scoring

#### 8.4 Data Compression

- Compress measurement history
- Reduce memory footprint

**Priority:** Medium (depends on application requirements)

---

## 9. Future Hardware Variations

### Potential Hardware Changes

#### 9.1 Different XM125 Variants

- Support XM122, XM132 modules
- Adapt to different I²C addresses

#### 9.2 Alternative I/O Expanders

- Support PCA9555 (16-bit)
- Support other GPIO expanders

#### 9.3 SPI Interface Option

- Parallel development for SPI-based systems
- Higher bandwidth alternative

#### 9.4 Wireless Integration

- Add wireless module support
- Remote satellite monitoring

**Priority:** Low (hardware stable for MVP)

---

## 10. Integration Enhancements

### System Integration Ideas

#### 10.1 RTOS Integration

- FreeRTOS task-based architecture
- Semaphore-based synchronization

#### 10.2 Logging Framework

- Structured logging (JSON/CSV)
- Log levels (DEBUG, INFO, WARN, ERROR)
- Remote logging via UART/network

#### 10.3 Diagnostic Web Interface

- HTTP server for status monitoring
- Real-time data visualization
- Configuration via web UI

#### 10.4 CAN Bus Integration

- Broadcast measurements on CAN
- Support for automotive protocols

#### 10.5 Over-the-Air Updates

- Firmware update capability
- Configuration update over network

**Priority:** Low to Medium (depends on system architecture)

---

## Implementation Priority Matrix

| Enhancement              | Priority | Complexity | Value  | Recommended Phase |
|--------------------------|----------|------------|--------|-------------------|
| Auto-Detection           | Low      | Low        | Medium | Post-MVP          |
| Measure-on-Wakeup Mode   | Medium   | Medium     | High   | Phase 2           |
| Power Optimization       | Low      | Medium     | Low    | Future            |
| Error Classification     | Medium   | Low        | High   | Phase 2           |
| Performance Profiling    | Low      | Low        | Medium | Post-MVP          |
| Configuration Management | Low      | Low        | Low    | Future            |
| Multi-Target Tracking    | Medium   | High       | High   | Phase 3           |
| Data Filtering           | Medium   | Medium     | High   | Phase 2           |
| Hardware Variations      | Low      | High       | Low    | Future            |
| RTOS Integration         | Medium   | Medium     | Medium | Phase 2           |
| Logging Framework        | Medium   | Low        | High   | Phase 2           |
| Diagnostic Web Interface | Low      | High       | Medium | Future            |

---

## Decision Guidelines

### When to Implement an Enhancement

**Implement Now:**

- Blocks critical functionality
- Required by customer/specification
- Low effort, high value

**Implement After MVP:**

- Nice-to-have, not essential
- Requires hardware validation first
- Dependencies on MVP completion

**Implement Later:**

- Optimization without proven need
- High complexity, uncertain value
- Driven by future requirements

### Evaluation Criteria

1. **Does it solve a real problem?** (not theoretical)
2. **Is the problem validated on hardware?** (not premature)
3. **What's the effort vs. value ratio?** (ROI)
4. **Does it add risk to current system?** (stability)
5. **Is there customer demand?** (priority)

---

## Notes & Observations

### Key Learnings

- **Current continuous polling is optimal for MVP use case** (wired system, real-time requirements)
- **Manual bank configuration is sufficient** (simple, reliable, fast)
- **Power optimization can wait** until battery operation becomes a requirement
- **Don't over-engineer** before validating on hardware

### Open Questions

- What is actual power consumption on target hardware?
- What measurement rate is truly needed for void detection?
- Are measurement gaps acceptable? (define tolerance)
- Will future variants require battery operation?

### Deferred Features

The following features were considered but **explicitly deferred**:

- Automatic bank detection (manual config works)
- Measure-on-wakeup mode (continuous polling sufficient)
- Advanced filtering (apply in business logic layer if needed)
- RTOS integration (bare-metal adequate for MVP)

---

## Revision History

| Date       | Author | Changes                                  |
|------------|--------|------------------------------------------|
| 2026-01-30 | System | Initial document created                 |
|            |        | Documented auto-detection idea           |
|            |        | Documented measure-on-wakeup alternative |
|            |        | Added 10 enhancement categories          |
|            |        | Added priority matrix and decision guide |

---

## Contributing

To add a new enhancement idea:

1. Choose appropriate section (or add new section)
2. Describe current implementation (if applicable)
3. Describe enhancement objective
4. Provide implementation approach (code examples encouraged)
5. List benefits and drawbacks
6. Assign priority (Low/Medium/High)
7. Define "when to implement" criteria
8. Update priority matrix if needed

**Keep ideas grounded in real requirements, not speculation!**

---

*This document is a living reference for future development. Ideas should be validated against actual hardware performance and user requirements before implementation.*
