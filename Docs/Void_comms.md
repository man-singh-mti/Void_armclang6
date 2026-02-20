# Void System Communication Protocol

Version: 1.7
Last updated: 2026-02-06
Target branch: BSCT-437-DH-Void-Classification-Engine

This document defines the ASCII command/event protocol between Uphole and Void for the current XM125 + PCA9534 + `mti_radar` + `mti_void` architecture.

---

## 1. Overview

- Commands use the `@` prefix (e.g., `@vd,...`).
- Events use the `!` prefix (e.g., `!void,1`).
- All messages are comma-delimited and terminated by `\n`.

---

## 2. Configuration Variables

| Variable            | Type | Units  | Direction   | Range  | Default | Description                                    |
|---------------------|------|--------|-------------|--------|---------|------------------------------------------------|
| void_profile        | int  | -      | Uphole→Void | 0–5    | 0       | XM125 profile ID (0 = none/not set; 1–5 valid) |
| void_threshold      | int  | mm     | Uphole→Void | 0–3000 | 250     | Distance threshold in millimeters              |
| void_min_strength   | int  | %      | Uphole→Void | 0–100  | 0       | Minimum peak strength filter (0 disables)      |
| void_hysteresis_pct | int  | %      | Uphole→Void | 0–100  | 5       | Exit hysteresis percent                        |
| void_debounce_enter | int  | frames | Uphole→Void | 1–10   | 3       | Debounce frames for enter                      |
| void_debounce_exit  | int  | frames | Uphole→Void | 1–10   | 5       | Debounce frames for exit                       |

### Profile IDs

| ID | Name            |
|----|-----------------|
| 1  | Balanced        |
| 2  | High Speed      |
| 3  | High Resolution |
| 4  | Fixed Threshold |
| 5  | Close Range     |

Profiles are defined in [`Device/Src/xm125_profiles.c`](../Device/Src/xm125_profiles.c) and selected via `xm125_profile_set()`.

---

## 3. API Reference

### 3.1 Commands

| Command                | Arguments/Format                     | Description                              | Response Format                                        | Version |
|------------------------|--------------------------------------|------------------------------------------|--------------------------------------------------------|---------|
| `@connect`             |                                      | Handshake                                | Device info                                            | 1.0     |
| `@init`                |                                      | Initialize                               | Status                                                 | 1.0     |
| `@vd,prf,<p>`          | `<p>`: int (0–5)                     | Set profile                              | `@vd,prf,<p>,thr,<mm>,str,<min>,hys,<pct>,deb,<e>,<x>` | 1.0     |
| `@vd,thr,<mm>`         | `<mm>`: int (0–3000)                 | Set threshold                            | same as above                                          | 1.0     |
| `@vd,str,<min>`        | `<min>`: int (0–100)                 | Set minimum strength                     | same as above                                          | 1.5     |
| `@vd,hys,<pct>`        | `<pct>`: int (0–100)                 | Set hysteresis percent                   | same as above                                          | 1.0     |
| `@vd,deb,<e>,<x>`      | `<e>`: int (1–10), `<x>`: int (1–10) | Set debounce frames (enter, exit)        | same as above                                          | 1.7     |
| `@vd,?`                |                                      | Query current config                     | same as above                                          | 1.0     |
| `@vd,state,?`          |                                      | Query void state (global + per-sat)      | `@vd,state,<g>,s1,<v1>,...,s6,<v6>`                    | 1.7     |
| `@vd,<p>,<mm>,<s>,<h>` | Compact: int,int,int,int             | Set all config (profile, threshold, etc) | same as above                                          | 1.7     |

> Note: `@vol` commands are not implemented and have been removed from this document.

### 3.2 Events

| Event          | Format          | Description       | Version |
|----------------|-----------------|-------------------|---------|
| `!void,<0\|1>` | `!void,<state>` | Void state change | 1.0     |

### 3.3 Debug Messages

| Message                       | Description                  | Trigger                  |
|-------------------------------|------------------------------|--------------------------|
| `@db,Invalid ...`             | Parameter validation failure | Invalid command argument |
| `@db,Void State Changed: <x>` | State transition             | Void detection or clear  |

---

## 4. Message Format Contract

### Command Format

```text
@<cmd>[,<arg1>[,<arg2>[,...]]]<LF>
```

- Prefix: `@`
- Command: 2–10 lowercase alphanumeric characters
- Arguments: comma-separated, no spaces
- Terminator: `\n` (LF, 0x0A)

### Event Format

```text
!<event>[,<value1>[,<value2>[,...]]]<LF>
```

- Prefix: `!`
- Event: lowercase alphanumeric identifier
- Values: comma-separated, no spaces
- Terminator: `\n` (LF, 0x0A)

### Response Format

```text
@<cmd>,<field1>,<value1>[,<field2>,<value2>[,...]]<LF>
```

### Debug Messages

```text
@db,<message><LF>
```

> **Note:** Debug messages are informational and should not be parsed for control flow.

---

## 5. Detection Semantics

- The first valid target in each satellite measurement is used by `mti_void_process()`.
- Targets are ordered by the active profile’s peak sorting method (current profiles use closest-first).
- Strength gating is applied globally by `mti_radar_set_min_strength()` before targets reach the void logic.

### Enter/Exit Logic

Let:

- $d$ = target distance (mm)
- $T$ = `void_threshold` (mm)
- $H$ = `void_hysteresis_pct` (%)
- $E$ = `void_debounce_enter` (frames, default 3)
- $X$ = `void_debounce_exit` (frames, default 5)

**Void Enter:**
If $d > T$ for $E$ consecutive frames, state transitions to "Detected".

**Void Exit:**
If $d < T \times (1 - H/100)$ for $X$ consecutive frames, state transitions to "Cleared".

**Strength gating:** Only targets with strength $\geq$ `void_min_strength` are considered (with -3% hysteresis for exit).

---

## 6. Error Handling

| Condition              | Response Example               | Action                                  |
|------------------------|--------------------------------|-----------------------------------------|
| Invalid command syntax | `@db,Invalid <cmd> args`       | Log and optionally retry                |
| Out-of-range value     | `@db,Invalid <param>: <value>` | Value rejected, previous retained       |
| Command not found      | *(none)*                       | Silently ignored (may change in future) |
| Timeout (no response)  | *(none)*                       | Uphole: log, retry, or fallback         |

> **Note:** Uphole should implement a 500 ms timeout for all commands.

---

## 7. Examples

### Set and confirm profile + thresholds (compact)

1. Host: `@vd,3,50,50,5`
2. Void: `@vd,prf,3,thr,50,str,50,hys,5`

### Set threshold then strength

1. Host: `@vd,thr,250`
2. Void: `@vd,prf,3,thr,250,str,0,hys,5`
3. Host: `@vd,str,80`
4. Void: `@vd,prf,3,thr,250,str,80,hys,5`

### Query current config

1. Host: `@vd,?`
2. Void: `@vd,prf,3,thr,250,str,80,hys,5`

---

## 8. Testing & Validation

### Unit Test Checklist

- [ ] All commands with valid arguments return expected format
- [ ] All commands with invalid arguments return `@db,Invalid ...`
- [ ] Query commands return current state
- [ ] Events are emitted only on state changes
- [ ] No spurious events or messages

### Integration Checklist

- [ ] Implement command timeout (500 ms)
- [ ] Log all `@db,Invalid ...` messages
- [ ] Validate response format matches command
- [ ] Handle missing/delayed responses gracefully
- [ ] Implement exponential backoff for retries

---

## 9. Migration Guide

### From Version 1.6 to 1.7

- **Added:** Compact command form: `@vd,<p>,<mm>,<s>,<h>`
- **Migration:** No changes required; compact form is optional

### From Version 1.x to 2.x (Future)

If breaking changes are introduced:

1. **Announce deprecation** in version N
2. **Maintain backward compatibility** in version N+1
3. **Remove deprecated features** in version N+2

---

## Appendix A: Implementation References

- `@vd` parsing: [`cmd_void()`](../Device/Src/vmt_command.c)
- Void classification: [`mti_void_process()`](../Device/Src/mti_void.c)
- Radar filtering: [`mti_radar_set_min_strength()`](../Device/Src/mti_radar.c)
- Profile selection: [`xm125_profile_set()`](../Device/Src/xm125_profiles.c)

---

## Appendix B: Extension Points

> **Note:** Developer-facing code patterns and extension points have been moved to
> this appendix for clarity. See `CONTRIBUTING.md` for more.

---
