# PCA9534 I2C Interface & Protocol Summary

**Source:** `pca9534.pdf`
**Role:** General Purpose I/O Expander (Used for XM125 Control/Status)

This document focuses specifically on the I²C-bus interface, addressing, and timing specifications for the PCA9534 8-bit I/O expander.

This document defines PCA9534 I²C and register behavior only; system-level sequencing and XM125 semantics are defined elsewhere.

---

## Role Clarification (Control Plane Manager)

- Drives XM125 `WAKE_UP` and `NRESET` control lines.
- Senses XM125 `MCU_INT` readiness via Input Port.
- No interrupt usage; PCA9534 `INT` pin is unused.

## Virtual Control Constraint (Global)

- `WAKE_UP`, `NRESET`, `MCU_INT` are physical XM125 pins.
- Only accessible via PCA9534 over I²C.
- Subject to I²C latency and serialization.
- Not interrupt-capable at the MCU level.

## Scope & Responsibility

Defines PCA9534 I²C-bus behavior, device addressing, command/pointer semantics, and timing characteristics. Explicitly defers XM125 detector behavior, data-plane register semantics, and system-level sequencing to hardware and XM125 specifications. This document does not define data-plane rules beyond exposing the `MCU_INT` input via the Input Port.

## MCU_INT Semantics (Invariant)

- MCU_INT is a level-based readiness signal
- MCU_INT HIGH ⇒ XM125 ready for I²C
- MCU_INT LOW ⇒ XM125 busy or asleep
- MCU_INT is polled, never interrupt-driven
- Any XM125 I²C transaction while MCU_INT LOW is undefined behavior

## Void Satellite Specific Configuration

- Configuration Register (0x03) direction: **`0x5F` (IO7 & IO5 outputs, IO6 input, others inputs).**
- Initial Output Port (0x01) latch: **`0xA0` (NRESET high on IO7, WAKE_UP high on IO5).**
- Required boot sequence:
  1. Write **`0x5F`** to Configuration (0x03).
  2. Write **`0xA0`** to Output Port (0x01).
  3. Read MCU_INT (Input bit6) until HIGH.

## Control Macros (Canonical Operations)

| Action                | Register       | Pointer | Bit | Mask  | Operation        |
|-----------------------|----------------|---------|-----|-------|------------------|
| WAKE_UP HIGH          | Output Port    | 0x01    | 5   | 0x20  | Set bit 5        |
| WAKE_UP LOW           | Output Port    | 0x01    | 5   | 0x20  | Clear bit 5      |
| NRESET HIGH (release) | Output Port    | 0x01    | 7   | 0x80  | Set bit 7        |
| NRESET LOW (assert)   | Output Port    | 0x01    | 7   | 0x80  | Clear bit 7      |
| Read MCU_INT          | Input Port     | 0x00    | 6   | 0x40  | Read bit 6       |
| Configure directions  | Configuration  | 0x03    | -   | -     | Write `0x5F`     |
| Set initial outputs   | Output Port    | 0x01    | -   | -     | Write `0xA0`     |

## I²C-Bus Interface Overview

The PCA9534 communicates via a two-line bidirectional I²C-bus consisting of a serial data line (SDA) and a serial clock line (SCL).

- **Bus Roles:** The PCA9534 operates as a slave device. The master (microcontroller) generates the SCL clock signal and the START/STOP conditions.
- **Pin Configuration:**
  - **SCL (Pin 1):** Serial clock line. Input.
  - **SDA (Pin 2):** Serial data line. Open-drain bidirectional.
- **Speed:** Supports I²C Fast-mode (400 kHz) and Standard-mode (100 kHz).

---

## Device Addressing

The PCA9534 has a 7-bit slave address, using a combination of fixed internal bits and hardware-selectable pins to allow up to 8 devices on the same bus.

### I²C Addressing

| Bits [7:4] | Bits [3:1] (A2, A1, A0) |       R/W Bit [0]        |
|:----------:|:-----------------------:|:------------------------:|
|   `0100`   |   Hardware Selectable   | `0` (Write) / `1` (Read) |

- **Base Address:** `0x20` (A2=0, A1=0, A0=0)
- **Range:** `0x20` to `0x27` depending on hardware strapping.

#### Slave Address Structure

- **Fixed Part:** Four MSBs fixed to `0100`.
- **Programmable Part:** Next three bits (A2, A1, A0) set by hardware.
- **Read/Write Bit:** LSB indicates operation (1 = Read, 0 = Write).

| Fixed | Fixed | Fixed | Fixed | A2  | A1  | A0  | R/W |
|-------|-------|-------|-------|-----|-----|-----|-----|
| 0     | 1     | 0     | 0     | 0/1 | 0/1 | 0/1 | 1/0 |

---

## Bus Protocol & Transactions

Data is transmitted to and from the PCA9534 in 8-bit bytes, MSB first.

The PCA9534 does not support register auto-increment; each transaction accesses a single register selected by the command byte.

---

## Register Map (Command Bytes)

The PCA9534 uses a "Pointer Register" (Command Byte) to select which register to access. Write this pointer value as the first byte after the address byte.

The command byte (pointer) remains active until explicitly changed by another write.

| Command (Hex) | Register Name      |  Access   | Power-up Default | Description                                           |
|:-------------:|:-------------------|:---------:|:----------------:|:------------------------------------------------------|
|    `0x00`     | Input Port         | Read Only |       `X`        | Reflects current logic level on pins.                 |
|    `0x01`     | Output Port        |    R/W    |      `0xFF`      | Sets outgoing logic levels (if configured as output). |
|    `0x02`     | Polarity Inversion |    R/W    |      `0x00`      | `1` = Inverts data in Input Port register.            |
|    `0x03`     | Configuration      |    R/W    |      `0xFF`      | `1` = Input (Default), `0` = Output.                  |

---

## Register Descriptions & Firmware Logic

### Configuration Register (0x03) — Direction Control

- **Set Bit = 1:** Pin is **INPUT** (High-Impedance).
- **Set Bit = 0:** Pin is **OUTPUT**.

*To read XM125 status (e.g., `INT`), set corresponding PCA pin bit to `1`. To control XM125 lines (e.g., `RST`, `WAKE`), set bits to `0`.*

### Input Port Register (0x00) — Polling Status

- **0:** Pin is Logic Low (GND).
- **1:** Pin is Logic High (VCC).

**Polling Implementation (No Hardware Interrupt):**
Firmware must poll the Input Port register; no interrupt-driven mechanism is available or supported.
Since the PCA9534's physical `INT` line is unused, the host must poll this register to detect changes.

## MCU_INT Semantics (Lock)

- PCA9534 `INT` pin unused.
- Poll Input Port (0x00).
- Level-based, not edge-based.
- No interrupt-driven logic permitted.

**Polling Sequence:**

1. Write I²C Address + Write Bit.
2. Write Command Byte `0x00` (Select Input Register).
3. Send RESTART.
4. Write I²C Address + Read Bit.
5. Read Data Byte.

*Check the bit corresponding to the XM125 `INT` line to see if the radar module is indicating readiness state.*

### Output Port Register (0x01) — Control Lines

- **0:** Drive Pin Low.
- **1:** Drive Pin High.

*Reads from this register return the value stored in the flip-flop, not the actual pin state.*

### Polarity Inversion Register (0x02)

- **1:** Corresponding Input Port register bit is inverted.
- **0:** Normal logic (Default).

*Recommendation: Keep at `0x00` to match logical High/Low signals from XM125.*

#### Command Byte Definitions

| Protocol Value | Register Name      | Description            |
|:--------------:|:-------------------|:-----------------------|
|      `00`      | Input Port         | Read-only input status |
|      `01`      | Output Port        | R/W output latch       |
|      `02`      | Polarity Inversion | R/W polarity control   |
|      `03`      | Configuration      | R/W direction control  |

---

## Writing to the Port

Write operation sequence:

1. **START** condition generated by the master.
2. **Slave Address** sent by master with R/W bit set to **0**.
3. **ACK** from PCA9534.
4. **Command Byte** (Pointer) sent by master (e.g., `0x01` for Output Port).
5. **ACK** from PCA9534.
6. **Data Byte** sent by master (data to be written to the register).
7. **ACK** from PCA9534.
8. **STOP** condition generated by the master.

---

## Reading from the Port

Read operation can occur in two modes:

**A. Read from Current Pointer Location:**

This mode is valid only if the pointer was previously set by a write transaction.

1. **START** condition.
2. **Slave Address** with R/W bit set to **1**.
3. **ACK** from PCA9534.
4. **Data Byte** transmitted by PCA9534.
5. **NACK** from master to indicate end of read.
6. **STOP** condition.

**B. Read from Specific Register (Random Read):**

1. Perform a "dummy write" to set the pointer: START → Slave Addr (W) → Command Byte (Pointer).
2. **Restart** (Repeated START) condition (do not send STOP).
3. **Slave Address** with R/W bit set to **1**.
4. **ACK** from PCA9534.
5. **Data Byte** transmitted by PCA9534.
6. **NACK** from master.
7. **STOP** condition.

---

## I²C-Bus Timing Characteristics

Key timing parameters for the I²C-bus interface at VCC = 2.3V to 5.5V and TA = -40°C to +85°C:

| Parameter                              | Symbol  | Standard-mode Max | Fast-mode Max | Unit |
|----------------------------------------|---------|-------------------|---------------|------|
| SCL Clock Frequency                    | fSCL    | 100               | 400           | kHz  |
| Bus Free Time (between STOP and START) | tBUF    | 4.7               | 1.3           | μs   |
| Hold Time (Repeated) START             | tHD;STA | 4.0               | 0.6           | μs   |
| Setup Time (Repeated) START            | tSU;STA | 4.7               | 0.6           | μs   |
| Setup Time for STOP                    | tSU;STO | 4.0               | 0.6           | μs   |
| Data Hold Time                         | tHD;DAT | 0                 | 0             | ns   |
| Data Setup Time                        | tSU;DAT | 250               | 100           | ns   |
| SCL Low Period                         | tLOW    | 4.7               | 1.3           | μs   |
| SCL High Period                        | tHIGH   | 4.0               | 0.6           | μs   |

*Note: The device incorporates a glitch filter on the SDA and SCL pins that suppresses noise spikes less than 50 ns (Fast-mode).

---

## Initialization Workflow (Example)

For a system where:

- **IO 5** = Output (WAKE_UP)
- **IO 7** = Output (NRESET)
- **IO 6** = Input (Poll for MCU_INT)

### Step 1: Configure Directions (Reg 0x03)

- Value: **`0101 1111` (Binary) → `0x5F`**
- Action: Write **`0x5F`** to Register `0x03` (IO7 & IO5 as outputs, IO6 as input, others inputs).

### Step 2: Set Initial Output States (Reg 0x01)

- Value: **`1010 0000` (NRESET High on IO7, WAKE_UP High on IO5) → `0xA0`**
- Action: Write **`0xA0`** to Register `0x01`.

### Step 3: Poll MCU_INT (Reg 0x00, Bit 6)

- Action: Periodically read Register `0x00`.
- Logic: Check if Bit 6 (MCU_INT) is High (XM125 ready for I²C).
- Proceed only when Bit 6 is High.
