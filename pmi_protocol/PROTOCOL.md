# PMI TCP Protocol Specification

This document describes the wire protocol used between `PMI_Client` and `PMI_Server`.

## 1. Common framing

- `SOF1`: `0xAA`
- `SOF2`: `0x55`
- `EOF`: `0xFE`

All frames start with `SOF1 SOF2` and end with `EOF`.

---

## 2. Client -> Server frame (command)

### 2.1 Layout

`[SOF1][SOF2][CMD][LEN][DATA...][CHECKSUM][EOF]`

- Total bytes: `6 + LEN`

### 2.2 Fields

- `CMD` (1 byte): command id
- `LEN` (1 byte): payload length (`0..255`)
- `DATA` (`LEN` bytes): command payload
- `CHECKSUM` (1 byte):
  - Formula: `(CMD + LEN + sum(DATA)) & 0xFF`

### 2.3 Command IDs

- `0x01`: Ping
- `0x10`: Servo On
- `0x11`: Stop (Servo Off)
- `0x12`: SetZero (capture motor/external encoder offsets)
- `0x20`: Mode Current
- `0x21`: Mode Velocity
- `0x22`: Mode Extended Position

---

## 3. Server -> Client frame (telemetry)

### 3.1 Layout

`[SOF1][SOF2][LEN][PAYLOAD][CHECKSUM][EOF]`

- `LEN` is payload byte length.
- Current protocol uses 4 axes.

### 3.2 Size

- Per-axis telemetry block: `59 bytes`
- Axis count: `4`
- Payload size: `59 * 4 = 236 bytes`
- Full frame size: `2 + 1 + 236 + 1 + 1 = 241 bytes`

### 3.3 Checksum

- Formula: `(LEN + sum(PAYLOAD)) & 0xFF`

---

## 4. Per-axis telemetry block (59 bytes)

Each axis block is serialized in little-endian format:

- `offset 0`   : `id_op_mode` (uint8)
- `offset 1`   : `servo_state` (uint8)
- `offset 2`   : `present_position` (float64, 8 bytes)
- `offset 10`  : `encoder_position` (float64, 8 bytes)
- `offset 18`  : `present_velocity` (float64, 8 bytes)
- `offset 26`  : `present_current` (float64, 8 bytes)
- `offset 34`  : `goal_position` (float64, 8 bytes)
- `offset 42`  : `goal_velocity` (float64, 8 bytes)
- `offset 50`  : `goal_current` (float64, 8 bytes)
- `offset 58`  : `error_state` (uint8)

### 4.1 `id_op_mode` packing

- High nibble: motor ID
- Low nibble: operation mode

Helper logic in code:

- `packTelemetryIdOp(id, op)`
- `telemetryIdFromIdOp(id_op_mode)`
- `telemetryOpModeFromIdOp(id_op_mode)`

---

## 5. Semantic notes

- `present_position`: motor position (DYNAMIXEL)
- `encoder_position`: external encoder position (AMT21)
  - If AMT21 read is unavailable, server may send `NaN`.
- Units used by the server payload:
  - Position: degrees
  - Velocity: degrees/second
  - Current: amperes

---

## 6. Runtime timing (current implementation)

- Server <-> motor/encoder polling: `5 ms`
- Server -> client telemetry send: `100 ms`
- Server terminal telemetry log print: `1 s`

