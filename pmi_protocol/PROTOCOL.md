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
- `0x23`: Mode Current-based Position
- `0x30`: SetWaypointBatch
- `0x31`: PlanPath
- `0x32`: StartTrajectoryIK
- `0x33`: StopTrajectoryIK
- `0x34`: SetInitialJointPose
- `0x35`: JogVelocity
- `0x40`: LogStart
- `0x41`: LogStop

### 2.4 Waypoint payload (`0x30`)

- Layout: `[count][t0][x0][y0][z0]...[tN-1][xN-1][yN-1][zN-1]`
- `count`: `uint8` (number of waypoints)
- `t/x/y/z`: each `float64` little-endian
- Total payload bytes: `1 + count * 32`
- `t` is in seconds, must be strictly increasing.

### 2.5 Initial joint pose payload (`0x34`)

- Layout: `[q1][q2][q3][q4]`
- `q1..q4`: each `float64` little-endian
- Unit: joint radian (output-link side, before gear ratio conversion)

### 2.6 Jog velocity payload (`0x35`)

- Layout: `[axis][joint_vel_deg_per_sec]`
- `axis`: `uint8` (`0..3`)
- `joint_vel_deg_per_sec`: `float64` little-endian (signed)
  - `> 0`: jog `+` direction
  - `< 0`: jog `-` direction
  - `= 0`: stop jog

### 2.7 Logging payload

- `0x40` (`LogStart`) payload:
  - Layout: `[duration_sec]`
  - `duration_sec`: `float64` little-endian
- `0x41` (`LogStop`) payload:
  - No payload (`LEN=0`)

---

## 3. Server -> Client frame (telemetry)

### 3.1 Layout

`[SOF1][SOF2][LEN][PAYLOAD][CHECKSUM][EOF]`

- `LEN` is payload byte length.
- Current protocol uses 4 axes.

### 3.2 Size

- Per-axis telemetry block: `58 bytes`
- Axis count: `4`
- Payload size: `58 * 4 = 232 bytes`
- Full frame size: `2 + 1 + 232 + 1 + 1 = 237 bytes`

### 3.3 Checksum

- Formula: `(LEN + sum(PAYLOAD)) & 0xFF`

### 3.4 Server ACK frame (control/status)

- Layout: `[SOF1][SOF2][MSG][LEN][DATA...][CHECKSUM][EOF]`
- `MSG` currently uses `0xA0` (`kSrvAck`)
- `CHECKSUM` formula: `(MSG + LEN + sum(DATA)) & 0xFF`
- Used for status messages such as:
  - `LOG_START_OK:<file_path>`
  - `LOG_STOP_OK:<file_path>`
  - `LOG_START_FAIL:<reason>`
  - `INIT_POSE_PROGRESS:<percent>`
  - `INIT_POSE_DONE`

---

## 4. Per-axis telemetry block (58 bytes)

Each axis block is serialized in little-endian format:

- `offset 0`   : `servo_state` (uint8)
- `offset 1`   : `present_position` (float64, 8 bytes)
- `offset 9`   : `encoder_position` (float64, 8 bytes)
- `offset 17`  : `present_velocity` (float64, 8 bytes)
- `offset 25`  : `present_current` (float64, 8 bytes)
- `offset 33`  : `goal_position` (float64, 8 bytes)
- `offset 41`  : `goal_velocity` (float64, 8 bytes)
- `offset 49`  : `goal_current` (float64, 8 bytes)
- `offset 57`  : `error_state` (uint8)

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

- Server <-> motor/encoder polling: `2 ms`
- Server -> client telemetry send: `100 ms` (port `9000`, telemetry-only)
- Client <-> server command/ACK exchange: `100 ms` cadence (port `9001`)
- Server terminal telemetry log print: `1 s`
