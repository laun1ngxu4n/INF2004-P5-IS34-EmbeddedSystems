# MQTT wire format (binary, version 1)

The robot's MQTT application messages are compact binary (Change
Requirement 1, Minimum-Bit Data Representation). This is the specification
both ends follow:
- robot side: [`app_program/subsystem1/wire.c`](../../app_program/subsystem1/wire.c);
- laptop side: [`tools/panel/is34_wire.py`](../../tools/panel/is34_wire.py).

Check that the two agree with the golden-vector test:
`python tools/panel/test_wire.py`. It needs `gcc`. It builds the robot's
`wire.c` on the laptop and checks every field of every test message,
including the clamping.

## Rules

- **Byte 0 is the format version**, now `1`. A message with another version, or the wrong length, is refused.
- **Then the fields, in the order of the tables below.** Each field takes exactly its width in bits, lowest bit first. Packing starts at bit 0 of byte 1, so the bytes after the version byte read as one little-endian number. In Python: `int.from_bytes(payload[1:], "little")`.
- **Packing uses explicit shifts and masks,** never a C bit-field struct sent as it is: the compiler chooses the order and padding of bit-fields.
- **Signed fields are two's complement** in their width.
- **A value outside its field's range is clamped** to the range, never cut off.
- **Spare bits at the end of the last byte are 0** and ignored on receipt.
- **Letter codes (6 bits):**
  - `0` none;
  - `1`–`26` = `A`–`Z`;
  - `27`–`36` = `0`–`9`;
  - `63` = another Code 39 character (`-`, `.`, `$`, ...).

## Robot → laptop

### `is34/telemetry`: status, every 1 s, 15 bytes (text before: about 93)

| Field | Bits | Signed | Range / meaning |
|---|---|---|---|
| mission | 2 | | 0 IDLE, 1 CAL, 2 RUN, 3 MANOEUVRE |
| line | 2 | | 0 ON, 1 SEARCH, 2 LOST, 3 FAULT |
| stop | 3 | | STOP reasons: 1 button, 2 laptop, 4 fault |
| faults | 8 | | 0x01 motor stall, 0x02 encoder (warning), 0x04 IMU (warning), 0x08 no sonar echo, 0x10 one line sensor stuck, 0x20 both stuck, 0x40 Wi-Fi lost (warning), 0x80 Motion task hung |
| speed_l, speed_r | 12 each | yes | measured wheel speed, mm/s, ±2047 |
| dist_mm | 20 | yes | odometer since boot, mm, ±524,287 |
| junctions | 8 | | junctions since boot, mod 256 |
| last | 6 | | last barcode read (letter code; 0 = none or READ_FAIL) |
| map_L, map_R, map_F, map_U | 6 each | | the letter mapped to left, right, forward, U-turn (0 = no mapping) |
| ramp_peak_mm10 | 12 | | last ramp height reported, 0.1 mm, 0..4095 |

109 bits, plus 3 spare bits, fill 14 bytes; with the version byte, 15.

### `is34/sensors`: live readings, every 0.5 s, 8 bytes (text before: about 59)

| Field | Bits | Signed | Range / meaning |
|---|---|---|---|
| ir_l, ir_r, ir_b | 1 each | | left / right line sensor, barcode sensor: 1 = black |
| hold | 1 | | steering held straight (barcode sensor over bars) |
| inbar | 1 | | inside a barcode |
| imu_ok | 1 | | the IMU answers |
| steer | 8 | yes | steering, mm/s (±127; in use ±40) |
| front_status | 2 | | 0 = front_mm is a distance, 1 = nothing in range, 2 = no ECHO (sensor fault) |
| front_mm | 13 | | forward distance, mm, 0..8191 (0 unless front_status is 0) |
| tilt | 11 | yes | sin(pitch) × 1000, ±1023 |
| h10 | 13 | yes | climbed height, 0.1 mm, ±409.5 mm |

53 bits, plus 3 spare bits, fill 7 bytes; with the version byte, 8.

### `is34/ack/map`: answer to a mapping, 5 bytes (text before: 18)

| Field | Bits | Range / meaning |
|---|---|---|
| ok | 1 | 1 = the new mapping was accepted |
| set | 1 | 1 = a mapping is in use |
| map_L, map_R, map_F, map_U | 6 each | the mapping now in use (letter codes) |

When a mapping is refused, the reason goes out as an `is34/event` sentence.

### Kept as text

| Topic | Why |
|---|---|
| `is34/event` | sentences for the operator ("junction 2 (crossing): LEFT (barcode K)"), not values |
| `is34/log` | a copy of the console text (`L<seq> <dropped>|<text>`) |

## Laptop → robot

| Topic | Payload | Size (before) |
|---|---|---|
| `is34/cmd/map` | version byte, then the letter codes for L, R, F, U, 6 bits each (24 bits = 3 bytes) | 4 bytes (15) |
| `is34/cmd/start` | empty: the topic is the command | 0 bytes (2) |
| `is34/cmd/stop` | empty: the topic is the command. The robot sets its STOP bit before anything else | 0 bytes (3) |

A map command is refused, and the old mapping kept, in these cases:
- the version or length is wrong;
- a code is not A–Z / 0–9;
- a letter or a move appears twice.

The robot still accepts the text form `K=L,Q=R,X=F,M=U` typed by hand
(`mosquitto_pub`); the panel always sends the binary form.

## Example

A status message with mission RUN, line ON, no STOP, Wi-Fi lost
(0x40), speeds 120 / −118 mm/s, 12,345 mm, 3 junctions, last barcode `F`,
mapping `K=L,Q=R,X=F,M=U` and ramp 12.3 mm encodes as:

```text
01 02 20 3c 50 fc 1c 18 18 30 96 08 6b f6 00
```

(This is the first golden vector in `tools/panel/wire_golden.c`.)
