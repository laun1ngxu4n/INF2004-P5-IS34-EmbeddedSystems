"""The robot's binary MQTT messages, laptop side (Change Requirement 1).

The format is docs/team/MQTT_WIRE_FORMAT.md; the robot side is
app_program/subsystem1/wire.c.  Byte 0 = version, then the fields in
order, lowest bit first, starting at bit 0 of byte 1 (the bytes after the
version read as one little-endian number).  Signed fields are two's
complement in their width.

    import is34_wire
    is34_wire.unpack_status(payload)   # dict, or None if not a status message
    is34_wire.pack_map({"L": "K", "R": "Q", "F": "X", "U": "M"})   # 4 bytes
"""

VERSION = 1

# name, bits, signed
STATUS_FIELDS = [
    ("mission", 2, False), ("line", 2, False), ("stop", 3, False),
    ("faults", 8, False), ("speed_l", 12, True), ("speed_r", 12, True),
    ("dist_mm", 20, True), ("junctions", 8, False), ("last", 6, False),
    ("map_L", 6, False), ("map_R", 6, False), ("map_F", 6, False),
    ("map_U", 6, False), ("ramp_peak_mm10", 12, False),
]
STATUS_LEN = 15
SENSOR_FIELDS = [
    ("ir_l", 1, False), ("ir_r", 1, False), ("ir_b", 1, False),
    ("hold", 1, False), ("inbar", 1, False), ("imu_ok", 1, False),
    ("steer", 8, True), ("front_status", 2, False), ("front_mm", 13, False),
    ("tilt", 11, True), ("h10", 13, True),
]
SENSORS_LEN = 8
ACK_FIELDS = [
    ("ok", 1, False), ("set", 1, False), ("map_L", 6, False),
    ("map_R", 6, False), ("map_F", 6, False), ("map_U", 6, False),
]
ACK_LEN = 5
MAP_LEN = 4
MOVES = "LRFU"

MISSION_NAMES = ["IDLE", "CAL", "RUN", "MANOEUVRE"]
LINE_NAMES = ["ON", "SEARCH", "LOST", "FAULT"]
STOP_BITS = {1: "button", 2: "laptop", 4: "fault"}
FAULT_BITS = {0x01: "motor stall", 0x02: "encoder (warning)", 0x04: "IMU (warning)",
              0x08: "no sonar echo", 0x10: "line sensor stuck (one)",
              0x20: "line sensors stuck (both)", 0x40: "Wi-Fi lost (warning)",
              0x80: "Motion task hung"}
FRONT_STATUS = {0: None, 1: "nothing in range", 2: "NO ECHO (fault)"}
OTHER = 63


def letter(code):
    """6-bit code -> 'A'..'Z', '0'..'9', '' (none) or '?' (another character)"""
    if 1 <= code <= 26:
        return chr(ord("A") + code - 1)
    if 27 <= code <= 36:
        return chr(ord("0") + code - 27)
    return "" if code == 0 else "?"


def code(ch):
    """'A'..'Z' / '0'..'9' -> 6-bit code"""
    if "A" <= ch <= "Z":
        return 1 + ord(ch) - ord("A")
    if "0" <= ch <= "9":
        return 27 + ord(ch) - ord("0")
    raise ValueError(f"{ch!r} cannot be on a barcode mapping")


def _unpack(payload, fields, length):
    if len(payload) != length or payload[0] != VERSION:
        return None
    v = int.from_bytes(payload[1:], "little")
    out, pos = {}, 0
    for name, bits, signed in fields:
        x = (v >> pos) & ((1 << bits) - 1)
        if signed and x >> (bits - 1):
            x -= 1 << bits
        out[name] = x
        pos += bits
    return out


def unpack_status(payload):
    return _unpack(payload, STATUS_FIELDS, STATUS_LEN)


def unpack_sensors(payload):
    return _unpack(payload, SENSOR_FIELDS, SENSORS_LEN)


def unpack_ack(payload):
    return _unpack(payload, ACK_FIELDS, ACK_LEN)


def map_text(d):
    """the mapping in a status / ack dict as 'K=L,Q=R,X=F,M=U', '' if none"""
    letters = [letter(d[f"map_{mv}"]) for mv in MOVES]
    if not all(letters):
        return ""
    return ",".join(f"{ch}={mv}" for ch, mv in zip(letters, MOVES))


def pack_map(choice):
    """{'L': 'K', 'R': 'Q', 'F': 'X', 'U': 'M'} -> the 4-byte map command"""
    v = 0
    for i, mv in enumerate(MOVES):
        v |= code(choice[mv]) << (6 * i)
    return bytes([VERSION]) + v.to_bytes(3, "little")


def bits_names(value, names):
    return ", ".join(n for b, n in names.items() if value & b) or "none"
