"""Golden-vector test of the binary MQTT messages (Change Requirement 1).

Builds the robot's own encoder (app_program/subsystem1/wire.c) for the laptop
with tools/panel/wire_golden.c, and checks:
  - every message it packs decodes with the panel's decoder (is34_wire.py)
    to the values it was given, clamped as docs/team/MQTT_WIRE_FORMAT.md says;
  - every map command the panel packs unpacks on the robot side to the same
    letters, and a broken one is refused.

    python tools/panel/test_wire.py          (needs gcc on the PATH)
"""
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import is34_wire as wire  # noqa: E402

FAILS = []


def check(cond, what):
    if not cond:
        FAILS.append(what)


def clamp(v, bits, signed):
    if signed:
        lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    else:
        lo, hi = 0, (1 << bits) - 1
    return max(lo, min(hi, v))


def letter_code(ascii_code):
    ch = chr(ascii_code)
    if ascii_code == 0:
        return 0
    if "A" <= ch <= "Z" or "0" <= ch <= "9":
        return wire.code(ch)
    return wire.OTHER


def build():
    exe = os.path.join(tempfile.mkdtemp(prefix="wire_golden_"), "wire_golden.exe")
    cmd = ["gcc", "-std=gnu11", "-Wall", "-Wextra", "-I", os.path.join(REPO, "app_program"),
           "-DEXPORT=", "-DIMPORT=extern", "-DLOCAL=static", "-DInline=static inline",
           os.path.join(HERE, "wire_golden.c"),
           os.path.join(REPO, "app_program", "subsystem1", "wire.c"), "-o", exe]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0 or r.stderr.strip():
        print(r.stderr)
        sys.exit("could not build wire_golden.c cleanly")
    return exe


def main():
    exe = build()
    lines = subprocess.run([exe], capture_output=True, text=True, check=True).stdout.split("\n")
    n = 0
    for line in filter(None, lines):
        kind, *parts = line.split()
        given = dict(p.split("=", 1) for p in parts)
        payload = bytes.fromhex(given.pop("hex"))
        length = int(given.pop("len"))
        check(len(payload) == length, f"{kind}: length {len(payload)} != {length}")
        if kind == "status":
            d = wire.unpack_status(payload)
            fields = wire.STATUS_FIELDS
        elif kind == "sensors":
            d = wire.unpack_sensors(payload)
            fields = wire.SENSOR_FIELDS
        else:
            d = wire.unpack_ack(payload)
            fields = wire.ACK_FIELDS
        check(d is not None, f"{kind}: did not decode: {line}")
        if d is None:
            continue
        for name, bits, signed in fields:
            if name == "set":                       # ack: a mapping is in use
                want = 1 if int(given["map_L"]) else 0
            elif name.startswith("map_") or name == "last":
                want = letter_code(int(given[name]))
            elif name == "junctions":
                want = int(given[name]) % 256       # sent mod 256
            elif name == "ramp_peak_mm10":
                want = clamp(max(0, int(given[name])), bits, False)
            elif name == "front_status":
                f = int(given["front"])
                want = 1 if f == -1 else (2 if f < 0 else 0)
            elif name == "front_mm":
                f = int(given["front"])
                want = 0 if f < 0 else clamp(f, bits, False)
            else:
                want = clamp(int(given[name]), bits, signed)
            check(d[name] == want, f"{kind} {name}: got {d[name]}, want {want} ({line})")
        n += 1

    # laptop -> robot: the map command
    for choice in ({"L": "K", "R": "Q", "F": "X", "U": "M"},
                   {"L": "0", "R": "9", "F": "A", "U": "Z"}):
        payload = wire.pack_map(choice)
        check(len(payload) == wire.MAP_LEN, "map: length")
        out = subprocess.run([exe, payload.hex()], capture_output=True, text=True).stdout.split()
        check(out == ["1", "".join(choice[m] for m in wire.MOVES)], f"map {choice}: robot read {out}")
        n += 1
    for bad in ("02" + wire.pack_map({"L": "A", "R": "B", "F": "C", "U": "D"}).hex()[2:],  # version
                wire.pack_map({"L": "A", "R": "B", "F": "C", "U": "D"}).hex()[:6],          # short
                "01000000"):                                                                # no letters
        out = subprocess.run([exe, bad], capture_output=True, text=True).stdout.split()
        check(out == ["0"], f"map {bad}: should be refused, robot read {out}")
        n += 1

    if FAILS:
        print("\n".join(FAILS))
        sys.exit(f"FAIL: {len(FAILS)} problems")
    print(f"PASS: {n} golden vectors (robot encoder <-> panel decoder)")


if __name__ == "__main__":
    main()
