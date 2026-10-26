#!/usr/bin/env python3
"""
Reconnect (NFR5 / A1) trials with laptop-clock timing, no button marks.
P5-IS34 Buddy 1. Works with wifi_mqtt_test / wifi_mqtt_test_prompt (any build).

Broker mode (default): this script runs Mosquitto itself, stops it, waits a
random time, starts it again, and records when the robot is back.
    recovery = robot's first heartbeat after the broker is listening again
               (and, with --serial, the time the robot printed "connected")
All times come from this laptop's clock, so there is no reaction time in them.

Hotspot mode (--mode hotspot): you press Enter FIRST and then switch the
phone hotspot off (or on) straight away. Because the mark comes before the
switch, detection and recovery are upper bounds (they include your delay and
the hotspot's start-up), so a PASS is conservative. Pressing Enter after the
switch would make the numbers too small: do not do that.

Phase (--phase sweep, broker mode, needs --serial): instead of a random
restart time, each trial restarts the broker at a planned offset after the
robot starts a connect attempt (0 .. --cycle-ms spread over the trials), so the
whole retry cycle is covered. The achieved offset is logged as restore_phase_ms.

Results per trial: PASS / FAIL / TIMEOUT, or not counted: INVALID (no real
outage, or the laptop's own monitor was down), LATE_MARK (robot already back
before the "on" mark), REBOOT (robot came back with a new boot_id).

Why not use mqtt_watch.py for this: it connects through the same broker, so
when the broker stops it disconnects too and reconnects with its own back-off.
Its gaps on 18 Oct were 140 s and 74 s for robot outages of 118 s and 68 s.

Setup (Windows):
    pip install paho-mqtt pyserial
    Stop any Mosquitto service first:   net stop mosquitto   (admin prompt)
    Close Tera Term if you use --serial (only one program can open the port).

Examples:
    python tools/outage_test.py --trials 5 --serial COM5
    python tools/outage_test.py --mode hotspot --trials 5 --serial COM5
    python tools/outage_test.py --trials 10 --serial COM5 --phase sweep
    python tools/outage_test.py --broker-cmd "C:\\Program Files\\mosquitto\\mosquitto.exe -c tools\\mosquitto.conf -v"

Outputs (in --out, default docs/logs):
    outage_<date>_<time>.csv      one row per trial
    outage_<date>_<time>_serial.txt   full robot serial log with laptop timestamps (--serial)
    outage_<date>_<time>_broker.txt   Mosquitto's own log
"""
import argparse
import csv
import json
import os
import random
import re
import shlex
import socket
import statistics
import subprocess
import sys
import threading
import time

import paho.mqtt.client as mqtt

LIMIT_MS = 20000          # NFR5: back within 20 s
HB_TOPIC = "robot/heartbeat"

RE_LOST = re.compile(r"\[outage\] #(\d+) LOST at (\d+) ms \(([^)]*)\)")
RE_CONNECTED = re.compile(r"\[mqtt\] connected in (\d+) ms")
RE_CONNECTING = re.compile(r"state -> MQTT_CONNECTING")
RE_JOINED = re.compile(r"joined \+ DHCP in (\d+) ms")


def now():
    return time.monotonic()


def wall(t_mono, t0_mono, t0_wall):
    """Monotonic time -> wall-clock string, for the CSV."""
    return time.strftime("%H:%M:%S", time.localtime(t0_wall + (t_mono - t0_mono))) + \
        ".%03d" % int(((t0_wall + (t_mono - t0_mono)) % 1) * 1000)


class Events:
    """Thread-safe log of timestamped robot events."""

    def __init__(self):
        self.lock = threading.Lock()
        self.hb = []          # (t, boot_id, seq)
        self.serial = []      # (t, kind, value, line)

    def add_hb(self, t, boot_id, seq):
        with self.lock:
            self.hb.append((t, boot_id, seq))

    def add_serial(self, t, kind, value, line):
        with self.lock:
            self.serial.append((t, kind, value, line))

    def first_hb_after(self, t0):
        with self.lock:
            for e in self.hb:
                if e[0] >= t0:
                    return e
        return None

    def hb_between(self, t0, t1):
        with self.lock:
            return [e for e in self.hb if t0 <= e[0] <= t1]

    def last_hb(self):
        with self.lock:
            return self.hb[-1] if self.hb else None

    def serial_after(self, t0, kind):
        with self.lock:
            for e in self.serial:
                if e[0] >= t0 and e[1] == kind:
                    return e
        return None

    def serial_last_before(self, t0, kind, not_before=None):
        with self.lock:
            hit = None
            for e in self.serial:
                if e[0] > t0:
                    break
                if e[1] == kind and (not_before is None or e[0] >= not_before):
                    hit = e
        return hit


def serial_reader(port, baud, ev, log_path, stop):
    import serial  # pyserial, only needed with --serial
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except serial.SerialException as exc:
        print(f"[serial] cannot open {port}: {exc} (close Tera Term?)")
        os._exit(2)
    buf = b""
    with open(log_path, "w", encoding="utf-8", errors="replace") as log:
        while not stop.is_set():
            chunk = ser.read(512)
            t = now()
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").rstrip("\r")
                log.write(f"{time.time():.3f} {line}\n")
                log.flush()
                m = RE_LOST.search(line)
                if m:
                    ev.add_serial(t, "lost", int(m.group(2)), line)
                    print(f"  [robot] {line.strip()}")
                    continue
                m = RE_CONNECTED.search(line)
                if m:
                    ev.add_serial(t, "connected", int(m.group(1)), line)
                    print(f"  [robot] {line.strip()}")
                    continue
                if RE_CONNECTING.search(line):
                    ev.add_serial(t, "connecting", 0, line)
                    continue
                m = RE_JOINED.search(line)
                if m:
                    ev.add_serial(t, "joined", int(m.group(1)), line)
                    print(f"  [robot] {line.strip()}")
    ser.close()


class Watcher:
    """Our own MQTT client on 127.0.0.1. Recreated for every broker start."""

    def __init__(self, port, ev):
        self.port = port
        self.ev = ev
        self.client = None
        self.subscribed = False

    def healthy(self):
        return (self.client is not None and self.client.is_connected()
                and self.subscribed)

    def start(self):
        self.subscribed = False
        try:
            c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                            client_id="p5is34-outage-test")
        except AttributeError:  # paho-mqtt 1.x
            c = mqtt.Client(client_id="p5is34-outage-test")
        def on_connect(cl, *a):
            self.subscribed = False
            cl.subscribe(HB_TOPIC, qos=0)

        def on_subscribe(cl, *a):
            self.subscribed = True

        c.on_connect = on_connect
        c.on_subscribe = on_subscribe
        c.on_message = self._on_message
        c.reconnect_delay_set(min_delay=1, max_delay=1)
        c.connect("127.0.0.1", self.port, keepalive=30)
        c.loop_start()
        self.client = c

    def _on_message(self, client, userdata, msg):
        t = now()
        try:
            d = json.loads(msg.payload)
        except ValueError:
            return
        self.ev.add_hb(t, d.get("boot_id"), d.get("seq"))

    def stop(self):
        if self.client:
            try:
                self.client.disconnect()
            except Exception:
                pass
            self.client.loop_stop()
            self.client = None


def port_open(port):
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.2):
            return True
    except OSError:
        return False


def wait_port(port, want_open, timeout_s):
    t_end = now() + timeout_s
    while now() < t_end:
        if port_open(port) == want_open:
            return now()
        time.sleep(0.02)
    return None


class Broker:
    def __init__(self, cmd, port, log_path):
        self.cmd = cmd
        self.port = port
        self.log = open(log_path, "a", encoding="utf-8", errors="replace")
        self.proc = None

    def start(self):
        self.log.write(f"\n===== start {time.strftime('%H:%M:%S')} =====\n")
        self.log.flush()
        self.proc = subprocess.Popen(self.cmd, stdout=self.log, stderr=subprocess.STDOUT)
        t_up = wait_port(self.port, True, 10.0)
        if t_up is None:
            print("[broker] did not start listening within 10 s - check --broker-cmd")
            self.stop()
            sys.exit(2)
        return t_up

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(5)
        self.proc = None
        t_down = wait_port(self.port, False, 5.0)
        self.log.write(f"===== stopped {time.strftime('%H:%M:%S')} =====\n")
        self.log.flush()
        return t_down if t_down is not None else now()


def wait_for_robot(ev, timeout_s, steady_s):
    """Wait for steady_s seconds of uninterrupted heartbeats (no gap over 1.6 s)
    from one boot_id. Returns that boot_id, or None on timeout."""
    t_end = now() + timeout_s
    while now() < t_end:
        t1 = now()
        hbs = ev.hb_between(t1 - steady_s - 1.0, t1)
        if hbs and (t1 - hbs[-1][0]) <= 1.6:
            run = [hbs[-1]]
            for e in reversed(hbs[:-1]):
                if run[0][0] - e[0] > 1.6 or e[1] != run[0][1]:
                    break
                run.insert(0, e)
            if run[-1][0] - run[0][0] >= steady_s - 1.0:
                return run[-1][1]
        time.sleep(0.1)
    return None


def ms(x):
    return None if x is None else int(round(x * 1000.0))


def run_trial(i, args, ev, broker, watcher, t_ref, t0_wall, boot_before, phase_ms):
    row = {"trial": i, "mode": args.mode, "planned_phase_ms": phase_ms}
    if args.mode == "broker":
        print(f"\n--- trial {i}/{args.trials}: stopping broker")
        watcher.stop()
        t_off = broker.stop()
        off_s = random.uniform(args.off_min, args.off_max)
        row["off_wall"] = wall(t_off, t_ref, t0_wall)
        row["off_planned_s"] = round(off_s, 1)
        if phase_ms is None:
            print(f"    broker down at {row['off_wall']}, restarting in {off_s:.1f} s")
            time.sleep(off_s)
        else:
            print(f"    broker down at {row['off_wall']}, restarting {phase_ms} ms after a "
                  f"connect attempt that starts after {off_s:.1f} s")
            time.sleep(off_s)
            att = None
            t_look = now()
            while att is None and now() < t_look + 15.0:
                att = ev.serial_after(t_look, "connecting")
                time.sleep(0.005)
            if att is None:
                print("    no connect attempt seen on serial within 15 s - restarting anyway")
            else:
                delay = att[0] + phase_ms / 1000.0 - now()
                if delay > 0:
                    time.sleep(delay)
        t_on = broker.start()
        watcher.start()
    else:
        input(f"\n--- trial {i}/{args.trials}: press Enter, THEN switch the hotspot OFF straight away ")
        t_off = now()
        row["off_wall"] = wall(t_off, t_ref, t0_wall)
        input("    wait until the robot reports the loss, then press Enter and switch the hotspot ON straight away ")
        t_on = now()
        monitor_ok = watcher.healthy()
    row["on_wall"] = wall(t_on, t_ref, t0_wall)

    deadline = t_on + args.give_up
    hb = None
    while now() < deadline:
        hb = ev.first_hb_after(t_on - 0.1)
        if hb:
            break
        time.sleep(0.02)
    row["recovery_hb_ms"] = max(0, ms(hb[0] - t_on)) if hb else None

    lost = ev.serial_after(t_off - 0.5, "lost") if args.serial else None
    row["detection_ms"] = ms(lost[0] - t_off) if lost else None
    row["lost_reason"] = lost[3].split("(")[-1].rstrip(")") if lost else None
    # 0.1 s tolerance: the robot can connect a moment before our port probe sees the broker.
    # The heartbeat (MQTT) can arrive before the serial line, so wait up to 3 s for it.
    conn = None
    if args.serial:
        t_wait = (hb[0] if hb else now()) + 3.0
        while conn is None and now() < t_wait:
            conn = ev.serial_after(t_on - 0.1, "connected")
            if conn is None:
                time.sleep(0.02)
    row["recovery_serial_ms"] = max(0, ms(conn[0] - t_on)) if conn else None
    row["connect_ms"] = conn[2] if conn else None
    if args.serial and conn:
        # attempt that succeeded started connect_ms before the "connected" line
        t_attempt = conn[0] - conn[2] / 1000.0
        row["attempt_started_vs_on_ms"] = ms(t_attempt - t_on)
    else:
        row["attempt_started_vs_on_ms"] = None

    # phase actually achieved: time from the robot's last attempt start to broker up
    if args.serial:
        att = ev.serial_last_before(t_on, "connecting", not_before=t_off)
        row["restore_phase_ms"] = ms(t_on - att[0]) if att else None
    else:
        row["restore_phase_ms"] = None
    row["boot_id_before"] = boot_before
    row["boot_id_after"] = hb[1] if hb else None

    rec = row["recovery_serial_ms"] if row["recovery_serial_ms"] is not None \
        else row["recovery_hb_ms"]
    # Hotspot mode: only a trial where the robot really went offline counts.
    # Proof: the robot printed LOST (with --serial), or no heartbeat arrived in the
    # 3 s before the "on" Enter (3 missed heartbeats = the laptop offline rule).
    outage_seen = True
    if args.mode == "hotspot":
        lost_in_window = lost is not None and lost[0] <= t_on
        # a quiet monitor only counts if the monitor itself was connected and subscribed
        quiet = monitor_ok and not ev.hb_between(t_on - 3.0, t_on)
        outage_seen = lost_in_window or quiet
    rebooted = hb is not None and boot_before is not None and hb[1] != boot_before
    late = False
    if args.mode == "hotspot" and lost is not None and lost[0] <= t_on:
        back = ev.serial_after(lost[0], "connected")
        late = back is not None and back[0] < t_on - 0.1
    if rebooted:
        row["result"] = "REBOOT"
        rec = None
        print(f"    REBOOT: robot came back with a new boot_id ({boot_before} -> {hb[1]}); "
              "not a reconnect, not counted (check power)")
    elif late:
        row["result"] = "LATE_MARK"
        rec = None
        row["recovery_serial_ms"] = None
        row["recovery_hb_ms"] = None
        print("    LATE_MARK: the robot was already back before your 'on' Enter, so recovery "
              "was not measured (press Enter first, then switch the hotspot on straight away)")
    elif not outage_seen:
        row["result"] = "INVALID"
        rec = None
        print("    INVALID: no proof the robot went offline (no LOST on serial and the laptop "
              "monitor was not both connected and quiet); leave the hotspot off longer")
    else:
        row["result"] = "TIMEOUT" if rec is None else ("PASS" if rec <= LIMIT_MS else "FAIL")
    print(f"    result {row['result']}: recovery {rec} ms"
          + (f", detection {row['detection_ms']} ms" if row["detection_ms"] is not None else "")
          + (f", connect {row['connect_ms']} ms (attempt started "
             f"{row['attempt_started_vs_on_ms']} ms vs broker up)"
             if row["connect_ms"] is not None else ""))
    if args.mode == "broker" and not args.serial:
        print("    (no --serial: detection not measured)")
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--mode", choices=["broker", "hotspot"], default="broker")
    ap.add_argument("--trials", type=int, default=5)
    ap.add_argument("--broker-cmd", default="mosquitto -c tools/mosquitto.conf -v",
                    help="command that runs the broker in the foreground")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--serial", help="robot USB serial port, e.g. COM5 (optional)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--off-min", type=float, default=5.0,
                    help="shortest broker-off time, s (random per trial, so the restart "
                         "lands at a different point of the robot's retry cycle)")
    ap.add_argument("--off-max", type=float, default=25.0)
    ap.add_argument("--steady", type=float, default=10.0,
                    help="seconds of heartbeats required before each outage")
    ap.add_argument("--give-up", type=float, default=60.0,
                    help="seconds to wait for the robot after the restart")
    ap.add_argument("--out", default=os.path.join("docs", "logs"))
    ap.add_argument("--seed", type=int, help="random seed (to repeat a run)")
    ap.add_argument("--phase", choices=["random", "sweep"], default="random",
                    help="sweep: restart at planned offsets after a robot connect attempt "
                         "(broker mode, needs --serial)")
    ap.add_argument("--cycle-ms", type=int, default=5000,
                    help="retry cycle covered by --phase sweep (3 s connect timeout + 2 s gap)")
    args = ap.parse_args()

    if args.off_min > args.off_max:
        ap.error("--off-min must not be larger than --off-max")
    if args.phase == "sweep":
        if args.mode != "broker" or not args.serial:
            ap.error("--phase sweep needs broker mode and --serial")
        # long enough off time that the robot is in its steady 2 s back-off
        args.off_min = max(args.off_min, 12.0)
        args.off_max = max(args.off_max, args.off_min)
    if args.seed is not None:
        random.seed(args.seed)
    os.makedirs(args.out, exist_ok=True)
    stamp = time.strftime("%Y-%m-%d_%H%M%S")
    base = os.path.join(args.out, f"outage_{stamp}")
    t_ref, t0_wall = now(), time.time()

    ev = Events()
    stop = threading.Event()
    if args.serial:
        threading.Thread(target=serial_reader, daemon=True,
                         args=(args.serial, args.baud, ev, base + "_serial.txt", stop)).start()

    broker = None
    if args.mode == "broker":
        if port_open(args.port):
            print(f"Port {args.port} is already in use: stop the Mosquitto service "
                  f"(net stop mosquitto) or any other broker, then run again.")
            sys.exit(2)
        # Windows: pass the string as is (CreateProcess parses quotes itself)
        cmd = args.broker_cmd if os.name == "nt" else shlex.split(args.broker_cmd)
        broker = Broker(cmd, args.port, base + "_broker.txt")
        broker.start()
    elif not port_open(args.port):
        print(f"Hotspot mode needs a broker already running on port {args.port}.")
        sys.exit(2)

    watcher = Watcher(args.port, ev)
    watcher.start()
    rows = []
    try:
        for i in range(1, args.trials + 1):
            print(f"\nwaiting for {args.steady:.0f} s of steady heartbeats from the robot ...")
            boot = wait_for_robot(ev, 120.0, args.steady)
            if boot is None:
                print("no steady heartbeats within 120 s - is the robot on and connected?")
                break
            phase = None
            if args.phase == "sweep":
                phase = int(round((i - 1) * args.cycle_ms / args.trials))
            rows.append(run_trial(i, args, ev, broker, watcher, t_ref, t0_wall, boot, phase))
    except KeyboardInterrupt:
        print("\nstopped by user")
    finally:
        watcher.stop()
        if broker:
            broker.stop()
        stop.set()

    if not rows:
        return
    fields = ["trial", "mode", "off_wall", "off_planned_s", "on_wall", "result",
              "recovery_serial_ms", "recovery_hb_ms", "connect_ms",
              "attempt_started_vs_on_ms", "planned_phase_ms", "restore_phase_ms",
              "detection_ms", "lost_reason", "boot_id_before", "boot_id_after"]
    with open(base + ".csv", "w", newline="", encoding="ascii") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)

    rec = [r["recovery_serial_ms"] if r["recovery_serial_ms"] is not None
           else r["recovery_hb_ms"] for r in rows if r["result"] in ("PASS", "FAIL")]
    rec_ok = [x for x in rec if x is not None]
    print("\n===== summary =====")
    print(f"mode {args.mode}, {len(rows)} trial(s): "
          f"PASS {sum(r['result'] == 'PASS' for r in rows)}, "
          f"FAIL {sum(r['result'] == 'FAIL' for r in rows)}, "
          f"TIMEOUT {sum(r['result'] == 'TIMEOUT' for r in rows)}, "
          f"INVALID {sum(r['result'] == 'INVALID' for r in rows)}, "
          f"LATE_MARK {sum(r['result'] == 'LATE_MARK' for r in rows)}, "
          f"REBOOT {sum(r['result'] == 'REBOOT' for r in rows)} (limit {LIMIT_MS} ms)")
    print("only PASS/FAIL/TIMEOUT trials count; report the others with their reason")
    if rec_ok:
        print(f"recovery ms: min {min(rec_ok)}  median {int(statistics.median(rec_ok))}  "
              f"max {max(rec_ok)}")
    det = [r["detection_ms"] for r in rows if r["detection_ms"] is not None]
    if det:
        print(f"detection ms: min {min(det)}  max {max(det)}")
    if args.mode == "hotspot":
        print("hotspot times run from your Enter marks, pressed before each switch: "
              "upper bounds (include your delay and hotspot start-up)")
    print("recovery source: robot's '[mqtt] connected' line when --serial is used, "
          "otherwise its first heartbeat")
    print(f"saved {base}.csv" + (f", {base}_serial.txt" if args.serial else "")
          + (f", {base}_broker.txt" if broker else ""))


if __name__ == "__main__":
    main()
