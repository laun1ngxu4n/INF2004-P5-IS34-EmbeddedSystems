#!/usr/bin/env python3
"""
Laptop side of the WiFi/MQTT bring-up test (P5-IS34, Buddy 1).

Shows every heartbeat with its receive interval and RSSI, flags anything
outside 1 s +/- 100 ms (NFR5b), prints OFFLINE after 3 missed heartbeats,
and measures command round-trip time and rejection rules (boot_id + seq).

    pip install paho-mqtt
    python mqtt_watch.py --host 127.0.0.1 --csv hb_log.csv

Keys (type the letter, then Enter). Send one at a time for RTT numbers.
    p   valid ping (new seq)                    -> expect "accepted"
    d   resend the last seq (duplicate)         -> expect "rejected_duplicate"
    o   wrong boot_id                           -> expect "rejected_old_boot"
    u   unknown command name (new seq)          -> expect "rejected_unknown_cmd"
    m   malformed JSON                          -> expect "rejected_malformed"
    b   600-byte command (over the 512 B limit) -> expect "rejected_oversize"
    r   burst of 8 valid pings back to back     -> robot queue is 4: expect
        accepted + a cmd_dropped event if the burst arrives in one poll
    q   quit and print the summary
"""
import argparse
import collections
import json
import threading
import time

import paho.mqtt.client as mqtt

HB_PERIOD_MS = 1000
HB_TOL_MS = 100
OFFLINE_AFTER_S = 3.0  # 3 missed heartbeats; checked every 50 ms

lock = threading.Lock()
st = {
    "t_start": None,
    "last_hb": None,
    "boot_id": None,
    "seq": 0,
    "intervals": [],
    "offline": False,
    "offline_since": None,
    "gaps": [],
    "pending": collections.defaultdict(collections.deque),  # seq -> send times
    "oversize_pending": collections.deque(),
    "malformed_pending": collections.deque(),
    "rtts": [],
    "acks": collections.Counter(),
    "unmatched_acks": 0,
    "dropped_total": 0,
    "rssi": [],
    "csv": None,
}


def ts():
    return time.strftime("%H:%M:%S")


def on_connect(client, userdata, *args):
    print(f"[{ts()}] connected to broker, subscribing robot/#")
    client.subscribe("robot/#", qos=1)


def handle_heartbeat(d, now):
    with lock:
        if st["boot_id"] != d.get("boot_id"):
            print(f"[{ts()}] robot boot_id = {d.get('boot_id')} (new boot, seq reset)")
            st["boot_id"] = d.get("boot_id")
            st["seq"] = 0
            st["last_hb"] = None
        if st["offline"]:
            gap = now - st["offline_since"]
            print(f"[{ts()}] ONLINE again (offline for {gap:.1f} s after the 3-miss point)")
            st["offline"] = False
        iv = None
        if st["last_hb"] is not None:
            iv = (now - st["last_hb"]) * 1000.0
            if iv > OFFLINE_AFTER_S * 1000.0:
                st["gaps"].append(iv)
            else:
                st["intervals"].append(iv)
        st["last_hb"] = now
        if d.get("rssi") is not None:
            st["rssi"].append(d.get("rssi"))
        if st["csv"]:
            st["csv"].write(f"{time.time():.3f},{d.get('seq')},"
                            f"{'' if iv is None else round(iv, 1)},{d.get('rssi')},"
                            f"{d.get('poll_period_ms')},{d.get('poll_max_us')},"
                            f"{d.get('act_max_us')}\n")
            st["csv"].flush()
    if iv is not None:
        flag = "" if abs(iv - HB_PERIOD_MS) <= HB_TOL_MS else "   <-- outside +/-100 ms"
        print(f"[{ts()}] hb seq={d.get('seq')} interval={iv:6.0f} ms rssi={d.get('rssi')} dBm "
              f"poll_period={d.get('poll_period_ms')} ms poll_max={d.get('poll_max_us')} us "
              f"act_max={d.get('act_max_us')} us{flag}")


def handle_event(d, now):
    if "ack" in d:
        with lock:
            st["acks"][d["ack"]] += 1
            sent = None
            if d["ack"] == "rejected_oversize" and st["oversize_pending"]:
                sent = st["oversize_pending"].popleft()
            elif d["ack"] == "rejected_malformed" and not d.get("seq_known", True) \
                    and st["malformed_pending"]:
                sent = st["malformed_pending"].popleft()
            elif d.get("seq_known", True):
                q = st["pending"].get(d.get("seq"))
                if q:
                    sent = q.popleft()
            if sent is None:
                st["unmatched_acks"] += 1
            elif d["ack"] == "accepted":
                st["rtts"].append((now - sent) * 1000.0)
        rtt = f"{(now - sent) * 1000.0:.0f} ms" if sent else "n/a (unmatched)"
        print(f"[{ts()}] ACK {d['ack']:<22} seq={d.get('seq')} round-trip={rtt} "
              f"robot handling={d.get('handle_us')} us")
    elif d.get("event") == "cmd_dropped":
        with lock:
            st["dropped_total"] = d.get("total", 0)
        print(f"[{ts()}] ROBOT DROPPED COMMANDS (queue full): total={d.get('total')}")
    else:
        print(f"[{ts()}] event: {d}")


def on_message(client, userdata, msg):
    now = time.monotonic()
    if msg.topic == "robot/heartbeat":
        try:
            handle_heartbeat(json.loads(msg.payload), now)
        except ValueError:
            print(f"[{ts()}] bad heartbeat payload: {msg.payload!r}")
    elif msg.topic == "robot/telemetry/events":
        try:
            handle_event(json.loads(msg.payload), now)
        except ValueError:
            print(f"[{ts()}] event: {msg.payload!r}")
    elif msg.topic == "robot/test/size":
        print(f"[{ts()}] size probe: received {len(msg.payload)} bytes")


def watchdog():
    while True:
        time.sleep(0.05)
        with lock:
            now = time.monotonic()
            ref = st["last_hb"]
            if ref is None and st["boot_id"] is None:
                ref = st["t_start"]  # never seen a heartbeat since we started
            if ref and not st["offline"] and (now - ref) > OFFLINE_AFTER_S:
                st["offline"] = True
                st["offline_since"] = now
                what = "3 heartbeats missed" if st["last_hb"] else "no heartbeat yet"
                print(f"[{ts()}] OFFLINE ({what})")


def send_raw(client, text, seq=None, oversize=False, malformed=False, label=""):
    with lock:
        t = time.monotonic()
        if oversize:
            st["oversize_pending"].append(t)
        elif malformed:
            st["malformed_pending"].append(t)
        elif seq is not None:
            st["pending"][seq].append(t)
    client.publish("robot/cmd/control", text, qos=1)
    print(f"[{ts()}] sent {label} ({len(text)} B)")


def send(client, boot_id, seq, cmd="ping", pad=0):
    body = {"boot_id": boot_id, "seq": seq, "cmd": cmd}
    if pad:
        body["pad"] = "x" * pad
    text = json.dumps(body)
    send_raw(client, text, seq=seq, oversize=len(text) > 512,
             label=f"seq={seq} boot_id={boot_id} cmd={cmd}")


def next_seq():
    with lock:
        st["seq"] += 1
        return st["seq"]


def summary():
    iv = st["intervals"]
    print("\n===== summary =====")
    if iv:
        inside = sum(1 for x in iv if abs(x - HB_PERIOD_MS) <= HB_TOL_MS)
        print(f"heartbeat receive intervals: n={len(iv)} mean={sum(iv) / len(iv):.1f} ms "
              f"min={min(iv):.0f} max={max(iv):.0f} within +/-100 ms: "
              f"{inside}/{len(iv)} ({100.0 * inside / len(iv):.1f} %)")
        print("  (receive intervals include WiFi/broker delay; the robot prints its own send jitter)")
    print(f"outage gaps (> {OFFLINE_AFTER_S} s): {[round(g / 1000.0, 1) for g in st['gaps']]} s")
    if st["rssi"]:
        r = st["rssi"]
        print(f"rssi: min={min(r)} max={max(r)} dBm over {len(r)} heartbeats")
    if st["rtts"]:
        r = sorted(st["rtts"])
        print(f"command round-trip (accepted only): n={len(r)} mean={sum(r) / len(r):.0f} ms "
              f"median={r[len(r) // 2]:.0f} ms max={max(r):.0f} ms")
    print(f"acks: {dict(st['acks'])}")
    left = (sum(len(q) for q in st["pending"].values()) + len(st["oversize_pending"])
            + len(st["malformed_pending"]))
    print(f"sent but never acked: {left}   acks with no matching send: {st['unmatched_acks']}   "
          f"robot-reported drops: {st['dropped_total']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--csv", help="append every heartbeat to this CSV file")
    args = ap.parse_args()

    if args.csv:
        st["csv"] = open(args.csv, "a", encoding="ascii")
        if st["csv"].tell() == 0:
            st["csv"].write(
                "unix_time,seq,interval_ms,rssi,poll_period_ms,poll_max_us,act_max_us\n")

    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="p5is34-laptop")
    except AttributeError:  # paho-mqtt 1.x
        client = mqtt.Client(client_id="p5is34-laptop")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.host, args.port, keepalive=30)
    st["t_start"] = time.monotonic()
    client.loop_start()
    threading.Thread(target=watchdog, daemon=True).start()

    print(__doc__.split("Keys")[1])
    try:
        while True:
            key = input().strip().lower()
            if not key:
                continue
            with lock:
                boot_id = st["boot_id"]
            if boot_id is None and key in ("p", "d", "o", "u", "m", "b", "r"):
                print("no heartbeat yet, boot_id unknown")
                continue
            if key == "p":
                send(client, boot_id, next_seq())
            elif key == "d":
                with lock:
                    seq = st["seq"]
                send(client, boot_id, seq)
            elif key == "o":
                send(client, boot_id + 1, next_seq())
            elif key == "u":
                send(client, boot_id, next_seq(), cmd="dance")
            elif key == "m":
                seq = next_seq()
                send_raw(client, '{"boot_id":%d,"seq":%d,"cmd":"ping",}' % (boot_id, seq),
                         malformed=True, label=f"malformed (trailing comma) seq={seq}")
            elif key == "b":
                send(client, boot_id, next_seq(), pad=600)
            elif key == "r":
                for _ in range(8):
                    send(client, boot_id, next_seq())
            elif key == "q":
                break
    except (KeyboardInterrupt, EOFError):
        pass
    summary()
    client.loop_stop()


if __name__ == "__main__":
    main()
