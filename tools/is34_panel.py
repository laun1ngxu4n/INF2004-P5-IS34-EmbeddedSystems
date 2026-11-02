#!/usr/bin/env python3
"""
P5-IS34 robot control panel (laptop side of Stage 5 / the demo).

Shows the robot's 1 s status line (is34/telemetry) and every event
(is34/event, is34/ack/map), and sends START, STOP and the 4-pair mapping
(is34/cmd/start, is34/cmd/stop, is34/cmd/map).  Everything shown is also
written to a log file next to this script (is34_<date>_<time>.log).

    pip install paho-mqtt
    python tools/is34_panel.py                  # broker on this laptop, port 1884
    python tools/is34_panel.py --port 1883      # or another port / --host <ip>

The broker must accept connections from the hotspot.  "Start broker"
runs Mosquitto with tools/mosquitto_robot.conf (port 1884, any address),
or run it yourself:
    "C:\\Program Files\\mosquitto\\mosquitto.exe" -c tools/mosquitto_robot.conf -v

Keys: Space = STOP, Ctrl+G = START.
"""
import argparse
import os
import queue
import subprocess
import time
import tkinter as tk
from tkinter import ttk

import paho.mqtt.client as mqtt

HERE = os.path.dirname(os.path.abspath(__file__))
MOSQUITTO = r"C:\Program Files\mosquitto\mosquitto.exe"
OFFLINE_S = 3.0                 # no status line for this long = offline

LINE_NAMES = {"0": "ON", "1": "SEARCH", "2": "LOST"}
STOP_BITS = {1: "button", 2: "laptop", 4: "fault"}
FIELDS = [                      # (telemetry key, label)
    ("state", "State"), ("speed", "Speed L,R (mm/s)"), ("dist", "Distance (mm)"),
    ("line", "Line"), ("junctions", "Junctions"), ("barcode", "Last barcode"),
    ("map", "Mapping"), ("ramp_peak", "Ramp peak (mm)"), ("stop", "Stop reason"),
]
STATE_COLOURS = {"IDLE": "#888888", "CAL": "#d9a400", "RUN": "#2a9d3a"}


def parse_status(text):
    """'state=RUN speed=150,150 map=K=L,...' -> dict (split on the first '=')"""
    out = {}
    for part in text.split():
        if "=" in part:
            k, v = part.split("=", 1)
            out[k] = v
    return out


def pretty(key, value):
    if key == "line":
        return LINE_NAMES.get(value, value)
    if key == "stop":
        try:
            bits = int(value)
        except ValueError:
            return value
        if bits == 0:
            return "none"
        return ", ".join(n for b, n in STOP_BITS.items() if bits & b)
    if key == "map" and value == "none":
        return "none (send one)"
    return value


class Panel:
    def __init__(self, root, host, port):
        self.root = root
        self.host, self.port = host, port
        self.q = queue.Queue()          # MQTT thread -> GUI thread
        self.last_status = 0.0
        self.broker_proc = None
        self.log = open(os.path.join(HERE, time.strftime("is34_%Y%m%d_%H%M%S.log")),
                        "a", encoding="utf-8")

        root.title("P5-IS34 robot panel")
        root.minsize(620, 520)
        self._build()

        try:                            # paho 2.x
            self.cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="is34-panel")
        except AttributeError:          # paho 1.x
            self.cli = mqtt.Client(client_id="is34-panel")
        self.cli.on_connect = self._on_connect
        self.cli.on_disconnect = lambda c, u, *rest: self.q.put(("conn", False))
        self.cli.on_message = lambda c, u, m: self.q.put(
            ("msg", m.topic, m.payload.decode("utf-8", "replace")))
        self.connect()

        root.bind("<space>", lambda e: self.stop())
        root.bind("<Control-g>", lambda e: self.start())
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(100, self._poll)

    # ---- layout ----------------------------------------------------------
    def _build(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="Broker").pack(side="left")
        self.host_var = tk.StringVar(value=self.host)
        self.port_var = tk.StringVar(value=str(self.port))
        ttk.Entry(top, textvariable=self.host_var, width=15).pack(side="left", padx=4)
        ttk.Entry(top, textvariable=self.port_var, width=6).pack(side="left")
        ttk.Button(top, text="Connect", command=self.connect).pack(side="left", padx=4)
        ttk.Button(top, text="Start broker", command=self.start_broker).pack(side="left")
        self.conn_lbl = tk.Label(top, text="not connected", fg="#b00020")
        self.conn_lbl.pack(side="left", padx=8)
        self.robot_lbl = tk.Label(top, text="robot: no data", fg="#b00020")
        self.robot_lbl.pack(side="right")

        btns = ttk.Frame(self.root, padding=(8, 0))
        btns.pack(fill="x")
        tk.Button(btns, text="START", bg="#2a9d3a", fg="white", font=("Segoe UI", 16, "bold"),
                  width=10, command=self.start).pack(side="left", pady=6)
        tk.Button(btns, text="STOP  (Space)", bg="#c62828", fg="white",
                  font=("Segoe UI", 16, "bold"), width=14,
                  command=self.stop).pack(side="left", padx=8, pady=6)

        mp = ttk.Frame(self.root, padding=(8, 0))
        mp.pack(fill="x")
        ttk.Label(mp, text="Mapping (4 pairs, L R F U)").pack(side="left")
        self.map_var = tk.StringVar(value="K=L,Q=R,X=F,M=U")
        ent = ttk.Entry(mp, textvariable=self.map_var, width=24)
        ent.pack(side="left", padx=4)
        ent.bind("<Return>", lambda e: self.send_map())
        ttk.Button(mp, text="Send mapping", command=self.send_map).pack(side="left")
        self.ack_lbl = ttk.Label(mp, text="")
        self.ack_lbl.pack(side="left", padx=8)

        st = ttk.LabelFrame(self.root, text="Status (every 1 s)", padding=8)
        st.pack(fill="x", padx=8, pady=6)
        self.vals = {}
        for i, (key, label) in enumerate(FIELDS):
            ttk.Label(st, text=label).grid(row=i // 3 * 2, column=i % 3, sticky="w", padx=6)
            v = tk.Label(st, text="-", font=("Segoe UI", 13, "bold"), anchor="w")
            v.grid(row=i // 3 * 2 + 1, column=i % 3, sticky="w", padx=6, pady=(0, 6))
            self.vals[key] = v
        for c in range(3):
            st.columnconfigure(c, weight=1)

        ev = ttk.LabelFrame(self.root, text="Events", padding=4)
        ev.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.events = tk.Text(ev, height=12, state="disabled", font=("Consolas", 10))
        sb = ttk.Scrollbar(ev, command=self.events.yview)
        self.events.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.events.pack(fill="both", expand=True)

    # ---- actions -----------------------------------------------------------
    def connect(self):
        self.host, self.port = self.host_var.get().strip(), int(self.port_var.get())
        try:
            self.cli.disconnect()
        except Exception:
            pass
        try:
            self.cli.connect(self.host, self.port, keepalive=30)
            self.cli.loop_start()
        except OSError as e:
            self.add_event("panel", f"cannot reach broker {self.host}:{self.port}: {e}")

    def start_broker(self):
        conf = os.path.join(HERE, "mosquitto_robot.conf")
        if self.broker_proc and self.broker_proc.poll() is None:
            self.add_event("panel", "broker already started from this panel")
            return
        try:
            self.broker_proc = subprocess.Popen(
                [MOSQUITTO, "-c", conf], creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            self.add_event("panel", f"broker started (port 1884, {conf})")
            self.root.after(1000, self.connect)
        except OSError as e:
            self.add_event("panel", f"could not start Mosquitto: {e}")

    def publish(self, topic, payload):
        r = self.cli.publish(topic, payload, qos=1)
        ok = r.rc == mqtt.MQTT_ERR_SUCCESS
        self.add_event("sent", f"{topic} {payload}" + ("" if ok else "  (NOT SENT: not connected)"))

    def start(self):
        self.publish("is34/cmd/start", "go")

    def stop(self):
        self.publish("is34/cmd/stop", "now")

    def send_map(self):
        self.ack_lbl.configure(text="waiting for the robot ...", foreground="#666666")
        self.publish("is34/cmd/map", self.map_var.get().strip())

    def close(self):
        try:
            self.cli.loop_stop()
            self.cli.disconnect()
        except Exception:
            pass
        if self.broker_proc and self.broker_proc.poll() is None:
            self.broker_proc.terminate()
        self.log.close()
        self.root.destroy()

    # ---- MQTT -> GUI -------------------------------------------------------
    def _on_connect(self, c, u, flags, rc, *props):     # paho 1.x and 2.x
        if rc == 0:
            c.subscribe([("is34/#", 1)])
        self.q.put(("conn", rc == 0))

    def _poll(self):
        while not self.q.empty():
            item = self.q.get()
            if item[0] == "conn":
                up = item[1]
                self.conn_lbl.configure(text=f"broker {self.host}:{self.port}" if up
                                        else "not connected", fg="#2a9d3a" if up else "#b00020")
                self.add_event("panel", "connected to the broker" if up else "broker disconnected")
            else:
                self._message(item[1], item[2])
        age = time.time() - self.last_status
        if self.last_status == 0.0:
            self.robot_lbl.configure(text="robot: no data", fg="#b00020")
        elif age > OFFLINE_S:
            self.robot_lbl.configure(text=f"robot: OFFLINE ({age:.0f} s)", fg="#b00020")
        else:
            self.robot_lbl.configure(text="robot: online", fg="#2a9d3a")
        self.root.after(100, self._poll)

    def _message(self, topic, text):
        if topic == "is34/telemetry":
            self.last_status = time.time()
            self.log.write(time.strftime("%H:%M:%S ") + f"{topic} {text}\n")
            for key, val in parse_status(text).items():
                if key in self.vals:
                    self.vals[key].configure(text=pretty(key, val))
            state = parse_status(text).get("state", "")
            self.vals["state"].configure(fg=STATE_COLOURS.get(state, "#b00020"))
        elif topic == "is34/ack/map":
            good = text.startswith("OK")
            self.ack_lbl.configure(text=text, foreground="#2a9d3a" if good else "#b00020")
            self.add_event("robot", f"mapping: {text}")
        elif topic == "is34/event":
            self.add_event("robot", text)
        elif not topic.startswith("is34/cmd/"):     # our own commands are echoed back
            self.add_event("robot", f"{topic} {text}")

    def add_event(self, who, text):
        line = time.strftime("%H:%M:%S ") + f"[{who}] {text}"
        self.log.write(line + "\n")
        self.log.flush()
        self.events.configure(state="normal")
        self.events.insert("end", line + "\n")
        self.events.see("end")
        self.events.configure(state="disabled")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--host", default="127.0.0.1", help="broker address (default this laptop)")
    ap.add_argument("--port", type=int, default=1884, help="broker port (default 1884)")
    a = ap.parse_args()
    root = tk.Tk()
    Panel(root, a.host, a.port)
    root.mainloop()


if __name__ == "__main__":
    main()
