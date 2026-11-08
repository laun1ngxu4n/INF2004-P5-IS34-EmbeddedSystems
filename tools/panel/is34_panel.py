#!/usr/bin/env python3
"""
P5-IS34 robot control panel (laptop side of Stage 5 / the demo).

Shows the robot's 1 s status line (is34/telemetry), its live sensor
readings (is34/sensors, every 0.5 s) and every event
(is34/event, is34/ack/map), and sends START, STOP and the barcode mapping
(picked in the "Barcode mapping" box: which barcode means left, right,
straight, U-turn at the next junction)
(is34/cmd/start, is34/cmd/stop, is34/cmd/map).  Everything shown is also
written to logs/panel/is34_<date>_<time>.log (git-ignored).

Robot log tab: images built with MQTT send a copy of the whole console
(the same text as the USB console) on is34/log.  The tab shows it live;
"Save log..." writes everything since the last "Clear" to a text file
(send that file instead of a copied USB console).  It is also written
as it comes to logs/robot/is34_<date>_<time>_robot.txt (git-ignored).
START and STOP here also answer the stage tests' "press START" prompts.

    pip install paho-mqtt
    python tools/panel/is34_panel.py                  # broker on this laptop, port 1884
    python tools/panel/is34_panel.py --port 1883      # or another port / --host <ip>

The broker must accept connections from the hotspot.  "Start broker"
runs Mosquitto with tools/panel/mosquitto_robot.conf (port 1884, any address),
or run it yourself:
    "C:\\Program Files\\mosquitto\\mosquitto.exe" -c tools/panel/mosquitto_robot.conf -v

Keys: Space = STOP, Ctrl+G = START.

Change Requirement 1: the status, sensor and mapping-acknowledgement
messages are binary (is34_wire.py, docs/team/MQTT_WIRE_FORMAT.md); the
panel sends the mapping binary and START / STOP with an empty payload.
Messages from an older robot image (text) are still shown.
"""
import argparse
import math
import os
import queue
import random
import subprocess
import time
import tkinter as tk
from tkinter import filedialog, ttk

import paho.mqtt.client as mqtt

import is34_wire as wire

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))       # tools/panel -> the repo
LOG_DIR = os.path.join(REPO, "logs", "panel")        # what the panel showed
ROBOT_DIR = os.path.join(REPO, "logs", "robot")      # the robot's console
MOSQUITTO = r"C:\Program Files\mosquitto\mosquitto.exe"
OFFLINE_S = 3.0                 # no status line for this long = offline
LOG_VIEW_LINES = 4000           # lines kept in the Robot log view (the file keeps all)

LINE_NAMES = {"0": "ON", "1": "SEARCH", "2": "LOST", "3": "FAULT"}
STOP_BITS = wire.STOP_BITS
FIELDS = [                      # (telemetry key, label)
    ("state", "State"), ("speed", "Speed L,R (mm/s)"), ("dist", "Distance (mm)"),
    ("line", "Line"), ("junctions", "Junctions"), ("barcode", "Last barcode"),
    ("map", "Mapping"), ("ramp_peak", "Ramp peak (mm)"), ("stop", "Stop reason"),
    ("faults", "Faults / warnings"),
]
STATE_COLOURS = {"IDLE": "#888888", "CAL": "#d9a400", "RUN": "#2a9d3a"}


MOVES = [("L", "↰ Turn LEFT"), ("R", "↱ Turn RIGHT"), ("F", "↑ Go STRAIGHT"),
         ("U", "↶ U-TURN")]
CODE_CHARS = [chr(c) for c in range(ord("A"), ord("Z") + 1)] + [str(d) for d in range(10)]
MAP_FILE = os.path.join(HERE, "is34_panel_mapping.txt")   # the last one picked
MOVE_WORDS = {"L": "L", "LEFT": "L", "R": "R", "RIGHT": "R", "F": "F", "S": "F",
              "STRAIGHT": "F", "FORWARD": "F", "U": "U", "UTURN": "U", "U-TURN": "U",
              "UTURNS": "U", "BACK": "U"}


def parse_mapping(text):
    """'K=L,Q=R,X=F,M=U', 'K left, Q right, X straight, M u-turn',
    'left=K ...' -> {'L': 'K', ...}, or an error message"""
    import re
    parts = [p for p in re.split(r"[,;\n]+", text.strip()) if p.strip()]
    if len(parts) != 4:
        return f"{len(parts)} pairs, need 4"
    out = {}
    for p in parts:
        toks = [t for t in re.split(r"[\s=:>\-]+", p.strip().upper().replace("U-TURN", "UTURN"))
                if t]
        if len(toks) != 2:
            return f"'{p.strip()}' is not one barcode and one move"
        a, b = toks
        # the robot's own format is barcode=move; a whole word is always the move
        if b in MOVE_WORDS and a in CODE_CHARS:
            ch, mv = a, MOVE_WORDS[b]
        elif a in MOVE_WORDS and len(b) == 1 and b in CODE_CHARS:
            ch, mv = b, MOVE_WORDS[a]
        else:
            return f"'{p.strip()}' is not one barcode and one move"
        if mv in out:
            return f"two barcodes for the same move ({mv})"
        out[mv] = ch
    if len(set(out.values())) != 4:
        return "the same barcode twice"
    return out


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
    if key == "faults":
        try:
            return wire.bits_names(int(value), wire.FAULT_BITS)
        except ValueError:
            return value
    if key == "map" and value == "none":
        return "none (send one)"
    return value


def status_from_binary(d):
    """a decoded binary status message -> the text status line's fields"""
    return {
        "state": wire.MISSION_NAMES[d["mission"]],
        "speed": f"{d['speed_l']},{d['speed_r']}",
        "dist": str(d["dist_mm"]),
        "line": str(d["line"]),
        "junctions": str(d["junctions"]),
        "barcode": wire.letter(d["last"]) or "-",
        "map": wire.map_text(d) or "none",
        "ramp_peak": f"{d['ramp_peak_mm10'] // 10}.{d['ramp_peak_mm10'] % 10}",
        "stop": str(d["stop"]),
        "faults": str(d["faults"]),
    }


def sensors_from_binary(d):
    """a decoded binary sensors message -> the text sensor line's fields"""
    front = {0: d["front_mm"], 1: -1, 2: -2}.get(d["front_status"], -2)
    return {"ir": f"{d['ir_l']}{d['ir_r']}{d['ir_b']}", "steer": str(d["steer"]),
            "hold": str(d["hold"]), "inbar": str(d["inbar"]), "front": str(front),
            "tilt": str(d["tilt"]), "h10": str(d["h10"]), "imu": str(d["imu_ok"])}


class Panel:
    def __init__(self, root, host, port):
        self.root = root
        self.host, self.port = host, port
        self.q = queue.Queue()          # MQTT thread -> GUI thread
        self.last_status = 0.0
        self.broker_proc = None
        stamp = time.strftime("is34_%Y%m%d_%H%M%S")
        os.makedirs(LOG_DIR, exist_ok=True)
        os.makedirs(ROBOT_DIR, exist_ok=True)
        self.log = open(os.path.join(LOG_DIR, stamp + ".log"), "a", encoding="utf-8")
        self.robot_file = open(os.path.join(ROBOT_DIR, stamp + "_robot.txt"), "a",
                               encoding="utf-8")
        self.robot_text = []            # robot log since the last Clear
        self.robot_seq = None           # last is34/log message number
        self.robot_dropped = 0
        self.robot_lost = 0             # messages lost on the WiFi
        self.robot_lines = 0

        root.title("P5-IS34 robot panel")
        root.minsize(620, 520)
        self._build()

        # A name of its own: the broker drops a client when another one logs
        # in with the same name, so two panels with one name kept kicking
        # each other off once a second
        cid = f"is34-panel-{os.getpid()}-{random.randrange(1 << 16):04x}"
        try:                            # paho 2.x
            self.cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid)
        except AttributeError:          # paho 1.x
            self.cli = mqtt.Client(client_id=cid)
        self.cli.on_connect = self._on_connect
        self.cli.on_disconnect = lambda c, u, *rest: self.q.put(("conn", False))
        self.cli.on_message = lambda c, u, m: self.q.put(("msg", m.topic, bytes(m.payload)))
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

        # Barcode mapping: pick which barcode means which move, send it.
        # The robot stores the latest barcode it reads and does its move at
        # the NEXT junction (U-turn too: instructor brief, TEAM.md 7.1 B6).
        mp = ttk.LabelFrame(self.root, padding=6, text=(
            "Barcode mapping: after reading a barcode, the robot does this at the NEXT junction"))
        mp.pack(fill="x", padx=8)
        saved = self._load_mapping()
        self.map_combo = {}
        for col, (mv, label) in enumerate(MOVES):
            f = ttk.Frame(mp)
            f.grid(row=0, column=col, padx=(0, 18), sticky="w")
            ttk.Label(f, text=label, font=("Segoe UI", 10, "bold")).pack(anchor="w")
            r = ttk.Frame(f)
            r.pack(anchor="w")
            ttk.Label(r, text="barcode").pack(side="left")
            cb = ttk.Combobox(r, values=CODE_CHARS, width=3, state="readonly",
                              font=("Segoe UI", 12, "bold"))
            cb.set(saved[mv])
            cb.pack(side="left", padx=4)
            cb.bind("<<ComboboxSelected>>", lambda e: self._map_changed())
            self.map_combo[mv] = cb
        r2 = ttk.Frame(mp)
        r2.grid(row=1, column=0, columnspan=4, sticky="w", pady=(6, 0))
        tk.Button(r2, text="Send mapping to robot", bg="#1565c0", fg="white",
                  font=("Segoe UI", 10, "bold"), command=self.send_map).pack(side="left")
        ttk.Button(r2, text="Test set: A B C D",
                   command=lambda: self._set_map(dict(L="A", R="B", F="C", U="D"))
                   ).pack(side="left", padx=6)
        ttk.Label(r2, text="or paste one (e.g. K=L,Q=R,X=F,M=U):").pack(side="left", padx=(10, 2))
        self.map_var = tk.StringVar()
        ent = ttk.Entry(r2, textvariable=self.map_var, width=20)
        ent.pack(side="left")
        ent.bind("<Return>", lambda e: self._paste_map())
        ttk.Button(r2, text="Use", command=self._paste_map).pack(side="left", padx=2)
        r3 = ttk.Frame(mp)
        r3.grid(row=2, column=0, columnspan=4, sticky="w", pady=(4, 0))
        self.map_msg = tk.Label(r3, text="", anchor="w")
        self.map_msg.pack(side="left")
        self.robot_map_lbl = tk.Label(r3, text="robot: no mapping yet", fg="#b00020")
        self.robot_map_lbl.pack(side="left", padx=12)
        self.ack_lbl = ttk.Label(r3, text="")
        self.ack_lbl.pack(side="left", padx=8)
        self.robot_map = ""
        self._map_changed()

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

        sn = ttk.LabelFrame(self.root, text="Live sensors (every 0.5 s)", padding=8)
        sn.pack(fill="x", padx=8, pady=(0, 6))
        self.sens = {}
        for col, (key, label) in enumerate([("line", "Line + barcode"),
                                            ("obst", "Obstacle (sonar)"),
                                            ("imu", "IMU")]):
            ttk.Label(sn, text=label).grid(row=0, column=col, sticky="w", padx=6)
            v = tk.Label(sn, text="-", font=("Consolas", 11), anchor="w", justify="left")
            v.grid(row=1, column=col, sticky="nw", padx=6)
            self.sens[key] = v
            sn.columnconfigure(col, weight=1)

        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        ev = ttk.Frame(nb, padding=4)
        nb.add(ev, text="Events")
        self.events = tk.Text(ev, height=12, state="disabled", font=("Consolas", 10))
        sb = ttk.Scrollbar(ev, command=self.events.yview)
        self.events.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.events.pack(fill="both", expand=True)

        rl = ttk.Frame(nb, padding=4)
        nb.add(rl, text="Robot log")
        bar = ttk.Frame(rl)
        bar.pack(fill="x", pady=(0, 4))
        ttk.Button(bar, text="Save log...", command=self.save_robot_log).pack(side="left")
        ttk.Button(bar, text="Clear", command=self.clear_robot_log).pack(side="left", padx=4)
        self.follow_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="follow", variable=self.follow_var).pack(side="left", padx=4)
        self.robot_lbl2 = ttk.Label(bar, text="nothing yet (needs an MQTT image)")
        self.robot_lbl2.pack(side="left", padx=8)
        self.rlog = tk.Text(rl, height=12, state="disabled", font=("Consolas", 9), wrap="none")
        sb2 = ttk.Scrollbar(rl, command=self.rlog.yview)
        self.rlog.configure(yscrollcommand=sb2.set)
        sb2.pack(side="right", fill="y")
        self.rlog.pack(fill="both", expand=True)

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
            # none on this laptop yet: start one (once), then connect again
            if self.host in ("127.0.0.1", "localhost") and self.broker_proc is None:
                self.start_broker()

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

    def publish(self, topic, payload, shown=None):
        r = self.cli.publish(topic, payload, qos=1)
        ok = r.rc == mqtt.MQTT_ERR_SUCCESS
        if shown is None:
            shown = payload.hex(" ") if isinstance(payload, bytes) else payload
        self.add_event("sent", f"{topic} {shown}" + ("" if ok else
                       "  (NOT SENT: not connected to the broker, press Start broker)"))
        if ok and time.time() - self.last_status > OFFLINE_S:
            self.add_event("panel", "warning: the robot is not online, it will not see this")

    def start(self):
        self.publish("is34/cmd/start", b"", shown="(the topic is the command)")

    def stop(self):
        self.publish("is34/cmd/stop", b"", shown="(the topic is the command)")

    # ---- barcode mapping ---------------------------------------------------
    def _map_choice(self):
        """{'L': 'A', ...} from the four boxes"""
        return {mv: self.map_combo[mv].get() for mv, _ in MOVES}

    def _map_text(self):
        """the robot's format 'A=L,B=R,C=F,D=U', or None if two moves share a barcode"""
        ch = self._map_choice()
        if len(set(ch.values())) != 4:
            return None
        return ",".join(f"{ch[mv]}={mv}" for mv, _ in MOVES)

    def _map_changed(self):
        text = self._map_text()
        if text is None:
            self.map_msg.configure(text="each move needs a different barcode", fg="#b00020")
        else:
            self.map_msg.configure(text=f"will send: {text}", fg="#444444")
            self._save_mapping()
        self._show_robot_map()

    def _set_map(self, choice):
        for mv, _ in MOVES:
            self.map_combo[mv].set(choice[mv])
        self._map_changed()

    def _paste_map(self):
        choice = parse_mapping(self.map_var.get())
        if isinstance(choice, str):
            self.map_msg.configure(text=f"could not read that: {choice}", fg="#b00020")
            return
        self._set_map(choice)

    def _show_robot_map(self):
        """what the robot has (its status line), against what is picked here"""
        have = self.robot_map
        if not have or have == "none":
            self.robot_map_lbl.configure(text="robot: no mapping yet (press Send)", fg="#b00020")
            return
        want = self._map_text()
        same = want is not None and set(want.split(",")) == set(have.split(","))
        self.robot_map_lbl.configure(
            text=f"robot has: {have}" + ("  (same)" if same else "  (different: press Send)"),
            fg="#2a9d3a" if same else "#d9a400")

    def _load_mapping(self):
        try:
            with open(MAP_FILE, encoding="utf-8") as f:
                choice = parse_mapping(f.read())
            if isinstance(choice, dict):
                return choice
        except OSError:
            pass
        return dict(L="A", R="B", F="C", U="D")

    def _save_mapping(self):
        text = self._map_text()
        if text:
            try:
                with open(MAP_FILE, "w", encoding="utf-8") as f:
                    f.write(text + "\n")
            except OSError:
                pass

    def send_map(self):
        text = self._map_text()
        if text is None:
            self.map_msg.configure(text="not sent: each move needs a different barcode",
                                   fg="#b00020")
            return
        self.ack_lbl.configure(text="waiting for the robot ...", foreground="#666666")
        payload = wire.pack_map(self._map_choice())
        self.publish("is34/cmd/map", payload, shown=f"{text} = {payload.hex(' ')}")

    def save_robot_log(self):
        if not self.robot_text:
            self.add_event("panel", "robot log is empty, nothing saved")
            return
        path = filedialog.asksaveasfilename(
            parent=self.root, title="Save robot log", defaultextension=".txt",
            initialdir=ROBOT_DIR, initialfile=time.strftime("robot_log_%Y%m%d_%H%M.txt"),
            filetypes=[("Text", "*.txt"), ("All files", "*.*")])
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            f.write("".join(self.robot_text))
        self.add_event("panel", f"robot log saved: {path}")

    def clear_robot_log(self):
        self.robot_text = []
        self.robot_lost = self.robot_dropped = self.robot_lines = 0
        self.rlog.configure(state="normal")
        self.rlog.delete("1.0", "end")
        self.rlog.configure(state="disabled")
        self._robot_label()

    def _robot_label(self):
        n = self.robot_lines
        warn = []
        if self.robot_lost:
            warn.append(f"{self.robot_lost} messages lost on the WiFi")
        if self.robot_dropped:
            warn.append(f"{self.robot_dropped} bytes dropped on the robot")
        self.robot_lbl2.configure(text=f"{n} lines" + (" - " + ", ".join(warn) if warn else ""),
                                  foreground="#b00020" if warn else "")

    def _robot_log(self, text):
        # "L<seq> <dropped>|<text>"
        head, sep, body = text.partition("|")
        note = ""
        if sep and head.startswith("L"):
            try:
                seq, dropped = (int(x) for x in head[1:].split())
            except ValueError:
                seq, dropped, body = None, self.robot_dropped, text
            if seq is not None:
                if self.robot_seq is not None and seq <= self.robot_seq:
                    note = "\n[panel: the robot restarted]\n"
                    self.robot_dropped = 0
                elif self.robot_seq is not None and seq > self.robot_seq + 1:
                    gap = seq - self.robot_seq - 1
                    self.robot_lost += gap
                    note = f"\n[panel: {gap} log messages lost on the WiFi]\n"
                if dropped > self.robot_dropped:
                    note += (f"\n[panel: the robot dropped {dropped - self.robot_dropped} "
                             f"bytes, its buffer was full]\n")
                self.robot_dropped = dropped
                self.robot_seq = seq
        else:
            body = text
        chunk = note + body
        self.robot_text.append(chunk)
        self.robot_lines += chunk.count("\n")
        self.robot_file.write(chunk)
        self.robot_file.flush()
        self.rlog.configure(state="normal")
        self.rlog.insert("end", chunk)
        extra = int(self.rlog.index("end-1c").split(".")[0]) - LOG_VIEW_LINES
        if extra > 0:
            self.rlog.delete("1.0", f"{extra + 1}.0")
        if self.follow_var.get():
            self.rlog.see("end")
        self.rlog.configure(state="disabled")
        self._robot_label()

    def close(self):
        try:
            self.cli.loop_stop()
            self.cli.disconnect()
        except Exception:
            pass
        if self.broker_proc and self.broker_proc.poll() is None:
            self.broker_proc.terminate()
        self.log.close()
        self.robot_file.close()
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

    def _message(self, topic, payload):
        text = payload.decode("utf-8", "replace")
        if topic == "is34/telemetry":
            self.last_status = time.time()
            d = wire.unpack_status(payload)
            fields = status_from_binary(d) if d else parse_status(text)
            shown = " ".join(f"{k}={v}" for k, v in fields.items()) if d else text
            self.log.write(time.strftime("%H:%M:%S ") + f"{topic} {shown}\n")
            for key, val in fields.items():
                if key in self.vals:
                    self.vals[key].configure(text=pretty(key, val))
            state = fields.get("state", "")
            self.vals["state"].configure(fg=STATE_COLOURS.get(state, "#b00020"))
            robot_map = fields.get("map", "")
            if robot_map != self.robot_map:
                self.robot_map = robot_map
                self._show_robot_map()
        elif topic == "is34/sensors":
            d = wire.unpack_sensors(payload)
            fields = sensors_from_binary(d) if d else parse_status(text)
            shown = " ".join(f"{k}={v}" for k, v in fields.items()) if d else text
            self.log.write(time.strftime("%H:%M:%S ") + f"{topic} {shown}\n")
            self._sensors(fields)
        elif topic == "is34/ack/map":
            d = wire.unpack_ack(payload)
            if d:
                good = bool(d["ok"])
                have = wire.map_text(d) or "none"
                text = (f"OK {have}" if good else
                        f"REJECTED (the robot keeps {have}; why: see Events)")
            else:
                good = text.startswith("OK")
            self.ack_lbl.configure(text=text, foreground="#2a9d3a" if good else "#b00020")
            self.add_event("robot", f"mapping: {text}")
        elif topic == "is34/event":
            self.add_event("robot", text)
        elif topic == "is34/log":
            self._robot_log(text)
        elif not topic.startswith("is34/cmd/"):     # our own commands are echoed back
            self.add_event("robot", f"{topic} {text}")


    def _sensors(self, d):
        def num(key):
            try:
                return int(d.get(key, ""))
            except ValueError:
                return None

        ir = d.get("ir", "???")
        dot = lambda c: "●" if c == "1" else "○"      # filled = black
        self.sens["line"].configure(text=(
            f"L {dot(ir[0:1])}  R {dot(ir[1:2])}  B {dot(ir[2:3])}\n"
            f"steer {num('steer')} mm/s\n"
            f"in barcode: {'yes' if d.get('inbar') == '1' else 'no'}\n"
            f"steering held (B on bars): {'yes' if d.get('hold') == '1' else 'no'}"))
        front = num("front")
        front_txt = {None: "?", -1: "nothing in range", -2: "NO ECHO (fault)"}.get(
            front, f"{front} mm")
        self.sens["obst"].configure(text=f"front {front_txt}")
        tilt, h10 = num("tilt"), num("h10")
        if tilt is not None and h10 is not None:
            deg = math.degrees(math.asin(max(-1.0, min(1.0, tilt / 1000.0))))
            self.sens["imu"].configure(text=(
                f"tilt {deg:+.1f} deg\nclimbed {h10 / 10:.1f} mm\n"
                f"{'OK' if d.get('imu') == '1' else 'NOT ANSWERING'}"))

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
