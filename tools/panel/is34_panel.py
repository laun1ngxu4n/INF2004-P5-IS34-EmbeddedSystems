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
"Save log" writes everything since the last "Clear" to a new text file in logs/robot
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

Look: a dark and a light theme ("Light" / "Dark" button, top right); the
choice is kept in is34_panel_theme.txt next to the mapping file.

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
import sys
import time
import tkinter as tk
from tkinter import ttk

import paho.mqtt.client as mqtt

import is34_wire as wire
import is34_summary as summ

if getattr(sys, "frozen", False):
    # The packaged is34_panel.exe (tools/panel/build_exe.bat): its files are
    # unpacked to a temporary folder, so the broker conf comes from there and
    # everything written (logs, last mapping) goes next to the .exe.
    HERE = sys._MEIPASS                                  # bundled files
    WRITE = os.path.dirname(os.path.abspath(sys.executable))
    LOG_DIR = os.path.join(WRITE, "logs", "panel")
    ROBOT_DIR = os.path.join(WRITE, "logs", "robot")
else:
    HERE = os.path.dirname(os.path.abspath(__file__))
    WRITE = HERE
    REPO = os.path.dirname(os.path.dirname(HERE))   # tools/panel -> the repo
    LOG_DIR = os.path.join(REPO, "logs", "panel")    # what the panel showed
    ROBOT_DIR = os.path.join(REPO, "logs", "robot")  # the robot's console
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
STATE_TONES = {"IDLE": "idle", "CAL": "warn", "RUN": "ok"}   # anything else: bad

FONT = "Segoe UI"
MONO = "Consolas"
# Colours by role; every widget is painted from the current theme, so the
# theme can be switched while running.
THEMES = {
    "dark": dict(bg="#0e1318", card="#161c23", tile="#1d252e", line="#29333e",
                 text="#e6edf3", muted="#8b98a5", accent="#3b82f6", accent_hi="#5b9bff",
                 ok="#22c55e", warn="#f5b301", bad="#f05252", idle="#8b98a5",
                 start="#16a34a", start_hi="#22c55e", stop="#dc2626", stop_hi="#ef4444",
                 field="#0e1318", log="#0b0f13", btn="#26303b", btn_hi="#323e4b",
                 dot_off="#33404d", sel="#264f78", white="#ffffff"),
    "light": dict(bg="#eef2f6", card="#ffffff", tile="#f4f6f9", line="#dde3ea",
                  text="#0f172a", muted="#64748b", accent="#2563eb", accent_hi="#3b82f6",
                  ok="#15803d", warn="#b45309", bad="#dc2626", idle="#64748b",
                  start="#16a34a", start_hi="#22c55e", stop="#dc2626", stop_hi="#ef4444",
                  field="#ffffff", log="#fafbfc", btn="#e8edf3", btn_hi="#dbe2ea",
                  dot_off="#d3dbe4", sel="#bfdbfe", white="#ffffff"),
}
BUTTON_KINDS = {                # kind -> (background, hover, text) roles
    "start": ("start", "start_hi", "white"), "stop": ("stop", "stop_hi", "white"),
    "accent": ("accent", "accent_hi", "white"), "plain": ("btn", "btn_hi", "text"),
}


MOVES = [("L", "↰ Turn LEFT"), ("R", "↱ Turn RIGHT"), ("F", "↑ Go STRAIGHT"),
         ("U", "↶ U-TURN")]
CODE_CHARS = [chr(c) for c in range(ord("A"), ord("Z") + 1)] + [str(d) for d in range(10)]
MOVE_UI = {"L": ("↰", "Turn LEFT"), "R": ("↱", "Turn RIGHT"), "F": ("↑", "Go STRAIGHT"),
           "U": ("↶", "U-TURN")}
MAP_FILE = os.path.join(WRITE, "is34_panel_mapping.txt")   # the last one picked
THEME_FILE = os.path.join(WRITE, "is34_panel_theme.txt")   # "dark" or "light"
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
        self.summary = summ.Summary()   # the results that matter, per run

        root.title("P5-IS34 robot panel")
        # Sizes below are for a 100 % screen; scale them to this one
        # (fonts in points scale by themselves), and never past the screen.
        self.s = s = max(1.0, root.winfo_fpixels("1i") / 96.0)
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        w, h = min(int(1000 * s), sw - 40), min(int(820 * s), sh - 90)
        root.geometry(f"{w}x{h}")
        root.minsize(min(int(900 * s), w), min(int(680 * s), h))
        self.theme = self._load_theme()
        self.pal = THEMES[self.theme]
        self._paint = []                # (widget, {option: colour role})
        self._tones = {}                # widget -> colour role of its text
        self.ir = {"L": False, "R": False, "B": False}   # True = sees black
        self.style = ttk.Style(root)
        self.style.theme_use("clam")    # the one ttk theme that takes colours
        self._build()
        self._apply_theme()

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

    # ---- theme -------------------------------------------------------------
    def _load_theme(self):
        try:
            with open(THEME_FILE, encoding="utf-8") as f:
                name = f.read().strip()
            if name in THEMES:
                return name
        except OSError:
            pass
        return "dark"

    def toggle_theme(self):
        self.theme = "light" if self.theme == "dark" else "dark"
        try:
            with open(THEME_FILE, "w", encoding="utf-8") as f:
                f.write(self.theme + "\n")
        except OSError:
            pass
        self._apply_theme()

    def _reg(self, w, **roles):
        """paint w from the theme now and on every switch: option=role"""
        self._paint.append((w, roles))
        w.configure(**{k: self.pal[v] for k, v in roles.items()})
        return w

    def _tone(self, w, tone, text=None):
        """set a label's text colour by role (kept across theme switches)"""
        self._tones[w] = tone
        kw = {"fg": self.pal[tone]}
        if text is not None:
            kw["text"] = text
        w.configure(**kw)

    def _apply_theme(self):
        p = self.pal = THEMES[self.theme]
        self.root.configure(bg=p["bg"])
        for w, roles in self._paint:
            w.configure(**{k: p[v] for k, v in roles.items()})
        for w, tone in self._tones.items():
            w.configure(fg=p[tone])
        st = self.style
        st.configure(".", background=p["card"], foreground=p["text"], fieldbackground=p["field"],
                     bordercolor=p["line"], lightcolor=p["line"], darkcolor=p["line"],
                     troughcolor=p["card"], selectbackground=p["sel"],
                     selectforeground=p["text"], insertcolor=p["text"], arrowcolor=p["muted"],
                     font=(FONT, 10))
        st.configure("TFrame", background=p["card"])
        st.configure("TEntry", fieldbackground=p["field"], foreground=p["text"], padding=4)
        st.map("TEntry", bordercolor=[("focus", p["accent"])], lightcolor=[("focus", p["accent"])])
        st.configure("TCombobox", fieldbackground=p["field"], background=p["btn"],
                     foreground=p["text"], arrowcolor=p["text"], padding=3)
        st.map("TCombobox",
               fieldbackground=[("readonly", p["field"])], foreground=[("readonly", p["text"])],
               selectbackground=[("readonly", p["field"])],
               selectforeground=[("readonly", p["text"])],
               background=[("active", p["btn_hi"]), ("readonly", p["btn"])],
               bordercolor=[("focus", p["accent"])], lightcolor=[("focus", p["accent"])])
        st.configure("TNotebook", background=p["bg"], borderwidth=0, tabmargins=(0, 0, 0, 0))
        st.configure("TNotebook.Tab", background=p["tile"], foreground=p["muted"],
                     padding=(16, 6), borderwidth=0, font=(FONT, 10, "bold"))
        st.map("TNotebook.Tab", background=[("selected", p["card"])],
               foreground=[("selected", p["text"])], lightcolor=[("selected", p["card"])],
               bordercolor=[("selected", p["line"])])
        st.configure("Vertical.TScrollbar", background=p["btn"], troughcolor=p["log"],
                     bordercolor=p["log"], arrowcolor=p["muted"], lightcolor=p["btn"],
                     darkcolor=p["btn"])
        st.map("Vertical.TScrollbar", background=[("active", p["btn_hi"])])
        st.configure("TCheckbutton", background=p["card"], foreground=p["text"],
                     indicatorbackground=p["field"], indicatorforeground=p["white"])
        st.map("TCheckbutton", background=[("active", p["card"])],
               indicatorbackground=[("selected", p["accent"])])
        for t in (self.sum_text, self.events, self.rlog):
            t.configure(bg=p["log"], fg=p["text"], insertbackground=p["text"],
                        selectbackground=p["sel"], selectforeground=p["text"],
                        highlightbackground=p["line"], highlightcolor=p["line"])
        for tag, role in (("time", "muted"), ("robot", "accent"), ("sent", "ok"),
                          ("panel", "warn")):
            self.events.tag_configure(tag, foreground=p[role])
        for cb in self.map_combo.values():      # the drop-down list itself
            try:
                cb.tk.eval(f"[ttk::combobox::PopdownWindow {cb}].f.l configure "
                           f"-background {p['field']} -foreground {p['text']} "
                           f"-selectbackground {p['accent']} -selectforeground white")
            except tk.TclError:
                pass
        self.theme_btn.configure(text="☀  Light" if self.theme == "dark" else "☾  Dark")
        self._draw_dots()

    # ---- layout helpers ----------------------------------------------------
    def _frame(self, parent, bg="card"):
        return self._reg(tk.Frame(parent), bg=bg)

    def _lbl(self, parent, text="", bg="card", fg="text", font=(FONT, 10), **kw):
        w = tk.Label(parent, text=text, font=font, **kw)
        self._reg(w, bg=bg)
        self._tone(w, fg)
        return w

    def _button(self, parent, text, command, kind="plain", font=(FONT, 10, "bold"), **kw):
        bg, hi, fg = BUTTON_KINDS[kind]
        kw.setdefault("padx", 12)
        kw.setdefault("pady", 5)
        b = tk.Button(parent, text=text, command=command, relief="flat", bd=0,
                      cursor="hand2", font=font, **kw)
        self._reg(b, bg=bg, fg=fg, activebackground=hi, activeforeground=fg)
        b.bind("<Enter>", lambda e: b.configure(bg=self.pal[hi]))
        b.bind("<Leave>", lambda e: b.configure(bg=self.pal[bg]))
        return b

    def _card(self, parent, title, sub="", big=False):
        """a bordered card with a small caps title (big: a heading);
        returns (card, body)"""
        card = self._reg(tk.Frame(parent, highlightthickness=1), bg="card",
                         highlightbackground="line", highlightcolor="line")
        head = self._frame(card)
        head.pack(fill="x", padx=14, pady=(10, 6))
        if big:
            self._lbl(head, title, font=(FONT, 15, "bold")).pack(side="left")
        else:
            self._lbl(head, title.upper(), fg="muted", font=(FONT, 9, "bold")).pack(side="left")
        if sub:
            self._lbl(head, sub, fg="muted", font=(FONT, 9)).pack(side="left", padx=8)
        body = self._frame(card)
        body.pack(fill="both", expand=True, padx=14, pady=(0, 12))
        return card, body

    # ---- layout ----------------------------------------------------------
    def _build(self):
        outer = self._frame(self.root, bg="bg")
        outer.pack(fill="both", expand=True, padx=12, pady=12)

        # Header: name, broker and robot badges, theme switch; broker address
        head, hb = self._card(outer, "P5-IS34", "robot control panel", big=True)
        head.pack(fill="x")
        row = self._frame(hb)
        row.pack(fill="x")
        self._lbl(row, "Broker", fg="muted").pack(side="left")
        self.host_var = tk.StringVar(value=self.host)
        self.port_var = tk.StringVar(value=str(self.port))
        ttk.Entry(row, textvariable=self.host_var, width=15).pack(side="left", padx=(6, 4))
        ttk.Entry(row, textvariable=self.port_var, width=6).pack(side="left")
        self._button(row, "Connect", self.connect).pack(side="left", padx=(8, 4))
        self._button(row, "Start broker", self.start_broker).pack(side="left")
        self.theme_btn = self._button(row, "", self.toggle_theme)
        self.theme_btn.pack(side="right")
        self.robot_lbl = self._lbl(row, "●  robot: no data", bg="tile", fg="bad",
                                   font=(FONT, 10, "bold"), padx=10, pady=4)
        self.robot_lbl.pack(side="right", padx=8)
        self.conn_lbl = self._lbl(row, "●  not connected", bg="tile", fg="bad",
                                  font=(FONT, 10, "bold"), padx=10, pady=4)
        self.conn_lbl.pack(side="right")

        # START / STOP
        btns = self._frame(outer, bg="bg")
        btns.pack(fill="x", pady=10)
        btns.columnconfigure(0, weight=1, uniform="go")
        btns.columnconfigure(1, weight=1, uniform="go")
        self._button(btns, "▶   START", self.start, "start", font=(FONT, 18, "bold"),
                     pady=12).grid(row=0, column=0, sticky="ew", padx=(0, 5))
        self._button(btns, "■   STOP   (Space)", self.stop, "stop", font=(FONT, 18, "bold"),
                     pady=12).grid(row=0, column=1, sticky="ew", padx=(5, 0))

        # Barcode mapping: pick which barcode means which move, send it.
        # The robot stores the latest barcode it reads and does its move at
        # the NEXT junction (U-turn too: instructor brief, TEAM.md 7.1 B6).
        mpc, mp = self._card(outer, "Barcode mapping",
                             "after reading a barcode, the robot does this at the NEXT junction")
        mpc.pack(fill="x")
        saved = self._load_mapping()
        self.map_combo = {}
        tiles = self._frame(mp)
        tiles.pack(fill="x")
        for col, (mv, _) in enumerate(MOVES):
            arrow, name = MOVE_UI[mv]
            t = self._frame(tiles, bg="tile")
            t.grid(row=0, column=col, sticky="nsew", padx=(0 if col == 0 else 4, 0))
            tiles.columnconfigure(col, weight=1, uniform="map")
            self._lbl(t, arrow, bg="tile", fg="accent", font=(FONT, 20, "bold")).pack(
                side="left", padx=(12, 8), pady=8)
            f = self._frame(t, bg="tile")
            f.pack(side="left", pady=8)
            self._lbl(f, name, bg="tile", font=(FONT, 10, "bold")).pack(anchor="w")
            r = self._frame(f, bg="tile")
            r.pack(anchor="w")
            self._lbl(r, "barcode", bg="tile", fg="muted").pack(side="left")
            cb = ttk.Combobox(r, values=CODE_CHARS, width=3, state="readonly",
                              font=(FONT, 12, "bold"))
            cb.set(saved[mv])
            cb.pack(side="left", padx=6)
            cb.bind("<<ComboboxSelected>>", lambda e: self._map_changed())
            self.map_combo[mv] = cb
        r2 = self._frame(mp)
        r2.pack(fill="x", pady=(10, 0))
        self._button(r2, "Send mapping to robot", self.send_map, "accent").pack(side="left")
        self._button(r2, "Test set: A B C D",
                     lambda: self._set_map(dict(L="A", R="B", F="C", U="D"))).pack(
                         side="left", padx=6)
        self._lbl(r2, "or paste one (e.g. K=L,Q=R,X=F,M=U):", fg="muted").pack(
            side="left", padx=(10, 4))
        self.map_var = tk.StringVar()
        ent = ttk.Entry(r2, textvariable=self.map_var, width=20)
        ent.pack(side="left")
        ent.bind("<Return>", lambda e: self._paste_map())
        self._button(r2, "Use", self._paste_map).pack(side="left", padx=4)
        r3 = self._frame(mp)
        r3.pack(fill="x", pady=(8, 0))
        self.map_msg = self._lbl(r3, anchor="w", fg="muted")
        self.map_msg.pack(side="left")
        self.robot_map_lbl = self._lbl(r3, "robot: no mapping yet", fg="bad",
                                       font=(FONT, 10, "bold"))
        self.robot_map_lbl.pack(side="left", padx=14)
        self.ack_lbl = self._lbl(r3, fg="muted")
        self.ack_lbl.pack(side="left", padx=8)
        self.robot_map = ""
        self._map_changed()

        # Status (left) and live sensors (right), side by side
        mid = self._frame(outer, bg="bg")
        mid.pack(fill="x", pady=10)
        mid.columnconfigure(0, weight=3)
        mid.columnconfigure(1, weight=2)

        stc, st = self._card(mid, "Status", "every 1 s")
        stc.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        self.vals = {}
        for i, (key, label) in enumerate(FIELDS):
            t = self._frame(st, bg="tile")
            span = 3 if key == "faults" else 1
            t.grid(row=i // 3, column=i % 3, columnspan=span, sticky="nsew", padx=3, pady=3)
            self._lbl(t, label, bg="tile", fg="muted", font=(FONT, 9)).pack(
                anchor="w", padx=10, pady=(7, 0))
            v = self._lbl(t, "-", bg="tile", font=(FONT, 14, "bold"), anchor="w",
                          justify="left",
                          wraplength=int((520 if span == 3 else 170) * self.s))
            v.pack(anchor="w", padx=10, pady=(0, 7))
            self.vals[key] = v
        for c in range(3):
            st.columnconfigure(c, weight=1, uniform="status")

        snc, sn = self._card(mid, "Live sensors", "every 0.5 s")
        snc.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        sn.columnconfigure(0, weight=1)
        # line + barcode: one dot per sensor, filled = it sees black
        t = self._frame(sn, bg="tile")
        t.grid(row=0, column=0, sticky="nsew", pady=(0, 6))
        self._lbl(t, "Line + barcode", bg="tile", fg="muted", font=(FONT, 9)).pack(
            anchor="w", padx=10, pady=(7, 2))
        dots = self._frame(t, bg="tile")
        dots.pack(anchor="w", padx=8)
        self.dots = {}
        for name in "LRB":
            size = int(34 * self.s)
            c = tk.Canvas(dots, width=size, height=size, highlightthickness=0)
            self._reg(c, bg="tile")
            c.pack(side="left", padx=2)
            self.dots[name] = c
        self.sens_line = self._lbl(t, "-", bg="tile", fg="muted", font=(MONO, 9),
                                   justify="left", anchor="w")
        self.sens_line.pack(anchor="w", padx=10, pady=(4, 8))
        # obstacle and IMU next to each other
        two = self._frame(sn)
        two.grid(row=1, column=0, sticky="nsew")
        two.columnconfigure(0, weight=1, uniform="sens")
        two.columnconfigure(1, weight=1, uniform="sens")
        t = self._frame(two, bg="tile")
        t.grid(row=0, column=0, sticky="nsew", padx=(0, 3))
        self._lbl(t, "Obstacle (front)", bg="tile", fg="muted", font=(FONT, 9)).pack(
            anchor="w", padx=10, pady=(7, 0))
        self.sens_obst = self._lbl(t, "-", bg="tile", font=(FONT, 14, "bold"), anchor="w")
        self.sens_obst.pack(anchor="w", padx=10, pady=(0, 8))
        t = self._frame(two, bg="tile")
        t.grid(row=0, column=1, sticky="nsew", padx=(3, 0))
        self._lbl(t, "IMU tilt", bg="tile", fg="muted", font=(FONT, 9)).pack(
            anchor="w", padx=10, pady=(7, 0))
        self.sens_tilt = self._lbl(t, "-", bg="tile", font=(FONT, 14, "bold"), anchor="w")
        self.sens_tilt.pack(anchor="w", padx=10)
        self.sens_imu = self._lbl(t, "", bg="tile", fg="muted", font=(FONT, 9),
                                  justify="left", anchor="w")
        self.sens_imu.pack(anchor="w", padx=10, pady=(0, 8))

        # Run summary / events / robot log
        nb = ttk.Notebook(outer)
        nb.pack(fill="both", expand=True)
        # Run summary (20 Nov): the barcodes, junctions, obstacles, bypasses,
        # humps and stops of each run, picked out of the events, which the
        # robot's timing lines and scraps of bars otherwise drown
        rs = ttk.Frame(nb, padding=6)
        nb.add(rs, text="Run summary")
        bar0 = self._frame(rs)
        bar0.pack(fill="x", pady=(0, 6))
        self._button(bar0, "Save log + summary", self.save_robot_log).pack(side="left")
        self._button(bar0, "Clear", self.clear_summary).pack(side="left", padx=6)
        self._lbl(bar0, "each START begins a new block", fg="muted").pack(side="left", padx=8)
        # wrap: on a narrow window a long line wraps instead of being cut off
        self.sum_text = tk.Text(rs, height=12, state="disabled", font=(MONO, 10), wrap="word",
                                relief="flat", bd=0, highlightthickness=1, padx=8, pady=6)
        sb0 = ttk.Scrollbar(rs, command=self.sum_text.yview)
        self.sum_text.configure(yscrollcommand=sb0.set)
        sb0.pack(side="right", fill="y")
        self.sum_text.pack(fill="both", expand=True)
        self._show_summary()

        ev = ttk.Frame(nb, padding=6)
        nb.add(ev, text="Events")
        self.events = tk.Text(ev, height=8, state="disabled", font=(MONO, 10), relief="flat",
                              bd=0, highlightthickness=1, padx=8, pady=6)
        sb = ttk.Scrollbar(ev, command=self.events.yview)
        self.events.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.events.pack(fill="both", expand=True)

        rl = ttk.Frame(nb, padding=6)
        nb.add(rl, text="Robot log")
        bar = self._frame(rl)
        bar.pack(fill="x", pady=(0, 6))
        self._button(bar, "Save log + summary", self.save_robot_log).pack(side="left")
        self._button(bar, "Clear", self.clear_robot_log).pack(side="left", padx=6)
        self.follow_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="follow", variable=self.follow_var).pack(side="left", padx=4)
        self.robot_lbl2 = self._lbl(bar, "nothing yet (needs an MQTT image)", fg="muted")
        self.robot_lbl2.pack(side="left", padx=8)
        self.rlog = tk.Text(rl, height=8, state="disabled", font=(MONO, 9), wrap="none",
                            relief="flat", bd=0, highlightthickness=1, padx=8, pady=6)
        sb2 = ttk.Scrollbar(rl, command=self.rlog.yview)
        self.rlog.configure(yscrollcommand=sb2.set)
        sb2.pack(side="right", fill="y")
        self.rlog.pack(fill="both", expand=True)

    def _draw_dots(self):
        p = self.pal
        size = int(34 * self.s)
        for name, c in self.dots.items():
            on = self.ir[name]
            c.delete("all")
            c.create_oval(3, 3, size - 3, size - 3, fill=p["accent"] if on else p["dot_off"],
                          outline="")
            c.create_text(size // 2, size // 2, text=name, font=(FONT, 10, "bold"),
                          fill=p["white"] if on else p["muted"])

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
            self._tone(self.map_msg, "bad", "each move needs a different barcode")
        else:
            self._tone(self.map_msg, "muted", f"will send: {text}")
            self._save_mapping()
        self._show_robot_map()

    def _set_map(self, choice):
        for mv, _ in MOVES:
            self.map_combo[mv].set(choice[mv])
        self._map_changed()

    def _paste_map(self):
        choice = parse_mapping(self.map_var.get())
        if isinstance(choice, str):
            self._tone(self.map_msg, "bad", f"could not read that: {choice}")
            return
        self._set_map(choice)

    def _show_robot_map(self):
        """what the robot has (its status line), against what is picked here"""
        have = self.robot_map
        if not have or have == "none":
            self._tone(self.robot_map_lbl, "bad", "robot: no mapping yet (press Send)")
            return
        want = self._map_text()
        same = want is not None and set(want.split(",")) == set(have.split(","))
        self._tone(self.robot_map_lbl, "ok" if same else "warn",
                   f"robot has: {have}" + ("  (same)" if same else "  (different: press Send)"))

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
            self._tone(self.map_msg, "bad", "not sent: each move needs a different barcode")
            return
        self._tone(self.ack_lbl, "muted", "waiting for the robot ...")
        payload = wire.pack_map(self._map_choice())
        self.publish("is34/cmd/map", payload, shown=f"{text} = {payload.hex(' ')}")

    def save_robot_log(self):
        # Saved straight away, no file dialog (15 Nov: the dialog did not
        # appear and nothing was saved). One pair of files per press, in
        # logs/robot: the robot's log and the run summary (20 Nov).
        stamp = time.strftime("%Y%m%d_%H%M%S")
        saved = []
        try:
            os.makedirs(ROBOT_DIR, exist_ok=True)
            if self.robot_text:
                path = os.path.join(ROBOT_DIR, f"robot_log_{stamp}.txt")
                with open(path, "w", encoding="utf-8") as f:
                    f.write("".join(self.robot_text))
                saved.append(path)
            if self.summary.runs:
                path = os.path.join(ROBOT_DIR, f"robot_summary_{stamp}.txt")
                with open(path, "w", encoding="utf-8") as f:
                    f.write(self.summary.text())
                saved.append(path)
        except OSError as err:
            self.add_event("panel", f"log NOT saved: {err}")
            return
        if not saved:
            self.add_event("panel", "robot log and summary are empty, nothing saved")
            return
        for path in saved:
            self.add_event("panel", f"saved: {path}")

    def clear_summary(self):
        self.summary.clear()
        self._show_summary()

    def _show_summary(self):
        self.sum_text.configure(state="normal")
        self.sum_text.delete("1.0", "end")
        self.sum_text.insert("end", self.summary.text())
        self.sum_text.see("end")
        self.sum_text.configure(state="disabled")

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
        self._tone(self.robot_lbl2, "bad" if warn else "muted",
                   f"{n} lines" + (" - " + ", ".join(warn) if warn else ""))

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
                self._tone(self.conn_lbl, "ok" if up else "bad",
                           f"●  broker {self.host}:{self.port}" if up else "●  not connected")
                self.add_event("panel", "connected to the broker" if up else "broker disconnected")
            else:
                self._message(item[1], item[2])
        age = time.time() - self.last_status
        if self.last_status == 0.0:
            self._tone(self.robot_lbl, "bad", "●  robot: no data")
        elif age > OFFLINE_S:
            self._tone(self.robot_lbl, "bad", f"●  robot: OFFLINE ({age:.0f} s)")
        else:
            self._tone(self.robot_lbl, "ok", "●  robot: online")
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
            self._tone(self.vals["state"], STATE_TONES.get(state, "bad"))
            self._tone(self.vals["stop"],
                       "text" if self.vals["stop"].cget("text") == "none" else "warn")
            self._tone(self.vals["faults"],
                       "text" if self.vals["faults"].cget("text") == "none" else "bad")
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
            self._tone(self.ack_lbl, "ok" if good else "bad", text)
            self.add_event("robot", f"mapping: {text}")
            if good:
                self.summary.set_mapping(text[len("OK "):] if text.startswith("OK ") else text)
                self._show_summary()
        elif topic == "is34/event":
            self.add_event("robot", text)
            if self.summary.feed(text):
                self._show_summary()
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
        for i, name in enumerate("LRB"):              # lit = that sensor sees black
            self.ir[name] = ir[i:i + 1] == "1"
        self._draw_dots()
        self.sens_line.configure(text=(
            f"steer {num('steer')} mm/s\n"
            f"in barcode: {'yes' if d.get('inbar') == '1' else 'no'}\n"
            f"steering held (B on bars): {'yes' if d.get('hold') == '1' else 'no'}"))
        front = num("front")
        front_txt = {None: "?", -1: "nothing in range", -2: "NO ECHO (fault)"}.get(
            front, f"{front} mm")
        self._tone(self.sens_obst, "bad" if front == -2 else "text", front_txt)
        tilt, h10 = num("tilt"), num("h10")
        if tilt is not None and h10 is not None:
            deg = math.degrees(math.asin(max(-1.0, min(1.0, tilt / 1000.0))))
            ok = d.get("imu") == "1"
            self.sens_tilt.configure(text=f"{deg:+.1f}°")
            self._tone(self.sens_imu, "muted" if ok else "bad",
                       f"climbed {h10 / 10:.1f} mm\n"
                       f"{'IMU OK' if ok else 'IMU NOT ANSWERING'}")

    def add_event(self, who, text):
        line = time.strftime("%H:%M:%S ") + f"[{who}] {text}"
        self.log.write(line + "\n")
        self.log.flush()
        self.events.configure(state="normal")
        self.events.insert("end", time.strftime("%H:%M:%S "), "time")
        self.events.insert("end", f"[{who}]", who)        # coloured by who
        self.events.insert("end", f" {text}\n")
        self.events.see("end")
        self.events.configure(state="disabled")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--host", default="127.0.0.1", help="broker address (default this laptop)")
    ap.add_argument("--port", type=int, default=1884, help="broker port (default 1884)")
    a = ap.parse_args()
    try:    # crisp text on a scaled Windows screen (otherwise Windows blurs it)
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (AttributeError, OSError):
        pass
    root = tk.Tk()
    Panel(root, a.host, a.port)
    root.mainloop()


if __name__ == "__main__":
    main()
