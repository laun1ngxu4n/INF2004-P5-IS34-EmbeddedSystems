"""P5-IS34 software block diagram for the Tribunal 2 submission (A2).
Facts from app_program/ (app.h priorities, task periods, pins), TEAM.md
section 5 (IF-01..IF-05), CHANGE_REQ1.md section 6, changes 13-17 Nov."""
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

OUT = sys.argv[1]
W, H = 100.0, 128.0
fig = plt.figure(figsize=(7.0, 8.96), dpi=220)
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.axis("off")

OWN = {"B1": ("#dbe8f7", "#2f6db5"), "B2": ("#ddf0dc", "#2e8b3a"),
       "B3": ("#fde7cf", "#c8701a"), "B4": ("#ece0f5", "#7a3fa6"),
       "B5": ("#d6f0ef", "#1d8a85")}
RED, GREY, EVT = "#c62828", "#555555", "#8a7a00"
SHARED_F, SHARED_E = "#fff6c8", "#b59a00"


def box(x, y, w, h, title, body="", fill="#ffffff", edge="#333333", lw=1.0,
        ls="-", tsize=6.4, bsize=5.1):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=1.1",
                                fc=fill, ec=edge, lw=lw, ls=ls, zorder=2))
    if body:
        ax.text(x + w / 2, y + h - 1.2, title, ha="center", va="top",
                fontsize=tsize, weight="bold", zorder=3)
        ax.text(x + w / 2, y + h - 3.5, body, ha="center", va="top",
                fontsize=bsize, zorder=3, linespacing=1.17)
    else:
        ax.text(x + w / 2, y + h / 2, title, ha="center", va="center",
                fontsize=tsize, weight="bold", zorder=3, linespacing=1.12)


def task(x, y, w, h, name, owner, meta, body, bsize=5.0):
    f, e = OWN[owner]
    box(x, y, w, h, f"{name} ({owner})", f"{meta}\n{body}", fill=f, edge=e,
        lw=1.4, bsize=bsize)


def arrow(p0, p1, color=GREY, lw=0.9, ls="-", both=False, head=True):
    style = "<|-|>" if both else ("-|>" if head else "-")
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle=style, mutation_scale=6.5,
                                 color=color, lw=lw, ls=ls, zorder=4,
                                 shrinkA=0, shrinkB=0))


def label(x, y, text, color=GREY, size=4.7, ha="center"):
    ax.text(x, y, text, fontsize=size, color=color, ha=ha, va="center",
            zorder=6, bbox=dict(fc="white", ec="none", pad=0.35, alpha=0.95))


ax.text(W / 2, H - 1.0, "P5-IS34 software block diagram: 8 tasks on micro T-Kernel 3.0 (single core, RP2040 Pico W)",
        ha="center", va="top", fontsize=7.0, weight="bold")

# ---- external + legend ----------------------------------------------------
box(2, 112.5, 27, 9.5, "Laptop panel (B1)",
    "START / STOP, mapping,\nstatus, events, robot log", fill="#f4f4f4")
box(34, 112.5, 25, 9.5, "MQTT broker",
    "Mosquitto, port 1884,\non the phone hotspot", fill="#f4f4f4")
arrow((29, 117), (34, 117), both=True)

lx, ly = 63.0, 122.3
ax.text(lx, ly + 0.6, "Legend (owner = buddy)", fontsize=5.4, weight="bold", va="center")
for i, (o, lab) in enumerate([("B1", "B1 Telemetry, MQTT"), ("B2", "B2 Motion"),
                              ("B3", "B3 Line, barcode"), ("B4", "B4 IMU, Safety"),
                              ("B5", "B5 Mission, obstacle")]):
    f, e = OWN[o]
    yy = ly - 2.2 - i * 2.0
    ax.add_patch(FancyBboxPatch((lx, yy - 0.7), 2.2, 1.4, boxstyle="round,pad=0,rounding_size=0.3",
                                fc=f, ec=e, lw=1))
    ax.text(lx + 3.0, yy, lab, fontsize=4.7, va="center")
mx = lx + 18.5
ax.plot([mx, mx + 3], [ly - 2.2, ly - 2.2], color=RED, lw=1.5)
ax.text(mx + 3.7, ly - 2.2, "stop / safety path", fontsize=4.7, va="center")
ax.add_patch(FancyBboxPatch((mx, ly - 4.9), 3, 1.4, boxstyle="round,pad=0,rounding_size=0.3",
                            fc=SHARED_F, ec=SHARED_E, lw=1))
ax.text(mx + 3.7, ly - 4.2, "shared data", fontsize=4.7, va="center")
ax.plot([mx, mx + 3], [ly - 6.2, ly - 6.2], color=EVT, lw=0.9, ls=":")
ax.text(mx + 3.7, ly - 6.2, "event flag (IF-01)", fontsize=4.7, va="center")
ax.add_patch(FancyBboxPatch((mx, ly - 8.9), 3, 1.4, boxstyle="round,pad=0,rounding_size=0.3",
                            fc="white", ec="#444444", lw=0.9, ls="--"))
ax.text(mx + 3.7, ly - 8.2, "interrupt / handler", fontsize=4.7, va="center")

# ---- robot boundary ---------------------------------------------------------
ax.add_patch(FancyBboxPatch((1, 1), 98, 109, boxstyle="round,pad=0,rounding_size=2",
                            fc="none", ec="#999999", lw=1.0, ls="--", zorder=1))
ax.text(97.5, 108.6, "ROBOT", fontsize=6.0, color=GREY, ha="right", va="top", weight="bold")

box(27, 99.5, 46, 7.5, "WiFi + MQTT client (kernel tasks, pri 3-4)",
    "CYW43 + lwIP; app_mqtt_rx() / app_mqtt_tx() (IF-05)", fill="#f4f4f4", tsize=6.0)
arrow((46.5, 112.5), (46.5, 107), both=True)
label(24.5, 111.0, "binary (CR1): status 15 B / s, sensors 8 B, ack 5 B, map 4 B;  text: events, log",
      size=4.5)

# ---- telemetry row ---------------------------------------------------------
task(22, 84.5, 56, 12.5, "Telemetry", "B1", "pri 12, 100 ms",
     "binary status every 1 s + live sensors; events from every task;\n"
     "commands in: START, STOP, mapping (checked; ack or refused + reason);\n"
     "Wi-Fi loss = warning only; IF-01 1 s ping to Mission")
arrow((50, 99.5), (50, 97), both=True)
box(81, 85.5, 17, 10.5, "Barcode mapping",
    "4 pairs: L, R, F, U;\nunder APP_LOCK;\nwritten only from MQTT", fill=SHARED_F,
    edge=SHARED_E, tsize=5.8, bsize=4.7)
arrow((78, 90.75), (81, 90.75))

# ---- mission row -----------------------------------------------------------
box(2, 64, 15.5, 13, "flg_mission",
    "event flag (IF-01):\nSTART, STOP,\nJUNCTION,\nLINE_LOST,\nBARCODE,\nOBSTACLE, PING",
    fill=SHARED_F, edge=SHARED_E, tsize=5.8, bsize=4.6)
task(21.5, 63, 40, 16.5, "Mission", "B5", "pri 7, event + 100 ms",
     "IDLE / CAL / RUN / MANOEUVRE state machine;\n"
     "barcode letter -> its move at the next junction\n"
     "(none = straight; not mapped = stop);\n"
     "obstacle: stop, scan, width +/- U, bypass on the clear side,\n"
     "watch ahead on every leg, go further past a long box,\n"
     "back to its own line; halts with the reason")
task(65, 63, 22, 16.5, "Obstacle", "B5", "pri 11, 60 ms",
     "forward watch: 2 pings\n< 175 mm -> OBSTACLE;\n+/-40 deg scan, 5 pings\nper ray (servo, sonar);\nno ECHO = fault,\nnever 'clear'",
     bsize=4.7)
arrow((22, 92), (12, 77))                 # START / STOP into the flag
label(16.4, 85.6, "START,\nSTOP", size=4.4)
arrow((17.5, 70.5), (21.5, 70.5))          # flag -> Mission
arrow((41, 79.5), (41, 84.5), both=True)   # Mission <-> Telemetry
label(45.2, 82.0, "events", size=4.5)
arrow((61.5, 71), (65, 71), both=True)     # Mission <-> Obstacle scan
label(63.3, 73.4, "scan()", size=4.4)

# ---- event bus ---------------------------------------------------------------
BUS = 59.5
ax.plot([9.75, 92], [BUS, BUS], color=EVT, lw=0.9, ls=":", zorder=3)
arrow((9.75, BUS), (9.75, 64), color=EVT, lw=0.9, ls=":")   # bus -> flag
ax.plot([9.75, 9.75], [48, BUS], color=EVT, lw=0.9, ls=":", zorder=3)  # Line tap
ax.plot([76, 76], [63, BUS], color=EVT, lw=0.9, ls=":", zorder=3)      # Obstacle tap
ax.plot([92, 92], [48, BUS], color=EVT, lw=0.9, ls=":", zorder=3)      # Barcode tap
label(55.5, BUS, "events: JUNCTION, LINE_LOST (Line), BARCODE (Barcode), OBSTACLE (Obstacle)",
      color=EVT, size=4.4)

# ---- motion command slot ------------------------------------------------------
box(22.5, 50.5, 22, 7, "Motion command + status",
    "IF-04: drive / turn / follow + id;\nresult DONE / STOPPED / TIMEOUT",
    fill=SHARED_F, edge=SHARED_E, tsize=5.5, bsize=4.5)
arrow((33.5, 63), (33.5, 57.5), both=True)
arrow((33.5, 50.5), (33.5, 48), both=True)

# ---- worker row ---------------------------------------------------------------
task(2, 33, 15.5, 15, "Line", "B3", "pri 8, 20 ms",
     "2 IR sensors ->\nsteering (IF-03);\njunction X / T;\nline lost", bsize=4.7)
task(21.5, 33, 24, 15, "Motion", "B2", "pri 6, 10 ms",
     "encoders -> speed,\nPI per wheel -> PWM;\nfollow 120 mm/s, spins;\nrefuses while STOP set", bsize=4.7)
task(49, 33, 13, 15, "IMU", "B4", "pri 9, 10 ms",
     "LSM303D\n100 Hz: hump\nheight; lift /\ntip > 35 deg", bsize=4.6)
task(66, 33, 15, 15, "Safety", "B4", "pri 5, 20 ms",
     "wheel stall 1 s;\nMotion hung\n200 ms; encoder\nerrors = warning", bsize=4.6)
task(84.5, 33, 13.5, 15, "Barcode", "B3", "pri 10, 10 ms",
     "edges -> widths\n-> Code 39,\neither way;\nreads mapping", bsize=4.6)
arrow((17.5, 41), (21.5, 41))
label(19.5, 43.0, "IF-03", size=4.2)
arrow((89.5, 85.5), (89.5, 48), color=SHARED_E, lw=0.8)  # mapping -> Barcode
label(93.6, 81.5, "mapping", color=SHARED_E, size=4.3)

# ---- stop / fault word ---------------------------------------------------------
box(24, 19.5, 38, 10.5, "STOP / fault word (app_stop, IF-02)",
    "stop reasons: BUTTON | MQTT | FAULT;  faults: 8 bits\n"
    "(stall, encoder, IMU, no echo, line, Wi-Fi, Motion hung);\n"
    "setting it calls motor_safe_off() first, from any context",
    fill="#fde3e3", edge=RED, lw=1.6, tsize=5.9, bsize=4.6)
arrow((33.5, 30), (33.5, 33), color=RED, lw=1.4)          # inhibits Motion
label(30.0, 31.6, "inhibit", color=RED, size=4.3)
arrow((55.5, 33), (55.5, 30), color=RED, lw=1.4)          # IMU lift
label(58.6, 31.6, "lift", color=RED, size=4.3)
arrow((70, 33), (62, 27), color=RED, lw=1.4)              # Safety
label(70.3, 29.4, "stall / hung", color=RED, size=4.3)
ax.plot([63.5, 63.5], [84.5, 25], color=RED, lw=1.4, zorder=3)  # MQTT STOP + halt
arrow((63.5, 25), (62, 25), color=RED, lw=1.4)
ax.plot([61.5, 63.5], [66, 66], color=RED, lw=1.4, zorder=3)     # Mission halt joins
label(63.5, 81.8, "MQTT STOP", color=RED, size=4.3)
label(63.5, 52.0, "MQTT STOP,\nMission halt()", color=RED, size=4.3)

# ---- interrupts -------------------------------------------------------------
box(2, 10.5, 21, 7, "GPIO IRQ (app_gpio)",
    "encoders, IR L / R,\nsonar ECHO, buttons", ls="--", tsize=5.5, bsize=4.5)
box(66, 10.5, 15, 7, "Cyclic handlers",
    "wake the periodic\ntasks", ls="--", tsize=5.5, bsize=4.5)
box(84.5, 10.5, 13.5, 7, "ADC FIFO IRQ",
    "2 kHz: barcode\n-> edge ring", ls="--", tsize=5.5, bsize=4.5)
arrow((9.75, 17.5), (9.75, 33))                         # IR edges -> Line
arrow((15, 17.5), (22.5, 33))                           # encoder counts -> Motion
label(14.2, 26.0, "encoder\ncounts", size=4.2)
arrow((22, 17.5), (25.5, 19.5), color=RED, lw=1.4)      # STOP button
label(41.5, 15.6, "STOP button: motors off inside the GPIO interrupt (no task)",
      color=RED, size=4.3)
arrow((91.25, 17.5), (91.25, 33))                       # edges -> Barcode
label(94.7, 25.0, "edges", size=4.2)

# ---- hardware -----------------------------------------------------------------
hw = [("Motors, PWM\nGP8-11", 2, 11), ("Encoders\nGP4/5, 16/17", 14, 12), ("IR line\nGP0 / GP1", 27, 10),
      ("Barcode\nGP28 (ADC)", 38, 10.5), ("LSM303D I2C1\nGP26/27", 49.5, 12), ("HC-SR04 GP6/7\nservo GP14", 62.5, 13),
      ("Buttons\nGP20 / GP21", 76.5, 10.5), ("CYW43\nWiFi", 88, 10)]
for name, x, w in hw:
    box(x, 2.5, w, 6.2, name, fill="#ececec", edge="#888888", tsize=4.8)

fig.savefig(OUT, dpi=220)
print("saved", OUT)
