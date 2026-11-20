"""Fills the Tribunal 2 pre-tribunal template (python tools/docs/fill_tribunal2.py
<template.docx> docs/team/software_block_diagram.png <out.docx>; needs python-docx): header (team ID, repo link;
student IDs left for the team), A1 current requirements, A2 block diagram."""
import copy
import sys
import docx
from docx.shared import Pt, Inches
from docx.enum.text import WD_BREAK

SRC, IMG, OUT = sys.argv[1], sys.argv[2], sys.argv[3]

REQS = [
    ("FR1", "The robot shall follow the black line track autonomously with two IR line sensors at 120 mm/s."),
    ("FR2", "The robot shall read Code 39 barcodes (A-Z, 0-9) placed beside the track, in either direction, slowing to 50 mm/s over the bars, and keep the decoded character for the next junction. Damaged reads are repaired only when exactly one character fits; a code read with one start/stop character missing is used only if its character is mapped and cleanly read. No mapping is built in."),
    ("FR2a", "The robot shall accept a barcode-to-move mapping of exactly 4 pairs (Left, Right, Straight, U-turn; each character and move once) over WiFi/MQTT at any time without a restart, acknowledge it, and refuse an invalid one with the reason while keeping the previous mapping."),
    ("FR3", "The robot shall execute the stored character's move (Left, Right, Straight or U-turn) at the next cross-junction; a character with no mapping is reported and ignored (the robot carries on)."),
    ("FR4", "The robot shall detect a hump under one wheel from the IMU and report its peak height after each hump."),
    ("FR5", "The robot shall detect an obstacle ahead with the ultrasonic sensor, stop, scan +/-40 deg and report its width with its uncertainty (W +/- U mm)."),
    ("FR6", "The robot shall drive around the obstacle on a side the scan saw clear (stop and report if neither is clear), watch ahead on every bypass leg, go further along an obstacle longer than expected, and return to its own line behind the obstacle, turned back to its original heading."),
    ("FR7", "The robot shall publish a status message every 1 s over MQTT (state, wheel speeds, distance, line state, junctions, last barcode, mapping, hump peak, stop reasons, faults), live sensor readings, and an event for each barcode, junction move, obstacle width, hump and stop, with one short beep on the robot for each event."),
    ("FR8", "After a junction turn or a bypass the robot shall find the line and resume following, or stop and report if it is not found."),
    ("FR9", "The robot shall stop all motors and report the cause on: STOP (button or MQTT), wheel stall, Motion task hung, robot lifted or tipped over, line lost, no clear side or blocked bypass leg, ultrasonic not answering, or a manoeuvre not finished; it shall move again only after START. Encoder errors, IMU read errors and WiFi loss are reported as warnings and do not stop the robot."),
    ("FR10", "While connected, the 1 s status message serves as the heartbeat; the laptop panel shows the robot offline after 3 s without one."),
    ("FR11", "The robot shall detect cross- and T-junctions and apply the most recent unused barcode command at the next junction; with none (including after a failed read) it continues straight."),
    ("FR12", "The robot shall start on START (button GP20 or MQTT) and stop at once on STOP (button GP21 or MQTT) at any time; lifting the robot at the end of the run stops it (FR13)."),
    ("FR13", "The robot shall stop all motors within 100 ms when it is lifted or tipped more than 35 deg from its level at START (IMU). IMU collision detection is removed (lecturer, 13 Nov)."),
    ("FR14", "Every stop or fault shall be held in one shared stop/fault word that turns the motors off when set, from any context, and blocks all motor commands until cleared (STOP by START; stall or hung-task faults after 2 s with the motors off)."),
    ("FR15", "(Change Requirement 1) Shared data and MQTT messages shall use the minimum bits for their range: bit-fields with values clamped on the way in, and binary messages (status 15 B, sensors 8 B, acknowledgement 5 B, mapping 4 B, little-endian, versioned)."),
    ("FR16", "A blocked bypass leg shall stop the robot and be reported; the robot does not retrace."),
    ("NFR1", "Task periods: Motion 10 ms, IMU 10 ms (100 Hz), line 20 ms, Safety 20 ms, barcode 10 ms, obstacle 60 ms, telemetry 100 ms; no overruns after start-up in the run-time timing table."),
    ("NFR1a", "While following the line at 120 mm/s, no line-lost stop over 3 m of track (turns and bypasses excluded)."),
    ("NFR3", "One obstacle scan (17 rays, 5 deg steps, 3 pings per ray, servo settle >= 60 ms) shall finish within 10 s, measured from the stop to the width report (measured 7-9 s, 20 Nov); the robot is stopped while scanning."),
    ("NFR4", "A barcode placed at least 200 mm before its junction shall be decoded and its move stored before the robot reaches that junction."),
    ("NFR5", "Status at >= 1 Hz; after WiFi or broker loss the robot keeps running, reconnects by itself within 20 s of both being available, and its WiFi-loss warning clears within 1 s of reconnecting."),
    ("NFR6", "The forward watch trips at 180 mm (2 readings in a row); at 120 mm/s the robot stops at least 150 mm from the obstacle (measured 157-160 mm, 20 Nov)."),
    ("NFR6a", "During a bypass the robot stays within 300 mm of the original line, makes no contact with the obstacle and is back on its own line behind it."),
    ("NFR7", "C with the Pico C SDK on micro T-Kernel 3.0 (single core); Barr C 2018 with documented deviations only; course template repository structure."),
    ("NFR8", "Accuracy: 90 deg turn +/-10 deg, 180 deg turn +/-15 deg; barcode read >= 90 % over 20 passes; hump height +/-5 mm for humps of 10-30 mm; obstacle width within the reported +/- U."),
    ("NFR9", "Static RAM <= 200 KB of 264 KB (about 92 KB with WiFi, measured); every MQTT payload <= 160 bytes."),
    ("NFR10", "STOP button: motor drive off inside the GPIO interrupt. Safety task: motors off within 220 ms of the Motion task stopping (200 ms + one 20 ms period) and within 1 s of a wheel stalling, without waiting for Motion."),
    ("NFR11", "Each shared value has one writer or is accessed only under the one lock (APP_LOCK); a STOP inside a burst of MQTT START/STOP commands is never lost."),
    ("NFR12", "Every bypass leg drives with the forward watch on (2 readings < 150 mm stop the robot); turning on the spot beside an obstacle keeps the robot's centre >= 185 mm from it (corners swing <= 155 mm)."),
]

d = docx.Document(SRC)

# ---- header -------------------------------------------------------------
hdr = d.tables[0]


def set_cell(cell, text):
    p = cell.paragraphs[0]
    for r in p.runs:
        r.text = ""
    if p.runs:
        p.runs[0].text = text
    else:
        p.add_run(text)


set_cell(hdr.rows[0].cells[1], "P5-IS34")
set_cell(hdr.rows[6].cells[1], "https://github.com/laun1ngxu4n/INF2004-P5-IS34-EmbeddedSystems")

# ---- A1 table -----------------------------------------------------------
t = d.tables[1]
template_row = copy.deepcopy(t.rows[1]._tr)
while len(t.rows) < len(REQS) + 1:
    t._tbl.append(copy.deepcopy(template_row))
while len(t.rows) > len(REQS) + 1:
    t._tbl.remove(t.rows[-1]._tr)

for row, (rid, text) in zip(t.rows[1:], REQS):
    for cell, val, bold in ((row.cells[0], rid, True), (row.cells[1], text, False)):
        p = cell.paragraphs[0]
        for r in list(p.runs):
            r._r.getparent().remove(r._r)
        run = p.add_run(val)
        run.font.size = Pt(9)
        run.bold = bold
        p.paragraph_format.space_after = Pt(0)
        p.paragraph_format.space_before = Pt(0)

# ---- A2: page break before its heading, image in the box -------------------
for p in d.paragraphs:
    if p.text.startswith("A2. Final"):
        p.paragraph_format.page_break_before = True
box = d.tables[2].rows[0].cells[0]
for p in box.paragraphs[1:]:
    p._p.getparent().remove(p._p)
p0 = box.paragraphs[0]
for r in list(p0.runs):
    r._r.getparent().remove(r._r)
p0.add_run().add_picture(IMG, width=Inches(6.25))
p0.alignment = 1

d.save(OUT)
print("saved", OUT, "with", len(REQS), "requirements")
