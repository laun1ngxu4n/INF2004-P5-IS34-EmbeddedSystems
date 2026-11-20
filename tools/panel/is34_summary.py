"""Run summary for the panel: the results that matter, picked out of the
robot's events (is34/event), one block per run (a run starts with the
robot's "start" event).

    barcodes read (letter, move, bar widths), junction moves, each obstacle
    (trip and stop distance, width, scan time, bypass side and size, line
    found, heading after it, bypass time), hump heights, and every stop.

Used by is34_panel.py ("Run summary" tab, and a summary file with each
"Save log").  Test it on a saved panel log without the window:

    python tools/panel/is34_summary.py logs/panel/is34_20261120_145137.log
"""
import re
import sys
import time

BARCODE = re.compile(r"^barcode (\S)(.*?)\(narrow ([\d.]+) mm, wide ([\d.]+) mm\) -> (.*)$")
READ_FAIL = re.compile(r"^barcode READ_FAIL \((\d+) elements\)")
JUNCTION = re.compile(r"^junction (\d+) \(([^)]*)\): (\S+) \(barcode (.)\)")
OBST_AHEAD = re.compile(r"^obstacle ahead at (-?\d+) mm")
WIDTH = re.compile(r"^obstacle width (\d+) \+/- (\d+) mm .*front (-?\d+) mm")
BYPASS = re.compile(r"^bypass (LEFT|RIGHT): planned width (\d+) mm, out (\d+) mm, past (\d+) mm")
LINE_FOUND = re.compile(r"^line found (\d+) mm after the last turn")
DONE = re.compile(r"^bypass done.*?(?:heading (-?\d+) deg)?\S*$")
HUMP = re.compile(r"^ramp peak height ([\d.]+) mm")
MOVES = {"LEFT at next junction": "LEFT", "RIGHT at next junction": "RIGHT",
         "FORWARD at next junction": "STRAIGHT", "U-TURN at next junction": "U-TURN",
         "NO MAPPING": "no mapping", "NO MAPPING, ignored": "no mapping (ignored, carried on)"}
STOP_WORDS = ("stopped:", "lifted or tipped", "Safety:", "something in the way",
              "not answering")


def clock(t):
    return time.strftime("%H:%M:%S", time.localtime(t))


class Summary:
    def __init__(self):
        self.runs = []
        self.mapping = "none"

    def clear(self):
        self.runs = []

    def _run(self, t):
        if not self.runs:
            self._new_run(t, started=False)
        return self.runs[-1]

    def _new_run(self, t, started=True):
        self.runs.append(dict(t=t, started=started, mapping=self.mapping, barcodes=[],
                              fails=0, junctions=[], obstacles=[], humps=[], stops=[]))

    def set_mapping(self, text, t=None):
        """the mapping the robot acknowledged (is34/ack/map)"""
        t = time.time() if t is None else t
        if self.runs and text != self.mapping:
            self.runs[-1]["mapping"] += f" -> {text} (at {clock(t)})"
        self.mapping = text

    def feed(self, text, t=None):
        """one robot event; True when the summary changed"""
        t = time.time() if t is None else t
        text = text.strip()
        if text == "start":
            self._new_run(t)
            return True
        if (not self.runs or not self.runs[-1]["started"]) and "lifted or tipped" in text:
            return False                # picked up while being placed: not a result
        run = self._run(t)
        obst = run["obstacles"][-1] if run["obstacles"] else None
        m = BARCODE.match(text)
        if m:
            extra = m.group(2).strip()
            run["barcodes"].append(dict(t=t, letter=m.group(1), extra=extra,
                                        narrow=m.group(3), wide=m.group(4),
                                        move=MOVES.get(m.group(5), m.group(5))))
            return True
        if READ_FAIL.match(text):
            run["fails"] += 1
            return True
        m = JUNCTION.match(text)
        if m:
            run["junctions"].append(dict(t=t, n=m.group(1), kind=m.group(2),
                                         move=m.group(3), letter=m.group(4)))
            return True
        m = OBST_AHEAD.match(text)
        if m:
            run["obstacles"].append(dict(t=t, trip=int(m.group(1))))
            return True
        m = WIDTH.match(text)
        if m:
            if obst is None or "width" in obst or obst.get("nothing"):
                obst = dict(t=t, trip=None)      # its "obstacle ahead" was lost on the WiFi
                run["obstacles"].append(obst)
            obst.update(width=int(m.group(1)), unc=int(m.group(2)), front=int(m.group(3)),
                        scan_s=(t - obst["t"]) if obst["trip"] is not None else None)
            return True
        m = BYPASS.match(text)
        if m:
            if obst is None or "side" in obst:
                obst = dict(t=t, trip=None)
                run["obstacles"].append(obst)
            obst.update(side=m.group(1), out=int(m.group(3)), past=int(m.group(4)), t_bypass=t)
            return True
        if obst is not None:
            if text.startswith("scan found nothing ahead"):
                obst.update(nothing=True, scan_s=t - obst["t"])
                return True
            if text.startswith("box still beside: going"):
                obst["extended"] = obst.get("extended", 0) + 1
                return True
            m = LINE_FOUND.match(text)
            if m:
                obst["line_mm"] = int(m.group(1))
                return True
            m = DONE.match(text)
            if m and text.startswith("bypass done"):
                obst["done"] = True
                if m.group(1) is not None:
                    obst["heading"] = int(m.group(1))
                if "t_bypass" in obst:
                    obst["bypass_s"] = t - obst["t_bypass"]
                return True
        m = HUMP.match(text)
        if m:
            run["humps"].append(dict(t=t, mm=float(m.group(1))))
            return True
        if text == "STOP" or any(w in text for w in STOP_WORDS):
            why = text.replace(" (press START to go again)", "")
            if run["stops"] and run["stops"][-1]["why"] == why:
                run["stops"][-1]["n"] += 1      # the same again (lifted several times)
            else:
                run["stops"].append(dict(t=t, why=why, n=1))
            return True
        return False

    def text(self):
        if not self.runs:
            return "No run yet: press START (each START begins a new block).\n"
        out = []
        for i, run in enumerate(self.runs, 1):
            head = f"Run {i}, " + ("started " if run["started"] else "before START, from ")
            out.append(f"{head}{clock(run['t'])}   mapping: {run['mapping']}")
            out.append("  Barcodes:")
            for b in run["barcodes"]:
                extra = b["extra"].strip(" ()")
                extra = f"{extra}, " if extra else ""
                out.append(f"    {clock(b['t'])}  {b['letter']} -> {b['move']}"
                           f"   ({extra}narrow {b['narrow']} mm, wide {b['wide']} mm)")
            if run["fails"]:
                out.append(f"    {run['fails']} failed read(s) (READ_FAIL)")
            if not run["barcodes"] and not run["fails"]:
                out.append("    none read")
            out.append("  Junctions:")
            for j in run["junctions"]:
                out.append(f"    {clock(j['t'])}  {j['n']} ({j['kind']}): {j['move']}"
                           f"  (barcode {j['letter']})")
            if not run["junctions"]:
                out.append("    none")
            out.append("  Obstacles:")
            for o in run["obstacles"]:
                out.append("    " + self._obstacle(o))
            if not run["obstacles"]:
                out.append("    none")
            humps = run["humps"]
            if humps:
                peak = max(h["mm"] for h in humps)
                out.append("  Hump: " + ", ".join(f"{h['mm']:.1f} mm" for h in humps)
                           + f"   (peak {peak:.1f} mm)")
            else:
                out.append("  Hump: none reported")
            out.append("  Stops:")
            for s in run["stops"]:
                times = f"  (x{s['n']})" if s["n"] > 1 else ""
                out.append(f"    {clock(s['t'])}  {s['why']}{times}")
            if not run["stops"]:
                out.append("    none")
            out.append("")
        return "\n".join(out) + "\n"

    @staticmethod
    def _obstacle(o):
        trip = f"tripped at {o['trip']} mm" if o["trip"] is not None else "(trip lost on the WiFi)"
        parts = [f"{clock(o['t'])}  {trip}"]
        if o.get("nothing"):
            parts.append(f"scan found nothing ({o['scan_s']:.0f} s), went on")
            return ", ".join(parts)
        if "width" in o:
            parts.append(f"width {o['width']} +/- {o['unc']} mm, front {o['front']} mm at the scan")
            if o.get("scan_s") is not None:
                parts.append(f"scan took {o['scan_s']:.0f} s")
        line = ", ".join(parts)
        if "side" in o:
            line += (f"\n      bypass {o['side']} out {o['out']} mm, past {o['past']} mm")
            if o.get("extended"):
                line += f", {o['extended']} x 100 mm further (box still beside)"
            if "line_mm" in o:
                line += f", line found {o['line_mm']} mm after the last turn"
            if o.get("done"):
                line += ", back on the line"
                if "heading" in o:
                    line += f" (heading {o['heading']} deg)"
                if "bypass_s" in o:
                    line += f", {o['bypass_s']:.0f} s"
        return line


def main(paths):
    """summary of saved panel logs: their '[robot] ...' lines"""
    s = Summary()
    day = time.strftime("%Y-%m-%d ")
    for p in paths:
        for raw in open(p, encoding="utf-8", errors="replace"):
            m = re.match(r"^(\d\d:\d\d:\d\d) \[robot\] (.*)$", raw)
            if not m:
                continue
            t = time.mktime(time.strptime(day + m.group(1), "%Y-%m-%d %H:%M:%S"))
            if m.group(2).startswith("mapping: OK "):
                s.set_mapping(m.group(2)[len("mapping: OK "):], t)
            else:
                s.feed(m.group(2), t)
    sys.stdout.write(s.text())


if __name__ == "__main__":
    main(sys.argv[1:])
