# Logs (not in git)

The panel (`tools/panel/is34_panel.py`) writes here on every start:

| Folder | Files | What |
|---|---|---|
| `panel/` | `is34_<date>_<time>.log` | Everything the panel showed: status lines, sensors, events, what it sent. |
| `robot/` | `is34_<date>_<time>_robot.txt` | The robot's own console, as it came over WiFi (MQTT images). **Save log...** in the panel also saves here by default. |

Everything in this folder except this README is git-ignored: the logs stay on
the laptop. To keep a log as test evidence, copy it into
`testing/<subsystem>/logs/` with a name like `2026-11-08_stage3_bypass.txt`
and record it in that subsystem's `TEST_LOG.md`.
