# P5-IS34 Buddy 1 bring-up test results (WiFi/MQTT, status LED/buzzer)

Date: Sunday 18 Oct 2026. Tester: Buddy 1.
Status: standalone bench measurements, not NFR acceptance. Plain Pico SDK, no RTOS, nothing else running on the robot.

## Test conditions

- Board: Pico W on Cytron Robo Pico, powered from the laptop USB cable (no power bank).
- Network: Android phone hotspot (2.4 GHz, WPA2); robot and laptop/broker on the hotspot subnet.
- Broker: Mosquitto on Windows, `tools/mosquitto.conf` (port 1883, anonymous, private hotspot only).
- Laptop monitor: `tools/mqtt_watch.py` (paho-mqtt), heartbeat CSV `hb_2026-10-18.csv`.
- Serial: Tera Term (also PuTTY earlier), USB CDC.
- Firmware: `wifi_mqtt_test_prompt.uf2` from bring-up v3 (asks SSID/password/broker IP over serial). Three builds were used during the day, changes listed at the end. Lwip MQTT output buffer 1024 B, keep-alive 5 s, join timeout 5 s, MQTT connect timeout 3 s, backoff 0.5/1/2/2 s, command queue 4 slots.
- Evidence files to keep: Tera Term log(s), `hb_2026-10-18.csv`, copied mqtt_watch summaries, this file.

## Results

### 1. Connect times (A1 budgets) - 3 boots

| Boot | cyw43 init | WiFi join + DHCP | MQTT connect | Payload probe |
|---|---|---|---|---|
| 1 | not recorded | 2,831 ms | 61 ms | 128-600 B all accepted |
| 2 | 1 ms | 2,911 ms | 81 ms | all accepted |
| 3 | 2 ms | 4,391 ms | 71 ms | all accepted |

- Design budget: join 5 s, MQTT 3 s, 8 s per attempt. All within budget.
- Join + DHCP varies by about 1.5 s; worst 4.39 s leaves only 0.6 s of margin. 3 samples only.
- Capacity probe: "OK" = publish call accepted with the 1 KB buffer, so 512 B payloads fit. It is capacity only, not the NFR9 512 B limit rule. Broker receipt of the probe was not separately confirmed.

### 2. Steady running at 10 ms poll (NFR5b, CPU cost)

- cyw43_arch_poll call: average 15-20 us, window max about 0.45-0.55 ms, worst seen 1.3-1.4 ms (during the payload probe).
- Whole activation (poll + state machine + publish): about 1.75 ms max per second, about 0.5-0.8 ms otherwise. The extra ~1.3 ms is the RSSI read (cyw43_wifi_get_rssi call max 1.32-1.38 ms), done once per heartbeat.
- Start-up peaks only: 7.2-7.3 ms (join start), 33 ms in one session, at start-up.
- Poll gap max: 10 ms (schedule held exactly).
- Robot-side heartbeat send jitter: max 2-4 ms (up to 11 ms after reconnects), 0 over 100 ms. NFR5b met on the send side.
- Publish errors: 0 in clean runs. RSSI -25 to -64 dBm.
- Laptop side, clean run (n = 493 intervals, ~8 min): 97.6 % within 1 s +/- 100 ms; max interval 2,816 ms, min 0 ms (delayed then bunched delivery); no outage gaps over 3 s.
- So heartbeats leave the robot on time; the late ones are WiFi/hotspot delivery delays. Worst 2.8 s gap is close to the laptop's 3 s "offline after 3 missed heartbeats" rule (risk of a false OFFLINE in the demo). Suspected cause: CYW43 default WiFi power-save. Not yet tested.

### 3. Starvation (Telemetry task polled less often)

| Poll period | hb_dev max | Late (>100 ms) | MQTT | Verdict |
|---|---|---|---|---|
| 45 ms | invalid (test-code catch-up burst, see changes) | - | connected | covered by 95 ms row |
| 95 ms | 50 ms | 0 | connected | within +/-100 ms |
| 245 ms | 225 ms | 2 in ~14 s | connected | fails |
| 495 ms | 236 ms | 1 in ~10 s | connected | fails |
| 1995 ms | 1,490 ms | every heartbeat | connected | fails |

- Threshold between 95 and 245 ms: Telemetry must run at least every ~100 ms for NFR5b. Supports the v3 "poll gap <= 50 ms" proposal with margin.
- MQTT never dropped, even at 2 s polling (5 s keep-alive).
- Slow polling makes each poll longer (more queued work): poll average ~16 us at 10 ms, ~100 us at 95 ms, ~250 us at 245 ms, ~780 us at 495 ms, ~1 ms (peak 2.1 ms) at 1995 ms; activation reached 3.8 ms, over the 3 ms allowance. A starved Telemetry task runs longer when it finally runs.

### 4. Command handling (boot_id + seq rule)

| Sent | Count | Result |
|---|---|---|
| valid ping | 3 | accepted, round trip 83-87 ms |
| duplicate seq | 3 | rejected_duplicate |
| wrong boot_id | 3 | rejected_old_boot |
| unknown command | 3 | rejected_unknown_cmd |
| malformed JSON (trailing comma) | 3 | rejected_malformed |
| 660 B command | 2 | rejected_oversize |

- Robot handling time per command 0.2-0.8 ms; the rest of the round trip is WiFi + broker.
- Burst of 8 back to back: 5 accepted, 3 dropped; robot published `cmd_dropped total=3`, laptop counted 3 sent-never-acked. No silent loss.
- Burst made activation peak at 3.3 ms (queued commands waited 0.6 -> 2.0 ms).
- One malformed command had a 1,225 ms round trip during a WiFi delivery stall (same delayed-then-bunched pattern as the heartbeats).

### 5. Reconnect (NFR5: <= 20 s after AP and broker are back; retry gap <= 2 s)

| Trial | Type | Detection | Recovery | Result |
|---|---|---|---|---|
| 1 | broker off (Ctrl+C) | 3,134 ms (from manual OFF mark) | not measured: robot reconnected 12.1 s before the ON mark. Estimate <= about 8 s (one 5 s retry cycle + 2.8 s connect); see evidence review | PASS (inferred) |
| 2 | broker off | not extracted (search log for "[outage] #2 LOST") | 1,800 ms from manual ON mark (underestimate: excludes the delay between broker start and the press) | PASS |
| 3 | incomplete (broker not restarted or laptop IP changed) | - | did not reconnect within 21 attempts | not counted |

- Retry gaps 0.5 / 1 / 2 / 2 s: meets "retry gap <= 2 s". WiFi stayed up; only MQTT retried.
- Each failed attempt timed out after 3 s instead of being refused at once: Windows silently drops connections to port 1883 while Mosquitto is stopped (firewall). Retry cycle = 3 s timeout + 2 s gap = 5 s.
- Both successful reconnects took ~2.8 s to connect. Two explanations fit the logs: (a) the first SYN after the broker returned was lost and lwIP's ~3 s SYN retransmission got through, or (b) the attempt started while the broker was still down (Windows drops the SYN silently) and the retransmission arrived after it came back. The manual marks cannot tell them apart; (a) is unconfirmed. The 3 s MQTT connect timeout has only ~0.2 s margin over that retransmission.
- Design's WIFI_FAULT point (10 reconnect failures) reached about 43 s after loss, as designed; the test keeps retrying.
- Reconnect activation peak about 3.1 ms (connect + subscribe + first publish), just over 3 ms.
- Earlier run (old mark logic): one outage needed 8-25 s to detect (multiple presses, start time unclear), one apparently 120 s with the hotspot still up - cause not established (likely Mosquitto not actually stopped). Treat as unexplained, not evidence.
- Last session also showed 7 publish errors before an outage and one 42 ms activation spike. Cause unknown; record as unexplained.

### 6. Status LED / buzzer (A15)

- notify_test: both NeoPixels and the buzzer work (colours and beeps seen/heard). The A15 jitter numbers (QUIET/DESIGN/STRESS "period dev max") were not recorded. Rerun notify_test for 1 minute to get them.

## Findings that affect the design (for Version B change log / risks)

1. Join timeout 5 s is tight (measured up to 4.39 s) and the MQTT 3 s timeout is tight against lwIP's ~3 s SYN retransmit (2.8 s connects twice). Raising either pushes the worst-case reconnect sum (failed attempt + gap + successful attempt) past 20 s. Real successful attempts take ~3-4.5 s, so the 18 s budget is conservative. Team decision needed. (An earlier idea of MQTT 1 s / join 7 s is withdrawn: 1 s would never survive one lost SYN.)
2. Telemetry must run at least every ~100 ms to keep heartbeat within +/-100 ms; starvation also lengthens each activation (3.8 ms at 2 s polling).
3. Laptop receive gaps up to 2.8 s with a strong signal: close to the 3 s offline rule. Test WiFi power save off (cyw43_wifi_pm) before the demo; consider a slightly looser offline threshold or a seq-gap rule.
4. RSSI read costs ~1.35 ms per call: read it every 5-10 s, not every heartbeat, or budget it.
5. A command burst can overflow the 4-slot queue (drops are counted and reported). STOP/RESET must not depend on this queue (FR14): give them their own latched path.
6. A burst or reconnect pushes activation to 3.1-3.3 ms: cap commands handled per cycle or accept a longer activation after bursts.
7. After a stall, Telemetry should skip missed heartbeats, not send a catch-up burst (the burst caused 32 publish errors in testing).
8. Windows firewall makes a stopped broker look like a timeout, not a refusal; laptop IP may change when the hotspot restarts (the hotspot appears to use a randomised range). For the demo, keep the broker address fixed (static IP or a hotspot that keeps its range).

## Open items (Buddy 1)

- Hotspot-off and broker reconnect trials with tools/outage_test.py (bench). The two 18 Oct trials are observations, not NFR5 trials: NFR5 needs 20 new worst-phase trials under integrated load.
- Trial 2 detection time (in the log).
- WiFi power-save on/off comparison of laptop receive gaps.
- notify_test A15 numbers.
- Course walk with RSSI (A10).
- Optional: 2-3 more boots for join-time spread.

## Test-code changes made during testing (wifi_mqtt_test v3)

1. Serial prompt build (`wifi_mqtt_test_prompt`): SSID/password/broker IP typed at start-up, prompt reprints every 5 s.
2. Missed heartbeats are skipped and counted ("skipped:") instead of sent as a catch-up burst; buttons read every 1 ms so GP20/GP21 work at any poll period; period change applies immediately. (Found because the 45 ms starvation step was polluted by a catch-up burst and 32 publish errors.)
3. GP21 marks alternate OFF / ON (press 1 / press 2) whatever the connection state; reports "never detected a loss" and "already reconnected before the ON mark". (Old logic recorded every press as OFF if the robot had not yet noticed the outage.)

## Tribunal wording

"Standalone WiFi/MQTT bring-up on the Pico W (no RTOS): connect 2.8-4.4 s + 61-81 ms within the 5 s / 3 s budgets; robot heartbeat jitter <= 4 ms; 97.6 % received within +/-100 ms with worst delivery gap 2.8 s; heartbeat holds with polling every 95 ms but fails from 245 ms; all command rejection rules correct and queue overflow reported; 2 broker-loss trials recovered in <= 2.8 s and 1.8 s (detection 3.1 s), retry gap <= 2 s. These are bench measurements, not NFR acceptance: 2 of 20 reconnect trials, no hotspot-loss trials yet, no integrated load."

## Evidence review (added 25 Oct 2026, from the raw logs)

Solid (keep; re-measure later only inside the RTOS firmware):
- Command rejection rules, duplicate/boot checks and queue-overflow reporting: deterministic, each case repeated 2-3 times.
- Robot-side poll cost, activation time and heartbeat send jitter: instrumented in firmware, many windows.
- Laptop receive statistics: n = 493 intervals over about 8 min.
- Starvation threshold: consistent with arithmetic (a heartbeat can only leave at a poll, so deviation can reach the poll period: 95 ms passes, 245 ms fails). Run on the image before the skip-missed-heartbeat fix, about 10-14 s per step; the conclusion does not depend on either.
- Connect times: valid but only 3 boots.

Weak (redo):
- Reconnect timing. Only 2 trials, both broker-stop (a clean, easy failure); no hotspot loss. Both OFF and ON references were manual button presses. Trial 1 recovery was not actually measured (the earlier "<= 2.8 s" bound does not hold if the broker came back before the last attempt started). Trial 2's 1,800 ms excludes the reaction delay. Finding 1 (MQTT 3 s timeout too tight) rests on an unconfirmed explanation, see above.
- Retest method: tools/outage_test.py (added 25 Oct). Broker mode runs Mosquitto itself, stops it for a random 5-25 s and times the robot's return on the laptop clock, with no human mark; with --serial it also measures detection and whether the successful attempt started before the broker came back. Hotspot mode keeps an Enter-key mark, flags trials with no real outage (INVALID) or a late mark (LATE_MARK), and reports recovery from the mark (conservative). Do about 5 broker + 5 hotspot trials now; the full 20 worst-phase trials NFR5 asks for belong on the integrated firmware under load.

Heartbeat CSV check (hb_2026-10-18.csv, received 25 Oct; 1,747 heartbeats, 19:54-20:48):
- Confirms the clean-run figure: the first four sessions give 482 of 494 intervals within +/-100 ms (97.6 %), worst 2,816 ms.
- Starvation session (all poll periods): 78.9 % within +/-100 ms, as expected; one 7.2 s gap with consecutive seq numbers (20:23:21) is unexplained.
- Outage gaps on the laptop: 140.0 s and 74.2 s in the last session, against robot-side total outages of 118.1 s and 67.7 s for trials 1 and 2. The extra time and the seq jumps (21 -> 43, 126 -> 133) are heartbeats the robot sent before mqtt_watch had reconnected: mqtt_watch goes through the same broker, so it drops with it and reconnects on its own back-off. Laptop-side gaps therefore cannot time broker-outage recovery; outage_test.py avoids this.

Missing: A15 notify_test numbers; trial 2 detection time; the full Tera Term logs still need to be collected from the laptop into the repo (outage_test.py --serial saves the serial log itself from now on).

Note: Version C (submitted) quotes "2 broker-off trials recovered in <= 2.8 s and 1.8 s, measured from manual button marks". Carry the trial 1 correction into later submissions.

The tribunal wording above is kept as it was said on 19 Oct; use this section when quoting the reconnect numbers from now on.
