# Tribunal 1 record (PostTri), text copy

Text of `IS34_PostTri1.doc.docx` (student IDs removed; table cells separated by `|`). Versions B and C hold the team's decisions for Change Requirement 1; where they do not match the code: [`../CHANGE_REQ1_PLAN.md`](../CHANGE_REQ1_PLAN.md) section 6.

```text
INF2004 Embedded Systems Programming
Week 8 Tribunal 1 Template
Architecture Defence & Integration Readiness | AI ideation, live update, AI-vetted gap check
Item
 | Details
 | Purpose
 | Defend Week 6 architecture, justify changes, and show readiness for partial integration.
 | Core workflow
 | Pre-submission AI ideation -> live Zoom-updated version -> final AI-vetted gap check
 | Assessment focus
 | Evidence, engineering judgement, negotiation, and ability to identify missing work
 | Submission format
 | Completed document plus Zoom recording and evidence links
 | Important: this submission has THREE versions inside one document
Version A: Pre-tribunal AI ideation. Each buddy uses AI to ideate their subsystem position before Zoom.
Version B: Live Zoom-updated version. Design decisions are updated during the Zoom discussion through challenge, negotiation, and constraint response.
Version C: Final AI-vetted gap check. After discussion, AI is used as a reviewer to identify missing assumptions, risks, tests, or unclear decisions.
Marks are not awarded for blindly following AI. Marks are awarded for critical evaluation and human engineering judgement; meaning, I want to see you guys discuss during zoom.
 | Team Information
Item
 | Details
 | Group ID
 | P5-IS34
 | Repository Link
 | https://github.com/laun1ngxu4n/INF2004-P5-IS34-EmbeddedSystems.git
 | Buddy
 | Student ID
 | Primary subsystem / persona
 | 1. MUHAMMAD IKHWAN BIN MOHAMAD TAHIR
 | [student ID removed]
 | WiFi, MQTT, telemetry; also, the status LED and buzzer (Notification)
 | 2. NG CHERN LOONG BRANSON
 | [student ID removed]
 | Motors, encoders, PID
 | 3. LAU NING XUAN
 | [student ID removed]
 | IR sensors, line following, barcode
 | 4. MUHAMMAD HAZIQ BIN NAZMI
 | [student ID removed]
 | IMU, terrain, hump/motion events; also, the Safety / Fault Monitor
 | 5. MUHAMMAD IRFAN BIN MD ROSDI
 | [student ID removed]
 | Ultrasonic, servo, obstacle profiling; also, the Mission Controller and integration
 | 
Version A: Pre-Tribunal AI Ideation
Instructions
• Each buddy must use AI before Zoom session to prepare subsystem-specific arguments, risks, and negotiation points.
• The AI output must be critiqued. Identify what is useful, unrealistic, incomplete, or requires validation.
• For Tribunal 1, compare against the Week 6 architecture freeze. For Tribunal 2, compare against actual integration evidence.
 | 
 | Buddy
 | AI prompt used
 | Useful AI ideas
 | Weak / unrealistic AI ideas
 | Position before Zoom
 | 1. MUHAMMAD IKHWAN BIN MOHAMAD TAHIR
 | Act as a strict embedded-systems reviewer helping Buddy 1 prepare for INF2004 Tribunal 1. The attached document is our submitted Week 6 design; the tribunal compares against it. Platform: Pico W on Cytron Robo Pico, C/Pico SDK, micro T-Kernel. I own WiFi/MQTT, telemetry, remote commands and the status LED/buzzer. No test results are supplied in this chat. Treat performance figures as unverified targets unless supported by attached evidence. Distinguish hardware specifications, assumptions, design allocations and measurements.
Verify my summary against the attached submitted Week 6 report. If it conflicts with the attachment, flag the mismatch rather than accepting my wording as fact.
My Week 6 position: Telemetry/WiFi runs at priority 9, below every motion-critical task (only the separate Notification task, priority 10, was lower); the cyw43/lwIP poll runs every 10 ms and never blocks; 1 Hz telemetry and a 1 s ±100 ms heartbeat; reconnect within 20 s of AP and broker returning (budget 8 s failed attempt + 2 s gap + 8 s successful attempt = 18 s); a WiFi fault never stops the robot.
Proposals I plan to raise in Zoom (from our revision after reviewer feedback, not yet agreed by the team):
Merge the Notification task into Telemetry/WiFi (9 tasks become 8): LED and buzzer are driven from notify_q and fault_flg on each 10 ms poll. Reason: status output is not time-critical; saves a task, a stack and a queue consumer.
Drop offline telemetry buffering: while disconnected, telemetry is dropped and counted, only the latest fault event is kept and published with its original timestamp after reconnecting.
Define MQTT payloads (reviewer said schemas were undefined): JSON with units and a sample timestamp; QoS 0 for telemetry/heartbeat, QoS 1 for commands; commands carry {boot_id, seq} and are accepted only for the current boot_id with seq above the last accepted one; reject malformed, unknown, over-512-byte or wrong-state commands with a reason on robot/telemetry/events; retained messages ignored.
An MQTT reset can no longer clear collision or stall faults; those need the GP21 button at the robot.
Check the submitted report's overflow policies before treating them as changes. Retain drop-and-count handling for telemetry and notification where already specified. The proposed change is that safety-detail overflow only counts the loss because the fault cause is independently latched; control-buffer overflow remains subject to its defined inhibition policy.
Single core: preliminary allocation for the poll is <=3 ms per 10 ms, poll gap <=50 ms under full load, total of all tasks about 57 % of one core; to be measured.
Return: (1) up to three arguments I can defend; (2) weak or unrealistic assumptions, especially poll starvation at priority 9, reconnect time, loss-detection time and MQTT buffer/payload limits; (3) what I depend on from each other buddy and what they depend on from me; (4) up to two negotiation points; (5) a small repeatable test with pass criteria and evidence; (6) which of my proposals you would challenge and why. Keep Week 6 claims, my proposals and unmeasured targets separate. Do not invent code, tests or results, and do not fill in the tribunal template for me.
Identify the baseline quantities and dependencies I should bring to Zoom to analyse an unexpected requirement change. For each relevant quantity, state its current target or assumption, what evidence is missing, and which other buddy would be affected. Do not predict the actual injected requirement. Suggest up to three pointed follow-up questions about timing, memory, interfaces or hardware trade-offs.
 
 | Detection time is missing from every timing budget, and this is the strongest point in the output. Week 6 never sets an MQTT keep-alive value, and the 20 s reconnect clock only starts once the network is back. The time it takes to notice the link is down therefore sits outside every budget. On a silent drop, lwIP only notices after about 1.5 × the keep-alive, checked every 5 s, so a 5 s keep-alive adds roughly 10 s before reconnecting even starts. We should set a keep-alive, count detection time in the recovery budget, and use the GP21 marks in the WiFi test to measure how long detection actually takes.
The 512 B telemetry payloads will fail on lwIP’s default settings. lwIP’s MQTT client has a 256 B output buffer by default, so a 512 B publish is rejected before it is ever sent. Week 6 never mentions this. The fix is one line in lwipopts.h, but without it, every full-size message would be dropped, and nobody would notice unless the publish error code is checked. So the AI is right to raise it.
The boot_id + seq duplicate rejection has two holes. The Week 6 heartbeat does not carry boot_id, so the laptop has no way to learn the boot_id it must put in its commands, or to notice that the robot has restarted. In the other direction, if the laptop restarts, its seq goes back to zero and the robot rejects every command as a duplicate until seq catches up. Both fixes are small: add boot_id to the heartbeat, and give commands a sender session ID so the robot resets its last-seen seq when the session changes.
The 3 ms per poll estimate does not fit the CPU budget. At one poll every 10 ms, 3 ms is 30% of the CPU, which is more than half of the 57% total for a priority-9 task. Either the estimate is too pessimistic or the budget is wrong, and we cannot tell which without measuring. The WiFi test prints the poll’s average and worst-case time every second, so one run gives the real figure.

 | Merging Notification into Telemetry does not make the status LED more starvable. In Week 6, Notification ran at priority 10, below Telemetry at 9, so it was already the task most likely to be starved. Moving the LED and buzzer into Telemetry raises their effective priority. The real starvation risk is in Telemetry’s own network polling, and the poll budget and heartbeat jitter test already cover that. The AI argued against the merge without checking the Week 6 priority table it was given.
The PIO conflict depends on two conditions that have not happened. Encoders only move to PIO if the measured edge rate goes above about 10,000 edges/s, and encoder_test has not been run yet. Even then, the RP2040 has two PIO blocks with four state machines each, and CYW43 and the NeoPixel use only one state machine each. It is a resource to check if encoders move to PIO, not a design conflict we need to defend now.
CYW43 setup blocking and the venue hotspot are true but have nothing to do with runtime behaviour. The only blocking step is the CYW43 chip setup, which happens once at start-up before the robot is placed on the track. Joins and reconnects during a run use the non-blocking connect call polled every 10 ms, so WiFi cannot delay Motion or Safety. Hotspot availability is a logistics issue for the demo-day checklist, not a weakness in the architecture.
Missing command latency is only a gap if commands become time-critical. In the Week 6 design the robot runs its mission autonomously, and remote commands are operator inputs, not part of the safety path: GP21 and the Safety task can stop the robot without WiFi. A latency figure only matters if the requirement change puts a remote command on a time-critical path. The WiFi test already measures command round-trip time through the laptop watcher, so we have a figure to quote if that happens. This is something to prepare for the Zoom, not a flaw to fix now.
 
 | Defend: WiFi at lowest priority; a WiFi fault never stops the robot; reconnect ≤20 s. Propose: merge Notification into Telemetry; drop the offline buffer; reject duplicate MQTT commands (boot ID + sequence). Risk: poll starved at P9.
 | 2. NG CHERN LOONG BRANSON
 | Act as a strict embedded-systems reviewer helping Buddy 2 prepare for INF2004 Tribunal 1. The attached document is our submitted Week 6 design; the tribunal compares against it. Platform: Pico W on Cytron Robo Pico, C/Pico SDK, micro T-Kernel. I own the motor driver (GP8-GP11), quadrature wheel encoders (GP4/5, GP16/17), PID speed control, distance moves and spin turns. No test results are supplied in this chat. Treat performance figures as unverified targets unless supported by attached evidence. Distinguish hardware specifications, assumptions, design allocations and measurements.
 
Verify my summary against the attached submitted Week 6 report. If it conflicts with the attachment, flag the mismatch rather than accepting my wording as fact.
 
My Week 6 position: Motion owns normal motor writes; Safety/STOP may invoke the guarded motor_safe_off exception; PID runs every 10 ms; Mission is the only writer of motion_cmd_q and line steering is accepted only in LINE_STEER; targets are 90° turns within ±10°, 180° within ±15°, distance within ±5 % over 1 m, U-turn under 3 s.
 
Proposals I plan to raise in Zoom (from our revision after reviewer feedback, not yet agreed by the team):
1. Replace the Week 6 completion_slot handshake with a MOTION_DONE result message {epoch, ID, status, stopped} posted to mission_evt_q; Mission ignores results with a stale epoch/ID.
2. After Mission cancels a move, it sends a STOP command in the new epoch; that STOP's own result is the evidence that the robot is stationary.
3. Encoders stay on the shared SDK GPIO callback for now; if the measured edge rate at top speed is above about 10,000 edges/s, counting moves to PIO (reviewer flagged four quadrature pins plus barcode, echo and buttons on one callback).
4. Single core (core 0); preliminary allocation Motion <=1 ms per 10 ms, total about 57 % of one core; to be measured.
5. Critical control-buffer overflow must inhibit motion according to the agreed policy, and capacities must be validated against measured bursts. Check line_cmd_q separately: its row specifies draining to the newest steering value, but its full-buffer behaviour needs an explicit decision. Do not assume all buffers have identical overflow handling.
6. The bypass margin uses a provisional ±5° heading-error assumption. About 39 mm drift over a 450 mm leg assumes a 5° absolute heading error during that leg; relative errors from successive turns may accumulate. Clarify the required heading accuracy and validate the complete manoeuvre sequence before accepting the margin.
 
Return up to three defensible arguments, weak assumptions, dependencies on each other buddy, up to two negotiation points, and practical tests for encoder resolution, distance/turn accuracy and shutdown under load. Check whether ±5° per turn is realistic and whether it means relative error per turn or absolute heading after several turns. Explain when GPIO counting is adequate and what evidence would justify PIO. Challenge my proposals where they are weak, and do not suggest new handshakes without a clear need. Do not invent results or fill in the tribunal template for me.
 
Identify the baseline quantities and dependencies I should bring to Zoom to analyse an unexpected requirement change. For each relevant quantity, state its current target or assumption, what evidence is missing, and which other buddy would be affected. Do not predict the actual injected requirement. Suggest up to three pointed follow-up questions about timing, memory, interfaces or hardware trade-offs.
 
 | Heading error must be treated as cumulative, not per turn. The 39 mm drift only holds for a 5° absolute error over one 450 mm leg (450 × sin 5° ≈ 39 mm). NFR8 allows ±10° per 90° turn, which gives about 78 mm on its own. The 450 mm advance is leg 4, so only the spins in legs 1 and 3 come before it. If those two errors stack to ±20° in the worst case, leg 4 drifts about 154 mm (450 × sin 20°). That is above the proposed 100 mm margin, and if the drift is outward it could push the robot past the 300 mm NFR6a limit. This is arithmetic from the NFR8 targets, not a measurement. We need to ask B5 for the required absolute heading accuracy over the whole manoeuvre, and then test the full five-leg sequence.
Decide GPIO vs PIO on measured criteria, not a bare edge-rate figure. The 10,000 edges/s trigger is our own assumption and is not in the Week 6 report. The evidence to use is the encoder ISR worst-case time, invalid quadrature transitions, and added timestamp jitter on the barcode and echo edges. encoder_test already logs peak edges/s, ISR time and invalid transitions. It has not been run yet. It should be run first with the wheels lifted, then with the wheels loaded. Measuring jitter on barcode and echo edges needs a combined test with those interrupts on the same callback, which encoder_test does not do.
line_cmd_q needs its own full-buffer decision. Week 6 only says Motion drains to the newest value. It does not say what Line does when the buffer is full, and line_cmd_q is not marked critical like motion_cmd_q. A plain drop-and-count rule would throw away the newest steering value, which is the one Motion wants. The rule should keep the newest value instead: overwrite the oldest, or use a single-slot latest value. The existing 60 ms no-fresh-steering brake in LINE_STEER is then the backstop. We must confirm this with B3 and not copy the critical-overflow rule from the other queues.
 
 | Replacing completion_slot with a MOTION_DONE message is not justified yet. The Week 6 slot already carries {epoch, id, status, stopped}, so the message adds no new identity information. It also removes the back-pressure where Motion never overwrites an unacknowledged result. The result would then compete with junction and start events on mission_evt_q (16 × 32 B, where overflow is a critical fault). We should only accept the change if we can name a specific failure the slot cannot handle (A18), or show that removing the WAIT_ACK handshake saves enough complexity to justify losing the back-pressure. “Cleaner messaging” on its own is not enough.
Proposal 2 is not a new mechanism. Week 6 already confirms a stop with STOP_CONFIRM, and Motion’s STOP state only reports after the encoders show no movement for 100 ms, so a STOP result already is stationary evidence. The proposal is a clarification. If Motion hangs, no result is ever sent, so Mission’s timeout (A16) and Safety’s motor_safe_off must remain the backstop.
Several numbers are targets or allocations, not results. The ~57 % CPU, the ≤1 ms per 10 ms Motion budget, the 10,000 edges/s trigger and the ±5°/39 mm bypass figure are not in the submitted Week 6 design. None of them has been measured. Wheel diameter, counts per revolution and slip are also unmeasured, because encoder_test has not been run and the results file is blank. The concern that speed from Δcount per 10 ms may be too coarse at low speed depends on counts per revolution, so it is a risk to check, not a finding.
Shutdown tests must not blur the two paths. Week 6 separates the 20 ms responsive-CPU path from the watchdog path (≤500 ms plus boot time). A software-only timing check does not prove the driver pins are safe during reset and boot (A20). The test needs a scope on GP8–GP11 with Motion running, with Motion hung, and across a reset.
 
 | Defend: Motion is the only PWM writer; PID every 10 ms. Propose: replace the completion_slot handshake with a result message only if a failure the slot cannot handle is found (A18). Risk: encoder resolution unmeasured; bypass turns may need ±5°.
 | 3. LAU NING XUAN
 | Act as a strict embedded-systems reviewer helping Buddy 3 prepare for INF2004 Tribunal 1. The attached document is our submitted Week 6 design; the tribunal compares against it. Platform: Pico W on Cytron Robo Pico, C/Pico SDK, micro T-Kernel. I own two digital line sensors (IR_LEFT GP0, IR_CENTER GP1), the offset barcode sensor (IR_BARCODE GP2), line following every 20 ms, Code 39 decoding (A=Left, B=Right, C=Straight, D=U-Turn) and junction reporting. The line is 18 mm wide and barcodes start 20 mm beside it. No test results are supplied in this chat. Treat performance figures as unverified targets unless supported by attached evidence. Distinguish hardware specifications, assumptions, design allocations and measurements.
 
Verify my summary against the attached submitted Week 6 report. If it conflicts with the attachment, flag the mismatch rather than accepting my wording as fact.
 
My Week 6 position: line tracking with last-seen-side memory; barcode widths measured from ISR edges and odometry; READ_FAIL clears the old command and the next junction goes straight; right-branch pulses are never counted as READ_FAIL; at 150 mm/s the result reaches Mission within 200 ms of the last bar (133 ms for the 20 mm quiet zone, <=67 ms for decoding and posting).
 
Proposals I plan to raise in Zoom (from our revision after reviewer feedback, not yet agreed by the team):
1. The ISR stores a timestamp and the raw encoder count at every IR_BARCODE edge, instead of the 10 ms odometry used in Week 6.
2. Reviewer said encoder resolution for 3 mm bars is unproven: measure counts per revolution first; if resolution is coarser than about 1 mm, measure element widths by time at constant barcode speed instead, since the 1.8x wide/narrow test uses ratios and works at any steady speed.
3. Reviewer said two digital line sensors are a risk on curves: accepted as a risk; last-seen memory stays, and curves and right-hand drift are validated on the track before junction logic is built on top.
4. GP0/GP1 are the SDK's default UART0 debug pins: debug output moves to USB and GP0/GP1 stay as IR inputs.
5. Staged build: line following is in the first build stage, barcode decoding with junction commands in the second.
 
Challenge last-seen tracking on curves and right drift, separating 9 mm barcode bars from right branches, the count-versus-time fallback and its speed assumption, and the 200 ms budget. Explain my interface boundaries with Mission and Motion. Return up to three defensible arguments, weak assumptions, dependencies, up to two negotiation points and a small test plan (bar measurements, encoder resolution, curves/junctions, decode latency). Separate course specifications from my assumptions, do not claim anything has passed without data, and do not fill in the tribunal template for me.
 
Identify the baseline quantities and dependencies I should bring to Zoom to analyse an unexpected requirement change. For each relevant quantity, state its current target or assumption, what evidence is missing, and which other buddy would be affected. Do not predict the actual injected requirement. Suggest up to three pointed follow-up questions about timing, memory, interfaces or hardware trade-offs.
 | The 1.8× acceptance test can fail from a small threshold bias, even at perfect speed. If the potentiometer sits slightly off-centre, every bar reads wider and every space narrower by the same amount δ. Week 6 compares all nine elements together, so the worst case is a wide space (9 − δ) against a narrow bar (3 + δ). The test then fails once δ is above about 1.3 mm. Classifying bars against bars and spaces against spaces cancels the bias completely. A potentiometer sweep over a printed test barcode would measure δ before we trust any threshold. This is arithmetic from the Week 6 widths, not a measurement.
The left-junction threshold should be a distance, not a time. Week 6 detects a left junction as “sustained” IR_LEFT plus IR_CENTER black, calibrated against tight curves, but a time threshold changes meaning with speed. At 10 cm/s, 100 ms covers 10 mm of track; at 25 cm/s it covers 25 mm. Defining the threshold in millimetres of encoder travel keeps it the same at every speed in the A3 tests. Line already records the encoder position for each junction, so the distance is available without a new interface.
Keeping branch candidates separate from READ_FAIL is the strongest part of the design to defend. In Week 6 an over-length IR_BARCODE pulse is posted on branch_q at its trailing edge, and the partial capture is discarded without a READ_FAIL. Only Line confirms it as a junction, against IR_CENTER history. A right junction therefore can never cancel a valid pending command, and junction decisions have a single owner. The risk to check runs the other way. If the sensor’s spot is wide enough that a narrow 3 mm space never registers, a wide bar, narrow space and wide bar read as one black pulse of about 21 mm, and a real barcode is treated as a branch. The A7 pulse-length measurements need to show that margin.
 
 | The AI’s speed-tolerance figures overreach. It said element speeds can differ by about 60% with no threshold bias and only about 10% with a 1 mm bias. The gap behind it is real: Week 6 discards a capture on a “speed violation” but never sets a threshold. But the figures come from a simplified worst case, one wide element read at one speed and one narrow element at another. In practice the barcode is about 140 mm long (three characters of roughly 45 mm each, plus gaps), crossed in under a second at 150 mm/s, and speed drifts gradually under PID rather than jumping between elements. We should record the missing threshold as a gap, and set the real limit from speed ripple logged during the A11 passes, not quote 60% or 10%.
The AI’s view that the 200 ms budget is reasonable adds nothing. It agreed with the budget and then noted it is unmeasured, which we already knew, and its supporting claim that decoding takes microseconds is also unmeasured. Week 6 spends 133.3 ms on the 20 mm quiet zone, leaving at most 66.7 ms for the decoder to wake, the quiet-zone check on its 10 ms timeout, and preemption at priority 7 below five other tasks. The real question is worst-case preemption under full load. Only a GPIO toggle at the last bar edge and at the nav_cmd_q post, with every task running, can answer it. The AI also missed that the bound only holds at 150 mm/s: Week 6 itself notes the quiet zone alone takes 200 ms at 100 mm/s.
 
 | Defend: READ_FAIL clears the old command and the robot goes straight; branch pulses are never READ_FAIL. Risk: two line sensors on curves; 3 mm bars vs encoder resolution. Propose: raw encoder counts at edges, time-based fallback.
 | 4. MUHAMMAD HAZIQ BIN NAZMI
 | Act as a strict embedded-systems reviewer helping Buddy 4 prepare for INF2004 Tribunal 1. The attached document is our submitted Week 6 design; the tribunal compares against it. Platform: Pico W on Cytron Robo Pico, C/Pico SDK, micro T-Kernel. I own the IMU subsystem (LSM303DLHC on I2C1 at 100 Hz, tilt/hump estimation, collision detection, motion classification) and the Safety / Fault Monitor (fault latching, task heartbeats, battery monitoring on GP28, reset button on GP21, motor_safe_off, and hardware watchdog). No test results are supplied in this chat. Treat performance figures as unverified targets unless supported by attached evidence. Distinguish hardware specifications, assumptions, design allocations and measurements.
 
Verify my summary against the attached submitted Week 6 report. If it conflicts with the attachment, flag the mismatch rather than accepting my wording as fact. 
 
My Week 6 position: Safety runs at the highest task priority and forces the motors safe via the guarded motor_safe_off exception within 20 ms even if Motion hangs (backed by a ≤500 ms hardware watchdog if the CPU hangs); only Safety clears fault bits, and a reset never automatically restarts motion; the IMU task is the sole user of I2C1 at 100 Hz (data >40 ms old latches a fault); humps are detected when tilt exceeds 5° for 5 consecutive samples with peak height estimated to ±5 mm using odometry; impacts >2 g latch a collision fault.

Proposals I plan to raise in Zoom (from our revision after reviewer feedback, not yet agreed by the team): 
 
1. Replace fault generation counters with one latched bit per cause in stop_reasons / fault_flg, set before any message is posted and cleared only by Safety inside a short interrupts-off critical section. 
 
2. Collision and stall faults can only be cleared via the physical GP21 button on the robot (MQTT reset can no longer clear them), while keeping the rule that reset never restarts motion. 
 
3. Overflow on safety_evt_q only counts the lost detail event because the fault cause is independently latched in stop_reasons / fault_flg, whereas critical control-buffer overflow remains subject to its motion-inhibition policy. 
 
4. Because the LSM303DLHC has no gyroscope and linear acceleration corrupts pitch, test both hump-height estimators on 10/20/30 mm humps and freeze one before integration. 
 
5. Single core (core 0); preliminary CPU allocation for Safety and IMU fits within the ~57% total core 0 budget; to be measured. 

Return: (1) up to three arguments I can defend; (2) weak or unrealistic assumptions, especially accelerometer-only hump height (±5 mm), the >2 g collision threshold vs normal hump entry, stop_reasons race conditions, and the 20 ms safe-off budget; (3) what I depend on from each other buddy and what they depend on from me; (4) up to two negotiation points; (5) a small repeatable test with pass criteria and evidence; (6) which of my proposals you would challenge and why. Keep Week 6 claims, my proposals and unmeasured targets separate. Do not invent code, tests or results, and do not fill in the tribunal template for me. 
 
Identify the baseline quantities and dependencies I should bring to Zoom to analyse an unexpected requirement change. For each relevant quantity, state its current target or assumption, what evidence is missing, and which other buddy would be affected. Do not predict the actual injected requirement. Suggest up to three follow-up questions about timing, memory, interfaces or hardware trade-offs.
 
 | Order of Operations on Fault Latching: Set the fault bit in stop_reasons before posting to safety_evt_q. That way Motion still sees the stop on its next 10 ms check even if the queue is full, and a full safety_evt_q only drops the detail log instead of missing a stop. Clearing fault bits also needs to happen in a short interrupts-off check so a GP21 or MQTT reset doesn't accidentally wipe out a new fault or B5's STOP and OBSTACLE bits (A18).
 
Separating Hump Entry from Collision Impacts: Separate hump entry from collisions by how long the spike lasts, not just the peak value. Since the LSM303DLHC has no gyroscope, hitting the bottom of a hump gives a sharp jolt that throws off the tilt reading and could spike past 2 g (FR4, A4, A14). Requiring 5 samples in a row (50 ms at 100 Hz) helps filter out quick bumps, and we need to test on real 10/20/30 mm humps before fixing the 2 g collision threshold.
 
Two-Path Shutdown Verification: Test the 20 ms software stop and the 500 ms watchdog stop separately. Software logs alone don't prove the motor pins stay safe during a reset (A20). We need to check GP8–GP11 on a scope with Motion running normally, with Motion hung, and across a watchdog reset. 
 
 | 6-DOF Complementary or Kalman Filtering: The AI initially suggest Using a complementary or Kalman filter with a gyroscope doesn't match our hardware. The LSM303DLHC only has an accelerometer and a magnetometer, so there is no gyro to filter out linear acceleration when climbing a hump.

Double-Integrating Accelerometer Z-Axis for Hump Height: Double-integrating vertical acceleration to get hump height is unrealistic. With motor vibration and sensor drift, integrating acceleration twice will drift past the ±5 mm target very quickly. Combining tilt with B2's encoder distance is much more practical, though it still depends on wheel slip and pitch noise (A2, A4).

Treating Design Allocations as Validated Margins: 
Treating our Week 6 targets like measured results. The 0.03 g acceleration estimate, ±5 mm hump height, 2 g collision threshold, 40 ms IMU timeout, and 20 ms stop time are all unmeasured targets right now (A4, A14, A20). Our hump height calculation also depends on B2's encoder counts per revolution, which hasn't been measured yet.
 
 | Defend: Safety is highest priority and motor_safe_off works even if Motion hangs. Propose: one latched bit per cause instead of generation counters; collision/stall reset by button only. Risk: no gyroscope for hump height.
 | 5. MUHAMMAD IRFAN BIN MD ROSDI
 | Act as a strict embedded-systems reviewer helping Buddy 5 prepare for INF2004 Tribunal 1. The attached document is our submitted Week 6 design; the tribunal compares against it. Platform: Pico W on Cytron Robo Pico, C/Pico SDK, micro T-Kernel. I own the Mission Controller, the HC-SR04 ultrasonic sensor (TRIG GP6, ECHO GP7) on a scanning servo (GP14), obstacle profiling, the five-leg bypass and system integration. No test results are supplied in this chat. Treat performance figures as unverified targets unless supported by attached evidence. Distinguish hardware specifications, assumptions, design allocations and measurements.
Verify my summary against the attached submitted Week 6 report. If it conflicts with the attachment, flag the mismatch rather than accepting my wording as fact.
My Week 6 position: Mission is the only writer of motion_cmd_q, with a fixed event priority order; the first valid echo under the trip distance (>=300 mm) inhibits drive; stop >=150 mm from the obstacle at <=250 mm/s; bypass stays within 300 mm of the original line and rejoins within 500 mm.
Proposals I plan to raise in Zoom (from our revision after reviewer feedback, not yet agreed by the team):
Defer RETRACE/RETURN_CHECK to stretch scope: a blocked bypass leg stops the robot and latches FAULT.
Remove RELEASE_REQ: Mission clears OBSTACLE only after the applicable clearance and state checks, whether authorising an approved bypass leg, resuming after a confirmed-clear path, or entering line recovery after leg 5. Other stop/fault inhibits remain effective. The recovery handover must not leave OBSTACLE latched and prevent all movement.
After cancelling a move, Mission sends a STOP command in the new epoch and uses its result as evidence the robot is stationary.
Reviewer said bypass time and drift were not estimated: propose a provisional 100 mm side margin. A 5° absolute heading error over a 450 mm leg produces about 39 mm lateral drift. Successive relative turn errors may accumulate, so the margin and complete bypass geometry remain subject to measurement. The current estimate of about 15.5 s plus <=4 s line recovery also needs checking against the selected wheel speeds and settling times.
Wire the ultrasonic with jumper wires to GP6/GP7, not Grove 5/7, which share GP26 (IMU SDA) and GP28 (battery sense).
Staged build plan: Stage 1 vertical slice (Motion PID and encoder moves, line following, STOP/fault path with motor_safe_off, 1 Hz telemetry); Stage 2 barcode junction commands and IMU hump/collision; Stage 3 obstacle detection, scans and bypass. Stretch: retrace, second hump estimator, offline telemetry buffering, richer snapshots.
Challenge the five-leg bypass, per-leg OBSTACLE permission, side checks, coverage during the first turn, freshness of stationary scans, and the handover into line recovery after leg 5 (inhibition must not deadlock recovery). Check the 100 mm margin, accumulated heading error and the obstacle-depth assumption. Treat no-target sonar behaviour as module-specific and unverified. Return up to three defensible arguments, weak assumptions, dependencies on all buddies, up to two negotiation points and a small integration test sequence. Prefer short clarifications over new inter-task handshakes, do not invent clearance evidence, and do not fill in the tribunal template for me.
Identify the baseline quantities and dependencies I should bring to Zoom to analyse an unexpected requirement change. For each relevant quantity, state its current target or assumption, what evidence is missing, and which other buddy would be affected. Do not predict the actual injected requirement. Suggest up to three pointed follow-up questions about timing, memory, interfaces or hardware trade-offs.
 
 | Removing RELEASE_REQ is safe only if Mission keeps the checks Safety used to make. In Week 6, Safety cleared OBSTACLE only after confirming the robot was stationary, the clearance evidence was ≤120 ms old and no new cause had arrived. If Mission simply clears the bit, there is a race. The Obstacle task (priority 8) can set OBSTACLE again from a new echo after Mission has read the scan result, and Mission (priority 4) then wipes out the new obstacle. Mission’s clear should be a check-and-clear under the shared-state lock that refuses if OBSTACLE was set again since the scan it is answering. Week 6 also capped the wait with a 100 ms release timeout. Without RELEASE_REQ, Mission still needs its own limit (A16).
The echo rule can read a hung sensor as clear. Week 6 treats ECHO rising and staying high until the timeout as “nothing in range”, and two such readings as clear, which lets the robot continue. Some HC-SR04 clones are reported to hold ECHO high after a missed echo, and a stuck-high ECHO line would look the same, so a failed sensor could read clear on every ping. The fix is small. Require ECHO to be low before each trigger, or the reading is invalid. Only trust “high until timeout” once our module’s no-target waveform has been checked on a scope (A5). Our bring-up ultrasonic_test already reports STUCK_HIGH and NO_FALL separately, so it can show which behaviour our module has.
The forward beam is narrow at the trip distance. Week 6 gives the beam as about 15°. At the 300 mm minimum trip distance, that cone is only about 80 mm wide (2 × 300 × tan 7.5°). If the chassis is wider than that, an obstacle in front of one side of the robot enters the beam only once it is closer than the trip distance, or never, and the NFR6 stopping budget no longer holds. FR5 says obstacles are “on or near the line”. We should either confirm that course obstacles sit across the line, or state the sideways offset we can detect and test with offset targets. This is arithmetic from the Week 6 beam figure. The real beam of our module is unmeasured (A5).
 
 | Pinging faster than every 60 ms adds little and creates a new risk. The AI suggested cutting the ping period to 20–30 ms to shorten detection time. At 250 mm/s, saving 40 ms means 10 mm less travel, which is small next to the 150 mm stopping allowance inside the 300 mm trip distance. The HC-SR04 datasheet recommends a cycle of over 60 ms so that a late echo from the previous ping isn’t read as a close obstacle, which would cause false stops. The bigger unknowns in the NFR6 inequality are braking distance, sensor setback and the Obstacle task’s own delay. Those should be measured first.
Raising Mission’s priority targets the wrong task. The AI argued that Mission at priority 4 delays the reaction to obstacles. In Week 6, the Obstacle task sets the OBSTACLE inhibit before posting, and Motion checks stop_reasons before every PWM update, so stopping never waits for Mission. Mission’s delay only affects when the scan and bypass start, after the robot has already stopped. The delay that does count in the NFR6 budget is the Obstacle task’s, at priority 8, below Line, IMU and Barcode. That is the one to measure under load; it’s no reason to move Mission.
 
 | Defend: Mission is the only motion writer; first close echo inhibits drive. Propose: defer RETRACE (blocked leg = FAULT), drop RELEASE_REQ only with a guarded check-and-clear in Mission, 100 mm provisional side margin, staged build.
 | Week 6 Architecture Defence Snapshot Before Zoom
Select only 5 to 7 important Week 6 architecture claims that affect integration readiness. Do not repeat the entire Week 6 document. Each row should defend one claim using evidence, remaining weakness, and the buddy responsible. One smart-toaster example is shown; replace it with your own project claim.
Week 6 artefact
 | What the team claimed in Week 6
 | Evidence available now
 | Still weak or untested
 | Buddy defending it
 | Task table; Figures 1, 1b
 | Mission is the only writer of motion_cmd_q; Line steers only in LINE_STEER; events handled in a fixed priority order.
 | Design only: task table, Figures 1/1b, Section 5 dispatch order.
 | Not implemented; simultaneous events and stale results after cancel untested (A18).
 | Buddy 5
 | Task priorities; Figure 6; NFR10
 | Safety has the highest priority and forces the motors safe via motor_safe_off within 20 ms, even if Motion hangs.
 | Design only: priority table, Figure 6, A20 plan.
 | Shutdown time unmeasured; driver state at reset/boot unchecked (A20).
 | Buddy 4 (with Buddy 2)
 | Figure 4; NFR8
 | Encoder PID every 10 ms: 90° turns within ±10°, distance within ±5 % over 1 m, U-turn under 3 s.
 | Design only: Figure 4, NFR8, U-turn estimate ≈1.4 s.
 | Encoder counts/rev, wheel size and slip unmeasured (A2).
 | Buddy 2
 | Figure 5; NFR4
 | Code 39 decoded from ISR edge timestamps and encoder counts; READ_FAIL clears the old command and the next junction goes straight.
 | Design only: Figure 5, NFR4 budget (133 ms quiet zone, ≤67 ms left).
 | Resolution for 3 mm bars unmeasured (A11); bar widths unconfirmed (A7); two-sensor tracking on curves untested (A3).
 | Buddy 3
 | Figure 6; FR4
 | Humps detected from tilt (>5° for 5 samples); peak height to ±5 mm; impacts >2 g latch a collision fault.
 | Design only: Figure 6, 0.03 g analysis.
 | No gyroscope; pitch disturbed by acceleration; ±5 mm and collision threshold untested (A4, A14).
 | Buddy 4
 | Figure 7; NFR6, NFR6a
 | First valid echo under the trip distance (≥300 mm) inhibits drive; stop ≥150 mm from the obstacle at ≤250 mm/s; bypass within 300 mm.
 | Design only: Figure 7, NFR6 trip-distance formula.
 | Braking distance, sensor setback and 3.3 V HC-SR04 unvalidated (A5); bypass clearance untested (A9).
 | Buddy 5
 | Figure 3; NFR5
 | Telemetry 1 Hz, heartbeat 1 s; reconnect ≤20 s after AP and broker return; WiFi faults never stop the robot.
 | Design only: Figure 3, 18 s reconnect budget.
 | No reconnect trials (A1); priority-9 poll starvation unmeasured.
 | Buddy 1
 | 
Shared Resource Allocation Matrix
Enter at least 3 shared resources, or more if the project has additional shared buses, timers, interrupts, actuators, sensors, queues, event groups, or shared data. A resource is “shared” if more than one task, module, interrupt, or buddy can affect it. 
Shared resource
 | Used by task(s)
 | Owner task / module
 | Access type
 | Contention risk
 | Mitigation
 | Evidence needed
 | GPIO interrupt callback (one SDK handler)
 | Encoder, Barcode, Echo and Button ISRs
 | ISR dispatcher (Buddy 5)
 | Interrupt
 | Encoder edges (×4 per count) at top speed delay barcode/echo timestamps.
 | ISRs only count, timestamp, signal; move encoders to PIO if edge rate too high.
 | Edge-rate calculation; ISR timing log (A21).
 | Motor outputs GP8–GP11
 | Motion, Safety (motor_safe_off), STOP handler
 | Motion Control
 | PWM write / forced safe state
 | Motion's next PWM update could undo a safe-off.
 | Inhibit latched first; Motion checks stop_reasons before every write.
 | Safe-off log with Motion running and hung (A20).
 | stop_reasons / fault_flg
 | All fault producers, Button ISR, Safety, Motion, Mission
 | Safety (faults); Mission (STOP, OBSTACLE)
 | Set, read, cleared
 | A clear could erase a new fault or a STOP during START.
 | Set before any message; clear in a short interrupts-off check.
 | Fault-injection race tests (A18).
 | Encoder counts / odometry
 | Encoder ISR (write); Motion, Barcode ISR, IMU, Mission (read)
 | Motion Control
 | Multi-word shared data
 | Torn reads; coarse barcode widths from 10 ms odometry.
 | Raw counts from ISR; snapshots copied with interrupts off.
 | Bar widths vs ruler; odometry check.
 | PWM slices
 | Motion (slices 4/5), Obstacle servo (slice 7), buzzer (slice 3)
 | Each owner its own slice
 | PWM output
 | A shared slice would force one frequency (servo needs 50 Hz).
 | Separate slices by pin choice.
 | Pin/PWM test with all running.
 | CPU time on core 0
 | All tasks; WiFi poll at priority 9
 | RTOS scheduler
 | Processor time
 | WiFi poll starved; MQTT keep-alive dropped.
 | Single core; allocations ≈57 %; Telemetry lowest priority.
 | Timing trace; longest poll gap (A21).
 | 
Subsystem Interface and Code Structure Map
Enter at least one interface or module per buddy. Each row should identify who provides the module, who consumes it, what inputs and outputs it exposes, and how it will be validated. 
Module / file / interface
 | Provider
 | Consumer(s)
 | Purpose
 | Inputs
 | Outputs
 | Interface rule
 | Validation rule
 | telemetry.c
 | Buddy 1
 | All tasks, Mission, Barcode, Safety
 | WiFi/MQTT, telemetry, command routing, LED/buzzer
 | telemetry_q, notify_q, MQTT
 | MQTT publishes; start/stop/map/reset to queues
 | Only task using CYW43/lwIP; never blocks.
 | Reconnect ≤20 s; heartbeat 1 s ±100 ms.
 | motion.c, motor_safe_off()
 | Buddy 2
 | Mission, Line, Safety
 | PID, moves, spin turns, stall detection
 | motion_cmd_q, line_cmd_q, encoders, stop_reasons
 | PWM GP8–11; result {epoch, ID, stopped}; odometry
 | Only Motion writes PWM; Safety may only call motor_safe_off.
 | 1 m ±5 %; 90° ±10°; safe-off ≤20 ms.
 | line.c
 | Buddy 3
 | Motion, Mission
 | Line tracking, junctions, line loss
 | IR GP0/GP1 every 20 ms; branch_q; line_ctl_q
 | line_cmd_q; JUNCTION, LINE_LOST, LINE_FOUND
 | No direct motor calls.
 | 3 laps, no loss >300 ms.
 | barcode.c
 | Buddy 3
 | Mission, Line
 | Code 39 decoding, mapping table
 | Edge ring (time + encoder count); barcode_cfg_q
 | nav_cmd_q (VALID or READ_FAIL); branch_q
 | Owns the map; updates between captures only.
 | 20 passes per A–D, ≥18/20.
 | imu.c
 | Buddy 4
 | Safety, Telemetry
 | Tilt, humps, collisions, motion class
 | LSM303DLHC on I2C1 at 100 Hz; odometry
 | Hump events; collision bit + safety_flg
 | Only IMU task uses I2C1; data >40 ms old = fault.
 | 10/20/30 mm humps ±5 mm.
 | safety.c
 | Buddy 4
 | Motion, Mission, all producers
 | Fault latch, heartbeats, battery, reset, watchdog
 | stop_reasons, safety_evt_q, alive_flg, GP28, GP21
 | fault_flg; motor_safe_off(); watchdog feed
 | Only Safety clears fault bits; reset never restarts motion.
 | Fault injection: safe ≤20 ms, no restart.
 | obstacle.c
 | Buddy 5
 | Mission
 | Ranging, scans, profile, bypass legs
 | ECHO GP7, servo GP14, obs_ctl_q
 | obs_evt_q; OBSTACLE bit
 | Sets OBSTACLE before posting; never drives motors.
 | Stop ≥150 mm at 25 cm/s; width ±20 mm.
 | mission.c
 | Buddy 5
 | Motion, Line, Obstacle, Barcode
 | System state machine; motion arbiter
 | nav_cmd_q, obs_evt_q, mission_evt_q, results
 | motion_cmd_q, line_ctl_q, obs_ctl_q, ARM/DISARM
 | Only motion_cmd_q writer; stale epoch/ID ignored.
 | Simultaneous-event injection: no stale advance.
 | 
Week 6 Assumption Closure
Enter at least one assumption per buddy, preferably copied from Week 6. Each row should show whether the assumption is still open, validated, deferred, or converted into a risk. 
Week 6 assumption ID
 | Original assumption
 | Current status
 | Evidence collected
 | Remaining risk
 | Next action
 | A1
 | WiFi/MQTT supports 1 Hz telemetry and reconnects within 20 s.
 | Open
 | None yet (design only).
 | Reconnect time and poll starvation unmeasured.
 | 20 AP off/on cycles under load. (Buddy 1)
 | A2
 | Encoders give accurate speed and distance without significant slip.
 | Open
 | None yet (design only).
 | Counts/rev, wheel size and slip unknown.
 | Measure counts/rev; 1 m runs and 90°/180° turns. (Buddy 2)
 | A3
 | Digital IR sensors separate line, junctions and tight curves.
 | Open
 | None yet (design only).
 | Two sensors may lose the line on curves.
 | Tune pots on track; log 3 laps. (Buddy 3)
 | A11
 | Encoder resolution can measure 3 mm and 9 mm bars.
 | Open
 | None yet (design only).
 | Bars may not separate; decoding fails.
 | Test barcode 20× at 10/15/25 cm/s; time-based fallback. (Buddies 2, 3)
 | A4
 | The IMU detects humps and estimates peak height to ±5 mm.
 | Open
 | None yet (design only).
 | No gyroscope; ±5 mm may be unreachable.
 | Test both estimators on real humps; freeze one. (Buddy 4)
 | A5
 | HC-SR04 meets ranging targets at 3.3 V (or protected 5 V).
 | Open
 | None yet (design only).
 | May need 5 V + divider; no-target waveform unknown.
 | Scope ECHO; 50–500 mm range tests. (Buddy 5)
 | 
Version B: Live Zoom-Updated Version

 | Acceptable evidence examples: GitHub commit link, short test log, video timestamp, oscilloscope or serial monitor screenshot, pin test result, task timing measurement, AI prompt and critique, meeting decision note, or photo of hardware wiring.
Buddy Contributions During Zoom
Each buddy must complete one row. The contribution must be a real technical challenge, defence, trade-off, or correction raised during Zoom, not only an attendance statement.
Buddy
 | Key challenge, concern, or argument raised
 | Challenge received
 | Decision or compromise reached
 | Buddy 1
 | The change requirement says to put Wi-Fi loss in the shared fault bitmask. But our Week 6 design says a WiFi fault must never stop the robot, and Motion stops if any bit in that mask is set. If we follow the example literally, a WiFi drop stops the robot.

 | Bitmask is owned by Buddy 4, so we have to align what the flag would do.
 | Mask out the Wi-Fi loss bit for the motor, so instead of stopping the robot, we just send a warning.
 | Buddy 2
 | Packing should not touch the encoder ISR or the PID loop. If an ISR and a task write different fields in the same packed word, one write can overwrite the other. I proposed packing only where data is handed over (queue messages, the result slot, telemetry) and keeping encoder counts and PID values at 32 bits.
 | Buddy 3: "I completely agree with your safety concern. Packing the encoder ISR or PID loop into shared bit-fields is extremely dangerous and will cause race conditions. I fully support keeping the absolute encoder counts and PID values at 32 bits."
"However, we cannot just ignore the new requirement. The professor explicitly stated: 'For MQTT communication, application data should be represented using compact binary fields rather than sending individual values as text.' If we don't compress the encoder data somewhere, we will fail the Week 13 audit."
"So here is my challenge to your proposal: We must pack the data at the boundary, not at the source. You keep the ISR and PID at 32 bits. But when your Motion Task sends the motion_cmd_q message to Mission, or when it sends the result to mission_evt_q, we pack that message into a smaller byte array. For my barcode decoder, you don't send me the 32-bit absolute count; you send me a 16-bit delta. This satisfies the requirement while keeping your ISR safe."

 | Buddy 3 “I agree we pack at the boundary and keep the ISR and PID at 32 bits. Two corrections. Mission sends me the commands, I only send the result, so I’ll pack that. And I won’t send barcode deltas. You read my count in your ISR, keep the low 16 bits and subtract consecutive edges.”
 | Buddy 3
 | The new Minimum-Bit requirement means my current ISR edge struct (ir_edge_t) and the branch_q message sizes are too bloated. My ir_test.c uses an 8-byte struct per edge. I want to pack the 3 IR sensor IDs, the edge level, and the gap/ambiguous flags into a single uint8_t, keeping only the 32-bit timestamp separate. This cuts RAM usage by over 60%. However, I need to know if we have enough bits for the encoder counts at top speed without overflowing.
 | Buddy 2: "If we compress the encoder counts to 16 bits to save space, we might overflow during a long run.”

 | Buddy 3 will pack the ir_edge_t and the branch_q messages into bit-fields. The 32-bit timestamp stays separate. For nav_cmd_q, the mapping ID and command are packed into 8 bits. Buddy 2 will provide raw 16-bit counts for the barcode measurement, but Buddy 3 will handle the overflow by using a 16-bit delta from the last known position, not an absolute 16-bit position. All shared bit-fields will be read inside a critical section.
Buddy 2: “That works for me, as long as Motion keeps its 32-bit counts, you use differences between consecutive edges with the reference reset at each capture, and the critical section stays a few microseconds. Please measure it.”

 | Buddy 4
 | Connected my Version A proposal (stop_reasons using 1 latched bit per fault, set before posting messages) directly to the Curveball's shared fault bitmask rule, and packed IMU/Safety data into minimum bits (6 bits for 0–30 mm hump height, 3 bits for 6 motion classes, 3 bits for 7 IMU states, 2 bits for 3 Safety states). 

 | Raised two challenges: (1) adding Wi-Fi loss to the shared fault bitmask must not stop the motors, and (2) multiple tasks/ISRs packing bits into the same shared variable will overwrite each other unless protected.
 | Pause interrupts for a split second whenever any task edits shared bits in stop_reasons, allow only Safety to clear fault bits, and mask out the Wi-Fi loss bit during motor-stop checks so Wi-Fi drops only trigger a warning.
 | Buddy 5
 | The minimum-bit requirement means I need to pack my mission states, bypass progress and ultrasonic readings into smaller storage. However, a missing echo must not be treated as a clear path, and reducing command IDs or timestamps could cause an old result to be accepted. I proposed agreeing on shared data formats with the other buddies before changing the interfaces.
 | How will Mission distinguish a valid clearance reading from a timeout, and prevent an old movement completion from advancing the bypass after the data is packed?
 | Proposed: use a common header with agreed bit layouts and packing/unpacking functions. Keep explicit sensor-status information and justify command-ID and timestamp widths before reducing them. Retain freshness and command-identity checks, then measure memory savings and test that outdated results cannot resume movement.
 | Live Decision Change Log
Enter at least 3 changes AND 3 confirmed no-change decisions from the Zoom discussion. Include the change type and evidence link or timestamp whenever possible. One row may state “No change” if the team defended the Week 6 design successfully.
Original idea / Week 6 design
 | Updated decision
 | Change type
 | Reason for change
 | Evidence link / timestamp
 | Owner / follow-up
 | MQTT messages sent as text (JSON)
 | Compact binary messages, with a version number at the start
 | Major redesign
 | The change requirement says MQTT must use compact binary, not text
 | New Requirement doc page 1
 | B1: Design the formats, update the laptop decoder, record current text sizes first
 | WiFi is low priority and never stops the robot; reconnect within 20 s
 | Kept
 | No change 
 | Message format doesn’t affect connecting; bench reconnects took under 3 s
 | Buddy 1 bench results
 | Buddy 1
 | ir_edge_t uses a uint8_t sensor, uint8_t level, uint8_t ambiguous, uint8_t after_gap, plus uint32_t t_us.
 | Pack the flags and sensor ID into a single uint8_t using bitmasks (2 bits for sensor, 1 bit each for level, ambiguous, gap). Keep uint32_t t_us.
 | Simplified
 | The original struct wastes 3-4 bytes per edge due to C alignment/padding. The packed version saves ~60% memory per ISR event.
 | ir_test.c struct definition; New Requirement doc page 1.
 | Buddy 3
 | branch_q messages store the edge position, pulse length, and timestamp as separate 32-bit ints.
 | Pack the branch_q message: 14 bits for position (max 10,000 mm), 7 bits for pulse length (max 127 mm).
 | Simplified
 | Meets the new requirement to use only the bits needed for the valid range. Reduces queue RAM usage.
 | New Requirement doc page 2.
 | Buddy 3
 | Use standard enum (32-bit) for the Line Following state machine (IDLE, LINE_TRACKING, JUNCTION, etc.).
 | Use a uint8_t bit-field restricted to 3 bits (max 8 states).
 | Simplified
 | The state machine only has 5 states. Using 32 bits wastes 29 bits per state variable.
 | New Requirement doc page 1.
 | Buddy 3
 | No packing comparison
 | Freeze comparable builds and measure RAM, messages and timing.
 | Clarified
 | Logical bit savings do not prove allocated RAM savings.
 | [timestamp].
 | All; B5 consolidate
 | Unspecified field widths
 | Create one range/units/status/width table for all application data.
 | Clarified
 | Avoid incompatible scales and missing exceptional values.
 | Change Requirement 1; implementation pending.
 | B5 coordinates all.
 | Pin layout and hardware
 | Kept
 | No change
 | Software-only change
 | NIL
 | All
 | Safety runs at the highest priority (Priority 2) and cuts power to the motors via motor_safe_off within 20 ms even if the Motion task freezes, backed by a 500 ms hardware watchdog timer if the whole CPU locks up.
 | Kept
 | No change
 | Guarantees the robot can always execute an emergency stop during a crash even if Buddy 2's Motion task hangs.
 | NIL
 | B4/B2
 | Resource and Assumption Review
Item challenged
 | Evidence presented
 | Decision made
 | Impact on other buddies
 | CPU and shared interrupts
 | Single-core/~57% is a proposal/allocation.
 | Verify access contexts; measure IRQ, lock and polling gaps under load.
 | All; B5 coordinates timing.
 | Sonar and bypass
 | A5/A9 open; 50 mm baseline, 100 mm proposed margin.
 | Validate waveform and swept clearance; reject unknown/infeasible corridor.
 | B5 with B2 motion and B3 recovery.
 | Shared packed words
 | Week 6 guarded pending state; draft proposes bit latches.
 | Proposed: retain occurrence-aware acknowledgement or prove equivalent protection.
 | All producers; B4/B5 clear policy and B2 gating.
 | Encoder/PID precision
 | A2/A11 open; rate and resolution unmeasured.
 | Measure range/time bounds; analyse signed arithmetic and quantisation.
 | B2 supplies B3/B4/B5.
 | Queues and MQTT
 | Proposed capacities; actual allocation unknown.
 | Measure record and allocated bytes separately; preserve overflow rules.
 | All; B1 laptop and MQTT changes.
 | IMU height and collision
 | No gyro; A4/A14 open.
 | Freeze one tested estimator; do not delay collision shutdown using hump confirmation.
 | B4 with B2 odometry and B1 telemetry.
 | Risk Assessment and Subsystem Test Plan
Integration Risk Register
Enter at least 3 integration risks. Include at least one timing or scheduling risk, one hardware or wiring risk, and one interface or team-integration risk. 
Risk ID
 | Risk
 | Cause
 | Impact
 | Likelihood
 | Severity
 | Mitigation
 | Owner
 | Status
 | R1
 | Timing
 | Packing/IRQ/lock overhead
 | Missed control or sensing deadlines
 | Medium
 | High
 | Bound lock/ISR work; measure combined load.
 | All/B5
 | Open
 | R2
 | Wiring
 | Unverified sonar supply/ECHO voltage
 | Damage or unsafe range
 | Medium
 | High
 | Scope ECHO; validate GP6/GP7 and protected supply.
 | B5/B4
 | Open
 | R3
 | Interface
 | Scale/layout/version mismatch
 | Wrong command or measurement
 | Medium
 | High
 | Shared codec spec; reject malformed input; golden vectors.
 | B1/B5/all
 | Open
 | R4
 | Shared state
 | Lost update or stale reset evidence
 | Fresh stop/fault erased
 | Medium
 | High
 | Guard set/clear and acknowledgement; race tests.
 | B4/B5/B2
 | Open
 | R5
 | Identity
 | Wrap while stale records survive
 | Old completion accepted
 | Medium
 | High
 | Bound lifetime/reuse; test cancel, wrap and restart.
 | B5/B2/B1
 | Open
 | R6
 | Numerical
 | Narrowing/quantisation/overflow
 | Control or sensing error
 | Medium
 | High
 | Range/error analysis and physical regression.
 | B2/B3/B4/B5
 | Open
 | R7
 | Clearance
 | Invalid/stale sonar or incomplete sweep
 | Unsafe bypass/recovery
 | Medium
 | High
 | Preserve status, age and per-move guards; safe FAULT.
 | B5/B2/B3
 | Open
 | R8
 | Tasks Overwriting
 | Multiple buddies' tasks updating the shared bitmask at the same time without protection (race condition).
 | A new crash fault gets accidentally erased, so the robot fails to stop.
 | Medium
 | High
 | Briefly pause interrupts when editing shared fault bits so no bits get wiped out, and ignore the WIFI_LOSS bit during motor-stop checks.
 | B4/B1/B2/B5
 | Open
 | 
Subsystem Test Plan
Enter at least 4 cross-subsystem tests. Each test must involve the work of at least two buddies and must include a success criterion and evidence type, such as a video, serial log, screenshot, timing result, or commit link. At least TWO test should involve timing, safety, or shared-resource concern. 
Subsystem
 | Test case
 | Success criteria
 | Evidence required
 | Owner
 | Status
 | Motion/Safety/Mission
 | STOP/fault during PWM; Motion unresponsive.
 | Responsive-CPU safe output <=20 ms; no re-enable while inhibited. Watchdog checked separately.
 | Scope request/output trace; build ID.
 | B2/B4/B5
 | Planned
 | Safety/shared state
 | Two causes; new fault during reset; full detail queue.
 | No lost inhibit; STOP/OBSTACLE retained; reset never restarts.
 | Injected interleaving and output logs.
 | B4/B5/B2
 | Planned
 | Mission/Motion
 | Cancelled completion and forced identity wrap.
 | No stale leg advance; STOPPED confirmation not ordinary completion.
 | Epoch/ID/state trace; lifetime bound.
 | B5/B2
 | Planned
 | Mapping/navigation
 | Update during capture; READ_FAIL + junction; bad packet.
 | One map per capture; old turn clears; rejected map leaves valid map intact.
 | Packet/capture/navigation logs.
 | B1/B3/B5
 | Planned
 | Binary interfaces
 | Boundary/signed/reserved/short/version inputs.
 | Valid values round-trip; invalid input rejected before partial application.
 | Firmware/laptop golden vectors.
 | B1/all
 | Planned
 | Integrated load
 | All tasks, reconnects and telemetry saturation.
 | Control deadlines; >=1 Hz telemetry; 1 s +/-100 ms heartbeat; <=20 s reconnect after services return.
 | Task/IRQ/poll/queue/broker logs.
 | All/B5
 | Planned
 | Obstacle/recovery
 | Threshold, invalid sonar, blocked leg and leg-five handover.
 | Unknown cannot grant clearance; normal stop >=150 mm at <=250 mm/s; recovery <=4 s or safe FAULT.
 | Waveform, distance and state/video evidence.
 | B5/B2/B3/B4
 | Planned
 | Motion/hump regression
 | 1 m/turn trials; known 10/20/30 mm humps.
 | Distance +/-5%; 90/180 turns +/-10/15 degrees; valid hump peaks +/-5 mm; count failed trials.
 | Measured trials; raw/decoded data.
 | B2/B3/B4/B5
 | Planned
 | RAM/messages
 | Equivalent baseline and modified builds.
 | Actual allocations and message sizes reported; required limits checked; no unsupported savings.
 | Map/size files; payloads; stack high-water.
 | All/B5
 | Planned
 | 
Constraint Injection / Curveball Response
Instructions
• Record the injected constraint exactly as given.
• Explain how it affects all five buddies, even if the impact differs by subsystem.
• A good response shows prioritisation, trade-off reasoning, fallback behaviour, and evidence-aware decision making.
 | Injected constraint
 | Buddy affected
 | Immediate impact
 | Adaptation / compromise
 | Minimum-Bit Data Representation; Change Requirement 1.
 | Buddy 1
 | MQTT, telemetry, command/session fields and laptop formats change.
 | Proposed: explicit binary codecs; validate length/version/range; preserve ages/validity; measure payload and allocated RAM separately.
 | Same requirement for IMU, Safety and fault conditions.
 | Buddy 4
 | Raw/derived readings, ages, masks and acknowledgement fields change.
 | Proposed: configured sensor ranges/error budgets; IRQ-safe access; retain diagnostic severity, new causes and reset without restart.
 | Same requirement for Mission, obstacle handling and integration.
 | Buddy 5
 | States/events, legs, angle, echo status, geometry, freshness and IDs change.
 | Proposed: count actual states; common field table; guard clearance/identity; migrate both ends together; collect Week 13 evidence.
 | Same requirement throughout application data.
 | Buddy 2
 | Counts, speed, PWM, PID, odometry and completions need bounds.
 | Proposed: measure rate/resolution; justify modulo interval and numerical scales; preserve snapshots, safe-off and stale rejection.
 | Same requirement for Line/Barcode and shared capture data.
 | Buddy 3
 | Sensor/action/status, map versions, ring records and indices change.
 | Proposed: retain NONE/READ_FAIL/branch semantics; publish entries safely; update mapping between captures; verify physical precision.
 | Integration Readiness Self-Assessment
Complete all rows. Use Green only if there is evidence, Yellow if partly ready or untested, and Red if the area is unclear or blocking Week 10 integration.
Readiness area
 | Status
 | Evidence
 | Concern
 | Hardware wiring known
 | Green
 | Pin layout is confirmed on week 6
 | Nil
 | RTOS task ownership clear
 | Yellow
 | Task/interface ownership documented.
 | Conflicting or unseen ownership
 | Shared resources identified
 | Green
 | GPIO/PWM, snapshots, fault/stop words and CPU listed.
 | Access correctness untested; document per-object protection.
 | Critical statecharts implementable
 | Yellow
 | Week 6 transitions and event precedence.
 | Release, acknowledgement, wrap and recovery handover unverified.
 | Safety or fault handling defined
 | Yellow
 | Discussed and Inhibition, safe-off and reset contracts.
 | Bit-only reset and check-to-PWM ordering unresolved; shutdown unmeasured.
 | Week 10 partial integration feasible
 | Yellow
 | Defined interfaces and staged build proposal.
 | Needs electrical/safe-off closure, basic integration and agreed formats.
 | 
Updated readiness position before the Week 10 partial integration demo:
________________________________________________________________________________
________________________________________________________________________________
________________________________________________________________________________
________________________________________________________________________________
________________________________________________________________________________
Version C: Final AI-Vetted Gap Check
Instructions
• After the live discussion, give the updated Version B content to AI and ask AI to act as a strict embedded systems reviewer.
• Ask AI to identify missing assumptions, weak evidence, unclear interfaces, untested failure modes, and resource conflicts.
• For every AI-identified gap, decide whether to fix, reject, or defer it, and explain why.
 | 
 | AI reviewer prompt used
 | Act as a strict embedded-systems reviewer for INF2004 Tribunal 1, group P5-IS34 (Pico W on Cytron Robo Pico, C/Pico SDK, micro T-Kernel). Attached are our submitted Week 6 design (the baseline), Change Requirement 1 (Minimum-Bit Data Representation), Buddy 1’s bench test results and our Version B tribunal document. Review Version B only. Identify missing assumptions, weak or missing evidence, unclear interfaces, untested failure modes, resource conflicts, and contradictions between sections or with the Week 6 design. For each gap, say where it is, why it matters, and what evidence or decision would close it. Keep Week 6 claims, proposals and measurements separate. Do not rewrite our entries, do not decide fix/reject/defer for us, and do not invent results.
 | AI-identified gap
 | Team response
 | Action taken
 | Evidence / reason
 | The fault-bitmask decision contradicts itself across sections. The Buddy 4 row adopts one latched bit per cause, which matches the change requirement’s example. But the Resource Review row says “retain occurrence-aware acknowledgement or prove equivalent protection”, which means keeping the Week 6 generation counts. The readiness row then calls “bit-only reset” unresolved. Version B doesn’t say which rule the team adopted. With bits only, a reset must clear just the bits it checked, so a fault arriving mid-reset survives. That needs a stated snapshot-then-clear rule and the A18 race test.
 | Fix, but not with the suggested rule. We keep one latched bit per cause. Snapshot-then-clear is not enough: if the same fault happens again while its bit is already set, the bit does not change, so clearing it still erases the new occurrence.
 | B4 will choose one reset rule with B5, and the team will use it everywhere. We propose a second bitmask of “new since last check” bits: producers set the fault bit and its tracking bit together; reset clears the tracking bits before it starts validating, and at the end, in one critical section, clears a fault bit only if its tracking bit is still clear and the cause is gone, so a persistent fault is never cleared.
The fault mask keeps one bit per cause; the extra mask holds reset-tracking information, so B4 will justify its need and memory cost. The rule stays a proposal until it passes the A18 test (a different cause and the same cause injected during reset, with reset never restarting motion) and the team agrees it.
 | Version B's Buddy 4 row and Resource Review row disagree. Week 6 A18 already requires injecting the same safety bit during acknowledgement, so any replacement for generation counts must pass that test.
 | The Wi-Fi-loss warning conflicts with “only Safety clears fault bits”. Buddies 1 and 4 agreed to mask WIFI_LOSS out of the motor-stop check. Version B doesn’t say: 
Which word the bit lives in. If it’s in stop_reasons, Telemetry at priority 9 does interrupts-off read-modify-writes on a safety-critical word.
Who clears it. The Buddy 4 decision says only Safety clears fault bits, so the warning may stay latched after WiFi reconnects.
Week 6 already treats Telemetry heartbeat loss as a Safety warning, so the relationship between the two isn’t stated. No test checks that a WiFi drop leaves the robot running.
 | Fix. We keep the Zoom decision: WIFI_LOSS is a bit in the shared fault bitmask, as the requirement's example says, but it is a warning, not a stop cause. Masking it only in Motion is not enough, because Mission, Line, Barcode and Obstacle also read fault_flg, so FAULT_ACTIVE must never be raised by the WiFi bit.
 | We propose two masks: a motion-inhibit mask (stop/fault bits, excluding WIFI_LOSS) and a warning mask, so only inhibit bits can raise FAULT_ACTIVE. Telemetry would set WIFI_LOSS on disconnect and clear it on reconnect through the shared IRQ-safe helper, with “only Safety clears” applying to inhibit bits. Telemetry heartbeat loss stays a Safety warning, as in Week 6. B1 and B4 will agree the final rule.
B1 will test it by dropping WiFi during line following: the robot should keep running, with the warning set and then cleared after reconnect.
 | Change Requirement 1 lists Wi-Fi loss as an individual bit in a shared fault bitmask. Week 6 Section 5 handles WiFi loss without system FAULT. The Week 6 RTOS object list shows fault_flg is read by Motion, Mission, Line, Barcode, Obstacle and Notification.
 | MQTT STOP can be lost: an untested failure mode that’s missing entirely. In Buddy 1’s bench test, a burst of 8 commands into the 4-slot command queue dropped 3. Week 6 FR14 says operator STOP must stay pending whatever the buffer capacity. Version B never says that MQTT STOP sets the latched STOP bit independently of that queue, and no test sends STOP during a command burst. Packing makes commands smaller, but it doesn’t remove the drop path.
 | Fix, high priority. We would word it as “STOP surviving a full command queue is not shown yet” rather than “STOP can be lost”, but the gap is real. FR14 already says operator STOP must stay pending whatever the buffer capacity, and smaller messages do not remove the drop path.
 | B1, with B5, will make Telemetry set stop_reasons.STOP as soon as it accepts an MQTT STOP, through the same set helper the GP21 button uses, before posting anything to a queue.
They will test it by sending STOP in the middle of an 8-command burst: the robot should stop, and no later queued command should restart it.
 | Buddy 1 bench results, 18 Oct: an 8-command burst into the 4-slot queue gave 5 accepted, 3 dropped. The drops were counted and reported (cmd_dropped total=3), so loss is visible, but a dropped STOP would still not stop the robot. Week 6 FR14; Section 4: “A STOP press sets stop_reasons.STOP immediately.”
 | The Buddy 2 decision may not meet the change requirement. Keeping encoder counts and PID values at 32 bits is a reasonable concurrency choice. But the requirement explicitly lists “motion control and encoder data” and asks each field to use only the bits its valid range needs. Version B gives no range showing that 32 bits is the minimum (it depends on unmeasured counts per revolution and run length). It also hasn’t asked the lecturer whether computational state is in scope. Without one of those, an assessor can read this as non-compliance.
 | Partly accept. Keeping working encoder and PID values at 32 bits is still our choice for arithmetic and concurrency reasons, but we agree that a blanket rule does not show minimum-bit compliance. Each kept width needs a range.
 | B2 will measure counts per revolution (A2) and, from the longest run and top speed, give the range, signedness and width of each stored field, listing PID accumulators and temporary intermediates separately.
B5 will ask the lecturer whether internal computational state (including PID accumulators) and temporary arithmetic intermediates count as application data.
 | Change Requirement 1 lists “motion control and encoder data” first among the areas it covers, so encoder data is clearly in scope. Counts per revolution and run length are unmeasured (A2 open), so no width can be justified yet.
 | The buddy rows disagree on who truncates encoder counts. 
In the Buddy 3 row, “Buddy 2 will provide raw 16-bit counts”, and Buddy 3 uses a 16-bit delta.
In the Buddy 2 row, Buddy 3’s ISR reads the 32-bit count, keeps the low 16 bits and subtracts consecutive edges.
The branch_q change row uses an absolute 14-bit position in mm.
That’s three different interfaces for one value. Units are also inconsistent: Week 6 branch_q carries encoder positions, not mm. A mismatch here gives wrong bar widths or turn positions without any visible error.
 | Fix. One interface: Motion keeps the 32-bit count. The Barcode ISR reads it at each edge and stores the low 16 bits, and bar widths are differences between consecutive edges (mod 2¹⁶). Motion sends no separate 16-bit value. Branch position is a separate field with its own unit.
 | B2 and B3 will write this interface and the branch_q unit (encoder counts, or mm converted once by Barcode using B2's measured mm per count) into the shared field table.
They will test it by decoding the same capture from 16-bit differences and from 32-bit counts; both should give the same bar classification.
 | Buddy 2 and Buddy 3 rows describe different providers; Week 6 branch_q carries encoder positions, not mm. A 16-bit difference is unambiguous only while fewer than 65,536 counts pass between two edges. That should hold easily for 3–9 mm bars, but it must be confirmed once counts per mm are measured (A2), and a capture with an edge gap longer than the capture timeout must be discarded. It also assumes forward motion; Barcode is disarmed during turns.
 | The ir_edge_t RAM claim is arithmetically wrong. The original struct (four uint8_t fields, then a uint32_t) has no padding: it’s 8 bytes. The proposed struct (one uint8_t plus one uint32_t) is padded back to 8 bytes by the compiler, so the saving is 0 %, not “~60 %”. The stated reason (“wastes 3–4 bytes due to padding”) is also incorrect. A real saving needs, for example, the 5 flag bits plus a 27-bit timestamp in one 32-bit word (4 bytes, 50 %), a time delta, or separate arrays. Also, in Week 6 only IR_BARCODE (GP2) produces interrupt edges, so the 2-bit sensor ID may not be needed in the final firmware.
 | Accept. The “~60%” saving is wrong: the original layout is already 8 bytes with no padding, and the proposed one is padded back to 8 bytes. We withdraw the claim.
 | We withdrew the claim here, since Version B cannot be edited.
B3 will choose a layout that really saves memory (for example, flag bits plus a shorter timestamp in one 32-bit word, or separate arrays), report sizeof and ring allocation before and after, and check whether a sensor ID is needed at all.
 | 4 × uint8_t + uint32_t = 8 bytes; uint8_t + uint32_t = 8 bytes after alignment. Week 6 has a 128-slot edge ring. Only IR_BARCODE (GP2) is an interrupt input in the Week 6 pin table. A 27-bit µs timestamp wraps after about 134 s, which is fine for differences within one capture.
 | he concurrency rule is backwards in places. The Buddy 3 decision says “all shared bit-fields will be read inside a critical section”. On the RP2040, an aligned 32-bit read is already atomic. The real hazard is several writers doing read-modify-write on one word, and the Cortex-M0+ has no exclusive-access instructions for that. Version B doesn’t state a per-word ownership rule (which words have one writer and need no lock, which have several and need the interrupts-off helper). It also relies on single-core operation, which is still a v3 proposal, not an agreed decision. On dual core, disabling interrupts isn’t enough.
 | Fix, with a qualification. We agree the missing part is the writer side. We do not agree that reading inside a critical section is backwards: an aligned 32-bit read is atomic, but snapshots of several fields (e.g. odometry x, y, heading) still need the lock.
 | B5 will add a per-object table, with input from all buddies, listing each object's writers and readers, whether it has one writer or several, and its protection (none, interrupts-off helper, or spinlock if dual core). The team will confirm single core as a decision, and B5 will measure the longest interrupts-off section.
 | The RP2040's Cortex-M0+ has no exclusive-access instructions, so read-modify-write with several writers needs interrupts off. Week 6 Section 4 already specifies a short IRQ-safe lock, plus a hardware spinlock if both cores are used.
 | The curveball wasn’t recorded “exactly as given”, and the impact entries are generic. The template asks for the constraint word for word. Version B has only the title. The five impact rows (split across two tables) don’t quote Week 6 numbers, don’t rate impact (High or Low), don’t state fallback behaviour, and don’t prioritise across buddies, which the template says a good response shows.
 | Fix the wording; partly accept the rest. We have now quoted the constraint exactly. The template asks for prioritisation rather than High/Low labels, so we added a priority order instead.
 | We added the exact requirement wording to Version C:
“Change Requirement: Minimum-Bit Data Representation. The robot car firmware must minimise the number of bits used to represent application data. Each data field representing application information must use only the number of bits required to represent its valid range, using C bit-fields or explicit masks and shifts where appropriate. Examples include: A value ranging from 1 to 5: 3 bits; A state machine with up to 8 states: 3 bits; Fault conditions such as motor stall, encoder fault, IMU fault, no echo and Wi-Fi loss: use individual bits within a shared fault bitmask, rather than separate integer variables. The requirement applies to application data throughout the firmware, including: 1. motion control and encoder data; 2. line and barcode sensing; 3. obstacle detection; 4. hump estimation; 5. vehicle and controller states; 6. data shared between tasks or between an ISR and a task; 7. inter-task queue messages; and 8. MQTT telemetry and command payloads. For MQTT communication, application data should be represented using compact binary fields rather than sending individual values as text. When multiple values are packed into the same storage unit and that data is shared between tasks, or between an ISR and a task, access must be implemented safely. Read-modify-write operations on shared packed data must not introduce race conditions, and atomic access or appropriate synchronisation must be used where required.” Week 13 evidence: 1. Data representation table; 2. RAM usage comparison; 3. Message size comparison; 4. Correctness and performance.
We set this priority order: (1) shared fault/stop words (B4, B5), because a lost fault is a safety failure; (2) cross-buddy formats: MQTT wire format (B1) and encoder interface (B2, B3); (3) each buddy's local fields.
Fallback: if packing a shared field into one word cannot be made race-free, that field will keep its minimum width but get its own storage unit instead of sharing a word.
 | Version B's curveball table records only the requirement title and general proposals.
 | Some field widths don’t match their stated ranges. 
Hump height “6 bits for 0–30 mm”: 0–30 at 1 mm resolution needs 5 bits. 6 bits only makes sense with a stated finer resolution (0.5 mm) or a range above 30 mm (A8).
Line state “only has 5 states”: Week 6 Figure 5 has 6 (IDLE, LINE_TRACKING, JUNCTION_DETECTED, PAUSED, LINE_SEARCH, FAULT). It’s still 3 bits, but the justification is wrong.
branch_q’s 10,000 mm maximum assumes a course of 10 m or less, which isn’t stated as an assumption.

 | Fix. All three points are correct.
 | B4 will use 5 bits for hump height (0–30 mm at 1 mm), or keep 6 bits only with a stated 0.5 mm resolution, and will define the out-of-range value.
B3 will correct the Line state count to 6 (still 3 bits) and state the course-length assumption behind the 14-bit branch position (max 16,383), plus the behaviour beyond it.
 | 0–30 is 31 values, which fit in 5 bits. Week 6 Line states: IDLE, LINE_TRACKING, JUNCTION_DETECTED, PAUSED, LINE_SEARCH and FAULT.
 | The wire format isn’t defined. Buddy 1’s row says “explicit binary codecs”, but Version B never says the MQTT format must use masks and shifts rather than C bit-fields. Bit-field order is chosen by the compiler, so the laptop decoder could read the bits in the wrong order. Byte order (little-endian), the version-byte rule and the reserved-bit handling aren’t stated, so the golden-vector test has no specification to test against.
 | Fix. MQTT payloads will be packed into a byte array with explicit shifts and masks. A C bit-field struct is never sent directly.
 | B1 will write the wire spec: little-endian, byte offsets, widths, scales, a version byte, and reserved bits sent as 0 and ignored on receipt. B1 will then update the laptop decoder to match and share golden test vectors between firmware and laptop.
 | C bit-field order and padding are chosen by the compiler, so the laptop could decode bits in the wrong order. The golden-vector test in Version B has no spec to test against yet.
 | Evidence is weak across the change log and contributions. 
No recording timestamps anywhere.
Two no-change rows say “NIL”.
Several rows cite only the requirement document, which shows where a rule came from, not that the team decided something.
The WiFi no-change row (“bench reconnects under 3 s”) leaves out that this is 2 of the 20 trials NFR5 requires, both broker-off, with no RTOS load.
The A1 entries in Version A’s Snapshot and Assumption Closure tables say “no reconnect trials” and “None yet”, but Buddy 1’s bench data now exists. Version C should state A1’s updated status, since Version A can’t be edited.

 | Accept. The bench results show partial progress on A1, not readiness under integrated load.
 | We updated A1's status here to partly evidenced, still open. Bench results (standalone test, no RTOS): join plus DHCP 2.8–4.4 s and MQTT connect 61–81 ms, within the 5 s / 3 s budgets; 2 broker-off trials recovered in ≤2.8 s and 1.8 s, measured from manual button marks; retry gaps ≤2 s. Hotspot-loss trials and the full 20 worst-phase trials under integrated load remain to be done.
In later submissions, we will give a reason instead of “NIL” (e.g. “software-only change, no hardware evidence needed”).
 | Buddy 1 bench results file, 18 Oct, with the Tera Term log and hb_2026-10-18.csv. Version A’s A1 entries say “None yet”; Version A cannot be edited, so A1’s updated status is recorded here.
 | The change log is missing decisions made in the contributions. No change rows record Buddy 2’s “pack at the boundary, keep the ISR and PID at 32 bits”, Buddy 4’s fault bitmask with the Wi-Fi mask, or Buddy 5’s common header with explicit sensor-status fields. Buddy 2 and Buddy 4 have no change rows at all.
 | Fix. These decisions are recorded here, since Version B cannot be edited.
 | We recorded the missing decisions here: (a) B2/B3: pack at interfaces, keep the encoder ISR and PID at 32 bits. Agreed; widths still to be justified (row 4). (b) B1/B4: one latched bit per cause; the WiFi loss bit masked out of motor-stop checks so it only gives a warning. Agreed in Zoom; the reset rule (row 1) and the extra masking detail (row 2) are post-review proposals. (c) B5: common header with packing functions and explicit sensor status. Proposed.
 | Version B Buddy Contributions rows for Buddies 2, 4 and 5, recorded during the Zoom session.
 | Key decisions have no test. 
Nothing checks that a WiFi drop during line following leaves the robot running with a warning (decision 2).
Nothing checks MQTT STOP during a command burst (gap 3).
Nothing checks that packed 16-bit encoder deltas give the same bar widths as 32-bit counts on the same capture (gaps 4 and 5).
The RAM/messages test has no defined “before” build. Very little code exists, so the team needs to say whether “before” means an unpacked implementation, a compile-time switch, or the Week 6 design sizes.

 | Fix. Each key decision gets a test with a pass criterion.
 | We will add three tests: (1) WiFi drop during line following: the robot keeps running, with the warning set and cleared (B1, B3, B4). (2) MQTT STOP inside a command burst: the robot stops and does not restart (B1, B5, B2). (3) Packed versus 32-bit encoder widths on the same capture: same classification (B2, B3).
For the RAM baseline, we will use one build with a compile-time switch (packed on/off), keeping the same features, buffer capacities and compiler settings.
 | None of these cases is in the Version B test plan. Very little unpacked code exists, so a switch gives a fair before/after; comparing against Week 6 estimated sizes would not measure real savings.
 | Readiness contradicts the risk register. “Hardware wiring known” is Green with “pin layout confirmed in Week 6”, but R2 lists unverified sonar supply and ECHO voltage as an open wiring risk. Green needs evidence such as a wiring photo or pin test, and a design table isn’t that. “Updated readiness position” is blank
 | Accept. A pin table is a design, not proof that the wiring works. Hardware readiness drops to Yellow.
 | We changed Hardware wiring readiness to Yellow: pins are assigned, but sonar supply/ECHO and motor safe-off are not verified.
Updated readiness position: partly ready. Ownership and interfaces are defined, and the curveball's impact is understood, but nothing is integrated yet. Before Week 10 we need the STOP and safe-off path working on hardware, the sonar electrical check, the field table and MQTT wire spec frozen, and the first build stage (motion, line following, STOP/fault, telemetry) running.
B5 and B4 will check ECHO voltage on a scope, take a wiring photo and check the motor outputs across reset.
 | R2 (sonar supply/ECHO) is still open. Version B's readiness statement was left blank.
 | The risk ratings are undifferentiated. All eight risks are Medium likelihood and High severity, and several owners are “All”, which suggests the risks weren’t rated individually. There’s also no hardware-dependency risk for the fact that encoder field widths depend on unmeasured counts per revolution (A2)
 | Partly accept. Equal ratings are not wrong in themselves, but we did not justify each one, and “All” is not a usable owner.
 | B5 will re-rate each risk with a one-line reason, with input from all buddies. We propose these lead owners: R1 B5, R2 B5, R3 B1, R4 B4, R5 B5, R6 B2, R7 B5, R8 B4.
We added R9: packed encoder field widths depend on unmeasured counts per revolution (A2), with B2 as lead.
 | Version B lists R1–R8 all as Medium/High. Proposed lead owners follow the Week 6 subsystem ownership.
 | Pre-existing inconsistency (lower priority). Week 6’s A10 passes with “no heartbeat gap > 5 s”, but NFR5b shows the robot offline after 3 missed heartbeats (about 3 s). The bench measured a worst laptop-side gap of 2.8 s, which passes A10 but nearly trips NFR5b. Version B doesn’t address this.
 | Accept the clarification; defer closure until tested. A10 and NFR5b test different things: A10 is WiFi coverage around the course (no gap over 5 s), and NFR5b is when the laptop shows the robot offline (3 missed 1 s heartbeats).
 | B1 will keep both criteria and state which one each test uses. B1 will test against the existing 3-missed-heartbeat rule first, comparing laptop receive gaps with CYW43 WiFi power-save on and off, then measure under integrated load and during a deliberate outage. Any looser offline rule would be a proposed requirement change needing approval, not a fix.
 | Buddy 1 bench results, 18 Oct: robot-side heartbeat send jitter ≤4 ms, but on the laptop 97.6% of 493 intervals were within 1 s ±100 ms and the worst gap was 2,816 ms. Firmware send timing is stable while laptop receive intervals have outliers; the source of the delay is unconfirmed, and WiFi power-save is one hypothesis. 2.8 s passes A10 but nearly trips the 3 s offline rule.
 | 5. Final Tribunal Summary
Final question
 | Team answer
 | What is the strongest part of the current design?
 | The safety path, as designed (not yet implemented or measured). Stop causes are latched before any message is sent, Safety can force the motors off without waiting for Motion, and Mission is the only motion-command writer, with epoch/ID checks against stale results. During the curveball discussion the team kept these rules rather than trading them for bit savings.
 | What is most likely to fail?
 | Integration of shared state and interfaces under the minimum-bit change: fault/stop words with several writers, the encoder interface between Buddy 2 and Buddy 3, and the MQTT binary format between firmware and laptop. Hardware quantities that every packed width depends on (encoder counts per revolution, sonar behaviour) are also still unmeasured.
 | What evidence supports readiness?
 | Limited so far. Most evidence is design: the Week 6 report and the decisions recorded in Version B. The only measured results are Buddy 1's standalone WiFi/MQTT bench tests on 18 Oct, with no RTOS. They show connect times within budget, every command rejection case tested on the bench behaving correctly, robot-side heartbeat timing within ±100 ms with polling every 95 ms but not from 245 ms (laptop receive timing had outliers up to 2.8 s), and 2 broker-loss recoveries in under 3 s. Everything else in the test plan is planned, not passed.
 | What did AI miss that human discussion revealed?
 | The discussion revealed that treating every bit in the shared fault mask as a stop cause would conflict with our Week 6 rule that WiFi loss must not stop the robot. It also found that packing ISR and task data into the same word creates lost-update races (Buddy 2), and that a missing echo must not count as a clear path once data is packed (Buddy 5). Earlier, in Version A, buddies rejected AI suggestions that did not fit our hardware or design: a gyro-based filter on a gyro-less IMU, pinging the HC-SR04 faster than its 60 ms cycle, and raising Mission's priority when stopping never waits for Mission.
 | What did AI correctly identify after discussion?
 | That the ir_edge_t memory saving was wrong; that MQTT STOP must not depend on queue space; that the binary wire format is undefined; that “Hardware wiring: Green” contradicts open risk R2; that several Zoom decisions were missing from the change log; and that some field widths did not match their ranges.
 |
```
