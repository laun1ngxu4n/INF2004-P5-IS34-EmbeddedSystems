# IMU requirement change (lecturer, 13 Nov)

The lecturer's answer to our question about the conflict between
collision avoidance (the ultrasonic bypass) and IMU collision detection,
word for word:

> After reviewing the requirement, I agree that there is a conflict between
> collision avoidance and the IMU collision detection requirement.
>
> I will therefore simplify the requirement. Collision detection using the
> IMU will be removed. Instead, the IMU should be used to detect when the
> robot has been lifted or overturned. When such a condition is detected,
> the motors should immediately stop. During the demonstration, lifting the
> robot will be used to signal the end of the run. The robot should
> therefore detect this condition and stop all motor movement.
>
> Please update your implementation and documentation accordingly.

Clarification given with it: this is a simplification, not an extra IMU
feature. Collision detection using the IMU is removed. The only
replacement is that lifting the robot at the end of the demonstration
stops the motors. No lift / overturn classification is needed; it is a
small extension of the existing IMU work (Buddy 4 scope).

## What we changed (13 Nov)

- **IMU collision detection:** removed from the requirements (FR9, FR13).
  It had never been built (`docs/team/week6_report_changes.md`), so no code
  was taken out.
- **Lift / tip-over stop (built 13 Nov):** `app_program/subsystem4/imu.c`.
  Every 10 ms the IMU task compares the direction of gravity with the one
  measured at the last level calibration (at START in the Mission). If it
  is more than 35 deg away for 100 ms, the motors go off at once as if STOP
  were pressed, Mission is told, and the panel gets the event "robot lifted
  or tipped over (tilt over 35 deg): motors off, START to go again". START
  clears it, as after a STOP; while the robot is still tilted, a START is
  stopped again.
  - Why 35 deg: the largest tilt seen while driving was about 19 deg (the
    25 mm hump, 9 Nov). A robot picked up by hand tilts more than 35.
    Upside down is 180 deg, so overturned is caught too (pitch and side
    tilt alone would read about 0 there).
  - Limit: a lift that keeps the robot within 35 deg of level all the
    time is not seen (an accelerometer feels gravity, not height). Pick it
    up the usual way (tilted), as at the end of the demo.
  - It uses the existing STOP reason, so the binary status message and the
    panel are unchanged (Change Requirement 1 format kept).
- **Test:** S4-L01 in the Stage 4 image (3 tries: it drives slowly, the
  tester picks it up; pass = every lift stops the motors). In the Mission,
  lifting at the end of each run tests it too.
- **The hump height (FR4) stays** as before.
