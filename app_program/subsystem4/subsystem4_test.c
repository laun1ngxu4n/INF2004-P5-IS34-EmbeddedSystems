/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem4/logs/ and record the result
 *	in testing/subsystem4/TEST_LOG.md.  Test IDs: S4-U01, S4-U02, ...
 */

#include "subsystem4.h"
#include "../test_report.h"
#include "../app_stage_test.h"
#include "../subsystem2/subsystem2.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
	TEST_INIT();

	/* S4-U01 / S4-U02: Safety wakes every 20 ms without overruns */
	TEST_PERIODIC("S4-U01", "S4-U02", tmg_safety);

	/* S4-U03 / S4-U04: IMU wakes every 10 ms without overruns */
	TEST_PERIODIC("S4-U03", "S4-U04", tmg_imu);

	TEST_SUMMARY("subsystem4");
}

/* ---- Stage 4 hands-on tests ---------------------------------------- */

EXPORT void subsystem4_stage_test(void)
{
	INT	k, t;
	TEST_INIT();

	tm_printf((UB*)"\n[DRIVE] === Stage 4: IMU ramp height ===\n");
	for (t = 0; t < 30 && !imu_ok(); t++) tk_dly_tsk(100);
	TEST_CHECK("S4-I01", imu_ok(), "LSM303D found at 0x1D");
	if (!imu_ok()) {
		TEST_SUMMARY("subsystem4 stage 4");
		return;
	}
	test_wait_start("Robot level and still: calibrating for 1 s.");
	imu_calibrate();
	while (!imu_calibrated()) tk_dly_tsk(10);
	tm_printf((UB*)"[S4] tilt view for 10 s: lift the front a little (sin x1000)\n");
	for (t = 0; t < 20; t++) {
		tm_printf((UB*)"[S4] sin(pitch) x1000 = %d\n", imu_pitch_mdeg_sin());
		tk_dly_tsk(500);
	}

	for (k = 1; k <= 10; k++) {
		UW n0 = imu_ramp_reports();

		tm_printf((UB*)"\n[DRIVE] S4-R01 run %d of 10\n", k);
		test_wait_start("Robot level about 30 cm before the ramp, pointing "
				"straight at it. It calibrates 1 s, then drives 1.5 m.");
		imu_calibrate();
		while (!imu_calibrated()) tk_dly_tsk(10);
		{
			W odo0 = motion_odometer();
			UW j0 = imu_jolts();
			INT hmax = 0, smax = 0;

			motion_drive(S2_SPEED_BARCODE, 1500);
			for (t = 0; t < 1500; t++) {
				MOTION_STATUS st;
				motion_get_status(&st);
				if (st.result != MOT_RES_BUSY) break;
				if (imu_height_mm10() > hmax) hmax = imu_height_mm10();
				{
					INT s = imu_pitch_mdeg_sin();
					if (s < 0) s = -s;
					if (s > smax) smax = s;
				}
				/* reported (level again on top): stop here, so the
				   robot does not drive off the end of the ramp */
				if (imu_ramp_reports() != n0) {
					motion_halt();
					break;
				}
				/* what the IMU sees on the way: a climb shows as
				   sin > 30 and a rising height */
				if (t % 50 == 0) {
					tm_printf((UB*)"[S4] %d mm driven, sin(pitch) x1000 %d, ",
						  (INT)S2_COUNTS_TO_MM(motion_odometer() - odo0),
						  imu_pitch_mdeg_sin());
					test_print_x10("height ", imu_height_mm10(), " mm\n");
				}
				tk_dly_tsk(10);
			}
			/* also when STOP came before it was level again */
			test_print_x10("[S4-R01] highest climbed this run ", hmax, " mm\n");
			/* short hump: the robot tips over it, height = L x sin */
			tm_printf((UB*)"[S4-R01] peak |sin(pitch)| x1000 %d, x L %d mm = ",
				  smax, S4_L_MM);
			test_print_x10("", smax * S4_L_MM / 100, " mm (L not measured yet)\n");
			tm_printf((UB*)"[S4-R01] %d samples skipped as jolts (10 ms each)\n",
				  (INT)(imu_jolts() - j0));
		}
		tk_dly_tsk(500);
		if (imu_ramp_reports() != n0) {
			test_print_x10("[S4-R01] reported ramp height ", imu_ramp_peak_mm10(),
				       " mm (write down the measured ramp height)\n");
		} else {
			tm_printf((UB*)"[S4-R01] no ramp reported this run\n");
		}
		if (test_stop_pressed()) {
			tm_printf((UB*)"[S4-R01] STOP pressed: runs ended\n");
			break;
		}
	}
	TEST_SUMMARY("subsystem4 stage 4");
}

#endif /* APP_TEST */
