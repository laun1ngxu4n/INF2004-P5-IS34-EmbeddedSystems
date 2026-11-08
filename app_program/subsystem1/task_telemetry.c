/*
 *	task_telemetry.c
 *	Subsystem 1 -- the Telemetry task (Notification merged in), the event
 *	outbox and the MQTT command inbox.
 *
 *	Every 100 ms: IF-01 ping once a second, the status line once a second,
 *	the live sensor line twice a second, the timing table every 10 s.
 *	Events from every subsystem go into a small outbox; the WiFi task
 *	pulls them with app_mqtt_tx() and hands
 *	commands to app_mqtt_rx().  Both hooks only copy under APP_LOCK and
 *	never wait, because they run in the kernel's WiFi task.
 *
 *	MQTT images also send a copy of the whole console on is34/log, so a
 *	test can run on the battery with no USB cable: the panel shows it and
 *	saves it (tools/panel/is34_panel.py, "Save log").  Each message is
 *	"L<seq> <dropped>|<text>": seq counts messages (a gap = lost on the
 *	WiFi), dropped counts bytes the robot could not keep (buffer full).
 */

#include "subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

#define OUTBOX_N	8

typedef struct {
	UB	topic;			/* 0 event, 1 telemetry, 2 ack/map, 3 sensors */
	UB	text[TELE_TEXT_MAX];
} OUTMSG;

LOCAL OUTMSG		outbox[OUTBOX_N];
LOCAL UW		ob_w;
LOCAL UW ob_r;	/* under APP_LOCK */
LOCAL UW		out_dropped;
LOCAL const char	*state_text = "IDLE";
LOCAL volatile UW	start_count;	/* MQTT START commands since boot */

EXPORT UW tele_start_count(void)	{ return start_count; }

#if defined(TM_MQTT) && TM_MQTT
/* Copy of the console for is34/log.  8 KB holds the start-up and Stage 0
   output until the WiFi is up (~4 KB), and a few seconds of the busiest
   test output; the link takes about 2.5 KB/s. */
#define LOG_RING	8192		/* a power of 2 */
LOCAL UB		log_ring[LOG_RING];
LOCAL UW		log_w;
LOCAL UW log_r;	/* under APP_LOCK */
LOCAL UW		log_dropped;	/* bytes lost: ring full */
LOCAL UW		log_seq;
LOCAL BOOL		log_turn;	/* last message sent was a log chunk */

/* Called by the console driver for every byte it sends: any context */
EXPORT void tm_console_mirror(const UB *buf, INT size)
{
	UINT	s;
	INT	i;

	APP_LOCK(s);
	for (i = 0; i < size; i++) {
		if ('\r' == buf[i]) continue;		/* the panel wants \n only */
		if ((log_w - log_r) >= LOG_RING) {
			log_dropped++;
		} else {
			log_ring[log_w & (LOG_RING - 1)] = buf[i];
			log_w++;
		}
	}
	APP_UNLOCK(s);
}

/* The next log chunk into buf; 0 = nothing waiting */
LOCAL int log_take(char *topic, unsigned tmax, unsigned char *buf, unsigned bmax)
{
	UB		head[24];
	UINT		s;
	unsigned	i;
	unsigned n;
	unsigned h;
	UW		avail;

	APP_LOCK(s);
	avail = log_w - log_r;
	APP_UNLOCK(s);
	if ((0u == avail) || (bmax < 64u)) return 0;

	h = (unsigned)tm_sprintf(head, (const UB*)"L%d %d|", (INT)(log_seq + 1u),
				 (INT)log_dropped);
	for (i = 0; i < h; i++) buf[i] = head[i];
	n = bmax - 1u - h;
	if (n > avail) n = (unsigned)avail;
	APP_LOCK(s);
	for (i = 0; i < n; i++) {
		buf[h + i] = log_ring[(log_r + i) & (LOG_RING - 1)];
	}
	log_r += n;
	APP_UNLOCK(s);
	log_seq++;

	for (i = 0; ((i + 1) < tmax) && (TOPIC_LOG[i] != '\0'); i++) topic[i] = TOPIC_LOG[i];
	topic[i] = '\0';
	return (int)(h + n);
}
#endif

LOCAL void outbox_put(UB topic, const UB *text)
{
	UINT	s;
	INT	i;

	APP_LOCK(s);
	if ((ob_w - ob_r) >= OUTBOX_N) {
		ob_r++;			/* full: drop the oldest */
		out_dropped++;
	}
	outbox[ob_w % OUTBOX_N].topic = topic;
	for (i = 0; (i < (TELE_TEXT_MAX - 1)) && (text[i] != '\0'); i++) {
		outbox[ob_w % OUTBOX_N].text[i] = text[i];
	}
	outbox[ob_w % OUTBOX_N].text[i] = '\0';
	ob_w++;
	APP_UNLOCK(s);
}

EXPORT void tele_event(const UB *text)
{
	tm_printf((UB*)"[EVT] %s\n", text);
	outbox_put(0, text);
}

EXPORT void tele_set_state(const char *state)
{
	state_text = state;
}

/* ---- MQTT hooks, called from the WiFi task (mqtt_utk.c) ------------- */

/* TRUE if s starts with prefix; *rest = what follows */
LOCAL BOOL starts_with(const char *s, const char *prefix, const char **rest)
{
	while (*prefix != '\0') {
		if (*s++ != *prefix++) return FALSE;
	}
	*rest = s;
	return TRUE;
}

LOCAL BOOL same(const char *a, const char *b)
{
	while ((*a != '\0') && (*a == *b)) {
		a++;
		b++;
	}
	return *a == *b;
}

EXPORT int app_mqtt_tx(char *topic, unsigned tmax, unsigned char *buf, unsigned bmax)
{
	static const char	*names[4] = {TOPIC_EVENT, TOPIC_TELEMETRY, TOPIC_ACK_MAP,
					  TOPIC_SENSORS};
	OUTMSG			m;
	UINT			s;
	unsigned		i;
	BOOL			have;

	APP_LOCK(s);
	have = (ob_r != ob_w);
	APP_UNLOCK(s);
#if defined(TM_MQTT) && TM_MQTT
	/* Outbox and log take turns, so a long log never holds up an event */
	if ((!have) || (!log_turn)) {
		int n = log_take(topic, tmax, buf, bmax);
		if (n > 0) {
			log_turn = TRUE;
			return n;
		}
	}
	log_turn = FALSE;
#endif
	APP_LOCK(s);
	if (ob_r == ob_w) {
		APP_UNLOCK(s);
		return 0;
	}
	m = outbox[ob_r % OUTBOX_N];
	ob_r++;
	APP_UNLOCK(s);

	for (i = 0; ((i + 1) < tmax) && (names[m.topic][i] != '\0'); i++) {
		topic[i] = names[m.topic][i];
	}
	topic[i] = '\0';
	for (i = 0; ((i + 1) < bmax) && (m.text[i] != '\0'); i++) {
		buf[i] = m.text[i];
	}
	return (int)i;
}

EXPORT void app_mqtt_rx(const char *topic, const unsigned char *data, unsigned len)
{
	const char	*cmd;
	UB		why[64];
	UB		msg[TELE_TEXT_MAX];

	if (!starts_with(topic, TOPIC_CMD_PREFIX, &cmd)) {
		return;
	}
	if (same(cmd, "map")) {
		if (barcode_set_mapping(data, (INT)len, why, sizeof(why))) {
			UB map[24];
			barcode_mapping_text(map, sizeof(map));
			tm_sprintf(msg, (const UB*)"OK %s", map);
		} else {
			tm_sprintf(msg, (const UB*)"REJECTED %s (old mapping kept)", why);
		}
		tm_printf((UB*)"[MQTT] map: %s\n", msg);
		outbox_put(2, msg);
	} else if (same(cmd, "start")) {
		tm_printf((UB*)"[MQTT] start\n");
		start_count++;
		tk_set_flg(flg_mission, MSN_FLG_START);
	} else if (same(cmd, "stop")) {
		app_stop_request(STOP_MQTT);
		tk_set_flg(flg_mission, MSN_FLG_STOP);
		tm_printf((UB*)"[MQTT] stop\n");
	}
}

/* ---- the task -------------------------------------------------------- */

LOCAL void status_line(void)
{
	MOTION_STATUS	st;
	BARCODE_RESULT	bc;
	UB		text[TELE_TEXT_MAX];
	UB		map[24];

	motion_get_status(&st);
	barcode_last(&bc);
	barcode_mapping_text(map, sizeof(map));
	tm_sprintf(text, (const UB*)
		"state=%s speed=%d,%d dist=%d line=%d junctions=%d "
		"barcode=%c map=%s ramp_peak=%d.%d stop=%d",
		state_text, st.speed_l, st.speed_r,
		(INT)S2_COUNTS_TO_MM(motion_odometer()), line_state(),
		(INT)line_junctions(), (bc.ok ? bc.ch : '-'),
		(map[0] != '\0') ? (char*)map : "none",
		imu_ramp_peak_mm10() / 10, imu_ramp_peak_mm10() % 10,
		(INT)app_stop_reasons());
	outbox_put(1, text);
}

/* Live sensor readings for the laptop panel (not graded, just to see what
   the robot sees): ir = side, centre, barcode (1 = black), steer mm/s,
   hold = steering held straight (B over bars), inbar = inside a barcode, front = last
   forward sonar mm (-1 open, -2 no ECHO), tilt = sin(pitch) x1000,
   h10 = climbed height in 0.1 mm, imu = IMU answering */
LOCAL void sensor_line(void)
{
	UB	text[TELE_TEXT_MAX];

	tm_sprintf(text, (const UB*)
		"ir=%d%d%d steer=%d hold=%d inbar=%d front=%d tilt=%d h10=%d imu=%d",
		(INT)(IR_BLACK == app_gpio_get(PIN_IR_LINE_L)),
		(INT)(IR_BLACK == app_gpio_get(PIN_IR_LINE_R)),
		(INT)barcode_black(),
		line_steer_mm_s(), (INT)barcode_in_bars(), (INT)barcode_busy(),
		obstacle_front_mm(), imu_pitch_mdeg_sin(), imu_height_mm10(),
		(INT)imu_ok());
	outbox_put(3, text);
}

EXPORT void telemetry_task(INT stacd, void *exinf)
{
	UINT	report_tick = 0;
	UINT	ping_tick = 0;
	UINT	status_tick = 0;
	UINT	sensor_tick = 0;

	(void)stacd; (void)exinf;

	for (;;) {
		tk_slp_tsk(TMO_FEVR);			/* woken every 100 ms */
		app_timing_wake(tmg_telemetry, tk_can_wup(TSK_SELF));

		if (++ping_tick >= S1_PING_TICKS) {
			ping_tick = 0;
			CHECK(tk_set_flg(flg_mission, MSN_FLG_PING), "IF-01 ping");
		}
		if (++status_tick >= S1_STATUS_TICKS) {
			status_tick = 0;
			status_line();
		}
		if (++sensor_tick >= S1_SENSOR_TICKS) {
			sensor_tick = 0;
			sensor_line();
		}
		if (++report_tick >= S1_REPORT_TICKS) {
			report_tick = 0;
			/* The report's printing counts in this task's run time on
			   purpose: it is real Telemetry load at the lowest priority. */
			app_timing_report();
		}

		app_timing_done(tmg_telemetry);
	}
}
