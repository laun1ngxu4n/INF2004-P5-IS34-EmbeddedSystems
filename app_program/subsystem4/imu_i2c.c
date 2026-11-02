/*
 *	imu_i2c.c
 *	Subsystem 4 -- polled I2C1 master for the IMU (GP26 SDA, GP27 SCL).
 *
 *	Only imu_task uses I2C1 (TEAM.md section 3).  The kernel keeps I2C1 in
 *	reset, so it is released here.  Timing values for 400 kHz at 125 MHz
 *	follow the Pico SDK's i2c_set_baudrate().
 */

#include "subsystem4.h"

#define I2C1_BASE		0x40048000
#define IC_CON			(I2C1_BASE + 0x00)
#define IC_TAR			(I2C1_BASE + 0x04)
#define IC_DATA_CMD		(I2C1_BASE + 0x10)
#define IC_FS_SCL_HCNT		(I2C1_BASE + 0x1C)
#define IC_FS_SCL_LCNT		(I2C1_BASE + 0x20)
#define IC_RAW_INTR_STAT	(I2C1_BASE + 0x34)
#define IC_CLR_TX_ABRT		(I2C1_BASE + 0x54)
#define IC_ENABLE		(I2C1_BASE + 0x6C)
#define IC_STATUS		(I2C1_BASE + 0x70)
#define IC_RXFLR		(I2C1_BASE + 0x78)
#define IC_SDA_HOLD		(I2C1_BASE + 0x7C)
#define IC_FS_SPKLEN		(I2C1_BASE + 0xA0)

#define CMD_READ		0x100u
#define CMD_STOP		0x200u
#define CMD_RESTART		0x400u
#define RAW_TX_ABRT		0x040u
#define STATUS_TFE		0x004u
#define STATUS_ACTIVITY		0x001u
#define TIMEOUT_US		3000u

#define PIN_SDA			26
#define PIN_SCL			27

LOCAL BOOL aborted(void)
{
	if ((in_w(IC_RAW_INTR_STAT) & RAW_TX_ABRT) != 0u) {
		(void)in_w(IC_CLR_TX_ABRT);
		return TRUE;
	}
	return FALSE;
}

LOCAL void set_target(UB addr)
{
	out_w(IC_ENABLE, 0);
	out_w(IC_TAR, addr);
	out_w(IC_ENABLE, 1);
}

EXPORT void imu_i2c_init(void)
{
	UINT pin;

	app_hw_release(RESETS_RESET_I2C1);
	for (pin = PIN_SDA; pin <= PIN_SCL; pin++) {
		clr_w(GPIO(pin), GPIO_OD | GPIO_PDE);
		set_w(GPIO(pin), GPIO_IE | GPIO_PUE);
		out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_I2C);
	}
	out_w(IC_ENABLE, 0);
	/* master, fast mode, restart on, slave off, TX_EMPTY_CTRL */
	out_w(IC_CON, 0x165);
	out_w(IC_FS_SCL_HCNT, 126);
	out_w(IC_FS_SCL_LCNT, 187);
	out_w(IC_FS_SPKLEN, 11);
	out_w(IC_SDA_HOLD, 38);
	out_w(IC_ENABLE, 1);
}

/* Read n (<= 8) registers from reg up (LSM303D: bit 7 = auto-increment) */
EXPORT ER imu_i2c_read(UB addr, UB reg, UB *buf, INT n)
{
	UW	t0;
	INT	i;

	set_target(addr);
	(void)aborted();
	out_w(IC_DATA_CMD, (UW)(reg | ((n > 1) ? 0x80u : 0u)));
	for (i = 0; i < n; i++) {
		out_w(IC_DATA_CMD, CMD_READ | ((i == 0) ? CMD_RESTART : 0u) |
			((i == n - 1) ? CMD_STOP : 0u));
	}
	t0 = app_time_us();
	for (i = 0; i < n; i++) {
		while (in_w(IC_RXFLR) == 0u) {
			if (aborted()) return E_IO;
			if (app_time_us() - t0 > TIMEOUT_US) return E_TMOUT;
		}
		buf[i] = (UB)(in_w(IC_DATA_CMD) & 0xFFu);
	}
	return E_OK;
}

EXPORT ER imu_i2c_write(UB addr, UB reg, UB val)
{
	UW t0;

	set_target(addr);
	(void)aborted();
	out_w(IC_DATA_CMD, reg);
	out_w(IC_DATA_CMD, (UW)val | CMD_STOP);
	t0 = app_time_us();
	while ((in_w(IC_STATUS) & STATUS_TFE) == 0u ||
	       (in_w(IC_STATUS) & STATUS_ACTIVITY) != 0u) {
		if (aborted()) return E_IO;
		if (app_time_us() - t0 > TIMEOUT_US) return E_TMOUT;
	}
	return aborted() ? E_IO : E_OK;
}
