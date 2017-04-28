/*
 * PSX (Play Station 1/2) pad (SPI Interface)
 *
 * Copyright (C) 2017 AZO <typesylph@gmail.com>
 * Licensed under the GPL-2 or later.
 *
 * PSX pad plug (not socket)
 *  123 456 789
 * (...|...|...)
 *
 * 1: DAT -> MISO (pullup with 1k owm to 3.3V)
 * 2: CMD -> MOSI
 * 3: 9V (for motor, if not use N.C.)
 * 4: GND
 * 5: 3.3V
 * 6: Attention -> CS(SS)
 * 7: SCK -> SCK
 * 8: N.C.
 * 9: ACK -> N.C.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>

//#define PSXPAD_ENABLE_ANALOG2

#define PSXPAD_DEFAULT_SPI_DELAY	100
#define PSXPAD_DEFAULT_INTERVAL		16
#define PSXPAD_DEFAULT_INTERVAL_MIN	8
#define PSXPAD_DEFAULT_INTERVAL_MAX	32
#define PSXPAD_DEFAULT_INPUT_PHYSIZE	32

#define REVERSE_BIT(x) ((((x) & 0x80) >> 7) | (((x) & 0x40) >> 5) | (((x) & 0x20) >> 3) | (((x) & 0x10) >> 1) | (((x) & 0x08) << 1) | (((x) & 0x04) << 3) | (((x) & 0x02) << 5) | (((x) & 0x01) << 7))

enum {
	PSXPAD_KEYSTATE_TYPE_DIGITAL = 0,
	PSXPAD_KEYSTATE_TYPE_ANALOG1,
	PSXPAD_KEYSTATE_TYPE_ANALOG2,
	PSXPAD_KEYSTATE_TYPE_UNKNOWN
};

static const u8 PSX_CMD_INIT_PRESSURE[]	= {0x01, 0x40, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
static const u8 PSX_CMD_ALL_PRESSURE[]	= {0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00};
static const u8 PSX_CMD_POLL[]		= {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const u8 PSX_CMD_ENTER_CFG[]	= {0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
static const u8 PSX_CMD_EXIT_CFG[]	= {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
static const u8 PSX_CMD_ENABLE_MOTOR[]	= {0x01, 0x4D, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const u8 PSX_CMD_AD_MODE[]	= {0x01, 0x44, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};

struct psxpad {
	struct spi_device *spi;
	struct input_polled_dev *pdev;
	struct input_dev *idev;
	char phys[PSXPAD_DEFAULT_INPUT_PHYSIZE];
	u16 spi_delay;
	bool analog_mode;
	bool mode_lock;
	bool motor1enable;
	bool motor2enable;
	u8 motor1level;
	u8 motor2level;

	/* for suspend/resume */
	bool sus_analog_mode;
	bool sus_mode_lock;
	bool sus_motor1enable;
	bool sus_motor2enable;
	u8 sus_motor1level;
	u8 sus_motor2level;

	u8 spi_speed;
	u8 poolcmd[sizeof(PSX_CMD_POLL)];
	u8 enablemotor[sizeof(PSX_CMD_ENABLE_MOTOR)];
	u8 admode[sizeof(PSX_CMD_AD_MODE)];
	u8 sendbuf[0x20] ____cacheline_aligned;
	u8 response[sizeof(PSX_CMD_POLL)] ____cacheline_aligned;
};

static int psxpad_command(struct psxpad *pad, const u8 sendcmd[], u8 response[], const u8 sendcmdlen)
{
	struct spi_transfer xfers = {
		.tx_buf		= pad->sendbuf,
		.rx_buf		= response,
		.len		= sendcmdlen,
		.delay_usecs	= pad->spi_delay,
	};
	struct spi_message msg;
	int err;
	u8 loc;

	spi_message_init(&msg);

	for (loc = 0; loc < sendcmdlen; loc++)
		pad->sendbuf[loc] = REVERSE_BIT(sendcmd[loc]);

	err = spi_sync_transfer(pad->spi, &xfers, 1);
	if (err) {
		dev_err(&pad->idev->dev, "psxpad-spi: failed SPI xfers!!\n");
		goto err_end;
	}

	for (loc = 0; loc < sendcmdlen; loc++)
		response[loc] = REVERSE_BIT(response[loc]);

	return 0;

 err_end:

	return err;
}

static void psxpad_setadmode(struct psxpad *pad, const bool analog_mode, const bool mode_lock)
{
	int err;

	pad->analog_mode	= analog_mode;
	pad->mode_lock	= mode_lock;

	pad->admode[3] = pad->analog_mode	? 0x01 : 0x00;
	pad->admode[4] = pad->mode_lock		? 0x03 : 0x00;

	err = psxpad_command(pad, PSX_CMD_ENTER_CFG, pad->response, sizeof(PSX_CMD_ENTER_CFG));
	if (err) {
		dev_err(&pad->idev->dev, "psxpad-spi: setadmode: enter cfg failed!!\n");
		return;
	}
	err = psxpad_command(pad, pad->admode, pad->response, sizeof(PSX_CMD_AD_MODE));
	if (err) {
		dev_err(&pad->idev->dev, "psxpad-spi: setadmode: set admode failed!!\n");
		return;
	}
#ifdef PSXPAD_ENABLE_ANALOG2
	err = psxpad_command(pad, PSX_CMD_INIT_PRESSURE, pad->response, sizeof(PSX_CMD_INIT_PRESSURE));
	if (err) {
		dev_err(&pad->idev->dev, "psxpad-spi: setadmode: init pressure failed!!\n");
		return;
	}
	err = psxpad_command(pad, PSX_CMD_ALL_PRESSURE, pad->response, sizeof(PSX_CMD_ALL_PRESSURE));
	if (err) {
		dev_err(&pad->idev->dev, "psxpad-spi: setadmode: set all pressure failed!!\n");
		return;
	}
#endif	/* PSXPAD_ENABLE_ANALOG2 */
	err = psxpad_command(pad, PSX_CMD_EXIT_CFG, pad->response, sizeof(PSX_CMD_EXIT_CFG));
	if (err) {
		dev_err(&pad->idev->dev, "psxpad-spi: setadmode: exit config failed!!\n");
		return;
	}
}

#ifdef CONFIG_JOYSTICK_PSXPAD_SPI_FF
static void psxpad_setenablemotor(struct psxpad *pad, const bool motor1enable, const bool motor2enable)
{
	int err;

	pad->motor1enable = motor1enable;
	pad->motor2enable = motor2enable;

	pad->enablemotor[3] = pad->motor1enable ? 0x00 : 0xFF;
	pad->enablemotor[4] = pad->motor2enable ? 0x01 : 0xFF;

	err = psxpad_command(pad, PSX_CMD_ENTER_CFG, pad->response, sizeof(PSX_CMD_ENTER_CFG));
	if (err) {
		dev_err(&pad->idev->dev, "psxpad-spi: setenablemotor: enter cfg failed!!\n");
		return;
	}
	err = psxpad_command(pad, pad->enablemotor, pad->response, sizeof(PSX_CMD_ENABLE_MOTOR));
	if (err) {
		dev_err(&pad->idev->dev, "psxpad-spi: setenablemotor: enable motor failed!!\n");
		return;
	}
	err = psxpad_command(pad, PSX_CMD_EXIT_CFG, pad->response, sizeof(PSX_CMD_EXIT_CFG));
	if (err) {
		dev_err(&pad->idev->dev, "psxpad-spi: setenablemotor: exit config failed!!\n");
		return;
	}
}

static void psxpad_setmotorlevel(struct psxpad *pad, const u8 motor1level, const u8 motor2level)
{
	pad->motor1level = motor1level ? 0xFF : 0x00;
	pad->motor2level = motor2level;

	pad->poolcmd[3] = pad->motor1level;
	pad->poolcmd[4] = pad->motor2level;
}
#else	/* CONFIG_JOYSTICK_PSXPAD_SPI_FF */
static void psxpad_setenablemotor(struct psxpad *pad, const bool motor1enable, const bool motor2enable) { }

static void psxpad_setmotorlevel(struct psxpad *pad, const u8 motor1level, const u8 motor2level) { }
#endif	/* CONFIG_JOYSTICK_PSXPAD_SPI_FF */

static void psxpad_spi_poll_open(struct input_polled_dev *pdev)
{
	struct psxpad *pad = pdev->private;

	pm_runtime_get_sync(&pad->spi->dev);
}

static void psxpad_spi_poll_close(struct input_polled_dev *pdev)
{
	struct psxpad *pad = pdev->private;

	pm_runtime_put_sync(&pad->spi->dev);
}

static void psxpad_spi_poll(struct input_polled_dev *pdev)
{
	struct psxpad *pad = pdev->private;
	int err;

	err = psxpad_command(pad, pad->poolcmd, pad->response, sizeof(PSX_CMD_POLL));
	if (err) {
		dev_err(&pad->idev->dev, "psxpad-spi: poll: poll cmd failed!!\n");
		return;
	}

	switch (pad->response[1]) {
#ifdef PSXPAD_ENABLE_ANALOG2
	case 0x79:	/* analog 2 */
		input_report_abs(pad->idev, ABS_HAT0Y,		pad->response[11]);
		input_report_abs(pad->idev, ABS_HAT1Y,		pad->response[12]);
		input_report_abs(pad->idev, ABS_HAT0X,		pad->response[10]);
		input_report_abs(pad->idev, ABS_HAT1X,		pad->response[9]);
		input_report_abs(pad->idev, ABS_MISC,		pad->response[13]);
		input_report_abs(pad->idev, ABS_PRESSURE,	pad->response[14]);
		input_report_abs(pad->idev, ABS_BRAKE,		pad->response[15]);
		input_report_abs(pad->idev, ABS_THROTTLE,	pad->response[16]);
		input_report_abs(pad->idev, ABS_HAT2X,		pad->response[17]);
		input_report_abs(pad->idev, ABS_HAT3X,		pad->response[18]);
		input_report_abs(pad->idev, ABS_HAT2Y,		pad->response[19]);
		input_report_abs(pad->idev, ABS_HAT3Y,		pad->response[20]);
		input_report_abs(pad->idev, ABS_X,		pad->response[7]);
		input_report_abs(pad->idev, ABS_Y,		pad->response[8]);
		input_report_abs(pad->idev, ABS_RX,		pad->response[5]);
		input_report_abs(pad->idev, ABS_RY,		pad->response[6]);
		input_report_key(pad->idev, BTN_DPAD_UP,	false);
		input_report_key(pad->idev, BTN_DPAD_DOWN,	false);
		input_report_key(pad->idev, BTN_DPAD_LEFT,	false);
		input_report_key(pad->idev, BTN_DPAD_RIGHT,	false);
		input_report_key(pad->idev, BTN_X,		false);
		input_report_key(pad->idev, BTN_A,		false);
		input_report_key(pad->idev, BTN_B,		false);
		input_report_key(pad->idev, BTN_Y,		false);
		input_report_key(pad->idev, BTN_TL,		false);
		input_report_key(pad->idev, BTN_TR,		false);
		input_report_key(pad->idev, BTN_TL2,		false);
		input_report_key(pad->idev, BTN_TR2,		false);
		input_report_key(pad->idev, BTN_THUMBL,		(pad->response[3] & 0x02U) ? false : true);
		input_report_key(pad->idev, BTN_THUMBR,		(pad->response[3] & 0x04U) ? false : true);
		input_report_key(pad->idev, BTN_SELECT,		(pad->response[3] & 0x01U) ? false : true);
		input_report_key(pad->idev, BTN_START,		(pad->response[3] & 0x08U) ? false : true);
		break;
#endif	/* PSXPAD_ENABLE_ANALOG2 */

	case 0x73:	/* analog 1 */
#ifdef PSXPAD_ENABLE_ANALOG2
		input_report_abs(pad->idev, ABS_HAT0Y,		0);
		input_report_abs(pad->idev, ABS_HAT1Y,		0);
		input_report_abs(pad->idev, ABS_HAT0X,		0);
		input_report_abs(pad->idev, ABS_HAT1X,		0);
		input_report_abs(pad->idev, ABS_MISC,		0);
		input_report_abs(pad->idev, ABS_PRESSURE,	0);
		input_report_abs(pad->idev, ABS_BRAKE,		0);
		input_report_abs(pad->idev, ABS_THROTTLE,	0);
		input_report_abs(pad->idev, ABS_HAT2X,		0);
		input_report_abs(pad->idev, ABS_HAT3X,		0);
		input_report_abs(pad->idev, ABS_HAT2Y,		0);
		input_report_abs(pad->idev, ABS_HAT3Y,		0);
#endif	/* PSXPAD_ENABLE_ANALOG2 */
		input_report_abs(pad->idev, ABS_X,		pad->response[7]);
		input_report_abs(pad->idev, ABS_Y,		pad->response[8]);
		input_report_abs(pad->idev, ABS_RX,		pad->response[5]);
		input_report_abs(pad->idev, ABS_RY,		pad->response[6]);
		input_report_key(pad->idev, BTN_DPAD_UP,	(pad->response[3] & 0x10U) ? false : true);
		input_report_key(pad->idev, BTN_DPAD_DOWN,	(pad->response[3] & 0x40U) ? false : true);
		input_report_key(pad->idev, BTN_DPAD_LEFT,	(pad->response[3] & 0x80U) ? false : true);
		input_report_key(pad->idev, BTN_DPAD_RIGHT,	(pad->response[3] & 0x20U) ? false : true);
		input_report_key(pad->idev, BTN_X,		(pad->response[4] & 0x10U) ? false : true);
		input_report_key(pad->idev, BTN_A,		(pad->response[4] & 0x20U) ? false : true);
		input_report_key(pad->idev, BTN_B,		(pad->response[4] & 0x40U) ? false : true);
		input_report_key(pad->idev, BTN_Y,		(pad->response[4] & 0x80U) ? false : true);
		input_report_key(pad->idev, BTN_TL,		(pad->response[4] & 0x04U) ? false : true);
		input_report_key(pad->idev, BTN_TR,		(pad->response[4] & 0x08U) ? false : true);
		input_report_key(pad->idev, BTN_TL2,		(pad->response[4] & 0x01U) ? false : true);
		input_report_key(pad->idev, BTN_TR2,		(pad->response[4] & 0x02U) ? false : true);
		input_report_key(pad->idev, BTN_THUMBL,		(pad->response[3] & 0x02U) ? false : true);
		input_report_key(pad->idev, BTN_THUMBR,		(pad->response[3] & 0x04U) ? false : true);
		input_report_key(pad->idev, BTN_SELECT,		(pad->response[3] & 0x01U) ? false : true);
		input_report_key(pad->idev, BTN_START,		(pad->response[3] & 0x08U) ? false : true);
		break;

	case 0x41:	/* digital */
#ifdef PSXPAD_ENABLE_ANALOG2
		input_report_abs(pad->idev, ABS_HAT0Y,		0);
		input_report_abs(pad->idev, ABS_HAT1Y,		0);
		input_report_abs(pad->idev, ABS_HAT0X,		0);
		input_report_abs(pad->idev, ABS_HAT1X,		0);
		input_report_abs(pad->idev, ABS_MISC,		0);
		input_report_abs(pad->idev, ABS_PRESSURE,	0);
		input_report_abs(pad->idev, ABS_BRAKE,		0);
		input_report_abs(pad->idev, ABS_THROTTLE,	0);
		input_report_abs(pad->idev, ABS_HAT2X,		0);
		input_report_abs(pad->idev, ABS_HAT3X,		0);
		input_report_abs(pad->idev, ABS_HAT2Y,		0);
		input_report_abs(pad->idev, ABS_HAT3Y,		0);
#endif	/* PSXPAD_ENABLE_ANALOG2 */
		input_report_abs(pad->idev, ABS_X,		0x80);
		input_report_abs(pad->idev, ABS_Y,		0x80);
		input_report_abs(pad->idev, ABS_RX,		0x80);
		input_report_abs(pad->idev, ABS_RY,		0x80);
		input_report_key(pad->idev, BTN_DPAD_UP,	(pad->response[3] & 0x10U) ? false : true);
		input_report_key(pad->idev, BTN_DPAD_DOWN,	(pad->response[3] & 0x40U) ? false : true);
		input_report_key(pad->idev, BTN_DPAD_LEFT,	(pad->response[3] & 0x80U) ? false : true);
		input_report_key(pad->idev, BTN_DPAD_RIGHT,	(pad->response[3] & 0x20U) ? false : true);
		input_report_key(pad->idev, BTN_X,		(pad->response[4] & 0x10U) ? false : true);
		input_report_key(pad->idev, BTN_A,		(pad->response[4] & 0x20U) ? false : true);
		input_report_key(pad->idev, BTN_B,		(pad->response[4] & 0x40U) ? false : true);
		input_report_key(pad->idev, BTN_Y,		(pad->response[4] & 0x80U) ? false : true);
		input_report_key(pad->idev, BTN_TL,		(pad->response[4] & 0x04U) ? false : true);
		input_report_key(pad->idev, BTN_TR,		(pad->response[4] & 0x08U) ? false : true);
		input_report_key(pad->idev, BTN_TL2,		(pad->response[4] & 0x01U) ? false : true);
		input_report_key(pad->idev, BTN_TR2,		(pad->response[4] & 0x02U) ? false : true);
		input_report_key(pad->idev, BTN_THUMBL,		false);
		input_report_key(pad->idev, BTN_THUMBR,		false);
		input_report_key(pad->idev, BTN_SELECT,		(pad->response[3] & 0x01U) ? false : true);
		input_report_key(pad->idev, BTN_START,		(pad->response[3] & 0x08U) ? false : true);
		break;
	}

	input_sync(pad->idev);
	psxpad_setenablemotor(pad, true, true);
}

#ifdef CONFIG_JOYSTICK_PSXPAD_SPI_FF
static int psxpad_spi_ff(struct input_dev *idev, void *data, struct ff_effect *effect)
{
	struct psxpad *pad = idev->dev.platform_data;

	switch (effect->type) {
	case FF_RUMBLE:
		psxpad_setmotorlevel(pad, (effect->u.rumble.weak_magnitude >> 8) & 0xFFU, (effect->u.rumble.strong_magnitude >> 8) & 0xFFU);
		break;
	}

	return 0;
}

static int psxpad_spi_init_ff(struct psxpad *pad)
{
	int err;

	input_set_capability(pad->idev, EV_FF, FF_RUMBLE);
	err = input_ff_create_memless(pad->idev, NULL, psxpad_spi_ff);
	if (err) {
		dev_err(&pad->idev->dev, "psxpad-spi: init_ff: ff alloc failed!!\n");
		err = -ENOMEM;
	}

	return err;
}

static void psxpad_spi_deinit_ff(struct psxpad *pad)
{
	input_ff_destroy(pad->idev);
}
#else	/* CONFIG_JOYSTICK_PSXPAD_SPI_FF */
static inline int psxpad_spi_init_ff(struct psxpad *pad)
{
	return 0;
}

static void psxpad_spi_deinit_ff(struct psxpad *pad) { }
#endif	/* CONFIG_JOYSTICK_PSXPAD_SPI_FF */

static int psxpad_spi_probe(struct spi_device *spi)
{
	struct psxpad *pad;
	struct input_polled_dev *pdev;
	struct input_dev *idev;
	int err, i;

	pad = devm_kzalloc(&spi->dev, sizeof(struct psxpad), GFP_KERNEL);
	if (!pad) {
		err = -ENOMEM;
		goto err_free_mem;
	}
	pdev = input_allocate_polled_device();
	if (!pdev) {
		dev_err(&spi->dev, "psxpad-spi: probe: pdev alloc failed!!\n");
		err = -ENOMEM;
		goto err_free_mem;
	}
	for (i = 0; i < sizeof(PSX_CMD_POLL); i++)
		pad->poolcmd[i] = PSX_CMD_POLL[i];
	for (i = 0; i < sizeof(PSX_CMD_ENABLE_MOTOR); i++)
		pad->enablemotor[i] = PSX_CMD_ENABLE_MOTOR[i];
	for (i = 0; i < sizeof(PSX_CMD_AD_MODE); i++)
		pad->admode[i] = PSX_CMD_AD_MODE[i];
	pad->spi_delay = PSXPAD_DEFAULT_SPI_DELAY;

	/* input pool device settings */
	pad->pdev = pdev;
	pad->spi = spi;
	pdev->private = pad;
	pdev->open = psxpad_spi_poll_open;
	pdev->close = psxpad_spi_poll_close;
	pdev->poll = psxpad_spi_poll;
	pdev->poll_interval = PSXPAD_DEFAULT_INTERVAL;
	pdev->poll_interval_min = PSXPAD_DEFAULT_INTERVAL_MIN;
	pdev->poll_interval_max = PSXPAD_DEFAULT_INTERVAL_MAX;

	/* input device settings */
	idev = pdev->input;
	pad->idev = idev;
	idev->name = "PSX (PS1/2) pad";
	snprintf(pad->phys, sizeof(pad->phys), "%s/input", dev_name(&spi->dev));
	idev->id.bustype = BUS_SPI;
	idev->dev.parent = &spi->dev;
	idev->dev.platform_data = pad;

	/* key/value map settings */
	input_set_abs_params(idev, ABS_X,		0, 255, 0, 0);
	input_set_abs_params(idev, ABS_Y,		0, 255, 0, 0);
	input_set_abs_params(idev, ABS_RX,		0, 255, 0, 0);
	input_set_abs_params(idev, ABS_RY,		0, 255, 0, 0);
#ifdef PSXPAD_ENABLE_ANALOG2
	input_set_abs_params(idev, ABS_HAT0Y,		0, 255, 0, 0);	/* up */
	input_set_abs_params(idev, ABS_HAT1Y,		0, 255, 0, 0);	/* down */
	input_set_abs_params(idev, ABS_HAT0X,		0, 255, 0, 0);	/* left */
	input_set_abs_params(idev, ABS_HAT1X,		0, 255, 0, 0);	/* right */
	input_set_abs_params(idev, ABS_MISC,		0, 255, 0, 0);
	input_set_abs_params(idev, ABS_PRESSURE,	0, 255, 0, 0);
	input_set_abs_params(idev, ABS_BRAKE,		0, 255, 0, 0);
	input_set_abs_params(idev, ABS_THROTTLE,	0, 255, 0, 0);
	input_set_abs_params(idev, ABS_HAT2X,		0, 255, 0, 0);	/* L1 */
	input_set_abs_params(idev, ABS_HAT3X,		0, 255, 0, 0);	/* R1 */
	input_set_abs_params(idev, ABS_HAT2Y,		0, 255, 0, 0);	/* L2 */
	input_set_abs_params(idev, ABS_HAT3Y,		0, 255, 0, 0);	/* R2 */
#endif	/* PSXPAD_ENABLE_ANALOG2 */
	input_set_capability(idev, EV_KEY, BTN_DPAD_UP);
	input_set_capability(idev, EV_KEY, BTN_DPAD_DOWN);
	input_set_capability(idev, EV_KEY, BTN_DPAD_LEFT);
	input_set_capability(idev, EV_KEY, BTN_DPAD_RIGHT);
	input_set_capability(idev, EV_KEY, BTN_A);
	input_set_capability(idev, EV_KEY, BTN_B);
	input_set_capability(idev, EV_KEY, BTN_X);
	input_set_capability(idev, EV_KEY, BTN_Y);
	input_set_capability(idev, EV_KEY, BTN_TL);
	input_set_capability(idev, EV_KEY, BTN_TR);
	input_set_capability(idev, EV_KEY, BTN_TL2);
	input_set_capability(idev, EV_KEY, BTN_TR2);
	input_set_capability(idev, EV_KEY, BTN_THUMBL);
	input_set_capability(idev, EV_KEY, BTN_THUMBR);
	input_set_capability(idev, EV_KEY, BTN_SELECT);
	input_set_capability(idev, EV_KEY, BTN_START);

	/* force feedback */
	err = psxpad_spi_init_ff(pad);
	if (err) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	/* SPI settings */
	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	/* (PSX pad might be possible works 250kHz/500kHz) */
	spi->master->min_speed_hz = 125000;
	spi->master->max_speed_hz = 125000;
	spi_setup(spi);

	/* pad settings */
	psxpad_setmotorlevel(pad, 0, 0);

	/* register input pool device */
	err = input_register_polled_device(pdev);
	if (err) {
		dev_err(&spi->dev, "psxpad-spi: probe: failed register!!\n");
		input_free_polled_device(pdev);
		goto err_free_mem;
	}

	pm_runtime_enable(&spi->dev);

	return 0;

 err_free_mem:
	if (pdev) {
		psxpad_spi_deinit_ff(pad);
		input_free_polled_device(pdev);
	}
	devm_kfree(&spi->dev, pad);

	return err;
}


static int psxpad_spi_remove(struct spi_device *spi)
{
	return 0;
}

static int __maybe_unused psxpad_spi_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct psxpad *pad = spi_get_drvdata(spi);

	pad->sus_analog_mode = pad->analog_mode;
	pad->sus_mode_lock = pad->mode_lock;
	pad->sus_motor1enable = pad->motor1enable;
	pad->sus_motor2enable = pad->motor2enable;
	pad->sus_motor1level = pad->motor1level;
	pad->sus_motor2level = pad->motor2level;

	psxpad_setadmode(pad, false, false);
	psxpad_setmotorlevel(pad, 0, 0);
	psxpad_setenablemotor(pad, false, false);

	return 0;
}

static int __maybe_unused psxpad_spi_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct psxpad *pad = spi_get_drvdata(spi);

	psxpad_setadmode(pad, pad->sus_analog_mode, pad->sus_mode_lock);
	psxpad_setmotorlevel(pad, pad->sus_motor1enable, pad->sus_motor2enable);
	psxpad_setenablemotor(pad, pad->sus_motor1level, pad->sus_motor2level);

	return 0;
}

static SIMPLE_DEV_PM_OPS(psxpad_spi_pm, psxpad_spi_suspend, psxpad_spi_resume);

static const struct spi_device_id psxpad_spi_id[] = {
	{ "psxpad-spi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, psxpad_spi_id);

static struct spi_driver psxpad_spi_driver = {
	.driver = {
		.name = "psxpad-spi",
		.pm = &psxpad_spi_pm,
	},
	.id_table = psxpad_spi_id,
	.probe   = psxpad_spi_probe,
	.remove  = psxpad_spi_remove,
};

module_spi_driver(psxpad_spi_driver);

MODULE_AUTHOR("AZO <typesylph@gmail.com>");
MODULE_DESCRIPTION("PSX (Play Station 1/2) pad with SPI Bus Driver");
MODULE_LICENSE("GPL");
