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
#ifdef CONFIG_JOYSTICK_PSXPAD_SPI_FF
#define PSXPAD_ENABLE_FF
#endif	/* CONFIG_JOYSTICK_PSXPAD_SPI_FF */

enum {
	PSXPAD_SPI_SPEED_125KHZ = 0,
	PSXPAD_SPI_SPEED_250KHZ,
	PSXPAD_SPI_SPEED_500KHZ,
	PSXPAD_SPI_SPEED_UNKNOWN
};

#define PSXPAD_DEFAULT_SPI_DELAY	100
#define PSXPAD_DEFAULT_SPI_SPEED	PSXPAD_SPI_SPEED_125KHZ
#define PSXPAD_DEFAULT_INTERVAL		16
#define PSXPAD_DEFAULT_INTERVAL_MIN	8
#define PSXPAD_DEFAULT_INTERVAL_MAX	32
#define PSXPAD_DEFAULT_ADMODE		true
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

struct psxpad_keystate {
	int type;
	/* PSXPAD_KEYSTATE_TYPE_DIGITAL */
	bool select;
	bool start;
	bool up;
	bool right;
	bool down;
	bool left;
	bool l2;
	bool r2;
	bool l1;
	bool r1;
	bool triangle;
	bool circle;
	bool cross;
	bool square;
	/* PSXPAD_KEYSTATE_TYPE_ANALOG1 */
	u8 l3;
	u8 r3;
	u8 lx;
	u8 ly;
	u8 rx;
	u8 ry;
#ifdef PSXPAD_ENABLE_ANALOG2
	/* PSXPAD_KEYSTATE_TYPE_ANALOG2 */
	u8 a_right;
	u8 a_left;
	u8 a_up;
	u8 a_down;
	u8 a_triangle;
	u8 a_circle;
	u8 a_cross;
	u8 a_square;
	u8 a_l1;
	u8 a_r1;
	u8 a_l2;
	u8 a_r2;
#endif	/* PSXPAD_ENABLE_ANALOG2 */
};

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
	u8 response[sizeof(PSX_CMD_POLL)];
	u8 enablemotor[sizeof(PSX_CMD_ENABLE_MOTOR)];
	u8 admode[sizeof(PSX_CMD_AD_MODE)];
};

static void psxpad_command(struct psxpad *pad, const u8 sendcmd[], u8 response[], const u8 sendcmdlen)
{
	struct spi_transfer *xfers;
	struct spi_message msg;
	u8 loc;
	u8 sendbuf[0x40];

	xfers = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);
	if (!xfers)
		return;

	spi_message_init(&msg);

	for (loc = 0; loc < sendcmdlen; loc++)
		sendbuf[loc] = REVERSE_BIT(sendcmd[loc]);

	xfers->tx_buf		= sendbuf;
	xfers->rx_buf		= response;
	xfers->len		= sendcmdlen;
	xfers->bits_per_word	= 8;
	xfers->delay_usecs	= pad->spi_delay;
	switch (pad->spi_speed) {
	case PSXPAD_SPI_SPEED_250KHZ:
		xfers->speed_hz = 250000;
		break;
	case PSXPAD_SPI_SPEED_500KHZ:
		xfers->speed_hz = 500000;
		break;
	default:
		xfers->speed_hz = 125000;
		break;
	}
	spi_sync_transfer(pad->spi, xfers, 1);
	kfree(xfers);

	for (loc = 0; loc < sendcmdlen; loc++)
		response[loc] = REVERSE_BIT(response[loc]);
}

static void psxpad_setadmode(struct psxpad *pad, const bool analog_mode, const bool mode_lock)
{
	pad->analog_mode	= analog_mode;
	pad->mode_lock	= mode_lock;

	pad->admode[3] = pad->analog_mode	? 0x01 : 0x00;
	pad->admode[4] = pad->mode_lock		? 0x03 : 0x00;

	psxpad_command(pad, PSX_CMD_ENTER_CFG, pad->response, sizeof(PSX_CMD_ENTER_CFG));
	psxpad_command(pad, pad->admode, pad->response, sizeof(PSX_CMD_AD_MODE));
#ifdef PSXPAD_ENABLE_ANALOG2
	psxpad_command(pad, PSX_CMD_INIT_PRESSURE, pad->response, sizeof(PSX_CMD_INIT_PRESSURE));
	psxpad_command(pad, PSX_CMD_ALL_PRESSURE, pad->response, sizeof(PSX_CMD_ALL_PRESSURE));
#endif	/* PSXPAD_ENABLE_ANALOG2 */
	psxpad_command(pad, PSX_CMD_EXIT_CFG, pad->response, sizeof(PSX_CMD_EXIT_CFG));
}

#ifdef PSXPAD_ENABLE_FF
static void psxpad_setenablemotor(struct psxpad *pad, const bool motor1enable, const bool motor2enable)
{
	pad->motor1enable = motor1enable;
	pad->motor2enable = motor2enable;

	pad->enablemotor[3] = pad->motor1enable ? 0x00 : 0xFF;
	pad->enablemotor[4] = pad->motor2enable ? 0x01 : 0xFF;

	psxpad_command(pad, PSX_CMD_ENTER_CFG, pad->response, sizeof(PSX_CMD_ENTER_CFG));
	psxpad_command(pad, pad->enablemotor, pad->response, sizeof(PSX_CMD_ENABLE_MOTOR));
	psxpad_command(pad, PSX_CMD_EXIT_CFG, pad->response, sizeof(PSX_CMD_EXIT_CFG));
}

static void psxpad_setmotorlevel(struct psxpad *pad, const u8 motor1level, const u8 motor2level)
{
	pad->motor1level = motor1level ? 0xFF : 0x00;
	pad->motor2level = motor2level;

	pad->poolcmd[3] = pad->motor1level;
	pad->poolcmd[4] = pad->motor2level;
}
#else	/* PSXPAD_ENABLE_FF */
static void psxpad_setenablemotor(struct psxpad *pad, const bool motor1enable, const bool motor2enable) { }

static void psxpad_setmotorlevel(struct psxpad *pad, const u8 motor1level, const u8 motor2level) { }
#endif	/* PSXPAD_ENABLE_FF */

static void psxpad_getkeystate(struct psxpad *pad, struct psxpad_keystate *keystate)
{
	keystate->type = PSXPAD_KEYSTATE_TYPE_UNKNOWN;
#ifdef PSXPAD_ENABLE_ANALOG2
	keystate->a_right	= 0;
	keystate->a_left	= 0;
	keystate->a_up		= 0;
	keystate->a_down	= 0;
	keystate->a_triangle	= 0;
	keystate->a_circle	= 0;
	keystate->a_cross	= 0;
	keystate->a_square	= 0;
	keystate->a_l1		= 0;
	keystate->a_r1		= 0;
	keystate->a_l2		= 0;
	keystate->a_r2		= 0;
#endif	/* PSXPAD_ENABLE_ANALOG2 */
	keystate->rx		= 0x80;
	keystate->ry		= 0x80;
	keystate->lx		= 0x80;
	keystate->ly		= 0x80;
	keystate->l3		= false;
	keystate->r3		= false;
	keystate->select	= false;
	keystate->start		= false;
	keystate->up		= false;
	keystate->right		= false;
	keystate->down		= false;
	keystate->left		= false;
	keystate->l2		= false;
	keystate->r2		= false;
	keystate->l1		= false;
	keystate->r1		= false;
	keystate->triangle	= false;
	keystate->circle	= false;
	keystate->cross		= false;
	keystate->square	= false;

	switch (pad->response[1]) {
#ifdef PSXPAD_ENABLE_ANALOG2
	case 0x79:
		keystate->type = PSXPAD_KEYSTATE_TYPE_ANALOG2;
		keystate->a_right	= pad->response[9];
		keystate->a_left	= pad->response[10];
		keystate->a_up		= pad->response[11];
		keystate->a_down	= pad->response[12];
		keystate->a_triangle	= pad->response[13];
		keystate->a_circle	= pad->response[14];
		keystate->a_cross	= pad->response[15];
		keystate->a_square	= pad->response[16];
		keystate->a_l1		= pad->response[17];
		keystate->a_r1		= pad->response[18];
		keystate->a_l2		= pad->response[19];
		keystate->a_r2		= pad->response[20];
#endif	/* PSXPAD_ENABLE_ANALOG2 */
	case 0x73:
		if (keystate->type == PSXPAD_KEYSTATE_TYPE_UNKNOWN)
			keystate->type = PSXPAD_KEYSTATE_TYPE_ANALOG1;
		keystate->rx = pad->response[5];
		keystate->ry = pad->response[6];
		keystate->lx = pad->response[7];
		keystate->ly = pad->response[8];
		keystate->l3 = (pad->response[3] & 0x02U) ? false : true;
		keystate->r3 = (pad->response[3] & 0x04U) ? false : true;
	case 0x41:
		if (keystate->type == PSXPAD_KEYSTATE_TYPE_UNKNOWN)
			keystate->type = PSXPAD_KEYSTATE_TYPE_DIGITAL;
		keystate->select	= (pad->response[3] & 0x01U) ? false : true;
		keystate->start		= (pad->response[3] & 0x08U) ? false : true;
		keystate->up		= (pad->response[3] & 0x10U) ? false : true;
		keystate->right		= (pad->response[3] & 0x20U) ? false : true;
		keystate->down		= (pad->response[3] & 0x40U) ? false : true;
		keystate->left		= (pad->response[3] & 0x80U) ? false : true;
		keystate->l2		= (pad->response[4] & 0x01U) ? false : true;
		keystate->r2		= (pad->response[4] & 0x02U) ? false : true;
		keystate->l1		= (pad->response[4] & 0x04U) ? false : true;
		keystate->r1		= (pad->response[4] & 0x08U) ? false : true;
		keystate->triangle	= (pad->response[4] & 0x10U) ? false : true;
		keystate->circle	= (pad->response[4] & 0x20U) ? false : true;
		keystate->cross		= (pad->response[4] & 0x40U) ? false : true;
		keystate->square	= (pad->response[4] & 0x80U) ? false : true;
	}
}

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
	struct psxpad_keystate keystate;

	psxpad_command(pad, pad->poolcmd, pad->response, sizeof(PSX_CMD_POLL));
	psxpad_getkeystate(pad, &keystate);
	psxpad_setenablemotor(pad, true, true);

	switch (keystate.type) {
#ifdef PSXPAD_ENABLE_ANALOG2
	case PSXPAD_KEYSTATE_TYPE_ANALOG2:
		input_report_abs(pad->idev, ABS_HAT0Y,		keystate.a_up);
		input_report_abs(pad->idev, ABS_HAT1Y,		keystate.a_down);
		input_report_abs(pad->idev, ABS_HAT0X,		keystate.a_left);
		input_report_abs(pad->idev, ABS_HAT1X,		keystate.a_right);
		input_report_abs(pad->idev, ABS_MISC,		keystate.a_triangle);
		input_report_abs(pad->idev, ABS_PRESSURE,	keystate.a_circle);
		input_report_abs(pad->idev, ABS_BRAKE,		keystate.a_cross);
		input_report_abs(pad->idev, ABS_THROTTLE,	keystate.a_square);
		input_report_abs(pad->idev, ABS_HAT2X,		keystate.a_l1);
		input_report_abs(pad->idev, ABS_HAT3X,		keystate.a_r1);
		input_report_abs(pad->idev, ABS_HAT2Y,		keystate.a_l2);
		input_report_abs(pad->idev, ABS_HAT3Y,		keystate.a_r2);
		input_report_abs(pad->idev, ABS_X,		keystate.lx);
		input_report_abs(pad->idev, ABS_Y,		keystate.ly);
		input_report_abs(pad->idev, ABS_RX,		keystate.rx);
		input_report_abs(pad->idev, ABS_RY,		keystate.ry);
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
		input_report_key(pad->idev, BTN_THUMBL,		keystate.l3);
		input_report_key(pad->idev, BTN_THUMBR,		keystate.r3);
		input_report_key(pad->idev, BTN_SELECT,		keystate.select);
		input_report_key(pad->idev, BTN_START,		keystate.start);
		break;
#endif	/* PSXPAD_ENABLE_ANALOG2 */

	case PSXPAD_KEYSTATE_TYPE_ANALOG1:
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
		input_report_abs(pad->idev, ABS_X,		keystate.lx);
		input_report_abs(pad->idev, ABS_Y,		keystate.ly);
		input_report_abs(pad->idev, ABS_RX,		keystate.rx);
		input_report_abs(pad->idev, ABS_RY,		keystate.ry);
		input_report_key(pad->idev, BTN_DPAD_UP,	keystate.up);
		input_report_key(pad->idev, BTN_DPAD_DOWN,	keystate.down);
		input_report_key(pad->idev, BTN_DPAD_LEFT,	keystate.left);
		input_report_key(pad->idev, BTN_DPAD_RIGHT,	keystate.right);
		input_report_key(pad->idev, BTN_X,		keystate.triangle);
		input_report_key(pad->idev, BTN_A,		keystate.circle);
		input_report_key(pad->idev, BTN_B,		keystate.cross);
		input_report_key(pad->idev, BTN_Y,		keystate.square);
		input_report_key(pad->idev, BTN_TL,		keystate.l1);
		input_report_key(pad->idev, BTN_TR,		keystate.r1);
		input_report_key(pad->idev, BTN_TL2,		keystate.l2);
		input_report_key(pad->idev, BTN_TR2,		keystate.r2);
		input_report_key(pad->idev, BTN_THUMBL,		keystate.l3);
		input_report_key(pad->idev, BTN_THUMBR,		keystate.r3);
		input_report_key(pad->idev, BTN_SELECT,		keystate.select);
		input_report_key(pad->idev, BTN_START,		keystate.start);
		break;

	case PSXPAD_KEYSTATE_TYPE_DIGITAL:
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
		input_report_key(pad->idev, BTN_DPAD_UP,	keystate.up);
		input_report_key(pad->idev, BTN_DPAD_DOWN,	keystate.down);
		input_report_key(pad->idev, BTN_DPAD_LEFT,	keystate.left);
		input_report_key(pad->idev, BTN_DPAD_RIGHT,	keystate.right);
		input_report_key(pad->idev, BTN_X,		keystate.triangle);
		input_report_key(pad->idev, BTN_A,		keystate.circle);
		input_report_key(pad->idev, BTN_B,		keystate.cross);
		input_report_key(pad->idev, BTN_Y,		keystate.square);
		input_report_key(pad->idev, BTN_TL,		keystate.l1);
		input_report_key(pad->idev, BTN_TR,		keystate.r1);
		input_report_key(pad->idev, BTN_TL2,		keystate.l2);
		input_report_key(pad->idev, BTN_TR2,		keystate.r2);
		input_report_key(pad->idev, BTN_THUMBL,		false);
		input_report_key(pad->idev, BTN_THUMBR,		false);
		input_report_key(pad->idev, BTN_SELECT,		keystate.select);
		input_report_key(pad->idev, BTN_START,		keystate.start);
		break;
	}

	input_sync(pad->idev);
}

#ifdef PSXPAD_ENABLE_FF
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
		pr_err("psxpad-spi: ff alloc failed!!\n");
		err = -ENOMEM;
	}

	return err;
}

static void psxpad_spi_deinit_ff(struct psxpad *pad)
{
	input_ff_destroy(pad->idev);
}
#else	/* PSXPAD_ENABLE_FF */
static inline int psxpad_spi_init_ff(struct psxpad *pad)
{
	return 0;
}

static void psxpad_spi_deinit_ff(struct psxpad *pad) { }
#endif	/* PSXPAD_ENABLE_FF */

static int psxpad_spi_probe(struct spi_device *spi)
{
	struct psxpad *pad = NULL;
	struct input_polled_dev *pdev = NULL;
	struct input_dev *idev;
	int err, i;

	pad = devm_kzalloc(&spi->dev, sizeof(struct psxpad), GFP_KERNEL);
	pdev = input_allocate_polled_device();
	if (!pad || !pdev) {
		pr_err("psxpad-spi: alloc failed!!\n");
		err = -ENOMEM;
		goto err_free_mem;
	}
	pdev->input->ff = NULL;
	for (i = 0; i < sizeof(PSX_CMD_POLL); i++)
		pad->poolcmd[i] = PSX_CMD_POLL[i];
	for (i = 0; i < sizeof(PSX_CMD_ENABLE_MOTOR); i++)
		pad->enablemotor[i] = PSX_CMD_ENABLE_MOTOR[i];
	for (i = 0; i < sizeof(PSX_CMD_AD_MODE); i++)
		pad->admode[i] = PSX_CMD_AD_MODE[i];
	pad->spi_delay = PSXPAD_DEFAULT_SPI_DELAY;
	pad->spi_speed = PSXPAD_DEFAULT_SPI_SPEED;

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
	snprintf(pad->phys, PSXPAD_DEFAULT_INPUT_PHYSIZE, "%s/input", dev_name(&spi->dev));
	idev->id.bustype = BUS_SPI;
	idev->dev.parent = &spi->dev;
	idev->dev.platform_data = pad;

	/* key/value map settings */
	__set_bit(EV_ABS, idev->evbit);
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
	__set_bit(EV_KEY, idev->evbit);
	__set_bit(BTN_DPAD_UP,		idev->keybit);
	__set_bit(BTN_DPAD_DOWN,	idev->keybit);
	__set_bit(BTN_DPAD_LEFT,	idev->keybit);
	__set_bit(BTN_DPAD_RIGHT,	idev->keybit);
	__set_bit(BTN_A,		idev->keybit);
	__set_bit(BTN_B,		idev->keybit);
	__set_bit(BTN_X,		idev->keybit);
	__set_bit(BTN_Y,		idev->keybit);
	__set_bit(BTN_TL,		idev->keybit);
	__set_bit(BTN_TR,		idev->keybit);
	__set_bit(BTN_TL2,		idev->keybit);
	__set_bit(BTN_TR2,		idev->keybit);
	__set_bit(BTN_THUMBL,		idev->keybit);
	__set_bit(BTN_THUMBR,		idev->keybit);
	__set_bit(BTN_SELECT,		idev->keybit);
	__set_bit(BTN_START,		idev->keybit);

	/* force feedback */
	err = psxpad_spi_init_ff(pad);
	if (err) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	/* SPI settings */
	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	spi_setup(spi);

	/* pad settings */
	psxpad_setmotorlevel(pad, 0, 0);

	/* register input pool device */
	err = input_register_polled_device(pdev);
	if (err) {
		pr_err("psxpad-spi: failed register!!\n");
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
	struct psxpad *pad = spi_get_drvdata(spi);

	psxpad_spi_deinit_ff(pad);
	input_free_polled_device(pad->pdev);
	devm_kfree(&spi->dev, pad);

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

	spi->mode = SPI_MODE_3;
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
