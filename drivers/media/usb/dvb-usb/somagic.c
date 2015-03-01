/* USB 1-Seg ISDB-T receiver
 * Somagic reference design using:
 * - Maxim's MAX2163 tuner
 * - MegaChips MA50159 demodulator
 * - Somagic SMI2020-CBE
 *
 * (C) 2015 Felipe C. da S. Sanches <juca@members.fsf.org>
 *
 *   This program is free software; you can redistribute it and/or modify it
 *   under the terms of the GNU General Public License as published by the Free
 *   Software Foundation, version 2 (or later).
 *
 */
#include <media/tuner.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include "max2163.h"

/* Max transfer size done by I2C transfer functions */
#define MAX_XFER_SIZE  80

/* debug */
static int dvb_usb_somagic_debug;
module_param_named(debug, dvb_usb_somagic_debug, int, 0644);
MODULE_PARM_DESC(debug, "set debugging level (1=rc (or-able))." DVB_USB_DEBUG_STATUS);

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

#define deb_info(args...)   dprintk(dvb_usb_somagic_debug, 0x03, args)
#define deb_i2c(args...)    dprintk(dvb_usb_somagic_debug, 0x02, args)

static int somagic_ctrl_msg(struct dvb_usb_device *d,
			  u8 cmd, u8 *wbuf, int wlen, u8 *rbuf, int rlen)
{
	int wo = (rbuf == NULL || rlen == 0); /* write-only */
	u8 sndbuf[MAX_XFER_SIZE];

	if (1 + wlen > sizeof(sndbuf)) {
		warn("i2c wr: len=%d is too big!\n",
		     wlen);
		return -EOPNOTSUPP;
	}

	memset(sndbuf, 0, 1+wlen);

	sndbuf[0] = cmd;
	memcpy(&sndbuf[1], wbuf, wlen);
	if (wo)
		return dvb_usb_generic_write(d, sndbuf, 1+wlen);
	else
		return dvb_usb_generic_rw(d, sndbuf, 1+wlen, rbuf, rlen, 0);
}

/* GPIO */
static void somagic_gpio_tuner(struct dvb_usb_device *d, int onoff)
{
	struct somagic_state *st = d->priv;
	u8 o[2], i;

	if (st->gpio_write_state[GPIO_TUNER] == onoff)
		return;

	o[0] = GPIO_TUNER;
	o[1] = onoff;
	somagic_ctrl_msg(d, CMD_GPIO_WRITE, o, 2, &i, 1);

	if (i != 0x01)
		deb_info("gpio_write failed.\n");

	st->gpio_write_state[GPIO_TUNER] = onoff;
}

static int somagic_bluebird_gpio_rw(struct dvb_usb_device *d, u8 changemask,
				 u8 newval)
{
	u8 o[2], gpio_state;
	int rc;

	o[0] = 0xff & ~changemask;	/* mask of bits to keep */
	o[1] = newval & changemask;	/* new values for bits  */

	rc = somagic_ctrl_msg(d, CMD_BLUEBIRD_GPIO_RW, o, 2, &gpio_state, 1);
	if (rc < 0 || (gpio_state & changemask) != (newval & changemask))
		deb_info("bluebird_gpio_write failed.\n");

	return rc < 0 ? rc : gpio_state;
}

static void somagic_bluebird_gpio_pulse(struct dvb_usb_device *d, u8 pin, int low)
{
	somagic_bluebird_gpio_rw(d, pin, low ? 0 : pin);
	msleep(5);
	somagic_bluebird_gpio_rw(d, pin, low ? pin : 0);
}

static void somagic_nano2_led(struct dvb_usb_device *d, int onoff)
{
	somagic_bluebird_gpio_rw(d, 0x40, onoff ? 0 : 0x40);
}

static int somagic_d680_dmb_gpio_tuner(struct dvb_usb_device *d,
		u8 addr, int onoff)
{
	u8  o[2] = {addr, onoff};
	u8  i;
	int rc;

	rc = somagic_ctrl_msg(d, CMD_GPIO_WRITE, o, 2, &i, 1);

	if (rc < 0)
		return rc;
	if (i == 0x01)
		return 0;
	else {
		deb_info("gpio_write failed.\n");
		return -EIO;
	}
}

/* I2C */
static int somagic_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msg[],
			  int num)
{
	struct dvb_usb_device *d = i2c_get_adapdata(adap);
	int ret;
	int i;

	if (mutex_lock_interruptible(&d->i2c_mutex) < 0)
		return -EAGAIN;

	for (i = 0; i < num; i++) {

		if (le16_to_cpu(d->udev->descriptor.idVendor) == USB_VID_MEDION)
			switch (msg[i].addr) {
			case 0x63:
				somagic_gpio_tuner(d, 0);
				break;
			default:
				somagic_gpio_tuner(d, 1);
				break;
			}

		if (msg[i].flags & I2C_M_RD) {
			/* read only */
			u8 obuf[3], ibuf[MAX_XFER_SIZE];

			if (1 + msg[i].len > sizeof(ibuf)) {
				warn("i2c rd: len=%d is too big!\n",
				     msg[i].len);
				ret = -EOPNOTSUPP;
				goto unlock;
			}
			obuf[0] = 0;
			obuf[1] = msg[i].len;
			obuf[2] = msg[i].addr;
			if (somagic_ctrl_msg(d, CMD_I2C_READ,
					   obuf, 3,
					   ibuf, 1+msg[i].len) < 0) {
				warn("i2c read failed");
				break;
			}
			memcpy(msg[i].buf, &ibuf[1], msg[i].len);
		} else if (i+1 < num && (msg[i+1].flags & I2C_M_RD) &&
			   msg[i].addr == msg[i+1].addr) {
			/* write to then read from same address */
			u8 obuf[MAX_XFER_SIZE], ibuf[MAX_XFER_SIZE];

			if (3 + msg[i].len > sizeof(obuf)) {
				warn("i2c wr: len=%d is too big!\n",
				     msg[i].len);
				ret = -EOPNOTSUPP;
				goto unlock;
			}
			if (1 + msg[i + 1].len > sizeof(ibuf)) {
				warn("i2c rd: len=%d is too big!\n",
				     msg[i + 1].len);
				ret = -EOPNOTSUPP;
				goto unlock;
			}
			obuf[0] = msg[i].len;
			obuf[1] = msg[i+1].len;
			obuf[2] = msg[i].addr;
			memcpy(&obuf[3], msg[i].buf, msg[i].len);

			if (somagic_ctrl_msg(d, CMD_I2C_READ,
					   obuf, 3+msg[i].len,
					   ibuf, 1+msg[i+1].len) < 0)
				break;

			if (ibuf[0] != 0x08)
				deb_i2c("i2c read may have failed\n");

			memcpy(msg[i+1].buf, &ibuf[1], msg[i+1].len);

			i++;
		} else {
			/* write only */
			u8 obuf[MAX_XFER_SIZE], ibuf;

			if (2 + msg[i].len > sizeof(obuf)) {
				warn("i2c wr: len=%d is too big!\n",
				     msg[i].len);
				ret = -EOPNOTSUPP;
				goto unlock;
			}
			obuf[0] = msg[i].addr;
			obuf[1] = msg[i].len;
			memcpy(&obuf[2], msg[i].buf, msg[i].len);

			if (somagic_ctrl_msg(d, CMD_I2C_WRITE, obuf,
					   2+msg[i].len, &ibuf,1) < 0)
				break;
			if (ibuf != 0x08)
				deb_i2c("i2c write may have failed\n");
		}
	}

	if (i == num)
		ret = num;
	else
		ret = -EREMOTEIO;

unlock:
	mutex_unlock(&d->i2c_mutex);
	return ret;
}

static u32 somagic_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

static struct i2c_algorithm somagic_i2c_algo = {
	.master_xfer   = somagic_i2c_xfer,
	.functionality = somagic_i2c_func,
};

static int somagic_power_ctrl(struct dvb_usb_device *d, int onoff)
{
	u8 b = 0;
	if (onoff)
		return somagic_ctrl_msg(d, CMD_POWER_ON, &b, 1, NULL, 0);
	else
		return somagic_ctrl_msg(d, CMD_POWER_OFF, &b, 1, NULL, 0);
}

static int somagic_aver_power_ctrl(struct dvb_usb_device *d, int onoff)
{
	int ret;
	if (!onoff)
		return somagic_ctrl_msg(d, CMD_POWER_OFF, NULL, 0, NULL, 0);
	if (d->state == DVB_USB_STATE_INIT &&
	    usb_set_interface(d->udev, 0, 0) < 0)
		err("set interface failed");
	do {} while (!(ret = somagic_ctrl_msg(d, CMD_POWER_ON, NULL, 0, NULL, 0)) &&
		   !(ret = somagic_ctrl_msg(d, 0x15, NULL, 0, NULL, 0)) &&
		   !(ret = somagic_ctrl_msg(d, 0x17, NULL, 0, NULL, 0)) && 0);
	if (!ret) {
		/* FIXME: We don't know why, but we need to configure the
		 * lgdt3303 with the register settings below on resume */
		int i;
		u8 buf, bufs[] = {
			0x0e, 0x2, 0x00, 0x7f,
			0x0e, 0x2, 0x02, 0xfe,
			0x0e, 0x2, 0x02, 0x01,
			0x0e, 0x2, 0x00, 0x03,
			0x0e, 0x2, 0x0d, 0x40,
			0x0e, 0x2, 0x0e, 0x87,
			0x0e, 0x2, 0x0f, 0x8e,
			0x0e, 0x2, 0x10, 0x01,
			0x0e, 0x2, 0x14, 0xd7,
			0x0e, 0x2, 0x47, 0x88,
		};
		msleep(20);
		for (i = 0; i < sizeof(bufs)/sizeof(u8); i += 4/sizeof(u8)) {
			ret = somagic_ctrl_msg(d, CMD_I2C_WRITE,
					     bufs+i, 4, &buf, 1);
			if (ret)
				break;
			if (buf != 0x8)
				return -EREMOTEIO;
		}
	}
	return ret;
}

static int somagic_bluebird_power_ctrl(struct dvb_usb_device *d, int onoff)
{
	u8 b = 0;
	if (onoff)
		return somagic_ctrl_msg(d, CMD_POWER_ON, &b, 1, NULL, 0);
	else
		return 0;
}

static int somagic_nano2_power_ctrl(struct dvb_usb_device *d, int onoff)
{
	int rc = 0;

	rc = somagic_power_ctrl(d, onoff);
	if (!onoff)
		somagic_nano2_led(d, 0);

	return rc;
}

static int somagic_d680_dmb_power_ctrl(struct dvb_usb_device *d, int onoff)
{
	int ret;
	u8  b;
	ret = somagic_power_ctrl(d, onoff);
	if (!onoff)
		return ret;

	msleep(128);
	somagic_ctrl_msg(d, CMD_DIGITAL, NULL, 0, &b, 1);
	msleep(100);
	return ret;
}

static int somagic_streaming_ctrl(struct dvb_usb_adapter *adap, int onoff)
{
	u8 buf[2] = { 0x03, 0x00 };
	if (onoff)
		somagic_ctrl_msg(adap->dev, CMD_STREAMING_ON, buf, 2, NULL, 0);
	else
		somagic_ctrl_msg(adap->dev, CMD_STREAMING_OFF, NULL, 0, NULL, 0);

	return 0;
}

static int somagic_aver_streaming_ctrl(struct dvb_usb_adapter *adap, int onoff)
{
	if (onoff)
		somagic_ctrl_msg(adap->dev, CMD_AVER_STREAM_ON, NULL, 0, NULL, 0);
	else
		somagic_ctrl_msg(adap->dev, CMD_AVER_STREAM_OFF,
			       NULL, 0, NULL, 0);
	return 0;
}

static void somagic_d680_dmb_drain_message(struct dvb_usb_device *d)
{
	int       ep = d->props.generic_bulk_ctrl_endpoint;
	const int timeout = 100;
	const int junk_len = 32;
	u8        *junk;
	int       rd_count;

	/* Discard remaining data in video pipe */
	junk = kmalloc(junk_len, GFP_KERNEL);
	if (!junk)
		return;
	while (1) {
		if (usb_bulk_msg(d->udev,
			usb_rcvbulkpipe(d->udev, ep),
			junk, junk_len, &rd_count, timeout) < 0)
			break;
		if (!rd_count)
			break;
	}
	kfree(junk);
}

static void somagic_d680_dmb_drain_video(struct dvb_usb_device *d)
{
	struct usb_data_stream_properties *p = &d->props.adapter[0].fe[0].stream;
	const int timeout = 100;
	const int junk_len = p->u.bulk.buffersize;
	u8        *junk;
	int       rd_count;

	/* Discard remaining data in video pipe */
	junk = kmalloc(junk_len, GFP_KERNEL);
	if (!junk)
		return;
	while (1) {
		if (usb_bulk_msg(d->udev,
			usb_rcvbulkpipe(d->udev, p->endpoint),
			junk, junk_len, &rd_count, timeout) < 0)
			break;
		if (!rd_count)
			break;
	}
	kfree(junk);
}

static int somagic_d680_dmb_streaming_ctrl(
		struct dvb_usb_adapter *adap, int onoff)
{
	if (onoff) {
		u8 buf[2] = { 0x03, 0x00 };
		somagic_d680_dmb_drain_video(adap->dev);
		return somagic_ctrl_msg(adap->dev, CMD_STREAMING_ON,
			buf, sizeof(buf), NULL, 0);
	} else {
		int ret = somagic_ctrl_msg(adap->dev,
			CMD_STREAMING_OFF, NULL, 0, NULL, 0);
		return ret;
	}
}

static int somagic_rc_query(struct dvb_usb_device *d, u32 *event, int *state)
{
	struct rc_map_table *keymap = d->props.rc.legacy.rc_map_table;
	u8 ircode[4];
	int i;

	somagic_ctrl_msg(d, CMD_GET_IR_CODE, NULL, 0, ircode, 4);

	*event = 0;
	*state = REMOTE_NO_KEY_PRESSED;

	for (i = 0; i < d->props.rc.legacy.rc_map_size; i++) {
		if (rc5_custom(&keymap[i]) == ircode[2] &&
		    rc5_data(&keymap[i]) == ircode[3]) {
			*event = keymap[i].keycode;
			*state = REMOTE_KEY_PRESSED;

			return 0;
		}
	}

	return 0;
}

static int somagic_bluebird2_rc_query(struct dvb_usb_device *d, u32 *event,
				    int *state)
{
	struct rc_map_table *keymap = d->props.rc.legacy.rc_map_table;
	u8 ircode[4];
	int i;
	struct i2c_msg msg = { .addr = 0x6b, .flags = I2C_M_RD,
			       .buf = ircode, .len = 4 };

	*event = 0;
	*state = REMOTE_NO_KEY_PRESSED;

	if (somagic_i2c_xfer(&d->i2c_adap, &msg, 1) != 1)
		return 0;

	for (i = 0; i < d->props.rc.legacy.rc_map_size; i++) {
		if (rc5_custom(&keymap[i]) == ircode[1] &&
		    rc5_data(&keymap[i]) == ircode[2]) {
			*event = keymap[i].keycode;
			*state = REMOTE_KEY_PRESSED;

			return 0;
		}
	}

	return 0;
}

static int somagic_d680_dmb_rc_query(struct dvb_usb_device *d, u32 *event,
		int *state)
{
	struct rc_map_table *keymap = d->props.rc.legacy.rc_map_table;
	u8 ircode[2];
	int i;

	*event = 0;
	*state = REMOTE_NO_KEY_PRESSED;

	if (somagic_ctrl_msg(d, 0x10, NULL, 0, ircode, 2) < 0)
		return 0;

	for (i = 0; i < d->props.rc.legacy.rc_map_size; i++) {
		if (rc5_custom(&keymap[i]) == ircode[0] &&
		    rc5_data(&keymap[i]) == ircode[1]) {
			*event = keymap[i].keycode;
			*state = REMOTE_KEY_PRESSED;

			return 0;
		}
	}

	return 0;
}

static struct rc_map_table rc_map_dvico_mce_table[] = {
	{ 0xfe02, KEY_TV },
	{ 0xfe0e, KEY_MP3 },
	{ 0xfe1a, KEY_DVD },
	{ 0xfe1e, KEY_FAVORITES },
	{ 0xfe16, KEY_SETUP },
	{ 0xfe46, KEY_POWER2 },
	{ 0xfe0a, KEY_EPG },
	{ 0xfe49, KEY_BACK },
	{ 0xfe4d, KEY_MENU },
	{ 0xfe51, KEY_UP },
	{ 0xfe5b, KEY_LEFT },
	{ 0xfe5f, KEY_RIGHT },
	{ 0xfe53, KEY_DOWN },
	{ 0xfe5e, KEY_OK },
	{ 0xfe59, KEY_INFO },
	{ 0xfe55, KEY_TAB },
	{ 0xfe0f, KEY_PREVIOUSSONG },/* Replay */
	{ 0xfe12, KEY_NEXTSONG },	/* Skip */
	{ 0xfe42, KEY_ENTER	 },	/* Windows/Start */
	{ 0xfe15, KEY_VOLUMEUP },
	{ 0xfe05, KEY_VOLUMEDOWN },
	{ 0xfe11, KEY_CHANNELUP },
	{ 0xfe09, KEY_CHANNELDOWN },
	{ 0xfe52, KEY_CAMERA },
	{ 0xfe5a, KEY_TUNER },	/* Live */
	{ 0xfe19, KEY_OPEN },
	{ 0xfe0b, KEY_1 },
	{ 0xfe17, KEY_2 },
	{ 0xfe1b, KEY_3 },
	{ 0xfe07, KEY_4 },
	{ 0xfe50, KEY_5 },
	{ 0xfe54, KEY_6 },
	{ 0xfe48, KEY_7 },
	{ 0xfe4c, KEY_8 },
	{ 0xfe58, KEY_9 },
	{ 0xfe13, KEY_ANGLE },	/* Aspect */
	{ 0xfe03, KEY_0 },
	{ 0xfe1f, KEY_ZOOM },
	{ 0xfe43, KEY_REWIND },
	{ 0xfe47, KEY_PLAYPAUSE },
	{ 0xfe4f, KEY_FASTFORWARD },
	{ 0xfe57, KEY_MUTE },
	{ 0xfe0d, KEY_STOP },
	{ 0xfe01, KEY_RECORD },
	{ 0xfe4e, KEY_POWER },
};

static struct rc_map_table rc_map_dvico_portable_table[] = {
	{ 0xfc02, KEY_SETUP },       /* Profile */
	{ 0xfc43, KEY_POWER2 },
	{ 0xfc06, KEY_EPG },
	{ 0xfc5a, KEY_BACK },
	{ 0xfc05, KEY_MENU },
	{ 0xfc47, KEY_INFO },
	{ 0xfc01, KEY_TAB },
	{ 0xfc42, KEY_PREVIOUSSONG },/* Replay */
	{ 0xfc49, KEY_VOLUMEUP },
	{ 0xfc09, KEY_VOLUMEDOWN },
	{ 0xfc54, KEY_CHANNELUP },
	{ 0xfc0b, KEY_CHANNELDOWN },
	{ 0xfc16, KEY_CAMERA },
	{ 0xfc40, KEY_TUNER },	/* ATV/DTV */
	{ 0xfc45, KEY_OPEN },
	{ 0xfc19, KEY_1 },
	{ 0xfc18, KEY_2 },
	{ 0xfc1b, KEY_3 },
	{ 0xfc1a, KEY_4 },
	{ 0xfc58, KEY_5 },
	{ 0xfc59, KEY_6 },
	{ 0xfc15, KEY_7 },
	{ 0xfc14, KEY_8 },
	{ 0xfc17, KEY_9 },
	{ 0xfc44, KEY_ANGLE },	/* Aspect */
	{ 0xfc55, KEY_0 },
	{ 0xfc07, KEY_ZOOM },
	{ 0xfc0a, KEY_REWIND },
	{ 0xfc08, KEY_PLAYPAUSE },
	{ 0xfc4b, KEY_FASTFORWARD },
	{ 0xfc5b, KEY_MUTE },
	{ 0xfc04, KEY_STOP },
	{ 0xfc56, KEY_RECORD },
	{ 0xfc57, KEY_POWER },
	{ 0xfc41, KEY_UNKNOWN },    /* INPUT */
	{ 0xfc00, KEY_UNKNOWN },    /* HD */
};

static struct rc_map_table rc_map_d680_dmb_table[] = {
	{ 0x0038, KEY_UNKNOWN },	/* TV/AV */
	{ 0x080c, KEY_ZOOM },
	{ 0x0800, KEY_0 },
	{ 0x0001, KEY_1 },
	{ 0x0802, KEY_2 },
	{ 0x0003, KEY_3 },
	{ 0x0804, KEY_4 },
	{ 0x0005, KEY_5 },
	{ 0x0806, KEY_6 },
	{ 0x0007, KEY_7 },
	{ 0x0808, KEY_8 },
	{ 0x0009, KEY_9 },
	{ 0x000a, KEY_MUTE },
	{ 0x0829, KEY_BACK },
	{ 0x0012, KEY_CHANNELUP },
	{ 0x0813, KEY_CHANNELDOWN },
	{ 0x002b, KEY_VOLUMEUP },
	{ 0x082c, KEY_VOLUMEDOWN },
	{ 0x0020, KEY_UP },
	{ 0x0821, KEY_DOWN },
	{ 0x0011, KEY_LEFT },
	{ 0x0810, KEY_RIGHT },
	{ 0x000d, KEY_OK },
	{ 0x081f, KEY_RECORD },
	{ 0x0017, KEY_PLAYPAUSE },
	{ 0x0816, KEY_PLAYPAUSE },
	{ 0x000b, KEY_STOP },
	{ 0x0827, KEY_FASTFORWARD },
	{ 0x0026, KEY_REWIND },
	{ 0x081e, KEY_UNKNOWN },    /* Time Shift */
	{ 0x000e, KEY_UNKNOWN },    /* Snapshot */
	{ 0x082d, KEY_UNKNOWN },    /* Mouse Cursor */
	{ 0x000f, KEY_UNKNOWN },    /* Minimize/Maximize */
	{ 0x0814, KEY_UNKNOWN },    /* Shuffle */
	{ 0x0025, KEY_POWER },
};

static int somagic_dee1601_demod_init(struct dvb_frontend* fe)
{
	static u8 clock_config []  = { CLOCK_CTL,  0x38, 0x28 };
	static u8 reset []         = { RESET,      0x80 };
	static u8 adc_ctl_1_cfg [] = { ADC_CTL_1,  0x40 };
	static u8 agc_cfg []       = { AGC_TARGET, 0x28, 0x20 };
	static u8 gpp_ctl_cfg []   = { GPP_CTL,    0x33 };
	static u8 capt_range_cfg[] = { CAPT_RANGE, 0x32 };

	mt352_write(fe, clock_config,   sizeof(clock_config));
	udelay(200);
	mt352_write(fe, reset,          sizeof(reset));
	mt352_write(fe, adc_ctl_1_cfg,  sizeof(adc_ctl_1_cfg));

	mt352_write(fe, agc_cfg,        sizeof(agc_cfg));
	mt352_write(fe, gpp_ctl_cfg,    sizeof(gpp_ctl_cfg));
	mt352_write(fe, capt_range_cfg, sizeof(capt_range_cfg));

	return 0;
}

static int somagic_1seg_demod_init(struct dvb_frontend* fe)
{	/* used in both lgz201 and th7579 */
	static u8 clock_config []  = { CLOCK_CTL,  0x38, 0x29 };
	static u8 reset []         = { RESET,      0x80 };
	static u8 adc_ctl_1_cfg [] = { ADC_CTL_1,  0x40 };
	static u8 agc_cfg []       = { AGC_TARGET, 0x24, 0x20 };
	static u8 gpp_ctl_cfg []   = { GPP_CTL,    0x33 };
	static u8 capt_range_cfg[] = { CAPT_RANGE, 0x32 };

	mt352_write(fe, clock_config,   sizeof(clock_config));
	udelay(200);
	mt352_write(fe, reset,          sizeof(reset));
	mt352_write(fe, adc_ctl_1_cfg,  sizeof(adc_ctl_1_cfg));

	mt352_write(fe, agc_cfg,        sizeof(agc_cfg));
	mt352_write(fe, gpp_ctl_cfg,    sizeof(gpp_ctl_cfg));
	mt352_write(fe, capt_range_cfg, sizeof(capt_range_cfg));
	return 0;
}

static struct smi2020_config somagic_smi2020_demod_config = {
	.demod_address = 0xee,
	.demod_init    = somagic_1seg_demod_init,
};

static struct max2163_config somagic_max2163_cfg = {
	.i2c_address = 0xC0,
	.osc_clk = 32
};

static int somagic_tuner_attach(struct dvb_usb_adapter *adap)
{
	struct dvb_frontend *fe;
	fe = dvb_attach(max2163_attach, adap->fe_adap[0].fe,
			&adap->dev->i2c_adap, &somagic_max2163_cfg);
	return (fe == NULL) ? -EIO : 0;
}

somagic_smi2020cbe_max2163_config

static int somagic_1seg_frontend_attach(struct dvb_usb_adapter *adap)
{
	if (usb_set_interface(adap->dev->udev, 0, 1) < 0)
		err("set interface failed");

	adap->fe_adap[0].fe = dvb_attach(smi2020cbe_attach,
					 &somagic_smi2020cbe_max2163_config,
					 &adap->dev->i2c_adap);

	if ((adap->fe_adap[0].fe) != NULL)
		return 0;

	return -EIO;
}

static struct dvb_usb_device_properties somagic_1seg_properties;

static int somagic_probe(struct usb_interface *intf,
		       const struct usb_device_id *id)
{
	if (0 == dvb_usb_device_init(intf, &somagic_1seg_properties,
                                 THIS_MODULE, NULL, adapter_nr))
		return 0;

	return -EINVAL;
}

static void somagic_disconnect(struct usb_interface *intf)
{
	struct dvb_usb_device *d = usb_get_intfdata(intf);
	struct somagic_state *st = d->priv;
	struct i2c_client *client;

	/* remove I2C client for tuner */
	client = st->i2c_client_tuner;
	if (client) {
		module_put(client->dev.driver->owner);
		i2c_unregister_device(client);
	}

	/* remove I2C client for demodulator */
	client = st->i2c_client_demod;
	if (client) {
		module_put(client->dev.driver->owner);
		i2c_unregister_device(client);
	}

	dvb_usb_device_exit(intf);
}

static struct usb_device_id somagic_table [] = {
	{ USB_DEVICE(USB_VID_ZINWELL, USB_PID_ZINWELL) },
	{}		/* Terminating entry */
};
MODULE_DEVICE_TABLE (usb, somagic_table);

static struct dvb_usb_device_properties somagic_1seg_properties = {
	.caps = DVB_USB_IS_AN_I2C_ADAPTER,

	.usb_ctrl         = DEVICE_SPECIFIC,

	.size_of_priv     = sizeof(struct somagic_state),

	.num_adapters = 1,
	.adapter = {
		{
		.num_frontends = 1,
		.fe = {
                {
			        .streaming_ctrl   = somagic_streaming_ctrl,
			        .frontend_attach  = somagic_frontend_attach,
        			.tuner_attach     = somagic_tuner_attach,


			        /* parameter for the MPEG2-data transfer */
			        .stream = {
				        .type = USB_ISOC,
				        .count = 8,
				        .endpoint = 0x01,
			            .u = {
				            .isoc = {
					            .framesperurb = 64,
					            .framesize = 940,
					            .interval = 1,
				            }
			            }
			        },
		        }
            },
		},
	},

	.power_ctrl       = somagic_1seg_power_ctrl,

	.i2c_algo         = &somagic_i2c_algo,

	.generic_bulk_ctrl_endpoint = 0x01,

	.num_device_descs = 1,
	.devices = {
		{
			"Somagic 1-Seg ISDB-T",
			{ NULL },
			{ &somagic_table[20], NULL },
		},
	}
};

static struct usb_driver somagic_driver = {
	.name		= "isdbt_usb_somagic",
	.probe		= somagic_probe,
	.disconnect     = somagic_disconnect,
	.id_table	= somagic_table,
};

module_usb_driver(somagic_driver);

MODULE_AUTHOR("Felipe C. da S. Sanches <juca@members.fsf.org>");
MODULE_DESCRIPTION("Driver for Somagic 1-Seg ISDB-T reference design");
MODULE_LICENSE("GPL");
