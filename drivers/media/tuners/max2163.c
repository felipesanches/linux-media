/*
 *  Driver for Maxim MAX2163 silicon tuner
 *  ISDB-T 1-Segment Tuner
 *
 *  Copyright (c) 2015 Felipe C. da S. Sanches <juca@members.fsf.org>
 *  Copyright (c) 2010, Lucas C. Villa Real <lucasvr@gobolinux.org>
 *  Copyright (c) 2009 David T. L. Wong <davidtlwong@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/videodev2.h>
#include <linux/delay.h>
#include <linux/dvb/frontend.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include "dvb_frontend.h"

#include "max2163.h"
#include "max2163_priv.h"
#include "tuner-i2c.h"

#define dprintk(args...) \
	do { \
		if (debug) \
			printk(KERN_DEBUG "max2163: " args); \
	} while (0)

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off debugging (default:off).");

static int max2163_write_reg(struct max2163_priv *priv, u8 reg, u8 data)
{
	int ret;
	u8 buf[] = { reg, data };
	struct i2c_msg msg = { .flags = 0, .buf = buf, .len = 2 };

	msg.addr = priv->config->i2c_address;

	if (debug >= 2)
		dprintk("%s: reg=0x%02X, data=0x%02X\n", __func__, reg, data);

	ret = i2c_transfer(priv->i2c, &msg, 1);

	if (ret != 1)
		dprintk("%s: error reg=0x%x, data=0x%x, ret=%i\n",
			__func__, reg, data, ret);

	return (ret != 1) ? -EIO : 0;
}

static int max2163_read_reg(struct max2163_priv *priv, u8 reg, u8 *p_data)
{
	int ret;
	u8 dev_addr = priv->config->i2c_address;

	u8 b0[] = { reg };
	u8 b1[] = { 0 };
	struct i2c_msg msg[] = {
		{ .addr = dev_addr, .flags = 0, .buf = b0, .len = 1 },
		{ .addr = dev_addr, .flags = I2C_M_RD, .buf = b1, .len = 1 },
	};

	ret = i2c_transfer(priv->i2c, msg, 2);
	if (ret != 2) {
		dprintk("%s: error reg=0x%x, ret=%i\n", __func__, reg, ret);
		return -EIO;
	}

	*p_data = b1[0];
	if (debug >= 2)
		dprintk("%s: reg=0x%02X, data=0x%02X\n",
			__func__, reg, b1[0]);
	return 0;
}

static int max2163_mask_write_reg(struct max2163_priv *priv, u8 reg,
	u8 mask, u8 data)
{
	int ret;
	u8 v;

	data &= mask;
	ret = max2163_read_reg(priv, reg, &v);
	if (ret != 0)
		return ret;
	v &= ~mask;
	v |= data;
	ret = max2163_write_reg(priv, reg, v);

	return ret;
}

static int max2163_set_bandwidth(struct max2163_priv *priv, u32 bw)
{
	u8 val;

	switch(bw){
        case 43000000:
            val = BANDWIDTH_43MHZ;
            break;
        case 26000000:
            val = BANDWIDTH_26MHZ;
            break;
        case 17000000:
            val = BANDWIDTH_17MHZ;
            break;
        case 13000000:
            val = BANDWIDTH_13MHZ;
            break;
        default:
            dprintk("%s() bw=%d is unsupported. Using default 13MHz bandwidth configuration.\n", __func__, bw);
            val = BANDWIDTH_13MHZ;
    }

    max2163_mask_write_reg(priv, IF_FILTER_REG,
                                 IF_FILTER_REG_BANDWDITH_MASK, val);

	return 0;
}

static int max2163_set_frequency(struct max2163_priv *priv, u32 freq)
{
    int freq_range;
    int integer_divider;
    int rational_divider = DEFAULT_RDIVIDER;

	if (freq < 488)
		freq_range = UHF_RANGE_470_488MHZ;
	else if (freq < 512)
		freq_range = UHF_RANGE_488_512MHZ;
	else if (freq < 542)
		freq_range = UHF_RANGE_512_542MHZ;
	else if (freq < 572)
		freq_range = UHF_RANGE_542_572MHZ;
	else if (freq < 608)
		freq_range = UHF_RANGE_572_608MHZ;
	else if (freq < 656)
		freq_range = UHF_RANGE_608_656MHZ;
	else if (freq < 710)
		freq_range = UHF_RANGE_656_710MHZ;
	else
		freq_range = UHF_RANGE_710_806MHZ;

	/* Update frequency range */
	max2163_mask_write_reg(priv, RF_FILTER_REG, FREQ_RANGE_MASK, freq_range);
		
	integer_divider = (64 + freq * rational_divider) / priv->config->osc_clk;

	/* Set integer divider value */
	max2163_write_reg(priv, NDIVIDER_MSB_REG,
                                PLL_MOST_NDIVIDER(integer_divider));
	
	max2163_mask_write_reg(priv, NDIVIDER_LSB_REG, NDIVIDER_LSB_MASK,
                                     PLL_LEAST_NDIVIDER(integer_divider));

	/* Set rational divider value */
	max2163_write_reg(priv, RDIVIDER_MSB_REG,
                                PLL_MOST_RDIVIDER(rational_divider));
	
	max2163_mask_write_reg(priv, RDIVIDER_LSB_REG, RDIVIDER_LSB_MASK,
                                     PLL_LEAST_RDIVIDER(rational_divider));

	return 0;
}

static int max2163_set_params(struct dvb_frontend *fe)
{
	struct max2163_priv *priv = fe->tuner_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;

	dprintk("%s() frequency=%d\n", __func__, c->frequency);
	dprintk("%s() bandwidth=%d\n", __func__, c->bandwidth_hz);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	priv->bandwidth = c->bandwidth_hz;
	ret = max2163_set_bandwidth(priv, priv->bandwidth);

	priv->frequency = c->frequency;
	ret = max2163_set_frequency(priv, priv->frequency);
	mdelay(50); /* really needed ? */
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);

	if (ret != 0)
		return -EREMOTEIO;

	return 0;
}

static int max2163_get_frequency(struct dvb_frontend *fe, u32 *freq)
{
	struct max2163_priv *priv = fe->tuner_priv;
	dprintk("%s()\n", __func__);
	*freq = priv->frequency;
	return 0;
}

static int max2163_get_bandwidth(struct dvb_frontend *fe, u32 *bw)
{
	struct max2163_priv *priv = fe->tuner_priv;
	dprintk("%s()\n", __func__);
	*bw = priv->bandwidth;
	return 0;
}

static int max2163_get_status(struct dvb_frontend *fe, u32 *status)
{
//	struct max2163_priv *priv = fe->tuner_priv;
	u16 lock_status = 0;

	dprintk("%s()\n", __func__);

	if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 1);

	*status = lock_status;

	if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 0);

	return 0;
}

static int max2163_sleep(struct dvb_frontend *fe)
{
	dprintk("%s()\n", __func__);
	return 0;
}

static int max2163_init(struct dvb_frontend *fe)
{
	struct max2163_priv *priv = fe->tuner_priv;
	dprintk("%s()\n", __func__);

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	/* Setup initial values */

	/* Initialize the IF Filter Register */
    /* OBS: Datasheet suggests using BIAS_CURRENT_01 upon power-up
            but the observed log uses BIAS_CURRENT_11 */
    max2163_write_reg(priv, IF_FILTER_REG,
                            BANDWIDTH_13MHZ | BIAS_CURRENT_11 | 
                            FLTS_INTERNAL | CENTER_FREQUENCY_1_00);


	/* Initialize the VAS Register */
    max2163_write_reg(priv, VAS_REG,
                            START_AT_CURR_LOADED_REGS | ENABLE_VCO_AUTOSELECT | CPS_AUTOMATIC |
                            DISABLE_ADC_LATCH | ENABLE_ADC_READ | AUTOSELECT_45056_WAIT_TIME);

	/* Initialize the VCO Register */
    max2163_write_reg(priv, VCO_REG,
                            VCO_1 | SUB_BAND_4 | VCOB_LOW_POWER);

	/* Initialize the PDET/RF-FILT Register */
    max2163_write_reg(priv, RF_FILTER_REG,
                            UHF_RANGE_710_806MHZ | PWRDET_BUF_ON_GC1);

	/* Initialize the MODE Register */
    max2163_write_reg(priv, MODE_REG,
                            HIGH_SIDE_INJECTION | ENABLE_RF_FILTER |
                            ENABLE_3RD_STAGE_RFVGA);

	/* Initialize the R-Divider MSB Register */
    max2163_write_reg(priv, RDIVIDER_MSB_REG,
                            PLL_MOST_RDIVIDER(DEFAULT_RDIVIDER));
	
	/* Initialize the R-Divider LSB/CP Register */
    max2163_write_reg(priv, RDIVIDER_LSB_REG,
                            PLL_LEAST_RDIVIDER(DEFAULT_RDIVIDER) | RFDA_37DB |
                            ENABLE_RF_DETECTOR | CHARGE_PUMP_1_5MA);

	/* Initialize the N-Divider MSB Register */
    max2163_write_reg(priv, NDIVIDER_MSB_REG,
                            PLL_MOST_NDIVIDER(DEFAULT_NDIVIDER));
	
	/* Initialize the N-Divider LSB/LIN Register */
    max2163_write_reg(priv, NDIVIDER_LSB_REG,
                            STBY_NORMAL | RFVGA_NORMAL |
                            MIX_NORMAL | PLL_LEAST_NDIVIDER(DEFAULT_NDIVIDER));

	/* Initialize non-documented registers */
    max2163_write_reg(priv, RESERVED_0B_REG, 0x00);
    max2163_write_reg(priv, RESERVED_0C_REG, 0x00);
    max2163_write_reg(priv, RESERVED_0D_REG, 0x00);
    max2163_write_reg(priv, RESERVED_0E_REG, 0x00);
    max2163_write_reg(priv, RESERVED_0F_REG, 0x00);
    max2163_write_reg(priv, RESERVED_10_REG, 0x00);
    max2163_write_reg(priv, RESERVED_11_REG, 0x00);

	if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 0);

	return 0;
}

static int max2163_release(struct dvb_frontend *fe)
{
	struct max2163_priv *priv = fe->tuner_priv;
	dprintk("%s()\n", __func__);

	kfree(priv);
	fe->tuner_priv = NULL;

	return 0;
}

static const struct dvb_tuner_ops max2163_tuner_ops = {
    .info = {
        .name           = "Maxim MAX2163",
        .frequency_min  = 470000000,
        .frequency_max  = 806000000,
        .frequency_step =     50000,
    },

    .release	   = max2163_release,
    .init		   = max2163_init,
    .sleep		   = max2163_sleep,

    .set_params	   = max2163_set_params,
    .set_analog_params = NULL,
    .get_frequency = max2163_get_frequency,
    .get_bandwidth = max2163_get_bandwidth,
    .get_status    = max2163_get_status
};

struct dvb_frontend *max2163_attach(struct dvb_frontend *fe,
				   struct i2c_adapter *i2c,
				   struct max2163_config *cfg)
{
	struct max2163_priv *priv = NULL;

	dprintk("%s(%d-%04x)\n", __func__,
		i2c ? i2c_adapter_id(i2c) : -1,
		cfg ? cfg->i2c_address : -1);

	priv = kzalloc(sizeof(struct max2163_priv), GFP_KERNEL);
	if (priv == NULL)
		return NULL;

	memcpy(&fe->ops.tuner_ops, &max2163_tuner_ops,
		sizeof(struct dvb_tuner_ops));

	priv->config = cfg;
	priv->i2c = i2c;
	fe->tuner_priv = priv;

	max2163_init(fe);

	return fe;
}
EXPORT_SYMBOL(max2163_attach);

MODULE_AUTHOR("Felipe C. da S. Sanches <juca@members.fsf.org>");
MODULE_DESCRIPTION("Maxim MAX2163 silicon tuner driver");
MODULE_LICENSE("GPL");
