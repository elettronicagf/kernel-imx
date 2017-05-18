/*
 * SX8652 based touchscreen and sensor driver
 *
 *  Copyright (c) 2010 Wayne Roberts
 *
 * Using code from:
 *  Copyright (c) 2005 David Brownell
 *  Copyright (c) 2006 Nokia Corporation
 *  Various changes: Imre Deak <imre.deak@nokia.com>
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
*	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/spi/sx8652.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>


#define	 MAX_12BIT	((1<<12)-1)

#define SX8652_TS_PENUP_TIME	100

/* analog channels */
#define CH_X	0
#define CH_Y	1
#define CH_Z1	2
#define CH_Z2	3
#define CH_AUX	4
#define CH_SEQ	7

/* commands */
#define SX8652_CMD_WRITEREG		0x00
#define SX8652_CMD_READCHAN		0x20
#define SX8652_CMD_READREG		0x40
#define SX8652_CMD_SELECT		0x80
#define SX8652_CMD_CONVERT		0x90
#define SX8652_CMD_MANAUTO		0xb0
#define SX8652_CMD_PENDET		0xc0
#define SX8652_CMD_PENTRG		0xe0

/* register addresses */
#define SX8652_REG_CTRL0	0x00
#define SX8652_REG_CTRL1	0x01
#define SX8652_REG_CTRL2	0x02
#define SX8652_REG_CHANMSK	0x04
#define SX8652_REG_STATUS	0x05
#define SX8652_REG_RESET	0x1f

/* RATES */
#define RATE_20_CPS 	0x20

/* for POWDLY or SETDLY: */
#define DLY_0_5US	0x00
#define DLY_1_1US	0x01
#define DLY_2_2US	0x02
#define DLY_4_4US	0x03
#define DLY_9US		0x04
#define DLY_18US	0x05
#define DLY_35uS	0x06
#define DLY_71US	0x07
#define DLY_140US	0x08
#define DLY_280US	0x09
#define DLY_570US	0x0a
#define DLY_1_1MS	0x0b
#define DLY_2_3MS	0x0c
#define DLY_4_5MS	0x0d
#define DLY_9MS		0x0e
#define DLY_18MS	0x0f

// RegCtrl1
#define CONDIRQ 	0x20
#define FILT_NONE	0x00
#define FILT_3SA	0x01
#define FILT_5SA	0x02
#define FILT_7SA	0x03
#define SCREEN_5WIRE	0x10

#define RPDNT_200K	0x04

#define CONV_X		0x80
#define CONV_Y		0x40
#define CONV_Z1		0x20
#define CONV_Z2		0x10
#define CONV_AUX	0x08

#define RSTEVENT    (1<<5)

#define CHAN_MASK_4WIRE		(CONV_X | CONV_Y | CONV_Z1 | CONV_Z2)
#define CHAN_MASK_5WIRE		(CONV_X | CONV_Y)


#define RESET_VALUE			0xde

#define NUM_MAX_READ_REGS		4	/* count of words to read */
#define NUM_READ_REGS_4WIRE		4	/* count of words to read */
#define NUM_READ_REGS_5WIRE		2	/* count of words to read */

//barrel correction calibration data for FAE touch
#define KX		((s64)-15)//-0.0000000150f
#define KY		((s64)-35)//-0.0000000350f
#define SCALEX64	((s64)(1<<30))
#define SCALEY64	((s64)(1<<30))

//touch center
s64 XC = 2008;
s64 YC = 2002;


static void sx8652_parse_data(void *ads);

struct sx8652 {
	struct input_dev	*input;
	char			phys[32];
	struct spi_device	*spi;
	spinlock_t		lock;
	struct mutex	mutex;
	unsigned		disabled:1;
	struct timer_list	penup_timer;
	u16			model;
	u16			y_plate_ohms;
	u16			pressure_max;

	u8 pen_down;
	u8 ts_wires;

	int 	last_read_x;
	int 	last_read_y;
	int		read_rep;
	int 	read_cnt;

	u16			debounce_max;
	u16			debounce_tol;
	u16			debounce_rep;

	int			(*filter)(void *data, int *val_x, int *val_y);
	void			*filter_data;
	void			(*filter_cleanup)(void *data);

#if defined(CONFIG_HWMON) || defined(CONFIG_HWMON_MODULE)
	struct attribute_group	*attr_group;
	struct device		*hwmon;
#endif


	struct spi_message	 read_msg;
	struct spi_transfer read_xfer;
	u8 read_cmd;
	u8 data[(NUM_MAX_READ_REGS << 1) + 1];

	int			(*get_pendown_state)(void);
	void			(*wait_for_sync)(void);
	int			gpio_pendown;
	int			gpio_reset;

	int 		correction_enabled;
	struct     task_struct *task_restore;
	struct		mutex mutex_restore;
};

static ssize_t sx8652_enable_correction_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sx8652 *ts = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%u\n", ts->correction_enabled);
}

static ssize_t sx8652_enable_correction_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct sx8652 *ts = dev_get_drvdata(dev);

	if(*buf=='1')
		ts->correction_enabled = 1;
	else if(*buf=='0')
		ts->correction_enabled = 0;
	else
		return -EINVAL;

	dev_info(dev, "enabled correction: %d\n", ts->correction_enabled );

	return count;
}

static ssize_t rawcalibration_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int dummy, xmin, xmax, ymin, ymax;

	sscanf(buf, "%d %d %d %d %d", &dummy, &xmin, &xmax, &ymin, &ymax);

	XC = (xmax-xmin)/2+xmin;
	YC = (ymax-ymin)/2+ymin;

	dev_info(dev, "set touch center (%d,%d)\n", (int)XC, (int)YC );

	return count;
}

static DEVICE_ATTR(enable_correction, S_IRUGO | S_IWUSR, sx8652_enable_correction_show, sx8652_enable_correction_store);
static DEVICE_ATTR(rawcalibration, S_IWUSR, NULL, rawcalibration_store);

static struct attribute *sx8652_attributes[] = {
	&dev_attr_enable_correction.attr,
	&dev_attr_rawcalibration.attr,
	NULL,
};

static struct attribute_group sx8652_attr_group = {
	.attrs = sx8652_attributes,
};

static int sx8652_setup(struct sx8652 *ts, int checkMode);

static int get_pendown_state(struct sx8652 *ts)
{
	if (ts->get_pendown_state)
		return ts->get_pendown_state();

	return !gpio_get_value(ts->gpio_pendown);
}

static int sx8652_readreg(struct spi_device *spi, u8 reg, u8* value)
{
	int ret = -1;

	ret = spi_w8r8(spi, SX8652_CMD_READREG | (reg & 0x1f));

	dev_dbg(&spi->dev, "SX8652 sx8652_readreg [0x%02X]=0x%02X", reg, ret);

	if(ret>=0)
	{
		*value = (u8)ret;
		return 1;
	}

	return 0;
}

static int sx8652_writereg(struct spi_device *spi, u8 reg, u8 value)
{
	int err = -1;
	u8 txbuf[2];

	txbuf[0] = SX8652_CMD_WRITEREG | (reg & 0x1f);
	txbuf[1] = value;

	err = spi_write(spi, txbuf, 2*sizeof(u8));

	dev_dbg(&spi->dev, "SX8652 sx8652_writereg [0x%02X]=0x%02X", txbuf[0], value);

	return err>=0;
}

static int WriteSetting(struct spi_device *spi, u8 reg, u8 regValue, int checkMode, char* regName)
{
	u8 currentValue;
	int doWrite = 1;

	dev_dbg(&spi->dev, "WriteSetting: %s = 0x%02X", regName, regValue);

	if(checkMode)
	{
		if(sx8652_readreg(spi, reg, &currentValue)==1)
		{
			doWrite = (currentValue != regValue);
			if(doWrite)
				dev_err(&spi->dev, "check %s register current=%x value=%x...", regName, currentValue, regValue);
			else
				dev_dbg(&spi->dev, "OK");
		}
		else
		{
			dev_err(&spi->dev, "read %s register fail", regName);
			return 0;
		}
	}

	if(doWrite)
		if (!sx8652_writereg(spi, reg, regValue )) {
			dev_err(&spi->dev, "set %s register fail", regName);
			return 0;
		}

	udelay(1000);

	if(doWrite && checkMode)
	{
		if(sx8652_readreg(spi, reg, &currentValue)==1)
		{
			if(currentValue != regValue)
				dev_err(&spi->dev, "restore failed");
			else
				dev_err(&spi->dev, "restore OK");

			return 1;
		}

		dev_err(&spi->dev, "check %s register fail", regName);
		return 0;
	}


	return 1;
}


int restore_function(void *data)
{
	u8 regValue;
	int nTry = 1;

	struct sx8652 *ts = (struct sx8652 *)data;

	while(!kthread_should_stop())
	{
		msleep(5000);

		mutex_lock(&ts->mutex_restore);

		while(nTry<=3)
		{
			//check ESD event
			if(sx8652_readreg(ts->spi, SX8652_REG_STATUS, &regValue)==1)
			{
				nTry=1;
				dev_dbg(&ts->spi->dev, "REG_STAT=0x%X!\r\n",regValue);

				if(regValue & RSTEVENT)
				{
					dev_err(&ts->spi->dev, "reset detected, restore settings...");

					//restore all values
					if(sx8652_setup(ts,1))
						dev_err(&ts->spi->dev, "OK");
				}
				else //check settings anyway
					sx8652_setup(ts,1);

				break;
			}
			else
			{	//sx8652 non risponde -> reset hw
				dev_err(&ts->spi->dev, "timeout, trying hw reset...");
				//reset off
				if(gpio_is_valid(ts->gpio_reset)){
					gpio_set_value(ts->gpio_reset,0);
					mdelay(5);
					gpio_set_value(ts->gpio_reset,1);
					mdelay(10);
				}

				//restore settings
				if(sx8652_setup(ts,0))
					dev_err(&ts->spi->dev, "OK");

				dev_err(&ts->spi->dev, "CheckHardwareSettings try %d",nTry);
				nTry++;
			} //if readreg
		} //while ntry

		mutex_unlock(&ts->mutex_restore);
	} //kthread stop

	//non deve arrivare mai qui!!
	dev_err(&ts->spi->dev, "exiting restore_function!!");
	return 0;
}

static void sx8652_ts_penup_timer_handler(unsigned long data)
{
    struct sx8652 *ts = (struct sx8652 *)data;
	input_report_abs(ts->input, ABS_PRESSURE, 0);
	input_report_key(ts->input, BTN_TOUCH, 0);
	input_sync(ts->input);

	ts->pen_down = 0;
	ts->read_cnt = 0;
	ts->read_rep = 0;
}


static int setup_pendown(struct spi_device *spi, struct sx8652 *ts, const struct sx8652_platform_data *pdata)
{
	int err;

	if (pdata->get_pendown_state) {
		ts->get_pendown_state = pdata->get_pendown_state;
	} else if (gpio_is_valid(pdata->gpio_pendown)) {
		err = gpio_request_one(pdata->gpio_pendown, GPIOF_IN,
				       "sx8652_pendown");
		if (err) {
			dev_err(&spi->dev,
				"failed to request/setup pendown GPIO%d: %d\n",
				pdata->gpio_pendown, err);
			return err;
		}
		ts->gpio_pendown = pdata->gpio_pendown;

	} else {
		dev_err(&spi->dev, "no get_pendown_state nor gpio_pendown?\n");
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t sx8652_irq(int irq, void *handle)
{
	struct sx8652 *ts = handle;
	int status;

	/* If insufficient pullup resistor on nIRQ line:
	 * may need to make sure that pen is really down here, due to spurious interrupts  */
	if (likely(get_pendown_state(ts))) {
		mutex_lock(&ts->mutex_restore);
		status = spi_sync(ts->spi, &ts->read_msg);
		if (status)
			dev_err(&ts->spi->dev, "spi_async --> %d\n", status);
		else
			sx8652_parse_data(handle);

		/* kick pen up timer */
		mod_timer(&ts->penup_timer,
			jiffies + msecs_to_jiffies(SX8652_TS_PENUP_TIME));

		mutex_unlock(&ts->mutex_restore);
	}

	return IRQ_HANDLED;
}


static void null_wait_for_sync(void)
{
}

static int sx8652_debounce_filter(void *handle, int *val_x, int *val_y)
{
	struct sx8652 *ts = handle;

	if (!ts->read_cnt || (abs(ts->last_read_x - *val_x) > ts->debounce_tol) ||
		(abs(ts->last_read_y - *val_y) > ts->debounce_tol)) {
		/* Start over collecting consistent readings. */
		ts->read_rep = 0;
		/*
		 * Repeat it, if this was the first read or the read
		 * wasn't consistent enough.
		 */
		if (ts->read_cnt < ts->debounce_max) {
			ts->last_read_x = *val_x;
			ts->last_read_y = *val_y;
			ts->read_cnt++;
			return SX8652_FILTER_REPEAT;
		} else {
			/*
			 * Maximum number of debouncing reached and still
			 * not enough number of consistent readings. Abort
			 * the whole sample, repeat it in the next sampling
			 * period.
			 */
			ts->read_cnt = 0;
			return SX8652_FILTER_IGNORE;
		}
	} else {
		if (++ts->read_rep > ts->debounce_rep) {
			/*
			 * Got a good reading for this coordinate,
			 * go for the next one.
			 */
			ts->read_cnt = 0;
			ts->read_rep = 0;
			return SX8652_FILTER_OK;
		} else {
			/* Read more values that are consistent. */
			ts->read_cnt++;
			return SX8652_FILTER_REPEAT;
		}
	}
}

static void sx8652_parse_data(void *ads)
{
	struct sx8652 *ts = ads;
	int nreg;
	u16 *data_ptr;
	u32 rt = 0;
	int i;
	int x = 0, y = 0, z1 = 0, z2 = 0;
	u16 data;
	u8 ch;
	int ret;

	// first byte is command of readchan
	data_ptr = (u16 *)&ts->data[1];

	if(ts->ts_wires == 5)
		nreg = NUM_READ_REGS_5WIRE;
	else
		nreg = NUM_READ_REGS_4WIRE;

	for (i = 0; i < nreg; i++) {
		data = swab16(data_ptr[i]);
		ch = data >> 12;
		switch (ch) {
			case CH_X:
				x = data & 0xfff;
				break;
			case CH_Y:
				y = data & 0xfff;
				break;
			case CH_Z1:
				z1 = data & 0xfff;
				break;
			case CH_Z2:
				z2 = data & 0xfff;
				break;
			default:
				break;
		}
	}

	if(ts->ts_wires == 4) {
		if (likely(y && z1)) {
			rt = z2;
			rt -= z1;
			rt *= y;
			rt *= ts->y_plate_ohms;
			rt /= z1;
			rt = (rt + 2047) >> 12;
		}


		if (rt > ts->pressure_max) {
			return;
		}
	} else {
		rt = 1;
	}

	if(rt)	{
		//printk(KERN_INFO "point %4d %4d read_cnt %d read_rep %d %d %d\n", x, y, ts->read_cnt, ts->read_rep, ts->last_read_x, ts->last_read_y);
		ret = ts->filter(ts->filter_data, &x, &y);
		switch (ret) {
		case SX8652_FILTER_REPEAT:
		case SX8652_FILTER_IGNORE:
			return;
		case SX8652_FILTER_OK:
			if (!ts->pen_down) {
				input_report_key(ts->input, BTN_TOUCH, 1);
				ts->pen_down = 1;
			}

			input_report_abs(ts->input, ABS_X, x);
			input_report_abs(ts->input, ABS_Y, y);
			input_report_abs(ts->input, ABS_PRESSURE, ts->pressure_max - rt);
			input_sync(ts->input);

			//printk(KERN_INFO "OKKKKKKKKKKKKKKKKK point(%4d,%4d), pressure (%4u)\n",
			//		x, y, rt);
			break;

		default:
			BUG();
		}

	}
}

#ifdef CONFIG_OF
static const struct of_device_id sx8652_dt_ids[] = {
	{ .compatible = "semtech,sx8652",	.data = (void *) 8652 },
	{ }
};
MODULE_DEVICE_TABLE(of, sx8652_dt_ids);

static const struct sx8652_platform_data *sx8652_probe_dt(struct device *dev)
{
	struct sx8652_platform_data *pdata;
	struct device_node *node = dev->of_node;
	const struct of_device_id *match;

	if (!node) {
		dev_err(dev, "Device does not have associated DT data\n");
		return ERR_PTR(-EINVAL);
	}

	match = of_match_device(sx8652_dt_ids, dev);
	if (!match) {
		dev_err(dev, "Unknown device model\n");
		return ERR_PTR(-EINVAL);
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->model = (unsigned long)match->data;

	of_property_read_u16(node, "x-plate-ohms", &pdata->x_plate_ohms);
	of_property_read_u16(node, "y-plate-ohms", &pdata->y_plate_ohms);

	of_property_read_u16(node, "x-min", &pdata->x_min);
	of_property_read_u16(node, "y-min", &pdata->y_min);
	of_property_read_u16(node, "x-max", &pdata->x_max);
	of_property_read_u16(node, "y-max", &pdata->y_max);

	of_property_read_u16(node, "pressure-min", &pdata->pressure_min);
	of_property_read_u16(node, "pressure-max", &pdata->pressure_max);

	of_property_read_u16(node, "debounce-max", &pdata->debounce_max);
	of_property_read_u16(node, "debounce-tol", &pdata->debounce_tol);
	of_property_read_u16(node, "debounce-rep", &pdata->debounce_rep);

	of_property_read_u8(node, "ts-wires", &pdata->ts_wires);

	if ((pdata->ts_wires != 4) && (pdata->ts_wires != 5)){
		dev_err(dev, "Property error: ts-wires shall be 4 or 5\n");
		return ERR_PTR(-EINVAL);
	}

	pdata->gpio_pendown = of_get_named_gpio(dev->of_node, "pendown-gpio", 0);

	pdata->gpio_reset = of_get_named_gpio(dev->of_node, "reset-gpio", 0);

	return pdata;
}
#else
static const struct sx8652_platform_data *sx8652_probe_dt(struct device *dev)
{
	dev_err(dev, "no platform data defined\n");
	return ERR_PTR(-EINVAL);
}
#endif

static int sx8652_setup(struct sx8652 *ts, int checkMode)
{
	u8 val;
	u8 txbuf;
	int err;
	struct spi_device* spi = ts->spi;

	if(!WriteSetting(spi, SX8652_REG_CTRL0, DLY_280US, checkMode, "REG_CTRL0"))
		return 0;

	val = RPDNT_200K;
	if(ts->ts_wires == 5)
		val |= SCREEN_5WIRE | FILT_3SA;
	else
		val |= FILT_5SA;
	if(!WriteSetting(spi, SX8652_REG_CTRL1, val, checkMode, "REG_CTRL1"))
		return 0;

	if(!WriteSetting(spi, SX8652_REG_CTRL2, DLY_9US, checkMode, "REG_CTRL2"))
		return 0;

	if(ts->ts_wires == 5)
		val = CHAN_MASK_5WIRE;
	else
		val = CHAN_MASK_4WIRE;
	if(!WriteSetting(spi, SX8652_REG_CHANMSK, val, checkMode, "CHANMSK"))
		return 0;

	txbuf = SX8652_CMD_PENTRG;
	err = spi_write(spi, &txbuf, 1 * sizeof(u8));
	if(err<0)
	{
		dev_err(&spi->dev, "pentrg fail\n");
		return 0;
	}

	return 1;
}

static int sx8652_probe(struct spi_device *spi)
{
	struct sx8652 *ts;
	struct input_dev *input_dev;
	const struct sx8652_platform_data	*pdata;
	int err = -1;
	struct spi_message		*m;
	struct spi_transfer		*x;
	u8 txbuf[2];
	int error;

	pdata = dev_get_platdata(&spi->dev);
	if (!pdata){
		pdata = sx8652_probe_dt(&spi->dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	/* don't exceed max specified SCLK frequency */
	if (spi->max_speed_hz > 5000000) {
		dev_dbg(&spi->dev, "SCLK %d KHz?\n", spi->max_speed_hz/1000);
		return -EINVAL;
	}

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	err = spi_setup(spi);
	if (err < 0)
		return err;

	ts = kzalloc(sizeof(struct sx8652), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	dev_set_drvdata(&spi->dev, ts);

	ts->spi = spi;
	ts->input = input_dev;

	if(gpio_is_valid(pdata->gpio_reset)){
		gpio_request_one(pdata->gpio_reset, 0, "sx8652_reset");
		gpio_set_value(pdata->gpio_reset,0);
		udelay(150);
		gpio_set_value(pdata->gpio_reset,1);
	}

	/* Send a software reset command */
	txbuf[0] = SX8652_CMD_WRITEREG | SX8652_REG_RESET;
	txbuf[1] = RESET_VALUE;
	dev_dbg(&spi->dev, "resetting SX8652 (0x%02X 0x%02X)\n",
			txbuf[0], txbuf[1]);
	err = spi_write(spi, txbuf, 2 * sizeof(u8));
	if (err < 0)
		goto err_free_mem;
	/* sx8652 nirq is momentarily asserted after software reset */
	udelay(150);

    init_timer(&ts->penup_timer);
    setup_timer(&ts->penup_timer, sx8652_ts_penup_timer_handler,
            (unsigned long)ts);

	spin_lock_init(&ts->lock);
	mutex_init(&ts->mutex);
	mutex_init(&ts->mutex_restore);

	ts->model = pdata->model ? : 8652;
	ts->y_plate_ohms = pdata->y_plate_ohms ? : 400;
	ts->pressure_max = pdata->pressure_max ? : ~0;
	ts->ts_wires = pdata->ts_wires;
	dev_dbg(&spi->dev, "Touch is %d wires\n",ts->ts_wires);

	ts->filter = sx8652_debounce_filter;
	ts->filter_data = ts;
	ts->debounce_max = pdata->debounce_max;
	ts->debounce_tol = pdata->debounce_tol;
	ts->debounce_rep = pdata->debounce_rep;

	err = setup_pendown(spi, ts, pdata);
	if (err)
		goto err_free_mem;

	ts->wait_for_sync = pdata->wait_for_sync ? : null_wait_for_sync;
	//for older kernel: snprintf(ts->phys, sizeof(ts->phys), "%s/input0", spi->dev.bus_id);
	//for 2.6.32:
	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&spi->dev));

	input_dev->name = "SX8652 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->dev.parent = &spi->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X,
			pdata->x_min ? : 0,
			pdata->x_max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			pdata->y_min ? : 0,
			pdata->y_max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			pdata->pressure_min, pdata->pressure_max, 0, 0);

	error = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
					  sx8652_irq,
					  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					  "sx8652_ts", ts);
	if (error) {
		dev_err(&spi->dev, "Failed to register interrupt\n");
		return error;
	}

	dev_info(&spi->dev, "touchscreen, irq %d\n", spi->irq);

	m =	&ts->read_msg;
	x = &ts->read_xfer;

	spi_message_init(m);

	ts->read_cmd = SX8652_CMD_READCHAN;
	x->tx_buf = &ts->read_cmd;
	x->rx_buf = ts->data;
	if(ts->ts_wires == 5)
		x->len = 2 * 2 + 1;
	else
		x->len = 4 * 2 + 1;
	spi_message_add_tail(x, m);

	m->context = ts;

	err = sysfs_create_group(&spi->dev.kobj, &sx8652_attr_group);
	if (err)
		goto err_remove_hwmon;

	err = input_register_device(input_dev);
	if (err)
		goto err_remove_attr_group;

	sx8652_setup(ts,0);

	ts->task_restore = kthread_run(&restore_function,(void *)ts,"sx8652wdg");

	return 0;

 err_remove_attr_group:
	sysfs_remove_group(&spi->dev.kobj, &sx8652_attr_group);
 err_remove_hwmon:
	free_irq(spi->irq, ts);
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}


/* Must be called with ts->lock held */
static void sx8652_disable(struct sx8652 *ts)
{
	if (ts->disabled)
		return;

	disable_irq(ts->spi->irq);
	ts->disabled = 1;

	/* if timer is running, wait for it to finish */
	while (ts->pen_down) {
		msleep(5);
	}

}

static int sx8652_remove(struct spi_device *spi)
{
	struct sx8652		*ts = dev_get_drvdata(&spi->dev);

	kthread_stop(ts->task_restore);

	input_unregister_device(ts->input);

	mutex_lock(&ts->mutex);
	sx8652_disable(ts);
	mutex_unlock(&ts->mutex);

	sysfs_remove_group(&spi->dev.kobj, &sx8652_attr_group);

	free_irq(ts->spi->irq, ts);
	kfree(ts);

	dev_dbg(&spi->dev, "unregistered touchscreen\n");
	return 0;
}

static struct spi_driver sx8652_driver = {
	.driver = {
		.name	= "sx8652",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= sx8652_probe,
	.remove		= sx8652_remove,
};

module_spi_driver(sx8652_driver);

MODULE_DESCRIPTION("SX8652 TouchScreen Driver");
MODULE_LICENSE("GPL");
