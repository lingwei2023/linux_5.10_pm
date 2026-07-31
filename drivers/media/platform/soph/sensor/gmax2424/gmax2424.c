// SPDX-License-Identifier: GPL-2.0
/*
 * gmax2424 driver
 *
 * Copyright (C) 2024 Sophon Co., Ltd.
 *
 */
#include <linux/clk.h>
#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>

#include <linux/comm_cif.h>
#include <linux/sns_v4l2_uapi.h>

#include "gmax2424.h"

/* I2C per write of bits */
#define REG_VALUE_08BIT		1
#define REG_VALUE_16BIT		2
#define REG_VALUE_24BIT		3

/* GMAX2424 has no dedicated chip-id register in this datasheet
 * revision; read a valid reg only to detect I2C presence. */
#define GMAX2424_ID_CHECK_ADDR		0x3002

/*Sensor type for isp middleware*/
#define gmax2424_SNS_TYPE_SDR V4L2_GPIXEL_GMAX2424_MIPI_24M_18FPS_10BIT
// #define gmax2424_SNS_TYPE_WDR V4L2_SMS_gmax2424_MIPI_2M_60FPS_10BIT_WDR2TO1

static const enum mipi_wdr_mode_e gmax2424_wdr_mode = MIPI_WDR_MODE_VC;

volatile int gmax2424_count = 0;
static int force_bus[MAX_SENSOR_DEVICE] = {[0 ... (MAX_SENSOR_DEVICE - 1)] = -1};
module_param_array(force_bus, int, &gmax2424_count, 0644);

/* master(0)/slave(1) select: insmod ... slave_mode=1 for external VSYNC trigger.
 * Only differs by 0x3002: master=0x31(XMASTER=1), slave=0x30(XMASTER=0). */
static int slave_mode = 0;
module_param(slave_mode, int, 0644);
MODULE_PARM_DESC(slave_mode, "0=master(internal,free-run), 1=slave(external VSYNC trigger)");

static int gmax2424_probe_index;
static const unsigned short gmax2424_i2c_list[] = {0x10, 0x1A};
static const int gmax2424_bus_map[MAX_SENSOR_DEVICE] = {3, -1, -1, -1, -1, -1};

struct gmax2424_reg_list {
	u32 num_of_regs;
	const struct gmax2424_reg *regs;
};

/* Mode : resolution and related config&values */
struct gmax2424_mode {
	u32 max_width;
	u32 max_height;
	u32 width;
	u32 height;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 mipi_wdr_mode;
	u32 sns_type;
	char *sns_type_name;
	struct v4l2_fract max_fps;
	struct v4l2_fract wdr_max_fps;
	sns_sync_info_t gmax2424_sync_info;
	struct gmax2424_reg_list reg_list;
	struct gmax2424_reg_list wdr_reg_list;
};

/* Mode configs */
static struct gmax2424_mode supported_modes[] = {
	{
		.max_width  = 6144,	/* sensor outputs 6144 (6016 effective + 128 dummy); crop in ISP */
		.max_height = 4096,
		.width  = 6144,
		.height = 4096,
		.exp_def = 2000,
		.hts_def = 815,	/* MIPI 10bit min line time */
		.vts_def = 4106,	/* frame length; matches register VMAX=0x100A */
		.sns_type = V4L2_GPIXEL_GMAX2424_MIPI_24M_18FPS_10BIT,
		.sns_type_name  = "V4L2_GPIXEL_GMAX2424_MIPI_24M_18FPS_10BIT",
		.max_fps = {
			.numerator = 1,
			.denominator = 18,	/* 18fps, matches cmos_param.h f32MaxFps */
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_24m17_4l_regs),
			.regs = mode_24m17_4l_regs,
		},
	},
};

struct gmax2424 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct i2c_client *client;
	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	/* Current mode */
	struct gmax2424_mode *cur_mode;
	/* Mutex for serialized access */
	struct mutex mutex;
	/* Streaming on/off */
	bool streaming;
	/*dtsi config*/
	struct clk       *xvclk;
	struct gpio_desc *power_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	struct pinctrl   *pinctrl;

	unsigned int lane_num;
	unsigned int module_index;
};

#define to_gmax2424(_sd)	container_of(_sd, struct gmax2424, sd)

/* Read registers up to 4 at a time */
static int gmax2424_read_reg(struct gmax2424 *gmax2424, u16 reg, u32 len,
			    u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gmax2424->sd);
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	int ret;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);

	if (len > 4)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

/* Write registers up to 4 at a time */
static int gmax2424_write_reg(struct gmax2424 *gmax2424, u16 reg, u32 len,
			     u32 __val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gmax2424->sd);
	int buf_i, val_i;
	u8 buf[6], *val_p;
	__be32 val;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val = cpu_to_be32(__val);
	val_p = (u8 *)&val;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	usleep_range(100, 200);

	return 0;
}

/* Write a list of registers */
static int gmax2424_write_regs(struct gmax2424 *gmax2424,
			      const struct gmax2424_reg *regs, u32 len)
{
	printk("len = %d\n", len);
	struct i2c_client *client = v4l2_get_subdevdata(&gmax2424->sd);
	int ret;
	u32 i;

	for (i = 0; i < len; i++) {
		ret = gmax2424_write_reg(gmax2424, regs[i].address, 1,
					regs[i].val);
		// printk("!!!!!! write reg 0x%4.4x. value = %d, rror=%d\n",
		// 			    regs[i].address, regs[i].val, ret);
		if (ret) {
			dev_err_ratelimited(&client->dev, "Failed to write reg 0x%4.4x. value = %d, rror=%d\n",
					    regs[i].address, regs[i].val, ret);

			return ret;
		}
	}

	return 0;
}

/* Open sub-device */
static int gmax2424_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gmax2424 *gmax2424 = to_gmax2424(sd);
	struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(sd, fh->pad, 0);

	mutex_lock(&gmax2424->mutex);

	/* Initialize try_fmt */
	try_fmt->width = gmax2424->cur_mode->width;
	try_fmt->height = gmax2424->cur_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	/* No crop or compose */
	mutex_unlock(&gmax2424->mutex);

	return 0;
}

static int gmax2424_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gmax2424 *gmax2424 = container_of(ctrl->handler,
					       struct gmax2424, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&gmax2424->sd);

	pm_runtime_put(&client->dev);

	return 0;
}

static const struct v4l2_ctrl_ops gmax2424_ctrl_ops = {
	.s_ctrl = gmax2424_set_ctrl,
};

static int g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
			 struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_CSI2_DPHY;
	return 0;
}

static int enum_mbus_code(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_mbus_code_enum *code)
{
	/* Only one bayer order(GRBG) is supported */
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;

	return 0;
}

static int enum_frame_interval(struct v4l2_subdev *sd,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct gmax2424 *gmax2424 = to_gmax2424(sd);

	fie->width  = gmax2424->cur_mode->width;
	fie->height = gmax2424->cur_mode->height;

	fie->interval.numerator   = gmax2424->cur_mode->max_fps.numerator;
	fie->interval.denominator = gmax2424->cur_mode->max_fps.denominator;

	return 0;
}

static int enum_frame_size(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_frame_size_enum *fse)
{
	struct gmax2424 *gmax2424 = to_gmax2424(sd);

	fse->min_width = gmax2424->cur_mode->width;
	fse->max_width = gmax2424->cur_mode->max_width;
	fse->min_height = gmax2424->cur_mode->height;
	fse->max_height = gmax2424->cur_mode->max_height;

	return 0;
}

static void update_pad_format(const struct gmax2424_mode *mode, struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int get_pad_format(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gmax2424 *gmax2424 = to_gmax2424(sd);
	struct v4l2_mbus_framefmt *framefmt;
	int ret = 0;

	mutex_lock(&gmax2424->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *framefmt;
		ret = -ENOTTY;
	} else {
		update_pad_format(gmax2424->cur_mode, fmt);
	}
	mutex_unlock(&gmax2424->mutex);

	return ret;
}

static int set_pad_format(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gmax2424 *gmax2424 = to_gmax2424(sd);
	struct gmax2424_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&gmax2424->mutex);

	/* Only one raw bayer(GRBG) order is supported */
	if (fmt->format.code != MEDIA_BUS_FMT_SRGGB10_1X10)
		fmt->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);
	update_pad_format(mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*framefmt = fmt->format;
	} else {
		gmax2424->cur_mode = mode;
	}

	mutex_unlock(&gmax2424->mutex);

	return 0;
}

static void gmax2424_standby(struct gmax2424 *gmax2424)
{
	gmax2424_write_reg(gmax2424, 0x3000, REG_VALUE_08BIT, 0x00);
}

static void gmax2424_restart(struct gmax2424 *gmax2424)
{
	gmax2424_write_reg(gmax2424, 0x3000, REG_VALUE_08BIT, 0x00);
	usleep_range(20000, 20000);
	gmax2424_write_reg(gmax2424, 0x3000, REG_VALUE_08BIT, 0x01);
}

/* Start streaming */
static int start_streaming(struct gmax2424 *gmax2424)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gmax2424->sd);
	const struct gmax2424_reg_list *reg_list;
	const sns_sync_info_t *sync_info;
	int ret;

	reg_list = &gmax2424->cur_mode->reg_list;

	printk("reg_list->num_of_regs = %d\n", reg_list->num_of_regs);

	ret = gmax2424_write_regs(gmax2424, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}
	printk("gmax2424_write_regs success!\n");

	if (slave_mode) {
		gmax2424_write_reg(gmax2424, 0x3002, REG_VALUE_08BIT, 0x30); /* XMASTER=0 slave */
		dev_info(&client->dev, "gmax2424: SLAVE mode (0x3002=0x30), waits external VSYNC");
	} else {
		dev_info(&client->dev, "gmax2424: MASTER mode (0x3002=0x31), free-run");
	}

	/* Power-on / stream-on sequence (from regmap power-on tail).
	 * This 0x3C dither block is REQUIRED to bring MIPI clock lane up
	 * (verified: removing it => CK_HS=0). MIPI 4-lane uses 0x3C0B=0x3F. */
	usleep_range(5000, 6000);
	gmax2424_write_reg(gmax2424, 0x3000, REG_VALUE_08BIT, 0x01);	/* STREAM_EN */
	usleep_range(5000, 6000);
	gmax2424_write_reg(gmax2424, 0x3005, REG_VALUE_08BIT, 0x01);	/* CLK_EN */
	usleep_range(80000, 82000);
	gmax2424_write_reg(gmax2424, 0x4C00, REG_VALUE_08BIT, 0x01);	/* PWR_UP_DOWN_CTR */
	usleep_range(1000, 1500);
	gmax2424_write_reg(gmax2424, 0x3C24, REG_VALUE_08BIT, 0x00);
	usleep_range(1000, 1500);
	gmax2424_write_reg(gmax2424, 0x3C0B, REG_VALUE_08BIT, 0x3F);
	usleep_range(1000, 1500);
	gmax2424_write_reg(gmax2424, 0x3C23, REG_VALUE_08BIT, 0x06);
	usleep_range(1000, 1500);
	gmax2424_write_reg(gmax2424, 0x3C23, REG_VALUE_08BIT, 0x07);
	usleep_range(1000, 1500);
	gmax2424_write_reg(gmax2424, 0x3C24, REG_VALUE_08BIT, 0x02);
	usleep_range(1000, 1500);
	gmax2424_write_reg(gmax2424, 0x3C23, REG_VALUE_08BIT, 0x06);

	/* DEBUG test pattern: uncomment to output internal vertical color bar
	 * (TEST_IMG_EN bit[2:1]=2 -> 0x10|0x04=0x14). If test image streams
	 * continuously but normal capture does not, fault is in readout/exposure. */
	// gmax2424_write_reg(gmax2424, 0x3400, REG_VALUE_08BIT, 0x14); /* test pattern */
	// gmax2424_write_reg(gmax2424, 0x3800, REG_VALUE_08BIT, 0x14); /* test pattern */

	sync_info = &gmax2424->cur_mode->gmax2424_sync_info;
	printk("sync_info->num_of_regs = %d\n", sync_info->num_of_regs);

	if (sync_info->num_of_regs > 0) {
		ret = gmax2424_write_regs(gmax2424, (struct gmax2424_reg *)sync_info->regs,
					sync_info->num_of_regs);
		if (ret) {
			dev_err(&client->dev, "%s failed to set default\n", __func__);
			return ret;
		}
	}
	printk("sync_info gmax2424_write_regs success!\n");

	usleep_range(100 * 1000, 200 * 1000);
	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(gmax2424->sd.ctrl_handler);
	if (ret)
		return ret;

	dev_info(&client->dev, "wdr_mode(%d) reg setting done\n", gmax2424->cur_mode->mipi_wdr_mode);

	return ret;
}

/* Stop streaming */
static int stop_streaming(struct gmax2424 *gmax2424)
{
	gmax2424_standby(gmax2424);
	return 0;
}

static int set_stream(struct v4l2_subdev *sd, int enable)
{
	struct gmax2424 *gmax2424 = to_gmax2424(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&gmax2424->mutex);
	if (gmax2424->streaming == enable) {
		mutex_unlock(&gmax2424->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		//reset sensor
		if (!IS_ERR(gmax2424->reset_gpio)) {
			gpiod_set_value_cansleep(gmax2424->reset_gpio, 0);
			msleep(100);
			gpiod_set_value_cansleep(gmax2424->reset_gpio, 1);
			msleep(100);
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = start_streaming(gmax2424);
		if (ret)
			goto err_rpm_put;
	} else {
		stop_streaming(gmax2424);
		pm_runtime_put(&client->dev);
	}

	gmax2424->streaming = enable;
	mutex_unlock(&gmax2424->mutex);

	dev_info(&client->dev, "set stream(%d) success\n", enable);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&gmax2424->mutex);

	return ret;
}

static int __maybe_unused suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gmax2424 *gmax2424 = to_gmax2424(sd);

	if (gmax2424->streaming)
		stop_streaming(gmax2424);

	return 0;
}

static int __maybe_unused resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gmax2424 *gmax2424 = to_gmax2424(sd);
	int ret;

	if (gmax2424->streaming) {
		ret = start_streaming(gmax2424);
		if (ret)
			goto error;
	}

	return 0;

error:
	stop_streaming(gmax2424);
	gmax2424->streaming = false;
	return ret;
}

/* Detect device presence (GMAX2424 has no chip-id reg here) */
static int gmax2424_identify_module(struct gmax2424 *gmax2424)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gmax2424->sd);
	int ret;
	u32 val = 0;

	/* Put in standby, then read a valid register. A failed I2C
	 * transfer (no ACK) means no sensor at this address, so probe
	 * moves on to the next candidate address. */
	gmax2424_write_reg(gmax2424, 0x3000, REG_VALUE_08BIT, 0x00);/* STANDBY */
	usleep_range(20 * 1000, 20 * 1000);

	ret = gmax2424_read_reg(gmax2424, GMAX2424_ID_CHECK_ADDR,
				REG_VALUE_08BIT, &val);
	dev_info(&client->dev, "probe read 0x%04x=0x%02x ret:%d", GMAX2424_ID_CHECK_ADDR, val, ret);
	return ret;
}

static void gmax2424_mirror_flip(struct gmax2424 *gmax2424, int orient)
{
	/* Horizontal flip: FLIP_H_ODD=0x3400 bit0, FLIP_H_EVEN=0x3800 bit0.
	 * NOTE: full HW mirror also needs post-processor channel reversal
	 * (datasheet "Image Flipping"). Vertical flip: TODO per datasheet. */
	u8 h = (orient == 1 || orient == 3) ? 0x01 : 0x00;

	gmax2424_write_reg(gmax2424, 0x3400, REG_VALUE_08BIT, h);
	gmax2424_write_reg(gmax2424, 0x3800, REG_VALUE_08BIT, h);
}

static int gmax2424_update_link_menu(struct gmax2424 *gmax2424)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gmax2424->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_ctrl *ctrl;
	int wdr_index = SNS_CFG_TYPE_WDR_MODE;
	int id = gmax2424->module_index;
	int ret;
	int i;

	gmax2424_link_cif_menu[id][wdr_index] = gmax2424->cur_mode->mipi_wdr_mode;

	dev_info(&client->dev, "update mipi_mode:%lld", gmax2424_link_cif_menu[id][wdr_index]);

	ctrl_hdlr = gmax2424->sd.ctrl_handler;
	v4l2_ctrl_handler_free(ctrl_hdlr);

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 10);
	if (ret) {
		dev_err(&client->dev, "%s ctrl handler init failed (%d)\n",
			__func__, ret);
		return ret;
	}

	for (i = 0; i < SNS_CFG_TYPE_MAX; i++) {
		ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr, &gmax2424_ctrl_ops, V4L2_CID_LINK_FREQ,
					      gmax2424_link_cif_menu[id][i], 0,
					       (const s64 *)gmax2424_link_cif_menu[id]);

		if (ctrl)
			ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	}

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s new int menu failed (%d)\n",
			__func__, ret);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static long gmax2424_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gmax2424 *gmax2424 = to_gmax2424(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	long ret = 0;

	switch (cmd) {
	case SNS_V4L2_GET_TYPE:
	{
		int type = 0;

		if (gmax2424->cur_mode->mipi_wdr_mode == MIPI_WDR_MODE_NONE) {//linear
			type = gmax2424_SNS_TYPE_SDR;
		} else {//wdr
			dev_info(&client->dev, "Can not support wdr mode, default to sdr mode\n");
			type = gmax2424_SNS_TYPE_SDR;
		}
		memcpy(arg, &type, sizeof(int));
		break;
	}

	case SNS_V4L2_SET_MIRROR_FLIP:
	{
		int orient = 0;

		memcpy(&orient, arg, sizeof(int));
		gmax2424_mirror_flip(gmax2424, orient);
		break;
	}

	case SNS_V4L2_GET_I2C_INFO:
	{
		struct i2c_client *client = v4l2_get_subdevdata(&gmax2424->sd);
		sns_i2c_info_t i2c_info;

		i2c_info.i2c_addr =  client->addr;
		i2c_info.i2c_idx  =  client->adapter->i2c_idx;
		memcpy(arg, &i2c_info, sizeof(sns_i2c_info_t));
		break;
	}

#ifdef gmax2424_WDR
	case SNS_V4L2_SET_HDR_ON:
	{
		int hdr_on = 0;

		memcpy(&hdr_on, arg, sizeof(int));

		if (hdr_on)
			memcpy(gmax2424->cur_mode, &supported_modes[0], sizeof(struct gmax2424_mode));	/* single master mode */
		else
			memcpy(gmax2424->cur_mode, &supported_modes[0], sizeof(struct gmax2424_mode));

		gmax2424_update_link_menu(gmax2424);
		break;
	}
#endif

	case SNS_V4L2_SET_SNS_SYNC_INFO:
	{
		memcpy(&gmax2424->cur_mode->gmax2424_sync_info, arg, sizeof(sns_sync_info_t));
		break;
	}

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long gmax2424_compat_ioctl32(struct v4l2_subdev *sd,
				   unsigned int cmd, unsigned long arg)
{
	long ret;

	switch (cmd) {
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static const struct v4l2_subdev_core_ops gmax2424_core_ops = {
	.ioctl = gmax2424_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = gmax2424_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops gmax2424_video_ops = {
	.s_stream = set_stream,
};

static const struct v4l2_subdev_pad_ops gmax2424_pad_ops = {
	.enum_mbus_code = enum_mbus_code,
	.get_fmt = get_pad_format,
	.set_fmt = set_pad_format,
	.enum_frame_size = enum_frame_size,
	.enum_frame_interval = enum_frame_interval,
	.get_mbus_config = g_mbus_config,
};

static const struct v4l2_subdev_ops gmax2424_subdev_ops = {
	.core	= &gmax2424_core_ops,
	.video  = &gmax2424_video_ops,
	.pad    = &gmax2424_pad_ops,
};

static const struct media_entity_operations gmax2424_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops gmax2424_internal_ops = {
	.open = gmax2424_open,
};

/* Initialize control handlers */
static int gmax2424_init_controls(struct gmax2424 *gmax2424, int index_id)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gmax2424->sd);
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_ctrl *ctrl;
	int ret;
	int i;

	ctrl_hdlr = &gmax2424->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 10);
	if (ret) {
		dev_err(&client->dev, "%s ctrl handler init failed (%d)\n",
			__func__, ret);
		return ret;
	}

	mutex_init(&gmax2424->mutex);
	ctrl_hdlr->lock = &gmax2424->mutex;
	for (i = 0; i < SNS_CFG_TYPE_MAX; i++) {
		if (i == SNS_CFG_TYPE_WDR_MODE)
			gmax2424->cur_mode->mipi_wdr_mode = gmax2424_link_cif_menu[index_id][i];

		ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr, &gmax2424_ctrl_ops, V4L2_CID_LINK_FREQ,
					      gmax2424_link_cif_menu[index_id][i], 0,
					       (const s64 *)gmax2424_link_cif_menu[index_id]);

		if (ctrl)
			ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	}

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s new std menu failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &gmax2424_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	gmax2424->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&gmax2424->mutex);

	return ret;
}

static void gmax2424_free_controls(struct gmax2424 *gmax2424)
{
	v4l2_ctrl_handler_free(gmax2424->sd.ctrl_handler);
	mutex_destroy(&gmax2424->mutex);
}

static int gmax2424_probe(struct i2c_client *client,
			 const struct i2c_device_id *devid)
{
	struct gmax2424 *gmax2424;
	struct v4l2_subdev *sd;
	struct device *dev = &client->dev;
	int index_id = gmax2424_probe_index;
	int addr_num = sizeof(gmax2424_i2c_list) / sizeof(unsigned short);
	u32 bus_id, i2c_addr, use_defualt = 1;
	int ret = -1;
	int i;

	dev_info(dev, "probe id[%d] start\n", gmax2424_probe_index);

	gmax2424_probe_index++;

	if (index_id >= MAX_SENSOR_DEVICE || index_id < 0) {
		dev_info(dev, "invalid devid(%d)\n", index_id);
		return ret;
	}

	gmax2424 = devm_kzalloc(&client->dev, sizeof(*gmax2424), GFP_KERNEL);
	if (!gmax2424)
		return -ENOMEM;

	sd = &gmax2424->sd;

	if (!of_property_read_u32(client->dev.of_node,"reg-addr", &i2c_addr) &&
		!of_property_read_u32(client->dev.of_node,"bus-id", &bus_id) &&
		!gmax2424_count) {
		printk("gmax2424_probe reg = %x\n", i2c_addr);
		printk("gmax2424_probe bus-id = %x\n", bus_id);
		client->addr = i2c_addr;
		client->adapter = i2c_get_adapter(bus_id);
		gmax2424->client = client;
		v4l2_i2c_subdev_init(sd, client, &gmax2424_subdev_ops);
		/* Check module identity */
		ret = gmax2424_identify_module(gmax2424);
		if (ret) {
			dev_info(dev, "id[%d] bus[%d] i2c_addr[%d][0x%x] no sensor found,use default\n",
				index_id, bus_id, i, client->addr);
			use_defualt = 1;
		} else {
			dev_info(dev, "id[%d] bus[%d] i2c_addr[0x%x] sensor found\n",
				 index_id, bus_id, client->addr);
			use_defualt = 0;
		}
	}

	for (i = 0; i < addr_num && use_defualt; i++) {
		if (force_bus[index_id] < 0)
			bus_id = gmax2424_bus_map[index_id];
		else
			bus_id = force_bus[index_id];

		if (bus_id < 0 || bus_id > MAX_I2C_BUS_NUM)
			return ret;

		client->addr = gmax2424_i2c_list[i];
		client->adapter = i2c_get_adapter(bus_id);
		gmax2424->client = client;
		v4l2_i2c_subdev_init(sd, client, &gmax2424_subdev_ops);

		/* Check module identity */
		ret = gmax2424_identify_module(gmax2424);
		if (ret) {
			dev_info(dev, "id[%d] bus[%d] i2c_addr[%d][0x%x] no sensor found\n",
				 index_id, bus_id, i, client->addr);

			if (i == addr_num - 1)
				return ret;

			continue;
		} else {
			dev_info(dev, "id[%d] bus[%d] i2c_addr[0x%x] sensor found\n",
				 index_id, bus_id, client->addr);
			break;
		}
	}

	gmax2424->module_index = index_id;

	gmax2424->cur_mode = devm_kzalloc(&client->dev,
						sizeof(struct gmax2424_mode), GFP_KERNEL);
	memcpy(gmax2424->cur_mode, &supported_modes[0], sizeof(struct gmax2424_mode));	/* single master mode */

	memset(&gmax2424->cur_mode->gmax2424_sync_info, 0, sizeof(sns_sync_info_t));

	mutex_init(&gmax2424->mutex);

	ret = gmax2424_init_controls(gmax2424, index_id);
	if (ret)
		return ret;

	/* Initialize subdev */
	sd->internal_ops = &gmax2424_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.ops = &gmax2424_subdev_entity_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	gmax2424->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &gmax2424->pad);
	if (ret) {
		dev_err(&client->dev, "failed to init pads:%d\n", ret);
		goto error_handler_free;
	}

	snprintf(sd->name, sizeof(sd->name), "cam%d_%s %s",
		 gmax2424->module_index, "gmax2424", dev_name(sd->dev));

	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret < 0) {
		dev_err(&client->dev, "failed to async subdev:%d\n", ret);
		goto error_media_entity;
	}

	/*
	 * Device is already turned on by i2c-core with ACPI domain PM.
	 * Enable runtime PM and turn off the device.
	 */
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	dev_info(dev, "sensor_%d probe success\n", index_id);

	return 0;

error_media_entity:
	media_entity_cleanup(&gmax2424->sd.entity);

error_handler_free:
	gmax2424_free_controls(gmax2424);
	dev_err(&client->dev, "%s failed:%d\n", __func__, ret);

	return ret;
}

static int gmax2424_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gmax2424 *gmax2424 = to_gmax2424(sd);

	gmax2424_probe_index = 0;

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	gmax2424_free_controls(gmax2424);

	pm_runtime_disable(&client->dev);

	return 0;
}

static const struct of_device_id gmax2424_of_match[] = {
	{ .compatible = "v4l2,sensor0" },
	{ .compatible = "v4l2,sensor1" },
	{ .compatible = "v4l2,sensor2" },
	{ .compatible = "v4l2,sensor3" },
	{ .compatible = "v4l2,sensor4" },
	{ .compatible = "v4l2,sensor5" },
	{},
};
MODULE_DEVICE_TABLE(of, gmax2424_of_match);

static const struct dev_pm_ops gmax2424_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(suspend, resume)
};

static struct i2c_driver gmax2424_i2c_driver = {
	.driver = {
		.name = "gmax2424",
		.pm = &gmax2424_pm_ops,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gmax2424_of_match),
	},
	.probe    = gmax2424_probe,
	.remove   = gmax2424_remove,
};

static int __init sensor_mod_init(void)
{
	pr_info("== gmax2424 mod add ==\n");

	return i2c_add_driver(&gmax2424_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&gmax2424_i2c_driver);
}

module_init(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("gmax2424 sensor driver");
MODULE_LICENSE("GPL v2");
