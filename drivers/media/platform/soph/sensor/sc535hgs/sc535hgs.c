// SPDX-License-Identifier: GPL-2.0
/*
 * sc535hgs driver — SmartSens SC535HGS 5.35MP global shutter sensor
 *
 * Ported from Z project sc535hai.c (only FAE-provided init sequence is trusted
 * to stream). Register allocation matches sc233hgs template (FAE Q1-Q7 confirmed
 * 2026-08-04, see sc535hgs.h header). Based on SC535HGS V1.1 PDF §2.6.1.
 *
 * Copyright (C) 2024 Sophon Co., Ltd.
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

#include "sc535hgs.h"

/* I2C per write of bits */
#define REG_VALUE_08BIT		1
#define REG_VALUE_16BIT		2
#define REG_VALUE_24BIT		3

/* Chip ID */
#define SC535HGS_CHIP_ID_ADDR_H		0x3107
#define SC535HGS_CHIP_ID_ADDR_L		0x3108
#define SC535HGS_CHIP_ID			0xbe77

/*Sensor type for isp middleware*/
#define SC535HGS_SNS_TYPE_MASTER V4L2_SMS_SC535HGS_MASTER_MIPI_5M_60FPS_4lane_10BIT
#define SC535HGS_SNS_TYPE_SLAVE  V4L2_SMS_SC535HGS_SLAVE_MIPI_5M_60FPS_4lane_10BIT

static const enum mipi_wdr_mode_e sc535hgs_wdr_mode = MIPI_WDR_MODE_VC;

volatile int sc535hgs_count = 0;
static int force_bus[MAX_SENSOR_DEVICE] = {[0 ... (MAX_SENSOR_DEVICE - 1)] = -1};
module_param_array(force_bus, int, &sc535hgs_count, 0644);

static int sc535hgs_probe_index;
static const unsigned short sc535hgs_i2c_list[] = {0x30};
static const int sc535hgs_bus_map[MAX_SENSOR_DEVICE] = {3, -1, -1, -1, -1, -1};

struct sc535hgs_reg_list {
	u32 num_of_regs;
	const struct sc535hgs_reg *regs;
};

/* Mode : resolution and related config&values */
struct sc535hgs_mode {
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
	sns_sync_info_t sc535hgs_sync_info;
	struct sc535hgs_reg_list reg_list;
	struct sc535hgs_reg_list wdr_reg_list;
};

/* Mode configs — only 2 modes: master + slave, both 5M 60fps 4-lane linear */
static struct sc535hgs_mode supported_modes[] = {
	{
		.max_width  = 2448,
		.max_height = 2048,
		.width  = 2448,
		.height = 2048,
		.exp_def = 400,
		.hts_def = 900,
		.vts_def = 2100,
		.sns_type = V4L2_SMS_SC535HGS_MASTER_MIPI_5M_60FPS_4lane_10BIT,
		.sns_type_name  = "V4L2_SMS_SC535HGS_MASTER_MIPI_5M_60FPS_4lane_10BIT",
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,	/* 600000/10000 = 60fps */
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_linear_5m_60fps_4l_master_regs),
			.regs = mode_linear_5m_60fps_4l_master_regs,
		},
	},
	{
		.max_width  = 2448,
		.max_height = 2048,
		.width  = 2448,
		.height = 2048,
		.exp_def = 400,
		.hts_def = 900,
		.vts_def = 2100,
		.sns_type = V4L2_SMS_SC535HGS_SLAVE_MIPI_5M_60FPS_4lane_10BIT,
		.sns_type_name  = "V4L2_SMS_SC535HGS_SLAVE_MIPI_5M_60FPS_4lane_10BIT",
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_linear_5m_60fps_4l_slave_regs),
			.regs = mode_linear_5m_60fps_4l_slave_regs,
		},
	},
};

struct sc535hgs {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct i2c_client *client;
	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	/* Current mode */
	struct sc535hgs_mode *cur_mode;
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

#define to_sc535hgs(_sd)	container_of(_sd, struct sc535hgs, sd)

/* Read registers up to 4 at a time */
static int sc535hgs_read_reg(struct sc535hgs *sc535hgs, u16 reg, u32 len,
			     u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sc535hgs->sd);
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
static int sc535hgs_write_reg(struct sc535hgs *sc535hgs, u16 reg, u32 len,
			      u32 __val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sc535hgs->sd);
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
static int sc535hgs_write_regs(struct sc535hgs *sc535hgs,
			      const struct sc535hgs_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sc535hgs->sd);
	int ret;
	u32 i;

	for (i = 0; i < len; i++) {
		ret = sc535hgs_write_reg(sc535hgs, regs[i].address, 1,
					regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev, "Failed to write reg 0x%4.4x. value = %d, rror=%d\n",
					    regs[i].address, regs[i].val, ret);

			return ret;
		}
	}

	return 0;
}

/* Open sub-device */
static int sc535hgs_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc535hgs *sc535hgs = to_sc535hgs(sd);
	struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(sd, fh->pad, 0);

	mutex_lock(&sc535hgs->mutex);

	/* Initialize try_fmt */
	try_fmt->width = sc535hgs->cur_mode->width;
	try_fmt->height = sc535hgs->cur_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	/* No crop or compose */
	mutex_unlock(&sc535hgs->mutex);

	return 0;
}

static int sc535hgs_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc535hgs *sc535hgs = container_of(ctrl->handler,
					       struct sc535hgs, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&sc535hgs->sd);

	pm_runtime_put(&client->dev);

	return 0;
}

static const struct v4l2_ctrl_ops sc535hgs_ctrl_ops = {
	.s_ctrl = sc535hgs_set_ctrl,
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

	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;

	return 0;
}

static int enum_frame_interval(struct v4l2_subdev *sd,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct sc535hgs *sc535hgs = to_sc535hgs(sd);

	fie->width  = sc535hgs->cur_mode->width;
	fie->height = sc535hgs->cur_mode->height;

	fie->interval.numerator   = sc535hgs->cur_mode->max_fps.numerator;
	fie->interval.denominator = sc535hgs->cur_mode->max_fps.denominator;

	return 0;
}

static int enum_frame_size(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_frame_size_enum *fse)
{
	struct sc535hgs *sc535hgs = to_sc535hgs(sd);

	fse->min_width = sc535hgs->cur_mode->width;
	fse->max_width = sc535hgs->cur_mode->max_width;
	fse->min_height = sc535hgs->cur_mode->height;
	fse->max_height = sc535hgs->cur_mode->max_height;

	return 0;
}

static void update_pad_format(const struct sc535hgs_mode *mode, struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int get_pad_format(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc535hgs *sc535hgs = to_sc535hgs(sd);
	struct v4l2_mbus_framefmt *framefmt;
	int ret = 0;

	mutex_lock(&sc535hgs->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *framefmt;
		ret = -ENOTTY;
	} else {
		update_pad_format(sc535hgs->cur_mode, fmt);
	}
	mutex_unlock(&sc535hgs->mutex);

	return ret;
}

static int set_pad_format(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc535hgs *sc535hgs = to_sc535hgs(sd);
	struct sc535hgs_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&sc535hgs->mutex);

	/* Only one raw bayer(GRBG) order is supported */
	if (fmt->format.code != MEDIA_BUS_FMT_SGRBG10_1X10)
		fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);
	update_pad_format(mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*framefmt = fmt->format;
	} else {
		sc535hgs->cur_mode = mode;
	}

	mutex_unlock(&sc535hgs->mutex);

	return 0;
}

/* SC535HGS: stream ctrl register = 0x2100 (NOT 0x0100 like sc233hgs) */
static void sc535hgs_standby(struct sc535hgs *sc535hgs)
{
	sc535hgs_write_reg(sc535hgs, 0x2100, REG_VALUE_08BIT, 0x00);
}

static void sc535hgs_restart(struct sc535hgs *sc535hgs)
{
	sc535hgs_write_reg(sc535hgs, 0x2100, REG_VALUE_08BIT, 0x00);
	usleep_range(20000, 20000);
	sc535hgs_write_reg(sc535hgs, 0x2100, REG_VALUE_08BIT, 0x01);
}

/* Start streaming */
static int start_streaming(struct sc535hgs *sc535hgs)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sc535hgs->sd);
	const struct sc535hgs_reg_list *reg_list;
	const sns_sync_info_t *sync_info;
	int ret;

	reg_list = &sc535hgs->cur_mode->reg_list;

	ret = sc535hgs_write_regs(sc535hgs, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	sync_info = &sc535hgs->cur_mode->sc535hgs_sync_info;

	if (sync_info->num_of_regs > 0) {
		ret = sc535hgs_write_regs(sc535hgs, (struct sc535hgs_reg *)sync_info->regs,
					sync_info->num_of_regs);
		if (ret) {
			dev_err(&client->dev, "%s failed to set default\n", __func__);
			return ret;
		}
	}

	usleep_range(100 * 1000, 200 * 1000);
	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(sc535hgs->sd.ctrl_handler);
	if (ret)
		return ret;

	dev_info(&client->dev, "wdr_mode(%d) reg setting done\n", sc535hgs->cur_mode->mipi_wdr_mode);

	return ret;
}

/* Stop streaming */
static int stop_streaming(struct sc535hgs *sc535hgs)
{
	sc535hgs_standby(sc535hgs);
	return 0;
}

static int set_stream(struct v4l2_subdev *sd, int enable)
{
	struct sc535hgs *sc535hgs = to_sc535hgs(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&sc535hgs->mutex);
	if (sc535hgs->streaming == enable) {
		mutex_unlock(&sc535hgs->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		//reset sensor
		if (!IS_ERR(sc535hgs->reset_gpio)) {
			gpiod_set_value_cansleep(sc535hgs->reset_gpio, 0);
			msleep(100);
			gpiod_set_value_cansleep(sc535hgs->reset_gpio, 1);
			msleep(100);
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = start_streaming(sc535hgs);
		if (ret)
			goto err_rpm_put;
	} else {
		stop_streaming(sc535hgs);
		pm_runtime_put(&client->dev);
	}

	sc535hgs->streaming = enable;
	mutex_unlock(&sc535hgs->mutex);

	dev_info(&client->dev, "set stream(%d) success\n", enable);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&sc535hgs->mutex);

	return ret;
}

static int __maybe_unused suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc535hgs *sc535hgs = to_sc535hgs(sd);

	if (sc535hgs->streaming)
		stop_streaming(sc535hgs);

	return 0;
}

static int __maybe_unused resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc535hgs *sc535hgs = to_sc535hgs(sd);
	int ret;

	if (sc535hgs->streaming) {
		ret = start_streaming(sc535hgs);
		if (ret)
			goto error;
	}

	return 0;

error:
	stop_streaming(sc535hgs);
	sc535hgs->streaming = false;
	return ret;
}

/* Verify chip ID */
static int sc535hgs_identify_module(struct sc535hgs *sc535hgs)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sc535hgs->sd);
	int ret;
	int val1=0, val2=0;
	int read_data = 0;

	sc535hgs_write_reg(sc535hgs, 0x2100, REG_VALUE_08BIT, 0x00);/* STANDBY */

	usleep_range(20 * 1000, 20 * 1000);

	ret = sc535hgs_read_reg(sc535hgs, SC535HGS_CHIP_ID_ADDR_H,
			       REG_VALUE_08BIT, &val1);

	dev_info(&client->dev, "read id:0x%x, ret:%d", val1, ret);

	if (ret)
		return ret;

	ret = sc535hgs_read_reg(sc535hgs, SC535HGS_CHIP_ID_ADDR_L,
			       REG_VALUE_08BIT, &val2);

	read_data = ((val1 & 0xFF) << 8) | (val2 & 0xFF);

	//if (read_data != SC535HGS_CHIP_ID) {
	//	dev_err(&client->dev, "chip id(%x) mismatch, read(%x)\n",
	//		SC535HGS_CHIP_ID, read_data);
	//	return -EIO;
	//}

	return 0;
}

static void sc535hgs_mirror_flip(struct sc535hgs *sc535hgs, int orient)
{
	u8 val = 0;

	switch (orient) {
	case 0:
		break;
	case 1:
		val |= 0x6;
		break;
	case 2:
		val |= 0x60;
		break;
	case 3:
		val |= 0x66;
		break;
	default:
		return;
	}

	sc535hgs_write_reg(sc535hgs, 0x3221, REG_VALUE_08BIT, val);
}

static int sc535hgs_update_link_menu(struct sc535hgs *sc535hgs)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sc535hgs->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_ctrl *ctrl;
	int wdr_index = SNS_CFG_TYPE_WDR_MODE;
	int id = sc535hgs->module_index;
	int ret;
	int i;

	sc535hgs_link_cif_menu[id][wdr_index] = sc535hgs->cur_mode->mipi_wdr_mode;

	dev_info(&client->dev, "update mipi_mode:%lld", sc535hgs_link_cif_menu[id][wdr_index]);

	ctrl_hdlr = sc535hgs->sd.ctrl_handler;
	v4l2_ctrl_handler_free(ctrl_hdlr);

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 10);
	if (ret) {
		dev_err(&client->dev, "%s ctrl handler init failed (%d)\n",
			__func__, ret);
		return ret;
	}

	for (i = 0; i < SNS_CFG_TYPE_MAX; i++) {
		ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr, &sc535hgs_ctrl_ops, V4L2_CID_LINK_FREQ,
					      sc535hgs_link_cif_menu[id][i], 0,
					       (const s64 *)sc535hgs_link_cif_menu[id]);

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

static long sc535hgs_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc535hgs *sc535hgs = to_sc535hgs(sd);
	long ret = 0;

	switch (cmd) {
	case SNS_V4L2_GET_TYPE:
	{
		int type = 0;

		/* linear only — return master or slave SNS type */
		if (sc535hgs->cur_mode->sns_type == V4L2_SMS_SC535HGS_SLAVE_MIPI_5M_60FPS_4lane_10BIT)
			type = SC535HGS_SNS_TYPE_SLAVE;
		else
			type = SC535HGS_SNS_TYPE_MASTER;
		memcpy(arg, &type, sizeof(int));
		break;
	}

	case SNS_V4L2_SET_MIRROR_FLIP:
	{
		int orient = 0;

		memcpy(&orient, arg, sizeof(int));
		sc535hgs_mirror_flip(sc535hgs, orient);
		break;
	}

	case SNS_V4L2_GET_I2C_INFO:
	{
		struct i2c_client *client = v4l2_get_subdevdata(&sc535hgs->sd);
		sns_i2c_info_t i2c_info;

		i2c_info.i2c_addr =  client->addr;
		i2c_info.i2c_idx  =  client->adapter->i2c_idx;
		memcpy(arg, &i2c_info, sizeof(sns_i2c_info_t));
		break;
	}

	case SNS_V4L2_SET_SNS_SYNC_INFO:
	{
		memcpy(&sc535hgs->cur_mode->sc535hgs_sync_info, arg, sizeof(sns_sync_info_t));
		break;
	}

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sc535hgs_compat_ioctl32(struct v4l2_subdev *sd,
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

static const struct v4l2_subdev_core_ops sc535hgs_core_ops = {
	.ioctl = sc535hgs_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc535hgs_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc535hgs_video_ops = {
	.s_stream = set_stream,
};

static const struct v4l2_subdev_pad_ops sc535hgs_pad_ops = {
	.enum_mbus_code = enum_mbus_code,
	.get_fmt = get_pad_format,
	.set_fmt = set_pad_format,
	.enum_frame_size = enum_frame_size,
	.enum_frame_interval = enum_frame_interval,
	.get_mbus_config = g_mbus_config,
};

static const struct v4l2_subdev_ops sc535hgs_subdev_ops = {
	.core	= &sc535hgs_core_ops,
	.video  = &sc535hgs_video_ops,
	.pad    = &sc535hgs_pad_ops,
};

static const struct media_entity_operations sc535hgs_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops sc535hgs_internal_ops = {
	.open = sc535hgs_open,
};

/* Initialize control handlers */
static int sc535hgs_init_controls(struct sc535hgs *sc535hgs, int index_id)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sc535hgs->sd);
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_ctrl *ctrl;
	int ret;
	int i;

	ctrl_hdlr = &sc535hgs->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 10);
	if (ret) {
		dev_err(&client->dev, "%s ctrl handler init failed (%d)\n",
			__func__, ret);
		return ret;
	}

	mutex_init(&sc535hgs->mutex);
	ctrl_hdlr->lock = &sc535hgs->mutex;
	for (i = 0; i < SNS_CFG_TYPE_MAX; i++) {
		if (i == SNS_CFG_TYPE_WDR_MODE)
			sc535hgs->cur_mode->mipi_wdr_mode = sc535hgs_link_cif_menu[index_id][i];

		ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr, &sc535hgs_ctrl_ops, V4L2_CID_LINK_FREQ,
					      sc535hgs_link_cif_menu[index_id][i], 0,
					       (const s64 *)sc535hgs_link_cif_menu[index_id]);

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

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &sc535hgs_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	sc535hgs->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&sc535hgs->mutex);

	return ret;
}

static void sc535hgs_free_controls(struct sc535hgs *sc535hgs)
{
	v4l2_ctrl_handler_free(sc535hgs->sd.ctrl_handler);
	mutex_destroy(&sc535hgs->mutex);
}

static int sc535hgs_probe(struct i2c_client *client,
			 const struct i2c_device_id *devid)
{
	struct sc535hgs *sc535hgs;
	struct v4l2_subdev *sd;
	struct device *dev = &client->dev;
	int index_id = sc535hgs_probe_index;
	int addr_num = sizeof(sc535hgs_i2c_list) / sizeof(unsigned short);
	u32 bus_id, i2c_addr, use_defualt = 1;
	int ret = -1;
	int i;

	dev_info(dev, "probe id[%d] start\n", sc535hgs_probe_index);

	sc535hgs_probe_index++;

	if (index_id >= MAX_SENSOR_DEVICE || index_id < 0) {
		dev_info(dev, "invalid devid(%d)\n", index_id);
		return ret;
	}

	sc535hgs = devm_kzalloc(&client->dev, sizeof(*sc535hgs), GFP_KERNEL);
	if (!sc535hgs)
		return -ENOMEM;

	sd = &sc535hgs->sd;

	if (!of_property_read_u32(client->dev.of_node,"reg-addr", &i2c_addr) &&
		!of_property_read_u32(client->dev.of_node,"bus-id", &bus_id) &&
		!sc535hgs_count) {
		printk("sc535hgs_probe reg = %x\n", i2c_addr);
		printk("sc535hgs_probe bus-id = %x\n", bus_id);
		client->addr = i2c_addr;
		client->adapter = i2c_get_adapter(bus_id);
		sc535hgs->client = client;
		v4l2_i2c_subdev_init(sd, client, &sc535hgs_subdev_ops);
		/* Check module identity */
		ret = sc535hgs_identify_module(sc535hgs);
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
			bus_id = sc535hgs_bus_map[index_id];
		else
			bus_id = force_bus[index_id];

		if (bus_id < 0 || bus_id > MAX_I2C_BUS_NUM)
			return ret;

		client->addr = sc535hgs_i2c_list[i];
		client->adapter = i2c_get_adapter(bus_id);
		sc535hgs->client = client;
		v4l2_i2c_subdev_init(sd, client, &sc535hgs_subdev_ops);

		/* Check module identity */
		ret = sc535hgs_identify_module(sc535hgs);
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

	sc535hgs->module_index = index_id;

	sc535hgs->cur_mode = devm_kzalloc(&client->dev,
					sizeof(struct sc535hgs_mode), GFP_KERNEL);
	memcpy(sc535hgs->cur_mode, &supported_modes[0], sizeof(struct sc535hgs_mode));

	memset(&sc535hgs->cur_mode->sc535hgs_sync_info, 0, sizeof(sns_sync_info_t));

	mutex_init(&sc535hgs->mutex);

	ret = sc535hgs_init_controls(sc535hgs, index_id);
	if (ret)
		return ret;

	/* Initialize subdev */
	sd->internal_ops = &sc535hgs_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.ops = &sc535hgs_subdev_entity_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	sc535hgs->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &sc535hgs->pad);
	if (ret) {
		dev_err(&client->dev, "failed to init pads:%d\n", ret);
		goto error_handler_free;
	}

	snprintf(sd->name, sizeof(sd->name), "cam%d_%s %s",
		 sc535hgs->module_index, "sc535hgs", dev_name(sd->dev));

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
	media_entity_cleanup(&sc535hgs->sd.entity);

error_handler_free:
	sc535hgs_free_controls(sc535hgs);
	dev_err(&client->dev, "%s failed:%d\n", __func__, ret);

	return ret;
}

static int sc535hgs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc535hgs *sc535hgs = to_sc535hgs(sd);

	sc535hgs_probe_index = 0;

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	sc535hgs_free_controls(sc535hgs);

	pm_runtime_disable(&client->dev);

	return 0;
}

static const struct of_device_id sc535hgs_of_match[] = {
	{ .compatible = "v4l2,sensor0" },
	{ .compatible = "v4l2,sensor1" },
	{ .compatible = "v4l2,sensor2" },
	{ .compatible = "v4l2,sensor3" },
	{ .compatible = "v4l2,sensor4" },
	{ .compatible = "v4l2,sensor5" },
	{},
};
MODULE_DEVICE_TABLE(of, sc535hgs_of_match);

static const struct dev_pm_ops sc535hgs_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(suspend, resume)
};

static struct i2c_driver sc535hgs_i2c_driver = {
	.driver = {
		.name = "sc535hgs",
		.pm = &sc535hgs_pm_ops,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sc535hgs_of_match),
	},
	.probe    = sc535hgs_probe,
	.remove   = sc535hgs_remove,
};

static int __init sensor_mod_init(void)
{
	pr_info("== sc535hgs mod add ==\n");

	return i2c_add_driver(&sc535hgs_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc535hgs_i2c_driver);
}

module_init(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("sc535hgs sensor driver");
MODULE_LICENSE("GPL v2");
