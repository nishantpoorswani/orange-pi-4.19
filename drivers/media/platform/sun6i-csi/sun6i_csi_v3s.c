/*
 * Copyright (c) 2017 Magewell Electronics Co., Ltd. (Nanjing).
 * All rights reserved.
 * Author: Yong Deng <yong.deng@magewell.com>
 * Copyright (c) 2017 Ondrej Jirman <megous@megous.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DEBUG

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include "sun6i_csi.h"
#include "sun6i_csi_v3s.h"

#define MODULE_NAME	"sun6i-csi"

struct sun6i_csi_cfg {
	bool has_bt1120_if;
	bool has_16bit_yuv422_if;
};

struct sun6i_csi_dev {
	struct sun6i_csi		csi; /* must be first */
	const struct sun6i_csi_cfg	*cfg;

	struct regmap			*regmap;
	struct clk			*clk_ahb;
	struct clk			*clk_mod;
	struct clk			*clk_ram;
	struct reset_control		*rstc_ahb;

	int				planar_offset[3];
};

static inline struct sun6i_csi_dev *sun6i_csi_to_dev(struct sun6i_csi *csi)
{
	return container_of(csi, struct sun6i_csi_dev, csi);
}

static const u32 supported_pixformats[] = {
	V4L2_PIX_FMT_SBGGR8,
	V4L2_PIX_FMT_SGBRG8,
	V4L2_PIX_FMT_SGRBG8,
	V4L2_PIX_FMT_SRGGB8,
	V4L2_PIX_FMT_SBGGR10,
	V4L2_PIX_FMT_SGBRG10,
	V4L2_PIX_FMT_SGRBG10,
	V4L2_PIX_FMT_SRGGB10,
	V4L2_PIX_FMT_SBGGR12,
	V4L2_PIX_FMT_SGBRG12,
	V4L2_PIX_FMT_SGRBG12,
	V4L2_PIX_FMT_SRGGB12,
	V4L2_PIX_FMT_YUYV,
	V4L2_PIX_FMT_YVYU,
	V4L2_PIX_FMT_UYVY,
	V4L2_PIX_FMT_VYUY,
	V4L2_PIX_FMT_RGB565,
	V4L2_PIX_FMT_RGB555,
	V4L2_PIX_FMT_HM12,
	V4L2_PIX_FMT_NV12,
	V4L2_PIX_FMT_NV21,
	V4L2_PIX_FMT_YUV420,
	V4L2_PIX_FMT_YVU420,
	V4L2_PIX_FMT_NV16,
	V4L2_PIX_FMT_NV61,
	V4L2_PIX_FMT_YUV422P,
};

static int get_supported_pixformats(struct sun6i_csi *csi,
				    const u32 **pixformats)
{
	if (pixformats != NULL)
		*pixformats = supported_pixformats;

	return ARRAY_SIZE(supported_pixformats);
}

/* TODO add 10&12 bit YUV, RGB support */
static bool is_format_support(struct sun6i_csi *csi, u32 pixformat,
			      u32 mbus_code, struct sun6i_csi_subdev *csi_sd)
{
	//struct sun6i_csi_dev *sdev = sun6i_csi_to_dev(csi);
	enum v4l2_mbus_type bus_type = csi_sd->bus_type;
	bool is_parallel = bus_type == V4L2_MBUS_PARALLEL ||
			   bus_type == V4L2_MBUS_BT656;

	/*
	 * Some video receiver have capability both 8bit and 16bit.
	 * Identify the media bus format from device tree.
	 */
	if ((is_parallel && csi_sd->parallel.bus_width == 16) ||
	    bus_type == V4L2_MBUS_CSI2) {
		switch (pixformat) {
		case V4L2_PIX_FMT_HM12:
		case V4L2_PIX_FMT_NV12:
		case V4L2_PIX_FMT_NV21:
		case V4L2_PIX_FMT_NV16:
		case V4L2_PIX_FMT_NV61:
		case V4L2_PIX_FMT_YUV420:
		case V4L2_PIX_FMT_YVU420:
		case V4L2_PIX_FMT_YUV422P:
			switch (mbus_code) {
			case MEDIA_BUS_FMT_UYVY8_1X16:
			case MEDIA_BUS_FMT_VYUY8_1X16:
			case MEDIA_BUS_FMT_YUYV8_1X16:
			case MEDIA_BUS_FMT_YVYU8_1X16:
				return true;
			}
			break;
		}
		return false;
	}

	switch (pixformat) {
	case V4L2_PIX_FMT_SBGGR8:
		return mbus_code == MEDIA_BUS_FMT_SBGGR8_1X8;
	case V4L2_PIX_FMT_SGBRG8:
		return mbus_code == MEDIA_BUS_FMT_SGBRG8_1X8;
	case V4L2_PIX_FMT_SGRBG8:
		return mbus_code == MEDIA_BUS_FMT_SGRBG8_1X8;
	case V4L2_PIX_FMT_SRGGB8:
		return mbus_code == MEDIA_BUS_FMT_SRGGB8_1X8;
	case V4L2_PIX_FMT_SBGGR10:
		return mbus_code == MEDIA_BUS_FMT_SBGGR10_1X10;
	case V4L2_PIX_FMT_SGBRG10:
		return mbus_code == MEDIA_BUS_FMT_SGBRG10_1X10;
	case V4L2_PIX_FMT_SGRBG10:
		return mbus_code == MEDIA_BUS_FMT_SGRBG10_1X10;
	case V4L2_PIX_FMT_SRGGB10:
		return mbus_code == MEDIA_BUS_FMT_SRGGB10_1X10;
	case V4L2_PIX_FMT_SBGGR12:
		return mbus_code == MEDIA_BUS_FMT_SBGGR12_1X12;
	case V4L2_PIX_FMT_SGBRG12:
		return mbus_code == MEDIA_BUS_FMT_SGBRG12_1X12;
	case V4L2_PIX_FMT_SGRBG12:
		return mbus_code == MEDIA_BUS_FMT_SGRBG12_1X12;
	case V4L2_PIX_FMT_SRGGB12:
		return mbus_code == MEDIA_BUS_FMT_SRGGB12_1X12;

	case V4L2_PIX_FMT_RGB565:
		return mbus_code == MEDIA_BUS_FMT_RGB565_2X8_LE;
	case V4L2_PIX_FMT_RGB555:
		return mbus_code == MEDIA_BUS_FMT_RGB555_2X8_PADHI_LE;

	case V4L2_PIX_FMT_YUYV:
		return mbus_code == MEDIA_BUS_FMT_YUYV8_2X8;
	case V4L2_PIX_FMT_YVYU:
		return mbus_code == MEDIA_BUS_FMT_YVYU8_2X8;
	case V4L2_PIX_FMT_UYVY:
		return mbus_code == MEDIA_BUS_FMT_UYVY8_2X8;
	case V4L2_PIX_FMT_VYUY:
		return mbus_code == MEDIA_BUS_FMT_VYUY8_2X8;

	case V4L2_PIX_FMT_HM12:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_YUV422P:
		switch (mbus_code) {
		case MEDIA_BUS_FMT_UYVY8_2X8:
		case MEDIA_BUS_FMT_VYUY8_2X8:
		case MEDIA_BUS_FMT_YUYV8_2X8:
		case MEDIA_BUS_FMT_YVYU8_2X8:
			return true;
		}
		break;
	}

	return false;
}

static enum csi_input_fmt get_csi_input_format(u32 mbus_code, u32 pixformat)
{
	/* bayer */
	if ((mbus_code & 0xF000) == 0x3000)
		return CSI_INPUT_FORMAT_RAW;

	switch (pixformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB555:
		return CSI_INPUT_FORMAT_RAW;
	}

	/* not support YUV420 input format yet */
	return CSI_INPUT_FORMAT_YUV422;
}

static enum csi_output_fmt get_csi_output_format(u32 pixformat, u32 field)
{
	bool buf_interlaced = false;

	if (field == V4L2_FIELD_INTERLACED ||
	    field == V4L2_FIELD_INTERLACED_TB ||
	    field == V4L2_FIELD_INTERLACED_BT)
		buf_interlaced = true;

	switch (pixformat) {
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		return buf_interlaced ? CSI_FRAME_RAW_8 : CSI_FIELD_RAW_8;
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
		return buf_interlaced ? CSI_FRAME_RAW_10 : CSI_FIELD_RAW_10;
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
		return buf_interlaced ? CSI_FRAME_RAW_12 : CSI_FIELD_RAW_12;

	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB555:
		return buf_interlaced ? CSI_FRAME_RAW_8 : CSI_FIELD_RAW_8;

	case V4L2_PIX_FMT_HM12:
		return buf_interlaced ? CSI_FRAME_MB_YUV420 :
					CSI_FIELD_MB_YUV420;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		return buf_interlaced ? CSI_FRAME_UV_CB_YUV420 :
					CSI_FIELD_UV_CB_YUV420;
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		return buf_interlaced ? CSI_FRAME_PLANAR_YUV420 :
					CSI_FIELD_PLANAR_YUV420;
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		return buf_interlaced ? CSI_FRAME_UV_CB_YUV422 :
					CSI_FIELD_UV_CB_YUV422;
	case V4L2_PIX_FMT_YUV422P:
		return buf_interlaced ? CSI_FRAME_PLANAR_YUV422 :
					CSI_FIELD_PLANAR_YUV422;
	}

	return 0;
}

static enum csi_input_seq get_csi_input_seq(u32 mbus_code, u32 pixformat)
{

	switch (pixformat) {
	case V4L2_PIX_FMT_HM12:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YUV422P:
		switch (mbus_code) {
		case MEDIA_BUS_FMT_UYVY8_2X8:
		case MEDIA_BUS_FMT_UYVY8_1X16:
			return CSI_INPUT_SEQ_UYVY;
		case MEDIA_BUS_FMT_VYUY8_2X8:
		case MEDIA_BUS_FMT_VYUY8_1X16:
			return CSI_INPUT_SEQ_VYUY;
		case MEDIA_BUS_FMT_YUYV8_2X8:
		case MEDIA_BUS_FMT_YUYV8_1X16:
			return CSI_INPUT_SEQ_YUYV;
		case MEDIA_BUS_FMT_YVYU8_1X16:
		case MEDIA_BUS_FMT_YVYU8_2X8:
			return CSI_INPUT_SEQ_YVYU;
		}
		break;
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_YVU420:
		switch (mbus_code) {
		case MEDIA_BUS_FMT_UYVY8_2X8:
		case MEDIA_BUS_FMT_UYVY8_1X16:
			return CSI_INPUT_SEQ_VYUY;
		case MEDIA_BUS_FMT_VYUY8_2X8:
		case MEDIA_BUS_FMT_VYUY8_1X16:
			return CSI_INPUT_SEQ_UYVY;
		case MEDIA_BUS_FMT_YUYV8_2X8:
		case MEDIA_BUS_FMT_YUYV8_1X16:
			return CSI_INPUT_SEQ_YVYU;
		case MEDIA_BUS_FMT_YVYU8_1X16:
		case MEDIA_BUS_FMT_YVYU8_2X8:
			return CSI_INPUT_SEQ_YUYV;
		}
		break;
	}

	return CSI_INPUT_SEQ_YUYV;
}

static void sun6i_csi_setup_bus(struct sun6i_csi_dev *sdev,
				struct sun6i_csi_subdev *csi_sd)
{
	unsigned char bus_width = csi_sd->parallel.bus_width;
	u32 flags = csi_sd->parallel.flags;
	u32 cfg;

	regmap_read(sdev->regmap, CSI_IF_CFG_REG, &cfg);

	cfg &= ~(CSI_IF_CFG_CSI_IF_MASK | CSI_IF_CFG_MIPI_IF_MASK |
		 CSI_IF_CFG_IF_DATA_WIDTH_MASK |
		 CSI_IF_CFG_CLK_POL_MASK | CSI_IF_CFG_VREF_POL_MASK |
		 CSI_IF_CFG_HREF_POL_MASK | CSI_IF_CFG_FIELD_MASK);

	switch (csi_sd->bus_type) {
#if 0
	case V4L2_MBUS_CSI2:
		cfg |= CSI_IF_CFG_MIPI_IF_MIPI;
		break;
#endif
	case V4L2_MBUS_PARALLEL:
		cfg |= CSI_IF_CFG_MIPI_IF_CSI;

		cfg |= (bus_width == 16) ? CSI_IF_CFG_CSI_IF_YUV422_16BIT :
					   CSI_IF_CFG_CSI_IF_YUV422_INTLV;

		if (flags & V4L2_MBUS_FIELD_EVEN_LOW)
			cfg |= CSI_IF_CFG_FIELD_POSITIVE;

		if (flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH)
			cfg |= CSI_IF_CFG_VREF_POL_POSITIVE;
		if (flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH)
			cfg |= CSI_IF_CFG_HREF_POL_POSITIVE;

		if (flags & V4L2_MBUS_PCLK_SAMPLE_FALLING)
			cfg |= CSI_IF_CFG_CLK_POL_FALLING_EDGE;
		break;
	case V4L2_MBUS_BT656:
		cfg |= CSI_IF_CFG_MIPI_IF_CSI;

		cfg |= (bus_width == 16) ? CSI_IF_CFG_CSI_IF_BT1120 :
					   CSI_IF_CFG_CSI_IF_BT656;

		if (flags & V4L2_MBUS_FIELD_EVEN_LOW)
			cfg |= CSI_IF_CFG_FIELD_POSITIVE;

		if (flags & V4L2_MBUS_PCLK_SAMPLE_FALLING)
			cfg |= CSI_IF_CFG_CLK_POL_FALLING_EDGE;
		break;
	default:
		BUG();
		break;
	}

	switch (bus_width) {
	case 8:
		cfg |= CSI_IF_CFG_IF_DATA_WIDTH_8BIT;
		break;
	case 10:
		cfg |= CSI_IF_CFG_IF_DATA_WIDTH_10BIT;
		break;
	case 12:
		cfg |= CSI_IF_CFG_IF_DATA_WIDTH_12BIT;
		break;
	default:
		break;
	}

	regmap_write(sdev->regmap, CSI_IF_CFG_REG, cfg);
}

static void sun6i_csi_set_format(struct sun6i_csi_dev *sdev)
{
	struct sun6i_csi *csi = &sdev->csi;
	u32 cfg;
	u32 val;

	regmap_read(sdev->regmap, CSI_CH_CFG_REG, &cfg);

	cfg &= ~(CSI_CH_CFG_INPUT_FMT_MASK |
		 CSI_CH_CFG_OUTPUT_FMT_MASK | CSI_CH_CFG_VFLIP_EN |
		 CSI_CH_CFG_HFLIP_EN | CSI_CH_CFG_FIELD_SEL_MASK |
		 CSI_CH_CFG_INPUT_SEQ_MASK);

	val = get_csi_input_format(csi->current_fmt->mbus_code,
				   csi->fmt.fmt.pix.pixelformat);
	cfg |= CSI_CH_CFG_INPUT_FMT(val);

	val = get_csi_output_format(csi->current_fmt->mbus_code,
				    csi->fmt.fmt.pix.field);
	cfg |= CSI_CH_CFG_OUTPUT_FMT(val);

	val = get_csi_input_seq(csi->current_fmt->mbus_code,
				csi->fmt.fmt.pix.pixelformat);
	cfg |= CSI_CH_CFG_INPUT_SEQ(val);

	if (csi->fmt.fmt.pix.field == V4L2_FIELD_TOP)
		cfg |= CSI_CH_CFG_FIELD_SEL_FIELD0;
	else if (csi->fmt.fmt.pix.field == V4L2_FIELD_BOTTOM)
		cfg |= CSI_CH_CFG_FIELD_SEL_FIELD1;
	else
		cfg |= CSI_CH_CFG_FIELD_SEL_BOTH;

	regmap_write(sdev->regmap, CSI_CH_CFG_REG, cfg);
}

static void sun6i_csi_set_window(struct sun6i_csi_dev *sdev)
{
	struct sun6i_csi *csi = &sdev->csi;
	u32 bytesperline_y;
	u32 bytesperline_c;
	int *planar_offset = sdev->planar_offset;
	u32 width = csi->fmt.fmt.pix.width;
	u32 height = csi->fmt.fmt.pix.height;
	u32 hor_len = width;

	switch (csi->fmt.fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB555:
		hor_len *= 2;
		break;
	}

	regmap_write(sdev->regmap, CSI_CH_HSIZE_REG,
		     CSI_CH_HSIZE_HOR_LEN(hor_len) |
		     CSI_CH_HSIZE_HOR_START(0));
	regmap_write(sdev->regmap, CSI_CH_VSIZE_REG,
		     CSI_CH_VSIZE_VER_LEN(height) |
		     CSI_CH_VSIZE_VER_START(0));

	planar_offset[0] = 0;

	switch (csi->fmt.fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_HM12:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		bytesperline_y = width;
		bytesperline_c = width;
		planar_offset[1] = bytesperline_y * height;
		planar_offset[2] = -1;
		break;
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		bytesperline_y = width;
		bytesperline_c = width / 2;
		planar_offset[1] = bytesperline_y * height;
		planar_offset[2] = planar_offset[1] +
				bytesperline_c * height / 2;
		break;
	case V4L2_PIX_FMT_YUV422P:
		bytesperline_y = width;
		bytesperline_c = width / 2;
		planar_offset[1] = bytesperline_y * height;
		planar_offset[2] = planar_offset[1] +
				bytesperline_c * height;
		break;
	default: /* raw */
		bytesperline_y = (csi->current_fmt->bpp * width) / 8;
		bytesperline_c = 0;
		planar_offset[1] = -1;
		planar_offset[2] = -1;
		break;
	}

	regmap_write(sdev->regmap, CSI_CH_BUF_LEN_REG,
		     CSI_CH_BUF_LEN_BUF_LEN_C(bytesperline_c) |
		     CSI_CH_BUF_LEN_BUF_LEN_Y(bytesperline_y));
}

static int set_power(struct sun6i_csi *csi, bool enable)
{
	struct sun6i_csi_dev *sdev = sun6i_csi_to_dev(csi);
	struct regmap *regmap = sdev->regmap;
	int ret;

	if (!enable) {
		regmap_update_bits(regmap, CSI_EN_REG, CSI_EN_CSI_EN, 0);

		clk_disable_unprepare(sdev->clk_ram);
		clk_disable_unprepare(sdev->clk_mod);
		clk_disable_unprepare(sdev->clk_ahb);
		reset_control_assert(sdev->rstc_ahb);
		return 0;
	}

	ret = clk_prepare_enable(sdev->clk_ahb);
	if (ret) {
		dev_err(csi->dev, "Enable ahb clk err %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(sdev->clk_mod);
	if (ret) {
		dev_err(csi->dev, "Enable csi clk err %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(sdev->clk_ram);
	if (ret) {
		dev_err(csi->dev, "Enable clk_dram_csi clk err %d\n", ret);
		return ret;
	}

	if (!IS_ERR_OR_NULL(sdev->rstc_ahb)) {
		ret = reset_control_deassert(sdev->rstc_ahb);
		if (ret) {
			dev_err(csi->dev, "reset err %d\n", ret);
			return ret;
		}
	}

	regmap_update_bits(regmap, CSI_EN_REG, CSI_EN_CSI_EN, CSI_EN_CSI_EN);

	return 0;
}

static int apply_config(struct sun6i_csi *csi, struct sun6i_csi_subdev *csi_sd)
{
	struct sun6i_csi_dev *sdev = sun6i_csi_to_dev(csi);

	sun6i_csi_setup_bus(sdev, csi_sd);
	sun6i_csi_set_format(sdev);
	sun6i_csi_set_window(sdev);

	return 0;
}

static int update_buf_addr(struct sun6i_csi *csi, dma_addr_t addr)
{
	struct sun6i_csi_dev *sdev = sun6i_csi_to_dev(csi);
	/* transform physical address to bus address */
	dma_addr_t bus_addr = addr - 0x40000000;

	regmap_write(sdev->regmap, CSI_CH_F0_BUFA_REG,
		     (bus_addr + sdev->planar_offset[0]) >> 2);
	if (sdev->planar_offset[1] != -1)
		regmap_write(sdev->regmap, CSI_CH_F1_BUFA_REG,
			     (bus_addr + sdev->planar_offset[1]) >> 2);
	if (sdev->planar_offset[2] != -1)
		regmap_write(sdev->regmap, CSI_CH_F2_BUFA_REG,
			     (bus_addr + sdev->planar_offset[2]) >> 2);

	return 0;
}

static int set_stream(struct sun6i_csi *csi, bool enable)
{
	struct sun6i_csi_dev *sdev = sun6i_csi_to_dev(csi);
	struct regmap *regmap = sdev->regmap;

	if (!enable) {
		regmap_update_bits(regmap, CSI_CAP_REG, CSI_CAP_CH0_VCAP_ON, 0);
		regmap_write(regmap, CSI_CH_INT_EN_REG, 0);
		regmap_update_bits(regmap, CSI_EN_REG, CSI_EN_CSI_EN, 0);
		return 0;
	}

	/* reset */
	regmap_update_bits(regmap, CSI_EN_REG, CSI_EN_CSI_EN, CSI_EN_CSI_EN);
	regmap_update_bits(regmap, CSI_EN_REG, CSI_EN_CSI_EN, 0);
	regmap_update_bits(regmap, CSI_EN_REG, CSI_EN_CSI_EN, CSI_EN_CSI_EN);

	regmap_write(regmap, CSI_CH_INT_STA_REG, 0xFF);
	regmap_write(regmap, CSI_CH_INT_EN_REG,
		     CSI_CH_INT_EN_HB_OF_INT_EN |
		     CSI_CH_INT_EN_FIFO2_OF_INT_EN |
		     CSI_CH_INT_EN_FIFO1_OF_INT_EN |
		     CSI_CH_INT_EN_FIFO0_OF_INT_EN |
		     CSI_CH_INT_EN_FD_INT_EN |
		     CSI_CH_INT_EN_CD_INT_EN);

	regmap_update_bits(regmap, CSI_CAP_REG, CSI_CAP_CH0_VCAP_ON,
			   CSI_CAP_CH0_VCAP_ON);

	return 0;
}

static struct sun6i_csi_ops csi_ops = {
	.get_supported_pixformats	= get_supported_pixformats,
	.is_format_support		= is_format_support,
	.s_power			= set_power,
	.apply_config			= apply_config,
	.update_buf_addr		= update_buf_addr,
	.s_stream			= set_stream,
};

static irqreturn_t sun6i_csi_isr(int irq, void *dev_id)
{
	struct sun6i_csi_dev *sdev = (struct sun6i_csi_dev *)dev_id;
	struct regmap *regmap = sdev->regmap;
	u32 status;

	regmap_read(regmap, CSI_CH_INT_STA_REG, &status);

	if ((status & CSI_CH_INT_STA_FIFO0_OF_PD) ||
	    (status & CSI_CH_INT_STA_FIFO1_OF_PD) ||
	    (status & CSI_CH_INT_STA_FIFO2_OF_PD) ||
	    (status & CSI_CH_INT_STA_HB_OF_PD)) {
		regmap_write(regmap, CSI_CH_INT_STA_REG, 0xff);
		regmap_update_bits(regmap, CSI_EN_REG, CSI_EN_CSI_EN, 0);
		regmap_update_bits(regmap, CSI_EN_REG, CSI_EN_CSI_EN,
				   CSI_EN_CSI_EN);
		return IRQ_HANDLED;
	}

	if (status & CSI_CH_INT_STA_FD_PD)
		sun6i_video_frame_done(&sdev->csi);

	regmap_write(regmap, CSI_CH_INT_STA_REG, 0xff);
	return IRQ_HANDLED;
}

static const struct regmap_config sun6i_csi_regmap_config = {
	.reg_bits       = 32,
	.reg_stride     = 4,
	.val_bits       = 32,
	.max_register	= 0x1000,
};

static int sun6i_csi_resource_request(struct sun6i_csi_dev *sdev,
				      struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *io_base;
	int ret;
	int irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(io_base))
		return PTR_ERR(io_base);

	sdev->regmap = devm_regmap_init_mmio(&pdev->dev, io_base,
					    &sun6i_csi_regmap_config);
	if (IS_ERR(sdev->regmap)) {
		dev_err(&pdev->dev, "Failed to init register map\n");
		return PTR_ERR(sdev->regmap);
	}

	sdev->clk_ahb = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(sdev->clk_ahb)) {
		dev_err(&pdev->dev, "Unable to acquire ahb clock\n");
		return PTR_ERR(sdev->clk_ahb);
	}

	sdev->clk_mod = devm_clk_get(&pdev->dev, "mod");
	if (IS_ERR(sdev->clk_mod)) {
		dev_err(&pdev->dev, "Unable to acquire csi clock\n");
		return PTR_ERR(sdev->clk_mod);
	}

	sdev->clk_ram = devm_clk_get(&pdev->dev, "ram");
	if (IS_ERR(sdev->clk_ram)) {
		dev_err(&pdev->dev, "Unable to acquire dram-csi clock\n");
		return PTR_ERR(sdev->clk_ram);
	}

	sdev->rstc_ahb = devm_reset_control_get_optional_shared(&pdev->dev,
								NULL);
	if (IS_ERR(sdev->rstc_ahb)) {
		dev_err(&pdev->dev, "Cannot get reset controller\n");
		return PTR_ERR(sdev->rstc_ahb);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "No csi IRQ specified\n");
		ret = -ENXIO;
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, irq, sun6i_csi_isr, 0, MODULE_NAME,
			       sdev);
	if (ret) {
		dev_err(&pdev->dev, "Cannot request csi IRQ\n");
		return ret;
	}

	return 0;
}

static int sun6i_csi_probe(struct platform_device *pdev)
{
	struct sun6i_csi_dev *sdev;
	int ret;

	sdev = devm_kzalloc(&pdev->dev, sizeof(*sdev), GFP_KERNEL);
	if (!sdev)
		return -ENOMEM;

	sdev->csi.dev = &pdev->dev;
	platform_set_drvdata(pdev, sdev);

	sdev->cfg = of_device_get_match_data(&pdev->dev);

	ret = sun6i_csi_resource_request(sdev, pdev);
	if (ret)
		return ret;

	sdev->csi.ops = &csi_ops;
	ret = sun6i_csi_init(&sdev->csi);
	if (ret)
		return ret;

	return 0;
}

static int sun6i_csi_remove(struct platform_device *pdev)
{
	struct sun6i_csi_dev *sdev = platform_get_drvdata(pdev);

	sun6i_csi_cleanup(&sdev->csi);

	return 0;
}

static const struct sun6i_csi_cfg sun8i_v3s_cfg = {
	.has_bt1120_if = true,
	.has_16bit_yuv422_if = true,
};

static const struct sun6i_csi_cfg sun8i_a83t_cfg = {
	.has_bt1120_if = false,
	.has_16bit_yuv422_if = false,
};

static const struct of_device_id sun6i_csi_of_match[] = {
	{ .compatible = "allwinner,sun8i-v3s-csi", .data = &sun8i_v3s_cfg },
	{ .compatible = "allwinner,sun8i-a83t-csi", .data = &sun8i_a83t_cfg },
	{},
};
MODULE_DEVICE_TABLE(of, sun6i_csi_of_match);

static struct platform_driver sun6i_csi_platform_driver = {
	.probe = sun6i_csi_probe,
	.remove = sun6i_csi_remove,
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(sun6i_csi_of_match),
	},
};
module_platform_driver(sun6i_csi_platform_driver);

MODULE_DESCRIPTION("Allwinner V3s Camera Sensor Interface driver");
MODULE_AUTHOR("Yong Deng <yong.deng@magewell.com>");
MODULE_LICENSE("GPL v2");
