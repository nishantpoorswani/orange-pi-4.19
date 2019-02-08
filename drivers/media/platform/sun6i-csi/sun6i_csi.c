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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "sun6i_csi.h"

// {{{ utils

static struct sun6i_csi_subdev *
sun6i_get_enabled_subdev(struct sun6i_csi *csi)
{
	struct media_pad *remote;

	remote = media_entity_remote_pad(&csi->pad);

	if (!remote || !is_media_entity_v4l2_subdev(remote->entity))
		return NULL;

	return v4l2_get_subdev_hostdata(
		media_entity_to_v4l2_subdev(remote->entity));
}

static struct sun6i_csi_format *
sun6i_find_format_by_fourcc(struct sun6i_csi *csi, u32 fourcc)
{
	int i;

	for (i = 0; i < csi->num_formats; i++)
		if (csi->formats[i].fourcc == fourcc)
			return &csi->formats[i];

	return NULL;
}

// }}}
// {{{ vb2

struct sun6i_csi_buffer {
	struct vb2_v4l2_buffer		vb;
	struct list_head		list;

	dma_addr_t			dma_addr;
};

static int sun6i_video_queue_setup(struct vb2_queue *vq,
				 unsigned int *nbuffers, unsigned int *nplanes,
				 unsigned int sizes[],
				 struct device *alloc_devs[])
{
	struct sun6i_csi *csi = vb2_get_drv_priv(vq);
	unsigned int size = csi->fmt.fmt.pix.sizeimage;

	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = size;

	return 0;
}

static int sun6i_video_buffer_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct sun6i_csi_buffer *buf =
			container_of(vbuf, struct sun6i_csi_buffer, vb);
	struct sun6i_csi *csi = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = csi->fmt.fmt.pix.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		v4l2_err(csi->vdev.v4l2_dev, "buffer too small (%lu < %lu)\n",
			 vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);
	buf->dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	vbuf->field = csi->fmt.fmt.pix.field;

	return 0;
}

static int sun6i_sources_set_stream(struct sun6i_csi *csi, bool enable)
{
	struct media_entity *entity;
	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	int ret;

	entity = &csi->vdev.entity;
	while (1) {
		pad = &entity->pads[0];
		if (!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		pad = media_entity_remote_pad(pad);
		if (!pad || !is_media_entity_v4l2_subdev(pad->entity))
			break;

		entity = pad->entity;
		subdev = media_entity_to_v4l2_subdev(entity);

		ret = v4l2_subdev_call(subdev, video, s_stream, enable);
		if (enable && ret < 0 && ret != -ENOIOCTLCMD)
			return ret;
	}

	return 0;
}

static int sun6i_csi_apply_config(struct sun6i_csi *csi)
{
	struct sun6i_csi_subdev *csi_sd;

	csi_sd = sun6i_get_enabled_subdev(csi);
	if (csi_sd == NULL)
		return -ENXIO;

	if (csi->ops != NULL && csi->ops->apply_config != NULL)
		return csi->ops->apply_config(csi, csi_sd);

	return -ENOIOCTLCMD;
}

static int sun6i_video_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct sun6i_csi *csi = vb2_get_drv_priv(vq);
	struct sun6i_csi_buffer *buf;
	unsigned long flags;
	int ret;

	ret = media_pipeline_start(&csi->vdev.entity, &csi->vdev.pipe);
	if (ret < 0)
		goto err_queue_buffers;

	ret = sun6i_csi_apply_config(csi);
	if (ret < 0)
		goto err_stop_media_pipeline;

	spin_lock_irqsave(&csi->dma_queue_lock, flags);
	csi->sequence = 0;
	csi->skip_first_interrupt = true;
	buf = list_first_entry(&csi->dma_queue, struct sun6i_csi_buffer, list);
	ret = sun6i_csi_update_buf_addr(csi, buf->dma_addr);
	spin_unlock_irqrestore(&csi->dma_queue_lock, flags);
	if (ret < 0)
		goto err_stop_media_pipeline;

	ret = sun6i_csi_set_stream(csi, true);
	if (ret < 0)
		goto err_stop_media_pipeline;

	ret = sun6i_sources_set_stream(csi, true);
	if (ret < 0)
		goto err_stop_stream;

	return 0;

err_stop_stream:
	sun6i_sources_set_stream(csi, false);
err_stop_media_pipeline:
	media_pipeline_stop(&csi->vdev.entity);
err_queue_buffers:
	spin_lock_irqsave(&csi->dma_queue_lock, flags);
	list_for_each_entry(buf, &csi->dma_queue, list)
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
	INIT_LIST_HEAD(&csi->dma_queue);
	spin_unlock_irqrestore(&csi->dma_queue_lock, flags);

	return ret;
}

static void sun6i_video_stop_streaming(struct vb2_queue *vq)
{
	struct sun6i_csi *csi = vb2_get_drv_priv(vq);
	unsigned long flags;
	struct sun6i_csi_buffer *buf;

	sun6i_csi_set_stream(csi, false);
	sun6i_sources_set_stream(csi, false);
	media_pipeline_stop(&csi->vdev.entity);

	/* Release all active buffers */
	spin_lock_irqsave(&csi->dma_queue_lock, flags);
	list_for_each_entry(buf, &csi->dma_queue, list)
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	INIT_LIST_HEAD(&csi->dma_queue);
	spin_unlock_irqrestore(&csi->dma_queue_lock, flags);
}

static void sun6i_video_buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct sun6i_csi_buffer *buf =
			container_of(vbuf, struct sun6i_csi_buffer, vb);
	struct sun6i_csi *csi = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long flags;

	spin_lock_irqsave(&csi->dma_queue_lock, flags);
	list_add_tail(&buf->list, &csi->dma_queue);
	spin_unlock_irqrestore(&csi->dma_queue_lock, flags);
}

void sun6i_video_frame_done(struct sun6i_csi *csi)
{
	struct sun6i_csi_buffer *buf;
	struct sun6i_csi_buffer *next_buf;
	bool can_dequeue;

	spin_lock(&csi->dma_queue_lock);

	if (!vb2_is_streaming(&csi->vb2_vidq))
		goto out_unlock;

	/* we always keep two buffers in the queue and expect the third */
	buf = list_first_entry(&csi->dma_queue, struct sun6i_csi_buffer, list);
	next_buf = list_next_entry(buf, list);
	can_dequeue = !list_is_last(&next_buf->list, &csi->dma_queue);

	if (csi->skip_first_interrupt) {
		csi->skip_first_interrupt = false;
		sun6i_csi_update_buf_addr(csi, next_buf->dma_addr);
		goto out_unlock;
	} else if (!can_dequeue) {
		csi->skip_first_interrupt = true;
	} else {
		struct vb2_v4l2_buffer *vbuf = &buf->vb;
		struct vb2_buffer *vb = &vbuf->vb2_buf;

		vb->timestamp = ktime_get_ns();
		vbuf->sequence = csi->sequence++;
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);

		list_del(&buf->list);

		next_buf = list_next_entry(next_buf, list);
		sun6i_csi_update_buf_addr(csi, next_buf->dma_addr);
	}

out_unlock:
	spin_unlock(&csi->dma_queue_lock);
}

static struct vb2_ops sun6i_csi_vb2_ops = {
	.queue_setup		= sun6i_video_queue_setup,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.buf_prepare		= sun6i_video_buffer_prepare,
	.start_streaming	= sun6i_video_start_streaming,
	.stop_streaming		= sun6i_video_stop_streaming,
	.buf_queue		= sun6i_video_buffer_queue,
};

// }}}
// {{{ videodev ioct

static int sun6i_video_set_fmt(struct sun6i_csi *csi, struct v4l2_format *f,
			       bool try_only)
{
	struct v4l2_subdev_format sd_fmt;
	struct v4l2_subdev_pad_config padconf;
	struct v4l2_pix_format *pixfmt = &f->fmt.pix;
	struct sun6i_csi_format *csi_fmt;
	struct sun6i_csi_subdev *csi_sd;
	int ret;

	if (csi->num_formats == 0)
		return -EINVAL;

	csi_fmt = sun6i_find_format_by_fourcc(csi, pixfmt->pixelformat);
	if (csi_fmt == NULL)
		csi_fmt = &csi->formats[0];
	pixfmt->pixelformat = csi_fmt->fourcc;

	csi_sd = sun6i_get_enabled_subdev(csi);
	if (csi_sd == NULL)
		return -ENXIO;

	sd_fmt.pad = csi_sd->pad;
	sd_fmt.which = V4L2_SUBDEV_FORMAT_TRY;
	v4l2_fill_mbus_format(&sd_fmt.format, pixfmt, csi_fmt->mbus_code);
	ret = v4l2_subdev_call(csi_sd->sd, pad, set_fmt, &padconf, &sd_fmt);
	if (ret)
		return ret;

	v4l2_fill_pix_format(pixfmt, &sd_fmt.format);
	pixfmt->bytesperline = (pixfmt->width * csi_fmt->bpp) / 8;
	pixfmt->sizeimage = pixfmt->bytesperline * pixfmt->height;
	pixfmt->flags = 0;

	if (!try_only) {
		sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		ret = v4l2_subdev_call(csi_sd->sd, pad, set_fmt, NULL, &sd_fmt);
		if (ret)
			return ret;

		//XXX: check that we got an expected format

		csi->fmt = *f;
		csi->current_fmt = csi_fmt;
	}

	return 0;
}

static int sun6i_querycap(struct file *file, void *priv,
			  struct v4l2_capability *cap)
{
	struct sun6i_csi *csi = video_drvdata(file);

	strlcpy(cap->driver, "sun6i-video", sizeof(cap->driver));
	strlcpy(cap->card, csi->vdev.name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 csi->dev->of_node->name);

	return 0;
}

static int sun6i_try_fmt_vid_cap(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct sun6i_csi *csi = video_drvdata(file);

	return sun6i_video_set_fmt(csi, f, true);
}

static int sun6i_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct sun6i_csi *csi = video_drvdata(file);

	*fmt = csi->fmt;

	return 0;
}

static int sun6i_s_fmt_vid_cap(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct sun6i_csi *csi = video_drvdata(file);

	if (vb2_is_streaming(&csi->vb2_vidq))
		return -EBUSY;

	return sun6i_video_set_fmt(csi, f, false);
}

static int sun6i_enum_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_fmtdesc *f)
{
	struct sun6i_csi *csi = video_drvdata(file);

	if (f->index >= csi->num_formats)
		return -EINVAL;

	f->flags = 0;
	f->description[0] = '\0';
	f->pixelformat = csi->formats[f->index].fourcc;

	return 0;
}

static int sun6i_enum_framesizes(struct file *file, void *priv,
				 struct v4l2_frmsizeenum *fsize)
{
	struct sun6i_csi *csi = video_drvdata(file);
	struct sun6i_csi_format *csi_fmt;
	struct sun6i_csi_subdev *csi_sd;
	struct v4l2_subdev_frame_size_enum fse = {
		.index = fsize->index,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	csi_fmt = sun6i_find_format_by_fourcc(csi, fsize->pixel_format);
	if (!csi_fmt)
		return -EINVAL;

	fse.code = csi_fmt->mbus_code;

	csi_sd = sun6i_get_enabled_subdev(csi);
	if (csi_sd == NULL)
		return -ENXIO;

	ret = v4l2_subdev_call(csi_sd->sd, pad, enum_frame_size, NULL, &fse);
	if (ret)
		return ret;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = fse.max_width;
	fsize->discrete.height = fse.max_height;

	return 0;
}

static int sun6i_enum_frameintervals(struct file *file, void *priv,
				     struct v4l2_frmivalenum *fival)
{
	struct sun6i_csi *csi = video_drvdata(file);
	struct sun6i_csi_format *csi_fmt;
	struct sun6i_csi_subdev *csi_sd;
	struct v4l2_subdev_frame_interval_enum fie = {
		.index = fival->index,
		.width = fival->width,
		.height = fival->height,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	csi_fmt = sun6i_find_format_by_fourcc(csi, fival->pixel_format);
	if (!csi_fmt)
		return -EINVAL;

	fie.code = csi_fmt->mbus_code;

	csi_sd = sun6i_get_enabled_subdev(csi);
	if (csi_sd == NULL)
		return -ENXIO;

	ret = v4l2_subdev_call(csi_sd->sd, pad,
			       enum_frame_interval, NULL, &fie);
	if (ret)
		return ret;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete = fie.interval;

	return 0;
}

static int sun6i_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct sun6i_csi *csi = video_drvdata(file);
	struct sun6i_csi_subdev *csi_sd;

	csi_sd = sun6i_get_enabled_subdev(csi);
	if (csi_sd == NULL)
		return -ENXIO;

	return v4l2_g_parm_cap(video_devdata(file), csi_sd->sd, a);
}

static int sun6i_s_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct sun6i_csi *csi = video_drvdata(file);
	struct sun6i_csi_subdev *csi_sd;

	csi_sd = sun6i_get_enabled_subdev(csi);
	if (csi_sd == NULL)
		return -ENXIO;

	return v4l2_g_parm_cap(video_devdata(file), csi_sd->sd, a);
}

static int sun6i_enum_input(struct file *file, void *priv,
			   struct v4l2_input *i)
{
	struct sun6i_csi *csi = video_drvdata(file);
	struct v4l2_subdev *subdev;
	int ret, s, idx;

	idx = 0;
	for (s = 0; s < SUN6I_CSI_NUM_SENSORS; s++) {
		subdev = csi->sensors[s].sd;
		if (!subdev)
			continue;

		if (idx == i->index) {
			ret = v4l2_subdev_call(subdev, video, g_input_status,
					       &i->status);
			if (ret < 0 && ret != -ENOIOCTLCMD && ret != -ENODEV)
				return ret;

			i->type = V4L2_INPUT_TYPE_CAMERA;
			strlcpy(i->name, "Camera", sizeof(i->name));
			return 0;
		}

		idx++;
	}

	return -EINVAL;
}

static int sun6i_g_input(struct file *file, void *priv, unsigned int *i)
{
	//XXX: get index of the enabled input

	*i = 0;

	return 0;
}

static int sun6i_s_input(struct file *file, void *priv, unsigned int i)
{
	//XXX: enable link

	if (i > 0)
		return -EINVAL;

	return 0;
}

static const struct v4l2_ioctl_ops sun6i_video_ioctl_ops = {
	.vidioc_querycap		= sun6i_querycap,

	.vidioc_try_fmt_vid_cap		= sun6i_try_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= sun6i_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= sun6i_s_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap	= sun6i_enum_fmt_vid_cap,

	.vidioc_g_parm			= sun6i_g_parm,
	.vidioc_s_parm			= sun6i_s_parm,
	.vidioc_enum_framesizes		= sun6i_enum_framesizes,
	.vidioc_enum_frameintervals	= sun6i_enum_frameintervals,

	.vidioc_enum_input		= sun6i_enum_input,
	.vidioc_g_input			= sun6i_g_input,
	.vidioc_s_input			= sun6i_s_input,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,

	.vidioc_log_status		= v4l2_ctrl_log_status,
	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

// }}}
// {{{ videodev fops

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */
static int sun6i_video_open(struct file *file)
{
	struct sun6i_csi *csi = video_drvdata(file);
	struct v4l2_format format;
	int ret;

	if (mutex_lock_interruptible(&csi->lock))
		return -ERESTARTSYS;

	ret = v4l2_fh_open(file);
	if (ret < 0)
		goto unlock;

	ret = v4l2_pipeline_pm_use(&csi->vdev.entity, 1);
	if (ret < 0)
		goto fh_release;

	if (!v4l2_fh_is_singular_file(file))
		goto unlock;

	ret = sun6i_csi_set_power(csi, true);
	if (ret < 0)
		goto fh_release;

	/* setup default format */
	if (csi->num_formats > 0) {
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		format.fmt.pix.width = 1280;
		format.fmt.pix.height = 720;
		format.fmt.pix.pixelformat = csi->formats[0].fourcc;
		sun6i_video_set_fmt(csi, &format, false);
	}

	mutex_unlock(&csi->lock);
	return 0;

fh_release:
	v4l2_fh_release(file);
unlock:
	mutex_unlock(&csi->lock);
	return ret;
}

static int sun6i_video_close(struct file *file)
{
	struct sun6i_csi *csi = video_drvdata(file);
	bool last_fh;

	mutex_lock(&csi->lock);

	last_fh = v4l2_fh_is_singular_file(file);

	_vb2_fop_release(file, NULL);

	v4l2_pipeline_pm_use(&csi->vdev.entity, 0);

	if (last_fh)
		sun6i_csi_set_power(csi, false);

	mutex_unlock(&csi->lock);

	return 0;
}

static const struct v4l2_file_operations sun6i_video_fops = {
	.owner		= THIS_MODULE,
	.open		= sun6i_video_open,
	.release	= sun6i_video_close,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
	.poll		= vb2_fop_poll
};

// }}}
// {{{ media ops

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static bool
sun6i_csi_is_format_support(struct sun6i_csi *csi, u32 pixformat, u32 mbus_code,
		struct sun6i_csi_subdev *csi_sd)
{
	if (csi->ops != NULL && csi->ops->is_format_support != NULL)
		return csi->ops->is_format_support(csi, pixformat, mbus_code,
						   csi_sd);

	return -ENOIOCTLCMD;
}

static int sun6i_video_formats_init(struct sun6i_csi *csi)
{
	struct v4l2_subdev_mbus_code_enum mbus_code = { 0 };
	struct sun6i_csi_subdev *csi_sd;
	const u32 *pixformats;
	int pixformat_count = 0;
	u32 subdev_codes[32];
	int codes_count = 0;
	int num_fmts = 0;
	int i, j;

	csi_sd = sun6i_get_enabled_subdev(csi);
	if (csi_sd == NULL)
		return -ENXIO;

	/* Get supported pixformats of CSI */
	pixformat_count = sun6i_csi_get_supported_pixformats(csi, &pixformats);
	if (pixformat_count <= 0)
		return -ENXIO;

	/* Get subdev formats codes */
	mbus_code.pad = csi_sd->pad;
	mbus_code.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	while (codes_count < ARRAY_SIZE(subdev_codes) &&
	       !v4l2_subdev_call(csi_sd->sd, pad, enum_mbus_code, NULL,
				 &mbus_code)) {
		subdev_codes[codes_count] = mbus_code.code;
		codes_count++;
		mbus_code.index++;
	}

	if (!codes_count)
		return -ENXIO;

	/* Get supported formats count */
	for (j = 0; j < pixformat_count; j++) {
		for (i = 0; i < codes_count; i++) {
			if (!sun6i_csi_is_format_support(csi, pixformats[j],
					subdev_codes[i], csi_sd))
				continue;
			num_fmts++;
			break;
		}
	}

	if (!num_fmts)
		return -ENXIO;

	if (csi->formats)
		devm_kfree(csi->dev, csi->formats);

	csi->num_formats = num_fmts;
	csi->formats = devm_kcalloc(csi->dev, num_fmts,
			sizeof(struct sun6i_csi_format), GFP_KERNEL);
	if (!csi->formats)
		return -ENOMEM;

	/* Get supported formats */
	num_fmts = 0;
	for (j = 0; j < pixformat_count; j++) {
		for (i = 0; i < codes_count; i++) {
			if (!sun6i_csi_is_format_support(csi, pixformats[j],
					subdev_codes[i], csi_sd))
				continue;

			dev_dbg(csi->dev, "supported format: pix=%d:bus=%d\n",
				pixformats[j], subdev_codes[i]);

			csi->formats[num_fmts].fourcc = pixformats[j];
			csi->formats[num_fmts].mbus_code = subdev_codes[i];
			csi->formats[num_fmts].bpp =
					v4l2_pixformat_get_bpp(pixformats[j]);
			num_fmts++;
			break;
		}
	}

	return 0;
}

static int sun6i_video_link_setup(struct media_entity *entity,
				  const struct media_pad *local,
				  const struct media_pad *remote, u32 flags)
{
	struct video_device *vdev = media_entity_to_video_device(entity);
	struct sun6i_csi *csi = video_get_drvdata(vdev);

	if (WARN_ON(csi == NULL))
		return 0;

	return sun6i_video_formats_init(csi);
}

static const struct media_entity_operations sun6i_video_media_ops = {
	.link_setup = sun6i_video_link_setup,
};

// }}}
// {{{ sun6i_csi video init/cleanup

static void sun6i_video_cleanup(struct sun6i_csi *csi)
{
	if (video_is_registered(&csi->vdev))
		video_unregister_device(&csi->vdev);

	media_entity_cleanup(&csi->vdev.entity);
}

static int sun6i_video_init(struct sun6i_csi *csi, const char *name)
{
	struct video_device *vdev = &csi->vdev;
	struct vb2_queue *vidq = &csi->vb2_vidq;
	int ret;

	/* Initialize the media entity... */
	csi->pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	vdev->entity.ops = &sun6i_video_media_ops;
	ret = media_entity_pads_init(&vdev->entity, 1, &csi->pad);
	if (ret < 0)
		return ret;

	mutex_init(&csi->lock);

	INIT_LIST_HEAD(&csi->dma_queue);
	spin_lock_init(&csi->dma_queue_lock);

	csi->sequence = 0;
	csi->num_formats = 0;

	/* Initialize videobuf2 queue */
	vidq->type			= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vidq->io_modes			= VB2_MMAP | VB2_DMABUF;
	vidq->drv_priv			= csi;
	vidq->buf_struct_size		= sizeof(struct sun6i_csi_buffer);
	vidq->ops			= &sun6i_csi_vb2_ops;
	vidq->mem_ops			= &vb2_dma_contig_memops;
	vidq->timestamp_flags		= V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	vidq->lock			= &csi->lock;
	vidq->min_buffers_needed	= 2;
	vidq->dev			= csi->dev;

	ret = vb2_queue_init(vidq);
	if (ret) {
		v4l2_err(&csi->v4l2_dev, "vb2_queue_init failed: %d\n", ret);
		goto error;
	}

	/* Register video device */
	strlcpy(vdev->name, name, sizeof(vdev->name));
	vdev->release		= video_device_release_empty;
	vdev->fops		= &sun6i_video_fops;
	vdev->ioctl_ops		= &sun6i_video_ioctl_ops;
	vdev->vfl_type		= VFL_TYPE_GRABBER;
	vdev->vfl_dir		= VFL_DIR_RX;
	vdev->v4l2_dev		= &csi->v4l2_dev;
	vdev->queue		= vidq;
	vdev->lock		= &csi->lock;
	vdev->device_caps	= V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE;
	video_set_drvdata(vdev, csi);

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		v4l2_err(&csi->v4l2_dev,
			 "video_register_device failed: %d\n", ret);
		goto error;
	}

	return 0;

error:
	sun6i_video_cleanup(csi);
	return ret;
}

// }}}
// {{{ sun6i_csi init/cleanup - DT parsing/subdev setup

struct sun6i_csi_async_subdev {
	struct v4l2_async_subdev asd; /* must be first */
	unsigned int ep_id;
	enum v4l2_mbus_type bus_type;
	struct v4l2_fwnode_bus_parallel parallel;
};

#define notifier_to_csi(n) container_of(n, struct sun6i_csi, notifier)
#define asd_to_csi_asd(a) container_of(a, struct sun6i_csi_async_subdev, asd)

static int sun6i_csi_notify_bound(struct v4l2_async_notifier *notifier,
				     struct v4l2_subdev *subdev,
				     struct v4l2_async_subdev *asd)
{
	struct sun6i_csi *csi = notifier_to_csi(notifier);
	struct sun6i_csi_async_subdev *csi_asd = asd_to_csi_asd(asd);
	unsigned int i;

	dev_dbg(csi->dev, "bound subdev %s\n", subdev->name);

	if (subdev->entity.function != MEDIA_ENT_F_CAM_SENSOR) {
		dev_err(csi->dev, "subdev %s must be a camera sensor\n",
			subdev->name);
		return -EINVAL;
	}

	for (i = 0; i < SUN6I_CSI_NUM_SENSORS; i++) {
		if (csi->sensors[i].sd == NULL) {
			csi->sensors[i].sd = subdev;
			csi->sensors[i].ep_id = csi_asd->ep_id;
			csi->sensors[i].bus_type = csi_asd->bus_type;
			csi->sensors[i].parallel = csi_asd->parallel;
			v4l2_set_subdev_hostdata(subdev, &csi->sensors[i]);
			return 0;
		}
	}

	dev_err(csi->dev, "subdev %s not bound, not enough sensor slots\n",
		subdev->name);
	return -ENOMEM;
}

static void sun6i_csi_notify_unbind(struct v4l2_async_notifier *notifier,
				       struct v4l2_subdev *subdev,
				       struct v4l2_async_subdev *asd)
{
	struct sun6i_csi *csi = notifier_to_csi(notifier);
	//struct sun6i_csi_async_subdev *csi_asd = asd_to_csi_asd(asd);
	unsigned int i;

	dev_err(csi->dev, "unbind subdev %s\n", subdev->name);

	for (i = 0; i < SUN6I_CSI_NUM_SENSORS; i++) {
		if (csi->sensors[i].sd == subdev) {
			csi->sensors[i].sd = NULL;
			v4l2_set_subdev_hostdata(subdev, NULL);
			return;
		}
	}
}

static int sun6i_csi_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct sun6i_csi *csi = notifier_to_csi(notifier);
	struct v4l2_subdev *subdev;
	struct media_entity *sink = &csi->vdev.entity;
	struct media_entity *source;
	unsigned int pad;
	unsigned int i;
	int ret;
	bool first_sensor = true;

	dev_dbg(csi->dev, "notify complete, all subdevs bound\n");

	for (i = 0; i < SUN6I_CSI_NUM_SENSORS; i++) {
		subdev = csi->sensors[i].sd;
		if (subdev == NULL)
			continue;

		source = &subdev->entity;

		for (pad = 0; pad < source->num_pads; pad++) {
			if (!(source->pads[pad].flags & MEDIA_PAD_FL_SOURCE))
				continue;

			csi->sensors[i].pad = pad;

			ret = media_create_pad_link(source, pad, sink,
						    0, first_sensor ?
						    MEDIA_LNK_FL_ENABLED : 0);
			if (ret)
				return ret;

			dev_dbg(csi->dev, "created pad link %s:%u -> %s:0\n",
				subdev->name, pad, csi->vdev.name);

			if (first_sensor) {
				ret = media_entity_call(sink, link_setup,
							&sink->pads[0],
							&source->pads[pad], 0);
				if (ret)
					return ret;
			}

			first_sensor = false;
			goto next_sensor;
		}

		dev_err(csi->dev, "subdev %s - no source pad found\n",
			subdev->name);
		return -EINVAL;
next_sensor:;
	}

	ret = v4l2_device_register_subdev_nodes(&csi->v4l2_dev);
	if (ret < 0) {
		dev_err(csi->dev, "failed to register subdev nodes\n");
		return ret;
	}

	dev_dbg(csi->dev, "registering media device\n");

	return media_device_register(&csi->media_dev);
}

// this is called for each controller endpoint
static int sun6i_csi_parse_subdev_endpoint(struct device *dev,
				   struct v4l2_fwnode_endpoint *vep,
				   struct v4l2_async_subdev *asd)
{
	//struct sun6i_csi *csi = dev_get_drvdata(dev);
	struct sun6i_csi_async_subdev *csi_asd = asd_to_csi_asd(asd);

	if (vep->base.port) {
		dev_err(dev, "CSI has only one port\n");
		return -ENOTCONN;
	}

	switch (vep->bus_type) {
	case V4L2_MBUS_PARALLEL:
	case V4L2_MBUS_BT656:
		csi_asd->ep_id = vep->base.id;
		csi_asd->bus_type = vep->bus_type;
		csi_asd->parallel = vep->bus.parallel;
		return 0;
	default:
		dev_err(dev, "Unsupported media bus type\n");
		return -ENOTCONN;
	}
}

static const struct v4l2_async_notifier_operations sun6i_csi_notifier_ops = {
	.bound = sun6i_csi_notify_bound,
	.unbind = sun6i_csi_notify_unbind,
	.complete = sun6i_csi_notify_complete,
};

int sun6i_csi_init(struct sun6i_csi *csi)
{
	int ret;

	csi->media_dev.dev = csi->dev;
	strlcpy(csi->media_dev.model, "Allwinner Video Capture Device",
		sizeof(csi->media_dev.model));
	media_device_init(&csi->media_dev);

	ret = v4l2_ctrl_handler_init(&csi->ctrl_handler, 0);
	if (ret) {
		dev_err(csi->dev, "V4L2 controls handler init failed (%d)\n",
			ret);
		goto media_clean;
	}

	csi->v4l2_dev.mdev = &csi->media_dev;
	csi->v4l2_dev.ctrl_handler = &csi->ctrl_handler;
	ret = v4l2_device_register(csi->dev, &csi->v4l2_dev);
	if (ret < 0) {
		dev_err(csi->dev, "V4L2 device registration failed (%d)\n",
			ret);
		goto ctrls_clean;
	}

	ret = sun6i_video_init(csi, "sun6i-csi");
	if (ret < 0)
		goto v4l2_clean;

	// Parse DT and build notifier.subdevs (struct v4l2_async_subdev) list
	// that will be used to match and bind/unbind subdevices (sensors) to
	// the csi->v4l2_dev when they are probed and registered by their own
	// drivers. sun6i_csi_parse_subdev_endpoint callback can be used to:
	// - exclude certain subdev endpoints from being watched
	// - parse subdev DT endpoint properties and pass them later to bound
	//   callback via internediate struct sun6i_csi_async_subdev
	ret = v4l2_async_notifier_parse_fwnode_endpoints(
		csi->dev, &csi->notifier, sizeof(struct sun6i_csi_async_subdev),
		sun6i_csi_parse_subdev_endpoint);
	if (ret)
		goto video_clean;

	csi->notifier.ops = &sun6i_csi_notifier_ops;
	ret = v4l2_async_notifier_register(&csi->v4l2_dev, &csi->notifier);
	if (ret < 0) {
		dev_err(csi->dev, "Notifier registration failed\n");
		goto notifier_clean;
	}

	return 0;

notifier_clean:
	v4l2_async_notifier_cleanup(&csi->notifier);
video_clean:
	sun6i_video_cleanup(csi);
v4l2_clean:
	v4l2_device_unregister(&csi->v4l2_dev);
	media_device_unregister(&csi->media_dev);
ctrls_clean:
	v4l2_ctrl_handler_free(&csi->ctrl_handler);
media_clean:
	media_device_cleanup(&csi->media_dev);
	return ret;
}

int sun6i_csi_cleanup(struct sun6i_csi *csi)
{
	v4l2_async_notifier_unregister(&csi->notifier);
	v4l2_async_notifier_cleanup(&csi->notifier);
	sun6i_video_cleanup(csi);
	v4l2_device_unregister(&csi->v4l2_dev);
	v4l2_ctrl_handler_free(&csi->ctrl_handler);
	media_device_unregister(&csi->media_dev);
	media_device_cleanup(&csi->media_dev);

	return 0;
}

// }}}
