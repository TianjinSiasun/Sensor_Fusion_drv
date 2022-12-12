// SPDX-License-Identifier: GPL-2.0-only
/*
 * sensor_fusion_v4l2.c - A Video Core Driver, core initialization
 *
 * Copyright 2022 Tianjin Siasun Intelligent Technology Co.,Ltd. All rights reserved.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/font.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <linux/v4l2-dv-timings.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>

#include "sensor_fusion_register.h"
#include "sensor_fusion_v4l2.h"
#include "sensor_fusion_ov8865.h"

#define SIASUN_V4L2_MODULE_NAME "siasun_video"

//unsigned int adapter_num[ALTERA_VIDEO_NUM] = {21, 22, 25, 26, 23, 24};

int siasun_sync_addr[] = {CAMERA_LVDS_0_TERASIC_CAMERA_SYNC_0_BASE,
                          CAMERA_LVDS_1_TERASIC_CAMERA_SYNC_0_BASE,
                          CAMERA_LVDS_2_TERASIC_CAMERA_SYNC_0_BASE,
                          CAMERA_LVDS_3_TERASIC_CAMERA_SYNC_0_BASE,
                          CAMERA_MIPI_0_TERASIC_CAMERA_SYNC_0_BASE,
                          CAMERA_MIPI_1_TERASIC_CAMERA_SYNC_0_BASE};

/* ---------------------------------sub device ops start-------------------------------------*/

static struct siasun_video_dev *to_svid(struct v4l2_subdev *sd)
{
    return container_of(sd, struct siasun_video_dev, sd);
}

static int siasun_subdev_init(struct v4l2_subdev *sd, u32 arg)
{
    int ret = 0;
    struct siasun_video_dev *svid = to_svid(sd);

    DPRINTK("siasun_subdev_init index[%d]\n", svid->index);

    writel(0, svid->config_base + TERASIC_VIP_SCHEDULE_0_BASE + REG_BUF_CONTROL * 4);
    mdelay(100);
    writel(0, svid->config_base + siasun_sync_addr[svid->index] + REG_CAMERA_SYNC_GO * 4);
    mdelay(1);

    switch (svid->index)
    {
        case 0:
            siasun_lvds0_init(svid);
            ret = siasun_lvds_procedure(svid);
            break;
        case 1:
            siasun_lvds1_init(svid);
            ret = siasun_lvds_procedure(svid);
            break;
        case 2:
            siasun_lvds2_init(svid);
            ret = siasun_lvds_procedure(svid);
            break;
        case 3:
            siasun_lvds3_init(svid);
            ret = siasun_lvds_procedure(svid);
            break;
        case 4:
            ret = siasun_mipi_procedure(svid);
            break;
        case 5:
            ret = siasun_mipi_procedure(svid);
            break;
        default:
            printk(KERN_ERR "siasun_subdev_init index[%d] fault\n", svid->index);
            ret = -EINVAL;
    }

    writel(1, svid->config_base + siasun_sync_addr[svid->index] + REG_CAMERA_SYNC_GO * 4);
    mdelay(100);
    writel(1, svid->config_base + TERASIC_VIP_SCHEDULE_0_BASE + REG_BUF_CONTROL * 4);
    mdelay(1);

    return ret;
}

static int siasun_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct i2c_adapter *adap = NULL;
    struct siasun_video_dev *svid = to_svid(sd);

    if (enable != 0 && enable != 1)
    {
        printk(KERN_ERR "siasun_s_stream invalid param[%d]\n", enable);
        return -EINVAL;
    }

    adap = i2c_get_adapter(svid->bus_num);
    if (!adap)
    {
        printk(KERN_ERR "video[%d] get adapter failed\n", svid->index);
        return -ENODEV;
    }

    writel(0, svid->config_base + TERASIC_VIP_SCHEDULE_0_BASE + REG_BUF_CONTROL * 4);
    mdelay(10);

    writel(0, svid->config_base + siasun_sync_addr[svid->index] + REG_CAMERA_SYNC_GO * 4);

    siasun_i2c_write8(adap, MIPI_I2C_ADDR, 0x100, enable);

    writel(1, svid->config_base + siasun_sync_addr[svid->index] + REG_CAMERA_SYNC_GO * 4);


    mdelay(10);
    writel(1, svid->config_base + TERASIC_VIP_SCHEDULE_0_BASE + REG_BUF_CONTROL * 4);
    svid->streaming = enable;

    i2c_put_adapter(adap);

    return 0;
}

static const struct v4l2_subdev_core_ops siasun_core_ops = {
	.init = siasun_subdev_init,
};

static const struct v4l2_subdev_video_ops siasun_video_ops = {
    .s_stream   = siasun_s_stream,
};

static const struct v4l2_subdev_ops siasun_subdev_ops = {
    .core = &siasun_core_ops,
    .video = &siasun_video_ops,
};

/* ----------------------------------sub device ops end--------------------------------------*/

/* -----------------------------video device ioctl ops start-----------------------------------*/

static int siasun_v4l2_open(struct file *file)
{
    int ret =0;
    DPRINTK("enter %s\n", __FUNCTION__);

    return ret;
}

static int siasun_v4l2_release(struct file *file)
{
    int ret = 0;
    DPRINTK("enter %s\n", __FUNCTION__);

    return ret;
}

static int siasun_v4l2_mmap(struct file *file, struct vm_area_struct *vma)
{
    int ret = 0;
    unsigned long size;

    DPRINTK("enter %s\n", __FUNCTION__);

    if (!(vma->vm_flags & VM_SHARED)) {
        printk(KERN_ERR "invalid vma flags, VM_SHARED needed\n");
        return -EINVAL;
    }

    if (!(vma->vm_flags & VM_READ)) {
        printk(KERN_ERR "invalid vma flags, VM_READ needed\n");
        return -EINVAL;
    }

    size = vma->vm_end - vma->vm_start;
    DPRINTK("mmap, start %lx end %lx off %lx \n", vma->vm_start, vma->vm_end, vma->vm_pgoff);
    remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size, vma->vm_page_prot);

    vma->vm_flags |= (VM_IO | VM_DONTEXPAND | VM_DONTDUMP);

    return ret;
}

static int siasun_v4l2_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
    struct siasun_video_dev *svid = video_drvdata(file);

    DPRINTK("enter %s\n", __FUNCTION__);

    strscpy(cap->driver, SIASUN_V4L2_MODULE_NAME, sizeof(cap->driver));
    strscpy(cap->card, SIASUN_V4L2_MODULE_NAME, sizeof(cap->card));
    snprintf(cap->bus_info, sizeof(cap->bus_info), "pcie:%s%d", SIASUN_V4L2_MODULE_NAME, svid->index);

    cap->capabilities |= (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING);

    return 0;
}

static int siasun_v4l2_g_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
    struct v4l2_pix_format *pix;

    DPRINTK("enter %s\n", __FUNCTION__);

    switch (f->type)
    {
        case V4L2_BUF_TYPE_VIDEO_CAPTURE:
            pix = &f->fmt.pix;
            pix->width = 1600;
            pix->height = 1200;
            pix->field = V4L2_FIELD_NONE;
            pix->pixelformat = V4L2_PIX_FMT_RGB24;
            pix->sizeimage = 3 * 1600 * 1200 + 512;
            pix->colorspace = V4L2_COLORSPACE_DEFAULT;
            break;
        default:
            return -EINVAL;
    }

    return 0;
}

static int siasun_v4l2_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
    int ret = 0;
    struct siasun_video_dev *svid = video_drvdata(file);

    DPRINTK("enter %s\n", __FUNCTION__);

    if (svid->streaming)
    {
        printk(KERN_ERR "streaming active\n");
        return -EBUSY;
    }

    if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
        printk(KERN_ERR "%s type mismatch\n", __func__);
        return -EINVAL;
    }

    ret = v4l2_subdev_call(&svid->sd, core, init, 0);
    if (ret < 0)
    {
        printk(KERN_ERR "init failed\n");
    }

    svid->fmt.fmt.pix.width = f->fmt.pix.width;
    svid->fmt.fmt.pix.height = f->fmt.pix.height;
    svid->fmt.fmt.pix.sizeimage = 3 * f->fmt.pix.width * f->fmt.pix.height + 512;

    return ret;
}

int siasun_verify_memory_type(unsigned int memory, unsigned int type)
{
    if (!(memory & VB2_MEMORY_MMAP) || !(memory != VB2_MEMORY_DMABUF))
    {
        printk(KERN_ERR "unsupported memory type\n");
        return -EINVAL;
    }

    if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
        printk(KERN_ERR "requested type is incorrect\n");
        return -EINVAL;
    }

    return 0;
}

int siasun_v4l2_reqbufs(struct file *file, void *priv, struct v4l2_requestbuffers *p)
{
    int ret = 0;
    struct siasun_video_dev *svid = video_drvdata(file);

    DPRINTK("enter %s\n", __FUNCTION__);

    ret = siasun_verify_memory_type(p->memory, p->type);
    if (ret)
    {
        return ret;
    }

    if (svid->streaming)
    {
        printk(KERN_ERR "streaming active\n");
        return -EBUSY;
    }

    if (p->count != 2)
    {
        p->count = 2;
    }

    return ret;
}

int siasun_v4l2_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
    int ret = 0;
    struct siasun_video_dev *svid = video_drvdata(file);

    DPRINTK("enter %s\n", __FUNCTION__);
    ret = siasun_verify_memory_type(p->memory, p->type);
    if (ret)
    {
        return ret;
    }

    if (p->index >= 2)
    {
        printk(KERN_ERR "buffer index out of range\n");
        return -EINVAL;
    }

    if (p->index == 0)
    {
        p->length = CAMERA_BUF_SIZE1;
        p->m.offset = svid->phys_base_start;
    }
    else if (p->index == 1)
    {
        p->length = CAMERA_BUF_SIZE2;
        p->m.offset = svid->phys_base_start_h;
    }
    return ret;
}

int siasun_v4l2_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
    int ret = 0;
    struct siasun_video_dev *svid = video_drvdata(file);

    DPRINTK("enter %s\n", __FUNCTION__);

    ret = siasun_verify_memory_type(p->memory, p->type);
    if (ret)
    {
        return ret;
    }

    memset(svid->vir_base_start, 0, CAMERA_BUF_SIZE1);
    memset(svid->vir_base_start_h, 0, CAMERA_BUF_SIZE2);
    svid->data_ready = 0;

    return ret;
}

int siasun_v4l2_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
    int ret = 0;
    struct siasun_video_dev *svid = video_drvdata(file);

    DPRINTK("enter %s\n", __FUNCTION__);

    ret = siasun_verify_memory_type(p->memory, p->type);
    if (ret)
    {
        return ret;
    }

    ret = wait_event_interruptible(svid->wait, (svid->data_ready == 1));

    return ret;
}

int siasun_v4l2_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
    int ret = 0;
    struct siasun_video_dev *svid = video_drvdata(file);

    DPRINTK("enter %s\n", __FUNCTION__);

    if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
        printk(KERN_ERR "invalid stream type[%d]\n", i);
        return -EINVAL;
    }

    ret = v4l2_subdev_call(&svid->sd, video, s_stream, 1);
    if (ret)
    {
        printk(KERN_ERR "enable stream failed\n");
    }

    return ret;
}

int siasun_v4l2_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
    int ret = 0;
    struct siasun_video_dev *svid = video_drvdata(file);

    DPRINTK("enter %s\n", __FUNCTION__);

    if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
        printk(KERN_ERR "invalid stream type [%d]\n", i);
        return -EINVAL;
    }

    ret = v4l2_subdev_call(&svid->sd, video, s_stream, 0);
    if (ret)
    {
        printk(KERN_ERR "enable stream failed\n");
    }

    return ret;
}

static const struct v4l2_ioctl_ops siasun_v4l2_ioctl_ops = {
    .vidioc_querycap        = siasun_v4l2_querycap,
    .vidioc_g_fmt_vid_cap   = siasun_v4l2_g_fmt_cap,
    .vidioc_s_fmt_vid_cap   = siasun_v4l2_s_fmt,
    .vidioc_reqbufs         = siasun_v4l2_reqbufs,
    .vidioc_querybuf        = siasun_v4l2_querybuf,
    .vidioc_qbuf            = siasun_v4l2_qbuf,
    .vidioc_dqbuf           = siasun_v4l2_dqbuf,
    .vidioc_streamon        = siasun_v4l2_streamon,
    .vidioc_streamoff       = siasun_v4l2_streamoff,
};

static const struct v4l2_file_operations siasun_v4l2_fops = {
    .owner          = THIS_MODULE,
    .open           = siasun_v4l2_open,
    .release        = siasun_v4l2_release,
    .unlocked_ioctl = video_ioctl2,
    .mmap           = siasun_v4l2_mmap,
};

/* -----------------------------video device ioctl ops end-----------------------------------*/

/* -----------------------------------v4l2 dev ops start-------------------------------------*/

static void siasun_vid_release(struct v4l2_device *v4l2_dev)
{
    struct siasun_video_dev *svid = container_of(v4l2_dev, struct siasun_video_dev, v4l2_dev);
    v4l2_device_unregister(&svid->v4l2_dev);
}

/* ------------------------------------v4l2 dev ops end--------------------------------------*/

struct siasun_video_dev *siasun_v4l2_create_instance(struct siasun_pcie_device *pdev, int index)
{
    int ret = 0;
    struct siasun_video_dev *svid = NULL;

    DPRINTK("Enter siasun_v4l2_create_instance\n");

    /* allocate main vivid state structure */
    svid = kzalloc(sizeof(struct siasun_video_dev), GFP_KERNEL);
    if (!svid)
    {
        printk(KERN_ERR "kzalloc video dev failed\n");
        ret = -ENOMEM;
        goto fail;
    }

    svid->index = index;
//    svid->bus_num = adapter_num[index];
    svid->pci_dev = pdev->siasun_sf_dev[1]->pci_dev;
    svid->config_base = pdev->siasun_sf_dev[1]->base_addr[4];
    switch (index)
    {
        case 0:
            svid->phys_base_start = pdev->camera0.physaddr1;
            svid->phys_base_start_h = pdev->camera0.physaddr2;
            svid->vir_base_start = pdev->camera0.viraddr1;
            svid->vir_base_start_h = pdev->camera0.viraddr2;
            break;
        case 1:
            svid->phys_base_start = pdev->camera1.physaddr1;
            svid->phys_base_start_h = pdev->camera1.physaddr2;
            svid->vir_base_start = pdev->camera1.viraddr1;
            svid->vir_base_start_h = pdev->camera1.viraddr2;
            break;
        case 2:
            svid->phys_base_start = pdev->camera2.physaddr1;
            svid->phys_base_start_h = pdev->camera2.physaddr2;
            svid->vir_base_start = pdev->camera2.viraddr1;
            svid->vir_base_start_h = pdev->camera2.viraddr2;
            break;
        case 3:
            svid->phys_base_start = pdev->camera3.physaddr1;
            svid->phys_base_start_h = pdev->camera3.physaddr2;
            svid->vir_base_start = pdev->camera3.viraddr1;
            svid->vir_base_start_h = pdev->camera3.viraddr2;
            break;
        case 4:
            svid->phys_base_start = pdev->camera4.physaddr1;
            svid->phys_base_start_h = pdev->camera4.physaddr2;
            svid->vir_base_start = pdev->camera4.viraddr1;
            svid->vir_base_start_h = pdev->camera4.viraddr2;
            break;
        case 5:
            svid->phys_base_start = pdev->camera5.physaddr1;
            svid->phys_base_start_h = pdev->camera5.physaddr2;
            svid->vir_base_start = pdev->camera5.viraddr1;
            svid->vir_base_start_h = pdev->camera5.viraddr2;
            break;
        default:
            printk(KERN_ERR "index err[%d]\n", index);
            break;
    }
    svid->buf_len = CAMERA_BUF_SIZE1;
    svid->buf_len_h = CAMERA_BUF_SIZE2;

    /* register v4l2_device */
    snprintf(svid->v4l2_dev.name, sizeof(svid->v4l2_dev.name),
             "%s-%03d", SIASUN_V4L2_MODULE_NAME, index);
    svid->v4l2_dev.ctrl_handler = NULL;
    ret = v4l2_device_register(&svid->pci_dev->dev, &svid->v4l2_dev);
    if (ret) {
        printk(KERN_ERR "v4l2_device_register[%d] failed\n", index);
        goto free;
    }
    svid->v4l2_dev.release = siasun_vid_release;

    /* register v4l2_subdev */
    v4l2_subdev_init(&svid->sd, &siasun_subdev_ops);
    svid->sd.flags |= V4L2_SUBDEV_FL_IS_I2C;
    strscpy(svid->sd.name, svid->v4l2_dev.name, sizeof(svid->sd.name));
    v4l2_set_subdevdata(&svid->sd, svid);
    svid->sd.ctrl_handler = NULL;
    ret = v4l2_device_register_subdev(&svid->v4l2_dev, &svid->sd);
    if (ret)
    {
        printk(KERN_ERR "unable to register siausn sub device[%d]\n", index);
        goto unregister_v4l2;
    }

    /* register video_device */
    snprintf(svid->video_dev.name, sizeof(svid->video_dev.name),
             "%s-%03d", SIASUN_V4L2_MODULE_NAME, index);
    mutex_init(&svid->lock);
    svid->video_dev.v4l2_dev = &svid->v4l2_dev;
    svid->video_dev.fops = &siasun_v4l2_fops;
    svid->video_dev.release = video_device_release_empty;
    svid->video_dev.ioctl_ops = &siasun_v4l2_ioctl_ops;
    svid->video_dev.lock = &svid->lock;
    svid->video_dev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
    video_set_drvdata(&svid->video_dev, svid);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,7,0)
    ret = video_register_device(&svid->video_dev, VFL_TYPE_VIDEO, -1);
#else
    ret = video_register_device(&svid->video_dev, VFL_TYPE_GRABBER, -1);
#endif
    if (ret) {
        printk(KERN_ERR "video[%d]: can't register device\n", index);
        goto unregister_subdev;
    }

    init_waitqueue_head(&svid->wait);
    svid->data_ready = 0;

    printk(KERN_INFO "video[%d] is created\n", index);
    return svid;

unregister_subdev:
    v4l2_device_unregister_subdev(&svid->sd);
unregister_v4l2:
    v4l2_device_unregister(&svid->v4l2_dev);
free:
    kfree(svid);
fail:
    return NULL;
}

void siasun_v4l2_free_instance(struct siasun_pcie_device *pdev, int index)
{
    struct siasun_video_dev *svid = pdev->siasun_videos[index];

    video_unregister_device(&svid->video_dev);
    v4l2_device_put(&svid->v4l2_dev);
    kfree(svid);
    svid = NULL;
}
