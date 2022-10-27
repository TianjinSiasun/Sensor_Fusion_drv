/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SIASUN_VL42_CORE_H__
#define __SIASUN_V4L2_CORE_H__

#include <linux/fb.h>
#include <linux/workqueue.h>
#include <linux/pci.h>
#include <media/cec.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ctrls.h>
#include <media/tpg/v4l2-tpg.h>

#include "sensor_fusion_main.h"

/**
 * struct siasun_video_dev - camera device information
 * @index: the num of camera
 * @bus_num: I2C bus num
 * @video_dev: video_dev of camera
 * @v4l2_dev: v4l2_dev of camera
 * @sd: subdev of camera
 * @config_base: the base addr of pcie
 * @pci_dev: Associated pcie device
 * @streaming: express the camera is capturing or not
 * @phys_base_start: the first section physical address of camera data space
 * @vir_base_start: the first section virtual address of camera data space
 * @buf_len: the first section length of camera data space
 * @phys_base_start_h:the second section physical address of camera data space
 * @vir_base_start_h: the second section virtual address of camera data space
 * @buf_len_h: the second section length of camera data space
 * @fmt: picture format
 * @lock: video device lock for v4l2
 * @wait: the wait queue head of camera
 * @data_ready: the flag of wait queue
 */
struct siasun_video_dev
{
    unsigned int            index;
    unsigned int            bus_num;
    struct video_device     video_dev;
    struct v4l2_device      v4l2_dev;
    struct v4l2_subdev      sd;
    void __iomem *          config_base;
    struct pci_dev *        pci_dev;
    unsigned char           streaming;

    phys_addr_t             phys_base_start;
    unsigned char*          vir_base_start;
    unsigned int            buf_len;

    phys_addr_t             phys_base_start_h;
    unsigned char*          vir_base_start_h;
    unsigned int            buf_len_h;

    struct v4l2_format      fmt;
    struct mutex            lock;

    wait_queue_head_t       wait;
    unsigned int            data_ready;
};

struct siasun_video_dev *siasun_v4l2_create_instance(struct siasun_pcie_device *pdev, int index);
void siasun_v4l2_free_instance(struct siasun_pcie_device *pdev, int index);

#endif /* __SIASUN_V4L2_CORE_H__ */
