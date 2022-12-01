/* SPDX-License-Identifier: GPL-2.0 */
/* Linux fpga configure Driver header file */

#ifndef __FPGA_CDEV_H__
#define __FPGA_CDEV_H__

#include "sensor_fusion_main.h"
#include <linux/cdev.h>

struct fpga_cdev
{
    struct cdev cdev;
    void __iomem *pcie_base;
    void __iomem *vip_schedule;
    void __iomem *timestamp;
    dev_t devt;
    struct class *cdev_class;
    queue *lidar_q;
    queue *camera0_q;
    queue *camera1_q;
    queue *camera2_q;
    queue *camera3_q;
};

int fpga_cdev_init(struct siasun_pcie_device *siasun_pcie_dev);
void fpga_cdev_exit(void);

#endif
