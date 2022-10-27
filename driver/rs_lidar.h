/* SPDX-License-Identifier: GPL-2.0 */
/* Linux RS-Lidar-16 Driver header file */

#ifndef __RS_LIDAR_H__
#define __RS_LIDAR_H__

#include <linux/cdev.h>
#include "sensor_fusion_main.h"
#include "sensor_fusion_queue.h"

struct lidar_cdev
{
    struct cdev cdev;
    void __iomem *base_addr;
    void __iomem *mac_base;
    dev_t devt;
    struct class *cdev_class;
    wait_queue_head_t wait;
    unsigned int data_ready;
    unsigned char lidar_pkgs;
    unsigned int phy_offset;
    unsigned char *viraddr;
};

int siasun_lidar_init(struct siasun_pcie_device *siasun_pcie_dev);
void siasun_lidar_exit(void);

#endif