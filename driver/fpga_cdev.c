/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/device.h>
#include <linux/device.h>

#include "fpga_cdev.h"
#include "fpga_schedule.h"

struct fpga_cdev *fpga = NULL;

static int fpga_cdev_open(struct inode *inode, struct file *file)
{
    static struct fpga_cdev *pdev;
    DPRINTK(KERN_DEBUG "%s@%d\n", __FUNCTION__, __LINE__);
    pdev = container_of(inode->i_cdev, struct fpga_cdev, cdev);
    file->private_data = pdev;
    return 0;
}

static int fpga_cdev_release(struct inode *inode, struct file *file)
{
    DPRINTK(KERN_DEBUG "%s@%d\n", __FUNCTION__, __LINE__);
    file->private_data = NULL;
    return 0;
}

static long fpga_cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    static struct fpga_cdev *pdev;

    pdev = file->private_data;

    switch (cmd)
    {
        case BUF_SYNS_STOP:
            fpga_set_buf_control(pdev, 0);
            break;

        case BUF_SYNS_START:
            fpga_set_buf_control(pdev, 1);
            break;

        case BUF_CONFIG:
        {
            BufCfg cfg;
            if (copy_from_user(&cfg, (BufCfg *)arg, sizeof(BufCfg)))
            {
                printk(KERN_ERR "%s@%d copy_from_user failled\n.", __FUNCTION__, __LINE__);
                return -EFAULT;
            }
            fpga_buffer_config(pdev, cfg);
            break;
        }

        case TRIGGER_MODE_SET:
        {
            unsigned char mode;
            if (copy_from_user(&mode, (unsigned char *)arg, sizeof(mode)))
            {
                printk(KERN_ERR "%s@%d copy_from_user failled\n.", __FUNCTION__, __LINE__);
                return -EFAULT;
            }
            fpga_set_camera_target_mode(pdev, mode);
            break;
        }

        case FPGA_TIME_SET:
        {
            TimeStampCfg timecfg;
            if (copy_from_user(&timecfg, (unsigned short *)arg, sizeof(TimeStampCfg)))
            {
                printk(KERN_ERR "%s@%d copy_from_user failled\n.", __FUNCTION__, __LINE__);
                return -EFAULT;
            }
            fpga_time_set(pdev, timecfg);
            break;
        }

        case FPGA_TIME_GET:
        {
            TimeStamp time;
            fpga_time_get(pdev, &time);

            if (copy_to_user((TimeStamp *)arg, &time, sizeof(time)))
            {
                printk(KERN_ERR "%s@%d: copy_to_user failled\n.", __FUNCTION__, __LINE__);
                return -EFAULT;
            }
            break;
        }

        case ANGLE_FPS_SET:
        {
            SyncCfg sync;
            if (copy_from_user(&sync, (SyncCfg *)arg, sizeof(SyncCfg)))
            {
                printk(KERN_ERR "%s@%d copy_from_user failled\n.", __FUNCTION__, __LINE__);
                return -EFAULT;
            }
            fpga_set_camera_angles(pdev, sync);
            break;
        }

        case ANGLE_FPS_GET:
        {
            SyncCfg sync;
            fpga_get_current_angle_fps(pdev, &sync);
            if (copy_to_user((SyncCfg *)arg, &sync, sizeof(SyncCfg)))
            {
                printk(KERN_ERR "%s@%d: copy_to_user failled\n.", __FUNCTION__, __LINE__);
                return -EFAULT;
            }
            break;
        }

        case IRQ_MASK_CONFIG:
        {
            unsigned int mask;
            if (copy_from_user(&mask, (unsigned int *)arg, sizeof(mask)))
            {
                printk(KERN_ERR "%s@%d copy_from_user failled\n.", __FUNCTION__, __LINE__);
                return -EFAULT;
            }
            irq_mask_config(pdev, mask);
            if (mask & 0x1)
            {
                lidar_irq_status_clean(pdev);
                siasun_queue_clear(pdev->lidar_q);
            }
            if (mask & 0x2)
            {
                camera0_irq_status_clean(pdev);
                siasun_queue_clear(pdev->camera0_q);
            }
            if (mask & 0x4)
            {
                camera1_irq_status_clean(pdev);
                siasun_queue_clear(pdev->camera1_q);
            }
            if (mask & 0x8)
            {
                camera2_irq_status_clean(pdev);
                siasun_queue_clear(pdev->camera2_q);
            }
            if (mask & 0x10)
            {
                camera3_irq_status_clean(pdev);
                siasun_queue_clear(pdev->camera3_q);
            }
            break;
        }

        default:
            printk(KERN_ERR "fpga_cdev ioctl err\n");
            break;
    }

    return 0;
}

struct file_operations fpga_cdev_fops = {
    .owner = THIS_MODULE,
    .open = fpga_cdev_open,
    .release = fpga_cdev_release,
    .unlocked_ioctl = fpga_cdev_ioctl,
};

int fpga_cdev_init(struct siasun_pcie_device *siasun_pcie_dev)
{
    int ret;
    fpga = (struct fpga_cdev *)kmalloc(sizeof(struct fpga_cdev), GFP_KERNEL);

    fpga->pcie_base = siasun_pcie_dev->siasun_sf_dev[1]->base_addr[4];
    fpga->vip_schedule = siasun_pcie_dev->siasun_sf_dev[0]->base_addr[4] + TERASIC_VIP_SCHEDULE_0_BASE;
    fpga->timestamp = siasun_pcie_dev->siasun_sf_dev[0]->base_addr[4] + TIME_STAMP_0_BASE;
    fpga->lidar_q = siasun_pcie_dev->lidar_q;
    fpga->camera0_q = siasun_pcie_dev->camera0_q;
    fpga->camera1_q = siasun_pcie_dev->camera1_q;
    fpga->camera2_q = siasun_pcie_dev->camera2_q;
    fpga->camera3_q = siasun_pcie_dev->camera3_q;

    ret = alloc_chrdev_region(&(fpga->devt), 0, 1, "fpga_cdev");
    if (ret)
    {
        printk(KERN_ERR "ERROR:%s@%d alloc char device failed\n", __FUNCTION__, __LINE__);
        goto fail_malloc;
    }

    cdev_init(&fpga->cdev, &fpga_cdev_fops);

    ret = cdev_add(&fpga->cdev, fpga->devt, 1);
    if (ret)
    {
        printk(KERN_ERR "ERROR:%s@%d add device failed\n", __FUNCTION__, __LINE__);
        goto fail_malloc;
    }

    fpga->cdev_class = class_create(THIS_MODULE, "fpga_cdev");
    if (IS_ERR(fpga->cdev_class))
    {
        ret = PTR_ERR(fpga->cdev_class);
        printk(KERN_ERR "ERROR:%s@%d create class failed\n", __FUNCTION__, __LINE__);
        goto fail_malloc;
    }

    device_create(fpga->cdev_class, NULL, fpga->devt, 0, "fpga_cdev");
    siasun_pcie_dev->fpga_cfg = fpga;

    return 0;

fail_malloc:
    unregister_chrdev_region(fpga->devt, 1);
    kfree(fpga);
    return ret;
}

void fpga_cdev_exit(void)
{
    if (!fpga)
    {
        return;
    }

    device_destroy(fpga->cdev_class, fpga->devt);
    class_destroy(fpga->cdev_class);
    cdev_del(&fpga->cdev);
    unregister_chrdev_region(fpga->devt, 1);
    kfree(fpga);
}
