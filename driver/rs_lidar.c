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
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/device.h>
#include <linux/device.h>
#include <linux/time64.h>
#include <linux/time.h>

#include "rs_lidar.h"
#include "lidar_module.h"
#include "sensor_fusion_register.h"

#define DEV_NAME "rs16_dev"

struct lidar_cdev *lidar = NULL;

static int siasun_lidar_open(struct inode *inode, struct file *file)
{
    struct lidar_cdev *pdev;

    DPRINTK(KERN_DEBUG "%s@%d\n", __FUNCTION__, __LINE__);
    pdev = container_of(inode->i_cdev, struct lidar_cdev, cdev);
    file->private_data = pdev;

    return 0;
}

static int siasun_lidar_release(struct inode *inode, struct file *file)
{
    DPRINTK(KERN_DEBUG "%s@%d\n", __FUNCTION__, __LINE__);
    file->private_data = NULL;

    return 0;
}

static long siasun_lidar_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct lidar_cdev *pdev;
    pdev = file->private_data;

    switch (cmd)
    {

        case UCWP_SET_MOTOR_SPEED:
        {
            unsigned int speed;

            if (copy_from_user(&speed, (unsigned int *)arg, sizeof(speed)))
            {
                printk(KERN_ERR "%s @UCWP_SET_MOTOR_SPEED: copy_from_user failled\n.", __FUNCTION__);
                return -EFAULT;
            }
            lidar_set_motor_speed(pdev, speed);
            lidar_send_ucwp(pdev);
            lidar_set_motor_speed(pdev, speed);
            lidar_send_ucwp(pdev);
            break;
        }

        case UCWP_SET_ETH:
        {
            RSEthNet eth;

            if (copy_from_user(&eth, (RSEthNet *)arg, sizeof(eth)))
            {
                printk(KERN_ERR "%s @UCWP_SET_ETH: copy_from_user failled\n.", __FUNCTION__);
                return -EFAULT;
            }
            lidar_set_ethnet(pdev, eth);
            lidar_send_ucwp(pdev);
            lidar_set_ethnet(pdev, eth);
            lidar_send_ucwp(pdev);
            break;
        }

        case PIB_SET_ETH:
        {
            PIBEthNet eth;
            if (copy_from_user(&eth, (PIBEthNet *)arg, sizeof(eth)))
            {
                printk(KERN_ERR "%s @PIB_SET_ETH: copy_from_user failled\n.", __FUNCTION__);
                return -EFAULT;
            }
            lidar_set_pib_eth(pdev, eth);
            break;
        }

        case UCWP_SET_FOV:
        {
            RSFOV fov_set;

            if (copy_from_user(&fov_set, (RSFOV *)arg, sizeof(fov_set)))
            {
                printk(KERN_ERR "%s @UCWP_SET_FOV: copy_from_user failled\n.", __FUNCTION__);
                return -EFAULT;
            }
            lidar_set_fov_set(pdev, fov_set);
            lidar_send_ucwp(pdev);
            lidar_set_fov_set(pdev, fov_set);
            lidar_send_ucwp(pdev);
            break;
        }

        case UCWP_SET_UTC_TIME:
        {
            RSTimestampYMD time;

            if (copy_from_user(&time, (RSTimestampYMD *)arg, sizeof(time)))
            {
                printk(KERN_ERR "%s @UCWP_SET_UTC_TIME: copy_from_user failled\n.", __FUNCTION__);
                return -EFAULT;
            }
            lidar_set_utc_time(pdev, time);
            lidar_send_ucwp(pdev);
            lidar_set_utc_time(pdev, time);
            lidar_send_ucwp(pdev);
            break;
        }

        case UCWP_SET_MOT_PHASE:
        {
            unsigned short mot_phase;
            if (copy_from_user(&mot_phase, (unsigned short *)arg, sizeof(mot_phase)))
            {
                printk(KERN_ERR "%s @UCWP_SET_MOT_PHASE: copy_from_user failled\n.", __FUNCTION__);
                return -EFAULT;
            }
            lidar_set_mot_phase(pdev, mot_phase);
            lidar_send_ucwp(pdev);
            lidar_set_mot_phase(pdev, mot_phase);
            lidar_send_ucwp(pdev);
            break;
        }

        case DIFOP_GET_PKG:
        {
            int i;
            RS16DifopPkt *difop = (RS16DifopPkt *)kzalloc(sizeof(RS16DifopPkt), GFP_KERNEL);

            difop->id = 0x555511115a00ffa5;
            difop->rpm = lidar_get_motor_speed(pdev);
            lidar_get_ethnet(pdev, &difop->eth);
            lidar_get_fov_set(pdev, &difop->fov);
            difop->reserved_0 = 0;
            difop->phase_lock_angle = lidar_get_mot_phase(pdev);
            lidar_get_frm_ver(pdev, &difop->version);
            memset(difop->reserved_1, 0, 242);
            lidar_get_serial_number(pdev, &difop->sn);
            difop->zero_cali = 0; 
            difop->echo_mode = lidar_get_echo_mode(pdev);
            difop->sw_ver = lidar_get_software_ver(pdev);
            lidar_get_utc_time(pdev, &difop->timestamp);
            lidar_get_status(pdev, &difop->status);
            memset(difop->reserved_2, 0, 11);
            lidar_get_diagno(pdev, &difop->diagno);
            memset(difop->gprmc, 0, 86); 
            memset(difop->reserved_3, 0, 697);

            for (i = 0; i < 16; i++)
            {
                lidar_get_cor_vert_ang(pdev, i, &difop->ver_angle_cali[i]);
            }
            memset(difop->reserved_4, 0, 33);
            difop->tail = 0x0F0F;

            if (copy_to_user((RS16DifopPkt *)arg, difop, sizeof(RS16DifopPkt)))
            {
                printk(KERN_ERR "%s @DIFOP_GET_PKG: copy_to_user failled\n.", __FUNCTION__);
                return -EFAULT;
            }
            kfree(difop);
            break;
        }

        case LIDAR_INIT:
        {
            int rc;

            rc = lidar_init_ucwp(pdev);
            if (rc < 0)
            {
                printk(KERN_ERR "%s @LIDAR_INIT: lidar_init_ucwp failled\n.", __FUNCTION__);
                return rc;
            }
            lidar_send_ucwp(pdev);
            lidar_send_ucwp(pdev);
            break;
        }

        case LIDAR_TARGET_ANGLE_SET:
        {
            unsigned int angle;
            
            if (copy_from_user(&angle, (unsigned int *)arg, sizeof(angle)))
            {
                printk(KERN_ERR "%s @UCWP_SET_MOT_PHASE: copy_from_user failled\n.", __FUNCTION__);
                return -EFAULT;
            }
            lidar_set_target_angle(pdev, angle);
            break;
        }

        case LIDAR_TARGET_ANGLE_GET:
        {
            unsigned int angle;

            angle = lidar_get_target_angle(pdev) / 100;
            if (copy_to_user((unsigned int *)arg, &angle, sizeof(unsigned int)))
            {
                printk(KERN_ERR "%s @LIDAR_TARGET_ANGLE_GET: copy_to_user failled\n.", __FUNCTION__);
                return -EFAULT;
            }
            break;
        }

        case MMAP_RHY_OFFSET:
            if (copy_to_user((unsigned int *)arg, &pdev->phy_offset, sizeof(unsigned int)))
            {
                printk(KERN_ERR "%s @MMAP_RHY_OFFSET: copy_to_user failled\n.", __FUNCTION__);
                return -EFAULT;
            }
            break;

        case MSOP_GET_PKG:
            memset(pdev->viraddr, 0, 300 * 1024);
            pdev->data_ready = 0;
            wait_event_interruptible(pdev->wait, (pdev->data_ready == 1));

            if (copy_to_user((unsigned char *)arg, &pdev->lidar_pkgs, sizeof(unsigned char)))
            {
                printk(KERN_ERR "%s @DIFOP_GET_PKG: copy_to_user failled\n.", __FUNCTION__);
                return -EFAULT;
            }
            break;

        default:
            printk(KERN_DEBUG "%s IOCTL cmd error\n", __FUNCTION__);
            break;
        }

    return 0;
}

static int siasun_lidar_mmap(struct file *file, struct vm_area_struct *vma)
{
    int ret = 0;
    unsigned long size;
    DPRINTK(KERN_DEBUG "%s@%d\n", __FUNCTION__, __LINE__);
    vma->vm_flags |= VM_IO;
    size = vma->vm_end - vma->vm_start;
    remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size, vma->vm_page_prot);

    return ret;
}

struct file_operations siasun_lidar_fops = {
    .owner = THIS_MODULE,
    .open = siasun_lidar_open,
    .release = siasun_lidar_release,
    .unlocked_ioctl = siasun_lidar_ioctl,
    .mmap = siasun_lidar_mmap,
};

int siasun_lidar_init(struct siasun_pcie_device *siasun_pcie_dev)
{
    int ret;
    lidar = (struct lidar_cdev *)kmalloc(sizeof(struct lidar_cdev), GFP_KERNEL);

    lidar->base_addr = siasun_pcie_dev->siasun_sf_dev[0]->base_addr[4] + LIDAR_0_LIDAR_UDP_PARSER_0_BASE;
    lidar->mac_base = siasun_pcie_dev->siasun_sf_dev[0]->base_addr[4] + LIDAR_0_MAC_FOR_LIDAR_0_BASE;

    ret = alloc_chrdev_region(&(lidar->devt), 0, 1, DEV_NAME);
    if (ret)
    {
        printk(KERN_ERR "ERROR:%s@%d\n", __FUNCTION__, __LINE__);
        goto fail_malloc;
    }

    cdev_init(&lidar->cdev, &siasun_lidar_fops);
    ret = cdev_add(&lidar->cdev, lidar->devt, 1);
    if (ret)
    {
        printk(KERN_ERR "ERROR:%s@%d\n", __FUNCTION__, __LINE__);
        goto fail_malloc;
    }

    lidar->cdev_class = class_create(THIS_MODULE, DEV_NAME);
    if (IS_ERR(lidar->cdev_class))
    {
        ret = PTR_ERR(lidar->cdev_class);
        printk(KERN_ERR "ERROR:%s@%d\n", __FUNCTION__, __LINE__);
        goto fail_malloc;
    }

    device_create(lidar->cdev_class, NULL, lidar->devt, 0, DEV_NAME);

    init_waitqueue_head(&lidar->wait);
    lidar->data_ready = 0;
    lidar->lidar_pkgs = 0;
    lidar->phy_offset = siasun_pcie_dev->lidar.physaddr;
    lidar->viraddr = siasun_pcie_dev->lidar.viraddr;

    siasun_pcie_dev->lidar_dev = lidar;

    return 0;

fail_malloc:
    unregister_chrdev_region(lidar->devt, 1);
    kfree(lidar);

    return ret;
}

void siasun_lidar_exit(void)
{
    if (!lidar)
    {
        return;
    }
    device_destroy(lidar->cdev_class, lidar->devt);
    class_destroy(lidar->cdev_class);
    cdev_del(&lidar->cdev);
    unregister_chrdev_region(lidar->devt, 1);
    kfree(lidar);
}
