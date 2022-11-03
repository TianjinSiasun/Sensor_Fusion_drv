/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/time64.h>
#include <linux/pci.h>

#include "sensor_fusion_v4l2.h"
#include "sensor_fusion_main.h"
#include "i2c_adapter_siasun.h"
#include "rs_lidar.h"
#include "fpga_cdev.h"

#define SENSORS_FUSION_DRIVER_NAME "sf_drv"

static const struct pci_device_id altera_pcie_id[] = {
    {PCI_DEVICE(ALTERA_PCIE_VID, ALTERA_PCIE_DID_L)},
    {PCI_DEVICE(ALTERA_PCIE_VID, ALTERA_PCIE_DID_H)},
    {
        0,
    },
};

MODULE_DEVICE_TABLE(pci, altera_pcie_id);

struct siasun_pcie_device siasun_pcie_dev;

static int set_write_table(struct dma_descriptor *wr_desc, unsigned long long src,
                           dma_addr_t dst, unsigned int len, unsigned int id)
{
    wr_desc->src_addr_udw = (src >> 32) & 0xffffffff;
    wr_desc->src_addr_ldw = src & 0xffffffff;
    wr_desc->dest_addr_udw = (dst >> 32) & 0xffffffff;
    wr_desc->dest_addr_ldw = dst & 0xffffffff;
    wr_desc->ctl_dma_len = len | (id << 18);
    wr_desc->reserved[0] = 0;
    wr_desc->reserved[1] = 0;
    wr_desc->reserved[2] = 0;

    DPRINTK("set table: src u:%#x, l:%#x  dst u:%#x, l:%#x, ctl len = %#x", wr_desc->src_addr_udw, wr_desc->src_addr_ldw,
            wr_desc->dest_addr_udw, wr_desc->dest_addr_ldw, wr_desc->ctl_dma_len);

    return 0;
}

static void task_handler(unsigned long data)
{
    unsigned int i = 0;
    unsigned int buf_index;
    unsigned int write_127 = 0;
    unsigned int last_id = 0;
    unsigned int table_index = 0;
    unsigned char dma_flag = 0;
    unsigned int timeout = 0x2000000;
    struct siasun_pcie_device *siasun_dev = (struct siasun_pcie_device *)data;

    DPRINTK(" task handler \n");

    if (siasun_dev->dma_ready)
    {
        dma_flag = 0;
        siasun_dev->dma_ready = 0;

        for (i = 0; i < ALTERA_DMA_DESCRIPTOR_NUM; i++)
        {
            siasun_dev->table_virt_addr->header.flags[i] = 0;
        }
        wmb();

        last_id = readl(siasun_dev->dma_base + ALTERA_LITE_DMA_WR_LAST_PTR);
        DPRINTK("last id = %#x\n", last_id);

        if ((last_id == 0xff) || (last_id == 127))
            table_index = 0;
        else
            table_index = last_id + 1;
        DPRINTK("start table_index = %d\n", table_index);
        if ((!siasun_queue_empty(siasun_pcie_dev.lidar_q)) && (!siasun_dev->lidar_dev->data_ready))
        {
            buf_index = siasun_queue_del(siasun_pcie_dev.lidar_q);
            DPRINTK("liadr index = %d\n", buf_index);
            set_write_table(&siasun_dev->table_virt_addr->descriptors[table_index],
                            FPGA_DDR3_BASE + buf_index * FPGA_BUF_SIZE + FPGA_LIDAR_OFFSET,
                            siasun_dev->lidar.busaddr, FPGA_LIDAR_MOVE_SIZE, table_index);
            if (table_index == 127)
            {
                table_index = 0;
                write_127 = 1;
            }
            else
            {
                table_index++;
            }
            dma_flag |= 0x1;
        }

        if ((!siasun_queue_empty(siasun_pcie_dev.camera0_q)) && (!siasun_dev->siasun_videos[0]->data_ready))
        {
            buf_index = siasun_queue_del(siasun_pcie_dev.camera0_q);
            DPRINTK("camera0 start buf_index = %d, table_index = %d\n", buf_index, table_index);

            for (i = 0; i < 8; i++)
            {
                set_write_table(&siasun_dev->table_virt_addr->descriptors[table_index],
                                FPGA_DDR3_BASE + buf_index * FPGA_BUF_SIZE + FPGA_CAMERA0_OFFSET + i * FPGA_CAMERA_MOVE_OFFSET,
                                siasun_dev->camera0.busaddr1 + i * FPGA_CAMERA_MOVE_OFFSET, FPGA_CAMERA_MOVE_SIZE, table_index);

                if (table_index == 127)
                {
                    table_index = 0;
                    write_127 = 1;
                }
                else
                {
                    table_index++;
                }
            }

            for (i = 0; i < 3; i++)
            {
                set_write_table(&siasun_dev->table_virt_addr->descriptors[table_index],
                                FPGA_DDR3_BASE + buf_index * FPGA_BUF_SIZE + (i + 8) * FPGA_CAMERA_MOVE_OFFSET,
                                siasun_dev->camera0.busaddr2 + i * FPGA_CAMERA_MOVE_OFFSET, FPGA_CAMERA_MOVE_SIZE, table_index);

                if (table_index == 127)
                {
                    table_index = 0;
                    write_127 = 1;
                }
                else
                {
                    table_index++;
                }
            }

            dma_flag |= 0x2;
        }

        if ((!siasun_queue_empty(siasun_pcie_dev.camera1_q)) && (!siasun_dev->siasun_videos[1]->data_ready))
        {
            buf_index = siasun_queue_del(siasun_pcie_dev.camera1_q);
            DPRINTK("camera1 start buf_index = %d, table_index = %d\n", buf_index, table_index);

            for (i = 0; i < 8; i++)
            {
                set_write_table(&siasun_dev->table_virt_addr->descriptors[table_index],
                                FPGA_DDR3_BASE + buf_index * FPGA_BUF_SIZE + FPGA_CAMERA1_OFFSET + i * FPGA_CAMERA_MOVE_OFFSET,
                                siasun_dev->camera1.busaddr1 + i * FPGA_CAMERA_MOVE_OFFSET, FPGA_CAMERA_MOVE_SIZE, table_index);

                if (table_index == 127)
                {
                    table_index = 0;
                    write_127 = 1;
                }
                else
                {
                    table_index++;
                }
            }

            for (i = 0; i < 3; i++)
            {
                set_write_table(&siasun_dev->table_virt_addr->descriptors[table_index],
                                FPGA_DDR3_BASE + buf_index * FPGA_BUF_SIZE + FPGA_CAMERA1_OFFSET + (i + 8) * FPGA_CAMERA_MOVE_OFFSET,
                                siasun_dev->camera1.busaddr2 + i * FPGA_CAMERA_MOVE_OFFSET, FPGA_CAMERA_MOVE_SIZE, table_index);

                if (table_index == 127)
                {
                    table_index = 0;
                    write_127 = 1;
                }
                else
                {
                    table_index++;
                }
            }

            dma_flag |= 0x4;
        }

        if ((!siasun_queue_empty(siasun_pcie_dev.camera2_q)) && (!siasun_dev->siasun_videos[2]->data_ready))
        {
            buf_index = siasun_queue_del(siasun_pcie_dev.camera2_q);
            DPRINTK("camera2 start buf_index = %d, table_index = %d\n", buf_index, table_index);

            for (i = 0; i < 8; i++)
            {
                set_write_table(&siasun_dev->table_virt_addr->descriptors[table_index],
                                FPGA_DDR3_BASE + buf_index * FPGA_BUF_SIZE + FPGA_CAMERA2_OFFSET + i * FPGA_CAMERA_MOVE_OFFSET,
                                siasun_dev->camera2.busaddr1 + i * FPGA_CAMERA_MOVE_OFFSET, FPGA_CAMERA_MOVE_SIZE, table_index);

                if (table_index == 127)
                {
                    table_index = 0;
                    write_127 = 1;
                }
                else
                {
                    table_index++;
                }
            }

            for (i = 0; i < 3; i++)
            {
                set_write_table(&siasun_dev->table_virt_addr->descriptors[table_index],
                                FPGA_DDR3_BASE + buf_index * FPGA_BUF_SIZE + FPGA_CAMERA2_OFFSET + (i + 8) * FPGA_CAMERA_MOVE_OFFSET,
                                siasun_dev->camera2.busaddr2 + i * FPGA_CAMERA_MOVE_OFFSET, FPGA_CAMERA_MOVE_SIZE, table_index);

                if (table_index == 127)
                {
                    table_index = 0;
                    write_127 = 1;
                }
                else
                {
                    table_index++;
                }
            }
            dma_flag |= 0x8;
        }

        if ((!siasun_queue_empty(siasun_pcie_dev.camera3_q)) && (!siasun_dev->siasun_videos[3]->data_ready))
        {
            buf_index = siasun_queue_del(siasun_pcie_dev.camera3_q);
            DPRINTK("camera3 start buf_index = %d, table_index = %d\n", buf_index, table_index);

            for (i = 0; i < 8; i++)
            {
                set_write_table(&siasun_dev->table_virt_addr->descriptors[table_index],
                                FPGA_DDR3_BASE + buf_index * FPGA_BUF_SIZE + FPGA_CAMERA3_OFFSET + i * FPGA_CAMERA_MOVE_OFFSET,
                                siasun_dev->camera3.busaddr1 + i * FPGA_CAMERA_MOVE_OFFSET, FPGA_CAMERA_MOVE_SIZE, table_index);

                if (table_index == 127)
                {
                    table_index = 0;
                    write_127 = 1;
                }
                else
                {
                    table_index++;
                }
            }

            for (i = 0; i < 3; i++)
            {
                set_write_table(&siasun_dev->table_virt_addr->descriptors[table_index],
                                FPGA_DDR3_BASE + buf_index * FPGA_BUF_SIZE + FPGA_CAMERA3_OFFSET + (i + 8) * FPGA_CAMERA_MOVE_OFFSET,
                                siasun_dev->camera3.busaddr2 + i * FPGA_CAMERA_MOVE_OFFSET, FPGA_CAMERA_MOVE_SIZE, table_index);

                if (table_index == 127)
                {
                    table_index = 0;
                    write_127 = 1;
                }
                else
                {
                    table_index++;
                }
            }
            dma_flag |= 0x10;
        }

        if ((!siasun_queue_empty(siasun_pcie_dev.camera4_q)) && (!siasun_dev->siasun_videos[4]->data_ready))
        {
            buf_index = siasun_queue_del(siasun_pcie_dev.camera4_q);

            for (i = 0; i < 8; i++)
            {
                set_write_table(&siasun_dev->table_virt_addr->descriptors[table_index],
                                FPGA_DDR3_BASE + buf_index * FPGA_BUF_SIZE + FPGA_CAMERA4_OFFSET + i * FPGA_CAMERA_MOVE_OFFSET,
                                siasun_dev->camera3.busaddr1 + i * FPGA_CAMERA_MOVE_OFFSET, FPGA_CAMERA_MOVE_SIZE, table_index);

                if (table_index == 127)
                {
                    table_index = 0;
                    write_127 = 1;
                }
                else
                {
                    table_index++;
                }
            }

            for (i = 0; i < 3; i++)
            {
                set_write_table(&siasun_dev->table_virt_addr->descriptors[table_index],
                                FPGA_DDR3_BASE + buf_index * FPGA_BUF_SIZE + FPGA_CAMERA4_OFFSET + (i + 8) * FPGA_CAMERA_MOVE_OFFSET,
                                siasun_dev->camera3.busaddr2 + i * FPGA_CAMERA_MOVE_OFFSET, FPGA_CAMERA_MOVE_SIZE, table_index);

                if (table_index == 127)
                {
                    table_index = 0;
                    write_127 = 1;
                }
                else
                {
                    table_index++;
                }
            }
            dma_flag |= 0x20;
        }

        if ((!siasun_queue_empty(siasun_pcie_dev.camera5_q)) && (!siasun_dev->siasun_videos[5]->data_ready))
        {
            buf_index = siasun_queue_del(siasun_pcie_dev.camera5_q);

            for (i = 0; i < 8; i++)
            {
                set_write_table(&siasun_dev->table_virt_addr->descriptors[table_index],
                                FPGA_DDR3_BASE + buf_index * FPGA_BUF_SIZE + FPGA_CAMERA5_OFFSET + i * FPGA_CAMERA_MOVE_OFFSET,
                                siasun_dev->camera3.busaddr1 + i * FPGA_CAMERA_MOVE_OFFSET, FPGA_CAMERA_MOVE_SIZE, table_index);

                if (table_index == 127)
                {
                    table_index = 0;
                    write_127 = 1;
                }
                else
                {
                    table_index++;
                }
            }

            for (i = 0; i < 3; i++)
            {
                set_write_table(&siasun_dev->table_virt_addr->descriptors[table_index],
                                FPGA_DDR3_BASE + buf_index * FPGA_BUF_SIZE + FPGA_CAMERA5_OFFSET + (i + 8) * FPGA_CAMERA_MOVE_OFFSET,
                                siasun_dev->camera3.busaddr2 + i * FPGA_CAMERA_MOVE_OFFSET, FPGA_CAMERA_MOVE_SIZE, table_index);

                if (table_index == 127)
                {
                    table_index = 0;
                    write_127 = 1;
                }
                else
                {
                    table_index++;
                }
            }
            dma_flag |= 0x40;
        }

        if (dma_flag)
        {
            if (table_index == 0)
            {
                last_id = 127;
                write_127 = 0;
            }
            else
            {
                last_id = table_index - 1;
            }

            if (write_127)
            {
                writel(127, siasun_dev->dma_base + ALTERA_LITE_DMA_WR_LAST_PTR);
            }
            writel(last_id, siasun_dev->dma_base + ALTERA_LITE_DMA_WR_LAST_PTR);

            while (1)
            {
                if (siasun_dev->table_virt_addr->header.flags[last_id])
                {
                    break;
                }

                if (timeout == 0)
                {
                    printk("wirte DMA time out\n");
                    break;
                }
                timeout--;
                cpu_relax();
            }

            if (dma_flag & 0x1)
            {
                siasun_dev->lidar_dev->data_ready = 1;
                wake_up_interruptible(&siasun_dev->lidar_dev->wait);
            }

            if (dma_flag & 0x2)
            {
                siasun_dev->siasun_videos[0]->data_ready = 1;
                wake_up_interruptible(&siasun_dev->siasun_videos[0]->wait);
            }

            if (dma_flag & 0x4)
            {
                siasun_dev->siasun_videos[1]->data_ready = 1;
                wake_up_interruptible(&siasun_dev->siasun_videos[1]->wait);
            }
            if (dma_flag & 0x8)
            {
                siasun_dev->siasun_videos[2]->data_ready = 1;
                wake_up_interruptible(&siasun_dev->siasun_videos[2]->wait);
            }
            if (dma_flag & 0x10)
            {
                siasun_dev->siasun_videos[3]->data_ready = 1;
                wake_up_interruptible(&siasun_dev->siasun_videos[3]->wait);
            }
            if (dma_flag & 0x20)
            {
                siasun_dev->siasun_videos[4]->data_ready = 1;
                wake_up_interruptible(&siasun_dev->siasun_videos[4]->wait);
            }
            if (dma_flag & 0x40)
            {
                siasun_dev->siasun_videos[5]->data_ready = 1;
                wake_up_interruptible(&siasun_dev->siasun_videos[5]->wait);
            }
        }

        siasun_dev->dma_ready = 1;
    }
}

irqreturn_t siasun_irq(int irq, void *dev_id)
{
    unsigned int irq_status;
    int ret = IRQ_NONE;
    unsigned int lidar_stat;
    unsigned int val;
    unsigned int buf_index;
    struct siasun_pcie_device *siasun_dev = (struct siasun_pcie_device *)dev_id;

    if (siasun_dev == NULL)
    {
        DPRINTK("siasun_dev is NULL\n");
        return ret;
    }

    irq_status = readl(siasun_dev->irq_base + PCIE_CRA_IRQ_STATUS);
    writel(0x0, siasun_dev->irq_base + PCIE_CRA_IRQ_ENABLE);
    if (irq_status != 0x00)
    {
        DPRINTK("irq_stat = %#x\n", irq_status);

        if (irq_status & PCIE_LIDAR_IRQ)
        {
            lidar_stat = readl(siasun_dev->irq_base + PCIE_LIDAR_IRQ_STATUS);
            val = (lidar_stat >> 24) & 0xff;
            if (val == 0)
            {
                buf_index = 15;
            }
            else
            {
                buf_index = val - 1;
            }

            siasun_dev->lidar_dev->lidar_pkgs = (lidar_stat >> 16) & 0xff;
            DPRINTK("add buf_index = %d, lidar_pkgs = %u, lidar stat:%#x\n", buf_index, siasun_dev->lidar_dev->lidar_pkgs, lidar_stat);

            siasun_queue_add(siasun_pcie_dev.lidar_q, buf_index);

            writel(0x01, siasun_dev->irq_base + PCIE_LIDAR_IRQ_STATUS);
            ret = IRQ_HANDLED;
        }

        if (irq_status & PCIE_CAMERA0_IRQ)
        {
            buf_index = (readl(siasun_dev->irq_base + PCIE_CAMERA0_IRQ_STATUS) >> 24) & 0xff;

            siasun_queue_add(siasun_pcie_dev.camera0_q, buf_index);

            writel(0x01, siasun_dev->irq_base + PCIE_CAMERA0_IRQ_STATUS);
            ret = IRQ_HANDLED;
        }

        if (irq_status & PCIE_CAMERA1_IRQ)
        {
            buf_index = (readl(siasun_dev->irq_base + PCIE_CAMERA1_IRQ_STATUS) >> 24) & 0xff;

            siasun_queue_add(siasun_pcie_dev.camera1_q, buf_index);

            writel(0x01, siasun_dev->irq_base + PCIE_CAMERA1_IRQ_STATUS);
            ret = IRQ_HANDLED;
        }

        if (irq_status & PCIE_CAMERA2_IRQ)
        {
            buf_index = (readl(siasun_dev->irq_base + PCIE_CAMERA2_IRQ_STATUS) >> 24) & 0xff;

            siasun_queue_add(siasun_pcie_dev.camera2_q, buf_index);

            writel(0x01, siasun_dev->irq_base + PCIE_CAMERA2_IRQ_STATUS);
            ret = IRQ_HANDLED;
        }

        if (irq_status & PCIE_CAMERA3_IRQ)
        {
            buf_index = (readl(siasun_dev->irq_base + PCIE_CAMERA3_IRQ_STATUS) >> 24) & 0xff;

            siasun_queue_add(siasun_pcie_dev.camera3_q, buf_index);

            writel(0x01, siasun_dev->irq_base + PCIE_CAMERA3_IRQ_STATUS);
            ret = IRQ_HANDLED;
        }

        if (irq_status & PCIE_CAMERA4_IRQ)
        {
            buf_index = (readl(siasun_dev->irq_base + PCIE_CAMERA4_IRQ_STATUS) >> 24) & 0xff;

            siasun_queue_add(siasun_pcie_dev.camera4_q, buf_index);

            writel(0x01, siasun_dev->irq_base + PCIE_CAMERA4_IRQ_STATUS);
            ret = IRQ_HANDLED;
        }

        if (irq_status & PCIE_CAMERA5_IRQ)
        {
            buf_index = (readl(siasun_dev->irq_base + PCIE_CAMERA5_IRQ_STATUS) >> 24) & 0xff;

            siasun_queue_add(siasun_pcie_dev.camera5_q, buf_index);

            writel(0x01, siasun_dev->irq_base + PCIE_CAMERA5_IRQ_STATUS);
            ret = IRQ_HANDLED;
        }

        tasklet_schedule(&siasun_dev->task);
    }

    writel(0x7f, siasun_dev->irq_base + PCIE_CRA_IRQ_ENABLE);

    return ret;
}

static int sf_search_and_create_pci_devices(struct pci_dev *pdev, struct sf_pcie_device *sf_pdev)
{
    int bar_channel, ret = 0;
    unsigned long bar_start, bar_end, bar_flags;

    for (bar_channel = 0; bar_channel < ALTERA_PCIE_BAR_NUM; bar_channel++)
    {
        bar_start = pci_resource_start(pdev, bar_channel);
        bar_end = pci_resource_end(pdev, bar_channel);
        bar_flags = pci_resource_flags(pdev, bar_channel);
        sf_pdev->bar_length[bar_channel] = pci_resource_len(pdev, bar_channel);
        printk(KERN_INFO "pcie bar[%d] start=0x%08lx end=0x%08lx flags=0x%08lx\n",
                            bar_channel, bar_start, bar_end, bar_flags);
        if (!sf_pdev->bar_length[bar_channel])
        {
            sf_pdev->base_addr[bar_channel] = NULL;
            continue;
        }

        sf_pdev->base_addr[bar_channel] = ioremap(bar_start, sf_pdev->bar_length[bar_channel]);
        if (!sf_pdev->base_addr[bar_channel])
        {
            dev_err(&pdev->dev, "ioremap failed\n");
            ret = -ENOMEM;
            goto err_out;
        }
    }

err_out:
    return ret;
}

static void sf_delete_pci_devices(struct pci_dev *pdev, struct sf_pcie_device *sf_pdev)
{
    int i;

    for (i = 0; i < ALTERA_PCIE_BAR_NUM; i++)
    {
        if (sf_pdev->base_addr[i])
        {
            pci_iounmap(pdev, sf_pdev->base_addr[i]);
            sf_pdev->base_addr[i] = NULL;
        }
    }
}

static int alloc_lidar_dma(struct pci_dev *pdev, struct lidar_buf_address *lidar)
{
    lidar->viraddr = pci_alloc_consistent(pdev, LIDAR_BUF_SIZE, &lidar->busaddr);
    if (!lidar->viraddr)
    {
        printk(KERN_ERR "lidar no dma mem\n");
        return -ENOMEM;
    }
    lidar->physaddr = virt_to_phys((void *)lidar->viraddr);

    memset(lidar->viraddr, 0, LIDAR_BUF_SIZE);
    printk(KERN_INFO "lidar dma mem alloc success, phy = %#llx\n", lidar->physaddr);

    return 0;
}

static void free_lidar_dma(struct pci_dev *pdev, struct lidar_buf_address *lidar)
{
    pci_free_consistent(pdev, LIDAR_BUF_SIZE, lidar->viraddr, lidar->busaddr);
    DPRINTK("lidar dma mem free\n");
}

static int alloc_camera_dma(struct pci_dev *pdev, struct camera_buf_address *camera)
{
    camera->viraddr1 = pci_alloc_consistent(pdev, CAMERA_BUF_SIZE1, &camera->busaddr1);
    if (!camera->viraddr1)
    {
        printk(KERN_ERR "camera no dma mem1\n");
        return -ENOMEM;
    }
    camera->physaddr1 = virt_to_phys((void *)camera->viraddr1);

    camera->viraddr2 = pci_alloc_consistent(pdev, CAMERA_BUF_SIZE2, &camera->busaddr2);
    if (!camera->viraddr2)
    {
        printk(KERN_ERR "camera no dma mem2\n");
        pci_free_consistent(pdev, CAMERA_BUF_SIZE1, camera->viraddr1, camera->busaddr1);
        return -ENOMEM;
    }
    camera->physaddr2 = virt_to_phys((void *)camera->viraddr2);

    memset(camera->viraddr1, 0, CAMERA_BUF_SIZE1);
    memset(camera->viraddr2, 0, CAMERA_BUF_SIZE2);

    printk(KERN_INFO "camera dma mem alloc success, phy1 = %#llx, phy2 = %#llx\n", camera->physaddr1, camera->physaddr2);

    return 0;
}

static void free_camera_dma(struct pci_dev *pdev, struct camera_buf_address *camera)
{
    pci_free_consistent(pdev, CAMERA_BUF_SIZE1, camera->viraddr1, camera->busaddr1);
    pci_free_consistent(pdev, CAMERA_BUF_SIZE2, camera->viraddr2, camera->busaddr2);
    DPRINTK("camera dma mem free\n");
}

static void sensor_dev_queue_free(void)
{
    if (siasun_pcie_dev.lidar_q)
    {
        siasun_destory_queue(siasun_pcie_dev.lidar_q);
    }

    if (siasun_pcie_dev.camera0_q)
    {
        siasun_destory_queue(siasun_pcie_dev.camera0_q);
    }

    if (siasun_pcie_dev.camera1_q)
    {
        siasun_destory_queue(siasun_pcie_dev.camera1_q);
    }

    if (siasun_pcie_dev.camera2_q)
    {
        siasun_destory_queue(siasun_pcie_dev.camera2_q);
    }
    if (siasun_pcie_dev.camera3_q)
    {
        siasun_destory_queue(siasun_pcie_dev.camera3_q);
    }

    if (siasun_pcie_dev.camera4_q)
    {
        siasun_destory_queue(siasun_pcie_dev.camera4_q);
    }

    if (siasun_pcie_dev.camera5_q)
    {
        siasun_destory_queue(siasun_pcie_dev.camera5_q);
    }
    
}

static int sf_pcie_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    int i = 0;
    int ret = 0;
    unsigned int last_id = 0;
    struct sf_pcie_device *sf_dev = NULL;

    DPRINTK(KERN_INFO "enter probe \n");
    sf_dev = kzalloc(sizeof(struct sf_pcie_device), GFP_KERNEL);
    if (!sf_dev)
    {
        ret = -ENOMEM;
        goto fail_alloc;
    }

    sf_dev->pci_dev = pdev;
    pci_set_drvdata(pdev, sf_dev);

    ret = pci_enable_device(pdev);
    if (ret)
    {
        dev_err(&pdev->dev, "Enable pci device failed\n");
        goto fail_enalbe;
    }

    ret = pci_request_regions(pdev, SENSORS_FUSION_DRIVER_NAME);
    if (ret)
    {
        dev_err(&pdev->dev, "Request pci regions failed\n");
        goto fail_regions;
    }

    pci_set_master(pdev);

    pci_read_config_byte(pdev, PCI_REVISION_ID, &sf_dev->revison);

    ret = sf_search_and_create_pci_devices(pdev, sf_dev);
    if (ret)
    {
        dev_err(&pdev->dev, "search and create pci devices failed\n");
        goto fail_create_pci;
    }

    if (ent->device == ALTERA_PCIE_DID_L)
    {
        siasun_pcie_dev.siasun_sf_dev[0] = sf_dev;

        ret = fpga_cdev_init(&siasun_pcie_dev);
        if (ret)
        {
            dev_err(&pdev->dev, "FPGA CFG init err\n");
            goto fail_create_dev;
        }

        ret = i2c_adapter_siasun_init(&siasun_pcie_dev);
        if (ret)
        {
            dev_err(&pdev->dev, "i2c_adapter init err\n");
            goto fail_create_dev;
        }

        ret = siasun_lidar_init(&siasun_pcie_dev);
        if (ret)
        {
            dev_err(&pdev->dev, "Lidar init err\n");
            goto fail_create_dev;
        }
    }
    else if (ent->device == ALTERA_PCIE_DID_H)
    {
        siasun_pcie_dev.siasun_sf_dev[1] = sf_dev;
        siasun_pcie_dev.dma_base = sf_dev->base_addr[0];

        ret = pci_alloc_irq_vectors(pdev, 1, 4, PCI_IRQ_MSI);
        if (ret < 0)
        {
            dev_err(&pdev->dev, "enable msi failed\n");
            goto fail_create_pci;
        }

        pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
        sf_dev->msi_enabled = 1;

        siasun_pcie_dev.table_virt_addr = ((struct lite_dma_desc_table *)pci_alloc_consistent(pdev, sizeof(struct lite_dma_desc_table), &siasun_pcie_dev.table_bus_addr));
        if (!siasun_pcie_dev.table_virt_addr)
        {
            ret = -ENOMEM;
            goto err_wr_table;
        }
        iowrite32(((dma_addr_t)siasun_pcie_dev.table_bus_addr) >> 32,
                  sf_dev->base_addr[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_WR_RC_HIGH_SRC_ADDR);
        iowrite32((dma_addr_t)siasun_pcie_dev.table_bus_addr,
                  sf_dev->base_addr[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_WR_RC_LOW_SRC_ADDR);
        last_id = readl(siasun_pcie_dev.dma_base + ALTERA_LITE_DMA_WR_LAST_PTR);
        if (last_id == 0xff)
        {
            iowrite32(WR_CTRL_BUF_BASE_HI, sf_dev->base_addr[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_WR_CTRL_HIGH_DEST_ADDR);
            iowrite32(WR_CTRL_BUF_BASE_LOW, sf_dev->base_addr[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_WR_CTLR_LOW_DEST_ADDR);
            iowrite32(0, sf_dev->base_addr[0] + DESC_CTRLLER_BASE + ALTERA_LITE_DMA_WR_CONTROL);
        }
        wmb();

        ret = alloc_lidar_dma(pdev, &siasun_pcie_dev.lidar);
        if (ret)
        {
            dev_err(&pdev->dev, "alloc lidar dma failed\n");
            goto err_lidar_buffer;
        }

        ret = alloc_camera_dma(pdev, &siasun_pcie_dev.camera0);
        if (ret)
        {
            dev_err(&pdev->dev, "alloc camera0 dma failed\n");
            goto err_camera_buffer0;
        }

        ret = alloc_camera_dma(pdev, &siasun_pcie_dev.camera1);
        if (ret)
        {
            dev_err(&pdev->dev, "alloc camera1 dma failed\n");
            goto err_camera_buffer1;
        }

        ret = alloc_camera_dma(pdev, &siasun_pcie_dev.camera2);
        if (ret)
        {
            dev_err(&pdev->dev, "alloc camera2 dma failed\n");
            goto err_camera_buffer2;
        }

        ret = alloc_camera_dma(pdev, &siasun_pcie_dev.camera3);
        if (ret)
        {
            dev_err(&pdev->dev, "alloc camera3 dma failed\n");
            goto err_camera_buffer3;
        }

        ret = alloc_camera_dma(pdev, &siasun_pcie_dev.camera4);
        if (ret)
        {
            dev_err(&pdev->dev, "alloc camera4 dma failed\n");
            goto err_camera_buffer4;
        }

        ret = alloc_camera_dma(pdev, &siasun_pcie_dev.camera5);
        if (ret)
        {
            dev_err(&pdev->dev, "alloc camera5 dma failed\n");
            goto err_camera_buffer5;
        }

        for (i = 0; i < ALTERA_VIDEO_NUM; i++)
        {
            siasun_pcie_dev.siasun_videos[i] = siasun_v4l2_create_instance(&siasun_pcie_dev, i);
            if (siasun_pcie_dev.siasun_videos[i] == NULL)
            {
                dev_err(&pdev->dev, "create video[%d] instance failed\n", i);
                goto err_create_instance;
            }
        }

        ret = request_irq(pdev->irq, siasun_irq, 0, SENSORS_FUSION_DRIVER_NAME, &siasun_pcie_dev);
        if (ret)
        {
            dev_err(&pdev->dev, "Could not request IRQ #%d, error %d", pdev->irq, ret);
            goto err_irq;
        }

        pci_write_config_byte(pdev, PCI_INTERRUPT_LINE, pdev->irq);
        dev_info(&pdev->dev, "Succesfully requested IRQ #%d", pdev->irq);

        siasun_pcie_dev.lidar_q = siasun_create_queue();
        if (siasun_pcie_dev.lidar_q == NULL) {
             dev_err(&pdev->dev, "create lidar queue is NULL\n");
             goto err_create_queue;
        }
           
        siasun_pcie_dev.camera0_q = siasun_create_queue();
        if (siasun_pcie_dev.camera0_q == NULL) {
            dev_err(&pdev->dev, "create camera0 queue is NULL\n");
            goto err_create_queue;
        }
            
        siasun_pcie_dev.camera1_q = siasun_create_queue();
        if (siasun_pcie_dev.camera1_q == NULL) {
            dev_err(&pdev->dev, "create camera1 queue is NULL\n");
            goto err_create_queue;
        }
           
        siasun_pcie_dev.camera2_q = siasun_create_queue();
        if (siasun_pcie_dev.camera2_q == NULL) {
            dev_err(&pdev->dev, "create camera2 queue is NULL\n");
            goto err_create_queue;
        }
        
        siasun_pcie_dev.camera3_q = siasun_create_queue();
        if (siasun_pcie_dev.camera3_q == NULL) {
            dev_err(&pdev->dev, "create camera3 queue is NULL\n");
            goto err_create_queue;
        }
           
        siasun_pcie_dev.camera4_q = siasun_create_queue();
        if (siasun_pcie_dev.camera4_q == NULL) {
            dev_err(&pdev->dev, "create camera4 queue is NULL\n");
            goto err_create_queue;
        }
        
        siasun_pcie_dev.camera5_q = siasun_create_queue();
        if (siasun_pcie_dev.camera5_q == NULL) {
            dev_err(&pdev->dev, "create camera5 queue is NULL\n");
            goto err_create_queue;
        }

        siasun_pcie_dev.dma_ready = 1;
        tasklet_init(&siasun_pcie_dev.task, task_handler, (unsigned long)&siasun_pcie_dev);

        siasun_pcie_dev.irq_base = sf_dev->base_addr[4];

        writel(0, sf_dev->base_addr[4] + PCIE_CRA_IRQ_ENABLE);
    }

    printk(KERN_INFO "%#x is load\n", ent->device);

    return ret;

err_create_queue:
    sensor_dev_queue_free();

err_irq:
    for (i = 0; i < ALTERA_VIDEO_NUM; i++)
    {
        if (siasun_pcie_dev.siasun_videos[i] != NULL)
        {
            siasun_v4l2_free_instance(&siasun_pcie_dev, i);
        }
    }

err_create_instance:
    free_camera_dma(pdev, &siasun_pcie_dev.camera5);

err_camera_buffer5:
    free_camera_dma(pdev, &siasun_pcie_dev.camera4);

err_camera_buffer4:
    free_camera_dma(pdev, &siasun_pcie_dev.camera3);

err_camera_buffer3:
    free_camera_dma(pdev, &siasun_pcie_dev.camera2);

err_camera_buffer2:
    free_camera_dma(pdev, &siasun_pcie_dev.camera1);

err_camera_buffer1:
    free_camera_dma(pdev, &siasun_pcie_dev.camera0);

err_camera_buffer0:
    free_lidar_dma(pdev, &siasun_pcie_dev.lidar);

err_lidar_buffer:
    pci_free_consistent(pdev, sizeof(struct lite_dma_desc_table), siasun_pcie_dev.table_virt_addr,
                        siasun_pcie_dev.table_bus_addr);

err_wr_table:
    dev_err(&pdev->dev, "failed alloc lite_table_wr_cpu_virt_addr\n");
    pci_disable_msi(pdev);
    sf_dev->msi_enabled = 0;
    goto fail_create_pci;

fail_create_dev:
    siasun_lidar_exit();
    i2c_adapter_siasun_exit(&siasun_pcie_dev);
    fpga_cdev_exit();
fail_create_pci:
    sf_delete_pci_devices(pdev, sf_dev);
    pci_release_regions(pdev);
fail_regions:
    pci_disable_device(pdev);
fail_enalbe:
    kfree(sf_dev);
fail_alloc:
    dev_err(&pdev->dev, "failed alloc memory\n");

    return ret;
}

static void sf_pcie_remove(struct pci_dev *pdev)
{
    int i = 0;
    struct sf_pcie_device *sf_dev = NULL;
    sf_dev = pci_get_drvdata(pdev);

    if (pdev->device == ALTERA_PCIE_DID_L)
    {
        fpga_cdev_exit();
        i2c_adapter_siasun_exit(&siasun_pcie_dev);
        siasun_lidar_exit();
    }
    else if (pdev->device == ALTERA_PCIE_DID_H)
    {

        writel(0, sf_dev->base_addr[4] + PCIE_CRA_IRQ_ENABLE);
        writel(0x01, sf_dev->base_addr[4] + PCIE_LIDAR_IRQ_STATUS);
        writel(0x01, sf_dev->base_addr[4] + PCIE_CAMERA0_IRQ_STATUS);
        writel(0x01, sf_dev->base_addr[4] + PCIE_CAMERA1_IRQ_STATUS);
        writel(0x01, sf_dev->base_addr[4] + PCIE_CAMERA2_IRQ_STATUS);
        writel(0x01, sf_dev->base_addr[4] + PCIE_CAMERA3_IRQ_STATUS);

        tasklet_kill(&siasun_pcie_dev.task);

        sensor_dev_queue_free();

        free_irq(pdev->irq, &siasun_pcie_dev);

        for (i = 0; i < ALTERA_VIDEO_NUM; i++)
        {
            siasun_v4l2_free_instance(&siasun_pcie_dev, i);
        }

        free_camera_dma(pdev, &siasun_pcie_dev.camera5);
        free_camera_dma(pdev, &siasun_pcie_dev.camera4);
        free_camera_dma(pdev, &siasun_pcie_dev.camera3);
        free_camera_dma(pdev, &siasun_pcie_dev.camera2);
        free_camera_dma(pdev, &siasun_pcie_dev.camera1);
        free_camera_dma(pdev, &siasun_pcie_dev.camera0);
        free_lidar_dma(pdev, &siasun_pcie_dev.lidar);
        pci_free_consistent(pdev, sizeof(struct lite_dma_desc_table), siasun_pcie_dev.table_virt_addr,
                            siasun_pcie_dev.table_bus_addr);
    }

    sf_delete_pci_devices(pdev, sf_dev);
    if (sf_dev->msi_enabled)
    {
        pci_disable_msi(pdev);
        sf_dev->msi_enabled = 0;
    }
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    kfree(sf_dev);

    if (pdev->device == ALTERA_PCIE_DID_L)
    {
        siasun_pcie_dev.siasun_sf_dev[0] = NULL;
    }
    else if (pdev->device == ALTERA_PCIE_DID_H)
    {
        siasun_pcie_dev.siasun_sf_dev[1] = NULL;
    }
}

static struct pci_driver sf_pcie_driver = {
    .name = SENSORS_FUSION_DRIVER_NAME,
    .id_table = altera_pcie_id,
    .probe = sf_pcie_probe,
    .remove = sf_pcie_remove,
};

/* sensors_init_module - Driver Registration Routine */
static int __init sensors_init_module(void)
{
    int ret = 0;

    printk(KERN_INFO "Diver init\n");
    memset(&siasun_pcie_dev, 0, sizeof(siasun_pcie_dev));
    ret = pci_register_driver(&sf_pcie_driver);

    return ret;
}

/* sensors_exit_module - Driver Exit Cleanup Routine */
static void __exit sensors_exit_module(void)
{
    printk("Driver exit\n");
    pci_unregister_driver(&sf_pcie_driver);
}

module_init(sensors_init_module);
module_exit(sensors_exit_module);

MODULE_AUTHOR("Tianjin SIASUN Intelligent Technology Co.,Ltd.");
MODULE_DESCRIPTION("Sensor Fusion Driver");
MODULE_LICENSE("GPL v2");
