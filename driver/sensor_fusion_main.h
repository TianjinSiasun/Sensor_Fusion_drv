/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SENSOR_FUSION_MAIN_H__
#define __SENSOR_FUSION_MAIN_H__

#include <linux/pci.h>
#include <linux/interrupt.h>
#include "sensor_fusion_lidar_include.h"
#include "sensor_fusion_queue.h"

typedef unsigned long long dma_addr_t;
typedef unsigned long long phys_addr_t;

//#define DEBUG
#ifdef DEBUG
#define DPRINTK printk
#else
#define DPRINTK(stuff...)
#endif

#define ALTERA_PCIE_BAR_NUM     6
#define ALTERA_VIDEO_NUM        6
#define I2C_BUS_NUM             6

#define ALTERA_PCIE_VID     0x1172
#define ALTERA_PCIE_DID_L   0xE001
#define ALTERA_PCIE_DID_H   0xE003

#define ALTERA_DMA_DESCRIPTOR_NUM 128

#define LIDAR_ADDR_OFFSET						(100 * 1024)

#define ALTERA_LITE_DMA_RD_RC_LOW_SRC_ADDR      0x0000
#define ALTERA_LITE_DMA_RD_RC_HIGH_SRC_ADDR     0x0004
#define ALTERA_LITE_DMA_RD_CTLR_LOW_DEST_ADDR   0x0008
#define ALTERA_LITE_DMA_RD_CTRL_HIGH_DEST_ADDR  0x000C
#define ALTERA_LITE_DMA_RD_LAST_PTR             0x0010
#define ALTERA_LITE_DMA_RD_TABLE_SIZE           0x0014
#define ALTERA_LITE_DMA_RD_CONTROL              0x0018

#define ALTERA_LITE_DMA_WR_RC_LOW_SRC_ADDR      0x0100
#define ALTERA_LITE_DMA_WR_RC_HIGH_SRC_ADDR     0x0104
#define ALTERA_LITE_DMA_WR_CTLR_LOW_DEST_ADDR   0x0108
#define ALTERA_LITE_DMA_WR_CTRL_HIGH_DEST_ADDR  0x010C
#define ALTERA_LITE_DMA_WR_LAST_PTR             0x0110
#define ALTERA_LITE_DMA_WR_TABLE_SIZE           0x0114
#define ALTERA_LITE_DMA_WR_CONTROL              0x0118

#define DESC_CTRLLER_BASE                       0x0000
#define ALTERA_DMA_CHUNK_SIZE                   0x2000
#define DMA_TIMEOUT                             0x2000000

// PCIe control register addresses
#define ACL_PCI_CRA_BAR                         4
#define ACL_PCI_CRA_OFFSET                      0
#define ACL_PCI_CRA_SIZE                        0x4000
// PCI express control-register offsets
#define PCIE_CRA_IRQ_STATUS                     0xcf90
#define PCIE_CRA_IRQ_ENABLE                     0xcfa0
#define PCIE_CRA_ADDR_TRANS                     0x1000

#define PCIE_LIDAR_IRQ_STATUS                   0x94
#define PCIE_CAMERA0_IRQ_STATUS                 0x98
#define PCIE_CAMERA1_IRQ_STATUS                 0x9C
#define PCIE_CAMERA2_IRQ_STATUS                 0xA0
#define PCIE_CAMERA3_IRQ_STATUS                 0xA4
#define PCIE_CAMERA4_IRQ_STATUS                 0xB0
#define PCIE_CAMERA5_IRQ_STATUS                 0xB4

#define PCIE_LIDAR_IRQ                          0x01
#define PCIE_CAMERA0_IRQ                        0x02
#define PCIE_CAMERA1_IRQ                        0x04
#define PCIE_CAMERA2_IRQ                        0x08
#define PCIE_CAMERA3_IRQ                        0x10
#define PCIE_CAMERA4_IRQ                        0x20
#define PCIE_CAMERA5_IRQ                        0x40

#define RD_CTRL_BUF_BASE_LOW                    0x80000000
#define RD_CTRL_BUF_BASE_HI                     0x0000
#define WR_CTRL_BUF_BASE_LOW                    0x80002000
#define WR_CTRL_BUF_BASE_HI                     0x0000

#define MAX_NUM_DWORDS                          0x100000
#define LIDAR_BUF_SIZE                          0x100000
#define CAMERA_BUF_SIZE1                        0x400000
#define CAMERA_BUF_SIZE2                        0x200000

#define FPGA_DDR3_BASE                          0x100000000

#define FPGA_CAMERA0_OFFSET                     0x0
#define FPGA_CAMERA1_OFFSET                     0x800000
#define FPGA_CAMERA2_OFFSET                     0x1000000
#define FPGA_CAMERA3_OFFSET                     0x1800000
#define FPGA_CAMERA4_OFFSET                     0x2000000
#define FPGA_CAMERA5_OFFSET                     0x2800000
#define FPGA_LIDAR_OFFSET                       0x3000000

#define FPGA_BUF_SIZE                           0x4000000
#define FPGA_LIDAR_MOVE_SIZE                    0x12c00
#define FPGA_CAMERA_MOVE_SIZE                   0x20000
#define FPGA_CAMERA_MOVE_OFFSET                 0x80000

/**
 * struct sf_pcie_device - sensor fusion pcie device information
 * @pci_dev: Associated pcie device
 * @msi_enabled: msi enable status
 * @revison: revision id of pcie device
 * @base_addr: the base address of every bar spcae
 * @bar_length: the length of every bar space
 */
struct sf_pcie_device
{
    struct pci_dev *    pci_dev;
    unsigned int        msi_enabled;
    unsigned char       revison;
    void __iomem *      base_addr[ALTERA_PCIE_BAR_NUM];
    unsigned int        bar_length[ALTERA_PCIE_BAR_NUM];
};

/**
 * struct dma_descriptor - the DMA descriptor format for the Read and Write Movers
 * @src_addr_ldw: lower DWORD of the DMA source address
 * @src_addr_udw: upper DWORD of the DMA source address
 * @dest_addr_ldw: lower DWORD of the DMA destination address
 * @dest_addr_udw: upper DWORD of the DMA destination address
 * @ctl_dma_len: specifies the following information.
 *               [31:25] Reserved must be 0.
 *               [24:18] ID.Specifies the Descriptor ID.
 *               [17: 0] Size.The Transfer size in DWORDs.Must be non-zero.
 * @reserved: Reserved.
 */
struct dma_descriptor
{
    uint32_t src_addr_ldw;
    uint32_t src_addr_udw;
    uint32_t dest_addr_ldw;
    uint32_t dest_addr_udw;
    uint32_t ctl_dma_len;
    uint32_t reserved[3];
} __attribute__((packed));

/**
 * struct lite_dma_header - the DMA Descriptor ID table 
 * @flags: the descriptor status
 */
struct lite_dma_header
{
    volatile uint32_t flags[128];
} __attribute__((packed));

/**
 * struct lite_dma_desc_table - the DMA Descriptor table information 
 * @header: the descriptor status table
 * @descriptors: the DMA descriptor information
 */
struct lite_dma_desc_table
{
    struct lite_dma_header header;
    struct dma_descriptor descriptors[ALTERA_DMA_DESCRIPTOR_NUM];
}__attribute__((packed));

/**
 * struct lidar_buf_address - the lidar data space information 
 * @viraddr: the lidar data space virtual address
 * @busaddr: the lidar data space bus address
 * @physaddr: the lidar data space physical address
 */
struct lidar_buf_address
{
    unsigned char *viraddr;
    dma_addr_t busaddr;
    phys_addr_t physaddr;
};

/**
 * struct camera_buf_address - the camera data space information 
 * @viraddr1: the camera data space first section virtual address
 * @busaddr1: the camera data space first section bus address
 * @physaddr1: the camera data space first section physical address
 * @viraddr2: the camera data space second section virtual address
 * @busaddr2: the camera data space second section bus address
 * @physaddr2: the camera data space second section physical address
 */
struct camera_buf_address
{
    unsigned char *viraddr1;
    dma_addr_t busaddr1;
    phys_addr_t physaddr1;

    unsigned char *viraddr2;
    dma_addr_t busaddr2;
    phys_addr_t physaddr2;
};

/**
 * struct siasun_pcie_device - the sensor fusion device information
 * @siasun_sf_dev: the pcie device of sensor fusion
 * @siasun_videos: the camera device information
 * @i2c_adapter: the i2c device information
 * @i2c_nr: i2c bus bum
 * @lidar_dev: the lidar device information
 * @fpga_cfg: the fpga config device information
 * @dma_base: the DMA config base addr
 * @table_virt_addr: the DMA descriptor table virtual address
 * @table_bus_addr: the DMA descriptor table bus address
 * @dma_ready: the DMA status flag
 * @lidar：the lidar buffer address information
 * @camera0: the lvds camera 0 buffer address information
 * @camera1: the lvds camera 1 buffer address information
 * @camera2: the lvds camera 2 buffer address information
 * @camera3: the lvds camera 3 buffer address information
 * @camera4: the mipi camera 0 buffer address information
 * @camera5: the mipi camera 1 buffer address information
 * @lidar_q: the lidar data index queue
 * @camera0_q: the lvds camera 0 data index queue
 * @camera1_q: the lvds camera 1 data index queue
 * @camera2_q: the lvds camera 2 data index queue
 * @camera3_q: the lvds camera 3 data index queue
 * @camera4_q: the mipi camera 0 data index queue
 * @camera5_q: the mipi camera 1 data index queue
 * @irq_base: the interrupt base address of pcie
 * @task: the tasklet of interrupt
 */
struct siasun_pcie_device
{
    struct sf_pcie_device *siasun_sf_dev[2];
    struct siasun_video_dev *siasun_videos[ALTERA_VIDEO_NUM];
    struct siasun_i2c *i2c_adapter[I2C_BUS_NUM];
//    int i2c_nr[I2C_BUS_NUM];
    struct lidar_cdev *lidar_dev;
    struct fpga_cdev *fpga_cfg;

    void __iomem *dma_base;
    struct lite_dma_desc_table *table_virt_addr;
    dma_addr_t table_bus_addr;
    unsigned char dma_ready;

    struct lidar_buf_address lidar;
    struct camera_buf_address camera0;
    struct camera_buf_address camera1;
    struct camera_buf_address camera2;
    struct camera_buf_address camera3;
    struct camera_buf_address camera4;
    struct camera_buf_address camera5;

    queue *lidar_q;
    queue *camera0_q;
    queue *camera1_q;
    queue *camera2_q;
    queue *camera3_q;
    queue *camera4_q;
    queue *camera5_q;

    void __iomem *irq_base;
    struct tasklet_struct task;
};

#endif /* __SENSOR_FUSION_MAIN_H__ */
