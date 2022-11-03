/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/font.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include "sensor_fusion_main.h"
#include "sensor_fusion_ov8865.h"
#include "sensor_fusion_v4l2.h"

static SZ_MIPI_REG_T MipiBridgeReg[] = {
    {0xFFFF, 10},
    {0x0002, 0x0001},
    {0xFFFF, 10},
    {0x0002, 0x0000},
    {0x0016, ((PLL_PRD << 12) + PLL_FBD)},
    {0x0018, ((PLL_FRS << 10) + (0x2 << 8) + (0x1 << 1) + 0x1)},
    {0xFFFF, 10},
    {0x0018, ((PLL_FRS << 10) + (0x2 << 8) + (0x1 << 4) + (0x1 << 1) + 0x1)},
    {0x0020, ((PPICLKDIV << 4) + (MCLKREFDIV << 2) + SCLKDIV)},
    {0x000C, ((MCLK_HL << 8) + MCLK_HL)},
    {0x0060, 0x8004},
    {0x0006, FIFO_LEVEL},
    {0x0008, DATA_FORMAT},
    {0x0004, 0x8045}
};

static SZ_CONFIG_T MipiCameraReg[] = {
    {0x6c, 0x0103, 0x01},
    {TIME_DELAY, 0, 1},
    {0x6c, 0x0100, 0x00},
    {0x6c, 0x0100, 0x00},
    {0x6c, 0x0100, 0x00},
    {0x6c, 0x0100, 0x00},
    {TIME_DELAY, 0, 1},
    {0x6c, 0x3638, 0xff},
    {0x6c, 0x0302, 32},
    {0x6c, 0x0303, 0x00},
    {0x6c, 0x0304, 3},
    {0x6c, 0x030e, 0x00},
    {0x6c, 0x030f, 0x04},
    {0x6c, 0x0312, 0x01},
    {0x6c, 0x031e, 0x0c},
    {0x6c, 0x3015, 0x01},
    {0x6c, 0x3018, 0x72},
    {0x6c, 0x3018, 0x32},
    {0x6c, 0x3020, 0x93},
    {0x6c, 0x3022, 0x01},
    {0x6c, 0x3031, 0x0a},
    {0x6c, 0x3106, 0x01},
    {0x6c, 0x3305, 0xf1},
    {0x6c, 0x3308, 0x00},
    {0x6c, 0x3309, 0x28},
    {0x6c, 0x330a, 0x00},
    {0x6c, 0x330b, 0x20},
    {0x6c, 0x330c, 0x00},
    {0x6c, 0x330d, 0x00},
    {0x6c, 0x330e, 0x00},
    {0x6c, 0x330f, 0x40},
    {0x6c, 0x3307, 0x04},
    {0x6c, 0x3604, 0x04},
    {0x6c, 0x3602, 0x30},
    {0x6c, 0x3605, 0x00},
    {0x6c, 0x3607, 0x20},
    {0x6c, 0x3608, 0x11},
    {0x6c, 0x3609, 0x68},
    {0x6c, 0x360a, 0x40},
    {0x6c, 0x360c, 0xdd},
    {0x6c, 0x360e, 0x0c},
    {0x6c, 0x3610, 0x07},
    {0x6c, 0x3612, 0x86},
    {0x6c, 0x3613, 0x58},
    {0x6c, 0x3614, 0x28},
    {0x6c, 0x3617, 0x40},
    {0x6c, 0x3618, 0x5a},
    {0x6c, 0x3619, 0x9b},
    {0x6c, 0x361c, 0x00},
    {0x6c, 0x361d, 0x60},
    {0x6c, 0x3631, 0x60},
    {0x6c, 0x3633, 0x10},
    {0x6c, 0x3634, 0x10},
    {0x6c, 0x3635, 0x10},
    {0x6c, 0x3636, 0x10},
    {0x6c, 0x3641, 0x55},
    {0x6c, 0x3646, 0x86},
    {0x6c, 0x3647, 0x27},
    {0x6c, 0x364a, 0x1b},
    {0x6c, 0x3500, 0x00},
    {0x6c, 0x3501, 0x30},
    {0x6c, 0x3502, 0x00},
    {0x6c, 0x3503, 0x00},
    {0x6c, 0x3508, 0x03},
    {0x6c, 0x3509, 0x00},
    {0x6c, 0x3700, 0x48},
    {0x6c, 0x3701, 0x18},
    {0x6c, 0x3702, 0x50},
    {0x6c, 0x3703, 0x32},
    {0x6c, 0x3704, 0x28},
    {0x6c, 0x3705, 0x00},
    {0x6c, 0x3706, 0x70},
    {0x6c, 0x3707, 0x08},
    {0x6c, 0x3708, 0x48},
    {0x6c, 0x3709, 0x80},
    {0x6c, 0x370a, 0x01},
    {0x6c, 0x370b, 0x70},
    {0x6c, 0x370c, 0x07},
    {0x6c, 0x3718, 0x14},
    {0x6c, 0x3719, 0x31},
    {0x6c, 0x3712, 0x44},
    {0x6c, 0x3714, 0x12},
    {0x6c, 0x371e, 0x31},
    {0x6c, 0x371f, 0x7f},
    {0x6c, 0x3720, 0x0a},
    {0x6c, 0x3721, 0x0a},
    {0x6c, 0x3724, 0x04},
    {0x6c, 0x3725, 0x04},
    {0x6c, 0x3726, 0x0c},
    {0x6c, 0x3728, 0x0a},
    {0x6c, 0x3729, 0x03},
    {0x6c, 0x372a, 0x06},
    {0x6c, 0x372b, 0xa6},
    {0x6c, 0x372c, 0xa6},
    {0x6c, 0x372d, 0xa6},
    {0x6c, 0x372e, 0x0c},
    {0x6c, 0x372f, 0x20},
    {0x6c, 0x3730, 0x02},
    {0x6c, 0x3731, 0x0c},
    {0x6c, 0x3732, 0x28},
    {0x6c, 0x3733, 0x10},
    {0x6c, 0x3734, 0x40},
    {0x6c, 0x3736, 0x30},
    {0x6c, 0x373a, 0x04},
    {0x6c, 0x373b, 0x18},
    {0x6c, 0x373c, 0x14},
    {0x6c, 0x373e, 0x06},
    {0x6c, 0x3755, 0x40},
    {0x6c, 0x3758, 0x00},
    {0x6c, 0x3759, 0x4c},
    {0x6c, 0x375a, 0x0c},
    {0x6c, 0x375b, 0x26},
    {0x6c, 0x375c, 0x20},
    {0x6c, 0x375d, 0x04},
    {0x6c, 0x375e, 0x00},
    {0x6c, 0x375f, 0x28},
    {0x6c, 0x3767, 0x04},
    {0x6c, 0x3768, 0x04},
    {0x6c, 0x3769, 0x20},
    {0x6c, 0x376c, 0x00},
    {0x6c, 0x376d, 0x00},
    {0x6c, 0x376a, 0x08},
    {0x6c, 0x3761, 0x00},
    {0x6c, 0x3762, 0x00},
    {0x6c, 0x3763, 0x00},
    {0x6c, 0x3766, 0xff},
    {0x6c, 0x376b, 0x42},
    {0x6c, 0x3772, 0x46},
    {0x6c, 0x3773, 0x04},
    {0x6c, 0x3774, 0x2c},
    {0x6c, 0x3775, 0x13},
    {0x6c, 0x3776, 0x10},
    {0x6c, 0x37a0, 0x88},
    {0x6c, 0x37a1, 0x7a},
    {0x6c, 0x37a2, 0x7a},
    {0x6c, 0x37a3, 0x02},
    {0x6c, 0x37a4, 0x00},
    {0x6c, 0x37a5, 0x09},
    {0x6c, 0x37a6, 0x00},
    {0x6c, 0x37a7, 0x88},
    {0x6c, 0x37a8, 0xb0},
    {0x6c, 0x37a9, 0xb0},
    {0x6c, 0x3760, 0x00},
    {0x6c, 0x376f, 0x01},
    {0x6c, 0x37aa, 0x88},
    {0x6c, 0x37ab, 0x5c},
    {0x6c, 0x37ac, 0x5c},
    {0x6c, 0x37ad, 0x55},
    {0x6c, 0x37ae, 0x19},
    {0x6c, 0x37af, 0x19},
    {0x6c, 0x37b0, 0x00},
    {0x6c, 0x37b1, 0x00},
    {0x6c, 0x37b2, 0x00},
    {0x6c, 0x37b3, 0x84},
    {0x6c, 0x37b4, 0x84},
    {0x6c, 0x37b5, 0x66},
    {0x6c, 0x37b6, 0x00},
    {0x6c, 0x37b7, 0x00},
    {0x6c, 0x37b8, 0x00},
    {0x6c, 0x37b9, 0xff},
    {0x6c, 0x3808, 0x06},
    {0x6c, 0x3809, 0x40},
    {0x6c, 0x380a, 0x04},
    {0x6c, 0x380b, 0xb0},
    {0x6c, 0x380c, 0x0D},
    {0x6c, 0x380d, 0x00},
    {0x6c, 0x380e, 0x0E},
    {0x6c, 0x380f, 0x50},
    {0x6c, 0x3810, 0x00},
    {0x6c, 0x3811, 0x04},
    {0x6c, 0x3813, 0x02},
    {0x6c, 0x3814, 0x03},
    {0x6c, 0x3815, 0x01},
    {0x6c, 0x3820, 0x06},
    {0x6c, 0x3821, 0x70},
    {0x6c, 0x382a, 0x03},
    {0x6c, 0x382b, 0x01},
    {0x6c, 0x3830, 4},
    {0x6c, 0x3836, 1},
    {0x6c, 0x3837, 0x18},
    {0x6c, 0x3841, 0xff},
    {0x6c, 0x3846, 0x48},
    {0x6c, 0x3f08, 0x16},
    {0x6c, 0x4000, 0xf1},
    {0x6c, 0x4001, 0x04},
    {0x6c, 0x4005, 0x10},
    {0x6c, 0x400b, 0x0c},
    {0x6c, 0x400d, 0x10},
    {0x6c, 0x4011, 0x30},
    {0x6c, 0x4013, 0xcf},
    {0x6c, 0x401b, 0x00},
    {0x6c, 0x401d, 0x00},
    {0x6c, 0x4020, 0x02},
    {0x6c, 0x4021, 0x40},
    {0x6c, 0x4022, 0x03},
    {0x6c, 0x4023, 0x3f},
    {0x6c, 0x4024, 0x07},
    {0x6c, 0x4025, 0xc0},
    {0x6c, 0x4026, 0x08},
    {0x6c, 0x4027, 0xbf},
    {0x6c, 0x4028, 0x00},
    {0x6c, 0x4029, 0x02},
    {0x6c, 0x402a, 0x04},
    {0x6c, 0x402b, 0x04},
    {0x6c, 0x402c, 0x02},
    {0x6c, 0x402d, 0x02},
    {0x6c, 0x402e, 0x08},
    {0x6c, 0x402f, 0x02},
    {0x6c, 0x401f, 0x00},
    {0x6c, 0x4034, 0x3f},
    {0x6c, 0x4300, 0xff},
    {0x6c, 0x4301, 0x00},
    {0x6c, 0x4302, 0x0f},
    {0x6c, 0x4500, 0x68},
    {0x6c, 0x4503, 0x10},
    {0x6c, 0x4601, 0x10},
    {0x6c, 0x481f, 70},
    {0x6c, 0x4837, 0x16},
    {0x6c, 0x4850, 0x10},
    {0x6c, 0x4851, 0x32},
    {0x6c, 0x4b00, 0x2a},
    {0x6c, 0x4b0d, 0x00},
    {0x6c, 0x4d00, 0x04},
    {0x6c, 0x4d01, 0x18},
    {0x6c, 0x4d02, 0xc3},
    {0x6c, 0x4d03, 0xff},
    {0x6c, 0x4d04, 0xff},
    {0x6c, 0x4d05, 0xff},
    {0x6c, 0x5000, 0x16},
    {0x6c, 0x5001, 0x01},
    {0x6c, 0x5002, 0x08},
    {0x6c, 0x5901, 0x00},
    {0x6c, 0x5e00, 0x00},
    {0x6c, 0x5018, 0x15},
    {0x6c, 0x501A, 0x10},
    {0x6c, 0x501C, 0x15},
    {0x6c, 0x5e01, 0x41},
    {TIME_DELAY, 0, 1},
    {0x6c, 0x5780, 0xfc},
    {0x6c, 0x5781, 0xdf},
    {0x6c, 0x5782, 0x3f},
    {0x6c, 0x5783, 0x08},
    {0x6c, 0x5784, 0x0c},
    {0x6c, 0x5786, 0x20},
    {0x6c, 0x5787, 0x40},
    {0x6c, 0x5788, 0x08},
    {0x6c, 0x5789, 0x08},
    {0x6c, 0x578a, 0x02},
    {0x6c, 0x578b, 0x01},
    {0x6c, 0x578c, 0x01},
    {0x6c, 0x578d, 0x0c},
    {0x6c, 0x578e, 0x02},
    {0x6c, 0x578f, 0x01},
    {0x6c, 0x5790, 0x01},
    {0x6c, 0x5800, 0x1d},
    {0x6c, 0x5801, 0x0e},
    {0x6c, 0x5802, 0x0c},
    {0x6c, 0x5803, 0x0c},
    {0x6c, 0x5804, 0x0f},
    {0x6c, 0x5805, 0x22},
    {0x6c, 0x5806, 0x0a},
    {0x6c, 0x5807, 0x06},
    {0x6c, 0x5808, 0x05},
    {0x6c, 0x5809, 0x05},
    {0x6c, 0x580a, 0x07},
    {0x6c, 0x580b, 0x0a},
    {0x6c, 0x580c, 0x06},
    {0x6c, 0x580d, 0x02},
    {0x6c, 0x580e, 0x00},
    {0x6c, 0x580f, 0x00},
    {0x6c, 0x5810, 0x03},
    {0x6c, 0x5811, 0x07},
    {0x6c, 0x5812, 0x06},
    {0x6c, 0x5813, 0x02},
    {0x6c, 0x5814, 0x00},
    {0x6c, 0x5815, 0x00},
    {0x6c, 0x5816, 0x03},
    {0x6c, 0x5817, 0x07},
    {0x6c, 0x5818, 0x09},
    {0x6c, 0x5819, 0x06},
    {0x6c, 0x581a, 0x04},
    {0x6c, 0x581b, 0x04},
    {0x6c, 0x581c, 0x06},
    {0x6c, 0x581d, 0x0a},
    {0x6c, 0x581e, 0x19},
    {0x6c, 0x581f, 0x0d},
    {0x6c, 0x5820, 0x0b},
    {0x6c, 0x5821, 0x0b},
    {0x6c, 0x5822, 0x0e},
    {0x6c, 0x5823, 0x22},
    {0x6c, 0x5824, 0x23},
    {0x6c, 0x5825, 0x28},
    {0x6c, 0x5826, 0x29},
    {0x6c, 0x5827, 0x27},
    {0x6c, 0x5828, 0x13},
    {0x6c, 0x5829, 0x26},
    {0x6c, 0x582a, 0x33},
    {0x6c, 0x582b, 0x32},
    {0x6c, 0x582c, 0x33},
    {0x6c, 0x582d, 0x16},
    {0x6c, 0x582e, 0x14},
    {0x6c, 0x582f, 0x30},
    {0x6c, 0x5830, 0x31},
    {0x6c, 0x5831, 0x30},
    {0x6c, 0x5832, 0x15},
    {0x6c, 0x5833, 0x26},
    {0x6c, 0x5834, 0x23},
    {0x6c, 0x5835, 0x21},
    {0x6c, 0x5836, 0x23},
    {0x6c, 0x5837, 0x05},
    {0x6c, 0x5838, 0x36},
    {0x6c, 0x5839, 0x27},
    {0x6c, 0x583a, 0x28},
    {0x6c, 0x583b, 0x26},
    {0x6c, 0x583c, 0x24},
    {0x6c, 0x583d, 0xdf},
    {END_OF_SCRIPT, 0, 0}};

int siasun_i2c_addr[] = {CAMERA_LVDS_0_I2C_CAMERA_CTRL_BASE, CAMERA_LVDS_1_I2C_CAMERA_CTRL_BASE,
                         CAMERA_LVDS_2_I2C_CAMERA_CTRL_BASE, CAMERA_LVDS_3_I2C_CAMERA_CTRL_BASE,
                         CAMERA_MIPI_0_I2C_CAMERA_CTRL_BASE, CAMERA_MIPI_1_I2C_CAMERA_CTRL_BASE};

void siasun_lvds0_init(struct siasun_video_dev *svid)
{
    writel(0, svid->config_base + CAMERA_LVDS_0_IO_BRIDGE_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(10);
    writel(1, svid->config_base + CAMERA_LVDS_0_IO_BRIDGE_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(100);
    writel(0, svid->config_base + CAMERA_LVDS_0_IO_CAMERA_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(10);
    writel(1, svid->config_base + CAMERA_LVDS_0_IO_CAMERA_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(100);
}

void siasun_lvds1_init(struct siasun_video_dev *svid)
{
    writel(0, svid->config_base + CAMERA_LVDS_1_IO_BRIDGE_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(10);
    writel(1, svid->config_base + CAMERA_LVDS_1_IO_BRIDGE_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(100);
    writel(0, svid->config_base + CAMERA_LVDS_1_IO_CAMERA_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(10);
    writel(1, svid->config_base + CAMERA_LVDS_1_IO_CAMERA_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(100);
}

void siasun_lvds2_init(struct siasun_video_dev *svid)
{
    writel(0, svid->config_base + CAMERA_LVDS_2_IO_BRIDGE_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(10);
    writel(1, svid->config_base + CAMERA_LVDS_2_IO_BRIDGE_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(100);
    writel(0, svid->config_base + CAMERA_LVDS_2_IO_CAMERA_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(10);
    writel(1, svid->config_base + CAMERA_LVDS_2_IO_CAMERA_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(100);
}

void siasun_lvds3_init(struct siasun_video_dev *svid)
{
    writel(0, svid->config_base + CAMERA_LVDS_3_IO_BRIDGE_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(10);
    writel(1, svid->config_base + CAMERA_LVDS_3_IO_BRIDGE_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(100);
    writel(0, svid->config_base + CAMERA_LVDS_3_IO_CAMERA_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(10);
    writel(1, svid->config_base + CAMERA_LVDS_3_IO_CAMERA_CTRL_BASE + REG_IO_BRIDGE_CTRL * 4);
    mdelay(100);
}

static int siasun_i2c_clk_init(struct siasun_video_dev *svid, unsigned int ref_clk, 
                                     unsigned int  i2c_clk)
{
    int ret = 0;
    unsigned int read_data;
    unsigned int prescale;
    unsigned char prescale_high;
    unsigned char prescale_low;
    const unsigned char ControlValue = 0x80;

    prescale = (ref_clk / (5 * i2c_clk)) - 1;
    prescale_low = prescale & 0xFF;
    prescale_high = (prescale >> 8) & 0xFF;

    //write low byte of  prescale (reg 0)
    writel(prescale_low, svid->config_base + siasun_i2c_addr[svid->index]);
    //write high byte of prescale (reg 1)
    writel(prescale_high, svid->config_base + siasun_i2c_addr[svid->index] + 4);

    //enable the I2C core, but disable the IRQ
    writel(ControlValue, svid->config_base + siasun_i2c_addr[svid->index] + 8);

    // check prescale low byte
    if (!ret)
    {
        read_data = readl(svid->config_base + siasun_i2c_addr[svid->index]);
        if((read_data & 0x00ff) != prescale_low )
        {
            printk(KERN_ERR "prescale low byte [%#x] not [%#x]\n", read_data, prescale_low);
            ret = -EIO;
        }
    }

    // check prescale high byte
    if (!ret)
    {
        read_data = readl(svid->config_base + siasun_i2c_addr[svid->index] + 4);
        if((read_data & 0x00ff) != prescale_high)
        {
            printk(KERN_ERR "prescale high byte [%#x] not [%#x]\n", read_data, prescale_high);
            ret = -EIO;
        }
    }

    // check control
    if (!ret)
    {
        read_data = readl(svid->config_base + siasun_i2c_addr[svid->index] + 8);
        if((read_data & 0x00ff) != ControlValue)
        {
            printk(KERN_ERR "control byte [%#x] not [%#x]\n", read_data, ControlValue);
            ret = -EIO;
        }
    }

    if (!ret)
    {
        DPRINTK("I2C core is enabled!\n");
    }
    else
    {
        DPRINTK("I2C core is not enabled successfully!\n");
    }

    return ret;

}


static unsigned short siasun_i2c_read16(struct i2c_adapter *adap, unsigned char device_addr,
                                        unsigned short addr)
{
    int err = 0;
    struct i2c_msg msg[2];
    unsigned char reg[2];
    unsigned char data[2];

    msg[0].addr = device_addr >> 1; /* I2C address of chip */
    msg[0].flags = 0;
    msg[0].len = 2;
    reg[0] = (addr >> 8) & 0xff;
    reg[1] = addr & 0xff;
    msg[0].buf = reg;

    msg[1].addr = device_addr >> 1; /* I2C address */
    msg[1].flags = I2C_M_RD;
    msg[1].len = 2;
    msg[1].buf = data;
    err = i2c_transfer(adap, msg, 2);

    return ((data[1] << 8) | data[0]);
}

static void siasun_i2c_write16(struct i2c_adapter *adap, unsigned char device_addr,
                               unsigned short addr, unsigned short data)
{
    struct i2c_msg msg[1];
    unsigned char buf[4];

    msg[0].addr = device_addr >> 1; /* I2C address of chip */
    msg[0].flags = 0;
    msg[0].len = 4;
    buf[0] = (addr >> 8) & 0xff;
    buf[1] = addr & 0xff;
    buf[2] = (data >> 8) & 0xff;
    buf[3] = data & 0xff;
    msg[0].buf = buf;
    i2c_transfer(adap, msg, 1);
}

static unsigned char siasun_i2c_read8(struct i2c_adapter *adap, unsigned char device_addr,
                                      unsigned short addr)
{
    int err = 0;
    struct i2c_msg msg[2];
    unsigned char reg[2];
    unsigned char data;

    msg[0].addr = device_addr >> 1; /* I2C address of chip */
    msg[0].flags = 0;
    msg[0].len = 2;
    reg[0] = (addr >> 8) & 0xff;
    reg[1] = addr & 0xff;
    msg[0].buf = reg;

    msg[1].addr = device_addr >> 1; /* I2C address */
    msg[1].flags = I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = &data;
    err = i2c_transfer(adap, msg, 2);

    return data;
}

int siasun_i2c_write8(struct i2c_adapter *adap, unsigned char device_addr,
                      unsigned short addr, unsigned char data)
{
    int err = 0;
    struct i2c_msg msg[1];
    unsigned char buf[3];

    msg[0].addr = device_addr >> 1; /* I2C address of chip */
    msg[0].flags = 0;
    msg[0].len = 3;
    buf[0] = (addr >> 8) & 0xff;
    buf[1] = addr & 0xff;
    buf[2] = data;
    msg[0].buf = buf;
    err = i2c_transfer(adap, msg, 1);
    return err;
}

static unsigned char siasun_i2c_gmslread(struct i2c_adapter *adap, unsigned char device_addr,
                                         unsigned char addr)
{
    int err = 0;
    struct i2c_msg msg[2];
    unsigned char reg[1];
    unsigned char data;

    msg[0].addr = device_addr >> 1; /* I2C address of chip */
    msg[0].flags = 0;
    msg[0].len = 1;
    reg[0] = addr;
    msg[0].buf = reg;

    msg[1].addr = device_addr >> 1; /* I2C address */
    msg[1].flags = I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = &data;
    err = i2c_transfer(adap, msg, 2);

    mdelay(10);
    return data;
}

static unsigned char siasun_i2c_gmslwrite(struct i2c_adapter *adap, unsigned char device_addr,
                                          unsigned char addr, unsigned char data)
{
    int err = 0;
    struct i2c_msg msg[1];
    unsigned char buf[2];

    msg[0].addr = device_addr >> 1; /* I2C address of chip */
    msg[0].flags = 0;
    msg[0].len = 2;
    buf[0] = addr;
    buf[1] = data;
    msg[0].buf = buf;
    err = i2c_transfer(adap, msg, 1);

    mdelay(10);
    return err;
}

static int siasun_bridge_init(struct i2c_adapter *adap)
{
    int i = 0;
    unsigned short val = 0;
    unsigned short id;
    unsigned int num = sizeof(MipiBridgeReg) / sizeof(MipiBridgeReg[0]);

    val = siasun_i2c_read16(adap, MIPI_BRIDGE_I2C_ADDR, 0x0000);
    id = ((val >> 8) & 0xff) | ((val & 0xff) << 8);
    if (id != 0x4401)
    {
        printk(KERN_ERR "MIPI IC Chip ID error! not 0x4401 is [%#x]\n", id);
        return -EIO;
    }

    for (i = 0; i < num; i++)
    {
        if (MipiBridgeReg[i].Addr == 0xFFFF)
        {
            mdelay(MipiBridgeReg[i].Data);
        }
        else
        {
            siasun_i2c_write16(adap, MIPI_BRIDGE_I2C_ADDR, MipiBridgeReg[i].Addr, MipiBridgeReg[i].Data);
            mdelay(1);
        }
    }

    return 0;
}

static int siasun_camera_init(struct i2c_adapter *adap)
{
    int i = 0;
    int ret = 0;
    unsigned char id_l, id_h;
    unsigned int num = 0;

    id_h = siasun_i2c_read8(adap, MIPI_I2C_ADDR, 0x300B);
    udelay(100);
    id_l = siasun_i2c_read8(adap, MIPI_I2C_ADDR, 0x300C);
    udelay(100);
    if ((id_h != 0x88) || (id_l != 0x65))
    {
        printk(KERN_ERR "Camera Sensor Chip ID error! not 0x8865! ");
        printk(KERN_ERR "[%#x] [%#x]\n", id_h, id_l);
        return -EIO;
    }

    num = sizeof(MipiCameraReg) / sizeof(MipiCameraReg[0]);
    for (i = 0; i < num; i++)
    {
        if (MipiCameraReg[i].Type == TIME_DELAY)
        {
            mdelay(MipiCameraReg[i].Data);
        }
        else if (MipiCameraReg[i].Type == END_OF_SCRIPT)
        {
            break;
        }
        else if (MipiCameraReg[i].Type == 0x6c)
        {
            siasun_i2c_write8(adap, MIPI_I2C_ADDR, MipiCameraReg[i].Addr, MipiCameraReg[i].Data);
            udelay(100);
        }
    }

    return ret;
}

int siasun_lvds_procedure(struct siasun_video_dev *svid)
{
    int ret = 0;
    int data;
    struct i2c_adapter *adap = NULL;

    adap = i2c_get_adapter(svid->bus_num);
    if (!adap)
    {
        printk(KERN_ERR "video[%d] get adapter failed\n", svid->index);
        return -1;
    }

    ret = siasun_i2c_clk_init(svid, I2C_REF_CLK_RATE, I2C_LVDS_RATE);
    if (ret)
    {
        printk(KERN_ERR "failed to init GMSL RX i2c\r\n");
        return ret;
    }

    if (siasun_i2c_gmslread(adap, GMSL_RX_I2C_ADDR, MAX_ID) != 0x4a)
    {
        printk(KERN_ERR "GMSL MAX96706(PIB Side) Chip ID error![%#x] not 0x4a!\n", siasun_i2c_gmslread(adap, GMSL_RX_I2C_ADDR, MAX_ID));
        i2c_put_adapter(adap);
        return -EIO;
    }

    ret = siasun_i2c_gmslwrite(adap, GMSL_RX_I2C_ADDR, MAX_I2C_CONFIG, 0xEE);
    if (ret < 0)
    {
        i2c_put_adapter(adap);
        printk(KERN_ERR "failed to write RX I2C CONFIG\r\n");
        return ret;
    }

    ret = siasun_i2c_gmslwrite(adap, GMSL_TX_I2C_ADDR, MAX_MAIN_CONFIG, 0x47);
    if (ret < 0)
    {
        i2c_put_adapter(adap);
        printk(KERN_ERR "failed to write TX MAIN CONFIG\r\n");
        return ret;
    }

    ret = siasun_i2c_gmslwrite(adap, GMSL_RX_I2C_ADDR, MAX_I2C_CONFIG, 0x6E);
    if (ret < 0)
    {
        i2c_put_adapter(adap);
        printk(KERN_ERR "failed to write RX I2C CONFIG2\r\n");
        return ret;
    }

    ret = siasun_i2c_gmslwrite(adap, GMSL_TX_I2C_ADDR, 0x01, GMSL_RX_I2C_ADDR);
    if (ret < 0)
    {
        i2c_put_adapter(adap);
        printk(KERN_ERR "failed to write TX I2C ADDR\r\n");
        return ret;
    }

    if (siasun_i2c_gmslread(adap, GMSL_TX_I2C_ADDR, MAX_ID) != 0x41)
    {
        printk(KERN_ERR "GMSL MAX96705(Camera Side) Chip ID error! not 0x41!\n");
        i2c_put_adapter(adap);
        return -EIO;
    }

    ret = siasun_bridge_init(adap);
    if (ret)
    {
        i2c_put_adapter(adap);
        printk(KERN_ERR "video[%d] bridge init failed\n", svid->index);
        return ret;
    }

    ret = siasun_camera_init(adap);
    if (ret)
    {
        i2c_put_adapter(adap);
        printk(KERN_ERR "video[%d] camera init failed\n", svid->index);
        return ret;
    }

    mdelay(500);
    ret = siasun_i2c_gmslwrite(adap, GMSL_TX_I2C_ADDR, MAX_MAIN_CONFIG, 0x87);
    if (ret < 0)
    {
        i2c_put_adapter(adap);
        printk(KERN_ERR "max main config failed\n");
        return ret;
    }

    mdelay(500);

    switch (svid->index)
    {
        case 0:
            data = readl(svid->config_base + CAMERA_LVDS_0_IO_REG_TYPE_ID_BASE + REG_TYPE_ID * 4);
            break;
        case 1:
            data = readl(svid->config_base + CAMERA_LVDS_1_IO_REG_TYPE_ID_BASE + REG_TYPE_ID * 4);
            break;
        case 2:
            data = readl(svid->config_base + CAMERA_LVDS_2_IO_REG_TYPE_ID_BASE + REG_TYPE_ID * 4);
            break;
        case 3:
            data = readl(svid->config_base + CAMERA_LVDS_3_IO_REG_TYPE_ID_BASE + REG_TYPE_ID * 4);
            break;
        default:
            printk(KERN_ERR "index error[%d]\n", svid->index);
    }

    if (data & 0x04)
    {
        DPRINTK("1a Locked\r\n\n");
    }
    else
    {
        DPRINTK("1a Locked Failed!!!\r\n\n");
    }

    mdelay(10);

    ret = siasun_i2c_clk_init(svid, I2C_MAX_CLK_RATE, I2C_MAX_RATE);
    if (ret)
    {
        i2c_put_adapter(adap);
        printk(KERN_ERR "failed to init GMSL RX i2c\r\n");
        return ret;
    }

    ret = siasun_i2c_gmslwrite(adap, MIPI_AF_I2C_ADDR, 0x01, 0X4C);
    if (ret < 0)
    {
        printk(KERN_ERR "failed to focus move\r\n");
    }

    i2c_put_adapter(adap);

    return ret;
}

int siasun_mipi_procedure(struct siasun_video_dev *svid)
{
    int ret = 0;
    struct i2c_adapter *adap = NULL;

    adap = i2c_get_adapter(svid->bus_num);
    if (!adap)
    {
        printk(KERN_ERR "video[%d] get adapter failed\n", svid->index);
        return -1;
    }

    DPRINTK("video[%d] bridge init\n", svid->index);
    ret = siasun_bridge_init(adap);
    if (ret)
    {
        i2c_put_adapter(adap);
        printk(KERN_ERR "video[%d] bridge init failed\n", svid->index);
        return ret;
    }

    DPRINTK("video[%d] camera init\n", svid->index);
    ret = siasun_camera_init(adap);
    if (ret)
    {
        i2c_put_adapter(adap);
        printk(KERN_ERR "video[%d] camera init failed\n", svid->index);
        return ret;
    }

    mdelay(10);

    ret = siasun_i2c_clk_init(svid, I2C_MAX_CLK_RATE, I2C_MAX_RATE);
    if (ret)
    {
        printk(KERN_ERR "failed to init GMSL RX i2c\r\n");
        i2c_put_adapter(adap);
        return ret;
    }

    ret = siasun_i2c_gmslwrite(adap, MIPI_AF_I2C_ADDR, 0x01, 0X4C);
    if (ret < 0)
    {
        printk(KERN_ERR "failed to focus move\r\n");
    }

    i2c_put_adapter(adap);

    return ret;
}
