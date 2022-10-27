/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/kernel.h>
#include <linux/delay.h>
#include "fpga_schedule.h"

void fpga_write32(void *ipbase, unsigned int offset, unsigned int data)
{
    writel(data, ipbase + offset * 4);
}

unsigned int fpga_read32(void *ipbase, unsigned int offset)
{
    return readl(ipbase + offset * 4);
}

/* BUFFER CONFIG REG */
/*REG_BUF_CONTROL bit0 : set 0 stop buf&sync managements; set 1 start it*/
void fpga_set_buf_control(struct fpga_cdev *fpga, int go)
{
    fpga_write32(fpga->vip_schedule, REG_BUF_CONTROL, (go > 0 ? 1 : 0));
    DPRINTK("REG_BUF_CONTROL = %#x\n", fpga_read32(fpga->vip_schedule, REG_BUF_CONTROL));
}

/*number of mixed buffers used in memory*/
void fpga_set_frame_number(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_FRAME_NUMBER, data);
    DPRINTK("REG_FRAME_NUMBER = %#x\n", fpga_read32(fpga->vip_schedule, REG_FRAME_NUMBER));
}

/*size(bytes) of each mixed buffer in memory*/
void fpga_set_frame_offset(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_FRAME_OFFSET, data);
    DPRINTK("REG_FRAME_OFFSET = %#x\n", fpga_read32(fpga->vip_schedule, REG_FRAME_OFFSET));
}

/*size of each camera buffers that one by one*/
void fpga_set_camera_offset(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_CAMERA_OFFSET, data);
    DPRINTK("REG_CAMERA_OFFSET = %#x\n", fpga_read32(fpga->vip_schedule, REG_CAMERA_OFFSET));
}

/*size of lidar buffer, it must larger than 4*camera buffers*/
void fpga_set_lidar_offset(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_LIDAR_OFFSET, data);
    DPRINTK("REG_LIDAR_OFFSET = %#x\n", fpga_read32(fpga->vip_schedule, REG_LIDAR_OFFSET));
}

/**
 *LIDAR STATUS
 *bit31-24: BUF_ID
 *bit23-16:packets
 *bit4-2:Lidar writer status
 *bit1:irq occur when irq is high
 *bit0:irq(must clearn by manually)
 */
unsigned int fpga_get_lidar_status(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->vip_schedule, REG_LIDAR_STATUS);
    return data;
}

void lidar_irq_status_clean(struct fpga_cdev *fpga)
{
    fpga_write32(fpga->pcie_base, 0x80 + REG_LIDAR_STATUS, 1);
}

/**
 *CAMERA  STATUS
 *bit31-24: BUF_ID
 *bit4-2:camera writer status
 *bit1:irq occur when irq is high
 *bit0:irq(must clearn by manually)
 */
unsigned int fpga_get_camera0_status(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->vip_schedule, REG_CAMERA0_STATUS);
    return data;
}
void camera0_irq_status_clean(struct fpga_cdev *fpga)
{
    fpga_write32(fpga->pcie_base, 0x80 + REG_CAMERA0_STATUS, 1);
}

unsigned int fpga_get_camera1_status(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->vip_schedule, REG_CAMERA1_STATUS);
    return data;
}
void camera1_irq_status_clean(struct fpga_cdev *fpga)
{
    fpga_write32(fpga->pcie_base, 0x80 + REG_CAMERA1_STATUS, 1);
}

unsigned int fpga_get_camera2_status(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->vip_schedule, REG_CAMERA2_STATUS);
    return data;
}
void camera2_irq_status_clean(struct fpga_cdev *fpga)
{
    fpga_write32(fpga->pcie_base, 0x80 + REG_CAMERA2_STATUS, 1);
}

unsigned int fpga_get_camera3_status(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->vip_schedule, REG_CAMERA3_STATUS);
    return data;
}
void camera3_irq_status_clean(struct fpga_cdev *fpga)
{
    fpga_write32(fpga->pcie_base, 0x80 + REG_CAMERA3_STATUS, 1);
}

/*aotu calculate and load by library but not used*/
void fpga_set_first_sensor(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_FIRST_SENSOR, data);
}

/*Disable IRQ: bit0-3:camera0-camera3; bit7:lidar*/
void fpga_set_irq_disable(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_IRQ_DISABLE, data);
}

/* SYNC management */
/*set the lidar period (us)*/
void fpga_set_sensor_period(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_SENSOR_PERIOD, data);
}
unsigned int fpga_get_sensor_period(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->vip_schedule, REG_SENSOR_PERIOD);
    return data;
}

/*camera shift regiaster used to generate trigger*/
void fpga_set_camera0_shift(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_CAMERA0_SHIFT, data);
}
unsigned int fpga_get_camera0_shift(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->vip_schedule, REG_CAMERA0_SHIFT);
    return data;
}

void fpga_set_camera1_shift(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_CAMERA1_SHIFT, data);
}
unsigned int fpga_get_camera1_shift(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->vip_schedule, REG_CAMERA1_SHIFT);
    return data;
}

void fpga_set_camera2_shift(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_CAMERA2_SHIFT, data);
}
unsigned int fpga_get_camera2_shift(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->vip_schedule, REG_CAMERA2_SHIFT);
    return data;
}

void fpga_set_camera3_shift(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_CAMERA3_SHIFT, data);
}
unsigned int fpga_get_camera3_shift(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->vip_schedule, REG_CAMERA3_SHIFT);
    return data;
}

/*timestamp compensationvalue of each camera*/
void fpga_set_camera0_compensation(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_CAMERA0_COMPENSATE, data);
}

unsigned int fpga_get_camera0_compensation(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->vip_schedule, REG_CAMERA0_COMPENSATE);
    return data;
}

void fpga_set_camera1_compensation(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_CAMERA1_COMPENSATE, data);
}

unsigned int fpga_get_camera1_compensation(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->vip_schedule, REG_CAMERA1_COMPENSATE);
    return data;
}

void fpga_set_camera2_compensation(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_CAMERA2_COMPENSATE, data);
}

unsigned int fpga_get_camera2_compensation(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->vip_schedule, REG_CAMERA2_COMPENSATE);
    return data;
}

void fpga_set_camera3_compensation(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_CAMERA3_COMPENSATE, data);
}

unsigned int fpga_get_camera3_compensation(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->vip_schedule, REG_CAMERA3_COMPENSATE);
    return data;
}

void fpga_set_trigger_dead_time(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->vip_schedule, REG_TRIGGER_DEAD_TIME, data);
}

/*bit0 is the pattern bit : 0-taget to lidar, 1-target to a internal trigger*/
void fpga_set_camera_target_mode(struct fpga_cdev *fpga, unsigned int mode)
{
    fpga_write32(fpga->vip_schedule, REG_CAMERA_TARGET_MODE, (mode > 0 ? 1 : 0));
    DPRINTK("REG_CAMERA_TARGET_MODE = %#x\n", fpga_read32(fpga->vip_schedule, REG_CAMERA_TARGET_MODE));
}

/* Timestamp */
/*set_time-mode 1-updated by PCIe ;0-updated by lidar*/
void fpga_set_time_mode(struct fpga_cdev *fpga, unsigned int mode)
{
    fpga_write32(fpga->timestamp, REG_TIME_MODE, (mode > 0 ? 1 : 0));
}

/*set the value of h,m,s of FPGA timestamp through PCIe
 *iput data = (h<<16)|(m<<8)|s)
 */
void fpga_set_time_hms(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->timestamp, REG_TIME_BIG_SET, data);
}

/*set the value of ms,us of FPGA timestamp through PCIe
 *iput data = (ms<<16)|us)
 */
void fpga_set_time_msus(struct fpga_cdev *fpga, unsigned int data)
{
    fpga_write32(fpga->timestamp, REG_TIME_SMALL_SET, data);
}

/*set the value of y,m,d of FPGA timestamp through PCIe
 * bit23-16:year
 * bit15-8:month
 * bit7-0:day
 */
unsigned int fpga_get_time_ymd(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->timestamp, REG_TIME_HIGH_READ);
    return data;
}

/*
 *bit23-16:hour
 *bit15-8:minute
 *bit7-0:second
 */
unsigned int fpga_get_time_hms(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->timestamp, REG_TIME_BIG_READ);
    return data;
}

/*
 *bit31-16:ms
 *bit15-0:us
 */
unsigned int fpga_get_time_msus(struct fpga_cdev *fpga)
{
    unsigned int data = fpga_read32(fpga->timestamp, REG_TIME_SMALL_READ);
    return data;
}

void fpga_buffer_config(struct fpga_cdev *fpga, BufCfg cfg)
{
    fpga_set_frame_number(fpga, cfg.frame_num);
    fpga_set_frame_offset(fpga, cfg.frame_offset);
    fpga_set_camera_offset(fpga, cfg.camera_offset);
    fpga_set_lidar_offset(fpga, cfg.lidar_offset);
}

void fpga_set_camera_angles(struct fpga_cdev *fpga, SyncCfg cfg)
{
    unsigned int sensor_period, camera0_shift, camera1_shift, camera2_shift, camera3_shift;

    /* set camera angles for each camera */
    sensor_period = (unsigned int)(1000000 / cfg.fps);
    camera0_shift = (unsigned int)(cfg.camera0_angle * sensor_period / 360);
    camera1_shift = (unsigned int)(cfg.camera1_angle * sensor_period / 360);
    camera2_shift = (unsigned int)(cfg.camera2_angle * sensor_period / 360);
    camera3_shift = (unsigned int)(cfg.camera3_angle * sensor_period / 360);

    fpga_set_sensor_period(fpga, sensor_period);
    fpga_set_camera0_shift(fpga, camera0_shift);
    fpga_set_camera1_shift(fpga, camera1_shift);
    fpga_set_camera2_shift(fpga, camera2_shift);
    fpga_set_camera3_shift(fpga, camera3_shift);

    /* set  compensation values for each camera */
    fpga_set_camera0_compensation(fpga, cfg.camera0_comp);
    fpga_set_camera0_compensation(fpga, cfg.camera1_comp);
    fpga_set_camera0_compensation(fpga, cfg.camera2_comp);
    fpga_set_camera0_compensation(fpga, cfg.camera3_comp);

    fpga_set_trigger_dead_time(fpga, cfg.trigger_deadtime);
}

void fpga_get_current_angle_fps(struct fpga_cdev *fpga, SyncCfg *cfg)
{
    unsigned int sensor_period, camera0_shift, camera1_shift, camera2_shift, camera3_shift;
    sensor_period = fpga_get_sensor_period(fpga);
    camera0_shift = fpga_get_camera0_shift(fpga);
    camera1_shift = fpga_get_camera1_shift(fpga);
    camera2_shift = fpga_get_camera2_shift(fpga);
    camera3_shift = fpga_get_camera3_shift(fpga);

    cfg->fps = 1000000.0 / sensor_period;
    cfg->camera0_angle = (int)(camera0_shift * 360.0 / sensor_period + 0.5);
    cfg->camera1_angle = (int)(camera1_shift * 360.0 / sensor_period + 0.5);
    cfg->camera2_angle = (int)(camera2_shift * 360.0 / sensor_period + 0.5);
    cfg->camera3_angle = (int)(camera3_shift * 360.0 / sensor_period + 0.5);

    cfg->camera0_comp = fpga_get_camera0_compensation(fpga);
    cfg->camera1_comp = fpga_get_camera0_compensation(fpga);
    cfg->camera2_comp = fpga_get_camera0_compensation(fpga);
    cfg->camera3_comp = fpga_get_camera0_compensation(fpga);
}

void fpga_time_set(struct fpga_cdev *fpga, TimeStampCfg timecfg)
{
    fpga_set_time_mode(fpga, timecfg.mode);
    if (timecfg.mode == 0)
    {
        return;
    }

    fpga_set_time_hms(fpga, (timecfg.ts.hour << 16) | (timecfg.ts.minute << 8) | timecfg.ts.second);
    fpga_set_time_msus(fpga, (timecfg.ts.ms << 16) | timecfg.ts.us);
}

void fpga_time_get(struct fpga_cdev *fpga, TimeStamp *time)
{
    unsigned int fpga_date, fpga_time, fpga_time_us;

    fpga_date = fpga_get_time_ymd(fpga);
    fpga_time = fpga_get_time_hms(fpga);
    fpga_time_us = fpga_get_time_msus(fpga);

    time->year = (fpga_date >> 16) & 0xff;
    time->month = (fpga_date >> 8) & 0xff;
    time->day = fpga_date & 0xff;
    time->hour = (fpga_time >> 16) & 0xff;
    time->minute = (fpga_time >> 8) & 0xff;
    time->second = fpga_time & 0xff;
    time->ms = (fpga_time_us >> 16) & 0x3ff;
    time->us = fpga_time_us & 0x3ff;
}

/* first turn off go bit of the buf,and then turn it on */
void fpga_re_go(struct fpga_cdev *fpga)
{
    mdelay(100);
    fpga_set_buf_control(fpga, 0);
    mdelay(50);
    fpga_set_buf_control(fpga, 1);
}

void irq_mask_config(struct fpga_cdev *fpga, unsigned int mask)
{
    writel(mask, fpga->pcie_base + PCIE_DMA_CTL_PCIE_IRQ_DMA_IRQ_MASK_SLAVE_BASE);
    DPRINTK("PCIE_DMA_CTL_PCIE_IRQ_DMA_IRQ_MASK_SLAVE_BASE = %#x\n", readl(fpga->pcie_base + PCIE_DMA_CTL_PCIE_IRQ_DMA_IRQ_MASK_SLAVE_BASE));
}
