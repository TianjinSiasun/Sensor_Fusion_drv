/* SPDX-License-Identifier: GPL-2.0 */
/* Linux fpga schedule Driver header file */

#ifndef __FPGA_SCHEDULE_H__
#define __FPGA_SCHEDULE_H__

#include "sensor_fusion_register.h"
#include "fpga_cdev.h"

void fpga_set_buf_control(struct fpga_cdev *fpga, int go);
void fpga_buffer_config(struct fpga_cdev *fpga, BufCfg cfg);

unsigned int fpga_get_lidar_status(struct fpga_cdev *fpga);
void lidar_irq_status_clean(struct fpga_cdev *fpga);
unsigned int fpga_get_camera0_status(struct fpga_cdev *fpga);
void camera0_irq_status_clean(struct fpga_cdev *fpga);
unsigned int fpga_get_camera1_status(struct fpga_cdev *fpga);
void camera1_irq_status_clean(struct fpga_cdev *fpga);
unsigned int fpga_get_camera2_status(struct fpga_cdev *fpga);
void camera2_irq_status_clean(struct fpga_cdev *fpga);
unsigned int fpga_get_camera3_status(struct fpga_cdev *fpga);
void camera3_irq_status_clean(struct fpga_cdev *fpga);

void fpga_set_camera_target_mode(struct fpga_cdev *fpga, unsigned int mode);

void fpga_set_camera_angles(struct fpga_cdev *fpga, SyncCfg cfg);
void fpga_get_current_angle_fps(struct fpga_cdev *fpga, SyncCfg *cfg);
void fpga_set_time_mode(struct fpga_cdev *fpga, unsigned int data);
void fpga_time_set(struct fpga_cdev *fpga, TimeStampCfg timecfg);
void fpga_time_get(struct fpga_cdev *fpga, TimeStamp *time);
void fpga_re_go(struct fpga_cdev *fpga);

void irq_mask_config(struct fpga_cdev *fpga, unsigned int mask);

#endif
