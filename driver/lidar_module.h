#ifndef __LIDAR_MODULE_H__
#define __LIDAR_MODULE_H__

#include "rs_lidar.h"
#include "lidar_registers.h"

void lidar_set_lidar_control_reg(struct lidar_cdev *lidar, unsigned char data);
void lidar_set_target_angle(struct lidar_cdev *lidar, unsigned int angle);
unsigned int lidar_get_target_angle(struct lidar_cdev *lidar);

void lidar_set_motor_speed(struct lidar_cdev *lidar, unsigned short rpm);
void lidar_set_ethnet(struct lidar_cdev *lidar, RSEthNet eth);
void lidar_set_fov_set(struct lidar_cdev *lidar, RSFOV fov_set);
void lidar_set_mot_phase(struct lidar_cdev *lidar, unsigned short mot_phase);
void lidar_set_utc_time(struct lidar_cdev *lidar, RSTimestampYMD timestamp);

unsigned short lidar_get_motor_speed(struct lidar_cdev *lidar);
void lidar_get_ethnet(struct lidar_cdev *lidar, RSEthNet *eth);
void lidar_get_fov_set(struct lidar_cdev *lidar, RSFOV *fov_set);
unsigned short lidar_get_mot_phase(struct lidar_cdev *lidar);
void lidar_get_top_frm(struct lidar_cdev *lidar, unsigned char *top_ver);
void lidar_get_bot_frm(struct lidar_cdev *lidar, unsigned char *bot_ver);
void lidar_get_frm_ver(struct lidar_cdev *lidar, RSVersion *version);
void lidar_get_cor_vert_ang(struct lidar_cdev *lidar, unsigned int ch, RSCalibrationAngle *ver_angle_cali);
void lidar_get_serial_number(struct lidar_cdev *lidar, RSSn *sn);
unsigned short lidar_get_software_ver(struct lidar_cdev *lidar);
void lidar_get_utc_time(struct lidar_cdev *lidar, RSTimestampYMD *timestamp);
void lidar_get_status(struct lidar_cdev *lidar, RSStatus *status);
void lidar_get_diagno(struct lidar_cdev *lidar, RSDiagno *diagno);
unsigned int lidar_get_echo_mode(struct lidar_cdev *lidar);

int lidar_init_ucwp(struct lidar_cdev *lidar);
void lidar_send_ucwp(struct lidar_cdev *lidar);

void lidar_set_pib_eth(struct lidar_cdev *lidar, PIBEthNet eth);

#endif /* __LIDAR_MODULE_H__ */
