#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <errno.h>
#include "libsfconfig.h"

int Check_Lidar_RPM(unsigned short rpm)
{
    if ((rpm != LIDAR_MOTOR_SPEED_20_FPS) && (rpm != LIDAR_MOTOR_SPEED_10_FPS) && (rpm != LIDAR_MOTOR_SPEED_5_FPS))
    {
        printf("ERR: lidar fps can only be set to : 20、10 or 5.\n");
        return -EINVAL;
    }
    return 0;
}

int Check_Lidar_Ethnet(RSEthNet eth)
{

    if ((eth.lidar_ip[0] != 192) || (eth.lidar_ip[1] != 168) || (eth.lidar_ip[2] != 1))
    {
        printf("Err: lidar ip must be set to 192.168.1.x\n");
        return -EINVAL;
    }
    if ((eth.host_ip[0] != 192) || (eth.host_ip[1] != 168) || (eth.host_ip[2] != 1))
    {
        printf("Err: host ip must be set to 192.168.1.x\n");
        return -EINVAL;
    }
    if (eth.host_ip[3] == eth.lidar_ip[3])
    {
        printf("Err: lidar ip and host ip cann't be repeated\n");
        return -EINVAL;
    }
    if ((eth.port1 == eth.port3) || (eth.port2 == eth.port4))
    {
        printf("Err: lidar ports input repeated\n");
        return -EINVAL;
    }
    return 0;
}

int Check_Lidar_FOV(RSFOV fov)
{
    if ((fov.fov_start > 36000) || (fov.fov_end > 36000))
    {
        printf("ERR:The set fov is outside the range of 0 to 36000 .\n");
        return -EINVAL;
    }
    return 0;
}
int Check_Lidar_TimeStamp(RSTimestampYMD timestamp)
{
    if (timestamp.year > 225)
    {
        printf("Err: The set (year + 2000) is outside the range of 2000 to 2255 .\n");
        return -EINVAL;
    }
    if (timestamp.month > 12)
    {
        printf("Err:  The set month is outside of the range of 1 to 12 .\n");
        return -EINVAL;
    }
    if (timestamp.month == 4)
    {
        if (((((timestamp.year + 2000) % 4) == 0) && (((timestamp.year + 2000) % 100) != 0)) ||
            (((timestamp.year + 2000) % 400) == 0))
        {
            if (timestamp.day > 29)
            {
                printf("Err:The set day  in April is outside of the range of 1 to 29 .\n");
                return -EINVAL;
            }
        }
        else
        {
            if (timestamp.day > 28)
            {
                printf("Err: The set day in April is outside of the range of 1 to 28 .\n");
                return -EINVAL;
            }
        }
    }
    else
    {
        if ((timestamp.month == 1) || (timestamp.month == 3) || (timestamp.month == 5) ||
            (timestamp.month == 7) || (timestamp.month == 8) || (timestamp.month == 10) ||
            (timestamp.month == 12))
        {
            if (timestamp.day > 31)
            {
                printf("Err:The set day is outside of the range of 1 to 31 .\n");
                return -EINVAL;
            }
        }
        else
        {
            if (timestamp.day > 30)
            {
                printf("Err:The set day is outside of the range of 1 to 30 .\n");
                return -EINVAL;
            }
        }
    }

    if (timestamp.hour >= 24)
    {
        printf("Err:The set hour is outside of the range of 0 to 23 .\n");
        return -EINVAL;
    }
    if (timestamp.minute >= 60)
    {
        printf("Err: The set minute is outside of the range of 0 to 59 .\n");
        return -EINVAL;
    }
    if (timestamp.second >= 60)
    {
        printf("Err: The set second is outside of the range of 0 to 59  \n");
        return -EINVAL;
    }
    if (timestamp.ms > 999)
    {
        printf("Err: The set ms is outside of the range of 0 to 999  \n");
        return -EINVAL;
    }
    if (timestamp.us > 999)
    {
        printf("Err: The set us is outside of the range of 0 to 999 .\n");
        return -EINVAL;
    }
    return 0;
}

int Check_Lidar_Motor_Phase(unsigned short mot_phase)
{
    if ((mot_phase < 0) || (mot_phase > 360))
    {
        printf("ERR: The set mot_phase is outside the range of 0 to 360 .\n");
        return -EINVAL;
    }
    return 0;
}

int Check_Lidar_Param(RS16UcwpPkt ucwp)
{
    int ret = 0;

    ret = Check_Lidar_RPM(ucwp.rpm);
    if (ret < 0)
    {
        return ret;
    }

    ret = Check_Lidar_Ethnet(ucwp.eth);
    if (ret < 0)
    {
        return ret;
    }

    ret = Check_Lidar_FOV(ucwp.fov);
    if (ret < 0)
    {
        return ret;
    }

    ret = Check_Lidar_TimeStamp(ucwp.time);
    if (ret < 0)
    {
        return ret;
    }

    ret = Check_Lidar_Motor_Phase(ucwp.phase_lock_angle);
    if (ret < 0)
    {
        return ret;
    }
    return ret;
}

int Lidar_Open(void)
{
    int fd;
    fd = open("/dev/rs16_dev", O_RDWR);
    return fd;
}

int Lidar_Close(int fd)
{
    int ret = 0;
    ret = close(fd);
    return ret;
}

int Lidar_Init(int fd)
{

    int ret = 0;
    ret = ioctl(fd, LIDAR_INIT);
    return ret;
}

int Lidar_Get_DIFOP(int fd, RS16DifopPkt *difop)
{
    int ret = 0;
    ret = ioctl(fd, DIFOP_GET_PKG, difop);
    return ret;
}

int Lidar_Set_fps(int fd, unsigned int fps)
{
    int ret = 0;
    unsigned int rpm;
    switch (fps)
    {
    case 20:
        rpm = LIDAR_MOTOR_SPEED_20_FPS;
        break;
    case 10:
        rpm = LIDAR_MOTOR_SPEED_10_FPS;
        break;
    case 5:
        rpm = LIDAR_MOTOR_SPEED_5_FPS;
        break;

    default:
        rpm = 0;
        break;
    }
    ret = Check_Lidar_RPM(rpm);
    if (ret < 0)
    {
        return ret;
    }

    ret = ioctl(fd, UCWP_SET_MOTOR_SPEED, &rpm);
    return ret;
}

int Lidar_Set_Ethnet(int fd, RSEthNet eth)
{
    int ret = 0;
    ret = Check_Lidar_Ethnet(eth);
    if (ret < 0)
    {
        return ret;
    }
    ret = ioctl(fd, UCWP_SET_ETH, &eth);
    return ret;
}

int PIB_Set_Ethnet(int fd, PIBEthNet eth)
{
    int ret = 0;
    if ((eth.pib_ip[0] != 192) || (eth.pib_ip[1] != 168) || (eth.pib_ip[2] != 1))
    {
        printf("ERR: pib ip must be set to 192.168.1.x(as same as lidar host ip).\n");
        return -EINVAL;
    }
    if (eth.difop_port == eth.msop_port)
    {
        printf("ERR:disop  port and msop port is repeated\n");
        return -EINVAL;
    }
    ret = ioctl(fd, PIB_SET_ETH, &eth);
    return ret;
}

int Lidar_Set_FOV(int fd, RSFOV fov)
{
    int ret = 0;
    ret = Check_Lidar_FOV(fov);
    if (ret < 0)
    {
        return ret;
    }

    ret = ioctl(fd, UCWP_SET_FOV, &fov);
    return ret;
}

int Lidar_Set_TimeStamp(int fd, RSTimestampYMD timestamp)
{
    int ret = 0;
    ret = Check_Lidar_TimeStamp(timestamp);
    if (ret < 0)
    {
        return ret;
    }

    ret = ioctl(fd, UCWP_SET_UTC_TIME, &timestamp);
    return ret;
}

int Lidar_Set_Motor_Phase(int fd, unsigned short mot_phase)
{
    int ret = 0;
    ret = Check_Lidar_Motor_Phase(mot_phase);
    if (ret < 0)
    {
        return ret;
    }
    ret = ioctl(fd, UCWP_SET_MOT_PHASE, &mot_phase);
    return ret;
}

int Lidar_Set_Target_Angle(int fd, unsigned int angle)
{
    int ret = 0;
    if ((angle < 0) || (angle > 36000))
    {
        printf("ERR: lidar target_angle is outside of the range of 0 to 360.\n");
        return -EINVAL;
    }
    ret = ioctl(fd, LIDAR_TARGET_ANGLE_SET, &angle);
    return ret;
}

int Lidar_mmap_Addr(int fd, unsigned int *phy_addr)
{
    int ret = 0;
    *phy_addr = 0;
    ret = ioctl(fd, MMAP_RHY_OFFSET, phy_addr);
    if (*phy_addr == 0)
    {
        ret = -EPERM;
    }

    return ret;
}

int Lidar_Get_MSOP(int fd, unsigned char *mem, unsigned char *pkgnum, RS16MsopPkt *msop)
{
    int ret = 0;
    unsigned char udp_buf[1248] = {0x55, 0xaa, 0x05, 0x0a, 0x5a, 0xa5, 0x50, 0xa0};
    ret = ioctl(fd, MSOP_GET_PKG, pkgnum);
    for (int i = 0; i < (*pkgnum); i++)
    {
        memcpy(&udp_buf[8], (mem + i * 1240), 1240);
        memcpy(&msop[i], udp_buf, 1248);
    }

    return ret;
}

int FPGA_Open(void)
{
    int fd = 0;
    fd = open("/dev/fpga_cdev", O_RDWR);
    return fd;
}

int FPGA_Close(int fd)
{
    int ret = 0;
    ret = close(fd);
    return ret;
}

int Buf_Sys_Enable(int fd, unsigned char enable)
{
    int ret = 0;
    if (enable)
    {
        ret = ioctl(fd, BUF_SYNS_START);
    }
    else
    {
        ret = ioctl(fd, BUF_SYNS_STOP);
    }
    return ret;
}

int Buf_Config(int fd, BufCfg cfg)
{
    int ret = 0;
    ret = ioctl(fd, BUF_CONFIG, &cfg);
    return ret;
}

int FPGA_TimeStamp_Set(int fd, TimeStampCfg time)
{
    int ret = 0;
    if (time.mode != 0)
    {
        if (time.ts.hour >= 24)
        {
            printf("Err:The set hour is outside of the range of 0 to 23 .\n");
            return -EINVAL;
        }
        if (time.ts.minute >= 60)
        {
            printf("Err: The set minute is outside of the range of 0 to 59 .\n");
            return -EINVAL;
        }
        if (time.ts.second >= 60)
        {
            printf("Err: The set second is outside of the range of 0 to 59  \n");
            return -EINVAL;
        }
        if (time.ts.ms > 999)
        {
            printf("Err: The set ms is outside of the range of 0 to 999  \n");
            return -EINVAL;
        }
        if (time.ts.us > 999)
        {
            printf("Err: The set us is outside of the range of 0 to 999 .\n");
            return -EINVAL;
        }
    }

    ret = ioctl(fd, FPGA_TIME_SET, &time);
    return ret;
}

int FPGA_TimeStamp_Get(int fd, TimeStamp *time)
{
    int ret = 0;
    ioctl(fd, FPGA_TIME_GET, time);
    return ret;
}

int Trigger_Mode_Set(int fd, unsigned char mode)
{
    int ret = 0;
    ioctl(fd, TRIGGER_MODE_SET, &mode);
    return ret;
}

int Camera_Angle_FPS_Set(int fd, SyncCfg sync)
{
    int ret = 0;
    if ((sync.fps != 20) && (sync.fps != 10) && (sync.fps != 5))
    {
        printf("ERR: The set fps can only be set to : 20、10 or 5.\n");
        return -EINVAL;
    }
    if ((sync.camera0_angle > 360) || (sync.camera1_angle > 360) ||
        (sync.camera2_angle > 360) || (sync.camera3_angle > 360))
    {
        printf("ERR: The set camer angle is outside the range of 0 to 360.\n");
        return -EINVAL;
    }
    ret = ioctl(fd, ANGLE_FPS_SET, &sync);
    return ret;
}

int Camera_Angle_FPS_Get(int fd, SyncCfg *sync)
{
    int ret = 0;
    ioctl(fd, ANGLE_FPS_GET, sync);
    return ret;
}

int IRQ_Mask_Enable(int fd, unsigned int mask)
{
    int ret = 0;
    ioctl(fd, IRQ_MASK_CONFIG, &mask);
    return ret;
}
