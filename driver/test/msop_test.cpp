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
#include <time.h>
#include "../lib/libsfconfig.h"

#define MSOP_PORT 6699
#define DIFOP_PORT 7788
#define LIDAR_SIZE (300 * 1024)

unsigned char lidarip[4] = {192, 168, 1, 200};
unsigned char hostip[4] = {192, 168, 1, 102};
unsigned char mac[6] = {64, 44, 110, 130, 104, 226};

void SYS_TIME_READ(RSTimestampYMD *time)
{
    struct timeval tv;
    struct timezone tz;
    struct tm *ptm;
    gettimeofday(&tv, &tz);
    ptm = localtime(&tv.tv_sec);
    time->year = ptm->tm_year + 1900 - 2000;
    time->month = ptm->tm_mon + 1;
    time->day = ptm->tm_mday;
    time->hour = ptm->tm_hour;
    time->minute = ptm->tm_min;
    time->second = ptm->tm_sec;
    time->ms = tv.tv_usec / 1000;
    time->us = tv.tv_usec % 1000;
}

int main(int argc, char *argv[])
{
    int ret = 0;
    int fd_lidar;
    int fd_cfg;
    unsigned int phy_addr;
    RSTimestampYMD lidar_time;
    unsigned char *lidar_output = (unsigned char *)malloc(LIDAR_SIZE);
    TimeStampCfg fpga_time;
    RS16DifopPkt difop;
    RSFOV fov_set;
    unsigned short ms = 0;
    unsigned short us = 0;
    unsigned char pkgnum;
    RS16MsopPkt pkt[64];

    PIBEthNet pib_eth;

    struct timespec a;

    fd_cfg = FPGA_Open();
    if (fd_cfg < 0)
    {
        printf("open /dev/fpga_cdev failed\n");
        return -1;
    }

    fd_lidar = Lidar_Open();
    if (fd_lidar < 0)
    {
        printf("open /dev/rs16_dev failed\n");
        return -1;
    }

    memset(lidar_output, 0, LIDAR_SIZE);
    ret = Lidar_mmap_Addr(fd_lidar, &phy_addr);
    if (ret != 0)
    {
        printf("ERRNO = %d", ret);
        return ret;
    }
    lidar_output = (unsigned char *)mmap(0, LIDAR_SIZE, PROT_READ, MAP_SHARED, fd_lidar, phy_addr);
    ret = IRQ_Mask_Enable(fd_cfg, 0x0); // disable lidar irq
    if (ret != 0)
    {
        printf("ERRNO = %d", ret);
        return ret;
    }
    ret = Buf_Sys_Enable(fd_cfg, 0); // Disable sys
    if (ret != 0)
    {
        printf("ERRNO = %d", ret);
        return ret;
    }

    fpga_time.mode = 0;
    SYS_TIME_READ((RSTimestampYMD *)&fpga_time.ts);
    ret = FPGA_TimeStamp_Set(fd_cfg, fpga_time); // set fpga time update by lidar
    if (ret != 0)
    {
        printf("ERRNO = %d", ret);
        return ret;
    }

    memcpy(pib_eth.pib_ip, hostip, 4);
    pib_eth.msop_port = MSOP_PORT;
    pib_eth.difop_port = DIFOP_PORT;
    ret = PIB_Set_Ethnet(fd_lidar, pib_eth); // set PIB ETH
    if (ret != 0)
    {
        printf("ERRNO = %d", ret);
        return ret;
    }

    ret = ret = Lidar_Init(fd_lidar);
    if (ret != 0)
    {
        printf("ERRNO = %d", ret);
        return ret;
    }

    SYS_TIME_READ(&lidar_time);
    ret = Lidar_Set_TimeStamp(fd_lidar, lidar_time);
    if (ret != 0)
    {
        printf("ERRNO = %d", ret);
        return ret;
    }

    sleep(10);

    ret = Lidar_Get_DIFOP(fd_lidar, &difop);
    if (ret != 0)
    {
        printf("ERRNO = %d", ret);
        return ret;
    }

    printf("timestamp : %d-%d-%d  %d:%d:%d:%d:%d\n",
           difop.timestamp.year + 2000, difop.timestamp.month, difop.timestamp.day,
           difop.timestamp.hour, difop.timestamp.minute, difop.timestamp.second,
           ((difop.timestamp.ms & 0xff00) >> 8) | ((difop.timestamp.ms & 0x00ff) << 8),
           ((difop.timestamp.us & 0xff00) >> 8) | ((difop.timestamp.us & 0x00ff) << 8));

    ret = Buf_Sys_Enable(fd_cfg, 1);
    if (ret != 0)
    {
        printf("ERRNO = %d", ret);
        return ret;
    }

    ret = IRQ_Mask_Enable(fd_cfg, 0x1);
    if (ret != 0)
    {
        printf("ERRNO = %d", ret);
        return ret;
    }

    ret = Lidar_Get_MSOP(fd_lidar, lidar_output, &pkgnum, pkt); // fist msop packet need abandon
    for (int i = 0; i < 20; i++)
    {
        ret = Lidar_Get_MSOP(fd_lidar, lidar_output, &pkgnum, pkt);

        clock_gettime(0, &a);

        ms = pkt[pkgnum - 1].header.timestamp.ms;
        ms = (ms & 0xff) << 8 | (ms & 0xff00) >> 8;

        us = pkt[pkgnum - 1].header.timestamp.us;
        us = (us & 0xff) << 8 | (us & 0xff00) >> 8;

        printf("[%d]pkt[%d]: %d-%d-%d %hu-%hu --------------", i, pkgnum, pkt[pkgnum - 1].header.timestamp.hour, pkt[pkgnum - 1].header.timestamp.minute, pkt[pkgnum - 1].header.timestamp.second, ms, us);
        printf("time: %.9fs\n", ((double)a.tv_nsec * 1.0e-9 + (double)a.tv_sec));
    }

    ret = IRQ_Mask_Enable(fd_cfg, 0x0);
    if (ret != 0)
    {
        printf("ERRNO = %d", ret);
        return ret;
    }
    ret = Buf_Sys_Enable(fd_cfg, 0);
    if (ret != 0)
    {
        printf("ERRNO = %d", ret);
        return ret;
    }

    FPGA_Close(fd_cfg);
    Lidar_Close(fd_lidar);
    return 0;
}
