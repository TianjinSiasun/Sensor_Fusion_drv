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

unsigned char lidarip[4] = {192, 168, 1, 200};
unsigned char hostip[4] = {192, 168, 1, 102};
unsigned char mac[6] = {64, 44, 110, 130, 104, 226}; //本机

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
    unsigned char *lidar_output = (unsigned char *)malloc(300 * 1024);
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

    memset(lidar_output, 0, 300 * 1024);
    Lidar_mmap_Addr(fd_lidar, &phy_addr);
    lidar_output = (unsigned char *)mmap(0, 300 * 1024, PROT_READ, MAP_SHARED, fd_lidar, phy_addr);
    IRQ_Mask_Enable(fd_cfg, 0x0); // enable lidar irq
    Buf_Sys_Enable(fd_cfg, 0);    // Disable sys

    fpga_time.mode = 0;
    SYS_TIME_READ((RSTimestampYMD *)&fpga_time.ts);
    FPGA_TimeStamp_Set(fd_cfg, fpga_time); // set fpga time update by lidar

    memcpy(pib_eth.pib_ip, hostip, 4);
    pib_eth.msop_port = MSOP_PORT;
    pib_eth.difop_port = DIFOP_PORT;
    PIB_Set_Ethnet(fd_lidar, pib_eth); // set PIB ETH

    Lidar_Init(fd_lidar);

    SYS_TIME_READ(&lidar_time);
    Lidar_Set_TimeStamp(fd_lidar, lidar_time);

    sleep(10);

    Lidar_Get_DIFOP(fd_lidar, &difop);
    printf("timestamp : %d-%d-%d  %d:%d:%d:%d:%d\n",
           difop.timestamp.year + 2000, difop.timestamp.month, difop.timestamp.day,
           difop.timestamp.hour, difop.timestamp.minute, difop.timestamp.second,
           ((difop.timestamp.ms & 0xff00) >> 8) | ((difop.timestamp.ms & 0x00ff) << 8),
           ((difop.timestamp.us & 0xff00) >> 8) | ((difop.timestamp.us & 0x00ff) << 8));

    Buf_Sys_Enable(fd_cfg, 1);

    IRQ_Mask_Enable(fd_cfg, 0x1);

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

    IRQ_Mask_Enable(fd_cfg, 0x0);
    Buf_Sys_Enable(fd_cfg, 0);

    FPGA_Close(fd_cfg);
    Lidar_Close(fd_lidar);
    return 0;
}
