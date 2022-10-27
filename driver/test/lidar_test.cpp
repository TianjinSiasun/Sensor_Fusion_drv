#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/time.h>
#include <time.h>
#include <sys/mman.h>
#include "../lib/libsfconfig.h"

typedef enum
{
    MENU_INIT_LIDAR = 0,
    MENU_READ_DEVICE_INFO,
    MENU_SET_LIDAR_TARGET_ANGLE,
    MENU_SET_LIDAR_UTC_TIME,
    MENU_SET_FPS,
    MENU_SET_LIDAR_ETH,
    MENU_SET_PIB_ETH,
    MENU_SET_LIDAR_FOV,
    MENU_SET_LIDAR_MOT_PHASE,
    MENU_QUIT = 99
} MENU_ID;

unsigned short swap_16(unsigned short val)
{
    return (((val & 0xff00) >> 8) | ((val & 0x00ff) << 8));
}

void UI_ShowMenu(void)
{
    printf("==============================\r\n");
    printf("[%d]: INIT LIDAR\n", MENU_INIT_LIDAR);
    printf("[%d]: Read LIDAR Info\n", MENU_READ_DEVICE_INFO);
    printf("[%d]: Set LIDAR Target Angle\n", MENU_SET_LIDAR_TARGET_ANGLE);
    printf("[%d]: Set LiDAR UTC Time\n", MENU_SET_LIDAR_UTC_TIME);
    printf("[%d]: Set LIDAR FPS\n", MENU_SET_FPS);
    printf("[%d]: Set LIDAR ETH\n", MENU_SET_LIDAR_ETH);
    printf("[%d]: Set PIB ETH\n", MENU_SET_PIB_ETH);
    printf("[%d]: Set LIDAR FOV\n", MENU_SET_LIDAR_FOV);
    printf("[%d]: Set LIDAR MOT PHASE\n", MENU_SET_LIDAR_MOT_PHASE);
    printf("[%d]: Quit\r\n", MENU_QUIT);
    printf("Plesae input your selection:");
}

int UI_UserSelect(void)
{
    int nSel;
    scanf("%d", &nSel);
    return nSel;
}

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
    int nSel, bQuit = 0;
    char lidar_dev[] = "/dev/rs16_dev";
    unsigned short value;
    unsigned char localip[4] = {192, 168, 1, 200};
    unsigned char destip[4] = {192, 168, 1, 102};
    unsigned char mac[6] = {64, 44, 110, 130, 104, 226}; //本机

    RS16UcwpPkt ucwp;
    RS16DifopPkt difop;
    RSTimestampYMD time;
    unsigned char fps;
    unsigned short speed;
    RSFOV fov;
    unsigned int angle1, angle2;
    RSEthNet lidar_eth;
    PIBEthNet pib_eth;
    unsigned short mot_phase;

    fd_lidar = open(lidar_dev, O_RDWR);
    if (fd_lidar < 0)
    {
        printf("open %s FAILD!\n", lidar_dev);
        return -1;
    }

    while (!bQuit)
    {
        UI_ShowMenu();
        nSel = UI_UserSelect();
        switch (nSel)
        {
        case MENU_INIT_LIDAR:
            ret = Lidar_Init(fd_lidar);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            break;

        case MENU_READ_DEVICE_INFO:
            ret = Lidar_Get_DIFOP(fd_lidar, &difop);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            difop.id = 0xa5ff005A11115555;
            printf("id = %#lx\n", difop.id);
            printf("rpm = %d\n", swap_16(difop.rpm));

            printf("eth.host_ip = %d.%d.%d.%d\n", difop.eth.host_ip[0], difop.eth.host_ip[1], difop.eth.host_ip[2], difop.eth.host_ip[3]);
            printf("eth.lidar_ip = %d.%d.%d.%d\n", difop.eth.lidar_ip[0], difop.eth.lidar_ip[1], difop.eth.lidar_ip[2], difop.eth.lidar_ip[3]);
            printf("eth.mac_addr = %d-%d-%d-%d-%d-%d\n", difop.eth.mac_addr[0], difop.eth.mac_addr[1], difop.eth.mac_addr[2], difop.eth.mac_addr[3], difop.eth.mac_addr[4], difop.eth.mac_addr[5]);
            printf("eth.port1 = %d\n", swap_16(difop.eth.port1));
            printf("eth.port2 = %d\n", swap_16(difop.eth.port2));
            printf("eth.port3 = %d\n", swap_16(difop.eth.port3));
            printf("eth.port4 = %d\n", swap_16(difop.eth.port4));

            printf("fov_start = %d\n", swap_16(difop.fov.fov_start));
            printf("fov_end = %d\n", swap_16(difop.fov.fov_end));
            printf("phase_lock_angle = %d\n", swap_16(difop.phase_lock_angle));
            printf("top_ver = T%xR%xV%x_T%x_%x\n", difop.version.top_ver[0], difop.version.top_ver[1], difop.version.top_ver[2], difop.version.top_ver[3], difop.version.top_ver[4]);
            printf("bot_ver = B%xR%xV%x_T%x_%x\n", difop.version.bot_ver[0], difop.version.bot_ver[1], difop.version.bot_ver[2], difop.version.bot_ver[3], difop.version.bot_ver[4]);
            printf("sn = %X%X%X%X%X%X\n", difop.sn.num[0], difop.sn.num[1], difop.sn.num[2], difop.sn.num[3], difop.sn.num[4], difop.sn.num[5]);
            printf("zero_cali = %d\n", swap_16(difop.zero_cali));
            printf("echo_mode = %d\n", difop.echo_mode);
            printf("sw_ver = %d\n", swap_16(difop.sw_ver));
            printf("timestamp : %d-%d-%d  %d:%d:%d:%d:%d\n", difop.timestamp.year + 2000, difop.timestamp.month, difop.timestamp.day, difop.timestamp.hour, difop.timestamp.minute, difop.timestamp.second, swap_16(difop.timestamp.ms), swap_16(difop.timestamp.us));

            printf("status.idat1_reg = %d\n", ((difop.status.idat1_reg[0] & 0x7f) << 16) | (difop.status.idat1_reg[1] << 8) | difop.status.idat1_reg[2]);
            printf("status.idat2_reg = %d\n", ((difop.status.idat2_reg[0] & 0x7f) << 16) | (difop.status.idat2_reg[1] << 8) | difop.status.idat2_reg[2]);
            printf("status.vdat_12v_m_reg = %d\n", swap_16(difop.status.vdat_12v_m_reg));
            printf("status.vdat_12v_reg = %d\n", swap_16(difop.status.vdat_12v_reg));
            printf("status.vdat_5v_reg = %d\n", swap_16(difop.status.vdat_5v_reg));
            printf("status.vdat_3v3_reg = %d\n", swap_16(difop.status.vdat_3v3_reg));
            printf("status.vdat_2v5_reg = %d\n", swap_16(difop.status.vdat_2v5_reg));
            printf("status.vdat_1v2_reg = %d\n", swap_16(difop.status.vdat_1v2_reg));

            printf("diagno.cksum_st = %d\n", difop.diagno.cksum_st);
            printf("diagno.cur_rpm1 = %d,diagno.cur_rpm2 = %d\n", difop.diagno.cur_rpm1, difop.diagno.cur_rpm2);
            printf("diagno.gps_status = %d\n", difop.diagno.gps_status);
            printf("diagno.manc_err1 = %d\n", swap_16(difop.diagno.manc_err1));
            printf("diagno.manc_err2 = %d\n", swap_16(difop.diagno.manc_err2));
            printf("diagno.temperature1 = %d\n", swap_16(difop.diagno.temperature1));
            printf("diagno.temperature2 = %d\n", swap_16(difop.diagno.temperature2));
            printf("diagno.temperature3 = %d\n", swap_16(difop.diagno.temperature3));
            printf("diagno.temperature4 = %d\n", swap_16(difop.diagno.temperature4));
            printf("diagno.temperature5 = %d\n", swap_16(difop.diagno.temperature5));
            for (int i = 0; i < 16; i++)
            {
                printf("ver_angle_cali = %d\n", (difop.ver_angle_cali[i].value[0] << 16) | (difop.ver_angle_cali[i].value[1] << 8) | difop.ver_angle_cali[i].value[2]);
            }

            printf("tail = %#x\n", difop.tail);
            break;

        case MENU_SET_LIDAR_UTC_TIME:
            printf("input year(2000~2255):");
            scanf("%hd", &value);
            time.year = value - 2000;
            printf("input month(1~12):");
            scanf("%hhd", &time.month);
            printf("input day(1~31):");
            scanf("%hhd", &time.day);
            printf("input hour(0~23):");
            scanf("%hhd", &time.hour);
            printf("input min(0~59):");
            scanf("%hhd", &time.minute);
            printf("input sec(0~59):");
            scanf("%hhd", &time.second);
            printf("input ms(0~999):");
            scanf("%hd", &time.ms);
            printf("input us(0~999):");
            scanf("%hd", &time.us);

            ret = Lidar_Set_TimeStamp(fd_lidar, time);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            break;
        case MENU_SET_FPS:
            printf("input lidar fps: ");
            scanf("%hhd", &fps);
            ret = Lidar_Set_fps(fd_lidar, fps);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            break;

        case MENU_SET_LIDAR_TARGET_ANGLE:
            printf("input lidar target angle: ");
            scanf("%d", &angle1);
            ret = Lidar_Set_Target_Angle(fd_lidar, angle1);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            usleep(1000);
            ioctl(fd_lidar, LIDAR_TARGET_ANGLE_GET, &angle2); 
            printf("lidar target angle is set to %d\n", angle2);
            break;

        case MENU_SET_LIDAR_FOV:
            printf("input FOV_START:");
            scanf("%hd", &fov.fov_start);
            printf("input FOV_END:");
            scanf("%hd", &fov.fov_end);
            ret = Lidar_Set_FOV(fd_lidar, fov);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            break;

        case MENU_SET_LIDAR_ETH:
            printf("input lidar_ip:");
            scanf("%hhd.%hhd.%hhd.%hhd", &lidar_eth.lidar_ip[0], &lidar_eth.lidar_ip[1], &lidar_eth.lidar_ip[2], &lidar_eth.lidar_ip[3]);
            printf("input host_ip:");
            scanf("%hhd.%hhd.%hhd.%hhd", &lidar_eth.host_ip[0], &lidar_eth.host_ip[1], &lidar_eth.host_ip[2], &lidar_eth.host_ip[3]);
            printf("input mac_addr:");
            scanf("%hhd-%hhd-%hhd-%hhd-%hhd-%hhd", &lidar_eth.mac_addr[0], &lidar_eth.mac_addr[1], &lidar_eth.mac_addr[2], &lidar_eth.mac_addr[3], &lidar_eth.mac_addr[4], &lidar_eth.mac_addr[5]);

            printf("input port1:");
            scanf("%hd", &lidar_eth.port1);
            printf("input port2:");
            scanf("%hd", &lidar_eth.port2);
            printf("input port3:");
            scanf("%hd", &lidar_eth.port3);
            printf("input port4:");
            scanf("%hd", &lidar_eth.port4);
            ret = Lidar_Set_Ethnet(fd_lidar, lidar_eth);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            break;

        case MENU_SET_PIB_ETH:
            printf("input pib ip:");
            scanf("%hhd.%hhd.%hhd.%hhd", &pib_eth.pib_ip[0], &pib_eth.pib_ip[1], &pib_eth.pib_ip[2], &pib_eth.pib_ip[3]);
            printf("input msop port:");
            scanf("%hd", &pib_eth.msop_port);
            printf("input difop port:");
            scanf("%hd", &pib_eth.difop_port);
            ret = PIB_Set_Ethnet(fd_lidar, pib_eth);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            break;

        case MENU_SET_LIDAR_MOT_PHASE:
            printf("input motor phase: ");
            scanf("%hd", &mot_phase);
            ret = Lidar_Set_Motor_Phase(fd_lidar, mot_phase);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            break;

        case MENU_QUIT:
            bQuit = 1;
            printf("Bye!\r\n");
            break;

        default:
            break;
        }
    }

    Lidar_Close(fd_lidar);
    return 0;
}
