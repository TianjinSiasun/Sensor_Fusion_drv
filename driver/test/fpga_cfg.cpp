#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/time.h>

#include "../lib/libsfconfig.h"

#define FRAME_NUM 16
#define FRAME_OFFSET 0x4000000
#define CAMERA_OFFSET 0x800000
#define LIDAR_OFFSET 0x3000000

#define CAMERA_COMP  5000
#define DEADTIME  5000

typedef enum
{
    MENU_BUF_SYNC_EN = 0,
    MENU_MODE_SELECT,
    MENU_BUF_CFG,
    MENU_SET_CAM_TARGET_ANGLE,
    MENU_GET_CAM_TARGET_ANGLE,
    MENU_SET_FPGA_TIME,
    MENU_READ_FPGA_TIME,
    MENU_QUIT = 99
} MENU_ID;

void UI_ShowMenu(void)
{
    printf("==============================\r\n");
    printf("[%d]: Enable BUF & SYNC MANAGEMENT\r\n", MENU_BUF_SYNC_EN);
    printf("[%d]: Mode Select(Self or LiDAR Trigger)\r\n", MENU_MODE_SELECT);
    printf("[%d]: Buf Config \r\n", MENU_BUF_CFG);
    printf("[%d]: Set Camera Angle\r\n", MENU_SET_CAM_TARGET_ANGLE);
    printf("[%d]: Get Camera Angle\r\n", MENU_GET_CAM_TARGET_ANGLE);
    printf("[%d]: Set FPGA Time Stamp \r\n", MENU_SET_FPGA_TIME);
    printf("[%d]: Get FPGA Time Stamp \r\n", MENU_READ_FPGA_TIME);
    printf("[%d]: QUIT\n", MENU_QUIT);
    printf("Plesae input your selection:");
}

int main(int argc, char *argv[])
{
    
    int ret = 0;
    int fd;
    int nSel;
    char bQuit = 0;

    fd = FPGA_Open();
    if (!fd)
    {
        printf("open /dev/fpga_cdev FAILD!\n");
        return -1;
    }

    while (!bQuit)
    {
        UI_ShowMenu();
        scanf("%d", &nSel);
        switch (nSel)
        {
        case MENU_BUF_SYNC_EN:
        {
            int num;
            printf("input num: 1-enable, 0-disable\n");
            scanf("%d", &num);
            ret = Buf_Sys_Enable(fd, num);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            break;
        }

        case MENU_MODE_SELECT:
        {
            int mode = 0;
            printf("Plesae input trigger mode: 0-Lidar,1-Self.\n");
            scanf("%d", &mode);
            ret = Trigger_Mode_Set(fd, mode);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
        }
        break;

        case MENU_BUF_CFG:
        {
            int mode;
            BufCfg cfg;
            printf("BUF management config : 0--default, 1--manual:");
                scanf("%d", &mode);
            if (mode == 0)
            {
                printf("default config\n");
                cfg.frame_num = FRAME_NUM;
                cfg.frame_offset =  FRAME_OFFSET;
                cfg.camera_offset = CAMERA_OFFSET;
                cfg.lidar_offset = LIDAR_OFFSET;
            }
            else
            {
                printf("Please input frame num:");
                scanf("%d", &cfg.frame_num);
                printf("Please input frame offset:");
                scanf("%d", &cfg.frame_offset);
                printf("Please input camera offset:");
                scanf("%d", &cfg.camera_offset);
                printf("Please input lidar offset:");
                scanf("%d", &cfg.lidar_offset);
            }
            ret = Buf_Config(fd, cfg);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            break;
        }

        case MENU_SET_CAM_TARGET_ANGLE:
        {
            SyncCfg cfg;
            printf("Please input fps:");
            scanf("%d", &cfg.fps);
            printf("Please input cam 0 angle:");
            scanf("%d", &cfg.camera0_angle);
            printf("Please input cam 1 angle:");
            scanf("%d", &cfg.camera1_angle);
            printf("Please input cam 2 angle:");
            scanf("%d", &cfg.camera2_angle);
            printf("Please input cam 3 angle:");
            scanf("%d", &cfg.camera3_angle);
            cfg.camera0_comp = CAMERA_COMP;
            cfg.camera1_comp = CAMERA_COMP;
            cfg.camera2_comp = CAMERA_COMP;
            cfg.camera3_comp = CAMERA_COMP;
            cfg.trigger_deadtime = DEADTIME;

            ret = Camera_Angle_FPS_Set(fd,cfg);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            break;
        }

        case MENU_GET_CAM_TARGET_ANGLE:
        {
            SyncCfg cfg;

            ret = Camera_Angle_FPS_Get(fd,&cfg);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            printf("fps = %d\n", cfg.fps);
            printf("camera0_angle = %d\n", cfg.camera0_angle);
            printf("camera1_angle = %d\n", cfg.camera1_angle);
            printf("camera2_angle = %d\n", cfg.camera2_angle);
            printf("camera3_angle = %d\n", cfg.camera3_angle);

            printf("camera0_comp = %d\n", cfg.camera0_comp);
            printf("camera1_comp = %d\n", cfg.camera1_comp);
            printf("camera2_comp = %d\n", cfg.camera2_comp);
            printf("camera3_comp = %d\n", cfg.camera3_comp);
            break;
        }

        case MENU_SET_FPGA_TIME:
        {
            int mode;
            TimeStampCfg timecfg;

            printf("Please select FPGA time updata mode: 0-Lidar, 1-PCIE:");
            scanf("%d", &timecfg.mode);

            if (timecfg.mode != 0)
            {
                printf("year / month / day is update by Lidar.\n");
                printf("input hour(0~23):");
                scanf("%hhd", &timecfg.ts.hour);
                printf("input min(0~59):");
                scanf("%hhd", &timecfg.ts.minute);
                printf("input sec(0~59):");
                scanf("%hhd", &timecfg.ts.second);
                printf("input ms(0~999):");
                scanf("%hd", &timecfg.ts.ms);
                printf("input us(0~999):");
                scanf("%hd", &timecfg.ts.us);
            }
            else
            {
                printf("FPGA time is updata by lidar.\n");
            }

            ret = FPGA_TimeStamp_Set(fd,timecfg);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            usleep(1);
            break;
        }

        case MENU_READ_FPGA_TIME:
        {
            TimeStamp ts;
            ret = FPGA_TimeStamp_Get(fd,&ts);
            if (ret < 0)
            {
                printf("ret = %d\n", ret);
                break;
            }
            printf("FPGA time is: %d-%d-%d  %d:%d:%d:%d:%d \n\r",
                   ts.year + 2000, ts.month, ts.day, ts.hour, ts.minute, ts.second, ts.ms, ts.us);
            break;
        }

        case MENU_QUIT:
            bQuit = 1;
            printf("Bye!\r\n");
            break;

        default:
            break;
        }
    }

    FPGA_Close(fd);
    return 0;
}
