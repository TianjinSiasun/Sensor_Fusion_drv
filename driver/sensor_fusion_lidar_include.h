/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SIASUN_HEAD_H__
#define __SIASUN_HEAD_H__

#pragma pack(push, 1)

typedef struct
{
    unsigned char year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;
    unsigned short ms;
    unsigned short us;
} RSTimestampYMD;

typedef struct
{
    unsigned char lidar_ip[4];
    unsigned char host_ip[4];
    unsigned char mac_addr[6];
    unsigned short port1;
    unsigned short port2;
    unsigned short port3;
    unsigned short port4;
} RSEthNet;

typedef struct
{
    unsigned char pib_ip[4];
    unsigned short msop_port;
    unsigned short difop_port;
} PIBEthNet;

/*FOV*/
typedef struct
{
    unsigned short fov_start;
    unsigned short fov_end;
} RSFOV;

typedef struct
{
    unsigned char top_ver[5];
    unsigned char bot_ver[5];
} RSVersion;

typedef struct
{
    unsigned char num[6];
} RSSn;

typedef struct
{
    unsigned char idat1_reg[3];
    unsigned char idat2_reg[3];
    unsigned short vdat_12v_reg;
    unsigned short vdat_12v_m_reg;
    unsigned short vdat_5v_reg;
    unsigned short vdat_3v3_reg;
    unsigned short vdat_2v5_reg;
    unsigned short vdat_1v2_reg;
} RSStatus;

typedef struct
{
    unsigned char reserved_1[10];
    unsigned char cksum_st;
    unsigned short manc_err1;
    unsigned short manc_err2;
    unsigned char gps_status;
    unsigned short temperature1;
    unsigned short temperature2;
    unsigned short temperature3;
    unsigned short temperature4;
    unsigned short temperature5;
    unsigned char reserved_2[5];
    unsigned char cur_rpm1;
    unsigned char cur_rpm2;
    unsigned char reserved_3[7];
} RSDiagno;

typedef struct
{
    unsigned char value[3];
} RSCalibrationAngle;

typedef struct
{
    unsigned long id;
    unsigned char reserved_1[12];
    RSTimestampYMD timestamp;
    unsigned char lidar_type;
    unsigned char reserved_2[11];
} RSMsopHeader;

typedef struct
{
    unsigned short distance;
    unsigned char reflectivity;
} RSChannel;

typedef struct
{
    unsigned short id;
    unsigned short azimuth;
    RSChannel channels[32];
} RS16MsopBlock;

typedef struct
{
    RSMsopHeader header;
    RS16MsopBlock blocks[12];
    unsigned int reserved;
    unsigned short tail;
} RS16MsopPkt;

typedef struct
{
    unsigned long id;
    unsigned short rpm;
    RSEthNet eth;
    RSFOV fov;
    unsigned short reserved_0;
    unsigned short phase_lock_angle;
    RSVersion version;
    unsigned char reserved_1[242];
    RSSn sn;
    unsigned short zero_cali;
    unsigned char echo_mode;
    unsigned short sw_ver;
    RSTimestampYMD timestamp;
    RSStatus status;
    unsigned char reserved_2[11];
    RSDiagno diagno;
    unsigned char gprmc[86];
    unsigned char reserved_3[697];
    RSCalibrationAngle ver_angle_cali[16];
    unsigned char reserved_4[33];
    unsigned short tail;
} RS16DifopPkt;

typedef struct
{
    unsigned long id;
    unsigned short rpm;
    RSEthNet eth;
    RSFOV fov;
    RSTimestampYMD time;
    unsigned short phase_lock_angle;
} RS16UcwpPkt;

typedef struct
{
    int frame_num;
    int frame_offset;
    int camera_offset;
    int lidar_offset;
} BufCfg;

typedef struct
{
    int fps;
    int camera0_angle;
    int camera1_angle;
    int camera2_angle;
    int camera3_angle;
    int camera0_comp;
    int camera1_comp;
    int camera2_comp;
    int camera3_comp;
    int trigger_deadtime;
} SyncCfg;

typedef struct
{
    unsigned char year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;
    unsigned short ms;
    unsigned short us;
} TimeStamp;

typedef struct
{
    int mode;
    TimeStamp ts;
} TimeStampCfg;

#pragma pack(pop)

#define MAGIC 'A'

#define UCWP_SET_MOTOR_SPEED _IOW(MAGIC, 1, unsigned short)
#define UCWP_SET_ETH _IOW(MAGIC, 2, RSEthNet)
#define UCWP_SET_FOV _IOW(MAGIC, 3, RSFOV)
#define UCWP_SET_UTC_TIME _IOW(MAGIC, 4, RSTimestampYMD)
#define UCWP_SET_MOT_PHASE _IOW(MAGIC, 5, unsigned short)
#define DIFOP_GET_PKG _IOR(MAGIC, 6, RS16DifopPkt)
#define MSOP_GET_PKG _IOR(MAGIC, 7, unsigned char)
#define MMAP_RHY_OFFSET _IOR(MAGIC, 8, unsigned int)
#define PIB_SET_ETH _IOW(MAGIC, 9, PIBEthNet)
#define LIDAR_INIT _IO(MAGIC, 10)
#define LIDAR_TARGET_ANGLE_SET _IOW(MAGIC, 11, unsigned int)
#define LIDAR_TARGET_ANGLE_GET _IOR(MAGIC, 12, unsigned int)

#define FPGA_TIME_SET _IOW(MAGIC, 101, TimeStampCfg)
#define FPGA_TIME_GET _IOR(MAGIC, 102, TimeStamp)
#define TRIGGER_MODE_SET _IOR(MAGIC, 103, unsigned char)
#define ANGLE_FPS_SET _IOW(MAGIC, 104, SyncCfg)
#define ANGLE_FPS_GET _IOR(MAGIC, 105, SyncCfg)
#define BUF_SYNS_STOP _IO(MAGIC, 106)
#define BUF_SYNS_START _IO(MAGIC, 107)
#define BUF_CONFIG _IOW(MAGIC, 108, BufCfg)

#define IRQ_MASK_CONFIG _IOW(MAGIC, 110, unsigned int)

#endif
