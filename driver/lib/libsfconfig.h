#ifndef __LIBSFCONFIG_H__
#define __LIBSFCONFIG_H__

#pragma pack(push, 1)

/**
 * RSTimestampYMD - The UTC_TIME struct
 * @year:  the year value is equal to the actual year minus 2000
 * @month:  actual month
 * @day:  actual day
 * @hour:  actual hour
 * @minute: actual minute
 * @second: actual second
 * @ms: actual ms
 * @us: actual us
 */
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

/**
 * RSEthNet - The Ethnet struct
 * @lidar_ip:  IP address of the lidar unit,the factory default value of the lidar is 192.168.1.200;
 * @host_ip:  IP adress of the destination PC,the factory default value of the lidar is 192.168.1.102;
 * @mac_addr: MAC address of the lidar device;
 * @port1: LiDAR output port number of the MSOP package,the default value is 6699;
 * @port2: Destination PC receiving port number of the MSOP packet,the default value is 6699;
 * @port3: LiDAR output port number of the DIFOP package,the default value is 7788;
 * @port4: Destination PC receiving port number of the DIFIOP packet,the default value is 7788;
 */
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

/**
 * RSFOV - The FOV struct
 * @fov_start: The start of the effective data horizontal Angle of the lidar output,The adjustment range is 0 to 36000;
 * @fov_end: The end of the effective data horizontal Angle of the lidar output,The adjustment range is 0 to 36000.
 */
typedef struct
{
    unsigned short fov_start;
    unsigned short fov_end;
} RSFOV;

/**
 * RSVersion - The version struct
 * @top_ver: Mainboard Firmware Version;
 * @bot_ver: Bottomboard Firmware Version;
 */
typedef struct
{
    unsigned char top_ver[5];
    unsigned char bot_ver[5];
} RSVersion;

/*RSSn - the SN struct*/
typedef struct
{
    unsigned char num[6];
} RSSn;

/**
 * RSStatus - The running status struct
 * @brief:Idat includes two currents, where Idat1_reg is the power supply current of the device
 * and Idat2_reg is the power supply current of the mainboard. Each current value consists of 3 bytes,
 * which is Idat_reg[23:0]. The highest bit Idat_reg[23] is the symbol flag bit, Idat_reg[23] = 1 is negative,
 * Idat_reg[23] = 0 is positive. Idat_reg[22:0] corresponds to the current value,
 *
 * Vdat consists of six voltage values. Each voltage value consists of 2 bytes and constitutes Vdat_reg[15:0].
 * The high 4-bit Vdat_reg[15:12] data is invalid. Vdat_reg[11:0] corresponds to the voltage value.
 */
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

/**
 * RSDiagno - the fault diagnosis struct
 *@cksum_st: the error status indicator of reading the checksum of the temperature drift compensation value
 *            of EEPROM. cksum=0x00 indicates that the temperature drift compensation value is normally available,
 *            and cksum=0x01 indicates that the temperature drift compensation value is not normal;
 *@manc_err: manc_err1 and manc_err2 are used to calculate the bit error rate of data communication transmission,
 *            manc_err1 is used to calculate the bit error rate of 1bit, and manc_err2 is used to calculate the bit error rate of 2bit.
 *@gps_status:the GPS signal input status;
 *@tempeture:temperature1 and temperature2 are temperature of the floor while temperature3 and temperature4 are temperature of the roof.
 *           The temperature value consists of 2 bytes. The lower 3 temperature[2:0] data has no meaning. The highest 13 temperature bits [15:3]
 *           are valid, and the highest temperature bit [15] is a symbol marker.
 *           temperature5 is the temperature of the floor. The temperature value consists of 2 bytes. Data higher than 4 temperature[15:12]
 *           is meaningless. The lowest 12-bit temperature[11:0] is valid, and the highest [11] is a symbol bit.
 *@cur_rpm:Real-time motor speed = (256 * r rpm1 + r rpm2)÷6.
 */
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

/*Vertical right Angle calibration*/
typedef struct
{
    unsigned char value[3];
} RSCalibrationAngle;

/**
 * RSMsopHeader - the MSOP header struct.
 * @id:the first 8 bytes is defined as 0x55, 0xaa, 0x05, 0x0a, 0x5a, 0xa5, 0x50, 0xa0 that are used for packet header detection.
 * @timestamp:MSOP packet time stamp;
 * @lidar_type:Type of Lidar
 */
typedef struct
{
    unsigned long id;
    unsigned char reserved_1[12];
    RSTimestampYMD timestamp;
    unsigned char lidar_type;
    unsigned char reserved_2[11];
} RSMsopHeader;

/**
 * RSChannel - channel data struct.
 * @distance:distance information of the lidar channel
 * @reflectivity:reflectivity information of the lidar channel
 */
typedef struct
{
    unsigned short distance;
    unsigned char reflectivity;
} RSChannel;

/**
 * RS16MsopBlock - The MSOP Data block struct.
 * @id: The fixed value is 0xffee;
 * @azimuth: Horizontal rotation Angle information of lidar channel;
 * @channels: Lidar channel data.
 */
typedef struct
{
    unsigned short id;
    unsigned short azimuth;
    RSChannel channels[32];
} RS16MsopBlock;

/**
 * RS16MsopPkt - The MSOP packet struct
 * @header: Frame header;
 * @blocks: Data block interval;
 * @tail: Frame tail is fixed to 0x00,0xff;
 */
typedef struct
{
    RSMsopHeader header;
    RS16MsopBlock blocks[12];
    unsigned int reserved;
    unsigned short tail;
} RS16MsopPkt;

/**/
/**
 * RS16DifopPkt - The DIFOP Packet struct
 * @id: frame head,is fixed to 0xA5,0xFF,0x00,0x5A,0x11,0x11,0x55,0x55;
 * @rpm: motor speed;
 * @eth: ethnet;
 * @fov: FOV set;
 * @phase_lock_angle:Motor phase lock phase;
 * @version:Version;
 * @sn:serial number;
 * @zero_cali:Zero Angle calibration value;
 * @echo_mode:the echo mode;
 * @sw_ver:Host driver compatibility information;
 * @timestamp:Time stamp of DIFOP packet;
 * @status: Operational status of LiDAR;
 * @diagno:Fault Diagnosis Information;
 * @gprmc:GPRMC value is 0 because there is no GPS module;
 * @ver_angle_cali:The value of vertical Angle calibration for each channel
 * @tail:The frame tail is equal to 0x0f0f;
 * */
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

/*UCWP Packet*/
/**
 * RS16UcwpPkt - The UCWP packet struct
 * @id:Frame head,is fixed to 0xAA,0x00,0xFF,0x11,0x22,0x22,0xAA,0xAA;
 * @rpm: Motor speed,The value can be 1200, 600, or 300;
 * @eth: Ethnet;
 * @fov: FOV set;
 * @phase_lock_angle:Motor phase lock phase;
 * @tail:The frame tail is equal to 0xAA,0x00,0xFF,0x11;
 */
typedef struct
{
    unsigned long id;
    unsigned short rpm;
    RSEthNet eth;
    RSFOV fov;
    RSTimestampYMD time;
    unsigned short phase_lock_angle;
} RS16UcwpPkt;

/*PIB ETH*/
/**
 * PIBEthNet - The PIB Ethnet set struct
 * @pib_ip: IP adress of the FPGA MAC,the pib ip must be consistent with the ip address of the radar destination  host;
 * @msop_port:receiving port number of the MSOP packet,must be consistent with the lidar port2;
 * @difop_port:receiving port number of the DIFOP packet,Must be consistent with the lidar port4.
 */
typedef struct
{
    unsigned char pib_ip[4];
    unsigned short msop_port;
    unsigned short difop_port;
} PIBEthNet;

/**
 * BufCfg - The FPGA buffer configuration struct
 * @frame_num:Number of mixed buffers used in Memory;
 * @frame_offset:Size (bytes) of each large buffers used in Memory
 * @camera_offset:Offset (bytes) of each camera, camera buffers one by one
 * @lidar_offset:Offset (bytes) of LiDA , must larger than 4(or 6)*BUF_CAMERA_OFFSETS_IN_FRMAE
 */
typedef struct
{
    int frame_num;
    int frame_offset;
    int camera_offset;
    int lidar_offset;
} BufCfg;

/**
 * SyncCfg - The FPGA SYNC configuration struct
 * @fps:The camera fps can only be set to 20, 10, or 5.
 * @camera0_angle:The mounting Angle of camera 0
 * @camera1_angle:The mounting Angle of camera 1
 * @camera2_angle:The mounting Angle of camera 2
 * @camera3_angle:The mounting Angle of camera 3
 * @camera0_comp:The timestamp compensation value of camera 0(unit:us)
 * @camera0_comp:The timestamp compensation value of camera 1(unit:us)
 * @camera0_comp:The timestamp compensation value of camera 2(unit:us)
 * @camera0_comp:The timestamp compensation value of camera 3(unit:us)
 * @trigger_deadtime:  "Camera Trigger dead time"is used to solve the trigger twice situation
 */
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

/**
 * TimeStamp - The FPGA TimeStamp struct
 * @year:  the year value is equal to the actual year minus 2000
 * @month:  actual month
 * @day:  actual day
 * @hour:  actual hour
 * @minute: actual minute
 * @second: actual second
 * @ms: actual ms
 * @us: actual us
 */
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
/**
 * TimeStampCfg - The FPGA TimeStamp Configration struct
 * mode: FPGA time update mode:0-lidar,1-by PCIE
 * ts:TimeStamp
 */
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

/**
 * @brief Open the file descriptor of FPGA_cdev
 *
 * @return fd
 */
int FPGA_Open(void);

/**
 * @brief Close the file descriptor of FPGA_cdev
 *
 * @param fd the file descriptor of FPGA_cdev
 * @return  errno
 */
int FPGA_Close(int fd);

/**
 * @brief Enable fpga BUF&SYNC management
 *
 * @param fd:the file descriptor of FPGA_cdev
 * @param enable:0-disable,1-enable
 * @return errno
 */
int Buf_Sys_Enable(int fd, unsigned char enable); // Enable BUF&SYNC management
/**
 * @brief Set BUF management,the interface is reserved
 *
 * @param fd:the file descriptor of FPGA_cdev
 * @param cfg:fpga buffer configration
 * @return errno
 */
int Buf_Config(int fd, BufCfg cfg);

/**
 * @brief Set FPGA timestamp
 *
 * @param fd:the file descriptor of FPGA_cdev
 * @param time:fpga timestamp configration
 * @return errno
 */
int FPGA_TimeStamp_Set(int fd, TimeStampCfg time);

/**
 * @brief Get FPGA timestamp
 *
 * @param fd :the file descriptor of FPGA_cdev
 * @param time :fpga timestamp
 * @return errno
 */
int FPGA_TimeStamp_Get(int fd, TimeStamp *time);

/**
 * @brief set camera trigger mode
 *
 * @param fd the file descriptor of FPGA_cdev
 * @param mode camera trigger mode, if wset 0 camera  is triggered by lidar,set 1 is triggered by itself.
 * @return errno
 */
int Trigger_Mode_Set(int fd, unsigned char mode);

/**
 * @brief Set camera fps,mounting angle,time compensation and trigger deadtime, used to SYNS management
 *
 * @param fd the file descriptor of FPGA_cdev
 * @param sync the sync managment configration of cameras
 * @return errno
 */
int Camera_Angle_FPS_Set(int fd, SyncCfg sync);

/**
 * @brief Get camera fps,mounting angle,time compensation and trigger deadtime from SYNS management
 *
 * @param fd the file descriptor of FPGA_cdev
 * @param sync the sync managment configration of cameras
 * @return errno
 */
int Camera_Angle_FPS_Get(int fd, SyncCfg *sync); // Set camera fps,mounting angle,time compensation

/**
 * @brief  Enable lidar and camera data transmission interrupt
 *
 * @param fd the file descriptor of FPGA_cdev
 * @param mask The interrupt mask: bit0-lidar, bit1-camera0, bit2-camera1, bit3-camera2, bit4-camera3,
 * @return errno
 */
int IRQ_Mask_Enable(int fd, unsigned int mask);

/**
 * @brief Open the file descriptor of lidar
 *
 * @return fd
 */
int Lidar_Open(void);

/**
 * @brief Close the file descriptor of lidar
 *
 * @param fd the file descriptor of lidar
 * @return errno
 */
int Lidar_Close(int fd);

/**
 * @brief Init lidar.if user wants to modify tje configration,the lidar must be initialized. 
 *
 * @param fd the file descriptor of lidar
 * @return errno
 */
int Lidar_Init(int fd);

/**
 * @brief Get lidar difop packet
 *
 * @param fd the file descriptor of lidar
 * @param difop DIFOP packet
 * @return errno
 */
int Lidar_Get_DIFOP(int fd, RS16DifopPkt *difop);

/**
 * @brief Set lidar fps
 *
 * @param fd the file descriptor of lidar
 * @param fps Lidar fps only can be set to 20, 10 or 5
 * @return errno
 */
int Lidar_Set_fps(int fd, unsigned int fps); 

/**
 * @brief Set lidar ETH
 *
 * @param fd the file descriptor of lidar
 * @param eth Lidar Ethernet Configuration
 * @return errno
 */
int Lidar_Set_Ethnet(int fd, RSEthNet eth); 

/**
 * @brief Set PIB ETH
 *
 * @param fd the file descriptor of lidar
 * @param  eth PIB Ethernet Configuration
 * @return errno
 */
int PIB_Set_Ethnet(int fd, PIBEthNet eth); 

/**
 * @brief Set lidar FOV
 *
 * @param fd the file descriptor of lidar
 * @param fov Lidar FOV Configuration
 * @return errno
 */
int Lidar_Set_FOV(int fd, RSFOV fov); 

/**
 * @brief Set lidar timestamp
 *
 * @param fd the file descriptor of lidar
 * @param time Lidar TimeStamp Configuration
 * @return errno
 */
int Lidar_Set_TimeStamp(int fd, RSTimestampYMD time); 

/**
 * @brief Set lidar motor phase lock angle
 *
 * @param fd the file descriptor of lidar
 * @param mot_phase Lidar motor  phase lock angle(unit:°)
 * @return errno
 */
int Lidar_Set_Motor_Phase(int fd, unsigned short mot_phase); 
/**
 * @brief Set lidar target angle
 *
 * @param fd the file descriptor of lidar
 * @param angle Target angle(unit:0.01°)
 * @return errno
 */
int Lidar_Set_Target_Angle(int fd, unsigned int angle);

/**
 * @brief Get lidar MSOP data address,used to mmap
 *
 * @param fd the file descriptor of lidar
 * @param phy_addr MSOP data address
 * @return errno
 */
int Lidar_mmap_Addr(int fd, unsigned int *phy_addr);

/**
 * @brief Get lidar MSOP frame
 *
 * @param fd the file descriptor of lidar
 * @param mem  mmap address
 * @param pkgnum  number of msop packet
 * @param msop msop frame
 * @return errno
 */
int Lidar_Get_MSOP(int fd, unsigned char *mem, unsigned char *pkgnum, RS16MsopPkt *msop); // 

#endif
