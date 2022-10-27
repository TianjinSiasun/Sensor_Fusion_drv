#include "lidar_module.h"

void lidar_write32(void *ipbase, unsigned int offset, unsigned int data)
{
    writel(data, ipbase + offset * 4);
}

unsigned int lidar_read32(void *ipbase, unsigned int offset)
{
    return readl(ipbase + offset * 4);
}

unsigned short swap_16(unsigned short val)
{
    return (((val & 0xff00) >> 8) | ((val & 0x00ff) << 8));
}

void lidar_set_lidar_control_reg(struct lidar_cdev *lidar, unsigned char data)
{
    lidar_write32(lidar->base_addr, LIDAR_CONTROL_REG, data);
}

unsigned int lidar_get_lidar_status(struct lidar_cdev *lidar)
{
    unsigned int lidar_status;
    lidar_status = lidar_read32(lidar->base_addr, LIDAR_STATUS_REG);
    return lidar_status;
}

void lidar_set_target_angle(struct lidar_cdev *lidar, unsigned int angle)
{
    lidar_write32(lidar->base_addr, LIDAR_TARGET_ANGLE_REG, angle);
}

unsigned int lidar_get_target_angle(struct lidar_cdev *lidar)
{
    unsigned short angle = lidar_read32(lidar->base_addr, LIDAR_TARGET_ANGLE_REG) & 0xffff;
    return angle;
}

void lidar_set_motor_speed(struct lidar_cdev *lidar, unsigned short rpm)
{
    lidar_write32(lidar->base_addr, LIDAR_MOTOR_SPEED_REG, rpm);
}

unsigned short lidar_get_motor_speed(struct lidar_cdev *lidar)
{
    unsigned short rpm;
    rpm = lidar_read32(lidar->base_addr, LIDAR_MOTOR_SPEED_REG) & 0xffff;
    rpm = swap_16(rpm);
    return rpm;
}

void lidar_set_lidar_local_ip(struct lidar_cdev *lidar, unsigned char *lidar_ip)
{
    unsigned int ipaddr;
    ipaddr = (lidar_ip[0] << 24) | (lidar_ip[1] << 16) | (lidar_ip[2] << 8) | lidar_ip[3];
    lidar_write32(lidar->base_addr, LIDAR_LIDAR_IP_REG, ipaddr);
}

void lidar_get_lidar_local_ip(struct lidar_cdev *lidar, unsigned char *lidar_ip)
{
    unsigned int ipaddr;
    ipaddr = lidar_read32(lidar->base_addr, LIDAR_LIDAR_IP_REG);
    lidar_ip[0] = (ipaddr >> 24) & 0xff;
    lidar_ip[1] = (ipaddr >> 16) & 0xff;
    lidar_ip[2] = (ipaddr >> 8) & 0xff;
    lidar_ip[3] = ipaddr & 0xff;
}

void lidar_set_lidar_dest_ip(struct lidar_cdev *lidar, unsigned char *host_ip)
{
    unsigned int ipaddr;
    ipaddr = (host_ip[0] << 24) | (host_ip[1] << 16) | (host_ip[2] << 8) | host_ip[3];
    lidar_write32(lidar->base_addr, LIDAR_DEST_IP_REG, ipaddr);
}

void lidar_get_lidar_dest_ip(struct lidar_cdev *lidar, unsigned char *host_ip)
{
    unsigned int ipaddr;
    ipaddr = lidar_read32(lidar->base_addr, LIDAR_DEST_IP_REG);
    host_ip[0] = (ipaddr >> 24) & 0xff;
    host_ip[1] = (ipaddr >> 16) & 0xff;
    host_ip[2] = (ipaddr >> 8) & 0xff;
    host_ip[3] = ipaddr & 0xff;
}

void lidar_set_lidar_mac_addr(struct lidar_cdev *lidar, unsigned char *mac_addr)
{
    unsigned int mac1_addr = 0, mac2_addr = 0;
    mac2_addr = mac_addr[0] << 8 | mac_addr[1];
    mac1_addr = mac_addr[2] << 24 | mac_addr[3] << 16 | mac_addr[4] << 8 | mac_addr[5];
    lidar_write32(lidar->base_addr, LIDAR_MAC_ADDR_REG2, mac2_addr);
    lidar_write32(lidar->base_addr, LIDAR_MAC_ADDR_REG1, mac1_addr);
}

void lidar_get_lidar_mac_addr(struct lidar_cdev *lidar, unsigned char *mac_addr)
{
    unsigned int mac1_addr, mac2_addr;
    mac1_addr = lidar_read32(lidar->base_addr, LIDAR_MAC_ADDR_REG1);
    mac2_addr = lidar_read32(lidar->base_addr, LIDAR_MAC_ADDR_REG2);
    mac_addr[0] = (mac2_addr >> 8) & 0xff;
    mac_addr[1] = mac2_addr & 0xff;
    mac_addr[2] = (mac1_addr >> 24) & 0xff;
    mac_addr[3] = (mac1_addr >> 16) & 0xff;
    mac_addr[4] = (mac1_addr >> 8) & 0xff;
    mac_addr[5] = mac1_addr & 0xff;
}

void lidar_set_msop_port(struct lidar_cdev *lidar, unsigned short port1, unsigned short port2)
{
    lidar_write32(lidar->base_addr, LIDAR_PORT1_PORT2_REG, (port1 << 16) | port2);
}

void lidar_set_difop_port(struct lidar_cdev *lidar, unsigned short port3, unsigned short port4)
{
    lidar_write32(lidar->base_addr, LIDAR_PORT3_PORT4_REG, (port3 << 16) | port4);
}

unsigned short lidar_get_msop_port1(struct lidar_cdev *lidar)
{
    unsigned short port1;
    port1 = lidar_read32(lidar->base_addr, LIDAR_PORT1_PORT2_REG) >> 16;
    port1 = swap_16(port1);
    return port1;
}

unsigned short lidar_get_msop_port2(struct lidar_cdev *lidar)
{
    unsigned short port2;
    port2 = lidar_read32(lidar->base_addr, LIDAR_PORT1_PORT2_REG) & 0xffff;
    port2 = swap_16(port2);
    return port2;
}

unsigned short lidar_get_difop_port3(struct lidar_cdev *lidar)
{
    unsigned short port3;
    port3 = lidar_read32(lidar->base_addr, LIDAR_PORT3_PORT4_REG) >> 16;
    port3 = swap_16(port3);
    return port3;
}

unsigned short lidar_get_difop_port4(struct lidar_cdev *lidar)
{
    unsigned short port4;
    port4 = lidar_read32(lidar->base_addr, LIDAR_PORT3_PORT4_REG) & 0xffff;
    port4 = swap_16(port4);
    return port4;
}

void lidar_set_ethnet(struct lidar_cdev *lidar, RSEthNet eth)
{
    lidar_set_lidar_local_ip(lidar, eth.lidar_ip);
    lidar_set_lidar_dest_ip(lidar, eth.host_ip);
    lidar_set_lidar_mac_addr(lidar, eth.mac_addr);
    lidar_set_msop_port(lidar, eth.port1, eth.port2);
    lidar_set_difop_port(lidar, eth.port3, eth.port4);
}

void lidar_get_ethnet(struct lidar_cdev *lidar, RSEthNet *eth)
{
    lidar_get_lidar_local_ip(lidar, eth->lidar_ip);
    lidar_get_lidar_dest_ip(lidar, eth->host_ip);
    lidar_get_lidar_mac_addr(lidar, eth->mac_addr);
    eth->port1 = lidar_get_msop_port1(lidar);
    eth->port2 = lidar_get_msop_port2(lidar);
    eth->port3 = lidar_get_difop_port3(lidar);
    eth->port4 = lidar_get_difop_port4(lidar);
}

void lidar_set_fov_set(struct lidar_cdev *lidar, RSFOV fov_set)
{
    unsigned int fov_val;
    fov_val = (fov_set.fov_start << 16) | (fov_set.fov_end & 0xffff);
    lidar_write32(lidar->base_addr, LIDAR_FOV_SET_REG, fov_val);
}

void lidar_get_fov_set(struct lidar_cdev *lidar, RSFOV *fov_set)
{
    unsigned int fov_val;

    fov_val = lidar_read32(lidar->base_addr, LIDAR_FOV_SET_REG);
    fov_set->fov_start = swap_16(fov_val >> 16);
    fov_set->fov_end = swap_16(fov_val & 0xffff);
}

void lidar_set_mot_phase(struct lidar_cdev *lidar, unsigned short mot_phase)
{
    lidar_write32(lidar->base_addr, LIDAR_MOT_PHASE_REG, mot_phase);
}

unsigned short lidar_get_mot_phase(struct lidar_cdev *lidar)
{
    unsigned short mot_phase;
    mot_phase = lidar_read32(lidar->base_addr, LIDAR_MOT_PHASE_REG) & 0xffff;
    mot_phase = swap_16(mot_phase);
    return mot_phase;
}

void lidar_get_top_frm(struct lidar_cdev *lidar, unsigned char *top_ver)
{
    unsigned int top_frm1, top_frm2;
    top_frm1 = lidar_read32(lidar->base_addr, LIDAR_TOP_FRM_REG1);
    top_frm2 = lidar_read32(lidar->base_addr, LIDAR_TOP_FRM_REG2);

    top_ver[0] = top_frm2 & 0xff;
    top_ver[1] = (top_frm1 >> 24) & 0xff;
    top_ver[2] = (top_frm1 >> 16) & 0xff;
    top_ver[3] = (top_frm1 >> 8) & 0xff;
    top_ver[4] = top_frm1 & 0xff;
}

void lidar_get_bot_frm(struct lidar_cdev *lidar, unsigned char *bot_ver)
{
    unsigned int bot_frm1, bot_frm2;
    bot_frm1 = lidar_read32(lidar->base_addr, LIDAR_BOT_FRM_REG1);
    bot_frm2 = lidar_read32(lidar->base_addr, LIDAR_BOT_FRM_REG2);

    bot_ver[0] = bot_frm2 & 0xff;
    bot_ver[1] = (bot_frm1 >> 24) & 0xff;
    bot_ver[2] = (bot_frm1 >> 16) & 0xff;
    bot_ver[3] = (bot_frm1 >> 8) & 0xff;
    bot_ver[4] = bot_frm1 & 0xff;
}

void lidar_get_frm_ver(struct lidar_cdev *lidar, RSVersion *version)
{
    lidar_get_top_frm(lidar, version->top_ver);
    lidar_get_bot_frm(lidar, version->bot_ver);
}

void lidar_get_cor_vert_ang(struct lidar_cdev *lidar, unsigned int ch, RSCalibrationAngle *ver_angle_cali)
{
    unsigned int data;
    data = lidar_read32(lidar->base_addr, (LIDAR_COR_VERT_ANG_REG1 - ch));
    ver_angle_cali->value[0] = (data >> 16) & 0xff;
    ver_angle_cali->value[1] = (data >> 8) & 0xff;
    ver_angle_cali->value[2] = data & 0xff;
}

void lidar_get_serial_number(struct lidar_cdev *lidar, RSSn *sn)
{
    unsigned int num1, num2;
    num1 = lidar_read32(lidar->base_addr, LIDAR_SN_REG1);
    num2 = lidar_read32(lidar->base_addr, LIDAR_SN_REG2);
    sn->num[0] = (num2 >> 8) & 0xff;
    sn->num[1] = num2 & 0xff;
    sn->num[2] = (num1 >> 24) & 0xff;
    sn->num[3] = (num1 >> 16) & 0xff;
    sn->num[4] = (num1 >> 8) & 0xff;
    sn->num[5] = num1 & 0xff;
}

unsigned short lidar_get_software_ver(struct lidar_cdev *lidar)
{
    unsigned short ver = lidar_read32(lidar->base_addr, LIDAR_SOFTWARE_VER_REG) & 0xffff;
    ver = swap_16(ver);
    return ver;
}

void lidar_set_utc_time(struct lidar_cdev *lidar, RSTimestampYMD timestamp)
{
    unsigned int utc_time_reg1, utc_time_reg2, utc_time_reg3;
    utc_time_reg3 = (((timestamp.year) & 0xff) << 8) | (timestamp.month & 0xf);
    utc_time_reg2 = ((timestamp.day & 0x1f) << 24) | ((timestamp.hour & 0x1f) << 16) | ((timestamp.minute & 0x3f) << 8) | (timestamp.second & 0x3f);
    utc_time_reg1 = ((timestamp.ms & 0x3ff) << 16) | (timestamp.us & 0x3ff);

    lidar_write32(lidar->base_addr, LIDAR_UTC_TIME_REG3, utc_time_reg3);
    lidar_write32(lidar->base_addr, LIDAR_UTC_TIME_REG2, utc_time_reg2);
    lidar_write32(lidar->base_addr, LIDAR_UTC_TIME_REG1, utc_time_reg1);
}

void lidar_get_utc_time(struct lidar_cdev *lidar, RSTimestampYMD *timestamp)
{
    unsigned int utc_time_reg1, utc_time_reg2, utc_time_reg3;

    utc_time_reg1 = lidar_read32(lidar->base_addr, LIDAR_UTC_TIME_REG1);
    utc_time_reg2 = lidar_read32(lidar->base_addr, LIDAR_UTC_TIME_REG2);
    utc_time_reg3 = lidar_read32(lidar->base_addr, LIDAR_UTC_TIME_REG3);

    timestamp->year = ((utc_time_reg3 >> 8) & 0xff);
    timestamp->month = (utc_time_reg3 >> 0) & 0xf;

    timestamp->day = (utc_time_reg2 >> 24) & 0x1f;
    timestamp->hour = (utc_time_reg2 >> 16) & 0x1f;
    timestamp->minute = (utc_time_reg2 >> 8) & 0x3f;
    timestamp->second = (utc_time_reg2 >> 0) & 0x3f;

    timestamp->ms = (utc_time_reg1 >> 16) & 0x3ff;
    timestamp->us = (utc_time_reg1 >> 0) & 0x3ff;
    timestamp->ms = swap_16(timestamp->ms);
    timestamp->us = swap_16(timestamp->us);
}

void lidar_get_status(struct lidar_cdev *lidar, RSStatus *status)
{
    unsigned int idat1, idat2;
    unsigned int vdata1, vdata2, vdata3;

    idat1 = lidar_read32(lidar->base_addr, LIDAR_STATUS_IDATA_REG1);
    idat2 = lidar_read32(lidar->base_addr, LIDAR_STATUS_IDATA_REG2);

    status->idat1_reg[0] = (idat2 >> 8) & 0xff;
    status->idat1_reg[1] = idat2 & 0xff;
    status->idat1_reg[2] = (idat1 >> 24) & 0xff;

    status->idat2_reg[0] = (idat1 >> 16) & 0xff;
    status->idat2_reg[1] = (idat1 >> 8) & 0xff;
    status->idat2_reg[2] = idat1 & 0xff;

    vdata1 = lidar_read32(lidar->base_addr, LIDAR_STATUS_VDATA_REG1);
    vdata2 = lidar_read32(lidar->base_addr, LIDAR_STATUS_VDATA_REG2);
    vdata3 = lidar_read32(lidar->base_addr, LIDAR_STATUS_VDATA_REG3);

    status->vdat_12v_reg = (vdata3 >> 16) & 0xffff;
    status->vdat_12v_m_reg = vdata3 & 0xffff;
    status->vdat_12v_reg = swap_16(status->vdat_12v_reg);
    status->vdat_12v_m_reg = swap_16(status->vdat_12v_m_reg);

    status->vdat_5v_reg = (vdata2 >> 16) & 0xffff;
    status->vdat_3v3_reg = vdata2 & 0xffff;
    status->vdat_5v_reg = swap_16(status->vdat_5v_reg);
    status->vdat_3v3_reg = swap_16(status->vdat_3v3_reg);

    status->vdat_2v5_reg = (vdata1 >> 16) & 0xffff;
    status->vdat_1v2_reg = vdata1 & 0xffff;
    status->vdat_2v5_reg = swap_16(status->vdat_2v5_reg);
    status->vdat_1v2_reg = swap_16(status->vdat_1v2_reg);
}

void lidar_get_diagno(struct lidar_cdev *lidar, RSDiagno *diagno)
{
    unsigned int data1, data2, data3, data4, data5;
    memset(diagno->reserved_1, 0, 10);
    memset(diagno->reserved_2, 0, 5);
    memset(diagno->reserved_3, 0, 7);

    data1 = lidar_read32(lidar->base_addr, LIDAR_FALT_DIGS_REG1);
    data2 = lidar_read32(lidar->base_addr, LIDAR_FALT_DIGS_REG2);
    data3 = lidar_read32(lidar->base_addr, LIDAR_FALT_DIGS_REG3);
    data4 = lidar_read32(lidar->base_addr, LIDAR_FALT_DIGS_REG4);
    data5 = lidar_read32(lidar->base_addr, LIDAR_FALT_DIGS_REG5);

    diagno->manc_err1 = (data1 >> 16) & 0xffff;
    diagno->manc_err2 = data1 & 0xffff;
    diagno->manc_err1 = swap_16(diagno->manc_err1);
    diagno->manc_err2 = swap_16(diagno->manc_err2);

    diagno->cksum_st = (data2 >> 24) & 0xff;
    diagno->gps_status = (data2 >> 16) & 0xff;
    diagno->cur_rpm1 = (data2 >> 8) & 0xff;
    diagno->cur_rpm2 = data2 & 0xff;

    diagno->temperature1 = data5 & 0xffff;
    diagno->temperature2 = (data4 >> 16) & 0xffff;
    diagno->temperature3 = data4 & 0xffff;
    diagno->temperature4 = (data3 >> 16) & 0xffff;
    diagno->temperature5 = data3 & 0xffff;
    diagno->temperature1 = swap_16(diagno->temperature1);
    diagno->temperature2 = swap_16(diagno->temperature2);
    diagno->temperature3 = swap_16(diagno->temperature3);
    diagno->temperature4 = swap_16(diagno->temperature4);
    diagno->temperature5 = swap_16(diagno->temperature5);
}

unsigned int lidar_get_echo_mode(struct lidar_cdev *lidar)
{
    unsigned int data = lidar_read32(lidar->base_addr, LIDAR_ECHO_MODE_REG); //??
    return data;
}

int lidar_init_ucwp(struct lidar_cdev *lidar)
{
    unsigned int lidar_status;
    unsigned short rpm;
    RSEthNet eth;
    RSFOV fov_set;
    RSTimestampYMD time;
    unsigned short mot_phase;
    struct timespec64 ts;
    struct tm tm;
    unsigned char lidarip[4] = {192, 168, 1, 200};
    unsigned char hostip[4] = {192, 168, 1, 102};

    rpm = 1200;
    lidar_set_motor_speed(lidar, rpm);

    lidar_get_ethnet(lidar, &eth);
    memcpy(eth.lidar_ip, lidarip, 4);
    memcpy(eth.host_ip, hostip, 4);
    eth.port1 = 6699;
    eth.port2 = 6699;
    eth.port3 = 7788;
    eth.port4 = 7788;
    lidar_set_ethnet(lidar, eth);

    fov_set.fov_start = 0;
    fov_set.fov_end = 36000;
    lidar_set_fov_set(lidar, fov_set);

    mot_phase = 0;
    lidar_set_mot_phase(lidar, mot_phase);

    ktime_get_real_ts64(&ts);
    time64_to_tm(ts.tv_sec, 8 * 60 * 60, &tm);
    time.year = tm.tm_year + 1900 - 2000;
    time.month = tm.tm_mon + 1;
    time.day = tm.tm_mday;
    time.hour = tm.tm_hour;
    time.minute = tm.tm_min;
    time.second = tm.tm_sec;
    time.ms = ts.tv_nsec / 1000 / 1000;
    time.us = ts.tv_nsec / 1000 % 1000;
    lidar_set_utc_time(lidar, time);

    lidar_status = lidar_get_lidar_status(lidar);
    if (lidar_status == 0xfff)
    {
        lidar_set_lidar_control_reg(lidar, 0);
        return 0;
    }
    else
    {
        printk(KERN_ERR "LIDAR_STATUS_REG is %#x not 0xfff.\n", lidar_status);
        return -EIO;
    }
}

void lidar_send_ucwp(struct lidar_cdev *lidar)
{
    lidar_set_lidar_control_reg(lidar, 1);
}

void lidar_set_pib_ip(struct lidar_cdev *lidar, unsigned char *pib_ip)
{
    unsigned int ipaddr;
    ipaddr = (pib_ip[0] << 24) | (pib_ip[1] << 16) | (pib_ip[2] << 8) | pib_ip[3];
    lidar_write32(lidar->mac_base, MAC_LOCAL_IP_REG, ipaddr);
}

void lidar_set_pib_msop_port(struct lidar_cdev *lidar, unsigned short port)
{
    lidar_write32(lidar->mac_base, MAC_MSOP_UDP_PORT_REG, port);
}

void lidar_set_pib_difop_port(struct lidar_cdev *lidar, unsigned short port)
{
    lidar_write32(lidar->mac_base, MAC_DIFOP_UDP_PORT_REG, port);
}

void lidar_set_pib_eth(struct lidar_cdev *lidar, PIBEthNet eth)
{
    lidar_set_pib_ip(lidar, eth.pib_ip);
    lidar_set_pib_msop_port(lidar, eth.msop_port);
    lidar_set_pib_difop_port(lidar, eth.difop_port);
}
