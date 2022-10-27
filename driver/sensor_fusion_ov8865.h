/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SIASUN_OV8865_INIT_H__
#define __SIASUN_OV8865_INIT_H__

#include "sensor_fusion_register.h"

#define MIPI_BRIDGE_I2C_ADDR    0x1C
#define MIPI_I2C_ADDR           0x6C
#define MIPI_AF_I2C_ADDR        0x18

#define FIFO_LEVEL 16
#define DATA_FORMAT 0x0010

#define PLL_PRD     1
#define PLL_FBD     49
#define PLL_FRS     1

#define MCLK_HL    2

//2b'00: div 8, 2b'01: div 4, 2b'10: div 2
#define PPICLKDIV   1
#define MCLKREFDIV  2
#define SCLKDIV     1

#define REG_8BIT 1
#define REG_16BIT 2
#define TIME_DELAY 3
#define END_OF_SCRIPT 4

#define GMSL_RX_I2C_ADDR          0xFC
#define GMSL_TX_I2C_POWER_UP_ADDR 0x80
#define GMSL_TX_I2C_ADDR          GMSL_TX_I2C_POWER_UP_ADDR

/*Deserlizer Registers*/
#define MAX_ID 0x1E
#define MAX_REVISION 0x1F
#define MAX_CTRL_CHANNEL 0x0A
#define MAX_CONFIG_CHANNEL0 0x3B
#define MAX_CONFIG_CHANNEL1 0x3F

#define MAX_I2C_CONFIG 0x0D
#define MAX_CONFIG 0x07
#define MAX_HIM 0x06
#define MAX_PKT_CRTL 0x9A
#define MAX_REV_GAIN 0x97
#define MAX_MAIN_CONFIG 0x04
#define MAX_SER_GAIN 0x08

#define I2C_MAX_RATE 400
#define I2C_LVDS_RATE 60
#define I2C_MIPI_RATE 80

/**
 * SZ_MIPI_REG_T - config data for bridge
 * @Addr: u16 address of mipi bridge
 * @Data: u16 data to config
 */
typedef struct{
    unsigned short Addr;
    unsigned short Data;
}SZ_MIPI_REG_T;

/**
 * SZ_CONFIG_T - config data for camera
 * @Type: operation of config, contain sleep and config register
 * @Addr: u16 address of mipi bridge
 * @Data: u8 data to config
 */
typedef struct{
    unsigned char Type;
    unsigned short Addr;
    unsigned char Data;
}SZ_CONFIG_T;

int siasun_i2c_write8(struct i2c_adapter *adap, unsigned char device_addr,
                      unsigned short addr, unsigned char data);
void siasun_lvds0_init(struct siasun_video_dev *svid);
void siasun_lvds1_init(struct siasun_video_dev *svid);
void siasun_lvds2_init(struct siasun_video_dev *svid);
void siasun_lvds3_init(struct siasun_video_dev *svid);

int siasun_mipi_procedure(struct siasun_video_dev *svid);
int siasun_lvds_procedure(struct siasun_video_dev *svid);


#endif /* __SIASUN_OV8865_INIT_H__ */
