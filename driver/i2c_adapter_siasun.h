/* SPDX-License-Identifier: GPL-2.0 */
/* Linux opencores I2C Master driver header file */

#ifndef __I2C_ADAPTER_SIASUN_H__
#define __I2C_ADAPTER_SIASUN_H__

#include <linux/i2c.h>
#include "sensor_fusion_main.h"

/* all registers are 8 bits wide but on 32 bit address boundaries.*/
/* reg definitions take from i2c_specs.pdf in the docs folder */
#define LEN (4)
/* prescal   clock/(5*desired_SCL) */
#define I2C_OCORES_PRERLO (0x00 * LEN)    /*Prescale Register LOW 8 bit*/
#define I2C_OCORES_PRERHI (0x01 * LEN)    /*Prescale Register HIGH 8 bit*/

#define I2C_OCORES_CTR   (0x02 * LEN)    /*Control register*/ 
#define     I2C_OCORES_CTR_EN            (0x80)  /* Enable Core (1=Enable) */
#define     I2C_OCORES_CTR_IEN           (0x40)  /* Enable interrupt (1=Enable)*/

#define I2C_OCORES_TXR  (0x03 * LEN)    /*Transmit register (WR)*/ 
/*In case of a data transfer this bit represent the data’s LSB. 
In case of a slave address transfer this bit represents the RW bit.*/
#define I2C_OCORES_TXR_RD             (0x1)  
#define I2C_OCORES_TXR_WR             (0x0)  

#define I2C_OCORES_RXR  (0x03 * LEN)    /*Receive register (RD)*/ 

#define I2C_OCORES_CR  (0x04 * LEN)    /*Command register (WR)*/ 
#define     I2C_OCORES_CR_STA             (0x80) /*generate (repeated) start condition*/
#define     I2C_OCORES_CR_STO             (0x40) /*generate stop condition*/
#define     I2C_OCORES_CR_RD              (0x20) /*read from slave*/
#define     I2C_OCORES_CR_WR              (0x10) /*write to slave*/
#define     I2C_OCORES_CR_NACK            (0x8)  /* 0=sent ACK  1= sent NACK */
#define     I2C_OCORES_CR_IACK            (0x1)  /*Acknowledge Interrupt*/

#define I2C_OCORES_SR  (0x04 * LEN)    /*Command register (RD)*/ 
#define     I2C_OCORES_SR_RXNACK          (0x80) /*Received ack from slave (0=Received ACK)*/
#define     I2C_OCORES_SR_BUSY            (0x40) /*I2C busy*/
#define     I2C_OCORES_SR_AL              (0x20) /*Arbitration lost*/
#define     I2C_OCORES_SR_TIP             (0x2)  /*Transfer in progress(0=transfer complete)*/
#define     I2C_OCORES_SR_IF              (0x1)  /* Interrupt Flag*/


#define I2C_ACK (0)
#define I2C_NOACK (1)
#define I2C_ABITRATION_LOST (2)

struct siasun_i2c
{
    void __iomem *base;
	struct device *dev;
	struct i2c_adapter adapter;

	int pos;
	u32 bus_clk;
	u32 ip_clk; 
};

int i2c_adapter_siasun_init(struct siasun_pcie_device *siasun_pcie_dev);
void i2c_adapter_siasun_exit(struct siasun_pcie_device *siasun_pcie_dev);


#endif  /* _I2C_ADAPTER_SIASUN_H__ */
