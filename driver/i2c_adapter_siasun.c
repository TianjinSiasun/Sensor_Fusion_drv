/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/types.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>

#include "i2c_adapter_siasun.h"
#include "sensor_fusion_register.h"

#define BUS_CLOCK_SET 100000

static void siasun_adapter_hw_init(struct siasun_i2c *i2c) 
{
    u32 prescale;
    DPRINTK(KERN_DEBUG "%s@%d\n", __FUNCTION__, __LINE__);
    prescale = (i2c->ip_clk / (5 * i2c->bus_clk)) - 1;

    /* turn off the core*/
    writeb(0x00, i2c->base + I2C_OCORES_CTR); 
    /* clearn any pening IRQ*/
    writeb(I2C_OCORES_CR_IACK, i2c->base + I2C_OCORES_CR); 
    /* load low presacle bit*/
    writeb((0xff & prescale), i2c->base + I2C_OCORES_PRERLO);
    /* load high presacle bit*/
    writeb((0xff & (prescale >> 8)), i2c->base + I2C_OCORES_PRERHI); 
    /* turn on the core*/
    writeb(I2C_OCORES_CTR_EN, i2c->base + I2C_OCORES_CTR); 
}

static int i2c_adapter_siasun_wait(struct siasun_i2c *i2c, unsigned long timeout)
{
    u8 status;
    unsigned long j;

    DPRINTK(KERN_DEBUG "%s@%d\n", __FUNCTION__, __LINE__);
    /* wait for the data to be transferred (8bit) */
    udelay(8 * 1000 * 1000 / i2c->bus_clk); 

    j = jiffies + timeout;
    while (1)
    {
        status = readb(i2c->base + I2C_OCORES_SR);
        if ((status & I2C_OCORES_SR_TIP) == 0)
        {
            break;
        }
        if (time_after(jiffies, j))
        {
            return -ETIMEDOUT; 
        }
    }
    return 0;
}

static int i2c_adapter_siasun_setaddr(struct siasun_i2c *i2c, struct i2c_msg *msg)
{
    int err;
    DPRINTK(KERN_DEBUG "%s@%d\n", __FUNCTION__, __LINE__);

    /* transmit the address shifted by one and the read/write bit*/
    writeb(((msg->addr << 1) | (msg->flags & I2C_M_RD ? 1 : 0)), i2c->base + I2C_OCORES_TXR);

    /* set start and write  bits which will start the transaction*/
    writeb(I2C_OCORES_CR_STA | I2C_OCORES_CR_WR, i2c->base + I2C_OCORES_CR);

    /* wait for the trnasaction to be over.*/ 
    err = i2c_adapter_siasun_wait(i2c, msecs_to_jiffies(1));
    return err;
}

static int i2c_adapter_siasun_check_ack(struct siasun_i2c *i2c) 
{
    u8 status;

    udelay(1000 * 1000 / i2c->bus_clk / 2);
    status = readb(i2c->base + I2C_OCORES_SR);
    if (status & I2C_OCORES_SR_AL)
    {
        printk("I2C_ABITRATION_LOST\n");
        return -EIO; 
    }

    if (status & I2C_OCORES_SR_RXNACK)
    {
        printk("I2C_NOACK\n");
        return -EIO; 
    }
    else
    {
        return I2C_ACK;
    }
}

static int i2c_adapter_siasun_read(struct siasun_i2c *i2c, struct i2c_msg *msg)
{
    int err, ret;
    i2c->pos = 0;

    DPRINTK(KERN_DEBUG "%s@%d\n", __FUNCTION__, __LINE__);
    while ((i2c->pos) < (msg->len))
    {
        /* start read*/
        writeb(I2C_OCORES_CR_RD | (i2c->pos == (msg->len - 1) ? I2C_OCORES_CR_NACK : 0), i2c->base + I2C_OCORES_CR);

        /* wait for the trnasaction to be over.*/
        ret = i2c_adapter_siasun_wait(i2c, msecs_to_jiffies(1));
        if (err < 0)
        {
            return err;
        }

        /* now read the data */
        msg->buf[i2c->pos++] = readb(i2c->base + I2C_OCORES_RXR);
    }
    return 0;
}

static int i2c_adapter_siasun_write(struct siasun_i2c *i2c, struct i2c_msg *msg)
{
    int err;
    i2c->pos = 0;
    while ((i2c->pos) < (msg->len))
    {
        /* transmit the data*/
        writeb(msg->buf[i2c->pos++], i2c->base + I2C_OCORES_TXR);

        /* start write*/
        writeb(I2C_OCORES_CR_WR, i2c->base + I2C_OCORES_CR);
        /* wait for the trnasaction to be over.*/
        err = i2c_adapter_siasun_wait(i2c, msecs_to_jiffies(1));
        if (err < 0)
        {
            return err;
        }
        /* now check to see if the address was acknowledged */
        err = i2c_adapter_siasun_check_ack(i2c);
        if (err < 0)
        {
            return -EIO;
        }
    }
    return 0;
}

static void i2c_adapter_siasun_stop(struct siasun_i2c *i2c)
{
    /* stop bit */
    writeb(I2C_OCORES_CR_STO, i2c->base + I2C_OCORES_CR);
}

static int siasun_i2c_xfer_core(struct siasun_i2c *i2c, struct i2c_msg *msgs, int num)
{
    int i, ret;

    for (i = 0; i < num; i++)
    {
        ret = i2c_adapter_siasun_setaddr(i2c, &msgs[i]);
        if (ret < 0)
        {
            break;
        }
        ret = i2c_adapter_siasun_check_ack(i2c);
        if (ret < 0)
        {
            break;
        }

        if (msgs[i].flags & I2C_M_RD)
        {
            ret = i2c_adapter_siasun_read(i2c, &msgs[i]);
            if (ret < 0)
            {
                break;
            }
        }
        else
        {
            ret = i2c_adapter_siasun_write(i2c, &msgs[i]);
            if (ret < 0)
            {
                break;
            }
        }
    }

    i2c_adapter_siasun_stop(i2c);

    return (ret < 0 ? ret : num);
}

static int siasun_i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
    return siasun_i2c_xfer_core(i2c_get_adapdata(adapter), msgs, num);
}

static u32 siasun_i2c_func(struct i2c_adapter *adapter)
{
    return I2C_FUNC_I2C;
}

static const struct i2c_algorithm siasun_i2c_algo =
    {
        .master_xfer = siasun_i2c_xfer,
        .functionality = siasun_i2c_func,
};

unsigned int offset[6] = {CAMERA_LVDS_0_I2C_CAMERA_CTRL_BASE, CAMERA_LVDS_1_I2C_CAMERA_CTRL_BASE,
                          CAMERA_MIPI_0_I2C_CAMERA_CTRL_BASE, CAMERA_MIPI_1_I2C_CAMERA_CTRL_BASE,
                          CAMERA_LVDS_2_I2C_CAMERA_CTRL_BASE, CAMERA_LVDS_3_I2C_CAMERA_CTRL_BASE};

int i2c_adapter_siasun_init(struct siasun_pcie_device *siasun_pcie_dev)
{
     int i;
    struct siasun_i2c *i2c;
    DPRINTK(KERN_DEBUG "%s@%d\n", __FUNCTION__, __LINE__);

    i2c = kzalloc(sizeof(struct siasun_i2c) * I2C_BUS_NUM, GFP_KERNEL);
    if (!i2c)
    {
        return -ENOMEM;
    }

    for (i = 0; i < I2C_BUS_NUM; i++)
    {
        i2c[i].base = siasun_pcie_dev->siasun_sf_dev[0]->base_addr[4] + offset[i]; 
        i2c[i].dev = &siasun_pcie_dev->siasun_sf_dev[0]->pci_dev->dev;
        i2c[i].ip_clk = 1 * 1000 * 1000; 
        if ((i < 2) || (i > 3))
        {
            i2c[i].bus_clk = 60 * 1000; 
        }
        else
        {
            i2c[i].bus_clk = 80 * 1000; 
        }

        /*Pre-scale the clk to generate the scl signal*/
        siasun_adapter_hw_init(&i2c[i]); 

        snprintf(i2c[i].adapter.name, sizeof(i2c[i].adapter.name), "siasun-i2c%x", i);
        i2c_set_adapdata(&i2c[i].adapter, &i2c[i]);
        i2c[i].adapter.owner = THIS_MODULE;
        i2c[i].adapter.algo = &siasun_i2c_algo;
        i2c[i].adapter.retries = 2;

        i2c[i].adapter.dev.parent = &siasun_pcie_dev->siasun_sf_dev[0]->pci_dev->dev; 

        i2c_add_adapter(&i2c[i].adapter);

        DPRINTK("i2c[%d].adapter.nr=%d\n",i,i2c[i].adapter.nr);
        siasun_pcie_dev->i2c_nr[i] = i2c[i].adapter.nr;

        siasun_pcie_dev->i2c_adapter[i] = &i2c[i];
    }
    return 0;
}

void i2c_adapter_siasun_exit(struct siasun_pcie_device *siasun_pcie_dev)
{
    int i = 0;
    DPRINTK(KERN_DEBUG "%s@%d\n", __FUNCTION__, __LINE__);
    for (i = 0; i < I2C_BUS_NUM ; i++)
    {
        writeb(0x00, siasun_pcie_dev->i2c_adapter[i]->base + I2C_OCORES_CTR); /* turn off i2c*/
        i2c_del_adapter(&siasun_pcie_dev->i2c_adapter[i]->adapter);
    }
}
