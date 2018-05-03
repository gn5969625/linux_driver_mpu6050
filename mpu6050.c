#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "mpu6050.h"
#include <linux/sysfs.h>

#define DRV_VERSION "V1.0"
static struct i2c_client *mpu6050_client;

static int i2c_mpu6050_read_len(struct i2c_client *client,unsigned char reg_addr,unsigned char len,unsigned char *buf)
{
	int ret;
        unsigned char txbuf = reg_addr;
        struct i2c_msg msg[] = {
                {client->addr,0,1,&txbuf},
                {client->addr,I2C_M_RD,len,buf}
        };
        ret = i2c_transfer(client->adapter,msg,2);
        if(ret < 0) {
                printk("i2c_transfer read len error\n");
                return -1;
        }
        return 0;
}
static int i2c_mpu6050_read_byte(struct i2c_client *client,unsigned char reg_addr)
{
	int ret;
	unsigned char txbuf = reg_addr;
	unsigned char rxbuf;
	struct i2c_msg msg[] = {
		{client->addr,0,1,&txbuf},
		{client->addr,I2C_M_RD,1,&rxbuf}
	};
	ret = i2c_transfer(client->adapter,msg,2);
	if(ret < 0) {
		printk("i2c_transfer read error\n");
                return -1;
	}
	return rxbuf;
}
static int i2c_mpu6050_write_byte(struct i2c_client *client,unsigned char reg_addr,unsigned char data_buf)
{
	int ret;
	unsigned char txbuf[] = {reg_addr,data_buf};
	struct i2c_msg msg[] = {client->addr,0,2,txbuf};
	ret = i2c_transfer(client->adapter,msg,1);
	if(ret < 0) {
		printk("i2c_transfer write error\n");
		return -1;
	}
	return 0;
}
static ssize_t mpu6050_outreg_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	unsigned char rxbuf_a[6],rxbuf_g[6],res_a,res_g;
	res_a = i2c_mpu6050_read_len(mpu6050_client,MPU_ACCEL_XOUTH_REG,6,rxbuf_a);
	/*
	if(res_a == 0) {
		 printk("ax = %d \n", ((u16)rxbuf[0] << 8) | rxbuf[1]);
		 printk("ay = %d \n", ((u16)rxbuf[2] << 8) | rxbuf[3]);
		 printk("az = %d \n", ((u16)rxbuf[4] << 8) | rxbuf[5]);
	}
	*/
	res_g = i2c_mpu6050_read_len(mpu6050_client,MPU_GYRO_XOUTH_REG,6,rxbuf_g);
	/*
	if(res_g == 0) {
                 printk("gx = %d \n", ((u16)rxbuf[0] << 8) | rxbuf[1]);
                 printk("gy = %d \n", ((u16)rxbuf[2] << 8) | rxbuf[3]);
                 printk("gz = %d \n", ((u16)rxbuf[4] << 8) | rxbuf[5]);
        }
	*/
	if(res_a == 0 && res_g == 0) {
	return sprintf(buf,"ax = %d ,ay = %d ,az = %d ,gx = %d ,gy = %d ,gz = %d \n"
			,((u16)rxbuf_a[0] << 8) | rxbuf_a[1]
			,((u16)rxbuf_a[2] << 8) | rxbuf_a[3]
			,((u16)rxbuf_a[4] << 8) | rxbuf_a[5]
			,((u16)rxbuf_g[0] << 8) | rxbuf_g[1]
			,((u16)rxbuf_g[2] << 8) | rxbuf_g[3]
			,((u16)rxbuf_g[4] << 8) | rxbuf_g[5]
			);
	}
	else
		return -1;
}
static ssize_t mpu6050_reg_set(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	if(count == 2)
		ret = i2c_mpu6050_write_byte(mpu6050_client,(unsigned char) buf[0],(unsigned char) buf[1]);
	return ret;
}
static DEVICE_ATTR(mpu6050_reg, 0644 , mpu6050_outreg_show, mpu6050_reg_set);
static struct attribute *mpu6050_attrs[] = {
    &dev_attr_mpu6050_reg.attr,
    NULL
};
static struct attribute_group mpu6050_attr_group = {
    .name = "mpu6050_reg",
    .attrs = mpu6050_attrs,
};

static int mpu6050_dev_init(void)
{
	char res;
	printk("%s called\n", __func__);
	i2c_mpu6050_write_byte(mpu6050_client, MPU_PWR_MGMT1_REG, 0X80);/*复位MPU6050*/
	mdelay(100);
	i2c_mpu6050_write_byte(mpu6050_client, MPU_PWR_MGMT1_REG, 0X00);
	i2c_mpu6050_write_byte(mpu6050_client, MPU_GYRO_CFG_REG, 3<<3);/*陀螺仪传感器,±2000dps*/
	i2c_mpu6050_write_byte(mpu6050_client, MPU_ACCEL_CFG_REG, 0<<3);/*加速度传感器,±2g*/
	i2c_mpu6050_write_byte(mpu6050_client, MPU_SAMPLE_RATE_REG, 1000 /50-1);/*设置采样率50Hz*/
	i2c_mpu6050_write_byte(mpu6050_client, MPU_CFG_REG, 4);/*自动设置LPF为采样率的一半*/
	i2c_mpu6050_write_byte(mpu6050_client, MPU_INT_EN_REG, 0X00);/*关闭所有中断*/
	i2c_mpu6050_write_byte(mpu6050_client, MPU_USER_CTRL_REG, 0X00);/*I2C主模式关闭*/
	i2c_mpu6050_write_byte(mpu6050_client, MPU_FIFO_EN_REG, 0X00);/*关闭FIFO*/
	i2c_mpu6050_write_byte(mpu6050_client, MPU_INTBP_CFG_REG, 0X80);/*INT引脚低电平有效*/
	res = i2c_mpu6050_read_byte(mpu6050_client, MPU_DEVICE_ID_REG);
	i2c_mpu6050_write_byte(mpu6050_client, MPU_CFG_REG, 3);//设置数字低通滤波器
	if(res == MPU_ADDR) {
		printk("I2C ID is right ! \n");
		i2c_mpu6050_write_byte(mpu6050_client, MPU_PWR_MGMT1_REG, 0X01);    /*设置CLKSEL,PLL X轴为参考*/
		i2c_mpu6050_write_byte(mpu6050_client, MPU_PWR_MGMT2_REG, 0X00);    /*加速度与陀螺仪都工作*/
        	return 0;
	}
	printk("failed !I2C ID is error ! \n");
	return -1;
}

static int mpu6050_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	//struct proc_dir_entry *file;
	int ret;
	dev_dbg(&i2c->dev, "%s\n", __func__);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C))
                return -ENODEV;

	dev_info(&i2c->dev, "chip found, driver version " DRV_VERSION "\n");
	mpu6050_client = i2c;
	mpu6050_dev_init();
	printk("mpu6050 device component found!~\n");
	ret = sysfs_create_group(&i2c->dev.kobj, &mpu6050_attr_group);
	return 0;
}
static int mpu6050_remove(struct i2c_client *i2c)
{
	sysfs_remove_group(&i2c->dev.kobj, &mpu6050_attr_group);
	return 0;
}

static const struct i2c_device_id mpu6050_id[] = {  
    { "mympu6050", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, mpu6050_id);

static struct of_device_id mpu6050_of_match[] = {
        { .compatible = "invensense,mpu6050"},
        { }
};
MODULE_DEVICE_TABLE(of, mpu6050_of_match);
struct i2c_driver mpu6050_driver = {
    .driver = {
        .name           = "mympu6050", //this decide the driver name
        .owner          = THIS_MODULE,
        .of_match_table = of_match_ptr(mpu6050_of_match),
    },
    .probe      = mpu6050_probe,
    .remove     = mpu6050_remove,
    .id_table   = mpu6050_id,
};
/*
static int mpu6050_init(void)
{
	return i2c_add_driver(&mpu6050_driver);
}

static void mpu6050_exit(void)
{
	i2c_del_driver(&mpu6050_driver);
}

module_init(mpu6050_init);
module_exit(mpu6050_exit);
*/
module_i2c_driver(mpu6050_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin.Shen");
MODULE_DESCRIPTION("A i2c-mpu6050 driver for testing module ");
MODULE_VERSION(DRV_VERSION);
