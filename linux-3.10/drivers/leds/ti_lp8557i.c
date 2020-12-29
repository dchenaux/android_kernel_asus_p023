#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/HWVersion.h>

static struct i2c_client *lp8557i_client;

extern int Read_HW_ID(void);
static int hw_id;

static int i2c_reg_read(struct i2c_client *client, u8 reg, u8 *value)
{
	int r;
	u8 tx_data[] = {
		reg & 0xff,
	};
	u8 rx_data[1];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		 },
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x error %d\n", __func__,
			reg, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x msgs %d\n", __func__,
			reg, r);
		return -EAGAIN;
	}

	*value = rx_data[0];

	dev_dbg(&client->dev, "%s: reg 0x%04x value 0x%08x\n", __func__,
		reg, *value);

	return 0;
}

static int i2c_reg_write(struct i2c_client *client, u8 reg, u8 value)
{
	int r;
	u8 tx_data[] = {
		reg & 0xff,
		value & 0xff,
	};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x error %d\n",
			__func__, reg, value, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
			__func__, reg, value, r);
		return -EAGAIN;
	}

	return 0;
}

//static int is_support = -1;
//int lp8557i_support(void)
//{
//	if(is_support != -1)
//		return is_support;
//
//	if(Read_HW_ID()==HW_ID_SR1) {
//		printk("[DISPLAY] %s: Not support LP85571\n", __func__);
//		is_support = 0;
//	}else{
//		printk("[DISPLAY] %s: Support LP85571\n", __func__);
//		is_support = 1;
//	}
//
//	return is_support;
//}
//EXPORT_SYMBOL(lp8557i_support);

void lp8557i_suspend(void)
{
	u8 value0=0;
	int ret0;

	i2c_reg_write(lp8557i_client, 0x00, 0x00);
	ret0 = i2c_reg_read(lp8557i_client, 0x00, &value0);
	printk("[DISPLAY] %s: 00h=0x%x(ret=%d)\n",
			__func__,
			value0, ret0);
	return;
}
EXPORT_SYMBOL(lp8557i_suspend);

void lp8557i_resume(void)
{
	u8 value0=0,value1=0,value2=0,value3=0,value4=0,value5=0,value6=0;
	u8 ret0=0,ret1=0,ret2=0,ret3=0,ret4=0,ret5=0,ret6=0;

	usleep_range(5000, 5000);

#if defined(CONFIG_Z380C)
	i2c_reg_write(lp8557i_client, 0x10, 0x84);
	i2c_reg_write(lp8557i_client, 0x11, 0x05);
	i2c_reg_write(lp8557i_client, 0x12, 0x2c);
	i2c_reg_write(lp8557i_client, 0x13, 0x03);
	i2c_reg_write(lp8557i_client, 0x15, 0xc3);
	i2c_reg_write(lp8557i_client, 0x7F, 0x21);
	i2c_reg_write(lp8557i_client, 0x7A, 0x88);
	i2c_reg_write(lp8557i_client, 0x16, 0x60);
	i2c_reg_write(lp8557i_client, 0x14, 0x9f);
	i2c_reg_write(lp8557i_client, 0x00, 0x01);
#elif defined(CONFIG_Z300C) || defined(CONFIG_Z300CG)
	i2c_reg_write(lp8557i_client, 0x10, 0x84);
	i2c_reg_write(lp8557i_client, 0x11, 0x06);
	i2c_reg_write(lp8557i_client, 0x12, 0x2c);
	i2c_reg_write(lp8557i_client, 0x13, 0x03);
	i2c_reg_write(lp8557i_client, 0x15, 0xc3);
	i2c_reg_write(lp8557i_client, 0x7F, 0x21);
	i2c_reg_write(lp8557i_client, 0x7A, 0x88);
	i2c_reg_write(lp8557i_client, 0x16, 0x60);
	i2c_reg_write(lp8557i_client, 0x14, 0x8f);
	i2c_reg_write(lp8557i_client, 0x00, 0x07);
#endif

	ret0 = i2c_reg_read(lp8557i_client, 0x10, &value0);
	ret1 = i2c_reg_read(lp8557i_client, 0x11, &value1);
	ret2 = i2c_reg_read(lp8557i_client, 0x12, &value2);
	ret3 = i2c_reg_read(lp8557i_client, 0x13, &value3);
	ret4 = i2c_reg_read(lp8557i_client, 0x14, &value4);
	ret5 = i2c_reg_read(lp8557i_client, 0x15, &value5);
	ret6 = i2c_reg_read(lp8557i_client, 0x00, &value6);

	printk("[DISPLAY] %s: 10h=0x%x(ret=%d),11h=0x%x(ret=%d),12h=0x%x(ret=%d),13h=0x%x(ret=%d),14h=0x%x(ret=%d),15h=0x%x(ret=%d),00h=0x%x(ret=%d)\n",
			__func__,
			value0, ret0,
			value1, ret1,
			value2, ret2,
			value3, ret3,
			value4, ret4,
			value5, ret5,
			value6, ret6);
	return;
}
EXPORT_SYMBOL(lp8557i_resume);

static int lp8557i_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int addr;

//	if(!lp8557i_support()) return 0;

	printk("[DISPLAY] %s: Enter\n",__func__);

	lp8557i_client = client;
	addr = client->addr;

	printk("[DISPLAY] %s: slave address=0x%x\n", __func__, addr);

	return 0;
}

static struct of_device_id lp8557i_i2c_table[] = {
	{ .compatible = "ti,lp8557i"}, //Compatible node must match dts
	{ },
};

static const struct i2c_device_id lp8557i_id[] = {
	{ "lp8557i", 0 },
	{ },
};

static struct i2c_driver lp8557i_driver = {
	.driver = {
		.name = "lp8557i",
		.owner = THIS_MODULE,
		.of_match_table = lp8557i_i2c_table,
	},
	.probe = lp8557i_probe,
	.id_table = lp8557i_id,
};

static int lp8557i_sr_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	printk("[DISPLAY] %s: Enter\n",__func__);
	lp8557i_probe(client, id);

	return 0;
}
static struct of_device_id lp8557i_sr_i2c_table[] = {
	{ .compatible = "ti,lp8557i-sr"}, //Compatible node must match dts
	{ },
};
static const struct i2c_device_id lp8557i_sr_id[] = {
	{ "lp8557i-sr", 0 },
	{ },
};
static struct i2c_driver lp8557i_sr_driver = {
	.driver = {
		.name = "lp8557i-sr",
		.owner = THIS_MODULE,
		.of_match_table = lp8557i_sr_i2c_table,
	},
	.probe = lp8557i_sr_probe,
	.id_table = lp8557i_sr_id,
};

static int __init lp8557i_I2C_init(void)
{
	int ret = 0;
	printk("[DISPLAY] %s: Enter\n",__func__);

#if defined(CONFIG_Z300C) || defined(CONFIG_Z300CG)
	hw_id = Read_HW_ID();

	if (hw_id == HW_ID_SR || hw_id == HW_ID_ER)
		ret = i2c_add_driver(&lp8557i_sr_driver);
	else
		ret = i2c_add_driver(&lp8557i_driver);

#elif defined(CONFIG_Z380C)
	ret = i2c_add_driver(&lp8557i_driver);
#endif

	return ret;
}

static void __exit lp8557i_I2C_exit(void)
{
	return;
}

module_init(lp8557i_I2C_init);
module_exit(lp8557i_I2C_exit);

MODULE_DESCRIPTION("lp8557i");
MODULE_LICENSE("GPL v2");
