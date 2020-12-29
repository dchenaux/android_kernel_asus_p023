/* drivers/input/misc/cm36686.c - cm36686 optical sensors driver
 *
 * Copyright (C) 2012 Capella Microsystems Inc.
 * Author: Frank Hsieh <pengyueh@gmail.com>
 *
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include "cm36686.h"
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <asm/uaccess.h>
#include <asm/setup.h>
#include <linux/file.h> 
#include <linux/mm.h> 
#include <linux/fs.h> 
#include <linux/kernel.h> 
#include "linux/cadiz.h"

/* Add by for define HW_ID */
#include <linux/HWVersion.h>
extern int Read_HW_ID(void);

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
#include <linux/suspend.h>
#endif

#define LDBG(s,args...)	{printk("[CM36686] : func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}

#define I2C_RETRY_DELAY()           usleep_range(1000, 2000)
/* wait 2ms for calibration ready */
#define WAIT_CAL_READY()            usleep_range(2000, 2500)
/* >3ms wait device ready */
#define WAIT_DEVICE_READY()         usleep_range(3000, 5000)
/* >5ms for device reset */
#define RESET_DELAY()               usleep_range(5000, 10000)
/* wait 10ms for self test  done */
#define SELF_TEST_DELAY()           usleep_range(10000, 15000)

#define I2C_RETRY_COUNT 5

#define NEAR_DELAY_TIME ((100 * HZ) / 1000)

#define CONTROL_INT_ISR_REPORT        0x00
#define CONTROL_ALS                   0x01
#define CONTROL_PS                    0x02

/* POWER SUPPLY VOLTAGE RANGE */
#define CM36686_VDD_MIN_UV	1620000
#define CM36686_VDD_MAX_UV	3600000
#define CM36686_VI2C_MIN_UV	1620000
#define CM36686_VI2C_MAX_UV	3600000

/* cm36686 polling rate in ms */
#define CM36686_LS_MIN_POLL_DELAY	1
#define CM36686_LS_MAX_POLL_DELAY	1000
#define CM36686_LS_DEFAULT_POLL_DELAY	2000

#define CM36686_PS_MIN_POLL_DELAY	1
#define CM36686_PS_MAX_POLL_DELAY	1000
#define CM36686_PS_DEFAULT_POLL_DELAY	100
#define LS_INI_PATH "/nvm_fs_partition/sensors/ls_cali.ini"
#define PS_CROS_INI_PATH "/persist/ps_cali.ini"
#define PS_L_THD_INI_PATH "/persist/ps_l_thd.ini"
#define PS_H_THD_INI_PATH "/persist/ps_h_thd.ini"
#define GS_INI_PATH "/persist/gs_cali.ini"

extern bool cadiz_support;
static int is_enable_cadiz_power = 0;
static bool already_shutdown = false;

static const int als_range[] = {
	[CM36686_ALS_IT0] = 6554,
	[CM36686_ALS_IT1] = 3277,
	[CM36686_ALS_IT2] = 1638,
	[CM36686_ALS_IT3] = 819,
};

static const int als_sense[] = {
	[CM36686_ALS_IT0] = 10,
	[CM36686_ALS_IT1] = 20,
	[CM36686_ALS_IT2] = 40,
	[CM36686_ALS_IT3] = 80,
};

static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

static int probe_success; /* Add by Tom For Check Probe successful */	

struct cm36686_info {
	struct class *cm36686_class;
	struct device *ls_dev;

	struct input_dev *ls_input_dev;

	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int intr_pin;
	int als_enable;

	uint16_t *adc_table;
	uint16_t cali_table[20];
	int irq;

	int ls_calibrate;
	
	int (*power)(int, uint8_t); /* power to the chip */

	uint32_t als_kadc;
	uint32_t als_gadc;
	uint16_t golden_adc;

	int lightsensor_opened;
	uint8_t slave_addr;

	uint32_t current_level;
	uint16_t current_adc;
    	uint16_t inte_cancel_set;


	uint16_t ls_cmd;
	uint8_t record_clear_int_fail;
	bool polling;
	atomic_t ls_poll_delay;
	struct regulator *vdd;
	struct regulator *vio;
	struct delayed_work ldwork;
	struct delayed_work pdwork;
	/* add by Tom for Load Calibration*/
	struct delayed_work cali_work;
	struct workqueue_struct *cali_wq;
	int is_read_cali;
};

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
static struct callback_data* callback_struct;
void cm36686_screen_chenged_listaner(const int state);
#endif

struct cm36686_info *lp_info;
int fLevel=-1;
static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static struct mutex CM36686_control_mutex;
static struct mutex wq_lock;
static int lightsensor_enable(struct cm36686_info *lpi);
static int lightsensor_disable(struct cm36686_info *lpi);
static int initial_cm36686(struct cm36686_info *lpi);

int32_t als_kadc;
// Add by Tom for calibration result
int LS_Cali_Result;
int PS_Cali_Result;
int Cali_ERR_Value;
static int debug;

static int control_and_report(struct cm36686_info *lpi, uint8_t mode,
		uint16_t param, int report);

/* operation for read/write calibration value */
mm_segment_t oldfs;

struct file *openFile(char *path,int flag,int mode) 
{ 
	struct file *fp; 
 
	fp=filp_open(path, O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO); 
 	return fp; 
 	 
} 
  
int readFile(struct file *fp,char *buf,int readlen) 
{ 
	if (fp->f_op && fp->f_op->read) 
		return fp->f_op->read(fp,buf,readlen, &fp->f_pos); 
	else 
		return -1; 
} 

int writeFile(struct file *fp,int value) 
{ 
	char buf[6] = {0};
	if (fp->f_op && fp->f_op->write)
	{
		sprintf(buf,"%d",value);
		return fp->f_op->write(fp,buf,sizeof(buf), &fp->f_pos); 
	}
	else
	{
		return -1; 
	}
}

int closeFile(struct file *fp) 
{ 
	filp_close(fp,NULL); 
	return 0; 
}

int read_ini_file(char *path) 
{
	int ret = 0 , value = 0;
	struct file *fp;
	char dummy[6];
	oldfs = get_fs(); 
	set_fs(KERNEL_DS); 
	fp = openFile(path, O_RDONLY, 0);
	if (!IS_ERR(fp)) 
	{ 
		memset(dummy, 0, 6); 
		if ((ret = readFile(fp, dummy, 6)) > 0)
		{ 
			value  = simple_strtol(dummy, NULL, 0);
			LDBG("File [%s] = [%d]\n", path, value); 
		}
		else
		{
			value = ret;
			LDBG("read file [%s] error %d\n", path, ret); 
		}
		closeFile(fp); 
	} 
	else
	{
		value = PTR_ERR(fp);
		LDBG("Open [%s] Fail Return [%ld]\n",path, PTR_ERR(fp));
	}
	set_fs(oldfs);
	return value;
}
int write_ini_file(char *path,int value) 
{ 
	int ret = 0;
	struct file *fp;
	oldfs = get_fs(); 
	set_fs(KERNEL_DS); 
	fp = openFile(path, O_CREAT | O_WRONLY, 0);
	if (!IS_ERR(fp)) 
	{ 
		if ((ret = writeFile(fp, value)) > 0)
		{ 
			LDBG("Write [%d] to %s \n", value, path); 
		}
		else
		{
			LDBG("Write file [%s] error %d\n", path, ret);
			ret = -1;
		}
		closeFile(fp); 
	} 
	else
	{
		ret = PTR_ERR(fp);
		LDBG("Open [%s] Fail return [%d]\n",path,ret);
	}
	set_fs(oldfs);
	if(ret < 0)
		return ret;
	else 
		return 0;
}

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	struct cm36686_info *lpi = lp_info;
	uint8_t subaddr[1];

	struct i2c_msg msgs[] = {
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_NOSTART,
		 .len = 1,
		 .buf = subaddr,
		 },
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },		 
	};

	if (already_shutdown)
		return 0;

	// Add by for Cadiz Power issue
	if (cadiz_support && is_enable_cadiz_power == 0) {
		cadiz_power_control(LIGHTSENSOR, true);
		is_enable_cadiz_power = 1;
		LDBG("Enable Cadiz Power\n");
	}

	subaddr[0] = cmd;

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {

		if (i2c_transfer(lp_info->i2c_client->adapter, msgs, 2) > 0)
			break;

		dev_err(&lpi->i2c_client->dev, "%s: I2C error(%d). Retrying.\n",
				__func__, cmd);
		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT) {
		dev_err(&lpi->i2c_client->dev, "%s: Retry count exceeds %d.",
				__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;
	struct cm36686_info *lpi = lp_info;

	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	if (already_shutdown)
		return 0;
	// Add by for Cadiz Power issue
	if (cadiz_support && is_enable_cadiz_power == 0) {
		cadiz_power_control(LIGHTSENSOR, true);
		is_enable_cadiz_power = 1;
		LDBG("Enable Cadiz Power\n");
	}

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		LDBG("%s: I2C error. Retrying...\n", __func__);
		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		dev_err(&lpi->i2c_client->dev, "%s: Retry count exceeds %d.",
				__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _cm36686_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	uint8_t buffer[2];
	int ret = 0;

	if (pdata == NULL)
	{
		LDBG("pdata [NULL] \n");
		return -EFAULT;
	}
	
	if (slaveAddr <= 0)
	{
		LDBG("slave Address [0] \n");
		return -EFAULT;
	}

	ret = I2C_RxData(slaveAddr, cmd, buffer, 2);
	if (ret < 0) {
		LDBG("%s: I2C RxData fail(%d).\n", __func__, cmd);
		return ret;
	}

	*pdata = (buffer[1]<<8)|buffer[0];

	return ret;
}

static int _cm36686_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;

	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data&0xff);
	buffer[2] = (uint8_t)((data&0xff00)>>8);	
	
	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0) {
		LDBG("%s: I2C_TxData failed.\n", __func__);
		return -EIO;
	}

	return ret;
}

static int get_ls_adc_value(uint16_t *als_step, bool resume)
{
	struct cm36686_info *lpi = lp_info;
	uint32_t tmp;
	int ret = 0;

	if (als_step == NULL)
		return -EFAULT;

	/* Read ALS data: */
	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, ALS_DATA, als_step);
	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev, "%s: I2C read word failed.\n",
				__func__);
		return -EIO;
	}

	if (!lpi->ls_calibrate) {
		tmp = (uint32_t)(*als_step) * lpi->als_gadc / lpi->als_kadc;
		if (tmp > 0xFFFF)
			*als_step = 0xFFFF;
		else
			*als_step = tmp;
	} else {
		LDBG("Raw Data [%d] \n", *als_step );
	}
	return ret;
}

static int set_lsensor_range(uint16_t low_thd, uint16_t high_thd)
{
	int ret = 0;
	struct cm36686_info *lpi = lp_info;

	_cm36686_I2C_Write_Word(lpi->slave_addr, ALS_THDH, high_thd);
	_cm36686_I2C_Write_Word(lpi->slave_addr, ALS_THDL, low_thd);

	return ret;
}



static void sensor_irq_do_work(struct work_struct *work)
{
	struct cm36686_info *lpi = lp_info;
	uint16_t intFlag;
	_cm36686_I2C_Read_Word(lpi->slave_addr, INT_FLAG, &intFlag);
	control_and_report(lpi, CONTROL_INT_ISR_REPORT, intFlag, 1);

	enable_irq(lpi->irq);
}

static int get_als_range(void)
{
	uint16_t ls_conf;
	int ret = 0;
	int index = 0;
	struct cm36686_info *lpi = lp_info;

	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, ALS_CONF, &ls_conf);
	if (ret) {
		dev_err(&lpi->i2c_client->dev, "read ALS_CONF from i2c error. %d\n",
				ret);
		return -EIO;
	}

	index = (ls_conf & 0xC0) >> 0x06;
	return  als_range[index];
}

static int get_als_sense(void)
{
	uint16_t ls_conf;
	int ret = 0;
	int index = 0;
	struct cm36686_info *lpi = lp_info;

	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, ALS_CONF, &ls_conf);
	if (ret) {
		dev_err(&lpi->i2c_client->dev, "read ALS_CONF from i2c error. %d\n",
				ret);
		return -EIO;
	}

	index = (ls_conf & 0xC0) >> 0x06;
	return  als_sense[index];
}


static void lsensor_delay_work_handler(struct work_struct *work)
{
	struct cm36686_info *lpi = lp_info;
	uint16_t adc_value = 0;
	int sense;


	mutex_lock(&wq_lock);

	get_ls_adc_value(&adc_value, 0);
	sense = get_als_sense();

	mutex_unlock(&wq_lock);

	if (sense > 0) {
		lpi->current_adc = adc_value;
		input_report_abs(lpi->ls_input_dev, ABS_MISC, adc_value/sense);
		input_sync(lpi->ls_input_dev);
	}
	schedule_delayed_work(&lpi->ldwork,
			msecs_to_jiffies(atomic_read(&lpi->ls_poll_delay)));
}

static irqreturn_t cm36686_irq_handler(int irq, void *data)
{
	struct cm36686_info *lpi = data;

	disable_irq_nosync(lpi->irq);
	queue_work(lpi->lp_wq, &sensor_irq_work);

	return IRQ_HANDLED;
}

static int als_power(int enable)
{
	struct cm36686_info *lpi = lp_info;

	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);

	return 0;
}

static void ls_initial_cmd(struct cm36686_info *lpi)
{	
	/*must disable l-sensor interrupt befrore IST create*//*disable ALS func*/
	lpi->ls_cmd &= CM36686_ALS_INT_MASK;
	lpi->ls_cmd |= CM36686_ALS_SD;
	_cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);  
}

void lightsensor_set_kvalue(struct cm36686_info *lpi)
{
	if (!lpi) {
		LDBG("%s: ls_info is empty\n", __func__);
		return;
	}

	LDBG("%s: ALS calibrated als_kadc=0x%x\n",
			__func__, als_kadc);

	if (als_kadc >> 16 == ALS_CALIBRATED)
		lpi->als_kadc = als_kadc & 0xFFFF;
	else {
		lpi->als_kadc = 0;
		LDBG("%s: no ALS calibrated\n",
				__func__);
	}

	if (lpi->als_kadc && lpi->golden_adc > 0) {
		lpi->als_kadc = (lpi->als_kadc > 0 && lpi->als_kadc < 0x1000) ?
				lpi->als_kadc : lpi->golden_adc;
		lpi->als_gadc = lpi->golden_adc;
	} else {
		lpi->als_kadc = 1;
		lpi->als_gadc = 1;
	}
	LDBG("%s: als_kadc=0x%x, als_gadc=0x%x\n",
		__func__, lpi->als_kadc, lpi->als_gadc);
}


static int ls_update_table(struct cm36686_info *lpi)
{
	uint32_t tmp_data[20];
	int i;
	for (i = 0; i < 20; i++) {
		tmp_data[i] = (uint32_t)(*(lpi->adc_table + i))
			* lpi->als_kadc / lpi->als_gadc;

		if (tmp_data[i] <= 0xFFFF)
			lpi->cali_table[i] = (uint16_t) tmp_data[i];
		else
			lpi->cali_table[i] = 0xFFFF;

		LDBG("Table[%d],%d -> %d\n", i ,lpi->adc_table[i], lpi->cali_table[i]);
	}

	return 0;
}


static int lightsensor_enable(struct cm36686_info *lpi)
{
	int ret = -EIO;
	unsigned int delay;

	mutex_lock(&als_enable_mutex);
	ret = control_and_report(lpi, CONTROL_ALS, 1, 0);
	mutex_unlock(&als_enable_mutex);

	delay = atomic_read(&lpi->ls_poll_delay);
	if (lpi->polling)
		schedule_delayed_work(&lpi->ldwork,
				msecs_to_jiffies(delay));

	return ret;
}

static int lightsensor_disable(struct cm36686_info *lpi)
{
	int ret = -EIO;
	mutex_lock(&als_disable_mutex);
	LDBG("disable lightsensor\n");

	if (lpi->polling)
		cancel_delayed_work_sync(&lpi->ldwork);

	if ( lpi->als_enable == 0 ) {
		dev_err(&lpi->i2c_client->dev, "already disabled\n");
		ret = 0;
	} else {
		ret = control_and_report(lpi, CONTROL_ALS, 0, 0);
	}
	
	mutex_unlock(&als_disable_mutex);
	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct cm36686_info *lpi = lp_info;
	int rc = 0;

	LDBG("%s\n", __func__);
	if (lpi->lightsensor_opened) {
		dev_err(&lpi->i2c_client->dev, "%s: already opened\n",
				__func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct cm36686_info *lpi = lp_info;

	LDBG("%s\n", __func__);
	lpi->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct cm36686_info *lpi = lp_info;

	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = lpi->als_enable;
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	default:
		LDBG("[LS][CM36686 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};


static ssize_t ls_cali_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	
	LDBG("LS_Cali_Result [%d]\n",LS_Cali_Result);
	switch (LS_Cali_Result) {
		case 0:
			ret = sprintf(buf, "0\n");
			LDBG("Calibration Not yet\n");
			break;
		case 1:
			ret = sprintf(buf, "1\n");
			break;
		case -1:
			ret = sprintf(buf, "0\n Enter Error Para\n");
			LDBG("Error Para\n");
			break;
		case -2:
			ret = sprintf(buf, "0\n Calibration value out of range [%d]\n", Cali_ERR_Value);
			LDBG("Calibration value out of range\n");
			break;
	};

	return ret;
}
static ssize_t ls_cali_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int i = 0 , ls_ini_value = 0;
	int tmp_max , tmp_min;
	int limit_max = 9999, limit_min = 0;
	uint32_t tmp_avg = 0;
	uint16_t raw_data = 0;	
	struct cm36686_info *lpi = lp_info;

	LDBG("===== Light Sensor Calibration Start =====\n");

	/* Scan Calibration Parameters */
	sscanf(buf, "%5d %5d", &tmp_min , &tmp_max);	
	LDBG("0. Calibration Para : limit_min [%d] , limit_max [%d] \n", limit_min , limit_max);
	if(tmp_min < 0 || tmp_max < 0) {
		LDBG("Error Para\n");
		LS_Cali_Result = -1;
		return count;
	} else {
		limit_min = tmp_min;
		limit_max = tmp_max;
	}


	/* Reset Old Calibration Value */
	LDBG("1. Reset - Old Calibration Value [%d]\n", lpi->als_kadc);
	lpi->ls_calibrate = 1;
	lpi->als_kadc = 1 ;
	lpi->als_gadc = 1 ;

	/* Calibration - Avg of 20 times */
	LDBG("2. Calibration - Avg of 20 times\n");
	lightsensor_enable(lpi);
	WAIT_DEVICE_READY();	
	for(i = 0 ; i < 21 ; i ++) {
		get_ls_adc_value(&raw_data, 0);
		/* bypass first times */
		if(i == 0) {
			continue;
		} else {
			tmp_avg += raw_data;
		}
		WAIT_DEVICE_READY();	
	}
	tmp_avg /= 20;
	LDBG("New Calibration Data Avg [%d] \n", tmp_avg);
	lpi->ls_calibrate = 0;
	lightsensor_disable(lpi);
	
	/* Check New Calibration values is normal */
	if(tmp_avg > limit_max || tmp_avg < limit_min) {
		LDBG("ERROR - New Calibration Data Out of range\n");
		LDBG("ERROR - New Calibration Data [%d] , Max Limit [%d] , Min Limit [%d]\n", tmp_avg, limit_max, limit_min);
		LS_Cali_Result = -2;
		Cali_ERR_Value = tmp_avg;
		return count;
	}
	
	/* Store Result to file */	
	LDBG("3. Store New Calibration [%d] to [%s]\n", tmp_avg , LS_INI_PATH);
	write_ini_file(LS_INI_PATH,tmp_avg);
	WAIT_CAL_READY();
	ls_ini_value = read_ini_file(LS_INI_PATH);
	LDBG("Read New Calibration [%d] from [%s]\n", ls_ini_value , LS_INI_PATH);
	if(ls_ini_value != tmp_avg)
	{
		LDBG("ini_file [%d] != store value [%d] \n", ls_ini_value , tmp_avg);
		LS_Cali_Result = -3;
	}

	/* Reset chip */
	lpi->als_kadc = tmp_avg ;
	lpi->als_gadc = 1000 ;	
	ls_update_table(lpi);
	LS_Cali_Result = 1;
	LDBG("===== Light Calibration End =====\n");
	return count;
}
static ssize_t ls_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm36686_info *lpi = lp_info;
	uint16_t adc_value = 0;
	uint16_t intFlag;
	_cm36686_I2C_Read_Word(lpi->slave_addr, INT_FLAG, &intFlag);

	get_ls_adc_value(&adc_value, 0);
	ret = sprintf(buf, "ADC[0x%04X] => level %d , ADC [%d] , intFlag [%d]\n",
		lpi->current_adc, lpi->current_level, adc_value, &intFlag);

	return ret;
}

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{

	int ret = 0;
	struct cm36686_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n",
			lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct cm36686_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
		return -EINVAL;

	/* If Probe read fail when Sensor Enable Retry again*/
	if(lpi->is_read_cali == 0)
	{	
		queue_delayed_work(lpi->cali_wq, &lpi->cali_work, 0);
	}

	if (ls_auto) {
		lpi->ls_calibrate = (ls_auto == 147) ? 1 : 0;
		ret = lightsensor_enable(lpi);
	} else {
		lpi->ls_calibrate = 0;
		ret = lightsensor_disable(lpi);
	}

	LDBG("als_enable: [0x%x] , is_read_calibration: [%d] , ls_auto: [0x%x]\n",
			lpi->als_enable, lpi->is_read_cali, ls_auto);
	if (ret < 0) {
		dev_err(&lpi->i2c_client->dev, "%s: set auto light sensor fail\n",
		__func__);
		return ret;
	}

	return count;
}


static ssize_t ls_kadc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm36686_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "kadc = 0x%x",
			lpi->als_kadc);

	return ret;
}

static ssize_t ls_kadc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm36686_info *lpi = lp_info;
	int kadc_temp = 0;

	sscanf(buf, "%d", &kadc_temp);

	mutex_lock(&als_get_adc_mutex);
	if (kadc_temp != 0) {
		lpi->als_kadc = kadc_temp;
		if (lpi->als_gadc != 0) {
			if (ls_update_table(lpi) < 0)
				dev_err(&lpi->i2c_client->dev, "%s: update ls table fail\n",
						__func__);
			else
				LDBG("%s: als_gadc =0x%x wait to be set\n",
						__func__, lpi->als_gadc);
		}
	} else {
		dev_err(&lpi->i2c_client->dev, "%s: als_kadc can't be set to zero\n",
				__func__);
	}
				
	mutex_unlock(&als_get_adc_mutex);
	return count;
}


static ssize_t ls_gadc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm36686_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "gadc = 0x%x\n", lpi->als_gadc);

	return ret;
}

static ssize_t ls_gadc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm36686_info *lpi = lp_info;
	int gadc_temp = 0;

	sscanf(buf, "%d", &gadc_temp);
	
	mutex_lock(&als_get_adc_mutex);
	if (gadc_temp != 0) {
		lpi->als_gadc = gadc_temp;
		if (lpi->als_kadc != 0) {
			if (ls_update_table(lpi) < 0)
				dev_err(&lpi->i2c_client->dev, "%s: update ls table fail\n",
						__func__);
		} else {
			LDBG("als_kadc =0x%x wait to be set\n",
					lpi->als_kadc);
		}
	} else {
		dev_err(&lpi->i2c_client->dev, "als_gadc can't be set to zero\n");
	}
	mutex_unlock(&als_get_adc_mutex);
	return count;
}


static ssize_t ls_adc_table_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned length = 0;
	int i;
	debug = 1;
	for (i = 0; i < 20; i++) {
		length += sprintf(buf + length,
				"Table[%d] =  0x%x ; %d, Cali_Table[%d] =  0x%x ; %d, \n",
				i, *(lp_info->adc_table + i),
				*(lp_info->adc_table + i),
				i, *(lp_info->cali_table + i),
				*(lp_info->cali_table + i));
	}
	return length;
}

static ssize_t ls_adc_table_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{

	struct cm36686_info *lpi = lp_info;
	char *token[10];
	uint16_t tempdata[10];
	int i;

	for (i = 0; i < 10; i++) {
		token[i] = strsep((char **)&buf, " ");
		tempdata[i] = simple_strtoul(token[i], NULL, 16);
		if (tempdata[i] < 1 || tempdata[i] > 0xffff) {
			dev_err(&lpi->i2c_client->dev,
			"adc_table[%d] =  0x%x error\n",
			i, tempdata[i]);
			return count;
		}
	}
	mutex_lock(&als_get_adc_mutex);
	for (i = 0; i < 10; i++)
		lpi->adc_table[i] = tempdata[i];

	if (ls_update_table(lpi) < 0)
		dev_err(&lpi->i2c_client->dev, "%s: update ls table fail\n",
		__func__);
	mutex_unlock(&als_get_adc_mutex);

	return count;
}

static ssize_t ls_conf_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm36686_info *lpi = lp_info;
	return sprintf(buf, "ALS_CONF = %x\n", lpi->ls_cmd);
}
static ssize_t ls_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm36686_info *lpi = lp_info;
	int value = 0;
	sscanf(buf, "0x%x", &value);

	lpi->ls_cmd = value;

	LDBG("ALS_CONF:0x%x\n", lpi->ls_cmd);

	_cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
	return count;
}

static ssize_t ls_poll_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cm36686_info *lpi = lp_info;
	return snprintf(buf, PAGE_SIZE, "%d\n",
			atomic_read(&lpi->ls_poll_delay));
}

static ssize_t ls_poll_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cm36686_info *lpi = lp_info;
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;

	if ((interval_ms < CM36686_LS_MIN_POLL_DELAY) ||
			(interval_ms > CM36686_LS_MAX_POLL_DELAY))
		return -EINVAL;

	atomic_set(&lpi->ls_poll_delay, (unsigned int) interval_ms);
	return count;
}

static ssize_t ls_fLevel_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "fLevel = %d\n", fLevel);
}
static ssize_t ls_fLevel_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm36686_info *lpi = lp_info;
	int value=0;
	sscanf(buf, "%d", &value);
	(value>=0)?(value=min(value,10)):(value=max(value,-1));
	fLevel=value;
	input_report_abs(lpi->ls_input_dev, ABS_MISC, fLevel);
	input_sync(lpi->ls_input_dev);

	msleep(1000);
	fLevel=-1;
	return count;
}

static ssize_t ls_light_status(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	uint16_t idReg = 0;
	struct cm36686_info *lpi = lp_info;

	if(probe_success == 0){
		LDBG("Probe Fail\n");
		return sprintf(buf, "0\n");
	}

	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, ID_REG, &idReg);
	
	LDBG("ID : 0x%4X \n",idReg);
	if(idReg == 390)
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");

}
static int lightsensor_setup(struct cm36686_info *lpi)
{
	int ret;
	int range;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		LDBG(
			"[LS][CM36686 error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "cm36686-ls";
	lpi->ls_input_dev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);

	range = get_als_range();
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, range, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		LDBG("[LS][CM36686 error]%s: can not register ls input device\n",
				__func__);
		goto err_free_ls_input_device;
	}

	return ret;

err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int initial_cm36686(struct cm36686_info *lpi)
{
	int val, ret;
	uint16_t idReg = 0;

	val = gpio_get_value(lpi->intr_pin);
	LDBG("INTERRUPT GPIO val = %d\n", val);

	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, ID_REG, &idReg);

	return ret;
}

static int cm36686_setup(struct cm36686_info *lpi)
{
	int ret = 0;

	// als_power(1);
	RESET_DELAY();
	
	LDBG("interrupt gpio = %d\n",lpi->intr_pin);

	ret = gpio_request(lpi->intr_pin, "gpio_cm36686_intr");
	if (ret < 0) {
		LDBG("[PS][CM36686 error]%s: gpio %d request failed (%d)\n",
			__func__, lpi->intr_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->intr_pin);
	if (ret < 0) {
		LDBG(
			"[PS][CM36686 error]%s: fail to set gpio %d as input (%d)\n",
			__func__, lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	ret = initial_cm36686(lpi);
	if (ret < 0) {
		LDBG(
			"[PS_ERR][CM36686 error]%s: fail to initial cm36686 (%d)\n",
			__func__, ret);
		goto fail_free_intr_pin;
	}
	
	/*Default disable P sensor and L sensor*/
	ls_initial_cmd(lpi);

	LDBG("lpi->polling [%d]\n",lpi->polling);
	if (!lpi->polling) {
		LDBG("request_irq(%d)\n",lpi->irq);
		ret =  request_any_context_irq(lpi->irq,
				cm36686_irq_handler,
				IRQF_TRIGGER_FALLING,
				"cm36686",
				lpi);
	}
	if (ret < 0) {
		LDBG(
			"[PS][CM36686 error]%s: req_irq(%d) fail for gpio %d (%d)\n",
			__func__, lpi->irq,
			lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	return ret;

fail_free_intr_pin:
	gpio_free(lpi->intr_pin);
	return ret;
}

static int cm36686_parse_dt(struct device *dev,
				struct cm36686_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32	levels[CM36686_LEVELS_SIZE], i;
	u32 temp_val;
	int rc;

	rc = of_get_named_gpio(np, "capella,interrupt-gpio",0);
	if (rc < 0) {
		dev_err(dev, "Unable to read interrupt pin number\n");
		return rc;
	} else {
		pdata->intr = rc;
		LDBG("pdata->intr = %d\n",pdata->intr);
	}

	rc = of_property_read_u32_array(np, "capella,levels", levels,
			CM36686_LEVELS_SIZE);
	if (rc) {
		dev_err(dev, "Unable to read levels data\n");
		return rc;
	} else {
		for (i = 0; i < CM36686_LEVELS_SIZE; i++){
			pdata->levels[i] = levels[i];
		}
	}


	rc = of_property_read_u32(np, "capella,ls_cmd", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read ls_cmd\n");
		return rc;
	} else {
		pdata->ls_cmd = (u16)temp_val;
	}


	//pdata->polling = of_property_read_bool(np, "capella,use-polling");
	pdata->polling = 0;

	return 0;
}

static int create_sysfs_interfaces(struct device *dev,
		struct device_attribute *attributes, int len)
{
	int i;
	int err;
	for (i = 0; i < len; i++) {
		err = device_create_file(dev, attributes + i);
		if (err)
			goto error;
	}
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return err;
}

static int remove_sysfs_interfaces(struct device *dev,
		struct device_attribute *attributes, int len)
{
	int i;
	for (i = 0; i < len; i++)
		device_remove_file(dev, attributes + i);
	return 0;
}
static void load_cali_for_chip(struct work_struct *work)
{
	struct cm36686_info *lpi = lp_info;
	int ls_ini_value = read_ini_file(LS_INI_PATH);
	LDBG("Load Light Calibration Value [%d] \n", ls_ini_value);
	if( ls_ini_value > 0 ) {
		/* Update Calibration Value & Update Table */
		lpi->als_kadc = ls_ini_value;
		lpi->als_gadc = 1000;
		ls_update_table(lpi);
		lpi->is_read_cali = 1;
	}
}
static struct device_attribute light_attr[] = {
	__ATTR(ls_adc, 0444, ls_adc_show, NULL),
	__ATTR(ls_kadc, 0664, ls_kadc_show, ls_kadc_store),
	__ATTR(ls_gadc, 0664, ls_gadc_show, ls_gadc_store),
	__ATTR(ls_conf, 0664, ls_conf_show, ls_conf_store),
	__ATTR(ls_adc_table, 0664,
			ls_adc_table_show, ls_adc_table_store),
	__ATTR(poll_delay, 0664, ls_poll_delay_show,
			ls_poll_delay_store),
	__ATTR(enable, 0664,
			ls_enable_show, ls_enable_store),
	__ATTR(light_status, 0444, ls_light_status, NULL),
	__ATTR(ls_flevel, 0664, ls_fLevel_show, ls_fLevel_store),
};


/* Add by Tom for sysfs in sys/bus/i2c/devices/3-0060 */
static DEVICE_ATTR(light_status, 0444,
		ls_light_status, NULL);
static DEVICE_ATTR(ls_cali, 0664,
		ls_cali_show, ls_cali_store);

static struct attribute *cm36686_attributes[] = {
	&dev_attr_light_status.attr,
	&dev_attr_ls_cali.attr,
	NULL
};

static struct attribute_group cm36686_attribute_group = {
	.attrs = cm36686_attributes
};

static int cm36686_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm36686_info *lpi;
	struct cm36686_platform_data *pdata;
	uint16_t idReg = 0;
	
	printk("===== Light Sensor Probe Start V1.0.3 =====\n");
	
	if (cadiz_support) {
		cadiz_power_control(LIGHTSENSOR, true);
		is_enable_cadiz_power = 1;
	}

	lpi = kzalloc(sizeof(struct cm36686_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	lpi->i2c_client = client;
	probe_success = 0;
	LS_Cali_Result = 0;
	debug = 0;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			LDBG("Failed to allocate memory for pdata\n");
			ret = -ENOMEM;
			goto err_platform_data_null;
		}

		ret = cm36686_parse_dt(&client->dev, pdata);
		pdata->slave_addr = client->addr;
		if (ret) {
			LDBG("Failed to get pdata from device tree\n");
			goto err_parse_dt;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			LDBG("%s: Assign platform_data error!!\n",
					__func__);
			ret = -EBUSY;
			goto err_platform_data_null;
		}
	}

	// init client->irq
	
	client->irq = irq_of_parse_and_map(client->dev.of_node,0);
	lpi->irq = client->irq;
	LDBG("lpi->irq [%d]\n",lpi->irq);

	i2c_set_clientdata(client, lpi);
	
	lpi->intr_pin = pdata->intr;
	lpi->adc_table = pdata->levels;
	lpi->power = pdata->power;
	lpi->slave_addr = pdata->slave_addr;
	lpi->polling = pdata->polling;
	atomic_set(&lpi->ls_poll_delay,
			(unsigned int) CM36686_LS_DEFAULT_POLL_DELAY);
	
	lpi->ls_cmd  = pdata->ls_cmd;
	
	lpi->record_clear_int_fail=0;
	
	LDBG("ls_cmd 0x%x\n",lpi->ls_cmd);
	
	if (pdata->ls_cmd == 0) {
		lpi->ls_cmd  = CM36686_ALS_IT_80ms | CM36686_ALS_GAIN_2;
	}

	lp_info = lpi;

	mutex_init(&CM36686_control_mutex);

	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);
	mutex_init(&als_get_adc_mutex);


	/*
	 * SET LUX STEP FACTOR HERE
	 * if adc raw value one step = 5/100 = 1/20 = 0.05 lux
	 * the following will set the factor 0.05 = 1/20
	 * and lpi->golden_adc = 1;
	 * set als_kadc = (ALS_CALIBRATED << 16) | 20;
	 */

	als_kadc = (ALS_CALIBRATED << 16) | 10;
	lpi->golden_adc = 100;
	lpi->ls_calibrate = 0;
	lpi->is_read_cali = 0;

	lightsensor_set_kvalue(lpi);
	ret = ls_update_table(lpi);
	if (ret < 0) {
		LDBG("[LS][CM36686 error]%s: update ls table fail\n",
			__func__);
		goto err_lightsensor_update_table;
	}

	lpi->lp_wq = create_singlethread_workqueue("cm36686_wq");
	if (!lpi->lp_wq) {
		LDBG("[PS][CM36686 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}
	
	
	LDBG("Create sysfs for sys/bus/i2c/devices/3-0060 !\n");
	ret = sysfs_create_group(&client->dev.kobj,
			&cm36686_attribute_group);
	if (ret < 0) {
		LDBG(
			"could not create sysfs for sys/bus/i2c/devices/3-0060\n");
	}

	LDBG("lpi->irq [%d]\n",lpi->irq);
	ret = cm36686_setup(lpi);
	if (ret < 0) {
		LDBG("[PS_ERR][CM36686 error]%s: cm36686_setup error!\n", __func__);
		goto err_lightsensor_setup;
	}
	RESET_DELAY();

	ret = lightsensor_setup(lpi);
	if (ret < 0) {
		LDBG("[LS][CM36686 error]%s: lightsensor_setup error!!\n",
			__func__);
		goto err_lightsensor_setup;
	}

	
	probe_success = 1;

	ret = create_sysfs_interfaces(&lpi->ls_input_dev->dev, light_attr,
			ARRAY_SIZE(light_attr));
	if (ret < 0) {
		LDBG("failed to create sysfs\n");
		goto err_input_cleanup;
	}


	mutex_init(&wq_lock);
	INIT_DELAYED_WORK(&lpi->ldwork, lsensor_delay_work_handler);
	/* Create loading calibration workqueue & Init delay work */
	lpi->cali_wq = create_singlethread_workqueue("sensor_cali_wq");
	if(!lpi->cali_wq)
	{	
		LDBG("create_singlethread_workqueue fail\n");
	} else {
		INIT_DELAYED_WORK(&lpi->cali_work, load_cali_for_chip);
		queue_delayed_work(lpi->cali_wq, &lpi->cali_work, 40*HZ);
	}
	LDBG("%s: Probe success!\n", __func__);

	ret = _cm36686_I2C_Read_Word(lpi->slave_addr, ID_REG, &idReg);
	LDBG("ID : 0x%4X , ret = %d\n",idReg,ret);

	int val = gpio_get_value(lpi->intr_pin);
	LDBG("INTERRUPT GPIO val = %d\n", val);

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
    	callback_struct = register_screen_state_notifier(&cm36686_screen_chenged_listaner);
#endif
	
	printk("===== Light & Proximity Probe End =====\n");
	return ret;

err_input_cleanup:
	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
err_lightsensor_setup:
	destroy_workqueue(lpi->lp_wq);
err_create_singlethread_workqueue:
err_lightsensor_update_table:
	mutex_destroy(&CM36686_control_mutex);
	mutex_destroy(&als_enable_mutex);
	mutex_destroy(&als_disable_mutex);
	mutex_destroy(&als_get_adc_mutex);
err_parse_dt:
	if (client->dev.of_node && (pdata != NULL))
		devm_kfree(&client->dev, pdata);
err_platform_data_null:
	kfree(lpi);
	LDBG("%s:error exit! ret = %d\n", __func__, ret);

	return ret;
}

static int control_and_report(struct cm36686_info *lpi, uint8_t mode,
	uint16_t param, int report)
{
	int ret = 0;
	uint16_t adc_value = 0;
	int level = 0, i;

	int val = gpio_get_value(lpi->intr_pin);
	if(mode == CONTROL_ALS) {
		LDBG("Mode[%s], ALS[%s], GPIO[%s], Report[%s], parem[0x%04X]\n",
				mode == CONTROL_ALS ? "CONTROL":"INT", 
				lpi->als_enable ? "Enable":"Disable", 
				val ? "H":"L",
				report?"Yes":"No",
				param);
		if (cadiz_support && param == 1 && is_enable_cadiz_power == 0) {
			cadiz_power_control(LIGHTSENSOR, true);
			is_enable_cadiz_power = 1;
		}
	}
	mutex_lock(&CM36686_control_mutex);

	if(mode == CONTROL_ALS) {
		if(param) {
			lpi->ls_cmd &= CM36686_ALS_SD_MASK;      
		} else {
			lpi->ls_cmd |= CM36686_ALS_SD;
		}
		_cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
		lpi->als_enable = param;
	}

	if(mode == CONTROL_ALS){  
		if( param == 1 ){
			msleep(100);  
		}
	}

	if(lpi->als_enable) {
		if( mode == CONTROL_ALS ||
				( mode == CONTROL_INT_ISR_REPORT && 
				  ((param & INT_FLAG_ALS_IF_L) || (param & INT_FLAG_ALS_IF_H)))){

			lpi->ls_cmd &= CM36686_ALS_INT_MASK;
			ret = _cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);  
			mdelay(20);
			
			get_ls_adc_value(&adc_value, 0);

			if(lpi->ls_calibrate) {
				for (i = 0 ; i < 20 ; i++) {
					if (adc_value <= (*(lpi->cali_table + i))) {
						level = i;
						if (*(lpi->cali_table + i))
							break;
					}
					if (i == 19) {/* avoid  i = 20, because 'cali_table' of size is 20 */
						level = i;
						break;
					}
				}
			} else {
				for (i = 0; i < 20; i++) {
					if (adc_value <= (*(lpi->adc_table + i))) {
						level = i;
						if (*(lpi->adc_table + i))
							break;
					}
					if ( i == 19) {/* avoid  i = 20 , because 'cali_table' of size is 20 */
						level = i;
						break;
					}
				}
			}

			if (!lpi->polling) {
				ret = set_lsensor_range(((i == 0) ||(adc_value == 0)) ? 0 :
						*(lpi->cali_table + (i - 1)) + 1,
						*(lpi->cali_table + i));

				lpi->ls_cmd |= CM36686_ALS_INT_EN;
			}

			ret = _cm36686_I2C_Write_Word(lpi->slave_addr, ALS_CONF,
					lpi->ls_cmd);

			if (report) {
				if(debug) LDBG("Light Sensor Report [%d] Level [%d], Set Threshold : H [%d] L [%d]\n",adc_value,level,
									*(lpi->adc_table + i) , *(lpi->adc_table + (i - 1)) + 1);
				lpi->current_level = level;
				lpi->current_adc = adc_value;
				input_report_abs(lpi->ls_input_dev, ABS_MISC, adc_value);
				input_sync(lpi->ls_input_dev);
			}
		}
	}

	mutex_unlock(&CM36686_control_mutex);
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int cm36686_suspend(struct device *dev)
{
	struct cm36686_info *lpi = lp_info;
	
	if (lpi->als_enable) {
		if (lightsensor_disable(lpi))
			goto out;
		lpi->als_enable = 1;
	}

	if (cadiz_support) {
		is_enable_cadiz_power = 0;
		cadiz_power_control(LIGHTSENSOR, false);
	}
	LDBG(" Light Sensor Suspend\n");
	return 0;

out:
	dev_err(&lpi->i2c_client->dev, "%s:failed during resume operation.\n",
			__func__);
	return -EIO;
}

static int cm36686_resume(struct device *dev)
{
	struct cm36686_info *lpi = lp_info;

	if (cadiz_support && is_enable_cadiz_power == 0) {
		cadiz_power_control(LIGHTSENSOR, true);
		is_enable_cadiz_power = 1;
	}
	
	if (lpi->als_enable) {
		ls_initial_cmd(lpi);
		if (lightsensor_enable(lpi))
			goto out;
	}
	LDBG(" Light Sensor Resume\n");
	return 0;

out:
	dev_err(&lpi->i2c_client->dev, "%s:failed during resume operation.\n",
			__func__);
	return -EIO;
}
#endif

//program call back
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
void cm36686_screen_chenged_listaner(const int state)
{

	if(state == NOTIFY_WHEN_SCREEN_OFF)
	{
		/* something you want to do at screen off */
		cm36686_suspend(&lp_info->i2c_client->dev);
	}
	else if(state == NOTIFY_WHEN_SCREEN_ON)
	{
		/* something you want to do at screen on*/
		cm36686_resume(&lp_info->i2c_client->dev);
	}
}
#else
static UNIVERSAL_DEV_PM_OPS(cm36686_pm, cm36686_suspend, cm36686_resume, NULL);
#endif

void cm36686_shutdown(struct i2c_client *client)
{
	if (cadiz_support) {
		is_enable_cadiz_power = 0;
		cadiz_power_control(LIGHTSENSOR, false);
	}
	already_shutdown = true;
	LDBG("shutdown\n");
}

static const struct i2c_device_id cm36686_i2c_id[] = {
	{CM36686_I2C_NAME, 0},
	{}
};

static struct of_device_id cm36686_match_table[] = {
	{ .compatible = "capella,cm36686",},
	{ },
};

static struct i2c_driver cm36686_driver = {
	.id_table = cm36686_i2c_id,
	.probe = cm36686_probe,
	.driver = {
		.name = CM36686_I2C_NAME,
		.owner = THIS_MODULE,
#ifndef CONFIG_PM_SCREEN_STATE_NOTIFIER
		.pm = &cm36686_pm,
#endif
		.of_match_table = cm36686_match_table,
	},
	.shutdown = cm36686_shutdown,
};

/* Add by Tom for SR */
static const struct i2c_device_id cm36686_i2c_id_SR[] = {
	{"lightsensor_sr", 0},
	{}
};

static struct of_device_id cm36686_match_table_SR[] = {
	{ .compatible = "capella,cm36686_SR",},
	{ },
};

static struct i2c_driver cm36686_driver_SR = {
	.id_table = cm36686_i2c_id_SR,
	.probe = cm36686_probe,
	.driver = {
		.name = CM36686_I2C_NAME,
		.owner = THIS_MODULE,
#ifndef CONFIG_PM_SCREEN_STATE_NOTIFIER
		.pm = &cm36686_pm,
#endif
		.of_match_table = cm36686_match_table_SR,
	},
	.shutdown = cm36686_shutdown,
};
static int __init cm36686_init(void)
{
	LDBG("Init! \n");
	int hw_id = Read_HW_ID();

	if (!cadiz_support) 
	{
		LDBG("Cadiz Not Support , Disable Light Senosr Function\n");
		return 0;
	}

	if (hw_id == HW_ID_SR) {
		return i2c_add_driver(&cm36686_driver_SR);
	} else if (hw_id == HW_ID_ER) {
		return i2c_add_driver(&cm36686_driver);
	} else {
		LDBG("Bypass CM36686 Light Sensor\n");
	}
}

static void __exit cm36686_exit(void)
{
	i2c_del_driver(&cm36686_driver);
}

module_init(cm36686_init);
module_exit(cm36686_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM36686 Driver");
