/*
 * This file is part of the AP3425, AP3212C and AP3216C sensor driver.
 * AP3425 is combined proximity and ambient light sensor.
 * AP3216C is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: John Huang <john.huang@dyna-image.com>
 *	    Templeton Tsai <templeton.tsai@dyna-image.com>
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: ap3425.c
 *
 * Summary:
 *	AP3425 device driver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 02/02/12 YC       1. Modify irq function to seperate two interrupt routine. 
 *					 2. Fix the index of reg array error in em write. 
 * 02/22/12 YC       3. Merge AP3425 and AP3216C into the same driver. (ver 1.8)
 * 03/01/12 YC       Add AP3212C into the driver. (ver 1.8)
 * 07/25/14 John	  Ver.2.1 , ported for Nexus 7
 * 08/21/14 Templeton AP3425 Ver 1.0, ported for Nexus 7
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/setup.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
#include <linux/suspend.h>
#endif
#include <linux/workqueue.h>
#include <linux/timer.h>
#include "ap3425.h"
#include "linux/cadiz.h"
#include <linux/file.h> 
#include <linux/mm.h> 
#include <linux/fs.h> 

/* Add by Tom for define HW_ID */
#include <linux/HWVersion.h>

/* Wait 1ms for i2x retry */
#define I2C_RETRY_DELAY()           usleep_range(1000, 2000)
/* Wait 2ms for calibration ready */
#define WAIT_CAL_READY()            usleep_range(2000, 2500)
/* >3ms wait device ready */
#define WAIT_DEVICE_READY()         usleep_range(3000, 5000)
/* >5ms for device reset */
#define RESET_DELAY()               usleep_range(5000, 10000)
/* Wait 10ms for self test done */
#define SELF_TEST_DELAY()           usleep_range(10000, 15000)

/* Calibration path */
#define LS_INI_PATH "/nvm_fs_partition/sensors/ls_cali.ini"

#define AP3425_DRV_NAME		"ap3425"
#define DRIVER_VERSION		"2"


#define PL_TIMER_DELAY 2000
#define POLLING_MODE 0

#define LSC_DBG
#ifdef LSC_DBG
#define LDBG(s,args...)	{printk(KERN_ERR"[AP3425] : func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#else
#define LDBG(s,args...) {}
#endif
static void plsensor_work_handler(struct work_struct *w);
#if POLLING_MODE
static void pl_timer_callback(unsigned long pl_data);
#endif

struct ap3425_data {
    struct i2c_client *client;
    u8 reg_cache[AP3425_NUM_CACHABLE_REGS];//TO-DO
    u8 power_state_before_suspend;
    int irq;
    struct input_dev	*lsensor_input_dev;
    struct workqueue_struct *plsensor_wq;
    struct work_struct plsensor_work;
#if POLLING_MODE
    struct timer_list pl_timer;
#endif
    /* add by Tom for Load Calibration*/
    struct delayed_work cali_work;
    struct workqueue_struct *cali_wq;
};

static struct ap3425_data *ap3425_data_g = NULL;
// AP3425 register
static u8 ap3425_reg[AP3425_NUM_CACHABLE_REGS] = 
{0x00,0x01,0x02,0x06,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
    0x10,0x14,0x1A,0x1B,0x1C,0x1D,
    0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x28,0x29,0x2A,0x2B,0x2C,0x2D, 0x30, 0x32};

// AP3425 range
static int ap3425_range[4] = {32768,8192,2048,512};

/* Add by for Interrupt mode */
static uint16_t defalut_table[23] = {0x02, 0x0f, 0x1e, 0x32, 0x64, 0xC8, 0x12C, 0x190, 0x1F4, 0x28A, 0x320, 0x3E8, 0x5DC, 0x7D0, 0xBB8, 0xFA0, 0x1388, 0x1B58, 0x2710, 0x30D4, 0x3A98, 0x445C, 0x4E20};
static uint16_t cali_table[23] = {0x02, 0x0f, 0x1e, 0x32, 0x64, 0xC8, 0x12C, 0x190, 0x1F4, 0x28A, 0x320, 0x3E8, 0x5DC, 0x7D0, 0xBB8, 0xFA0, 0x1388, 0x1B58, 0x2710, 0x30D4, 0x3A98, 0x445C, 0x4E20};
static uint32_t als_kadc = 250;
static uint32_t als_gadc = 1000;
static int is_do_cali = 0;
static int ls_cali_result = 0;
static int cali_error_value = 0;
static int DEBUG = 0;

/* Add by Tom for define HW_ID */
extern int Read_HW_ID(void);
extern int entry_mode; // add by Tom for skip COS/POS

/* Add by Tom for Cadiz Power Contorl */
extern bool cadiz_support;
static int is_enable_cadiz_power = 0;
static int is_suspend = 0;
static bool already_shutdown = false;

/* Add by Tom for file */
static mm_segment_t oldfs;

/* Add by Tom For Check Probe successful */	
static int probe_success = 0; 

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
static struct callback_data* callback_struct;
void ap3425_screen_chenged_listaner(const int state);
#endif

static u8 *reg_array;
static int *range;
static int reg_num = 0;

static int cali = 100;
static int misc_ls_opened = 0;

static DEFINE_MUTEX(ap3425_lock);
static DEFINE_MUTEX(ap3425_ls_lock);
static DEFINE_MUTEX(ap3425_cadiz_lock);
#define ADD_TO_IDX(addr,idx)	{														\
    int i;												\
    for(i = 0; i < reg_num; i++)						\
    {													\
	if (addr == reg_array[i])						\
	{												\
	    idx = i;									\
	    break;										\
	}												\
    }													\
}


/*
 * register access helpers
 */

static int __ap3425_read_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift)
{
    struct ap3425_data *data = i2c_get_clientdata(client);
    u8 idx = 0xff;

    if (already_shutdown)
	    return 0;

    // Add by for Cadiz Power issue
    if (cadiz_support && is_enable_cadiz_power == 0) {
    	LDBG("Cadiz Power is off");
	return -1;
    }

    ADD_TO_IDX(reg,idx)
	return (data->reg_cache[idx] & mask) >> shift;
}

static int __ap3425_write_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift, u8 val)
{
    struct ap3425_data *data = i2c_get_clientdata(client);
    int ret = 0;
    u8 tmp;
    u8 idx = 0xff;

    if (already_shutdown)
	    return 0;

    // Add by for Cadiz Power issue
    if (cadiz_support && is_enable_cadiz_power == 0) {
    	LDBG("Cadiz Power is off");
	return -1;
    }

    ADD_TO_IDX(reg,idx)
	if (idx >= reg_num)
	    return -EINVAL;

    tmp = data->reg_cache[idx];
    tmp &= ~mask;
    tmp |= val << shift;

    ret = i2c_smbus_write_byte_data(client, reg, tmp);
    if (!ret)
	data->reg_cache[idx] = tmp;

    return ret;
}

/* Operation for read/write calibration value */

static struct file *openFile(char *path,int flag,int mode) 
{ 
	struct file *fp; 
 
	fp=filp_open(path, O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO); 
 	return fp; 
 	 
} 
  
static int readFile(struct file *fp,char *buf,int readlen) 
{ 
	if (fp->f_op && fp->f_op->read) 
		return fp->f_op->read(fp,buf,readlen, &fp->f_pos); 
	else 
		return -1; 
} 

static int writeFile(struct file *fp,int value) 
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

static int closeFile(struct file *fp) 
{ 
	filp_close(fp,NULL); 
	return 0; 
}

static int read_ini_file(char *path) 
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
static int write_ini_file(char *path,int value) 
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

/*
 * internally used functions
 */

/* range */
static int ap3425_get_range(struct i2c_client *client)
{
    u8 idx = __ap3425_read_reg(client, AP3425_REG_ALS_CONF,
	    AP3425_ALS_RANGE_MASK, AP3425_ALS_RANGE_SHIFT); 
    return range[idx];
}

static int ap3425_set_range(struct i2c_client *client, int range)
{
    return __ap3425_write_reg(client, AP3425_REG_ALS_CONF,
	    AP3425_ALS_RANGE_MASK, AP3425_ALS_RANGE_SHIFT, range);
}


static int ap3425_set_ir_data(struct i2c_client *client, int en)
{
    int ret = 0;

    if(en == 0){
	ret = __ap3425_write_reg(client, AP3425_REG_SYS_CONF,
		AP3425_REG_SYS_CONF_MASK, AP3425_REG_SYS_CONF_SHIFT, AP3425_SYS_DEV_DOWN);
	mdelay(200);
    }

    return ret;
}
/* mode */
static int ap3425_get_mode(struct i2c_client *client)
{
    int ret;

    ret = __ap3425_read_reg(client, AP3425_REG_SYS_CONF,
	    AP3425_REG_SYS_CONF_MASK, AP3425_REG_SYS_CONF_SHIFT);
    return ret;
}

static int ap3425_set_mode(struct i2c_client *client, int mode)
{
    int ret;

    ret = __ap3425_write_reg(client, AP3425_REG_SYS_CONF,
	    AP3425_REG_SYS_CONF_MASK, AP3425_REG_SYS_CONF_SHIFT, mode);

    return ret;
}

/* ALS low threshold */
static int ap3425_get_althres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3425_read_reg(client, AP3425_REG_ALS_THDL_L,
	    AP3425_REG_ALS_THDL_L_MASK, AP3425_REG_ALS_THDL_L_SHIFT);
    msb = __ap3425_read_reg(client, AP3425_REG_ALS_THDL_H,
	    AP3425_REG_ALS_THDL_H_MASK, AP3425_REG_ALS_THDL_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3425_set_althres(struct i2c_client *client, int val)
{

    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3425_REG_ALS_THDL_L_MASK;

    err = __ap3425_write_reg(client, AP3425_REG_ALS_THDL_L,
	    AP3425_REG_ALS_THDL_L_MASK, AP3425_REG_ALS_THDL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3425_write_reg(client, AP3425_REG_ALS_THDL_H,
	    AP3425_REG_ALS_THDL_H_MASK, AP3425_REG_ALS_THDL_H_SHIFT, msb);

    return err;
}

/* ALS high threshold */
static int ap3425_get_ahthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3425_read_reg(client, AP3425_REG_ALS_THDH_L,
	    AP3425_REG_ALS_THDH_L_MASK, AP3425_REG_ALS_THDH_L_SHIFT);
    msb = __ap3425_read_reg(client, AP3425_REG_ALS_THDH_H,
	    AP3425_REG_ALS_THDH_H_MASK, AP3425_REG_ALS_THDH_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3425_set_ahthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3425_REG_ALS_THDH_L_MASK;

    err = __ap3425_write_reg(client, AP3425_REG_ALS_THDH_L,
	    AP3425_REG_ALS_THDH_L_MASK, AP3425_REG_ALS_THDH_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3425_write_reg(client, AP3425_REG_ALS_THDH_H,
	    AP3425_REG_ALS_THDH_H_MASK, AP3425_REG_ALS_THDH_H_SHIFT, msb);

    return err;
}


static int ap3425_get_adc_value(struct i2c_client *client)
{
    unsigned int lsb, msb, val ;
#ifdef LSC_DBG
    unsigned int range;
#endif
    uint32_t tmp;

    lsb = i2c_smbus_read_byte_data(client, AP3425_REG_ALS_DATA_LOW);

    if (lsb < 0) {
	return lsb;
    }

    msb = i2c_smbus_read_byte_data(client, AP3425_REG_ALS_DATA_HIGH);

    if (msb < 0)
	return msb;

    range = ap3425_get_range(client);

    val = msb << 8 | lsb;
    
    tmp = (uint32_t) val * als_gadc / als_kadc;

    if(tmp > 0xFFFF) 
    	val = 0xFFFF;
    else 
    	val = tmp;

    return val;
}



static int ap3425_get_intstat(struct i2c_client *client)
{
    int val;

    val = i2c_smbus_read_byte_data(client, AP3425_REG_SYS_INTSTATUS);
    val &= AP3425_REG_SYS_INT_MASK;

    return val >> AP3425_REG_SYS_INT_SHIFT;
}



static int ap3425_lsensor_enable(struct i2c_client *client)
{
    int ret = 0,mode;
    struct ap3425_data *data = ap3425_data_g;
    
    mode = ap3425_get_mode(client);
    if((mode & AP3425_SYS_ALS_ENABLE) == 0){
	ap3425_set_ahthres(data->client, 0);
	ap3425_set_althres(data->client, 65535);
	mode |= AP3425_SYS_ALS_ENABLE;
	ret = ap3425_set_mode(client,mode);
    }
    LDBG("Light Sensor Enable\n");

    return ret;
}

static int ap3425_lsensor_disable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3425_get_mode(client);
    if(mode & AP3425_SYS_ALS_ENABLE){
	mode &= ~AP3425_SYS_ALS_ENABLE;
	if(mode == AP3425_SYS_DEV_RESET)
	    mode = 0;
	ret = ap3425_set_mode(client,mode);
    }
    LDBG("Light Sensor Disable\n");

    return ret;
}

static int ls_update_table(void)
{
	uint32_t tmp_data[23];
	int i;
	for (i = 0; i < 23; i++) {
		tmp_data[i] = (uint32_t)(*(defalut_table + i))
			* als_kadc / als_gadc;

		if (tmp_data[i] <= 0xFFFF)
			cali_table[i] = (uint16_t) tmp_data[i];
		else
			cali_table[i] = 0xFFFF;

		LDBG("Table[%d],%d -> %d\n", i ,defalut_table[i], cali_table[i]);
	}

	return 0;
}

static void load_cali_for_chip(struct work_struct *work)
{
	int ls_ini_value = read_ini_file(LS_INI_PATH);
	LDBG("Load Light Calibration Value [%d] \n", ls_ini_value);
	if( ls_ini_value > 0 ) {
		/* Update Calibration Value & Update Table */
		als_kadc = ls_ini_value;
		als_gadc = 1000;
		ls_update_table();
		is_do_cali = 1;
	}
}

static int ap3425_register_lsensor_device(struct i2c_client *client, struct ap3425_data *data)
{
	int rc;

	LDBG("allocating input device lsensor\n");
	data->lsensor_input_dev = input_allocate_device();
	if (!data->lsensor_input_dev) {
		dev_err(&client->dev,"%s: could not allocate input device for lsensor\n", __FUNCTION__);
		rc = -ENOMEM;
		goto done;
	}
	input_set_drvdata(data->lsensor_input_dev, data);
	data->lsensor_input_dev->name = "cm36686-ls";
//	data->lsensor_input_dev->dev.parent = &client->dev;
	set_bit(EV_ABS, data->lsensor_input_dev->evbit);
	input_set_abs_params(data->lsensor_input_dev, ABS_MISC, 0, 8, 0, 0);

	rc = input_register_device(data->lsensor_input_dev);
	if (rc < 0) {
		pr_err("%s: could not register input device for lsensor\n", __FUNCTION__);
		goto done;
	}
done:
	return rc;
}


static void ap3425_unregister_lsensor_device(struct i2c_client *client, struct ap3425_data *data)
{
    input_unregister_device(data->lsensor_input_dev);
}



#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend ap3425_early_suspend;
#endif

//program call back
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
static void ap3425_early_suspend(void)
{
	LDBG("Light Sensor Suspend\n");
	if (misc_ls_opened)
		ap3425_lsensor_disable(ap3425_data_g->client);

    	mutex_lock(&ap3425_cadiz_lock);
	if (cadiz_support) {
		is_enable_cadiz_power = 0;
		is_suspend = 1 ;
		cadiz_power_control(LIGHTSENSOR, false);
	}
    	mutex_unlock(&ap3425_cadiz_lock);
}

static void ap3425_late_resume(void)
{

	LDBG(" Light Sensor Resume\n");
   	mutex_lock(&ap3425_cadiz_lock);
	if (cadiz_support && is_enable_cadiz_power == 0) {
		cadiz_power_control(LIGHTSENSOR, true);
		is_enable_cadiz_power = 1;
		is_suspend = 0;
	}
    	mutex_unlock(&ap3425_cadiz_lock);
	if (misc_ls_opened)
		ap3425_lsensor_enable(ap3425_data_g->client);
}

void ap3425_screen_chenged_listaner(const int state)
{

	if(state == NOTIFY_WHEN_SCREEN_OFF)
	{
		/* something you want to do at screen off */
		ap3425_early_suspend();
	}
	else if(state == NOTIFY_WHEN_SCREEN_ON)
	{
		/* something you want to do at screen on*/
		ap3425_late_resume();
	}
}
#endif


/* range */
static ssize_t ap3425_show_range(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3425_data *data = ap3425_data_g;
    return sprintf(buf, "%i\n", ap3425_get_range(data->client));
}

static ssize_t ap3425_store_range(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct ap3425_data *data = ap3425_data_g;
    unsigned long val;
    int ret;

    if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3))
	return -EINVAL;

    ret = ap3425_set_range(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}




static ssize_t ap3425_store_ir_data(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3425_data *data = ap3425_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3425_set_ir_data(data->client, val);

    if (ret < 0)
	return ret;
#if POLLING_MODE
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}

/* mode */
static ssize_t ap3425_show_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3425_data *data = ap3425_data_g;
    DEBUG = 1;
    return sprintf(buf, "%d\n", ap3425_get_mode(data->client));
}

static ssize_t ap3425_store_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3425_data *data = ap3425_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3425_set_mode(data->client, val);

    if (ret < 0)
	return ret;
#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}

static ssize_t ap3425_show_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "1\n");
}

static ssize_t ap3425_store_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    // LDBG(" Do Nothing !\n");
    return count;
}

static ssize_t ap3425_ls_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3425_data *data = ap3425_data_g;
    unsigned long mode;
    int ret;

    LDBG("Enter Mode [%s] From AP \n", buf);

    if (strict_strtoul(buf, 10, &mode) < 0)
	return -EINVAL;

    if(mode != 0 && mode !=1)
    	return -EINVAL;

    /* Add by Tom for Update Calibration */
    if(is_do_cali == 0) {
	    queue_delayed_work(data->cali_wq, &data->cali_work, 0);
    }

    // Add by for Cadiz Power issue
    mutex_lock(&ap3425_cadiz_lock);
    if (cadiz_support && is_enable_cadiz_power == 0) {
	    cadiz_power_control(LIGHTSENSOR, true);
	    is_enable_cadiz_power = 1;
	    is_suspend = 1;
	    LDBG("Enable Cadiz Power\n");
    }

    mutex_lock(&ap3425_ls_lock);
    if((mode == AP3425_SYS_ALS_ENABLE) && ap3425_get_mode(data->client) != AP3425_SYS_ALS_ENABLE) {
	misc_ls_opened = 1;
	ap3425_set_ahthres(data->client, 0);
	ap3425_set_althres(data->client, 65535);
	ret = __ap3425_write_reg(data->client, AP3425_REG_SYS_CONF,
		AP3425_REG_SYS_CONF_MASK, AP3425_REG_SYS_CONF_SHIFT, AP3425_SYS_ALS_ENABLE);
	if (ret < 0)
	    return ret;
    } else {
	misc_ls_opened = 0;
	ret = __ap3425_write_reg(data->client, AP3425_REG_SYS_CONF,
		AP3425_REG_SYS_CONF_MASK, AP3425_REG_SYS_CONF_SHIFT, AP3425_SYS_DEV_RESET);
	ret = __ap3425_write_reg(data->client, AP3425_REG_SYS_CONF,
		AP3425_REG_SYS_CONF_MASK, AP3425_REG_SYS_CONF_SHIFT, AP3425_SYS_DEV_DOWN);
    }
    LDBG("LS_Enable [%d] , IS_CALI [%d]\n", misc_ls_opened, is_do_cali);
    mutex_unlock(&ap3425_ls_lock);

    // Add by for Cadiz Power issue
    if (cadiz_support && is_enable_cadiz_power == 1 && is_suspend == 1 && mode != AP3425_SYS_ALS_ENABLE) {
	    is_enable_cadiz_power = 0;
	    is_suspend = 1;
	    cadiz_power_control(LIGHTSENSOR, false);
	    LDBG("Disable Cadiz Power\n");
    }
    mutex_unlock(&ap3425_cadiz_lock);

#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}

/* lux */
static ssize_t ap3425_show_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3425_data *data = ap3425_data_g;

    /* No LUX data if power down */
    if (ap3425_get_mode(data->client) == AP3425_SYS_DEV_DOWN)
	return sprintf((char*) buf, "%s\n", "Please power up first!");

    return sprintf(buf, "%d\n", ap3425_get_adc_value(data->client));
}

/* ALS low threshold */
static ssize_t ap3425_show_althres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3425_data *data = ap3425_data_g;
    return sprintf(buf, "%d\n", ap3425_get_althres(data->client));
}

static ssize_t ap3425_store_althres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3425_data *data = ap3425_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3425_set_althres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}



/* ALS high threshold */
static ssize_t ap3425_show_ahthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3425_data *data = ap3425_data_g;
    return sprintf(buf, "%d\n", ap3425_get_ahthres(data->client));
}

static ssize_t ap3425_store_ahthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3425_data *data = ap3425_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3425_set_ahthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}


/* calibration */
static ssize_t ap3425_show_calibration_state(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    return sprintf(buf, "%d\n", cali);
}

static ssize_t ap3425_store_calibration_state(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct ap3425_data *data = ap3425_data_g;
    int stdls, lux; 
    char tmp[10];

    LDBG("DEBUG ap3425_store_calibration_state..\n");

    /* No LUX data if not operational */
    if (ap3425_get_mode(data->client) == AP3425_SYS_DEV_DOWN)
    {
	printk("Please power up first!");
	return -EINVAL;
    }

    cali = 100;
    sscanf(buf, "%d %s", &stdls, tmp);

    if (!strncmp(tmp, "-setcv", 6))
    {
	cali = stdls;
	return -EBUSY;
    }

    if (stdls < 0)
    {
	printk("Std light source: [%d] < 0 !!!\nCheck again, please.\n\
		Set calibration factor to 100.\n", stdls);
	return -EBUSY;
    }

    lux = ap3425_get_adc_value(data->client);
    cali = stdls * 100 / lux;

    return -EBUSY;
}


#ifdef LSC_DBG
/* engineer mode */
static ssize_t ap3425_em_read(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct ap3425_data *data = ap3425_data_g;
    int i;
    u8 tmp;

    LDBG("DEBUG ap3425_em_read..\n");

    for (i = 0; i < reg_num; i++)
    {
	tmp = i2c_smbus_read_byte_data(data->client, reg_array[i]);

	LDBG("Reg[0x%x] Val[0x%x]\n", reg_array[i], tmp);
    }

    return 0;
}

static ssize_t ap3425_em_write(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct ap3425_data *data = ap3425_data_g;
    u32 addr,val,idx=0;
    int ret = 0;

    LDBG("DEBUG ap3425_em_write..\n");

    sscanf(buf, "%x%x", &addr, &val);

    printk("Write [%x] to Reg[%x]...\n",val,addr);

    ret = i2c_smbus_write_byte_data(data->client, addr, val);
    ADD_TO_IDX(addr,idx)
	if (!ret)
	    data->reg_cache[idx] = val;

    return count;
}
#endif

static ssize_t ls_light_status(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct ap3425_data *data = ap3425_data_g;
	int i;
	u8 tmp;
	
	if(probe_success == 0){
		LDBG("Probe Fail\n");
		return sprintf(buf, "0\n");
	}

	LDBG("DEBUG ap3425_em_read..\n");
	for (i = 0; i < reg_num; i++)
	{
		tmp = i2c_smbus_read_byte_data(data->client, reg_array[i]);
		LDBG("Reg[0x%x] Val[0x%x]\n", reg_array[i], tmp);
		if(tmp < 0) return sprintf(buf, "0\n");
	}
	return sprintf(buf, "1\n");

}

static ssize_t ls_cali_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	
	LDBG("ls_cali_result [%d] , Value [%d]\n", ls_cali_result, als_kadc);
	switch (ls_cali_result) {
		case 0:
			ret = sprintf(buf, "0\n Calibration Not yet\n");
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
			ret = sprintf(buf, "0\n Calibration value out of range [%d]\n", cali_error_value);
			LDBG("Calibration value out of range\n");
			break;
		case -3:
			ret = sprintf(buf, "0\n Back Up Fail\n");
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
	int limit_max = 99999, limit_min = 0;
	uint32_t tmp_avg = 0;
	uint16_t raw_data = 0;	
	struct ap3425_data *data = ap3425_data_g;

	LDBG("===== Light Sensor Calibration Start =====\n");

	if(probe_success == 0){
		LDBG("Probe Fail\n");
		return sprintf(buf, "0\n");
	}

	/* Scan Calibration Parameters */
	sscanf(buf, "%5d %5d", &tmp_min , &tmp_max);	
	LDBG("0. Calibration Para : limit_min [%d] , limit_max [%d] \n", limit_min , limit_max);
	if(tmp_min < 0 || tmp_max < 0) {
		LDBG("Error Para\n");
		ls_cali_result = -1;
		return count;
	} else {
		limit_min = tmp_min;
		limit_max = tmp_max;
	}


	/* Reset Old Calibration Value */
	LDBG("1. Reset - Old Calibration Value [%d]\n", als_kadc);
	als_kadc = 1000;
	als_gadc = 1000;

	/* Calibration - Avg of 20 times */
	LDBG("2. Calibration - Avg of 20 times\n");
	ap3425_lsensor_enable(data->client);
	WAIT_DEVICE_READY();	
	for(i = 0 ; i < 21 ; i ++) {
		raw_data = ap3425_get_adc_value(data->client);
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
	ap3425_lsensor_disable(data->client);
	
	/* Check New Calibration values is normal */
	if(tmp_avg > limit_max || tmp_avg < limit_min) {
		LDBG("ERROR - New Calibration Data Out of range\n");
		LDBG("ERROR - New Calibration Data [%d] , Max Limit [%d] , Min Limit [%d]\n", tmp_avg, limit_max, limit_min);
		ls_cali_result = -2;
		cali_error_value = tmp_avg;
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
		ls_cali_result = -3;
	}

	/* Reset chip */
	als_kadc = tmp_avg ;
	als_gadc = 1000 ;	
	ls_update_table();
	ls_cali_result = 1;
	LDBG("===== Light Calibration End =====\n");
	return count;
}
static struct device_attribute attributes[] = {
    __ATTR(range, 0664 , ap3425_show_range, ap3425_store_range),
    __ATTR(lsensor, 0664, ap3425_show_mode, ap3425_store_mode),
    __ATTR(enable, 0664, ap3425_show_mode, ap3425_ls_enable),
    __ATTR(poll_delay, 0664, ap3425_show_delay, ap3425_store_delay),
    __ATTR(lux, 0444 , ap3425_show_lux, NULL),
    __ATTR(althres, 0664 , ap3425_show_althres, ap3425_store_althres),
    __ATTR(ahthres, 0664 , ap3425_show_ahthres, ap3425_store_ahthres),
    __ATTR(calibration, 0664 , ap3425_show_calibration_state, ap3425_store_calibration_state),
    __ATTR(ir_data, 0664, ap3425_show_lux, ap3425_store_ir_data),
#ifdef LSC_DBG
    __ATTR(em, 0664 , ap3425_em_read, ap3425_em_write),
#endif
    __ATTR(light_status, 0444, ls_light_status, NULL),

};

/* Add by Tom for sysfs in sys/bus/i2c/devices */
static DEVICE_ATTR(light_status, 0444,
		ls_light_status, NULL);
static DEVICE_ATTR(ls_cali, 0664,
		ls_cali_show, ls_cali_store);

static struct attribute *ap3425_attributes[] = {
	&dev_attr_light_status.attr,
	&dev_attr_ls_cali.attr,
	NULL
};

static struct attribute_group ap3425_attribute_group = {
	.attrs = ap3425_attributes
};


static int create_sysfs_interfaces(struct ap3425_data *sensor)
{
    int i , j;
    struct class *ap3425_class = NULL;
    struct device *ap3425_dev = NULL;
    int ret;

    ap3425_class = class_create(THIS_MODULE, "sensors");
    if (IS_ERR(ap3425_class)) {
	ret = PTR_ERR(ap3425_class);
	ap3425_class = NULL;
	LDBG("%s: could not allocate ap3425_class, ret = %d\n", __func__, ret);
	goto ap3425_class_error;
    }

    ap3425_dev= device_create(ap3425_class,
	    NULL, 0, "%s", "di_sensors");

    if(ap3425_dev == NULL)
	goto ap3425_device_error;
    for (i = 0; i < ARRAY_SIZE(attributes); i++)
	if (device_create_file(ap3425_dev, attributes + i))
	    goto ap3425_create_file_error;

    /* Add by Tom for input folder */
    for (j = 0; j < ARRAY_SIZE(attributes); j++)
	if (device_create_file(&sensor->lsensor_input_dev->dev, attributes + j))
	    goto ap3425_create_input_file_error;

    return 0;

ap3425_create_input_file_error:
    for ( ; j >= 0; j--)
	device_remove_file(&sensor->lsensor_input_dev->dev, attributes + i);

ap3425_create_file_error:
    for ( ; i >= 0; i--)
	device_remove_file(ap3425_dev, attributes + i);

ap3425_device_error:
    class_destroy(ap3425_class);
ap3425_class_error:
    dev_err(&sensor->client->dev, "%s:Unable to create interface\n", __func__);
    return -1;
}
static int ap3425_init_client(struct i2c_client *client)
{
	struct ap3425_data *data = i2c_get_clientdata(client);
	int i;

	LDBG("AP3425_init_client, slave_address [0x%2X] ..\n", client->addr);

	/* read all the registers once to fill the cache.
	 * if one of the reads fails, we consider the init failed */
	for (i = 0; i < reg_num; i++) {
		int v = i2c_smbus_read_byte_data(client, reg_array[i]);
		if (v < 0) {
			LDBG("Reg[0x%X] Val[%d]   <---- ERROR\n", reg_array[i], v);
			return -ENODEV;
		}
		LDBG("Reg[0x%X] Val[0x%X]\n", reg_array[i], v);
		data->reg_cache[i] = v;
	}
	/* set defaults */

	ap3425_set_range(client, AP3425_ALS_RANGE_0);
	ap3425_set_mode(data->client, AP3425_SYS_DEV_DOWN);

	return 0;
}

#if POLLING_MODE
static void pl_timer_callback(unsigned long pl_data)
{
    struct ap3425_data *data;
    int ret =0;

    data = ap3425_data_g;
    queue_work(data->plsensor_wq, &data->plsensor_work);

    ret = mod_timer(&ap3425_data_g->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");

}
#endif
static void plsensor_work_handler(struct work_struct *w)
{

    struct ap3425_data *data =
	container_of(w, struct ap3425_data, plsensor_work);
    u8 int_stat;
    int ret;
    int value;
    int i;

    int_stat = ap3425_get_intstat(data->client);

    // ALS int
    if (int_stat & AP3425_REG_SYS_INT_AMASK)
    {
	value = ap3425_get_adc_value(data->client);
	
	// Reset Threshold 
	for(i = 0 ; i < 23 ; i++) {
		if(value <= *(defalut_table + i)) {
			ap3425_set_ahthres(data->client, *(cali_table + i));
			ap3425_set_althres(data->client, (i ==0) ? 0 : (*(cali_table + (i-1)) + 1));
			break;
		}
	}

	ret = __ap3425_write_reg(data->client, AP3425_REG_SYS_INTSTATUS,
		AP3425_REG_SYS_INT_AMASK, AP3425_REG_SYS_INT_LS_SHIFT, 0);
	input_report_abs(data->lsensor_input_dev, ABS_MISC, value);
	input_sync(data->lsensor_input_dev);
	if(DEBUG) LDBG("Value [%d] , Set H_THD [%d], L_THD [%d]\n",  value, *(defalut_table + i) ,  (i ==0) ? 0 : *(defalut_table + (i-1)));
    }

    enable_irq(data->client->irq);
}
/*
 * I2C layer
 */

static irqreturn_t ap3425_irq(int irq, void *data_)
{
    struct ap3425_data *data = data_;

    disable_irq_nosync(data->client->irq);
    queue_work(data->plsensor_wq, &data->plsensor_work);

    return IRQ_HANDLED;
}

static int ap3425_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct ap3425_data *data;
    int err = 0;
    u8 int_stat = 0;

    printk("===== Light Sensor Probe Start  =====\n");

    if (cadiz_support) {
	    cadiz_power_control(LIGHTSENSOR, true);
	    is_enable_cadiz_power = 1;
    }

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)){
	err = -EIO;
	goto exit_free_gpio;
    }

    reg_array = ap3425_reg;
    range = ap3425_range;
    reg_num = AP3425_NUM_CACHABLE_REGS;

    data = kzalloc(sizeof(struct ap3425_data), GFP_KERNEL);
    if (!data){
	err = -ENOMEM;
	goto exit_free_gpio;
    }

    data->client = client;
    i2c_set_clientdata(client, data);
    data->irq = client->irq;

    /* Add by Tom for sysfs */
    err = sysfs_create_group(&client->dev.kobj, &ap3425_attribute_group);
    if (err < 0) {
	    LDBG("could not create sysfs for sys/bus/i2c/devices/\n");
    }
    
    /* initialize the AP3425 chip */
    err = ap3425_init_client(client);
    if (err)
	goto exit_kfree;

    err = ap3425_register_lsensor_device(client,data);
    if (err){
	dev_err(&client->dev, "failed to register_lsensor_device\n");
	goto exit_kfree;
    }
    
    err = create_sysfs_interfaces(data);
    if (err)
	goto exit_free_ls_device;

#ifdef CONFIG_HAS_EARLYSUSPEND
    ap3425_early_suspend.suspend = ap3425_suspend;
    ap3425_early_suspend.resume  = ap3425_resume;
    ap3425_early_suspend.level   = 0x02;
    register_early_suspend(&ap3425_early_suspend);
#endif
 
    /* reset Interrupt pin */
    int_stat = ap3425_get_intstat(data->client);
    LDBG("Interrupt Status [0x%2X]\n", int_stat);

    data->plsensor_wq = create_singlethread_workqueue("light_sensor_wq");
    if (!data->plsensor_wq) {
	LDBG("%s: create workqueue failed\n", __func__);
	err = -ENOMEM;
	goto err_create_wq_failed;
    }

    INIT_WORK(&data->plsensor_work, plsensor_work_handler);

    err = request_any_context_irq(client->irq,
    	    ap3425_irq, IRQF_TRIGGER_FALLING,
	    "ap3425", data);
    if (err) {
	dev_err(&client->dev, "ret: %d, could not get IRQ %d\n",err,client->irq);
	goto exit_free_ls_device;
    }

#if POLLING_MODE
    LDBG("Timer module installing\n");
    setup_timer(&data->pl_timer, pl_timer_callback, 0);
#endif


    ap3425_data_g = data;
    
    /* Add by Tom for Loading Calibration Data */
    data->cali_wq = create_singlethread_workqueue("sensor_cali_wq");
    if(!data->cali_wq)
    {	
	    LDBG("create_singlethread_workqueue fail\n");
    } else {
	    INIT_DELAYED_WORK(&data->cali_work, load_cali_for_chip);
	    queue_delayed_work(data->cali_wq, &data->cali_work, 40*HZ);
    }
    LDBG("Driver version %s enabled\n", DRIVER_VERSION);
    probe_success = 1;

    ls_update_table();
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
    	callback_struct = register_screen_state_notifier(&ap3425_screen_chenged_listaner);
#endif
    printk("===== Light Probe End =====\n");

    return 0;

exit_free_ls_device:
    ap3425_unregister_lsensor_device(client,data);

err_create_wq_failed:
#if POLLING_MODE
    if(&data->pl_timer != NULL)
	del_timer(&data->pl_timer);
#endif
    if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);

exit_kfree:
    kfree(data);

exit_free_gpio:
    printk("===== Light Probe Fail =====\n");
    return err;
}

void ap3425_shutdown(struct i2c_client *client)
{
	if (cadiz_support) {
		is_enable_cadiz_power = 0;
		cadiz_power_control(LIGHTSENSOR, false);
	}
	already_shutdown = true;
	LDBG("shutdown\n");
}

static const struct i2c_device_id ap3425_id[] = {
	{ AP3425_DRV_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ap3425_id);


static struct of_device_id ap3425_match_table[] = {
	{ .compatible = "dyna,ap3425",},
	{ },
};

static struct i2c_driver ap3425_driver = {
    .driver = {
	.name	= AP3425_DRV_NAME,
	.owner	= THIS_MODULE,
	.of_match_table = ap3425_match_table,
    },
    .probe	= ap3425_probe,
    .id_table = ap3425_id,
    .shutdown = ap3425_shutdown,
};

static int __init ap3425_init(void)
{
	int ret;

	LDBG("ap3425_init\n");
	int hw_id = Read_HW_ID();

	if (!cadiz_support) {
		LDBG("Cadiz Not Support , Disable Light Senosr Function\n");
		return 0;
	}

	// add by Tom for skip COS/POS ++
	if(entry_mode != 1 ) {
		LDBG("Not in MOS, skip ! entry_mode [%d]\n", entry_mode);
		return 0;
	}

	if (hw_id == HW_ID_SR || hw_id == HW_ID_ER) {
		LDBG("HW ID [%s] , Bypass AP3425 Light Sensor\n",
			(hw_id == HW_ID_SR)?"SR":"ER");
		return 0;
	}

	ret = i2c_add_driver(&ap3425_driver);
	return ret;	

}

static void __exit ap3425_exit(void)
{
	i2c_del_driver(&ap3425_driver);
}

MODULE_AUTHOR("Templeton Tsai Dyna-Image Corporation.");
MODULE_DESCRIPTION("AP3425 driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(ap3425_init);
module_exit(ap3425_exit);



