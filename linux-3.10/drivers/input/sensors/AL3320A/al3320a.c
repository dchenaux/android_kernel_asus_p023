#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/kobject.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include "al3320a.h"
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/workqueue.h>
//#include <linux/cadiz.h>
#include <linux/wakelock.h>

// early suspend added by cheng_kao 2015.04.15 ++
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
#include <linux/suspend.h>
#endif

#include <linux/HWVersion.h>

//static int is_enable_cadiz_power = 0;
//extern bool cadiz_support;
extern int entry_mode;

#define AL3320A_DEV_NAME			"al3320a_dev"
#define AL3320A_INPUT_NAME		"al3320a_input"
#define DRIVER_VERSION				"1.0.0.0"
#define ALS_CALIBRATION_FILE_PATH	"/nvm_fs_partition/sensors/als_cali_z300c.ini"
#define AL3320A_NUM_CACHABLE_REGS	14
#define AL3320A_NUM_ADC_TO_LUX		8
#define AL3320A_NUM_LUX_LEVEL			20	// level = 20 + 2
#define AL3320A_NUM_LUX_TABLE_LEVEL	22
#define AL3320A_THRESHOLD_RANGE		20

#define AL3320A_DEFAULT_CALIBRATION	100	//(the display default 10%)
#define AL3320A_CALIBRATION_BASE		1000

#define AL3320A_SENSOR_STABLE_NUM	0x14	// 20 times

// For delay calibration
#define AL3320A_DELAY_CALITIME		20000
#define AL3320A_DELAY_RETRYCALITIME	3000
#define AL3320A_DELAY_CALIBRATION		1

// For Pollin mode
#define AL3320A_MIN_POLL_DELAY		1
#define AL3320A_MAX_POLL_DELAY		1000
//#define AL3320A_POLLIN_TIME				330
//#define AL3320A_POLLIN_MODE			1
//#define AL3320A_DELAY_WORKQUENE		1

//For Interrupt mode
#define AL3320A_INTERRUPT_MODE		1
#define AL3320A_WORKQUENE				1

#define AL3320A_INTERRUPT_TIME_STAG	50
#define AL3320A_INTERRUPT_TIME_LIMIT	3

// For fix light sensor read/write i2c fail
#define AL3320A_FIX_I2C_ERROR			1
#define AL3320A_RETRY_I2C_MAXTIME		20		// retry one : 3s --> 20 : 60s 
#define AL3320A_DELAY_RETRY_I2C		3000

static struct i2c_client *al3320a_i2c_client = NULL;

struct al3320a_data {
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct mutex lock;
	struct wake_lock als_wake_lock;
#ifdef AL3320A_WORK
	struct work_struct irq_work;
#endif
#ifdef AL3320A_WORKQUENE
	struct workqueue_struct *al3320a_wq;
#endif
#ifdef AL3320A_DELAY_WORKQUENE
	struct delayed_work delaywork;
	atomic_t delay;
#endif
	atomic_t enable;
	atomic_t pre_enable;
#ifdef AL3320A_DELAY_CALIBRATION
	struct delayed_work delayworkcalibration;
	atomic_t delaycalibration;
#endif
#ifdef AL3320A_FIX_I2C_ERROR
	struct delayed_work delayworki2cerror;
	atomic_t delayi2cerror;
#endif
	int al3320a_reg_status[AL3320A_NUM_CACHABLE_REGS];
	int al3320a_state;
	int al3320a_adc_to_lux;
	int al3320a_calibration;
	int int_gpio;
	int irq;
#ifdef AL3320A_POLLIN_MODE
	int val_level;
	int val_change;
#endif
#ifdef AL3320A_FIX_I2C_ERROR
	int retry_i2c_counter;
#endif
	int lux;
	int als_calibration_limit;
	int als_config_log;
};
static struct al3320a_data *light_sensor_data;

#ifdef AL3320A_INTERRUPT_MODE
static void al3320a_irq_work_function(struct work_struct *work);
#endif

#ifdef AL3320A_POLLIN_MODE
static void al3320a_delay_work_func(struct work_struct *work);
#endif

#ifdef CONFIG_AL3320A_WORK_DEFERRABLE
static struct workqueue_struct *al3320a_wq;
struct delayed_work al3320a_work;
#endif

#ifdef AL3320A_WORKQUENE
static DECLARE_WORK(al3320a_irq_work, al3320a_irq_work_function);
#endif

#ifdef AL3320A_DELAY_CALIBRATION
static void al3320a_delay_calibration_func(struct work_struct *work);
#endif

#ifdef AL3320A_FIX_I2C_ERROR
static void al3320a_delay_i2c_retry_func(struct work_struct *work);
#endif

// AL3320A register
static u8 al3320a_reg[AL3320A_NUM_CACHABLE_REGS] = {0x00,0x01,0x02,0x06,0x07,0x08,0x09,0x22,0x23,0x30,0x31,0x32,0x33,0x34};
//                                                                                           0       1      2      3      4       5      6      7       8     9     10     11    12     13

// AL3320A adc to lux mapping table
#define BASE_GAIN_VAL	0x03
#define BASE_GAIN_DENO	1000	//denominator ; numerator
static int al3320a_adc_to_lux_table[AL3320A_NUM_ADC_TO_LUX] ={508,1526,127,381,32,95,10,30};	// all value been * 1000

// AL3320A lux threshold level range  ; base on 0.381 lux/counter
static int al3320a_threshold_level[AL3320A_NUM_LUX_LEVEL] = {50, 100, 200, 300, 400, 500, 650, 800, 1000, 1500, 2000, 3000, 4000, 5000, 7000, 10000, 12500, 15000, 17500, 20000};

#ifdef AL3320A_INTERRUPT_MODE
//base table
//static u8 al3320a_threshold_table_uL[AL3320A_NUM_LUX_TABLE_LEVEL] = {0x0, 0x83, 0x06, 0x0D, 0x13, 0x1A, 0x20, 0xAA, 0x34, 0x41, 0x61, 0x81, 0xC2, 0x03,   0x43,   0xC5,   0x87,   0x28,   0xCA,  0x6C,  0x0D,   0xFF}; 
//static u8 al3320a_threshold_table_uH[AL3320A_NUM_LUX_TABLE_LEVEL] = {0x0,  0x0,  0x1,  0x2,   0x3,   0x4,   0x5,  0x6,   0x8,   0xA,   0xF,  0x14, 0x1E, 0x29,   0x33,   0x4C,   0x66,   0x80,   0x99,  0xB3,  0xCD,   0xFF};
static int al3320a_threshold_table[AL3320A_NUM_LUX_TABLE_LEVEL] ={             0,  131, 262,   525,   787,   1050, 1312, 1706, 2100, 2625, 3937, 5249, 7874, 10499, 13123, 18373, 26247, 32808, 39370, 45932, 52493, 65535};
//reduce the lux to 10% by panel , so change table to  al3320a_threshold_table={  0,  	13,   26,     53,     79,     105,   131,   171,   210,   263,  394,   525,  787,   1050,   1312,   1837,  2625,   3281,  3937,   4593,   5249, 65535};
//static u8 al3320a_threshold_table_uL[AL3320A_NUM_LUX_TABLE_LEVEL] = {0x0, 0x0D, 0x1A, 0x35, 0x4F, 0x69, 0x83, 0xAB, 0xD2, 0x07, 0x8A, 0x0D, 0x13,   0x1A,   0x20,   0x2D,   0x65,   0xD1,  0x61,  0xF3,   0x81,   0xFF}; 
//static u8 al3320a_threshold_table_uH[AL3320A_NUM_LUX_TABLE_LEVEL] = {0x0,  0x0,  0x0,  0x0,   0x0,   0x0,   0x0,  0x0,   0x0,   0x1,   0x1,    0x2,   0x3,   0x4,    0x5,      0x7,    0xA,     0xC,   0xF,    0x11,   0x14,   0xFF};
	#ifdef AL3320A_INTERRUPT_TIMESTAG
//struct als3320a_interrupt_state {
//	u64 event_jiffies64;
//	int state;
//};
//struct als3320a_interrupt_state g_als_interrupt_state;
	#endif
#endif

static char *al3320a_reg_name[AL3320A_NUM_CACHABLE_REGS]={"SYS_CONFIG","FLAG_STATUS","INT_CONFIG","WAITING","GAIN","PERSIST","MEAN_TIME","DATA_LOW","DATA_HIGH","THRES_LOW_L","THRES_LOW_H","THRES_HIGH_L","THRES_HIGH_H","CALIBRATION"};


// early suspend added by cheng_kao 2015.04.15 ++
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
static struct callback_data* callback_struct;
void als3320a_screen_changed_listener(const int state);
#endif
//	al3320a Unit Function ++
static int al3320a_check_device(int state);
static int al3320a_input_init(void);
static int al3320a_read_calibration(void);
static int al3320a_chip_init(void);
static int al3320a_enable_init(void);
static int al3320a_enable(int iEnable);
static int al3320a_get_adc_counter(void);
#ifdef AL3320A_INTERRUPT_MODE
static int al3320a_set_threshold(int ilux);
static int al3320a_set_spec_threshold(int adc);
#endif
static int als_write_to_califile(int ilux);
//	al3320a Unit  Function --

static struct kobject *android_light_kobj = NULL;	// Sys kobject variable

//	define for DEBUG message 
#define ALS_MESSAGE	1	// default 1 : for show status 
#define ALS_DEBUGMSG	0	// default 0 : for Test and debug


/* Add by for define HW_ID */
extern int Read_HW_ID(void);


/*--------------------------------------------------------------------------------*/
//unit function  ++

#ifdef AL3320A_INTERRUPT_MODE
static int al3320a_set_threshold(int ilux)
{
	int index=0,ret=0;
	u8 low_val_ul=0x00,low_val_uh=0x00,high_val_ul=0x00,high_val_uh=0x00;// ,als_persist=0x01;
	// get the threshold table index
	if(ilux>=0){
		for(index=0;index<AL3320A_NUM_LUX_LEVEL;index++){
			if(ilux<al3320a_threshold_level[index])
				break;
		}
		if(ilux>20000)
			index=AL3320A_NUM_LUX_LEVEL+1;
		
	//	low_val_ul = al3320a_threshold_table_uL[index];
	//	low_val_uh = al3320a_threshold_table_uH[index];
	//	high_val_ul = al3320a_threshold_table_uL[index+1];
	//	high_val_uh = al3320a_threshold_table_uH[index+1];
		low_val_ul = ((al3320a_threshold_table[index] << 8) >> 8);
		low_val_uh = (al3320a_threshold_table[index] >> 8);
		high_val_ul = ((al3320a_threshold_table[index+1] << 8) >> 8);
		high_val_uh = (al3320a_threshold_table[index+1] >> 8);
		if(ALS_DEBUGMSG) printk("alp.D : index(%d) , LL(%d), LH(%d), HL(%d), HH(%d)!!\n",index,low_val_ul,low_val_uh,high_val_ul,high_val_uh);
	}else{
		low_val_ul = 0xFF;
		low_val_uh = 0xFF;
		high_val_ul = 0x00;
		high_val_uh = 0x00;
		printk("alp.D : set threshold into init mode!!\n");
	}
	
	mutex_lock(&light_sensor_data->lock);
	// change the threshold
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_LOW_L, low_val_ul);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_set_threshold low_val_ul fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_LOW_L] = low_val_ul;

	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_LOW_H, low_val_uh);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_set_threshold low_val_uh fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_LOW_H] = low_val_uh;

	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_HIGH_L, high_val_ul);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_set_threshold high_val_ul fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_HIGH_L] = high_val_ul;

	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_HIGH_H, high_val_uh);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_set_threshold high_val_uh fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_HIGH_H] = high_val_uh;

	//Persist set to 0x01 or 0x03
//	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_PERSIST, als_persist);
//	if (ret){
//		if(ALS_MESSAGE) printk("alp.D : al3320a_set_threshold als_persist fail !!\n");
//		return ret;
//	}
//	light_sensor_data->al3320a_reg_status[INDEX_ALS_PERSIST] = als_persist;
	mutex_unlock(&light_sensor_data->lock);
	if(ALS_DEBUGMSG) 
		printk("alp.D : al3320a_set_threshold success!!\n");

	return 1;
}

static int al3320a_set_spec_threshold(int i_adc)
{
	int ret=0, upper_adc=0, lower_adc=0;
	u8 low_val_ul=0x00,low_val_uh=0x00,high_val_ul=0x00,high_val_uh=0x00;

	upper_adc = i_adc*(100+AL3320A_THRESHOLD_RANGE)/100 ;
	lower_adc = i_adc*(100-AL3320A_THRESHOLD_RANGE)/100 ;

	low_val_ul = ((lower_adc << 8) >> 8);
	low_val_uh = (lower_adc>> 8);
	high_val_ul = ((upper_adc<< 8) >> 8);
	high_val_uh = (upper_adc >> 8);
	if(light_sensor_data->als_config_log!=0)
		printk("alp.D : upper_adc:%d, lower_adc:%d , LL(%d), LH(%d), HL(%d), HH(%d)!!\n",upper_adc,lower_adc,low_val_ul,low_val_uh,high_val_ul,high_val_uh);

	mutex_lock(&light_sensor_data->lock);
	// change the threshold
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_LOW_L, low_val_ul);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_set_spec_threshold low_val_ul fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_LOW_L] = low_val_ul;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_LOW_H, low_val_uh);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_set_spec_threshold low_val_uh fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_LOW_H] = low_val_uh;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_HIGH_L, high_val_ul);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_set_spec_threshold high_val_ul fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_HIGH_L] = high_val_ul;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_HIGH_H, high_val_uh);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_set_spec_threshold high_val_uh fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_HIGH_H] = high_val_uh;

	mutex_unlock(&light_sensor_data->lock);
	if(ALS_DEBUGMSG) printk("alp.D : al3320a_set_spec_threshold success!!\n");

	return 1;
}

#endif

#ifdef AL3320A_POLLIN_MODE
static int al3320a_check_threshold_val(int ilux)
{
	int index=0;
	// get the threshold table index
	for(index=0;index<AL3320A_NUM_LUX_LEVEL;index++){
		if(ilux<al3320a_threshold_level[index])
			break;
	}
	if(ilux>20000)
		index=AL3320A_NUM_LUX_LEVEL+1;

	if(light_sensor_data->val_level!=index){
		light_sensor_data->val_level = index;
		light_sensor_data->val_change = 1;
		if(ALS_DEBUGMSG) printk("alp.D : al3320a_check_threshold_val changed!!\n");
	}
	return 1;
}
#endif

static int al3320a_get_adc_counter(void)
{
	int iadc_low=0,iadc_high=0, iadc=0; //, intr;
	// clean int flag
	iadc_low = i2c_smbus_read_byte_data(light_sensor_data->client, ADDR_ALS_DATA_LOW);
	if (iadc_low<0){
		if(ALS_MESSAGE) printk("alp.D : al3320a_chip_init ADDR_ALS_DATA_LOW fail !!\n");
		return iadc_low;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_DATA_LOW] = iadc_low;

	iadc_high = i2c_smbus_read_byte_data(light_sensor_data->client, ADDR_ALS_DATA_HIGH);
	if (iadc_high<0){
		if(ALS_MESSAGE) printk("alp.D : al3320a_chip_init ADDR_ALS_DATA_HIGH fail !!\n");
		return iadc_high;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_DATA_HIGH] = iadc_high;
	iadc = ( (iadc_high<<8) |iadc_low );

//	if(ALS_DEBUGMSG) {
//		printk("alp.D : al3320a_get_adc_counter(%d) !!\n",iadc);
//		intr = i2c_smbus_read_byte_data(light_sensor_data->client, ADDR_ALS_FLAG_STATUS);
//		if (intr<0){
//			printk("alp.D : al3320a_get_adc_counter ADDR_ALS_FLAG_STATUS fail !!\n");
//		}else{
//			printk("alp.D : al3320a_get_adc_counter ADDR_ALS_FLAG_STATUS (%d) !!\n",intr);
//		}
//	}

	if (iadc<0){
		if(ALS_MESSAGE) printk("alp.D : al3320a_get_adc_counter adc fail !!\n");
		return 0;
	}
	return iadc;
}

static int al3320a_enable(int iEnable)
{
	int ret = 0;
	u8 reg_val = 0x00;
	switch(iEnable)
	{
		case 0:
			reg_val = 0x00;
#ifdef AL3320A_WORKQUENE
//			disable_irq_nosync(light_sensor_data->irq);
#endif

			if(atomic_read(&light_sensor_data->enable)==1){
#ifdef AL3320A_DELAY_WORKQUENE
				cancel_delayed_work_sync(&light_sensor_data->delaywork);
				if(ALS_DEBUGMSG) printk("alp.D : disable al3220a pollin !!\n");
#endif
				atomic_set(&light_sensor_data->enable, 0);
			}else{
				if(ALS_DEBUGMSG) printk("alp.D : al3220a already disable!!\n");
			}
		break;

		case 1:
			reg_val = 0x01;
#ifdef CONFIG_AL3320A_WORK_DEFERRABLE
//			disable_irq_nosync(light_sensor_data->irq);
			queue_delayed_work(al3320a_wq, &al3320a_work,0);
			if(ALS_DEBUGMSG) printk("alp.D : CONFIG_AL3320A_WORK_DEFERRABLE queue_delayed_work!!\n");
#endif

#ifdef AL3320A_WORKQUENE
//			enable_irq(light_sensor_data->irq);
			if(ALS_DEBUGMSG) printk("alp.D : AL3320A_WORKQUENE !!\n");
#endif
			// enable light sensor and then change the threshold
			al3320a_set_threshold(-1);
			if(atomic_read(&light_sensor_data->enable)==0){
#ifdef AL3320A_DELAY_WORKQUENE
				schedule_delayed_work(&light_sensor_data->delaywork, msecs_to_jiffies(atomic_read(&light_sensor_data->delay)));
				if(ALS_DEBUGMSG) printk("alp.D : enable al3220a pollin !!\n");
#endif

				atomic_set(&light_sensor_data->enable, 1);
			}else{
				if(ALS_DEBUGMSG) printk("alp.D : al3220a already enable!!\n");
			}
		break;

		default:
			reg_val = 0x00;
		break;
	}
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_SYS_CONFIG, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_set_mode ADDR_ALS_SYS_CONFIG fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_SYS_CONFIG] = reg_val;	
	if(ALS_MESSAGE) printk(KERN_INFO "al3320a_enable(%d) !!\n",iEnable);
	return 1;
}

static int al3320a_input_init(void)
{
	int ret;

	// allocate light input_device 
	light_sensor_data->input_dev = input_allocate_device();
	if (!light_sensor_data->input_dev) {
		printk("could not allocate input device\n");
		goto err_light_all;
		}
	// input_set_drvdata(input_dev, light_sensor_data);
	light_sensor_data->input_dev->name = AL3320A_INPUT_NAME;
	input_set_capability(light_sensor_data->input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(light_sensor_data->input_dev, ABS_MISC, 0, 1, 0, 0);

	if(ALS_MESSAGE) printk("alp : al3320a registering light sensor input device\n");
	ret = input_register_device(light_sensor_data->input_dev);
	if (ret < 0) {
		printk("could not register input device\n");
		goto err_light_reg;
	}
	return 0;

err_light_reg:
	input_free_device(light_sensor_data->input_dev);
err_light_all:
	return (-1);   
}

static int al3320a_read_calibration(void)
{
	struct file *fp=NULL;
	char c_data[8]={0}; //c_data-> calibration data
	int ilen=0,ret=0;
//	u8 reg_val = 0x00;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp=filp_open(ALS_CALIBRATION_FILE_PATH,O_RDONLY,0660);
	if (IS_ERR_OR_NULL(fp)){
		printk("al3320a : File Open Failed \n");
		return -ENOENT;
	}
	if(fp->f_op != NULL && fp->f_op->read != NULL){
		ilen = fp->f_op->read(fp,c_data,8,&fp->f_pos);
		if(ALS_DEBUGMSG) printk("al3320a : fp->f_op->read (len = %d) \n",ilen);
	}
	set_fs(old_fs);		
	filp_close(fp,NULL);

	if(ilen<=0){
//			reg_val = 64;
			light_sensor_data->al3320a_calibration = AL3320A_DEFAULT_CALIBRATION;
			if(ALS_MESSAGE) printk(KERN_INFO "al3320a : read file fail\n");
	}else{
			if(ALS_MESSAGE) printk(KERN_INFO "al3320a : cal : %s\n",c_data);
//			reg_val = 6400/( (c_data[0]-48)*1000+(c_data[1]-48)*100+(c_data[2]-48)*10+(c_data[3]-48) );
			light_sensor_data->al3320a_calibration = (c_data[0]-48)*1000+(c_data[1]-48)*100+(c_data[2]-48)*10+(c_data[3]-48) ;
	}

	// change the threshold table 
//	for(ilen=1;ilen<(AL3320A_NUM_LUX_TABLE_LEVEL-1);ilen++){
//		al3320a_threshold_table[ilen]=al3320a_threshold_table[ilen]*light_sensor_data->al3320a_calibration/AL3320A_CALIBRATION_BASE;
//	}

//	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_CALIBRATION, reg_val);
//	if (ret){
//		if(ALS_MESSAGE) printk("alp.D : al3320a_read_calibration ADDR_ALS_CALIBRATION fail !!\n");
//		return ret;
//	}
//	printk(KERN_INFO "al3320a : org_cal=%d , new_cal=%d\n",light_sensor_data->al3320a_reg_status[INDEX_ALS_CALIBRATION],reg_val);
//	light_sensor_data->al3320a_reg_status[INDEX_ALS_CALIBRATION] = reg_val;
	ret = 1;

	return ret; 
}

static int al3320a_chip_init(void)
{
	int ret=0;
	u8 reg_val = 0x00;

	// power down chip at init
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_SYS_CONFIG, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_set_mode ADDR_ALS_SYS_CONFIG fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_SYS_CONFIG] = reg_val;	

	// clean int flag
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_FLAG_STATUS, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_chip_init ADDR_ALS_FLAG_STATUS fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_FLAG_STATUS] = reg_val;

	// set lux reg : 0~25K
	reg_val = BASE_GAIN_VAL;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_GAIN, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_chip_init ADDR_ALS_GAIN fail !!\n");
		return ret;
	}
	ret = reg_val;
	light_sensor_data->al3320a_reg_status[INDEX_ALS_GAIN] = reg_val;
	light_sensor_data->al3320a_adc_to_lux = al3320a_adc_to_lux_table[ret];

	//Persist set to 0x01 or AL3320A_SENSOR_STABLE_NUM
	reg_val = AL3320A_SENSOR_STABLE_NUM;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_PERSIST, reg_val );
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_chip_init als_persist fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_PERSIST] = reg_val;

	// set threshold : low-65535, high:0 to trigger when al3320a starts.
	reg_val = 0xFF;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_LOW_L, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_chip_init ADDR_ALS_THRES_LOW_L fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_LOW_L] = reg_val;
	reg_val = 0xFF;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_LOW_H, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_chip_init ADDR_ALS_THRES_LOW_H fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_LOW_H] = reg_val;
	reg_val = 0x00;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_HIGH_L, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_chip_init ADDR_ALS_THRES_HIGH_L fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_HIGH_L] = reg_val;
	reg_val = 0x00;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_HIGH_H, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_chip_init ADDR_ALS_THRES_HIGH_H fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_HIGH_H] = reg_val;

	al3320a_check_device(0);
	if(ALS_MESSAGE) printk("alp.D : al3320a_chip_init success!!\n");
	return 0;
}

static int al3320a_enable_init(void)
{
	int ret=0;
	u8 reg_val = 0x00;

	// clean int flag
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_FLAG_STATUS, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_enable_init ADDR_ALS_FLAG_STATUS fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_FLAG_STATUS] = reg_val;

	//Persist set to 0x01 or AL3320A_SENSOR_STABLE_NUM
	reg_val = AL3320A_SENSOR_STABLE_NUM;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_PERSIST, reg_val );
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_enable_init als_persist fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_PERSIST] = reg_val;

	// set threshold : low-65535, high:0 to trigger when al3320a starts.
	reg_val = 0xFF;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_LOW_L, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_enable_init ADDR_ALS_THRES_LOW_L fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_LOW_L] = reg_val;
	reg_val = 0xFF;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_LOW_H, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_enable_init ADDR_ALS_THRES_LOW_H fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_LOW_H] = reg_val;
	reg_val = 0x00;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_HIGH_L, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_enable_init ADDR_ALS_THRES_HIGH_L fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_HIGH_L] = reg_val;
	reg_val = 0x00;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_HIGH_H, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_enable_init ADDR_ALS_THRES_HIGH_H fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_HIGH_H] = reg_val;


	if(ALS_MESSAGE) printk("alp.D : al3320a_enable_init success!!\n");
	return 0;
}

static int als_write_to_califile(int ilux)
{
	int retval = 0, ilen=0;
	char tmp_data[8]={0};
//	read file data++
	struct file *fp=NULL;
	mm_segment_t old_fs;
//	read file data--
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp=filp_open(ALS_CALIBRATION_FILE_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR(fp)){
		printk(KERN_INFO "alp.D : filp_open fail\n");
		retval = -1;
		return retval;
	}
	if( (ilux>2000) || (ilux<45) )
		printk(KERN_INFO "alp.D : out of range, ilux(%d)\n",ilux);

	sprintf(tmp_data, "%04d\n", ilux);
	printk(KERN_INFO "%s \n",tmp_data);

	ilen = fp->f_op->write(fp,tmp_data,8,&fp->f_pos);
	printk(KERN_INFO "alp.D write file len : %d\n",ilen);
	set_fs(old_fs);
	filp_close(fp,NULL);
	retval = 1;
	
	return retval;
}

static int al3320a_check_device(int state)
{
	int iloop=0, ret=0;
	char buf[32]={0};
	for (iloop = 0; iloop < AL3320A_NUM_CACHABLE_REGS; iloop++) {
		ret = i2c_smbus_read_byte_data(light_sensor_data->client, al3320a_reg[iloop]);
		light_sensor_data->al3320a_reg_status[iloop] = ret ;
		if (ret < 0)
			return sprintf(buf, " iloop %d \n , %d , status fail!!",iloop ,ret);
		if(ALS_DEBUGMSG || state)	printk("alp.D : %s -> %d\n",al3320a_reg_name[iloop],light_sensor_data->al3320a_reg_status[iloop]);
	}
//	light_sensor_data->al3320a_state = 1;
	if(ALS_MESSAGE) printk("alp.D : al3320a_check_device success!!\n");
	return 1;
}
//unit function --
/*--------------------------------------------------------------------------------*/

// state ++
static ssize_t al3320a_show_state(struct device *dev,struct device_attribute *attr,char *buf)
{
	int ret = 0;
	al3320a_check_device(0);
	ret = light_sensor_data->al3320a_state;
	return sprintf(buf, "%d\n", ret);
}
// state --

// poll_delay ++
static ssize_t al3320a_show_delay(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	if(ALS_DEBUGMSG) printk("als : al3320a_show_delay !!\n");
	return sprintf(buf, "mode : %d\n", ret);
}
static ssize_t al3320a_set_delay(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
//	unsigned long val;
#ifdef AL3320A_POLLIN_MODE
	struct cm36686_info *lpi = lp_info;
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;

	if ((interval_ms < AL3320A_MIN_POLL_DELAY) ||
			(interval_ms > AL3320A_MAX_POLL_DELAY))
		return -EINVAL;

	atomic_set(&light_sensor_data->delay, (unsigned int) interval_ms); 
#endif
	if(ALS_DEBUGMSG) printk("als : al3320a_set_delay !!\n");	
	return count;
}
// poll_delay --

// enable ++
static ssize_t al3320a_show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	if(ALS_DEBUGMSG) printk("als : al3320a_show_mode !!\n");
	ret = i2c_smbus_read_byte_data(light_sensor_data->client, al3320a_reg[INDEX_ALS_SYS_CONFIG]);
	if (ret < 0)
		return sprintf(buf, " SYS_CONFIG status fail (%d)!!",ret);
	return sprintf(buf, "mode : %d\n", ret);
}
static ssize_t al3320a_set_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret=-1;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 7)){
		printk("als : al3320a_set_mode buf(%d) !!\n",buf[0]);
		return -EINVAL;
	}
	if(buf[0]=='0'){
		if(ALS_MESSAGE) printk("als : al3320a_set_mode 0 !!\n");
		ret = 0;
	}
	else if(buf[0]=='1'){
		if(ALS_MESSAGE) printk("als : al3320a_set_mode 1 !!\n");
		ret = 1;
	}else{
		printk("als : al3320a_set_mode fail !!\n");
		return ret;
	}
	al3320a_enable(ret);
	return count;
}
// enable --

// meantime ++
static ssize_t al3320a_show_mean(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	if(ALS_DEBUGMSG) printk("als : al3320a_show_mean !!\n");
	ret = i2c_smbus_read_byte_data(light_sensor_data->client, al3320a_reg[INDEX_ALS_MEAN_TIME]);
	if (ret < 0)
		return sprintf(buf, " ALS_MEAN_TIME read status fail (%d)!!",ret);
	return sprintf(buf, "mean time : %d\n", ret);
}
static ssize_t al3320a_set_mean(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret=-1;
	u8 reg_val = 0x01;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 7)){
		printk("als : al3320a_set_mean buf(%d) !!\n",buf[0]);
		return -EINVAL;
	}
	if( (buf[0]<=48)||(buf[0]>57) ){
		if(ALS_MESSAGE) printk("als : al3320a_set_mean out of range\n");
		reg_val = 0x00;
	}
	else{
		reg_val = (buf[0]-48);
		printk("als : set mean time val %d !!\n",reg_val);
		return ret;
	}
	if(reg_val!=0){
		ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_MEAN_TIME, reg_val);
		if (ret){
			if(ALS_MESSAGE) printk("alp.D : al3320a_set_mean ADDR_ALS_MEAN_TIME fail !!\n");
			return ret;
		}
		light_sensor_data->al3320a_reg_status[ADDR_ALS_MEAN_TIME] = reg_val;	
	}
	return count;
}
// meantime --

// adc ++
static ssize_t al3320a_show_adc(struct device *dev,struct device_attribute *attr,char *buf)
{
	int iadc=0;
	if(ALS_DEBUGMSG) printk("alp.D : al3320a_show_adc !!\n");
	iadc = al3320a_get_adc_counter();
	printk("alp.D : al3320a_show_adc(%d) !!\n",iadc);
	return sprintf(buf, "%d \n", iadc);
}
// adc --

// lux ++
static ssize_t al3320a_show_lux(struct device *dev,struct device_attribute *attr,char *buf)
{
	int ilux=0 ,ori_status_changed=0;
	if(ALS_DEBUGMSG) printk("alp.D: al3320a_show_lux !!\n");
	if(light_sensor_data->al3320a_reg_status[INDEX_ALS_SYS_CONFIG]==0){
		al3320a_enable(1);
		ori_status_changed=1;
	}
	if(light_sensor_data->al3320a_calibration==0)
		light_sensor_data->al3320a_calibration = AL3320A_DEFAULT_CALIBRATION;// use default setting
	ilux = (al3320a_get_adc_counter()* light_sensor_data->al3320a_adc_to_lux /light_sensor_data->al3320a_calibration);
	printk("alp.D: al3320a_show_lux(%d) !!\n",ilux);
	if(ori_status_changed==1){
		al3320a_enable(0);
	}
	return sprintf(buf, "%d \n", ilux);
}
// lux --

// config ++
static ssize_t al3320a_show_config(struct device *dev,struct device_attribute *attr,char *buf)
{
	if(ALS_DEBUGMSG) printk("alp.D : al3320a_show_config !!\n");
//	for(int i=0; i<AL3320A_NUM_CACHABLE_REGS; i++){
//		printk("alp.D : %s -> %d\n",al3320a_reg_name[i],light_sensor_data->al3320a_reg_status[i]);
//	}
	al3320a_check_device(1);
	return sprintf(buf, "%d \n", 0);
}
// config --

// gain ++
static ssize_t al3320a_show_gain(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
//	ret = al3320a_check_device(1);
	ret = light_sensor_data->al3320a_adc_to_lux;
	printk("alp.D : al3320a_show_config(%d) !!\n",ret);
//	al3320a_read_calibration();
	return sprintf(buf, "%d \n", ret);
}
// gain --

// calibration ++
static ssize_t al3320a_set_calibration(struct device *dev, struct device_attribute *attr, char *buf)
{
	int iloop=0, icount=0 ,ori_status_changed=0;
	int ilux=0, iadc=0, tempadc=0;
	if(light_sensor_data->al3320a_reg_status[INDEX_ALS_SYS_CONFIG]==0){
		al3320a_enable(1);
		ori_status_changed=1;
	}
	for(iloop=0;iloop<10;iloop++){
		if(iloop>4){
			tempadc = al3320a_get_adc_counter();
			if(tempadc > light_sensor_data->als_calibration_limit){
				iadc = tempadc+iadc;
				icount ++ ;
			}
		}
		udelay(500);
	}
	printk("alp.D : adc (%d) ,count(%d)!!\n",iadc,icount);
	if(icount>0)
		iadc = iadc /icount;
	else{
		printk("alp.D : calibration get rawdata fail !!\n");
		return sprintf(buf, "%d \n", 0);
	}
	if(ori_status_changed==1){
		al3320a_enable(0);
	}
	if(iadc > light_sensor_data->als_calibration_limit)	
		printk("alp.D : al3320a_set_calibration(%d) !!\n",iadc);
	else{
		printk("alp.D : calibration value low than limit(%d) !!\n",light_sensor_data->als_calibration_limit);
		return sprintf(buf, "%d \n", 0);
	}
//	ilux = (al3320a_get_adc_counter()* light_sensor_data->al3320a_adc_to_lux / light_sensor_data->al3320a_calibration);
	ilux = (iadc* light_sensor_data->al3320a_adc_to_lux/BASE_GAIN_DENO );
	if(als_write_to_califile(ilux)==1)
		al3320a_read_calibration();

	if(ilux>0){
		printk("alp.D : calibration success lux (%d)!!\n", ilux);
	}
	else{
		printk("alp.D : calibration fail!!\n");
	}
	if(light_sensor_data->al3320a_calibration==0)
		light_sensor_data->al3320a_calibration = AL3320A_DEFAULT_CALIBRATION;// use default setting
	return sprintf(buf, "%d \n", ilux);
}
// calibration --

// report ++
static ssize_t al3320a_set_report(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret=-1;
	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 7)){
		printk("als : al3320a_set_report buf(%d) !!\n",buf[0]);
		return -EINVAL;
	}
	if(buf[0]=='0'){
		if(ALS_MESSAGE) printk("als : al3320a_set_report 0 !!\n");
		input_report_abs(light_sensor_data->input_dev, ABS_MISC, 0);
		input_sync(light_sensor_data->input_dev);
	}
	else if(buf[0]=='1'){
		if(ALS_MESSAGE) printk("als : al3320a_set_report 1000 !!\n");
		input_report_abs(light_sensor_data->input_dev, ABS_MISC, 1000);
		input_sync(light_sensor_data->input_dev);
	}
	else if(buf[0]=='2'){
		if(ALS_MESSAGE) printk("als : al3320a_set_report 1000 !!\n");
		input_report_abs(light_sensor_data->input_dev, ABS_MISC, 8000);
		input_sync(light_sensor_data->input_dev);
	}else{
		if(ALS_MESSAGE) printk("als : al3320a_set_report 350 !!\n");
		input_report_abs(light_sensor_data->input_dev, ABS_MISC, 350);
		input_sync(light_sensor_data->input_dev);
		return ret;
	}
	return count;
}
// report --

// irq ++
static ssize_t al3320a_clean_irq(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0,istatus=0;
	u8 reg_val = 0x00;

	if(ALS_DEBUGMSG) printk("als : al3320a_clean_irq !!\n");
	istatus = i2c_smbus_read_byte_data(light_sensor_data->client, al3320a_reg[INDEX_ALS_FLAG_STATUS]);
	printk("alp.D : al3320a_show_irq(%d) and start to clean!!\n",istatus);

	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_FLAG_STATUS, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : al3320a_set_irq clean flag fail !!\n");
		return -EINVAL;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_FLAG_STATUS] = reg_val;
	return sprintf(buf, "%d \n", ret);
}
// irq --

// cal-limit ++
static ssize_t al3320a_show_cal_limit(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(ALS_DEBUGMSG) printk("alp.D : al3320a_show_cal_limit !!\n");
	return sprintf(buf, "cal-limit : %d\n", light_sensor_data->als_calibration_limit);
}
static ssize_t al3320a_set_cal_limit(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int lux=0;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 2)){
		printk("als : al3320a_set_cal_limit buf num(%d) !!\n",val);
		return -EINVAL;
	}
	if( (buf[0]<=48)||(buf[0]>57) || (buf[1]<=48)||(buf[2]>57) ){
		if(ALS_MESSAGE) printk("als : al3320a_set_cal_limit error input , use defaul limit \n");
		light_sensor_data->als_calibration_limit = 40*BASE_GAIN_DENO/light_sensor_data->al3320a_adc_to_lux;
	}
	else{
		lux = (buf[0]-48)*10 +(buf[1]-48);
		printk("als : set al3320a_set_cal_limit : %d !!\n",lux);
		light_sensor_data->als_calibration_limit = lux*BASE_GAIN_DENO/light_sensor_data->al3320a_adc_to_lux;
	}
	return count;
}
// cal-limit --

// log_statue ++
static ssize_t al3320a_show_log_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	if(ALS_DEBUGMSG) printk("alp.D : al3320a_show_log_state !!\n");
	ret = light_sensor_data->als_config_log;
	return sprintf(buf, "log_state : %d\n", ret);
}
static ssize_t al3320a_set_log_state(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if(buf[0]=='0'){
		if(ALS_MESSAGE) printk("alp.D : close log !!\n");
		light_sensor_data->als_config_log = 0;
	}
	else if(buf[0]=='1'){
		if(ALS_MESSAGE) printk("alp.D : open log !!\n");
		light_sensor_data->als_config_log = 1;
	}else{
		if(ALS_MESSAGE) printk("alp.D : other log state, keep opening !!\n");
		light_sensor_data->als_config_log = 1;
	}
	return count;
}
// log_statue --


static DEVICE_ATTR(state, S_IRUGO,al3320a_show_state,NULL);
static DEVICE_ATTR(poll_delay, 0660 ,al3320a_show_delay,al3320a_set_delay);
static DEVICE_ATTR(enable, 0660 ,al3320a_show_mode,al3320a_set_mode);
static DEVICE_ATTR(meantime, 0660 ,al3320a_show_mean,al3320a_set_mean);
static DEVICE_ATTR(adc, S_IRUGO,al3320a_show_adc,NULL);
static DEVICE_ATTR(lux, S_IRUGO,al3320a_show_lux,NULL);
static DEVICE_ATTR(config, S_IRUGO,al3320a_show_config,NULL);
static DEVICE_ATTR(gain, S_IRUGO,al3320a_show_gain,NULL);
static DEVICE_ATTR(calibration, S_IRUGO,al3320a_set_calibration,NULL);
static DEVICE_ATTR(report, 0660 ,NULL,al3320a_set_report);
static DEVICE_ATTR(cleanirq, 0660 ,al3320a_clean_irq,NULL);
static DEVICE_ATTR(cal_limit, 0660 ,al3320a_show_cal_limit,al3320a_set_cal_limit);
static DEVICE_ATTR(log_state, 0660 ,al3320a_show_log_state,al3320a_set_log_state);

static struct attribute *al3320a_attributes[] = {
	&dev_attr_state.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_meantime.attr,
	&dev_attr_adc.attr,
	&dev_attr_lux.attr,
	&dev_attr_config.attr,
	&dev_attr_gain.attr,
	&dev_attr_calibration.attr,
	&dev_attr_report.attr,
	&dev_attr_cleanirq.attr,
	&dev_attr_cal_limit.attr,
	&dev_attr_log_state.attr,
	NULL
};

static const struct attribute_group al3320a_attr_group = {
	.attrs = al3320a_attributes,
};

#ifdef AL3320A_INTERRUPT_MODE
// internal interrupt function ++
static irqreturn_t als_interrupt(int irq, void *dev_id)
{
	if(ALS_DEBUGMSG) printk("alp.D : als_interrupt !!\n");

//#ifdef AL3320A_BMMITEST_BYPASS
	if(light_sensor_data->al3320a_state==0){
		printk("alp.D : als_interrupt Not ready!!\n");
		return IRQ_HANDLED;
	}
//#endif

	wake_lock(&light_sensor_data->als_wake_lock);

#ifdef CONFIG_AL3320A_WORK_DEFERRABLE
//	disable_irq_nosync(irq);
	queue_delayed_work(al3320a_wq, &al3320a_work,0);
	if(ALS_DEBUGMSG) printk("alp.D : als_interrupt queue_delayed_work !!\n");
#endif

#ifdef AL3320A_WORK
//	disable_irq_nosync(irq);
	schedule_work(&light_sensor_data->irq_work);
	if(ALS_DEBUGMSG) printk("alp.D : als_interrupt irq_work !!\n");
#endif

#ifdef AL3320A_WORKQUENE
//	disable_irq_nosync(irq);
	queue_work(light_sensor_data->al3320a_wq, &al3320a_irq_work);
	if(ALS_DEBUGMSG) printk("alp.D : als_interrupt al3320a_irq_work !!\n");
#endif

	return IRQ_HANDLED;
}

static void al3320a_irq_work_function(struct work_struct *work)
{
	int ret=0,als_iadc=0,als_lux=0;
	u8 reg_val=0x00;

#ifdef AL3320A_INTERRUPT_TIMESTAG
//	u64 time_stag=0;
//	u64 new_jiffies=get_jiffies_64();	
//	if(ALS_DEBUGMSG) printk("alp.D : ap3212c_work_function ; jiffies : %lld!!\n",new_jiffies);	
#endif

	als_iadc = al3320a_get_adc_counter();
	if(als_iadc<0){
		if(ALS_MESSAGE) printk("alp.D : als_interrupt fail(%d)!!\n",als_iadc);
#ifdef AL3320A_FIX_I2C_ERROR
		atomic_set(&light_sensor_data->delayi2cerror, AL3320A_DELAY_RETRY_I2C);
		schedule_delayed_work(&light_sensor_data->delayworki2cerror, msecs_to_jiffies(atomic_read(&light_sensor_data->delayi2cerror)));
#endif
		wake_unlock(&light_sensor_data->als_wake_lock);
		return ;
	}		

	//	change the adc value to lux and reprot lux to hal
//	als_lux = (als_iadc * light_sensor_data->al3320a_adc_to_lux / BASE_GAIN_DENO)*(1000/light_sensor_data->al3320a_calibration);
	als_lux = (als_iadc * light_sensor_data->al3320a_adc_to_lux / light_sensor_data->al3320a_calibration);
	input_report_abs(light_sensor_data->input_dev, ABS_MISC, als_lux);
	input_sync(light_sensor_data->input_dev);

	//	change the threshold
	al3320a_set_spec_threshold(als_iadc);
	if(light_sensor_data->als_config_log!=0)
		printk("alp.D : als_interrupt adc(%d) ; lux(%d)!!\n",als_iadc ,als_lux);

#ifdef AL3320A_INTERRUPT_TIMESTAG
//	if(g_als_interrupt_state.event_jiffies64==0){
//		g_als_interrupt_state.event_jiffies64 = new_jiffies;
//	}
//	else{
//		time_stag = new_jiffies-g_als_interrupt_state.event_jiffies64;
//		if(time_stag < AL3320A_INTERRUPT_TIME_STAG){	// 500 ms
//			g_als_interrupt_state.state = g_als_interrupt_state.state +1;
//		}else{
//			g_als_interrupt_state.event_jiffies64 = new_jiffies;
//			g_als_interrupt_state.state = 0;
//		}
//		if(g_als_interrupt_state.state >= AL3320A_INTERRUPT_TIME_LIMIT){
//			//	change the threshold
//			printk("alp.D : als_interrupt adc(%d) ; lux(%d)!!\n",als_iadc ,als_lux);
//			al3320a_set_spec_threshold(als_iadc);
//		}else{
//			//	change the threshold
//			printk("alp.D : als_interrupt lux(%d)!!\n",als_lux);
//			al3320a_set_threshold(als_lux);
//		}
//	}
#endif

	udelay(500);
	//	clean the interrupt pin
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_FLAG_STATUS, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : als_interrupt clean flag fail !!\n");
		wake_unlock(&light_sensor_data->als_wake_lock);
		return ;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_FLAG_STATUS] = reg_val;
	wake_unlock(&light_sensor_data->als_wake_lock);
//	enable_irq(light_sensor_data->irq);
}
// internal interrupt  function --
#endif

#ifdef AL3320A_POLLIN_MODE
static void al3320a_delay_work_func(struct work_struct *work)
{
	unsigned long delay = msecs_to_jiffies(atomic_read(&light_sensor_data->delay));
	int als_iadc=0,als_lux=0;
//	int ret=0;
//	u8 reg_val=0x00;
	als_iadc = al3320a_get_adc_counter();
	if(als_iadc<0){
		if(ALS_DEBUGMSG) printk("alp.D : adc data fail(%d)!!\n",als_iadc);
		return ;
	}		

	//	change the adc value to lux and reprot lux to hal
//	als_lux = (als_iadc * light_sensor_data->al3320a_adc_to_lux / BASE_GAIN_DENO)*(1000/light_sensor_data->al3320a_calibration);
	als_lux = (als_iadc * light_sensor_data->al3320a_adc_to_lux / light_sensor_data->al3320a_calibration);
	al3320a_check_threshold_val(als_lux);
	if(light_sensor_data->val_change){
		input_report_abs(light_sensor_data->input_dev, ABS_MISC, als_lux);
		input_sync(light_sensor_data->input_dev);
		if(ALS_MESSAGE) printk("alp.D : als_interrupt lux(%d)!!\n",als_lux);
	}
	//	clean the status
	light_sensor_data->val_change = 0;
	
	schedule_delayed_work(&light_sensor_data->delaywork, delay);
}
#endif

#ifdef AL3320A_DELAY_CALIBRATION
static void al3320a_delay_calibration_func(struct work_struct *work)
{
//	unsigned long delay = msecs_to_jiffies(AL3320A_DELAY_RETRYCALITIME);
	int status=0;
	if(ALS_DEBUGMSG) printk("alp.D : al3320a_delay_calibration_func !!\n");
	status = al3320a_read_calibration();
	if(status<0){
		if(ALS_DEBUGMSG) printk("alp.D : read calibration fail , need to Retry !!\n");
//		schedule_delayed_work(&light_sensor_data->delayworkcalibration, delay);
		return ;
	}		
}
#endif


#ifdef AL3320A_FIX_I2C_ERROR
static void al3320a_delay_i2c_retry_func(struct work_struct *work)
{
	int als_iadc=0,als_lux=0;
	int ret = 0;
	u8 reg_val=0x00;

	if(light_sensor_data->retry_i2c_counter>AL3320A_RETRY_I2C_MAXTIME)
		return ;

	als_iadc = al3320a_get_adc_counter();
	if(als_iadc<0){
		light_sensor_data->retry_i2c_counter ++ ;
		printk("alp.D : al3320a_delay_i2c_retry_func fail adc (%d), retry time (%d)!!\n",als_iadc,light_sensor_data->retry_i2c_counter);
		if(atomic_read(&light_sensor_data->delayi2cerror)!=0)
			schedule_delayed_work(&light_sensor_data->delayworki2cerror, msecs_to_jiffies(atomic_read(&light_sensor_data->delayi2cerror)));
		return ;
	}		

	als_lux = (als_iadc * light_sensor_data->al3320a_adc_to_lux / light_sensor_data->al3320a_calibration);
	if(light_sensor_data->lux != als_lux){
		if(ALS_MESSAGE) printk("alp.D : al3320a_delay_i2c_retry_func lux(%d)!!\n",als_lux);
	}else{
		als_lux = als_lux+1 ;
	}
	light_sensor_data->lux = als_lux;
	input_report_abs(light_sensor_data->input_dev, ABS_MISC, als_lux);
	input_sync(light_sensor_data->input_dev);

	//	change the threshold
	al3320a_set_spec_threshold(als_iadc);
	if(light_sensor_data->als_config_log!=0)
		printk("alp.D : retry als_interrupt adc(%d) ; lux(%d)!!\n",als_iadc ,als_lux);

	udelay(500);
	//	clean the interrupt pin
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_FLAG_STATUS, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("alp.D : retry als_interrupt clean flag fail !!\n");
		if(atomic_read(&light_sensor_data->delayi2cerror)!=0)
			schedule_delayed_work(&light_sensor_data->delayworki2cerror, msecs_to_jiffies(atomic_read(&light_sensor_data->delayi2cerror)));
		return ;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_FLAG_STATUS] = reg_val;
	light_sensor_data->retry_i2c_counter = 0;
	atomic_set(&light_sensor_data->delayi2cerror, 0);
	printk("alp.D : retry success !!\n");
}
#endif

static int al3320a_open(struct inode *inode, struct file *file)
{
	if(ALS_DEBUGMSG) printk(KERN_INFO "[%s]\n", __FUNCTION__);
	return nonseekable_open(inode, file);
}

static int al3320a_release(struct inode *inode, struct file *file)
{
	if(ALS_DEBUGMSG) printk(KERN_INFO "[%s]\n", __FUNCTION__);
	return 0;
}


static struct file_operations al3320a_fops = {
	.owner			= 	THIS_MODULE,
	.open			=	al3320a_open,
	.release			=	al3320a_release,
};

static struct miscdevice al3320a_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AL3320A_DEV_NAME,
	.fops = &al3320a_fops,
};

static int al3320a_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	if(ALS_MESSAGE) printk(KERN_INFO "alp.D : al3320a_i2c_probe +\n");

//	if (cadiz_support) {
//		cadiz_power_control(LIGHTSENSOR, true);
//		is_enable_cadiz_power = 1;
//	}

	light_sensor_data = kzalloc(sizeof(struct al3320a_data), GFP_KERNEL);
	if (!light_sensor_data)
	{
		printk(KERN_ERR "alp.D : al3320a allocate al3320a_data failed.\n");
		return -ENOMEM;
	}
	light_sensor_data->client = client;

	// sys init ++
	android_light_kobj = kobject_create_and_add("android_light", NULL);	
	err = sysfs_create_file(android_light_kobj, &dev_attr_state.attr);
	if (err) {
		printk(KERN_ERR "alp.D : attr_light dev_attr_state fail !!\n");
	}
	err = sysfs_create_file(android_light_kobj, &dev_attr_calibration.attr);
	if (err) {
		printk(KERN_ERR "alp.D : attr_light dev_attr_calibration fail !!\n");
	}
	err = sysfs_create_file(android_light_kobj, &dev_attr_lux.attr);
	if (err) {
		printk(KERN_ERR "alp.D : attr_light  dev_attr_lux fail !!\n");
	}
	err = sysfs_create_file(android_light_kobj, &dev_attr_cal_limit.attr);
	if (err) {
		printk(KERN_ERR "alp.D : attr_light dev_attr_cal_limit fail !!\n");
	}
	err = sysfs_create_file(android_light_kobj, &dev_attr_enable.attr);
	if (err) {
		printk(KERN_ERR "alp.D : attr_light dev_attr_enable fail !!\n");
	}
	err = sysfs_create_file(android_light_kobj, &dev_attr_poll_delay.attr);
	if (err) {
		printk(KERN_ERR "alp.D : attr_light dev_attr_poll_delay fail !!\n");
	}
	err = sysfs_create_file(android_light_kobj, &dev_attr_log_state.attr);
	if (err) {
		printk(KERN_ERR "alp.D : attr_light dev_attr_log_state fail !!\n");
	}
	// sys init --

	if(ALS_MESSAGE) printk(KERN_INFO "alp.D : i2c_set_clientdata!!\n");
	i2c_set_clientdata(client, light_sensor_data);
	light_sensor_data->int_gpio = of_get_named_gpio(client->dev.of_node, "dynaimage,light-gpio", 0);	
	light_sensor_data->al3320a_state = 0;	//init state.
	if(ALS_MESSAGE) printk(KERN_INFO "alp.D : int_gpio=%d \n",light_sensor_data->int_gpio);

	if(ALS_MESSAGE) printk(KERN_INFO "alp.D : al3320a_check_device!!\n");
	if( (light_sensor_data!=NULL) && (light_sensor_data->client!=NULL) )
		al3320a_check_device(0);
	else
		printk(KERN_INFO "alp.D : al3320a_check_device error!!\n");

	if(ALS_MESSAGE) printk(KERN_INFO "alp.D : al3320a registered I2C driver!\n");
	err = misc_register(&al3320a_device);
	if (err) {
		if(ALS_MESSAGE) printk(KERN_ERR "alp.D : al3320a_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	
	mutex_init(&light_sensor_data->lock);
	wake_lock_init(&light_sensor_data->als_wake_lock, WAKE_LOCK_SUSPEND, "als_suspend_blocker");

//#ifdef AL3320A_BMMITEST_BYPASS
	// init chip status
	if(ALS_MESSAGE) printk(KERN_INFO "alp.D : al3320a_chip_init!!\n");
	err = al3320a_chip_init();
	if (err)
	{
		printk(KERN_ERR "alp.D : al3320a_chip_init failed.\n");
		goto exit_kfree;
	}
//#endif

	// init input subsystem	
	if(ALS_MESSAGE) printk(KERN_INFO "alp.D : al3320a_input_init!!\n");
	err = al3320a_input_init();
	if (err)
	{
		printk(KERN_ERR "alp.D : al3320a_input_init failed.\n");
		goto exit_kfree;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &al3320a_attr_group);
	if (err)
		goto exit_sysfs_create_group_failed;
	
#ifdef CONFIG_AL3320A_WORK_DEFERRABLE
	if(ALS_MESSAGE) printk(KERN_INFO "alp.D : Do CONFIG_AL3320A_WORK_DEFERRABLE!!\n");
	al3320a_wq = create_singlethread_workqueue("al3320a_wq");
	if (!al3320a_wq)
	{
		printk("alp.D : al3320a Create WorkQueue Failed\n");
		err = -ENOMEM;
		goto exit_kfree;
	}
	INIT_DELAYED_WORK_DEFERRABLE(&al3320a_work, al3320a_irq_work_function);
#endif

#ifdef AL3320A_WORK
	if(ALS_MESSAGE) printk(KERN_INFO "alp.D : Do AL3320A_WORK!!\n");
	INIT_WORK(&light_sensor_data->irq_work, al3320a_irq_work_function);
#endif

#ifdef AL3320A_WORKQUENE
	if(ALS_MESSAGE) printk(KERN_INFO "alp.D : Do AL3320A_WORKQUENE!!\n");
	light_sensor_data->al3320a_wq = create_singlethread_workqueue("al3320a_irq_wq");
	if (!light_sensor_data->al3320a_wq)
	{
		printk("alp.D : al3320a Create WorkQueue Failed\n");
		err = -ENOMEM;
		goto exit_kfree;
	}
#endif

	atomic_set(&light_sensor_data->enable, 0);  // default disable
	atomic_set(&light_sensor_data->pre_enable, 0);  // default disable
	
#ifdef AL3320A_POLLIN_MODE
	atomic_set(&light_sensor_data->delay, AL3320A_POLLIN_TIME);  // 330 ms
	INIT_DELAYED_WORK(&light_sensor_data->delaywork, al3320a_delay_work_func);
	// init poll-in value
	light_sensor_data->val_level = -1;
	light_sensor_data->val_change = 0;
#endif

#ifdef AL3320A_DELAY_CALIBRATION
	atomic_set(&light_sensor_data->delaycalibration, AL3320A_DELAY_CALITIME);  // 12000 ms
	INIT_DELAYED_WORK(&light_sensor_data->delayworkcalibration, al3320a_delay_calibration_func);
#endif


#ifdef AL3320A_FIX_I2C_ERROR
	atomic_set(&light_sensor_data->delayi2cerror, 0);  // default 0 --> not need to retry ; 3000 ms -> retry delay time
	INIT_DELAYED_WORK(&light_sensor_data->delayworki2cerror, al3320a_delay_i2c_retry_func);
#endif

#ifdef AL3320A_INTERRUPT_MODE
	// init interrupt pin	
	if(light_sensor_data->int_gpio>0){
		err = gpio_request(light_sensor_data->int_gpio, "al3320a-irq");
		if(err < 0)
			printk(KERN_ERR "alp.D : al3320a Failed to request GPIO%d (al3320a-irq) error=%d\n", light_sensor_data->int_gpio, err);
		err = gpio_direction_input(light_sensor_data->int_gpio);
		if (err){
			printk(KERN_ERR "alp.D : al3320a Failed to set interrupt direction, error=%d\n", err);
			gpio_free(light_sensor_data->int_gpio);
		}
		client->irq = irq_of_parse_and_map(client->dev.of_node,0);
		light_sensor_data->irq = client->irq ;
		if(ALS_MESSAGE) printk(KERN_INFO "alp.D : al3320a irq: %d\n",light_sensor_data->irq);
		if(light_sensor_data->irq > 0){
//			err =  request_irq(light_sensor_data->irq,als_interrupt,IRQF_TRIGGER_FALLING,"al3320a_interrupt",light_sensor_data);
//			err = request_any_context_irq(light_sensor_data->irq,als_interrupt,IRQF_TRIGGER_LOW,"al3320a_interrupt",light_sensor_data);			
			err = request_threaded_irq(light_sensor_data->irq, NULL,als_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT ,"al3320a_interrupt",NULL);
		  	if (err) {
				dev_err(&client->adapter->dev,"cannot register IRQ %d,err:%d\n",light_sensor_data->irq,err);
			}
		}
	}else{
		printk(KERN_ERR "alp.D : error gpio(%d), disable irq_request\n", light_sensor_data->int_gpio);
	}
#endif

#ifdef AL3320A_FIX_I2C_ERROR
	light_sensor_data->retry_i2c_counter = 0;
#endif

	light_sensor_data->al3320a_state = 1;
	light_sensor_data->al3320a_calibration = AL3320A_DEFAULT_CALIBRATION;
	al3320a_enable_init();
	
#ifdef AL3320A_DELAY_CALIBRATION
	schedule_delayed_work(&light_sensor_data->delayworkcalibration, msecs_to_jiffies(atomic_read(&light_sensor_data->delaycalibration)));
#endif

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
	callback_struct = register_screen_state_notifier(&als3320a_screen_changed_listener);
#endif

	light_sensor_data->als_calibration_limit = 40*BASE_GAIN_DENO/light_sensor_data->al3320a_adc_to_lux;	// limit into 40 lux
#ifdef AL3320A_INTERRUPT_TIMESTAG
//	g_als_interrupt_state.event_jiffies64 = 0;
//	g_als_interrupt_state.state = 0;
#endif
	light_sensor_data->als_config_log = 0;

	if(ALS_MESSAGE) printk(KERN_INFO "alp.D : al3320a_i2c_probe -\n");
	return 0;

exit_misc_device_register_failed:
exit_sysfs_create_group_failed:
exit_kfree:
//	kfree(pdata);
//	kfree(light_sensor_data);
	return err;
}

static int al3320a_i2c_remove(struct i2c_client *client)
{
	struct al3320a_data *data;
	u8 reg_val = 0x00;

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
	unregister_screen_state_notifier(callback_struct);
#endif

	// power down chip at remove
	i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_SYS_CONFIG, reg_val);
	light_sensor_data->al3320a_reg_status[INDEX_ALS_SYS_CONFIG] = reg_val;	
	
	sysfs_remove_group(&client->dev.kobj, &al3320a_attr_group);
	data = i2c_get_clientdata(client);
	al3320a_i2c_client = NULL;

	kfree(data);

	return 0;
}

static int al3320a_suspend(struct i2c_client *client)
{
	printk("alp.D :  al3320a_suspend\n");

	if(atomic_read(&light_sensor_data->enable)==1){
		atomic_set(&light_sensor_data->pre_enable, 1);
#ifdef AL3320A_DELAY_WORKQUENE
		cancel_delayed_work_sync(&light_sensor_data->delaywork);
		if(ALS_DEBUGMSG) printk("alp.D : disable al3220a pollin into suspend!!\n");
#endif
#ifdef AL3320A_FIX_I2C_ERROR
		if(atomic_read(&light_sensor_data->delayi2cerror)!=0)
			cancel_delayed_work_sync(&light_sensor_data->delayworki2cerror);
#endif
		al3320a_enable(0);
	}

	// init - input queue
	input_report_abs(light_sensor_data->input_dev, ABS_MISC, -1);
	input_sync(light_sensor_data->input_dev);

//	if (cadiz_support) {
//		cadiz_power_control(LIGHTSENSOR, false);
//		is_enable_cadiz_power = 0;
//	}

	return 0;
}

static int al3320a_resume(struct i2c_client *client)
{
	printk("alp.D :  al3320a_resume\n");

//	if (cadiz_support && is_enable_cadiz_power == 0) {
//		cadiz_power_control(LIGHTSENSOR, true);
//		is_enable_cadiz_power = 1;
//	}

#ifdef AL3320A_FIX_I2C_ERROR
	light_sensor_data->retry_i2c_counter = 0;
#endif
	
	if(atomic_read(&light_sensor_data->enable)==0){
		if(atomic_read(&light_sensor_data->pre_enable)==1){
			al3320a_enable(1);
			atomic_set(&light_sensor_data->pre_enable, 0);
		}
#ifdef AL3320A_DELAY_WORKQUENE
		schedule_delayed_work(&light_sensor_data->delaywork, msecs_to_jiffies(atomic_read(&light_sensor_data->delay)));
		if(ALS_DEBUGMSG) printk("alp.D : enable al3220a pollin into resume!!\n");
#endif

#ifdef AL3320A_FIX_I2C_ERROR
		if(atomic_read(&light_sensor_data->delayi2cerror)!=0)
			schedule_delayed_work(&light_sensor_data->delayworki2cerror, 0);
#endif
	}
	return 0;
}

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
void als3320a_screen_changed_listener(const int state)
{
	if(state == NOTIFY_WHEN_SCREEN_OFF){
		printk("alp.D :  NOTIFY_WHEN_SCREEN_OFF\n");
		al3320a_suspend(light_sensor_data->client);
	}else if(state == NOTIFY_WHEN_SCREEN_ON){
		printk("alp.D :  NOTIFY_WHEN_SCREEN_ON\n");
		al3320a_resume(light_sensor_data->client);
	}

}
#endif

struct i2c_device_id al3320a_id_table_ER[] = {
	{ "al3320a_er", 0 },
	{}
};

static const struct of_device_id al3320a_of_match_ER[] = {
	{ .compatible = "dynaimage,al3320a_er", },
	{ },
};


//MODULE_DEVICE_TABLE(i2c, al3320a_idtable);

static struct i2c_driver al3320a_i2c_driver_ER = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = AL3320A_DEV_NAME,
		.of_match_table = al3320a_of_match_ER,
	},
#ifndef CONFIG_PM_SCREEN_STATE_NOTIFIER
	.suspend	=	al3320a_suspend,
	.resume	=	al3320a_resume,
#endif
	.id_table	=	al3320a_id_table_ER,	
	.probe	=	al3320a_i2c_probe,
	.remove	=	al3320a_i2c_remove,
};

static const struct i2c_device_id al3320a_id_table[] = {
	{ "al3320a", 0 },
	{ }
};

static const struct of_device_id al3320a_of_match[] = {
	{ .compatible = "dynaimage,al3320a", },
	{ },
};


//MODULE_DEVICE_TABLE(i2c, bma2x2_id);

static struct i2c_driver al3320a_i2c_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = AL3320A_DEV_NAME,
		.of_match_table = al3320a_of_match,
	},
#ifndef CONFIG_PM_SCREEN_STATE_NOTIFIER
	.suspend    = al3320a_suspend,
	.resume     = al3320a_resume,
#endif
	.id_table   = al3320a_id_table,
	.probe      = al3320a_i2c_probe,
	.remove     = al3320a_i2c_remove,
	//.shutdown   = bma2x2_shutdown,
};


static int __init al3320a_init(void)
{
	int ret;
	int hw_id = Read_HW_ID();

	// added by cheng_kao to skip COS/POS
	if(entry_mode != 1)	{	// entry_mode==1  ==> MOS mode
		printk(KERN_ERR "alp.D : Non MOS mode , Disable Light Senosr , mode=%d\n",entry_mode);
		return 0;
	}

//	if (!cadiz_support) {
//		if(ALS_DEBUGMSG) printk(KERN_ERR "alp.D : Cadiz Not Support , Disable Light Senosr Function\n");
//		return 0;
//	}

	if(hw_id == HW_ID_ER)
		ret = i2c_add_driver(&al3320a_i2c_driver_ER);
	else 
		ret = i2c_add_driver(&al3320a_i2c_driver);
	
	if ( ret != 0 ) {
		if(ALS_DEBUGMSG) printk(KERN_ERR "[%s]can not add i2c driver\n",__FUNCTION__);
		return ret;
	}
	return ret;
}

static void __exit al3320a_exit(void)
{
	i2c_del_driver(&al3320a_i2c_driver);
}

MODULE_AUTHOR("Alp kao");
MODULE_DESCRIPTION("Dyna IMAGE al3320a driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(al3320a_init);
module_exit(al3320a_exit);
