/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <asm-generic/unaligned.h>
#include <linux/interrupt.h>
#include <linux/idr.h>

#include "asus_battery.h"
#include "bq27520_battery_core.h"
#include "bq27520_battery_upt_i2c.h"
#include "bq27520_proc_fs.h"

#include <linux/power/bq27520_battery.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/random.h>
#include <linux/proc_fs.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/HWVersion.h>
#include <linux/wakelock.h>
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
extern int Read_HW_ID(void);
static int HW_ID;

static int dev_type;

#define COSLIGHT 0
#define LG 1

#define RETRY_COUNT 3
#define I2C_RETRY_DELAY 5

#define BATT_LGC 0x304C
#define BATT_ATL 0x3041
extern int entry_mode;

struct battery_dev_info bq27520_dev_info;
DEFINE_MUTEX(bq27520_dev_info_mutex);

static struct wake_lock wakelock;
static struct wake_lock polling_wakelock;
#include <linux/spinlock.h>
spinlock_t spinlock_suspend_bq27520;
bool suspend_bq27520 = false;
bool suspend_bq27520_work_not_done = false;

static struct iio_chanel *channel;
//static struct battery_low_config batlow ;
//static struct asus_batgpio_set batlow_gp ;
//static struct workqueue_struct *batlow_wq=NULL;
//static struct work_struct batlow_work;
static struct dev_func bq27520_tbl;

struct bq27520_chip {
    struct i2c_client *client;
    struct bq27520_platform_data *pdata;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend es;
#endif

    struct switch_dev batt_dev;
    int batlow_irq;
    int adc_alert_irq;
    int adc_alert_n_gpio;
};

extern struct battery_info_reply batt_info;
/*
 * i2c specific code
 */
static int bq27520_i2c_txsubcmd(struct i2c_client *client,
                u8 reg, unsigned short subcmd)
{
    struct i2c_msg msg;
    unsigned char data[3];
    int err;

    if (!client || !client->adapter)
        return -ENODEV;

    memset(data, 0, sizeof(data));
    data[0] = reg;
    data[1] = subcmd & 0x00FF;
    data[2] = (subcmd & 0xFF00) >> 8;

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = 3;
    msg.buf = data;

    wake_lock(&wakelock);
    err = i2c_transfer(client->adapter, &msg, 1);
    wake_unlock(&wakelock);

    if (err < 0) {
#ifdef ME372CG_ENG_BUILD
        dev_err(&client->dev, "%s: to addr 0x%02X error with code: %d\n",
            __func__, reg, err);
#endif
         return -EIO;
    }

    return 0;
}

int bq27520_write_i2c(struct i2c_client *client,
                u8 reg, int value, int b_single)
{
    struct i2c_msg msg;
    unsigned char data[3];
    int err;

    if (!client || !client->adapter)
        return -ENODEV;

    memset(data, 0, sizeof(data));
    data[0] = reg;
    data[1] = value& 0x00FF;
    data[2] = (value & 0xFF00) >> 8;

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = b_single ? 2 : 3;
    msg.buf = data;

    wake_lock(&wakelock);
    err = i2c_transfer(client->adapter, &msg, 1);
    wake_unlock(&wakelock);

    if (err < 0) {
#ifdef ME372CG_ENG_BUILD
        dev_err(&client->dev, "%s: to addr 0x%02X error with code: %d\n",
            __func__, reg, err);
#endif
         return -EIO;
    }

    return 0;
}

int bq27520_read_i2c(struct i2c_client *client,
                u8 reg, int *rt_value, int b_single)
{
    struct i2c_msg msg[1];
    unsigned char data[2];
    int err;

    if (!client || !client->adapter)
        return -ENODEV;

    msg->addr = client->addr;
    msg->flags = 0;
    msg->len = 1;
    msg->buf = data;

    data[0] = reg;

    wake_lock(&wakelock);
    err = i2c_transfer(client->adapter, msg, 1);
    wake_unlock(&wakelock);

    if (err >= 0) {
        if (!b_single)
            msg->len = 2;
        else
            msg->len = 1;

        msg->flags = I2C_M_RD;
        wake_lock(&wakelock);
        err = i2c_transfer(client->adapter, msg, 1);
        wake_unlock(&wakelock);
        if (err >= 0) {
            if (!b_single)
                *rt_value = get_unaligned_le16(data);
            else
                *rt_value = data[0];

            return 0;
        }
    }
#ifdef ME372CG_ENG_BUILD
    dev_err(&client->dev, "%s: from addr 0x%02X error with code: %d\n",
        __func__, reg, err);
#endif
    return err;
}
#if defined(CONFIG_Z300C) || defined(CONFIG_Z300CG)
static ssize_t batt_switch_name(struct switch_dev *sdev, char *buf)
{
    /* firmware_version + firmware config version + chemical id + battery id */

    dev_status_t s;
    int fw_cell = BATT_ATL;
    int DF_ver;
    const char* FAIL = "0xFFFF";
    DF_ver = bq27520_read_DF_version();

    mutex_lock(&bq27520_dev_info_mutex);
    s = bq27520_dev_info.status;
    mutex_unlock(&bq27520_dev_info_mutex);

    return sprintf(buf, "%04X-%04X-0\n", DF_ver, fw_cell);

    /*mutex_lock(&bq27520_dev_info_mutex);
    s = bq27520_dev_info.status;
    mutex_unlock(&bq27520_dev_info_mutex);

    if (s == DEV_INIT_OK)
        return sprintf(buf, "%s-%s-%s-%s\n",
            batt_info.serial_number,
            batt_info.fw_cfg_version,
            batt_info.chemical_id,
            batt_info.manufacturer);*/

    //return sprintf(buf, "%s\n", FAIL);
}
#elif defined(CONFIG_Z380C)
static ssize_t batt_switch_name(struct switch_dev *sdev, char *buf)
{
    dev_status_t s;
    int fw_cell = BATT_ATL;
    int DF_ver;
    const char* FAIL = "0xFFFF";
    DF_ver = bq27520_read_DF_version();
    return sprintf(buf, "%04X-%04X-0\n", DF_ver, fw_cell);
}
#endif
static inline int bq27520_read(struct i2c_client *client,
                u8 reg, int *rt_value, int b_single)
{
    return bq27520_read_i2c(client, reg, rt_value, b_single);
}

static int bq27520_cntl_cmd(struct i2c_client *client,
                u16 sub_cmd)
{
    return bq27520_i2c_txsubcmd(client, BQ27520_REG_CNTL, sub_cmd);
}

int bq27520_send_subcmd(struct i2c_client *client, int *rt_value, u16 sub_cmd)
{
    int ret, tmp_buf = 0;
    int retry_count = RETRY_COUNT;

    do
    {
    ret = bq27520_cntl_cmd(client, sub_cmd);
    if (ret) {
        dev_err(&client->dev, "Send subcommand 0x%04X error.\n", sub_cmd);
        retry_count--;
        msleep(I2C_RETRY_DELAY);
    }
    } while (ret && retry_count > 0);

    if (!rt_value) return ret;

    retry_count = RETRY_COUNT;
    udelay(800);
    do
    {
    /* need read data to rt_value */
    //ret = bq27520_read(client, BQ27520_REG_CNTL, &tmp_buf, 0);
	ret = bq27520_read(client, 0x40, &tmp_buf, 0);
    if (ret)
    {
        dev_err(&client->dev, "Error!! %s subcommand %04X\n",
                         __func__, sub_cmd);
        retry_count--;
        msleep(I2C_RETRY_DELAY);
    }
    } while (ret && retry_count > 0);
    *rt_value = tmp_buf;
    return ret;
}

int bq27520_cmp_i2c(int reg_off, int value)
{
    int retry = 3;
    int val=0;
    int ret=0;
    struct i2c_client *i2c = NULL;

    mutex_lock(&bq27520_dev_info_mutex);
    i2c = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

    BAT_DBG_E("[%s] enter \n", __func__);

    while (retry--) {
        ret = bq27520_read_i2c(i2c, reg_off, &val, 1);
        if (ret < 0) continue;

        break;
    };
    if (!retry && ret < 0) {
        return ret;
    }

    return val == value ? PROC_TRUE : PROC_FALSE;
}

/*int TIgauge_i2c_write(struct i2c_client *client, u8 addr, int len, void *data)
{
    int i=0;
    int status=0;
    u8 buf[len + 1];

    if (!client || !client->adapter)
        return -ENODEV;

    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = len + 1,
            .buf = buf,
        },
    };
    int retries = 6;

    buf[0] = addr;
    memcpy(buf + 1, data, len);

    do {
        status = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
        if ((status < 0) && (i < retries)) {
            msleep(5);
            dev_err(&client->dev, "%s: retry %d times\n", __func__, i);
            i++;
        }
    } while ((status < 0) && (i < retries));

    if (status < 0)
        dev_err(&client->dev, "%s: i2c write error %d\n", __func__, status);

    return status;
}*/

/*int TIgauge_i2c_read(struct i2c_client *client, u8 addr, int len, void *data)
{
    int i=0;
    int retries=6;
    int status=0;

    if (!client || !client->adapter)
        return -ENODEV;

    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &addr,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = len,
            .buf = data,
        },
    };

    do {
        status = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
        if ((status < 0) && (i < retries)) {
            msleep(5);
            dev_err(&client->dev, "%s: retry %d times\n", __func__, i);
            i++;
        }
    } while ((status < 0) && (i < retries));

    if (status < 0)
        dev_err(&client->dev, "%s: i2c read error %d\n", __func__, status);

    return status;
}
*/
void TIgauge_LockStep(void)
{
    int status;
    uint8_t i2cdata[32] = {0};

    struct i2c_client *i2c = NULL;

    mutex_lock(&bq27520_dev_info_mutex);
    i2c = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

    dev_info(&i2c->dev, "%s enter\n", __func__);

    i2cdata[0] = 0x20;
    i2cdata[1] = 0x00;

    //status = TIgauge_i2c_write(i2c, 0x00, 2, i2cdata);

    if (status < 0)
        dev_err(&i2c->dev, "%s: i2c write error %d\n", __func__, status);
}

static ssize_t show_polling(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    u32 time;
    time = asus_update_all();
    return sprintf(buf, "polling!!\n");
}
static DEVICE_ATTR(polling, S_IRUGO, show_polling, NULL);

static char gbuffer[64];
/* Generate UUID by invoking kernel library */
static void generate_key(void)
{
    char sysctl_bootid[16];

    generate_random_uuid(sysctl_bootid);
    sprintf(gbuffer, "%pU", sysctl_bootid);
}

/* Acquire the UUID */
static ssize_t get_updateks(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    generate_key();
    return sprintf(buf, "%s\n", gbuffer);
}

static DEVICE_ATTR(updateks, S_IRUGO, get_updateks, NULL);
static struct attribute *dev_attrs[] = {
    &dev_attr_updateks.attr,
    &dev_attr_polling.attr,
    NULL,
};
static struct attribute_group dev_attr_grp = {
    .attrs = dev_attrs,
};
#if 0
#ifdef CONFIG_PROC_FS

static bool flag_create_bdge = false;

static int bq27520_proc_bridge_read(char *page, char **start, off_t off,
                            int count, int *eof, void *date)
{
    int len;

    if (bq27520_is_rom_mode()) {
        len = sprintf(page, "7");
    } else {
        len = sprintf(page, "8");
    }

    return len;
}

#ifndef CONFIG_NEW_PROC_FS
int bq27520_proc_fs_update_bridge(void)
{
    struct proc_dir_entry *entry=NULL;

    /* not create again */
    if (flag_create_bdge)
        return 0;

    entry = create_proc_entry("ubridge", 0666, NULL);
    if (!entry) {
        BAT_DBG_E("Unable to create ubridge\n");
        return -EINVAL;
    }
    entry->read_proc = bq27520_proc_bridge_read;
    entry->write_proc = bq27520_proc_bridge_write;

    /* lock on the flag */
    flag_create_bdge = true;

    return 0;
}
#else
static ssize_t nbq27520_proc_bridge_write(struct file *file,
    const char __user *buf, size_t count, loff_t * ppos)
{
    return count;
}
static int nbq27520_proc_bridge_read(struct seq_file *m, void *v)
{
    if (bq27520_is_rom_mode()) {
        seq_printf(m, "%s\n", "7");
    } else {
        seq_printf(m, "%s\n", "8");
    }

    return 0;
}
static int nbq27520_proc_bridge_open(struct inode *inode, struct file *file)
{
	return single_open(file, nbq27520_proc_bridge_read, NULL);
}
int bq27520_proc_fs_update_bridge(void)
{
    static const struct file_operations proc_fs_update_bridge_fops = {
        .owner = THIS_MODULE,
        .open = nbq27520_proc_bridge_open,
        .read = seq_read,
        .write = nbq27520_proc_bridge_write,
        .llseek = seq_lseek,
        .release = single_release,
    };
    struct proc_dir_entry *entry=NULL;

    entry = proc_create("ubridge", 0666, NULL,
        &proc_fs_update_bridge_fops);
    if (!entry) {
        BAT_DBG_E("Unable to create ubridge\n");
        return -EINVAL;
    }

    return 0;
}
#endif

static int bq27520_proc_read(char *page, char **start, off_t off,
                            int count, int *eof, void *date)
{
    int ret;

    if (!gbuffer)
        return count;

    /* print the key to screen for debugging */
    ret = sprintf(page, "%s\n", gbuffer);

    /* re-generate the key to clean the memory */
    generate_key();

    return ret;
}

static int bq27520_proc_write(struct file *file, const char *buffer,
                            unsigned long count, void *data)
{
    char proc_buf[64];

    if (count > sizeof(gbuffer)) {
        BAT_DBG("%s: data error\n", __func__);
        return -EINVAL;
    }

    if (copy_from_user(proc_buf, buffer, count)) {
        BAT_DBG("%s: read data from user space error\n", __func__);
        return -EFAULT;
    }

    if (!memcmp(proc_buf, gbuffer, 36)) {
        BAT_DBG_E("%s: EQ\n", __func__);

        /* cancel all the I2C polling work
           to avoid affect the update
        */
        asus_cancel_work();

        /* create update bridge */
        bq27520_proc_fs_update_bridge();

    }
    else {
        BAT_DBG_E("%s: NOT EQ\n", __func__);

        /* re-generate the key to clean memory to avoid
           "brute-force attack". we do not allow someone
           who using "repetitive try-error" to find out
           the correct one.
        */
        generate_key();
    }

    return count;
}

#ifndef CONFIG_NEW_PROC_FS
int bq27520_proc_fs_update_latch(void)
{
    struct proc_dir_entry *entry=NULL;

    entry = create_proc_entry("twinsheadeddragon", 0666, NULL);
    if (!entry) {
        BAT_DBG_E("Unable to create twinsheadeddragon\n");
        return -EINVAL;
    }
    entry->read_proc = bq27520_proc_read;
    entry->write_proc = bq27520_proc_write;

    return 0;
}
#else
static ssize_t nbq27520_proc_write(struct file *file,
    const char __user *buf, size_t count, loff_t * ppos)
{
    char proc_buf[64];

    if (count > sizeof(gbuffer)) {
        BAT_DBG("%s: data error\n", __func__);
        return -EINVAL;
    }

    if (copy_from_user(proc_buf, buf, count)) {
        BAT_DBG("%s: read data from user space error\n", __func__);
        return -EFAULT;
    }

    if (!memcmp(proc_buf, gbuffer, 36)) {
        BAT_DBG_E("%s: EQ\n", __func__);

        /* cancel all the I2C polling work
           to avoid affect the update
        */
        asus_cancel_work();

        /* create update bridge */
        bq27520_proc_fs_update_bridge();

    }
    else {
        BAT_DBG_E("%s: NOT EQ\n", __func__);

        /* re-generate the key to clean memory to avoid
           "brute-force attack". we do not allow someone
           who using "repetitive try-error" to find out
           the correct one.
        */
        generate_key();
    }

    return count;
}
static int nbq27520_proc_read(struct seq_file *m, void *v)
{
    if (!gbuffer)
        return 0;

    /* print the key to screen for debugging */
    seq_printf(m, "%s\n", gbuffer);

    /* re-generate the key to clean the memory */
    generate_key();

    return 0;
}
static int nbq27520_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nbq27520_proc_read, NULL);
}
int bq27520_proc_fs_update_latch(void)
{
    static const struct file_operations proc_fs_update_latch_fops = {
        .owner = THIS_MODULE,
        .open = nbq27520_proc_open,
        .read = seq_read,
        .write = nbq27520_proc_write,
        .llseek = seq_lseek,
        .release = single_release,
    };
    struct proc_dir_entry *entry=NULL;

    entry = proc_create("twinheadeddragon", 0666, NULL,
        &proc_fs_update_latch_fops);
    if (!entry) {
        BAT_DBG_E("Unable to create twinheadeddragon\n");
        return -EINVAL;
    }

    return 0;
}
#endif
#else
int bq27520_proc_fs_update_bridge(void) { return 0; }
int bq27520_proc_fs_update_latch(void) { return 0; }
#endif
#endif
int bq27520_asus_battery_dev_read_cycle_count(void)
{
    int ret;
    int cc=0;
    struct i2c_client *client = NULL;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_read_i2c(client, BQ27520_REG_CC, &cc, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read Cycle Count(CC) error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    mutex_unlock(&bq27520_dev_info_mutex);

    return cc;
}

#if (BATT_PERCENT_OPT)
 static int bq27520_battery_rsoc(struct i2c_client *client)
 {
    int ret, rsoc = 0;
    int bsoc = 0;
    ret = bq27520_read_i2c(client, BQ27520_REG_BSOC, &bsoc, 0);
	if (ret)
        return ret;
    return bsoc;
}

static int bq27520_battery_TIsoc(struct i2c_client *client)
{
    int ret, rsoc = 0;
     ret = bq27520_read_i2c(client, BQ27520_REG_SOC, &rsoc, 0);
	if (ret)
         return ret;

     return rsoc;
 }

 #else
/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27520_battery_rsoc(struct i2c_client *client)
{
    int ret, rsoc = 0;

    ret = bq27520_read_i2c(client, BQ27520_REG_SOC, &rsoc, 0);
    if (ret)
        return ret;

    return rsoc;
}
#endif

int bq27520_asus_battery_dev_read_percentage(void)
{
    struct i2c_client *client = NULL;
    int ret;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_battery_rsoc(client);
    if (ret < 0) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev,"Reading battery percentage error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    mutex_unlock(&bq27520_dev_info_mutex);

    return ret;
}

#if (BATT_PERCENT_OPT)
int bq27520_asus_battery_dev_read_TIpercentage(void)
{
    struct i2c_client *client = NULL;
    int ret;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_battery_TIsoc(client);
    if (ret < 0) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev,"Reading battery percentage error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    mutex_unlock(&bq27520_dev_info_mutex);

    return ret;
}
#endif

int bq27520_asus_battery_dev_read_current(void)
{
    int ret;
    int curr;
    struct i2c_client *client = NULL;

    curr=0;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_read_i2c(client, BQ27520_REG_AI, &curr, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read current error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    if (curr & BIT15)
        curr |= 0xFFFF0000; //neg. sign extend.

    curr += 0x10000; //add a base to be positive number.

    mutex_unlock(&bq27520_dev_info_mutex);

    return curr;
}

int bq27520_asus_battery_dev_read_volt(void)
{
    int ret;
    int volt=0;
    struct i2c_client *client = NULL;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_read_i2c(client, BQ27520_REG_VOLT, &volt, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read voltage error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    mutex_unlock(&bq27520_dev_info_mutex);

    return volt;
}

#define bq27520_asus_battery_dev_read_av_energy NULL

#if defined(CONFIG_Z300C) || defined(CONFIG_Z300CG)
int bq27520_asus_battery_dev_read_temp(void)
{
    int ret;
    struct i2c_client *client = NULL;
    int temp=0;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_read(client, BQ27520_REG_TEMP, &temp, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read temperature error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    //tenths of K -> tenths of degree Celsius
    temp -= 2730;
    mutex_unlock(&bq27520_dev_info_mutex);

    return temp;
}
#elif defined(CONFIG_Z380C)
int bq27520_asus_battery_dev_read_temp(void)
{
    int temp = 0;
    int ret;
    struct i2c_client *client = NULL;

    if( HW_ID == HW_ID_ER || HW_ID == HW_ID_PR )
    {
        BAT_DBG("*****OLD BATTERY*****\n");
        mutex_lock(&bq27520_dev_info_mutex);
        client = bq27520_dev_info.i2c;

        ret = bq27520_read(client, BQ27520_REG_TEMP, &temp, 0);
        if (ret) {
            mutex_unlock(&bq27520_dev_info_mutex);
            dev_err(&client->dev, "Read temperature error %d.\n", ret);
            return ERROR_CODE_I2C_FAILURE;
        }
    //tenths of K -> tenths of degree Celsius
    temp -= 2730;
    mutex_unlock(&bq27520_dev_info_mutex);
    }

    else
    {
        BAT_DBG("*****NEW BATTERY*****\n");
        ret = iio_read_channel_processed(channel, &temp);
        temp = temp*10;
    }
    return temp;
}
#endif


int bq27520_asus_battery_dev_nominal_available_capacity(void)
{
    int ret;
    struct i2c_client *client = NULL;
    int mAhr=0;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_read_i2c(client, BQ27520_REG_NAC, &mAhr, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read NAC error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    mutex_unlock(&bq27520_dev_info_mutex);
    return mAhr;
}

int bq27520_asus_battery_dev_read_remaining_capacity(void)
{
    int ret;
    struct i2c_client *client = NULL;
    int mAhr=0;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_read_i2c(client, BQ27520_REG_RM, &mAhr, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read remaining capacity error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    mutex_unlock(&bq27520_dev_info_mutex);
    return mAhr;
}

int bq27520_asus_battery_dev_read_full_charge_capacity(void)
{
    int ret;
    struct i2c_client *client = NULL;
    int mAhr=0;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_read_i2c(client, BQ27520_REG_FCC, &mAhr, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read full charge capacity error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    mutex_unlock(&bq27520_dev_info_mutex);
    return mAhr;
}

int bq27520_asus_battery_dev_read_fw_version(void)
{
    struct i2c_client *client = NULL;
    int fw_ver=0;
    int ret;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_send_subcmd(client, &fw_ver, BQ27520_SUBCMD_FW_VER);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read FW_VERSION error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    mutex_unlock(&bq27520_dev_info_mutex);
    return fw_ver;
}

int bq27520_asus_battery_dev_read_chemical_id(void)
{
    struct i2c_client *client = NULL;
    int chem_id=0;
    int ret;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_send_subcmd(client, &chem_id, BQ27520_SUBCMD_CHEM_ID);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read chemical ID error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    mutex_unlock(&bq27520_dev_info_mutex);
    return chem_id;
}

int bq27520_asus_battery_dev_read_fw_cfg_version(void)
{
    struct i2c_client *client = NULL;
    int fw_cfg_ver=0;
    int ret;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_write_i2c(client, 0x3F, 0x01, 1);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Get fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    udelay(800);
    ret = bq27520_read_i2c(client, 0x40, &fw_cfg_ver, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    mutex_unlock(&bq27520_dev_info_mutex);
    return fw_cfg_ver;
}

static int bq27520_bat_i2c_remove(struct i2c_client *i2c)
{
    dev_info(&i2c->dev, "%s\n", __func__);
    return 0;
}

/*static int bq27520_bat_i2c_shutdown(struct i2c_client *i2c)
{
    dev_info(&i2c->dev, "%s\n", __func__);
#if 0
    batlow_disable_irq(&batlow_gp, &batlow);
    cancel_work_sync(&batlow_work);
#endif
    asus_battery_exit();
    return 0;
}*/

static void config_suspend_irq(int irq)
{
    if (irq != -1) {
        disable_irq(irq);
        enable_irq_wake(irq);
    }
}

static void config_resume_irq(int irq)
{
    if (irq != -1) {
        enable_irq(irq);
        disable_irq_wake(irq);
    }
}

#ifdef CONFIG_PM
static int bq27520_bat_i2c_suspend(struct device *dev)
{
    struct bq27520_chip *chip = dev_get_drvdata(dev);
    unsigned long flags;

    spin_lock_irqsave(&spinlock_suspend_bq27520, flags);
    suspend_bq27520 = true;
    spin_unlock_irqrestore(&spinlock_suspend_bq27520, flags);

    dev_warn(&chip->client->dev, "%s\n", __func__);

    config_suspend_irq(chip->batlow_irq);
    config_suspend_irq(chip->adc_alert_irq);
	//asus_cancel_work();

    return 0;
}

static int bq27520_bat_i2c_resume(struct device *dev)
{
    struct bq27520_chip *chip = dev_get_drvdata(dev);
    unsigned long flags;

    dev_warn(&chip->client->dev, "%s\n", __func__);

    spin_lock_irqsave(&spinlock_suspend_bq27520, flags);
    suspend_bq27520 = false;
    if (suspend_bq27520_work_not_done) {
        wake_lock_timeout(&polling_wakelock, msecs_to_jiffies(500));
        asus_start_polling_work();
    }
    spin_unlock_irqrestore(&spinlock_suspend_bq27520, flags);

#if (BATT_PERCENT_OPT)
    ac_in_screen_off();
#endif
    //asus_start_polling_work();
    config_resume_irq(chip->batlow_irq);
    config_resume_irq(chip->adc_alert_irq);

    return 0;
}

#else
#define bq27520_bat_i2c_suspend NULL
#define bq27520_bat_i2c_resume  NULL
#endif

static int bq27520_chip_config(struct i2c_client *client)
{
    int flags = 0, ret = 0;

    ret = bq27520_send_subcmd(client, &flags, BQ27520_SUBCMD_CTNL_STATUS);
    if (ret)
        return ret;

    dev_info(&client->dev, "bq27520 flags: 0x%04X\n", flags);
    return 0;
}

static int bq27520_hw_config(struct i2c_client *client)
{
    int ret = 0;

    ret = bq27520_chip_config(client);
    if (ret)
        dev_err(&client->dev, "fail to config Bq27520 ret = %d\n", ret);

    return ret;
}

struct info_entry
{
    char name[30];
    u16 cmd;
};

struct info_entry dump_subcmd_info_tbl[] =
{
    {"control status", BQ27520_SUBCMD_CTNL_STATUS},
    {"device type", BQ27520_SUBCMD_DEVICE_TYPE},
    {"firmware version", BQ27520_SUBCMD_FW_VER},
    {"chemical id", BQ27520_SUBCMD_CHEM_ID},
};

struct info_entry dump_info_tbl[] =
{
    {"Temperature", BQ27520_REG_TEMP},
    {"Voltage", BQ27520_REG_VOLT},
    {"Flags", BQ27520_REG_FLAGS},
    {"RemainingCapacity", BQ27520_REG_RM},
    {"AverageCurrent", BQ27520_REG_AI},
    //{"OperationConfiguration", BQ27520_REG_OC}
};

static char _chemical_id[5];
static char _fw_cfg_version[5];

static char bq27520_firmware_version[5];
static int bq27520_get_firmware_version(struct i2c_client *client)
{
    int ret;
    int tmp_buf=0;
    int retry_count = RETRY_COUNT;

    do
    {
        ret = bq27520_cntl_cmd(client, BQ27520_SUBCMD_FW_VER);
        if (ret) {
            dev_err(&client->dev, "Send subcommand 0x%04X error.\n",
                BQ27520_SUBCMD_FW_VER);
            retry_count--;
            msleep(I2C_RETRY_DELAY);
        }
    } while (ret && retry_count > 0);

    retry_count = RETRY_COUNT;
    mdelay(500);

    do
    {
        ret = bq27520_read(client, BQ27520_REG_CNTL, &tmp_buf, 0);
        if (ret) {
            dev_err(&client->dev, "Error!! %s subcommand %04X\n",
                          __func__, BQ27520_REG_CNTL);
            retry_count--;
            msleep(I2C_RETRY_DELAY);
        }
    } while (ret && retry_count > 0);

    ret = sprintf(bq27520_firmware_version, "%04x", tmp_buf);
    //batt_info.serial_number = bq27520_firmware_version;

    return 0;
}

static void bq27520_dump_info(struct i2c_client *client)
{
    u32 i;

    for (i=0; i<sizeof(dump_subcmd_info_tbl)/sizeof(struct info_entry); i++) {
        int tmp_buf=0, ret=0;

        ret = bq27520_send_subcmd(client,
                &tmp_buf, dump_subcmd_info_tbl[i].cmd);
        if (ret) continue;

        dev_info(&client->dev, "%s, 0x%04X\n",
            dump_subcmd_info_tbl[i].name, tmp_buf);
    }

    for (i=0; i<sizeof(dump_info_tbl)/sizeof(struct info_entry); i++) {
        int tmp_buf=0, ret=0;

        ret = bq27520_read(client,
                dump_info_tbl[i].cmd, &tmp_buf, 0);
        if (ret) continue;

        dev_info(&client->dev, "%s, 0x%04X\n",
            dump_info_tbl[i].name, tmp_buf);
    }
}

static int bq27520_thermistor_check(struct i2c_client *client)
{
    int temperature;

    dev_warn(&client->dev, "%s enter\n", __func__);

    /* ME371MG, ME302C, ME372CG: refer to Gauge IC spec.
    Detect the thermal sensor in Battery Pack to check if normal.
    If the battery temperature is too low, we can almost confirm
    that there's somthing wrong with it or thermal sensor line
    is broken. Check the battery connector and the line... */

    temperature = bq27520_asus_battery_dev_read_temp();
    if (temperature == ERROR_CODE_I2C_FAILURE) {
        dev_err(&client->dev,
            "%s: read temperature error due to i2c error\n", __func__);
        return ERROR_CODE_I2C_FAILURE;
    }

    if (temperature < -350) {
        dev_err(&client->dev, "%s fail: temperature(%d) < -350(0.1C) \n",
            __func__, temperature);
        return -EINVAL;
    }

    return 0;
}
#if 0
static irqreturn_t bq27520_batlow_interrupt(int irq, void *data)
{
    irqreturn_t ret = IRQ_NONE;
    struct bq27520_chip *chip = data;

    pm_runtime_get_sync(&chip->client->dev);

    dev_warn(&chip->client->dev, "%s enter\n", __func__);

    /* Charger IC registers dump */
    #ifdef CONFIG_ME372CG_BATTERY_SMB345
    smb345_dump_registers(NULL);
    #endif

    if (!flag_create_bdge) asus_queue_update_all();

    pm_runtime_put_sync(&chip->client->dev);

    ret = IRQ_HANDLED;
    return ret;
}

static irqreturn_t bq27520_adc_alert_interrupt(int irq, void *data)
{
    irqreturn_t ret = IRQ_NONE;
    struct bq27520_chip *chip = data;

    pm_runtime_get_sync(&chip->client->dev);

    dev_warn(&chip->client->dev, "%s enter\n", __func__);

    /* Charger IC registers dump */
    #ifdef CONFIG_ME372CG_BATTERY_SMB345
    smb345_dump_registers(NULL);
    #endif

    if (!flag_create_bdge) asus_queue_update_all();

    pm_runtime_put_sync(&chip->client->dev);

    ret = IRQ_HANDLED;
    return ret;
}

static int disable_adc_alert(struct bq27520_chip *chip)
{
    struct bq27520_platform_data *pdata = chip->pdata;
    int ret;

    dev_warn(&chip->client->dev, "%s enter\n", __func__);

    ret = gpio_request_one(pdata->adc_alert,
                            GPIOF_DIR_OUT | GPIOF_INIT_HIGH,
                            "bq27520_adc_alert");
    if (ret < 0)
        dev_err(&chip->client->dev,
            "%s: ERROR: fail to disable adc_alert gpio\n", __func__);

    return ret;
}

#ifdef CONFIG_ADC_ALERT_GPIO_AS_WAKEUP
static int bq27520_adc_alert_irq_init(struct bq27520_chip *chip)
{
    struct bq27520_platform_data *pdata = chip->pdata;
    int ret;
    int adc_alert_irq;

    dev_warn(&chip->client->dev, "%s enter\n", __func__);

    if (gpio_get_value(chip->pdata->adc_alert))
        dev_info(&chip->client->dev, ">>> adc_alert(%d):  HIGH <<<\n",
            chip->pdata->adc_alert);
    else
        dev_info(&chip->client->dev, ">>> adc_alert(%d):  LOW <<<\n",
            chip->pdata->adc_alert);

    ret = gpio_request_one(pdata->adc_alert,
                    GPIOF_DIR_IN,
                    "bq27520_adc_alert");
    if (ret < 0)
        goto fail;

    adc_alert_irq = gpio_to_irq(pdata->adc_alert);
    ret = request_threaded_irq(adc_alert_irq, NULL,
                    bq27520_adc_alert_interrupt,
                    IRQF_PERCPU | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
                    IRQF_ONESHOT |
                    IRQF_TRIGGER_FALLING,
                    chip->client->name,
                    chip);
    if (ret < 0)
        goto fail_gpio;

    chip->adc_alert_irq = adc_alert_irq;
    if (chip->adc_alert_irq == -1)
        goto fail_gpio;

    return 0;

fail_irq:
    free_irq(adc_alert_irq, chip);
fail_gpio:
    gpio_free(pdata->adc_alert);
fail:
    chip->adc_alert_irq == -1;
    return ret;
}
#else
#if defined(CONFIG_ME302C)
static int bq27520_adc_alert_irq_init(struct bq27520_chip *chip)
{ return disable_adc_alert(chip); }
#else
static int bq27520_adc_alert_irq_init(struct bq27520_chip *chip)
{ return 0; }
#endif
#endif

#ifdef CONFIG_BATT_LOW_GPIO_AS_WAKEUP
static int bq27520_batlow_irq_init(struct bq27520_chip *chip)
{
    struct bq27520_platform_data *pdata = chip->pdata;
    int ret;
    int batlow_irq;

    dev_warn(&chip->client->dev, "%s enter\n", __func__);

    if (gpio_get_value(chip->pdata->low_bat))
        dev_info(&chip->client->dev, ">>> low_bat(%d):  HIGH <<<\n",
            chip->pdata->low_bat);
    else
        dev_info(&chip->client->dev, ">>> low_bat(%d):  LOW <<<\n",
            chip->pdata->low_bat);

    ret = gpio_request_one(pdata->low_bat,
                    GPIOF_DIR_IN,
                    "bq27520_low_bat");
    if (ret < 0)
        goto fail;

    batlow_irq = gpio_to_irq(pdata->low_bat);
    ret = request_threaded_irq(batlow_irq, NULL,
                    bq27520_batlow_interrupt,
                    IRQF_PERCPU | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
                    IRQF_ONESHOT |
                    IRQF_TRIGGER_FALLING,
                    chip->client->name,
                    chip);
    if (ret < 0)
        goto fail_gpio;


    chip->batlow_irq = batlow_irq;
    if (chip->batlow_irq == -1)
        goto fail_gpio;

    return 0;

fail_irq:
    free_irq(batlow_irq, chip);
fail_gpio:
    gpio_free(pdata->low_bat);
fail:
    chip->batlow_irq == -1;
    return ret;
}
#else
static int bq27520_batlow_irq_init(struct bq27520_chip *chip)
{ return 0; }
#endif

#ifndef CONFIG_A450CG
static int bq27520_irq_init(struct bq27520_chip *chip)
{
    struct bq27520_platform_data *pdata = chip->pdata;
    int ret;

    dev_warn(&chip->client->dev, "%s enter\n", __func__);

    ret = bq27520_batlow_irq_init(chip);
    if (ret < 0)
        return ret;

    ret = bq27520_adc_alert_irq_init(chip);
    if (ret < 0)
        return ret;

    return 0;
}
#else
static int bq27520_irq_init(struct bq27520_chip *chip) { return 0; }
#endif

static void batlow_work_func(struct work_struct *work)
{
    int ret;

    BAT_DBG("%s enter\n", __func__);

    ret = asus_battery_low_event();
    if (ret) {
        BAT_DBG("%s: battery low event error. %d \n", __func__, ret);
    }
}
#endif
static irqreturn_t adc_alert_n_interrupt(int irq, void *data)
{
    struct bq27520_chip *chip = data;
    dev_warn(&chip->client->dev, "%s enter\n", __func__);

    smb345_dump_registers(NULL);
    asus_queue_update_all();

    return IRQ_HANDLED;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void bq27520_early_suspend(struct early_suspend *h)
{
    struct bq27520_chip *chip = container_of(h, struct bq27520_chip, es);

    dev_info(&chip->client->dev, "enter %s\n", __func__);
}

static void bq27520_late_resume(struct early_suspend *h)
{
    struct bq27520_chip *chip = container_of(h, struct bq27520_chip, es);

    dev_info(&chip->client->dev, "enter %s\n", __func__);
    if (!flag_create_bdge) asus_queue_update_all();
}

static void bq27520_config_earlysuspend(struct bq27520_chip *chip)
{
    chip->es.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 10;
    chip->es.suspend = bq27520_early_suspend;
    chip->es.resume = bq27520_late_resume;
    register_early_suspend(&chip->es);
}
#else
#define bq27520_early_suspend NULL
#define bq27520_late_resume NULL
static void bq27520_config_earlysuspend(struct bq27520_chip *chip) { return; }
#endif

int bq27520_batt_current_sel_type(void)
{
    batt_info.manufacturer = LG;
    return TYPE_LG;
}

int bq27520_batt_fw_sel_type(void)
{
    int ret_val = 0;
    int cell_type = 0;

    ret_val = bq27520_asus_battery_dev_read_chemical_id();
    if (ret_val < 0) {
        BAT_DBG_E("[%s] Fail to get chemical_id\n", __func__);
        return -EINVAL;
    }

    if (ret_val == FW_CELL_TYPE_LG) {
        cell_type = TYPE_LG;

    } else if (ret_val == FW_CELL_TYPE_COS_LIGHT) {
        cell_type = TYPE_COS_LIGHT;

    } else {
        BAT_DBG_E("[%s] wrong chemical_id 0x%04X\n", __func__, ret_val);
        return -EINVAL;
    }

    return cell_type;
}
#if defined(CONFIG_Z300C) || defined(CONFIG_Z300CG)
int bq27520_read_fw_cell(void)
{
    int ret;
	int retry_count = RETRY_COUNT;
    int fw_cell=0;
    struct i2c_client *client = NULL;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    do
    {
    ret = bq27520_cntl_cmd(client, 0x004A);
    if (ret) {
        dev_err(&client->dev, "Send subcommand 0x%04X error.\n", 0x004A);
        retry_count--;
        msleep(I2C_RETRY_DELAY);
    }
    } while (ret && retry_count > 0);

    retry_count = RETRY_COUNT;
    udelay(800);
    do
    {

    ret = bq27520_read(client, 0x44, &fw_cell, 0);

    if (ret)
    {
        dev_err(&client->dev, "Error!! %s subcommand %04X\n",
                         __func__, 0x004A);
        retry_count--;
        msleep(I2C_RETRY_DELAY);
    }
    } while (ret && retry_count > 0);

    mutex_unlock(&bq27520_dev_info_mutex);

    return fw_cell;
}
#elif defined(CONFIG_Z380C)
int bq27520_read_fw_cell(void)
{
    int ret;
    int retry_count = RETRY_COUNT;
    int fw_cell=0;
    struct i2c_client *client = NULL;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_write_i2c(client, 0x00, 0x08, 1);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Get fw cfg version error %d - 2.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    msleep(50);

    ret = bq27520_read_i2c(client, 0x00, &fw_cell, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read fw cfg version error %d - 3.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    mutex_unlock(&bq27520_dev_info_mutex);

    return fw_cell;
}
#endif

#if defined (CONFIG_Z300C) || (CONFIG_Z300CG)
int bq27520_read_DF_version(void)
{
    int ret;
	int retry_count = RETRY_COUNT;
    int DF_version=0;
    struct i2c_client *client = NULL;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    do
    {
    ret = bq27520_cntl_cmd(client, 0x004A);
    if (ret) {
        dev_err(&client->dev, "Send subcommand 0x%04X error.\n", 0x004A);
        retry_count--;
        msleep(I2C_RETRY_DELAY);
    }
    } while (ret && retry_count > 0);

    retry_count = RETRY_COUNT;
    udelay(800);
    do
    {

    ret = bq27520_read(client, 0x45, &DF_version, 0);

    if (ret)
    {
        dev_err(&client->dev, "Error!! %s subcommand %04X\n",
                         __func__, 0x004A);
        retry_count--;
        msleep(I2C_RETRY_DELAY);
    }
    } while (ret && retry_count > 0);

    mutex_unlock(&bq27520_dev_info_mutex);

    return DF_version;
}
#elif defined(CONFIG_Z380C)
int bq27520_read_DF_version(void)
{
    int ret, DF_version;
    struct i2c_client *client = NULL;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_write_i2c(client, 0x3E, 0x40, 1);
	if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Get fw cfg version error %d - 1.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    msleep(50);

	ret = bq27520_write_i2c(client, 0x3F, 0x00, 1);
	if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Get fw cfg version error %d - 2.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    msleep(50);

    ret = bq27520_read_i2c(client, 0x43, &DF_version, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read fw cfg version error %d - 3.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    mutex_unlock(&bq27520_dev_info_mutex);

    return DF_version;
}
#endif

int bq27520_is_normal_mode()
{
    int retry = RETRY_COUNT;
    struct i2c_client *i2c = NULL;
    int val=0;
    int ret=0;

    BAT_DBG_E("[%s] enter \n", __func__);

    mutex_lock(&bq27520_dev_info_mutex);
    i2c = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

    while (retry--) {
        ret = bq27520_read_i2c(i2c, 0x00, &val, 1);
        if (ret < 0) continue;

        break;
    };
    if (ret < 0) {
        return 0;
    }
    return 1;
}
#if 0
int bq27520_dump_registers(struct seq_file *s)
{
    int ret, temp_buf = 0;
    struct i2c_client *client = NULL;
    client = bq27520_dev_info.i2c;

    seq_printf(s, "=================================================\n");
    seq_printf(s, "Name\t\t\t\tvalue\n");
    seq_printf(s, "=================================================\n");

    ret = bq27520_read(client, BQ27520_REG_CNTL, &temp_buf, 0);
    if(ret)
        seq_printf(s, "Control()\t\t\terror\n");
    else
        seq_printf(s, "Control()\t\t\t%d\tN/A\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_AR, &temp_buf, 0);
    if(ret)
        seq_printf(s, "AtRate()\t\t\terror\n");
    else
        seq_printf(s, "AtRate()\t\t\t%d\tmA\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_ARTTE, &temp_buf, 0);
    if(ret)
        seq_printf(s, "AtRateTimeToEmpty()\t\terror\n");
    else
        seq_printf(s, "AtRateTimeToEmpty()\t\t%d\tMinutes\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_TEMP, &temp_buf, 0);
    if(ret)
        seq_printf(s, "Temperature()\t\t\terror\n");
    else
        seq_printf(s, "Temperature()\t\t\t%d\t0.1K\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_VOLT, &temp_buf, 0);
    if(ret)
        seq_printf(s, "Voltage()\t\t\terror\n");
    else
        seq_printf(s, "Voltage()\t\t\t%d\tmV\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_FLAGS, &temp_buf, 0);
    if(ret)
        seq_printf(s, "Flags()\t\t\t\terror\n");
    else
        seq_printf(s, "Flags()\t\t\t\t%d\tN/A\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_NAC, &temp_buf, 0);
    if(ret)
        seq_printf(s, "NominalAvailableCapacity()\terror\n");
    else
        seq_printf(s, "NominalAvailableCapacity()\t%d\tmAh\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_FAC, &temp_buf, 0);
    if(ret)
        seq_printf(s, "FullAvailableCapacity()\t\terror\n");
    else
        seq_printf(s, "FullAvailableCapacity()\t\t%d\tmAh\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_RM, &temp_buf, 0);
    if(ret)
        seq_printf(s, "RemainingCapacity()\t\terror\n");
    else
        seq_printf(s, "RemainingCapacity()\t\t%d\tmAh\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_FCC, &temp_buf, 0);
    if(ret)
        seq_printf(s, "FullChargeCapacity()\t\terror\n");
    else
        seq_printf(s, "FullChargeCapacity()\t\t%d\tmAh\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_AI, &temp_buf, 0);
    if(ret)
        seq_printf(s, "AverageCurrent()\t\terror\n");
    else
        seq_printf(s, "AverageCurrent()\t\t%d\tmA\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_TTE, &temp_buf, 0);
    if(ret)
        seq_printf(s, "TimeToEmpty()\t\t\terror\n");
    else
        seq_printf(s, "TimeToEmpty()\t\t\t%d\tMinutes\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_SI, &temp_buf, 0);
    if(ret)
        seq_printf(s, "StandbyCurrent()\t\terror\n");
    else
        seq_printf(s, "StandbyCurrent()\t\t%d\tmA\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_STTE, &temp_buf, 0);
    if(ret)
        seq_printf(s, "StandbyTimeToEmpty()\t\terror\n");
    else
        seq_printf(s, "StandbyTimeToEmpty()\t\t%d\tMinutes\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_SOH, &temp_buf, 0);
    if(ret)
        seq_printf(s, "StateOfHealth()\t\t\terror\n");
    else
        seq_printf(s, "StateOfHealth()\t\t\t%d\t%/num\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_CC, &temp_buf, 0);
    if(ret)
        seq_printf(s, "CycleCount()\t\t\terror\n");
    else
        seq_printf(s, "CycleCount()\t\t\t%d\tnum\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_SOC, &temp_buf, 0);
    if(ret)
        seq_printf(s, "StateOfCharge()\t\t\terror\n");
    else
        seq_printf(s, "StateOfCharge()\t\t\t%d\t%\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_IC, &temp_buf, 0);
    if(ret)
        seq_printf(s, "InstantaneousCurrent()\t\terror\n");
    else
        seq_printf(s, "InstantaneousCurrent()\t\t%d\tmA\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_IT, &temp_buf, 0);
    if(ret)
        seq_printf(s, "InternalTemperature()\t\terror\n");
    else
        seq_printf(s, "InternalTemperature()\t\t%d\t0.1K\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_RS, &temp_buf, 0);
    if(ret)
        seq_printf(s, "ResistanceScale()\t\terror\n");
    else
        seq_printf(s, "ResistanceScale()\t\t%d\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_OC, &temp_buf, 0);
    if(ret)
        seq_printf(s, "OperationConfiguration()\terror\n");
    else
        seq_printf(s, "OperationConfiguration()\t%d\tN/A\n", temp_buf);

    ret = bq27520_read(client, BQ27520_REG_DC, &temp_buf, 0);
    if(ret)
        seq_printf(s, "DesignCapacity()\t\terror\n");
    else
        seq_printf(s, "DesignCapacity()\t\t%d\tmAh\n", temp_buf);

    ret = bq27520_read(client, 0x6c, &temp_buf, 0);
    if(ret)
        seq_printf(s, "UnfilteredRM()\t\t\terror\n");
    else
        seq_printf(s, "UnfilteredRM()\t\t\t%d\tmAh\n", temp_buf);

    ret = bq27520_read(client, 0x6e, &temp_buf, 0);
    if(ret)
        seq_printf(s, "FilteredRM()\t\t\terror\n");
    else
        seq_printf(s, "FilteredRM()\t\t\t%d\tmAh\n", temp_buf);

    ret = bq27520_read(client, 0x70, &temp_buf, 0);
    if(ret)
        seq_printf(s, "UnfilteredFCC()\t\t\terror\n");
    else
        seq_printf(s, "UnfilteredFCC()\t\t\t%d\tmAh\n", temp_buf);

    ret = bq27520_read(client, 0x72, &temp_buf, 0);
    if(ret)
        seq_printf(s, "FilteredFCC()\t\t\terror\n");
    else
        seq_printf(s, "FilteredFCC()\t\t\t%d\tmAh\n", temp_buf);

    ret = bq27520_read(client, 0x74, &temp_buf, 0);
    if(ret)
        seq_printf(s, "TrueSOC()\t\t\terror\n");
    else
        seq_printf(s, "TrueSOC()\t\t\t%d\t%\n", temp_buf);

    return 0;
}
#endif

int bq27520_dump_registers(struct seq_file *s)
{
#if 0
    int  ret, temp_buf = 0;
    struct i2c_client *client = NULL;
    int retry_count = RETRY_COUNT;

    client = bq27520_dev_info.i2c;

    ret = bq27520_i2c_txsubcmd(client, 0x3E, 0x40A2);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Get fw cfg version error %d.\n", ret);
    }

    udelay(800);
    ret = bq27520_read_i2c(client, 0x40, &temp_buf, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read fw cfg version error %d.\n", ret);
    }

    if(ret)
        seq_printf(s, "Reset count: error\n");
    else
        seq_printf(s, "Reset count: %04X\n", temp_buf);

    ret = bq27520_i2c_txsubcmd(client, 0x3E, 0x40A3);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Get fw cfg version error %d.\n", ret);
    }

    udelay(800);
    ret = bq27520_read_i2c(client, 0x40, &temp_buf, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read fw cfg version error %d.\n", ret);
    }

    if(ret)
        seq_printf(s, "Reset count WD: error\n");
    else
        seq_printf(s, "Reset count WD: %04X\n", temp_buf);
#endif
    int gauge_fw;
    gauge_fw = bq27520_read_DF_version();
    seq_printf(s, "%04X\n", gauge_fw);

    return 0;
}

static int bq27520_debugfs_show(struct seq_file *s, void *data)
{
    bq27520_dump_registers(s);
    return 0;
}

static int bq27520_debugfs_open(struct inode *inode, struct file *file)
{
    return single_open(file, bq27520_debugfs_show, inode->i_private);
}

static const struct file_operations bq27520_debugfs_fops = {
    .open = bq27520_debugfs_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

int auto_battery_cell_update(void)
{
    int ret;

    /* Auto Battery Cell Data update */
    ret = bq27520_bat_upt_main_update_flow();
    mutex_lock(&bq27520_dev_info_mutex);
    bq27520_dev_info.update_status = ret;
    mutex_unlock(&bq27520_dev_info_mutex);

    if (ret >= UPDATE_OK) {
        msleep(500);
        TIgauge_LockStep();
    }

    return ret;
}
EXPORT_SYMBOL(auto_battery_cell_update);

static int bq27520_bat_i2c_probe(struct i2c_client *client,
                            const struct i2c_device_id *id)
{
    int __fw_cfg_version = 0;
    int chemicalID = 0;
    int ret = 0;
    int IRQ = 0;
    //struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct bq27520_chip *chip;
    struct device_node *np = client->dev.of_node;
	
    debugfs_create_file("bq27520", 0444, NULL, NULL, &bq27520_debugfs_fops);

    mutex_lock(&bq27520_dev_info_mutex);
    bq27520_dev_info.status = DEV_INIT;
    mutex_unlock(&bq27520_dev_info_mutex);

    BAT_DBG("++++++++++++++++ %s ++++++++++++++++\n", __func__);
    spin_lock_init(&spinlock_suspend_bq27520);

    /*if (!client->dev.platform_data) {
        dev_err(&client->dev, "Platform Data is NULL");
        return -EFAULT;
    }

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
        dev_err(&client->dev, "SM bus doesn't support DWORD transactions\n");
        return -EIO;
    }*/

    chip = kzalloc(sizeof(*chip), GFP_KERNEL);
    if (!chip) {
        dev_err(&client->dev, "mem alloc failed\n");
        return -ENOMEM;
    }

    chip->client = client;
    chip->pdata = client->dev.platform_data;

    i2c_set_clientdata(client, chip);
    bq27520_dev_info.i2c = client;

    channel = iio_channel_get(NULL, "BATTEMP0_SENSOR");

    /* Auto Battery Cell Data update */
    ret = bq27520_bat_upt_main_update_flow();
    mutex_lock(&bq27520_dev_info_mutex);
    bq27520_dev_info.update_status = ret;
    mutex_unlock(&bq27520_dev_info_mutex);
    if (ret < UPDATE_VOLT_NOT_ENOUGH)
        goto update_fail;
    else if (ret >= UPDATE_OK) {
        msleep(500);
        TIgauge_LockStep();
    }

#ifndef ME372CG_ENG_BUILD
    mutex_lock(&bq27520_dev_info_mutex);
    if (bq27520_dev_info.update_status == UPDATE_VOLT_NOT_ENOUGH) {
        batt_info.update_cell_not_done = true;
    }
    mutex_unlock(&bq27520_dev_info_mutex);
#endif

    ret = bq27520_hw_config(client);
    if (ret < 0)
        return -EIO;

    bq27520_dump_info(client);

    ret = bq27520_thermistor_check(client);
    if (ret < 0)
        return -EIO;

    /* register power supply driver */

    bq27520_tbl.read_percentage = bq27520_asus_battery_dev_read_percentage;
    bq27520_tbl.read_current = bq27520_asus_battery_dev_read_current;
    bq27520_tbl.read_volt = bq27520_asus_battery_dev_read_volt;
    bq27520_tbl.read_av_energy = bq27520_asus_battery_dev_read_av_energy;
    bq27520_tbl.read_temp = bq27520_asus_battery_dev_read_temp;
    bq27520_tbl.read_fcc = bq27520_asus_battery_dev_read_full_charge_capacity;
    bq27520_tbl.read_cc = bq27520_asus_battery_dev_read_cycle_count;
    bq27520_tbl.read_nac = bq27520_asus_battery_dev_nominal_available_capacity;
    bq27520_tbl.read_rm = bq27520_asus_battery_dev_read_remaining_capacity;

#if (BATT_PERCENT_OPT)
    bq27520_tbl.read_TIpercentage = bq27520_asus_battery_dev_read_TIpercentage;
#endif

    ret = asus_register_power_supply(&client->dev, &bq27520_tbl);
    if (ret)
        goto power_register_fail;

    chip->batlow_irq = -1;
    chip->adc_alert_irq = -1;

    /*ret = bq27520_irq_init(chip);
    if (ret) {
        dev_err(&client->dev, "bq27520 irq init: error\n");
        goto irq_init_fail;
    }*/
/*----------------------get gpio from device tree--------------------------*/
    chip->adc_alert_n_gpio = of_get_named_gpio_flags(np, "intel,adc-alert-n", 0, 0);
    pr_info("intel,adc-alert-n-gpio = %d.\n", chip->adc_alert_n_gpio);

    if (!gpio_is_valid(chip->adc_alert_n_gpio)) {
        pr_err("gpio is not valid: adc_alert_n_gpio\n");
        ret = -EINVAL;
        goto irq_init_fail;
    }
	ret = gpio_request(chip->adc_alert_n_gpio,"adc_alert_n");
    if (ret) {
        dev_err(&chip->client->dev, "gpio_request failed for %d ret=%d\n",
            chip->adc_alert_n_gpio, ret);
        goto irq_init_fail;
    }

    if (gpio_get_value(chip->adc_alert_n_gpio))
        dev_info(&chip->client->dev, ">>> ADC_ALERT_N (HIGH) <<<\n");
    else
        dev_info(&chip->client->dev, ">>> ADC_ALERT_N (LOW) <<<\n");

    if (ret < 0) {
        pr_err("fail to initialize VBUS_IN_DET_N gpio: %d\n", ret);
        goto irq_init_fail;
    }

    /*IRQ = irq_of_parse_and_map(np, 0);
    dev_info(&chip->client->dev, "IRQ : %d\n", IRQ);

	ret = request_threaded_irq(IRQ, NULL, adc_alert_n_interrupt,
                    IRQF_PERCPU | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
                    IRQF_ONESHOT |
                    IRQF_TRIGGER_FALLING,
                    "adc_alert_n",
                    chip);
    if (ret < 0)
        dev_info(&chip->client->dev, "config VBUS_IN_DET_N gpio as IRQ fail!\n");*/

    /* Init Runtime PM State */
    pm_runtime_put_noidle(&chip->client->dev);
    pm_schedule_suspend(&chip->client->dev, MSEC_PER_SEC);

    //INIT_WORK(&batlow_work,batlow_work_func);

    mutex_lock(&bq27520_dev_info_mutex);
    bq27520_dev_info.status = DEV_INIT_OK;
    mutex_unlock(&bq27520_dev_info_mutex);

    //batt_info.model = BQ27520_DEV_NAME;
    bq27520_get_firmware_version(client);

    //bq27520_config_earlysuspend(chip);

    /* chemical id */
	
    chemicalID = bq27520_asus_battery_dev_read_chemical_id();
    if (chemicalID < 0) {
        BAT_DBG_E("[%s] Fail to get chemical_id\n", __func__);
        sprintf(_chemical_id, "%s", "xxxx");
    }
    else
        sprintf(_chemical_id, "%04x", chemicalID);
    //batt_info.chemical_id = _chemical_id;

    /* firmware config version */

    __fw_cfg_version = bq27520_asus_battery_dev_read_fw_cfg_version();
    if (__fw_cfg_version < 0) {
        BAT_DBG_E("[%s] Fail to get firmware config version\n", __func__);
        sprintf(_fw_cfg_version, "%s", "xxxx");
    }
    else
        sprintf(_fw_cfg_version, "%04x", __fw_cfg_version);
    //batt_info.fw_cfg_version = _fw_cfg_version;

    /* create device node */
    sysfs_create_group(&client->dev.kobj, &dev_attr_grp);

    /* switch added for battery version info */
    chip->batt_dev.name = "battery";
    chip->batt_dev.print_name = batt_switch_name;
    if (switch_dev_register(&chip->batt_dev) < 0) {
        BAT_DBG_E("fail to register battery switch\n");
    }

    dev_info(&client->dev, "%s done.", __func__);

power_register_fail:
irq_init_fail:
update_fail:
    return ret;
}

#ifdef CONFIG_PM_RUNTIME
static int bq27520_runtime_suspend(struct device *dev)
{
    dev_dbg(dev, "%s called\n", __func__);
    return 0;
}

static int bq27520_runtime_resume(struct device *dev)
{
    dev_dbg(dev, "%s called\n", __func__);
    return 0;
}

static int bq27520_runtime_idle(struct device *dev)
{
    dev_dbg(dev, "%s called\n", __func__);
    return 0;
}
#else
#define bq27520_runtime_suspend    NULL
#define bq27520_runtime_resume        NULL
#define bq27520_runtime_idle        NULL
#endif

static const struct of_device_id bq27520_match[] = {
	{ .compatible = "intel,bq27520" },
	{ },
	};

static struct i2c_device_id bq27520_bat_i2c_id[] = {
    { BQ27520_DEV_NAME, 0 },
    { },
};

static const struct dev_pm_ops bq27520_pm_ops = {
    .suspend = bq27520_bat_i2c_suspend,
    .resume = bq27520_bat_i2c_resume,
    .runtime_suspend = bq27520_runtime_suspend,
    .runtime_resume = bq27520_runtime_resume,
    .runtime_idle = bq27520_runtime_idle,
};

static struct i2c_driver bq27520_bat_i2c_driver = {
    .driver    = {
        .name  = BQ27520_DEV_NAME,
        .owner = THIS_MODULE,
        .pm    = &bq27520_pm_ops,
		.of_match_table = of_match_ptr(bq27520_match),
    },
    .probe     = bq27520_bat_i2c_probe,
    .remove    = bq27520_bat_i2c_remove,
    .id_table  = bq27520_bat_i2c_id,
};

static int __init bq27520_bat_i2c_init(void)
{
    int ret = 0;
    u32 test_flag=0;
    u32 test_major_flag, test_minor_flag;
    int cell_sel=0;
    struct asus_bat_config bat_cfg;

    printk("++++++++++++++++ %s ++++++++++++++++\n", __func__);
    HW_ID = Read_HW_ID();
    BAT_DBG("*****HW_ID***** : %d\n", HW_ID);
    wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "bq27520");
    wake_lock_init(&polling_wakelock, WAKE_LOCK_SUSPEND, "bq27520_polling_wakelock");

    /* if fw sel type is not equal,
    it will enter update process
    later in probe function */
    cell_sel = bq27520_batt_current_sel_type();
    if (cell_sel < 0) {
        BAT_DBG_E("Read battery selector fail.\n");
        return cell_sel;
    }
    BAT_DBG_E("current cell status = %d\n", cell_sel);

    //turn to jiffeys/s
    bat_cfg.polling_time *= HZ;
    bat_cfg.critical_polling_time *= HZ;

    mutex_lock(&bq27520_dev_info_mutex);
    bq27520_dev_info.test_flag = test_minor_flag;
    mutex_unlock(&bq27520_dev_info_mutex);

    /*ret = bq27520_bat_upt_i2c_init();
    if (ret)
            goto bq27520_upt_i2c_init_fail;*/
	/*
    ret = bq27520_register_proc_fs();
    if (ret) {
        BAT_DBG_E("Unable to create proc file\n");
        goto proc_fail;
    }
    ret = bq27520_proc_fs_update_latch();
    if (ret) {
        BAT_DBG_E("Unable to create proc file\n");
        goto proc_fail;
    }*/

    ret = asus_battery_init(bat_cfg.polling_time,
            bat_cfg.critical_polling_time,
            test_major_flag);
    if (ret)
        goto asus_battery_init_fail;

    ret =  i2c_add_driver(&bq27520_bat_i2c_driver);
    if (ret) {
        BAT_DBG_E("register bq27520 battery i2c driver failed\n");
        goto i2c_register_driver_fail;
    }

    return ret;

i2c_register_driver_fail:
    //asus_battery_exit();
asus_battery_init_fail:
bq27520_upt_i2c_init_fail:
//proc_fail:
//register_i2c_device_fail:

    return ret;
}
late_initcall(bq27520_bat_i2c_init);

static void __exit bq27520_bat_i2c_exit(void)
{
    struct i2c_client *client = NULL;

    //asus_battery_exit();

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

    //bq27520_bat_upt_i2c_exit();

    i2c_unregister_device(bq27520_dev_info.i2c);
    i2c_del_driver(&bq27520_bat_i2c_driver);

    BAT_DBG("%s exit\n", __func__);
}
module_exit(bq27520_bat_i2c_exit);

MODULE_AUTHOR("chris1_chang@asus.com");
MODULE_DESCRIPTION("battery bq27520 driver");
MODULE_LICENSE("GPL");
