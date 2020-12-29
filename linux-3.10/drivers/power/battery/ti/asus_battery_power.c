/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include "asus_battery.h"
#include "asus_battery_proc_fs.h"
#include "bq27520_battery_core.h"
#include "bq27520_battery_upt_i2c.h"
#include <linux/wakelock.h>
#include <linux/pm_wakeup.h>
#include <linux/suspend.h>

struct wakeup_source *ws;
extern spinlock_t spinlock_suspend_bq27520;
extern bool suspend_bq27520;
extern bool suspend_bq27520_work_not_done;
extern struct battery_dev_info bq27520_dev_info;
extern int auto_battery_cell_update(void);
extern bool PAD_CHARGING_FULL(void);
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
static struct callback_data* callback_struct;
#endif

extern int entry_mode;
#if defined(CONFIG_Z380KL) || defined(CONFIG_Z380C)
extern bool IS_CB81(void);
extern int smb345_get_charging_status(void);
#endif

struct delayed_work battery_poll_data_work;
struct delayed_work detect_cable_work;
struct workqueue_struct *battery_work_queue=NULL;

/* wake lock to prevent S3 during charging */
struct wake_lock wakelock;
struct wake_lock wakelock_t;
struct wake_lock updating_wakelock;

DEFINE_MUTEX(batt_info_mutex);

#if (BATT_PERCENT_OPT)
static int USOC;
static int PSOC = -1;
static int pre_PSOC = -1;
#endif

struct battery_info_reply batt_info = {
    .drv_status = DRV_NOT_READY,
    .cable_status = NO_CABLE,
#ifdef ME372CG_ENG_BUILD
    .eng_charging_limit = true,
    .charging_limit_threshold = CHARGING_LIMIT_THRESHOLD,
#endif
#ifndef ME372CG_USER_BUILD
    .emerg_poll = false,
#endif
#ifndef ME372CG_ENG_BUILD
    .update_cell_not_done = false,
    .pad_screen_off = false,
#endif
};

void asus_start_polling_work()
{
    BAT_DBG(" %s\n", __func__);
    queue_delayed_work(battery_work_queue,
        &battery_poll_data_work,
        0*HZ);
}

void asus_cancel_work()
{
    if (battery_work_queue) {
        BAT_DBG_E(" %s:\n", __func__);
        cancel_delayed_work_sync(&battery_poll_data_work);
        cancel_delayed_work_sync(&detect_cable_work);
    }
}

#ifndef ME372CG_USER_BUILD
int asus_emerg_poll_read(char *page, char **start, off_t off,
                    int count, int *eof, void *date)
{
    int len = 0;
    BAT_DBG_E(" %s:\n", __func__);

    mutex_lock(&batt_info_mutex);
    batt_info.emerg_poll = true;
    mutex_unlock(&batt_info_mutex);

    asus_queue_update_all();

    return len;
}

int asus_emerg_poll_write(struct file *file, const char *buffer,
                    unsigned long count, void *data)
{
    BAT_DBG_E(" %s:\n", __func__);
    return count;
}

static int nasus_emerg_poll_read(struct seq_file *m, void *v)
{
    seq_printf(m, "%s\n", __func__);
    BAT_DBG_E(" %s:\n", __func__);

    mutex_lock(&batt_info_mutex);
    batt_info.emerg_poll = true;
    mutex_unlock(&batt_info_mutex);

    asus_queue_update_all();

    return 0;
}
static ssize_t nasus_emerg_poll_write(struct file *file,
    const char __user *buf, size_t count, loff_t * ppos)
{
    BAT_DBG_E(" %s:\n", __func__);
    return count;
}
static int nasus_emerg_poll_open(struct inode *inode, struct file *file)
{
	return single_open(file, nasus_emerg_poll_read, NULL);
}
int init_emerg_poll_toggle(void)
{
    static const struct file_operations emerg_poll_toggle_fops = {
        .owner = THIS_MODULE,
        .open = nasus_emerg_poll_open,
        .read = seq_read,
        .write = nasus_emerg_poll_write,
        .llseek = seq_lseek,
        .release = single_release,
    };
    struct proc_dir_entry *entry=NULL;

    entry = proc_create("poll", 0644, NULL,
        &emerg_poll_toggle_fops);
    if (!entry) {
        BAT_DBG_E("Unable to create /proc/poll\n");
        return -EINVAL;
    }

    return 0;
}
#else
int init_emerg_poll_toggle(void) { return 0; }
#endif

#ifdef ME372CG_ENG_BUILD
static int nasus_charging_toggle_read(struct seq_file *m, void *v)
{
    seq_printf(m, "%s\n", __func__);
    BAT_DBG_E(" %s:\n", __func__);

    return 0;
}
static ssize_t nasus_charging_toggle_write(struct file *file,
    const char __user *buf, size_t count, loff_t * ppos)
{
    BAT_DBG_E(" %s:\n", __func__);

    char proc_buf[64];
    struct battery_info_reply tmp_batt_info;
    bool eng_charging_limit = true;

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    eng_charging_limit = tmp_batt_info.eng_charging_limit;

    if (count > sizeof(proc_buf)) {
        BAT_DBG("%s: data error\n", __func__);
        return -EINVAL;
    }

    if (copy_from_user(proc_buf, buf, count)) {
        BAT_DBG("%s: read data from user space error\n", __func__);
        return -EFAULT;
    }

    if (proc_buf[0] == '1') {
        /* turn on charging limit in eng mode */
        eng_charging_limit = true;
    }
    else if (proc_buf[0] == '0') {
        /* turn off charging limit in eng mode */
        eng_charging_limit = false;
    }

    tmp_batt_info.eng_charging_limit = eng_charging_limit;

    mutex_lock(&batt_info_mutex);
    batt_info = tmp_batt_info;
    mutex_unlock(&batt_info_mutex);

    asus_queue_update_all();

    return count;
}
static int nasus_charging_toggle_open(struct inode *inode, struct file *file)
{
	return single_open(file, nasus_charging_toggle_read, NULL);
}
int init_asus_charging_toggle(void)
{
    static const struct file_operations asus_charging_toggle_fops = {
        .owner = THIS_MODULE,
        .open = nasus_charging_toggle_open,
        .read = seq_read,
        .write = nasus_charging_toggle_write,
        .llseek = seq_lseek,
        .release = single_release,
    };
    struct proc_dir_entry *entry=NULL;

    entry = proc_create("driver/charger_limit_enable", 0666, NULL,
        &asus_charging_toggle_fops);
    if (!entry) {
        BAT_DBG_E("Unable to create asus_charging_toggle\n");
        return -EINVAL;
    }

    return 0;
}
#else
int init_asus_charging_toggle(void) { return 0; }
#endif

static enum power_supply_property asus_battery_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_CYCLE_COUNT,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_ENERGY_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_CAPACITY_LEVEL,
    POWER_SUPPLY_PROP_TEMP,
    //POWER_SUPPLY_PROP_MANUFACTURER,
    POWER_SUPPLY_PROP_MODEL_NAME,
    //POWER_SUPPLY_PROP_SERIAL_NUMBER,
    POWER_SUPPLY_PROP_CURRENT_AVG,
    POWER_SUPPLY_PROP_CHARGE_NOW,
    POWER_SUPPLY_PROP_CHARGE_FULL,
    /*POWER_SUPPLY_PROP_BATTERY_ID,
    POWER_SUPPLY_PROP_FIRMWARE_VERSION,
    POWER_SUPPLY_PROP_CHEMICAL_ID,
    POWER_SUPPLY_PROP_FW_CFG_VER,*/
};

static int asus_battery_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val);

static struct power_supply asus_power_supplies[] = {
    {
        .name = "battery",
        .type = POWER_SUPPLY_TYPE_BATTERY,
        .properties = asus_battery_props,
        .num_properties = ARRAY_SIZE(asus_battery_props),
        .get_property = asus_battery_get_property,
    },
};

int asus_battery_low_event()
{
    drv_status_t drv_status;

    mutex_lock(&batt_info_mutex);
    drv_status = batt_info.drv_status;
    mutex_unlock(&batt_info_mutex);

    drv_status = DRV_BATTERY_LOW;

    mutex_lock(&batt_info_mutex);
    batt_info.drv_status = drv_status;
    mutex_unlock(&batt_info_mutex);

    power_supply_changed(&asus_power_supplies[CHARGER_BATTERY]);

    return 0;
}

/*
 * Return the battery full charge capacity in mAh
 * Or < 0 if something fails.
 */
static int asus_battery_update_fcc(void)
{
    int fcc = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_fcc)
        fcc = tmp_batt_info.tbl->read_fcc();

    return fcc;
}
static int asus_battery_update_fcc_no_mutex(void)
{
    int fcc = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    tmp_batt_info = batt_info;
    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_fcc)
        fcc = tmp_batt_info.tbl->read_fcc();

    return fcc;
}

/*
 * Return the battery charging cycle count
 * Or < 0 if something fails.
 */
static int asus_battery_update_cc(void)
{
    int cc = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_cc)
        cc = tmp_batt_info.tbl->read_cc();

    return cc;
}
static int asus_battery_update_cc_no_mutex(void)
{
    int cc = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    tmp_batt_info = batt_info;
    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_cc)
        cc = tmp_batt_info.tbl->read_cc();

    return cc;
}

/*
 * Return the battery nominal available capacity
 * Or < 0 if something fails.
 */
static int asus_battery_update_nac(void)
{
    int nac = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_nac)
        nac = tmp_batt_info.tbl->read_nac();

    return nac;
}
static int asus_battery_update_nac_no_mutex(void)
{
    int nac = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    tmp_batt_info = batt_info;
    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_nac)
        nac = tmp_batt_info.tbl->read_nac();

    return nac;
}

/*
 * Return the battery remaining capacity
 * Or < 0 if something fails.
 */
static int asus_battery_update_rm(void)
{
    int rm = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_rm)
        rm = tmp_batt_info.tbl->read_rm();

    return rm;
}
static int asus_battery_update_rm_no_mutex(void)
{
    int rm = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    tmp_batt_info = batt_info;
    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_rm)
        rm = tmp_batt_info.tbl->read_rm();

    return rm;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int asus_battery_update_temp(void)
{
    int temperature = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_temp)
        temperature = tmp_batt_info.tbl->read_temp();

    return temperature;
}
static int asus_battery_update_temp_no_mutex(void)
{
    int temperature = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    tmp_batt_info = batt_info;
    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_temp)
        temperature = tmp_batt_info.tbl->read_temp();

    return temperature;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */

static int asus_battery_update_voltage(void)
{
    int volt = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_volt)
        volt = tmp_batt_info.tbl->read_volt();

    return volt;
}
static int asus_battery_update_voltage_no_mutex(void)
{
    int volt = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    tmp_batt_info = batt_info;
    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_volt)
        volt = tmp_batt_info.tbl->read_volt();

    return volt;
}

#if (BATT_PERCENT_OPT)
static int asus_battery_update_TIpercentage_no_mutex(void)
{
    int TIP = -EINVAL;
    struct battery_info_reply tmp_batt_info;
    tmp_batt_info = batt_info;
    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_TIpercentage)
        TIP = tmp_batt_info.tbl->read_TIpercentage();

    return TIP;
}
#endif

/*
 * Return the battery average current
 * Note that current can be negative signed as well(-65536 ~ 65535).
 * So, the value get from device need add a base(0x10000)
 * to be a positive number if no error.
 * Or < 0 if something fails.
 */
static int asus_battery_update_current(void)
{
    int curr = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_current)
        curr = tmp_batt_info.tbl->read_current();

    return curr;
}
static int asus_battery_update_current_no_mutex(void)
{
    int curr = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    tmp_batt_info = batt_info;
    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_current)
        curr = tmp_batt_info.tbl->read_current();

    return curr;
}

#if CURRENT_IC_VERSION == IC_VERSION_G3
static int asus_battery_update_available_energy(void)
{
    int mWhr = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_av_energy)
        mWhr = tmp_batt_info.tbl->read_av_energy();

    return mWhr;
}
static int asus_battery_update_available_energy_no_mutex(void)
{
    int mWhr = -EINVAL;
    struct battery_info_reply tmp_batt_info;

    tmp_batt_info = batt_info;
    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_av_energy)
        mWhr = tmp_batt_info.tbl->read_av_energy();

    return mWhr;
}
#endif

#ifdef BATTERY_PERCENTAGE_SHIFT
static int percentage_shift(int percentage)
{
    int ret;

    ret = ((percentage >= 100) ? 100 : percentage);

    /* start percentage shift:
        battery percentage remapping according
        to battery discharging curve
    */

    if (ret == 99) {
        ret = 100;
        return ret;
    }
    else if (ret >= 84 && ret <= 98) {
        ret++;
        return ret;
    }

    /* missing 26%, 47%, 58%, 69%, 79% */
    if (ret > 70 && ret < 80)
        ret -= 1;
    else if (ret > 60 && ret <= 70)
        ret -= 2;
    else if (ret > 50 && ret <= 60)
        ret -= 3;
    else if (ret > 30 && ret <= 50)
        ret -= 4;
    else if (ret >= 0 && ret <= 30)
        ret -= 5;

    /* final check to avoid negative value */
    if (ret < 0)
        ret = 0;

    return ret;
}
#endif

#if (BATT_PERCENT_OPT)
static int percentage_shift2()
{
	if(PSOC < 0)
    {
        if((pre_PSOC-USOC)>2 || (USOC-pre_PSOC)>2)
            pre_PSOC = USOC;
        else if(pre_PSOC > USOC)
            pre_PSOC -= 1;
        else if(PSOC < USOC)
            pre_PSOC += 1;

        PSOC = pre_PSOC;
    }
    else
    {
        if(PSOC > USOC)
            PSOC -= 1;
        else if(PSOC < USOC)
            PSOC += 1;
    }
    if(PSOC < 0)
        PSOC = 0;
    if(PSOC > 100)
        PSOC = 100;
    return PSOC;
}
#endif

static int asus_battery_update_percentage(void)
{
    int percentage = -EINVAL;
    drv_status_t drv_status;
    struct battery_info_reply tmp_batt_info;

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    drv_status = batt_info.drv_status;
    mutex_unlock(&batt_info_mutex);

    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_percentage) {
        percentage = tmp_batt_info.tbl->read_percentage();

        if (percentage == ERROR_CODE_I2C_FAILURE)
            return -EINVAL;

        if (percentage >= 0 && percentage <= 3) {
            percentage = 0;
            drv_status = DRV_SHUTDOWN;
        }

        /* illegal RSOC check for UPI gauge IC */
        if (percentage < 0) {
            BAT_DBG_E("*** Wrong battery percentage = %d ***\n",
                percentage);
            percentage = 0;
        }
        if (percentage > 100) {
            BAT_DBG_E("*** Wrong battery percentage = %d ***\n",
                percentage);
            percentage = 100;
        }

        mutex_lock(&batt_info_mutex);
        batt_info.drv_status = drv_status;
        mutex_unlock(&batt_info_mutex);
    }

    return percentage;
}
static int asus_battery_update_percentage_no_mutex(u32* rawpercentage)
{
    int percentage = -EINVAL;
    drv_status_t drv_status;
    struct battery_info_reply tmp_batt_info;

    tmp_batt_info = batt_info;
    drv_status = batt_info.drv_status;

    if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_percentage) {
        percentage = tmp_batt_info.tbl->read_percentage();

        if (percentage == ERROR_CODE_I2C_FAILURE)
            return -EINVAL;

        *rawpercentage = percentage;
#ifdef BATTERY_PERCENTAGE_SHIFT
        percentage = percentage_shift(percentage);
#else
        if (percentage >= 0 && percentage <= 3) {
            percentage = 0;
            drv_status = DRV_SHUTDOWN;
        }
#endif

#if (BATT_PERCENT_OPT)
        USOC = percentage;
		percentage = percentage_shift2();
        PSOC = percentage;
#endif

        batt_info.drv_status = drv_status;
    }

    return percentage;
}

/* range:   range_min <= X < range_max */
struct lvl_entry {
    char name[30];
    int range_min;
    int range_max;
    int ret_val;
};

#define MAX_DONT_CARE    1000000
#define MIN_DONT_CARE    -999999
struct lvl_entry lvl_tbl[] = {
{"FULL",    100,           MAX_DONT_CARE, POWER_SUPPLY_CAPACITY_LEVEL_FULL},
{"HIGH",    80,            100,           POWER_SUPPLY_CAPACITY_LEVEL_HIGH},
{"NORMAL",  20,            80,            POWER_SUPPLY_CAPACITY_LEVEL_NORMAL},
{"LOW",     5,             20,            POWER_SUPPLY_CAPACITY_LEVEL_LOW},
{"CRITICAL",0,             5,             POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL},
{"UNKNOWN", MIN_DONT_CARE, MAX_DONT_CARE, POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN},
};

static int asus_battery_update_capacity_level(int percentage)
{
    int i=0;

    for (i=0; i<sizeof(lvl_tbl)/sizeof(struct lvl_entry); i++) {
        if (lvl_tbl[i].range_max != MAX_DONT_CARE &&
            percentage >= lvl_tbl[i].range_max)
            continue;
        if (lvl_tbl[i].range_min != MIN_DONT_CARE &&
            lvl_tbl[i].range_min > percentage)
            continue;

        return lvl_tbl[i].ret_val;
    }

    return -EINVAL;
}

static int asus_battery_update_status_no_mutex(struct battery_info_reply tmp_batt_info)
{
    int status;
    int temperature = tmp_batt_info.batt_temp;
    int volt = tmp_batt_info.batt_volt;
    int percentage = tmp_batt_info.percentage;
    u32 cable_status;

    cable_status = tmp_batt_info.cable_status;

    if (cable_status == USB_ADAPTER || cable_status == USB_PC) {
        status = POWER_SUPPLY_STATUS_CHARGING;

#ifdef ME372CG_ENG_BUILD
        /* ME371MG, ME302C, ME372CG eng mode:
            stop charging when battery percentage is over 60%
        */
        if (percentage >= tmp_batt_info.charging_limit_threshold && tmp_batt_info.eng_charging_limit) {
            if (tmp_batt_info.tbl_chgr &&
                tmp_batt_info.tbl_chgr->charging_toggle)
                tmp_batt_info.tbl_chgr->charging_toggle(JEITA, false);
            status = POWER_SUPPLY_STATUS_DISCHARGING;
            goto final;
        }
#endif

        /* charger AICL current control */
        if (cable_status == USB_ADAPTER) {
            if (tmp_batt_info.tbl_chgr && tmp_batt_info.tbl_chgr->aicl_control)
                tmp_batt_info.tbl_chgr->aicl_control(1);
        }

        /* ME371MG, ME302C, ME372CG
            stop charging to protect battery from damaged when
            battery temperature is too low or too high
        */
        if (temperature != ERROR_CODE_I2C_FAILURE && volt != ERROR_CODE_I2C_FAILURE) {

            if (tmp_batt_info.tbl_chgr && tmp_batt_info.tbl_chgr->jeita_control)
                status = tmp_batt_info.tbl_chgr->jeita_control(temperature, volt);
        }
        if (percentage == 100)
            status = POWER_SUPPLY_STATUS_FULL;
#if defined(CONFIG_Z380KL) || defined(CONFIG_Z380C)
        else {
            if (IS_CB81())
                status = smb345_get_charging_status();
        }
#endif
    }
    else {
        if (percentage == 100)
            status = POWER_SUPPLY_STATUS_FULL;
        else
#if defined(CONFIG_Z380KL) || defined(CONFIG_Z380C)
        {
            if (IS_CB81())
                status = smb345_get_charging_status();
            else
                status = POWER_SUPPLY_STATUS_DISCHARGING;
        }
#else
            status = POWER_SUPPLY_STATUS_DISCHARGING;
#endif
    }

    if (percentage < 0)
        status = POWER_SUPPLY_STATUS_UNKNOWN;
final:
    return status;
}

static void print_battery_info_no_mutex(void)
{
    int temperature;
    int temperature10;

    /*  UNKN means UNKOWN, CHRG mean CHARGING, DISC means DISCHARGING,
        NOTC means NOT CHARGING
    */
    char batt_status_str[5][5] = {
        "UNKN",
        "CHRG",
        "DISC",
        "NOTC",
        "FULL"
    };
    char *negative_sign = "-";

    /* printk(...) does not support %f and %e (float point and double) */
    if (batt_info.batt_temp >= 10 || batt_info.batt_temp <= -10) {
        temperature10 = batt_info.batt_temp/10;
        temperature = batt_info.batt_temp - (temperature10 * 10);
        if (temperature < 0) {
            temperature = -temperature;
        }
        negative_sign = "";
    }
    else {
        temperature10 = 0;
        temperature = batt_info.batt_temp < 0 ?
                    -batt_info.batt_temp : batt_info.batt_temp;
        if (batt_info.batt_temp >= 0)
            negative_sign = "";
    }

#if (BATT_PERCENT_OPT)
    BAT_DBG_E(" P:%d(B:%d U:%d TI:%d)%%, V:%dmV, C:%dmA, T:%s%d.%dC, "
        "S:%s, N:%dmAh, L:%dmAh, R:%dmAh, CC:%d, P:%ds\n",
        batt_info.percentage,
        batt_info.raw_percentage,
        USOC,
        batt_info.TI_percentage,
        batt_info.batt_volt,
        batt_info.batt_current,
        negative_sign,
        temperature10,
        temperature,
        batt_status_str[batt_info.status],
        batt_info.nac,
        batt_info.fcc,
        batt_info.rm,
        batt_info.cc,
        batt_info.polling_time/HZ);

#else
    BAT_DBG_E(" P:%d(%d)%%, V:%dmV, C:%dmA, T:%s%d.%dC, "
        "S:%s, N:%dmAh, L:%dmAh, R:%dmAh, CC:%d, P:%ds\n",
        batt_info.percentage,
        batt_info.raw_percentage,
        batt_info.batt_volt,
        batt_info.batt_current,
        negative_sign,
        temperature10,
        temperature,
        batt_status_str[batt_info.status],
        batt_info.nac,
        batt_info.fcc,
        batt_info.rm,
        batt_info.cc,
        batt_info.polling_time/HZ);
#endif
}

static void asus_battery_get_info_no_mutex(void)
{
    struct battery_info_reply tmp_batt_info;
    static int pre_batt_percentage = -1;
    static int pre_cable_status = -1;
    int tmp = 0;
#ifdef ASUS_PROJECT_ME372CG_TOUCH
    bool touch_ctrl = false;
#endif

    tmp_batt_info = batt_info;

    tmp = asus_battery_update_voltage_no_mutex();
    if (tmp >= 0) tmp_batt_info.batt_volt = tmp;

#if (BATT_PERCENT_OPT)
    tmp = asus_battery_update_TIpercentage_no_mutex();
    if (tmp >= 0) tmp_batt_info.TI_percentage = tmp;
#endif

    tmp = asus_battery_update_current_no_mutex();
    if (tmp >= 0) tmp_batt_info.batt_current = tmp - 0x10000;

    tmp = asus_battery_update_percentage_no_mutex(
            &tmp_batt_info.raw_percentage);
    if (tmp >= 0) {
        if (pre_cable_status < 0)
            pre_cable_status = tmp_batt_info.cable_status;
        if (pre_batt_percentage < 0) {
            tmp_batt_info.percentage = tmp;
            pre_batt_percentage = tmp;
            BAT_DBG(" Init the battery percentage values = %d(%d)\n",
                pre_batt_percentage, tmp_batt_info.raw_percentage);
#ifdef ASUS_PROJECT_ME372CG_TOUCH
            /* Add for touch low power issue in initialization */
            if (entry_mode == 1)
                if ((tmp >= 0) && (tmp <= 100))
                    touch_ctrl = true;
#endif
        }
        else if ((tmp_batt_info.cable_status == NO_CABLE) &&
                (pre_cable_status == NO_CABLE) &&
#if defined(CONFIG_Z380KL) || defined(CONFIG_Z380C)
                (!IS_CB81()) &&
#endif
                (pre_batt_percentage < tmp)) {
            tmp_batt_info.percentage = pre_batt_percentage;
            BAT_DBG_E(" keep pre battery percentage = (pre:%d, now:%d)\n",
                pre_batt_percentage, tmp);
        }
        else {
            /* FIX:
                suddently battery percentage drop while it
                is nearly battery low. We adopt the Dichotomy
                method to report the percentage smoothly
            */
            if (tmp < 4 && pre_batt_percentage > 5) {
                BAT_DBG_E(" modify dropping percentage = (now:%d, mod:%d)\n",
                    tmp, (tmp+pre_batt_percentage)/2);
                tmp = (tmp + pre_batt_percentage) / 2;
            }

            /* Charger IC registers dump */
            if (pre_batt_percentage != 0 && pre_batt_percentage != 5)
                if (tmp == 0 || tmp == 5)
                    if (tmp_batt_info.tbl_chgr &&
                        tmp_batt_info.tbl_chgr->dump_registers)
                        tmp_batt_info.tbl_chgr->dump_registers(NULL);

#ifdef ASUS_PROJECT_ME372CG_TOUCH
            /* Add for touch low power issue */
            if (entry_mode == 1) {
                if ((tmp >= 5) && (pre_batt_percentage <= 4) ||
                    (tmp <= 4) && (pre_batt_percentage >= 5)) {
                    touch_ctrl = true;
                }
            }
#endif

            tmp_batt_info.percentage = tmp;
            pre_batt_percentage = tmp;
        }
        pre_cable_status = tmp_batt_info.cable_status;
    }
    else {
        /* DEF: define the returned value of battery capacity
            with -99 when i2c communication failure.
        */
        tmp_batt_info.percentage = -99;
    }

    tmp = asus_battery_update_capacity_level(tmp_batt_info.percentage);
    if (tmp >= 0)
        tmp_batt_info.level = tmp;

#if CURRENT_IC_VERSION == IC_VERSION_G3
    tmp = asus_battery_update_available_energy_no_mutex();
    if (tmp >= 0)
        tmp_batt_info.batt_energy = tmp;
#endif
    tmp = asus_battery_update_fcc_no_mutex();
    if (tmp != ERROR_CODE_I2C_FAILURE && tmp != -EINVAL)
        tmp_batt_info.fcc = tmp;

    tmp = asus_battery_update_rm_no_mutex();
    if (tmp != ERROR_CODE_I2C_FAILURE && tmp != -EINVAL)
        tmp_batt_info.rm = tmp;

    tmp = asus_battery_update_cc_no_mutex();
    if (tmp != ERROR_CODE_I2C_FAILURE && tmp != -EINVAL)
        tmp_batt_info.cc = tmp;

    tmp = asus_battery_update_nac_no_mutex();
    if (tmp != ERROR_CODE_I2C_FAILURE && tmp != -EINVAL)
        tmp_batt_info.nac = tmp;

    tmp = asus_battery_update_temp_no_mutex();
    if (tmp != ERROR_CODE_I2C_FAILURE && tmp != -EINVAL)
        tmp_batt_info.batt_temp = tmp;

    tmp = asus_battery_update_status_no_mutex(tmp_batt_info);
    if (tmp >= 0)
        tmp_batt_info.status = tmp;

    /* ME371MG, ME302C, ME372CG battery polling algorithm
     * According to charger IC and gauge IC spec.
     */

    if (tmp_batt_info.percentage >= 50 &&
        tmp_batt_info.percentage <= 100)
        tmp_batt_info.polling_time = 60*HZ;
    else if (tmp_batt_info.percentage >= 20 &&
        tmp_batt_info.percentage <= 49)
        tmp_batt_info.polling_time = 30*HZ;
    else if (tmp_batt_info.percentage >= 5 &&
        tmp_batt_info.percentage <= 19)
        tmp_batt_info.polling_time = 10*HZ;
    else if (tmp_batt_info.percentage >= 0 &&
        tmp_batt_info.percentage <= 4)
        tmp_batt_info.polling_time = 5*HZ;
    else {
        BAT_DBG_E("*** Battery percentage is out of the legal range "
            "(0% ~ 100%) ***\n");
        tmp_batt_info.polling_time = 5*HZ;
    }

    if (tmp_batt_info.batt_temp >= 550) {
        BAT_DBG("Critical condition!! "
            "-> temperature \n");
        tmp_batt_info.polling_time = BATTERY_CRITICAL_POLL_TIME;
    }
    else if (tmp_batt_info.batt_temp >= 500) {
        BAT_DBG("Nearly critical condition!! "
            "-> high battery temperature > 50.0C\n");
        if (tmp_batt_info.polling_time > (10*HZ))
            tmp_batt_info.polling_time = 10*HZ;
    }
    else if (tmp_batt_info.batt_temp < -100) {
        BAT_DBG("Critical condition!! "
            "-> low battery temperature < -10.0C\n");
        tmp_batt_info.polling_time = BATTERY_CRITICAL_POLL_TIME;
    }
    else if (tmp_batt_info.batt_temp < 30) {
        BAT_DBG("Nearly critical condition!! "
            "-> low battery temperature < 3.0C\n");
        if (tmp_batt_info.polling_time > (10*HZ))
            tmp_batt_info.polling_time = 10*HZ;
    }

#ifndef ME372CG_USER_BUILD
    /* battery emergency polling time interval */
    if (batt_info.emerg_poll)
        tmp_batt_info.polling_time = BATTERY_EMERGENCY_POLL_TIME;
#endif

    batt_info = tmp_batt_info;
    print_battery_info_no_mutex();
}

static void asus_polling_data(struct work_struct *dat)
{
    u32 polling_time = 60*HZ;

#ifndef ME372CG_ENG_BUILD
    static bool updating_cell_data = false;
    int ret;

    if (updating_cell_data)
        goto final;
#endif

    polling_time = asus_update_all();

#ifndef ME372CG_ENG_BUILD
    mutex_lock(&batt_info_mutex);
    if (batt_info.update_cell_not_done == true) {
        if (batt_info.pad_screen_off       == true                   &&
            batt_info.cable_status         == USB_ADAPTER            &&
            PAD_CHARGING_FULL()            == true)
        {
            updating_cell_data = true;
        }
        else {
            BAT_DBG(" bq27520_dev_info.update_status:%s, batt_info.pad_screen_off:%s, batt_info.cable_status:%s, PAD_CHARGING_FULL():%s\n",
                bq27520_dev_info.update_status == UPDATE_VOLT_NOT_ENOUGH ? "UPDATE_VOLT_NOT_ENOUGH" : "VOLT_OK",
                batt_info.pad_screen_off ?  "TRUE" : "FALSE",
                batt_info.cable_status == USB_ADAPTER ? "AC_IN" : "NOT_AC",
                PAD_CHARGING_FULL() ? "YES" : "NO");
        }
    }
    mutex_unlock(&batt_info_mutex);

    if (updating_cell_data) {

        wake_lock(&updating_wakelock);

        /* trigger cell data update here */
        ret = auto_battery_cell_update();

        wake_unlock(&updating_wakelock);

        updating_cell_data = false;

        if (ret >= UPDATE_OK) {

            mutex_lock(&batt_info_mutex);
            batt_info.update_cell_not_done = false;
            mutex_unlock(&batt_info_mutex);

            BAT_DBG(" %s: ============Auto Update Done============\n",
                __func__);
        }
        else {
            BAT_DBG(" %s: ============Auto Update Fail: %d============\n",
                __func__, ret);
        }
    }

final:
#endif
    queue_delayed_work(battery_work_queue,
        &battery_poll_data_work,
        polling_time);
}

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
void bq27520_screen_changed_listener(const int state)
{
    if (state == NOTIFY_WHEN_SCREEN_OFF) {
        BAT_DBG(" %s: ScreenOff\n", __func__);

        #ifndef ME372CG_ENG_BUILD
        mutex_lock(&batt_info_mutex);
        batt_info.pad_screen_off = true;
        mutex_unlock(&batt_info_mutex);
        #endif
    }
    else if (state == NOTIFY_WHEN_SCREEN_ON) {
        BAT_DBG(" %s: ScreenOn\n", __func__);

        #ifndef ME372CG_ENG_BUILD
        mutex_lock(&batt_info_mutex);
        batt_info.pad_screen_off = false;
        mutex_unlock(&batt_info_mutex);
        #endif
    }
}
#endif

u32 asus_update_all(void)
{
    u32 time;

    /* Do not i2c-5 transmit in the flow of system suspend */
    spin_lock(&spinlock_suspend_bq27520);
    if (suspend_bq27520) {
        suspend_bq27520_work_not_done = true;
        BAT_DBG(" %s: suspend_work_not_done = true!!!\n", __func__);
        spin_unlock(&spinlock_suspend_bq27520);
        return 30*HZ;
    }
    spin_unlock(&spinlock_suspend_bq27520);

    __pm_stay_awake(ws);
    mutex_lock(&batt_info_mutex);

    asus_battery_get_info_no_mutex();
    time = batt_info.polling_time;
    power_supply_changed(&asus_power_supplies[CHARGER_BATTERY]);

    mutex_unlock(&batt_info_mutex);
    __pm_relax(ws);

    spin_lock(&spinlock_suspend_bq27520);
    suspend_bq27520_work_not_done = false;
    spin_unlock(&spinlock_suspend_bq27520);

    return time;
}

static void USB_cable_status_worker(struct work_struct *dat)
{
    asus_update_all();
}

void asus_queue_update_all(void)
{
    queue_delayed_work(battery_work_queue,
        &detect_cable_work,
        0.1*HZ);
}

#if (BATT_PERCENT_OPT)
void ac_in_screen_off(void)
{
    pre_PSOC = PSOC;
    PSOC = -1;
}
#endif

//#ifdef CONFIG_ME372CG_BATTERY_BQ27520
void usb_to_battery_callback(u32 usb_cable_state)
{
    drv_status_t drv_sts;

    mutex_lock(&batt_info_mutex);
    drv_sts = batt_info.drv_status;
    mutex_unlock(&batt_info_mutex);

    mutex_lock(&batt_info_mutex);
    batt_info.cable_status = usb_cable_state;
    mutex_unlock(&batt_info_mutex);

    if (drv_sts != DRV_REGISTER_OK) {
        BAT_DBG_E("Battery module not ready\n");
        return;
    }

    mutex_lock(&batt_info_mutex);

    /* prevent system from entering s3 in COS
        while AC charger is connected
    */
    if (entry_mode == 4) {
        if (batt_info.cable_status == USB_ADAPTER) {
            if (!wake_lock_active(&wakelock)) {
                BAT_DBG(" %s: asus_battery_power_wakelock "
                    "-> wake lock\n",
                    __func__);
                wake_lock_timeout(&wakelock, 180*HZ);
            }
        }
        else if (batt_info.cable_status == NO_CABLE) {
            if (wake_lock_active(&wakelock)) {
                BAT_DBG(" %s: asus_battery_power_wakelock "
                        "-> wake unlock\n",
                        __func__);
                /* timeout value as same as the
                    <charger.exe>\asus_global.h
                    #define ASUS_UNPLUGGED_SHUTDOWN_TIME(3 sec)
                */
                wake_lock_timeout(&wakelock_t, 3*HZ);
                wake_unlock(&wakelock);
            } else { // for PC case
                wake_lock_timeout(&wakelock_t, 3*HZ);
            }
        }
    }

    mutex_unlock(&batt_info_mutex);

    asus_queue_update_all();
}
//#else
//void usb_to_battery_callback(u32 usb_cable_state) { return; }
//#endif
EXPORT_SYMBOL(usb_to_battery_callback);

int receive_USBcable_type(void)
{
    u32 cable_status;
    struct battery_info_reply tmp_batt_info;

    BAT_DBG("%s\n", __func__);

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    cable_status = tmp_batt_info.cable_status ;
    return cable_status;
}
EXPORT_SYMBOL(receive_USBcable_type);

static int asus_battery_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    int ret = 0;
    struct battery_info_reply tmp_batt_info;

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = tmp_batt_info.status;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = POWER_SUPPLY_HEALTH_GOOD;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = tmp_batt_info.present;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
        break;
    case POWER_SUPPLY_PROP_CYCLE_COUNT:
        val->intval = tmp_batt_info.cc;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = tmp_batt_info.batt_volt;
        /* ME371MG, ME302C, ME372CG:
            change the voltage unit from Milli
            Voltage (mV) to Micro Voltage (uV)
        */
        val->intval *= 1000;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        /* ME371MG, ME302C, ME372CG:
            change the current unit from Milli
            Ampere (mA) to Micro Ampere (uA)
        */
        val->intval = tmp_batt_info.batt_current;
        val->intval *= 1000;
        break;
    case POWER_SUPPLY_PROP_CURRENT_AVG:
        val->intval = tmp_batt_info.batt_current;
        break;
//#ifdef CONFIG_ME372CG_BATTERY_BQ27520
    case POWER_SUPPLY_PROP_CHARGE_NOW:
        val->intval = bq27520_asus_battery_dev_read_remaining_capacity();
        break;
//#endif
    case POWER_SUPPLY_PROP_CHARGE_FULL:
        val->intval = bq27520_asus_battery_dev_read_full_charge_capacity();
        break;
    case POWER_SUPPLY_PROP_ENERGY_NOW:
        val->intval = tmp_batt_info.batt_energy;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
#ifndef BQ27XXX_WITH_CELL_DATA
        /* System will never shutdown when battery rsoc acquired from
           Gauge IC is 0% if there is no battery cell data update */
        if (tmp_batt_info.percentage == 0)
            val->intval = 1;
        else
#endif
        val->intval = tmp_batt_info.percentage;
        break;
    case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
        val->intval = tmp_batt_info.level;
        break;
    case POWER_SUPPLY_PROP_MANUFACTURER:
    /*case POWER_SUPPLY_PROP_BATTERY_ID:
        val->strval = tmp_batt_info.manufacturer;*/
        break;
    case POWER_SUPPLY_PROP_MODEL_NAME:
        val->strval = tmp_batt_info.model;
        break;
    case POWER_SUPPLY_PROP_SERIAL_NUMBER:
    /*case POWER_SUPPLY_PROP_FIRMWARE_VERSION:
        val->strval = tmp_batt_info.serial_number;*/
        break;
#ifdef CONFIG_ME372CG_BATTERY_BQ27520
    case POWER_SUPPLY_PROP_CHEMICAL_ID:
        val->strval = tmp_batt_info.chemical_id;
        break;
    case POWER_SUPPLY_PROP_FW_CFG_VER:
        val->strval = tmp_batt_info.fw_cfg_version;
        break;
#endif
    case POWER_SUPPLY_PROP_TEMP:
        val->intval = tmp_batt_info.batt_temp;
        break;
    default:
        return -EINVAL;
    }

    return ret;
}

int asus_register_power_supply_charger(struct device *dev,
        struct chgr_dev_func *tbl)
{
    int ret;
    BAT_DBG_E("%s\n", __func__);

    if (!dev) {
        BAT_DBG_E("%s, device pointer is NULL.\n", __func__);
        return -EINVAL;
    }

    mutex_lock(&batt_info_mutex);
    batt_info.tbl_chgr = tbl;
    mutex_unlock(&batt_info_mutex);

    return 0;
}

int asus_register_power_supply(struct device *dev, struct dev_func *tbl)
{
    int ret;
    int test_flag=0;
    drv_status_t drv_status;
    int voltage=0, percentage=0;
    u32 cable_status;

    BAT_DBG_E("%s\n", __func__);

    mutex_lock(&batt_info_mutex);
    drv_status = batt_info.drv_status;
    test_flag = batt_info.test_flag;
    mutex_unlock(&batt_info_mutex);

    if (!dev) {
        BAT_DBG_E("%s, device pointer is NULL.\n", __func__);
        return -EINVAL;
    } else if (!tbl) {
        BAT_DBG_E("%s, dev_func pointer is NULL.\n", __func__);
        return -EINVAL;
    } else if (drv_status != DRV_INIT_OK) {
        BAT_DBG_E("%s, asus_battery not init ok.\n", __func__);
        return -EINVAL;
    } else if (test_flag & TEST_INFO_NO_REG_POWER) {
        BAT_DBG_E("%s, Not register power class.\n", __func__);
        return 0;
    }

    mutex_lock(&batt_info_mutex);
    batt_info.drv_status = DRV_REGISTER;
    batt_info.tbl = tbl;
    mutex_unlock(&batt_info_mutex);

    ret = power_supply_register(dev,
            &asus_power_supplies[CHARGER_BATTERY]);
    if (ret) {
        BAT_DBG_E("Fail to register battery\n");
        goto batt_err_reg_fail_battery;
    }

    /* init wake lock in COS */
    if (entry_mode == 4) {
        BAT_DBG(" %s: wake lock init: asus_battery_power_wakelock\n",
            __func__);
        wake_lock_init(&wakelock,
            WAKE_LOCK_SUSPEND,
            "asus_battery_power_wakelock");
        wake_lock_init(&wakelock_t,
            WAKE_LOCK_SUSPEND,
            "asus_battery_power_wakelock_timeout");
    }

    /* prevent system from entering s3 in COS
        while AC charger is connected
    */
    if (entry_mode == 4) {
        if (batt_info.cable_status == USB_ADAPTER) {
            if (!wake_lock_active(&wakelock)) {
                BAT_DBG(" %s: asus_battery_power_wakelock "
                    "-> wake lock\n", __func__);
                wake_lock_timeout(&wakelock, 180*HZ);
            }
        }
    }

    wake_lock_init(&updating_wakelock, WAKE_LOCK_SUSPEND,
            "asus_battery_power_updating_wakelock");

    /* start battery info polling */
    queue_delayed_work(battery_work_queue,
        &battery_poll_data_work,
        30*HZ);

    mutex_lock(&batt_info_mutex);
    asus_battery_get_info_no_mutex();
    batt_info.drv_status = DRV_REGISTER_OK;
    batt_info.present = 1;
    mutex_unlock(&batt_info_mutex);

    BAT_DBG("%s register OK.\n", __func__);
    return 0;

batt_err_reg_fail_battery:
    return ret;
}
EXPORT_SYMBOL(asus_register_power_supply);

int asus_battery_init(u32 polling_time,
                    u32 critical_polling_time, u32 test_flag)
{
    int ret=0;
    drv_status_t drv_sts;

    BAT_DBG("%s, %d, %d, 0x%08X\n", __func__,
        polling_time, critical_polling_time, test_flag);

    mutex_lock(&batt_info_mutex);
    drv_sts = batt_info.drv_status;
    mutex_unlock(&batt_info_mutex);

    if (drv_sts != DRV_NOT_READY) {
        /* other battery device registered. */
        BAT_DBG_E("Error!! Already registered by other driver\n");
        ret = -EINVAL;
        if (test_flag & TEST_INFO_NO_REG_POWER) {
            ret = 0;
        }

        goto already_init;
    }

    mutex_lock(&batt_info_mutex);
    batt_info.drv_status = DRV_INIT;
    batt_info.polling_time = polling_time > (5*HZ) ?
        polling_time : BATTERY_DEFAULT_POLL_TIME;
    batt_info.critical_polling_time = critical_polling_time > (3*HZ) ?
        critical_polling_time : BATTERY_CRITICAL_POLL_TIME;
    batt_info.critical_polling_time = BATTERY_CRITICAL_POLL_TIME;
    batt_info.percentage = 50;
    batt_info.batt_temp = 250;
    batt_info.test_flag = test_flag;
    if (test_flag)
        BAT_DBG("test_flag: 0x%08X\n", test_flag);
    mutex_unlock(&batt_info_mutex);

    if (test_flag & TEST_INFO_NO_REG_POWER) {
        BAT_DBG_E("Not allow initiallize  power class. Skip ...\n");
        ret = 0;

        mutex_lock(&batt_info_mutex);
        batt_info.drv_status = DRV_INIT_OK;
        mutex_unlock(&batt_info_mutex);

        goto already_init;
    }

    ret = asus_battery_register_proc_fs();
    if (ret) {
        BAT_DBG_E("Unable to create proc file\n");
        goto proc_fail;
    }

    battery_work_queue = create_singlethread_workqueue("asus_battery");
    if (battery_work_queue == NULL) {
        BAT_DBG_E("Create battery thread fail");
        ret = -ENOMEM;
        goto error_workq;
    }

    INIT_DELAYED_WORK(&battery_poll_data_work, asus_polling_data);
    INIT_DELAYED_WORK(&detect_cable_work, USB_cable_status_worker);

#if defined(ME372CG_ENG_BUILD) && !defined(CONFIG_UPI_BATTERY)
    ret = init_asus_charging_toggle();
    if (ret) {
        BAT_DBG_E("Unable to create proc file\n");
        goto proc_fail;
    }
#endif

    /*ret = init_emerg_poll_toggle();
    if (ret) {
        BAT_DBG_E("Unable to create proc file\n");
        goto proc_fail;
    }*/

    mutex_lock(&batt_info_mutex);
    batt_info.drv_status = DRV_INIT_OK;
    mutex_unlock(&batt_info_mutex);

    ws = wakeup_source_register("asus_battery_power_ws");
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
    callback_struct = register_screen_state_notifier(&bq27520_screen_changed_listener);
#endif
    BAT_DBG("%s: success\n", __func__);
    return 0;

error_workq:
proc_fail:
already_init:
    return ret;
}
EXPORT_SYMBOL(asus_battery_init);

void asus_battery_exit(void)
{
    drv_status_t drv_sts;

    BAT_DBG("Driver unload\n");

    mutex_lock(&batt_info_mutex);
    drv_sts = batt_info.drv_status;
    mutex_unlock(&batt_info_mutex);

    if (drv_sts == DRV_REGISTER_OK) {
        power_supply_unregister(&asus_power_supplies[CHARGER_BATTERY]);
        /*if (entry_mode == 4)
            wake_lock_destroy(&wakelock);*/
    }
    BAT_DBG("Driver unload OK\n");
}

MODULE_AUTHOR("chris1_chang@asus.com");
MODULE_DESCRIPTION("battery driver");
MODULE_LICENSE("GPL");

