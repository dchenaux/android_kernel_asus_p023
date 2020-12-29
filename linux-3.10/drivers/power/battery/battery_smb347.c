/*
 * -------------------------------------------------------------------------
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#define SMB345_NAME "smb345_charger"
#define pr_fmt(fmt) SMB345_NAME": "fmt

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/usb/otg.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/pinctrl/consumer.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_bus.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/usb/phy-intel.h>

#include <linux/time.h>
#include <linux/wakelock.h>
#include <linux/pm_wakeup.h>

/*
 * Development debugging is not enabled in release image to prevent
 * loss of event history in the debug array which has a limited size
 */
#include <linux/power/charger_debug.h>

/* I2C communication related */
#define I2C_RETRY_COUNT 3
#define I2C_RETRY_DELAY 5

#define CFG_CHARGE_CURRENT            0x00
#define CFG_CHARGE_CURRENT_FCC_MASK        0xe0
#define CFG_CHARGE_CURRENT_FCC_SHIFT        5
#define CFG_CHARGE_CURRENT_PCC_MASK        0x18
#define CFG_CHARGE_CURRENT_PCC_SHIFT        3
#define CFG_CHARGE_CURRENT_TC_MASK        0x07
#define CFG_CHARGE_CURRENT_ALL        0x41
#define CFG_CURRENT_LIMIT            0x01
#define CFG_CURRENT_LIMIT_DC_MASK        0xf0
#define CFG_CURRENT_LIMIT_DC_SHIFT        4
#define CFG_CURRENT_LIMIT_USB_MASK        0x0f
#define CFG_CURRENT_LIMIT_SMB346_MASK   0xf0
#define CFG_CURRENT_LIMIT_SMB346_VALUE_2000 0x70
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1800 0x60
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1500 0x50
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1200 0x40
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1000 0x30
#define CFG_CURRENT_LIMIT_SMB346_VALUE_700 0x20
#define CFG_CURRENT_LIMIT_SMB346_VALUE_500 0x10
#define IBUS_MIN_LIMIT_MA 100
#define IBUS_LIMIT_STEP_MA 400
#define IBUS_MAX_LIMIT_MA 900
#define FLOAT_VOLTAGE_REG        0x03

#define IOCHARGE_MAX_MA 1500
#define DEFAULT_CC 350
#define DEFAULT_CV 3380

#define MAX_NR_OF_I2C_RETRIES 1
#define CHRGR_WORK_DELAY (10*HZ)
#define BOOST_WORK_DELAY (10*HZ)
#define EVT_WAKELOCK_TIMEOUT (2*HZ)
#define DBG_STATE_FILENAME "charger_state"

/* Pin and enable control */
#define CFG_PIN                    0x06
#define CFG_PIN_EN_CTRL_MASK            0x60
#define CFG_PIN_EN_CTRL_ACTIVE_HIGH        0x40
#define CFG_PIN_EN_CTRL_ACTIVE_LOW        0x60
#define CFG_PIN_EN_APSD_IRQ            BIT(1)
#define CFG_PIN_EN_CHARGER_ERROR        BIT(2)
/* Command registers */
#define CMD_A                    0x30
#define CMD_A_CHG_ENABLED            BIT(1)
#define CMD_A_SUSPEND_ENABLED            BIT(2)
#define CMD_A_OTG_ENABLED            BIT(4)
#define CMD_A_ALLOW_WRITE            BIT(7)
#define CMD_B                    0x31
#define CMD_B_USB9_AND_HC_MODE    0x03
#define CMD_C                    0x33

/* Interrupt Status registers */
#define IRQSTAT_A                0x35
#define IRQSTAT_C                0x37
#define IRQSTAT_C_TERMINATION_STAT        BIT(0)
#define IRQSTAT_C_TERMINATION_IRQ        BIT(1)
#define IRQSTAT_C_TAPER_IRQ            BIT(3)
#define IRQSTAT_D                0x38
#define IRQSTAT_D_CHARGE_TIMEOUT_STAT        BIT(2)
#define IRQSTAT_D_CHARGE_TIMEOUT_IRQ        BIT(3)
#define IRQSTAT_E                0x39
#define IRQSTAT_E_USBIN_UV_STAT            BIT(0)
#define IRQSTAT_E_USBIN_UV_IRQ            BIT(1)
#define IRQSTAT_E_DCIN_UV_STAT            BIT(4)
#define IRQSTAT_E_DCIN_UV_IRQ            BIT(5)
#define IRQSTAT_F                0x3a
#define IRQSTAT_F_OTG_UV_IRQ            BIT(5)
#define IRQSTAT_F_OTG_UV_STAT            BIT(4)

/* Status registers */
#define STAT_A                    0x3b
#define STAT_A_FLOAT_VOLTAGE_MASK        0x3f
#define STAT_B                    0x3c
#define STAT_C                    0x3d
#define STAT_C_CHG_ENABLED            BIT(0)
#define STAT_C_HOLDOFF_STAT            BIT(3)
#define STAT_C_CHG_MASK                0x06
#define STAT_C_CHG_SHIFT            1
#define STAT_C_CHG_TERM                BIT(5)
#define STAT_C_CHARGER_ERROR            BIT(6)
#define STAT_E                    0x3f

/* PMU Register used for POK status */
#define PMU_CONFIG_O    0x00U
    #define PMU_CONFIG(_base) ((_base) + PMU_CONFIG_O)

    #define PMU_CONFIG_CHGDET_O (BIT(1))
    #define PMU_CONFIG_CHGDET_M (BIT(1))

#define CHARGER_CONTROL_O 0x0
#define CHARGER_CONTROL(_base) ((_base) + CHARGER_CONTROL_O)
	#define CHARGER_CONTROL_CIEDG_O 26
	#define CHARGER_CONTROL_CIEDG_M 0x1
	#define CHARGER_CONTROL_CILVL_O 25
	#define CHARGER_CONTROL_CILVL_M 0x1
	#define CHARGER_CONTROL_CISENS_O 24
	#define CHARGER_CONTROL_CISENS_M 0x1
	#define CHARGER_CONTROL_CIEN_O 23
	#define CHARGER_CONTROL_CIEN_M 0x1
	#define CHARGER_CONTROL_CIDBT_O 17
	#define CHARGER_CONTROL_CIDBT_M 0x7
	#define CHARGER_CONTROL_CHGLVL_O 1
	#define CHARGER_CONTROL_CHGLVL_M 0x1
	/*chrg det*/
	#define CHARGER_CONTROL_CDETSENS_O 10
	#define CHARGER_CONTROL_CDETSENS_M 0x1
	#define CHARGER_CONTROL_CHDETLVL_O 6
	#define CHARGER_CONTROL_CHDETLVL_M 0x1
	#define CHARGER_CONTROL_CHDWEN_O 5
	#define CHARGER_CONTROL_CHDWEN_M 0x1

#define CHARGER_CONTROL_CIEDG_FALLING 0
#define CHARGER_CONTROL_CILVL_LOW 0
#define CHARGER_CONTROL_CISENS_EDGE 1
#define CHARGER_CONTROL_CIEN_EN 0
#define CHARGER_CONTROL_CHGLVL_LOW 0
#define CHARGER_CONTROL_IRQ_DEBOUNCE_DISABLE 0
/*chrg det*/
#define CHARGER_CONTROL_CDETSENS_EDGE 1
#define CHARGER_CONTROL_CHDETLVL_LOW 0
#define CHARGER_CONTROL_CHDETLVL_HIGH 1
#define CHARGER_CONTROL_CHDWEN_EN 1

#define CHARGER_CONTROL_WR_O 0x8
#define CHARGER_CONTROL_WR(_base) ((_base) + CHARGER_CONTROL_WR_O)
	#define CHARGER_CONTROL_WR_WS_O 0
	#define CHARGER_CONTROL_WR_WS_M 0x1

#define SMB345_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))
#define OTG_TLIM_THERM_CNTRL_REG                0x0A
#define OTG_CURRENT_LIMIT_AT_USBIN_MASK    SMB345_MASK(2, 2)
#define OTG_CURRENT_LIMIT_750mA    (BIT(2) | BIT(3))
#define OTG_CURRENT_LIMIT_500mA    BIT(3)
#define OTG_CURRENT_LIMIT_250mA    BIT(2)
#define OTG_BATTERY_UVLO_THRESHOLD_MASK    SMB345_MASK(2, 0)
#define FLOAT_VOLTAGE_MASK                SMB345_MASK(6, 0)

#define fit_in_range(__val, __MIN, __MAX) ((__val > __MAX) ? __MAX : \
					(__val < __MIN) ? __MIN : __val)

#define SYSFS_FAKE_VBUS_SUPPORT 1

#define SYSFS_INPUT_VAL_LEN (1)
struct workqueue_struct *charger_work_queue = NULL;
struct delayed_work charger_work;

/* ================================== ASUS ================================== */
#include <linux/debugfs.h>
#include <linux/HWVersion.h>
#include <linux/proc_fs.h>
#include "smb345_external_include.h"
#include "ti/asus_battery.h"
#include <linux/pinctrl/pinctrl.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#define COVER_ATTACH_GPIO 49
#define MULT_NP_DET 16
#define USB_HS_ID 58
static struct pinctrl *ch_enable_pinctrl;
static struct pinctrl *otg_enable_pinctrl;

extern const char* Read_HW_ID_STR(void);
extern int Read_HW_ID(void);
extern void upi_ug31xx_attach(bool);
extern bool COVER_ATTACHED(void);
extern bool COVER_ATTACHED_UPI(void);
extern bool CHARGING_FULL(void);
extern bool VBUS_IN(void);
extern bool DCIN_VBUS(void);
extern bool IS_CA81(void);
extern bool IS_CB81(void);
extern bool disable_cover_otg(void);
extern int  smb3xxc_pre_config(bool initial, bool cover_changed_cable_changed);
extern int  smb3xxc_soc_control_jeita(void);
extern int  smb3xxc_jeita_control_for_sw_gauge(int usb_state);
extern int  smb345c_charging_toggle(charging_toggle_level_t level, bool on);
extern int  smb345c_dump_registers(struct seq_file *s);
extern void cover_aicl(int I_USB_IN);
extern void cover_usbin(bool turn_on);
extern void cover_otg_current(int curr);
extern void cover_otg(int on);
extern bool disable_cover_otg(void);
extern void upi_ug31xx_attach(bool);
extern bool get_mipi_state(void);

#define INTEL_STYLE 0
#define CFG_ADDRESS		0x0e
static struct dentry *dentry;
static struct chgr_dev_func smb345_tbl;
static int g_usb_state = CABLE_OUT;
static bool g_charging_toggle = true;
static int HW_ID;
extern int entry_mode;
static void smb345_config_max_current(int usb_state, bool cover_changed_cable_changed);
static void smb3xx_config_max_current(int usb_state, bool cover_changed_cable_changed);
void pad_cover_aicl(int pad_bat_rsoc, int cover_rsoc);
void cover_otg_control(void);
void cover_with_sdp(int pad_bat_rsoc, int cover_rsoc, bool cover_changed_cable_changed);
static bool g_is_power_supply_registered = false;
static int g_Flag1 = 0;
static int g_Flag2 = 0;
static int g_Flag3 = 0;
static bool upi_ug31xx_attach_state = false;

static const char *getCableCoverString[] = {
    "USB_IN",
    "AC_IN",
    "CABLE_OUT",
    "ENABLE_5V",
    "DISABLE_5V",
    "COVER_ATTACH",
    "COVER_DETTACH",
};

/* global charger type variable lock */
DEFINE_MUTEX(g_usb_state_lock);
static int g_cover_state = COVER_DETTACH;
static int g_extern_device = CABLE_OUT;
/* global software charging toggle lock */
DEFINE_MUTEX(g_charging_toggle_lock);
/* AICL Results Table (mA) : 3Fh */

static const unsigned int aicl_results[] = {
#if defined(CONFIG_Z580C)
	/* smb358 */
	300, 500, 700, 1000, 1200, 1300, 1800, 2000
#else
	/* PF400CG, Z300C/CG, Z380C adopted smb345, smb346, smb347 */
	300, 500, 700, 900, 1200, 1500, 1800, 2000, 2200, 2500
#endif
};

struct wake_lock wlock_t;
static spinlock_t spinlock_suspend;
static bool suspend_smb = false;
static bool suspend_work_not_done = false;
#ifdef ME372CG_ENG_BUILD
bool eng_suspend_mode = false;
static bool suspend_mode(bool);
static ssize_t nasus_suspend_mode_toggle_write(struct file *file,
    const char __user *buf, size_t count, loff_t * ppos)
{
	char proc_buf[64] = {0};

	if (count > sizeof(proc_buf)) {
		BAT_DBG("%s: data error\n", __func__);
		return -EINVAL;
	}
	if (copy_from_user(proc_buf, buf, count)) {
		BAT_DBG("%s: read data from user space error\n", __func__);
		return -EFAULT;
	}

	BAT_DBG_E(" %s: %s", __func__, proc_buf);

	if (!strncmp("1", proc_buf, 1)) {
		/* turn on suspend mode in eng build */
		eng_suspend_mode = true;
		BAT_DBG_E(" %s: turn on suspend mode", __func__);
	}
	else if (!strncmp("0", proc_buf, 1)) {
		/* turn off suspend mode in eng build */
		eng_suspend_mode = false;
		BAT_DBG_E(" %s: turn off suspend mode", __func__);
	}

	suspend_mode(eng_suspend_mode);

	return count;
}
static int nasus_suspend_mode_toggle_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%s: Suspend mode %s\n", __func__, eng_suspend_mode ? "On" : "Off");
	return 0;
}
static int nasus_suspend_mode_toggle_open(struct inode *inode, struct file *file)
{
	return single_open(file, nasus_suspend_mode_toggle_read, NULL);
}

int init_asus_charging_limit_toggle(void)
{
	/* interface for usb-in suspend mode */
	static const struct file_operations suspend_mode_toggle_fops = {
		.owner = THIS_MODULE,
		.open = nasus_suspend_mode_toggle_open,
		.read = seq_read,
		.write = nasus_suspend_mode_toggle_write,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *entry = NULL;

	entry = proc_create("driver/charger_suspend_mode", 0666, NULL,
		&suspend_mode_toggle_fops);
	if (!entry) {
		BAT_DBG_E("Unable to create suspend_mode\n");
		return -EINVAL;
	}

	return 0;
}
#else
static int twinsheaddragon_read(struct seq_file *m, void *v)
{
    seq_printf(m, "%s: Enable\n", __func__);

    setSMB345Charger(AC_IN);

    return 0;
}
static int twinsheaddragon_open(struct inode *inode, struct file *file)
{
    return single_open(file, twinsheaddragon_read, NULL);
}
int init_asus_charging_limit_toggle(void)
{
    struct proc_dir_entry *entry;

    static const struct file_operations twinsheaddragon_fops = {
        .owner = THIS_MODULE,
        .open = twinsheaddragon_open,
        .read = seq_read,
        .write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
    };

    entry = proc_create("driver/twinsheaddragon", 0666, NULL,
        &twinsheaddragon_fops);
    if (!entry) {
        BAT_DBG_E("Unable to create twinsheaddragon\n");
        return -EINVAL;
    }

    return 0;
}
#endif
/* ================================== ASUS ================================== */

enum {
	POK_B_VALID = 0,
	POK_B_INVAL,

	BOOSTOV_OCCURED = 1,

	BATUV_OCCURED = 1,

	TSD_OCCURRED = 1,

	OVP_OCCURRED = 1,

	OVP_RECOV_OCCURRED = 1,

	T32_TO_OCCURRED = 1,

	VBUS_FAULT = 1,

	TREG_IS_ON = 1,

	OT_RECOV_OCCURRED = 1,

	VBUS_OFF = 0,
	VBUS_ON,
};

enum charger_status {
	SMB345_STATUS_UNKNOWN,
	SMB345_STATUS_READY,
	SMB345_STATUS_FAULT,
};

/**
 * struct smb345_state - SMB345 charger current state
 * @status		charger driver status. Please see enum definition for
 *			details.
 * @vbus		equals 'VBUS_ON' (1) if valid vbus connected
 *			otherwise is 'VBUS_OFF'. '-1' -  means uninitialized.
 * @cc			current output current [mA] set on HW.
 * @max_cc		maximum output current [mA] that can be set on HW.
 * @cv			current output voltage [mV] set on HW.
 * @iterm		HW charging termination current [mA]. Not used as
 *			termination is controlled by SW.
 * @inlmt		input current limit [mA].
 * @health		charger chip health.
 * @cable_type		type of currently attached cable.
 * @charger_enabled	informs if charger is enabled for use.
 * @charging_enabled	informs if charging is currently enabled or not.
 * @to_enable_boost	informs whether boost is to be enabled
 * @boost_enabled	informs if boost mode is enabled
 * @pok_b		value of POK_B signal
 * @ovp_flag		value of OVP flag bit
 * @ovp_recov		value of OVP_RECOV flag bit
 * @t32s_timer_expired	charging watchdog timer expiration flag
 * @vbus_fault		'1' if vbus is fault, '0' otherwise
 * @treg_flag		charger thermal regulation active flag
 * @ot_recov_flag	over temperature recovery flag
 * @tsd_flag		thermal shutdown fault
 * @bat_uv		Battery voltage below 2.7v
 * @boost_ov		Boost out of regulation due to sustained current limit.
 */
struct smb345_state {
	enum charger_status status;
	int vbus;
	int cc;
	int max_cc;
	int cv;
	int iterm;
	int inlmt;
	int health;
	int cable_type;
	bool charger_enabled;
	bool charging_enabled;
	bool to_enable_boost;
	bool boost_enabled;
	unsigned int pok_b:1;
	unsigned int ovp_flag:1;
	unsigned int ovp_recov:1;
	unsigned int t32s_timer_expired:1;
	unsigned int vbus_fault:1;
	unsigned int treg_flag:1;
	unsigned int ot_recov_flag:1;
	unsigned int tsd_flag:1;

	/* Boost mode specific states */
	unsigned int bat_uv:1;
	unsigned int boost_ov:1;

};

/**
 * struct unfreezable_bh_struct -  structure describing suspend aware bottom half
 * @wq				pointer to  i2c client's structure.
 * @work			work associated with irq.
 * @evt_wakelock		wakelock acquired when bottom half is scheduled.
 * @in_suspend			Suspend flag. 'true' if driver was suspended
 *				'false' otherwise.
 * @pending_evt			pending event for bottom half execution flag.
 * @lock			spinlock protecting in_suspend and pending_evt
 *				flags.
 */
struct	unfreezable_bh_struct {
	struct workqueue_struct *wq;
	struct delayed_work work;
	struct wake_lock evt_wakelock;
	bool in_suspend;
	bool pending_evt;
	spinlock_t lock;
};


/**
 * struct smb345_charger - SMB345 charger driver internal structure
 * @client			pointer to  i2c client's structure
 * @ididev			pointer to idi device
 * @chgint_bh			structure describing bottom half of
 *				CHGINT irq. See structure definition for
 *				details.
 * @boost_op_bh			structure describing bottom half of boost
 *				enable/disable operation.
 * @charging_work		work providing charging heartbeat for PSC
 * @boost_work			work feeding watchdog during boost mode
 * @otg_handle			Pointer to USB OTG internal structure
 *				used for sending VBUS notifications.
 * @usb_psy			power supply instance struct for USB path
 * @ac_psy			power supply instance struct for AC path
 * @current_psy			pointer to psy representing current charging
 *				path.
 * @prop_lock			synchronization semaphore
 * @model_name			model name of charger chip
 * @manufacturer		manufacturer name of charger chip
 * @ack_time			last CONTINUE_CHARGING timestamp in jiffies
 * @otg_nb			OTG notifier block
 * @fake_vbus			value of fake vbus event
 * @state			charger state structure
 */
struct smb345_charger {
	struct i2c_client *client;
	struct idi_peripheral_device *ididev;

	struct unfreezable_bh_struct chgint_bh;
	struct unfreezable_bh_struct boost_op_bh;

	struct delayed_work charging_work;
	struct delayed_work boost_work;
	struct resource *ctrl_io_res;
	struct usb_phy *otg_handle;

	struct power_supply usb_psy;
	struct power_supply ac_psy;

	struct power_supply *current_psy;

	struct semaphore prop_lock;
	struct wake_lock suspend_lock;
	const char *model_name;
	const char *manufacturer;

	unsigned long ack_time;
	struct notifier_block otg_nb;

	int fake_vbus;
	struct smb345_state state;
	int chgint_irq;
	int chgdet_irq;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct pinctrl_state *ch_en_pins_default;
	struct pinctrl_state *ch_en_pins_sleep;
	struct device_pm_platdata *pm_platdata;
    charger_jeita_status_t charger_jeita_status;
    struct delayed_work aicl_dete_work;

	int chg_otg_en_gpio;
	struct power_supply_cable_props *props;
	unsigned long chgint_debounce;

	/* wake lock to prevent S3 during charging */
	struct wake_lock wakelock;
	struct wakeup_source *ws;
	int usb_acok, dcin_ok;
	int acok_irq, dcin_irq, vbusin_irq;

	int smb_chg_en_gpio;
	int vbus_in_det_n_gpio;
    struct delayed_work cover_detect_work;
    struct work_struct  chrgr_type_work;
};

static int cover_type_report(struct notifier_block *nb, unsigned long event, void *ptr);
static struct notifier_block cover_type_notifier = {
    .notifier_call = cover_type_report,
};
extern int cover_cable_status_register_client(struct notifier_block *nb);
extern int cover_cable_status_unregister_client(struct notifier_block *nb);
static int charger_type_report(struct notifier_block *nb, unsigned long event, void *ptr);
static struct notifier_block charger_type_notifier = {
    .notifier_call = charger_type_report,
};
extern int cable_status_register_client(struct notifier_block *nb);
extern int cable_status_unregister_client(struct notifier_block *nb);

static int smb345_otg_notification_handler(struct notifier_block *nb,
		unsigned long event, void *data);

static struct smb345_charger chrgr_data = {
	.model_name = "SMB345",
	.manufacturer = "TBD",

	.otg_nb = {
		.notifier_call = smb345_otg_notification_handler,
	},

	.chgint_bh = {
		.in_suspend = false,
		.pending_evt = false,
	},
	.boost_op_bh = {
		.in_suspend = false,
		.pending_evt = false,
	},
	.fake_vbus = -1,

	.state = {
		.status = SMB345_STATUS_UNKNOWN,
		.vbus = -1,

	},
	.charger_jeita_status = ROOM,
};

static int smb345_set_pinctrl_state(struct i2c_client *client,
		struct pinctrl_state *state)
{
	struct smb345_charger *chrgr = i2c_get_clientdata(client);
	int ret;

	ret = pinctrl_select_state(chrgr->pinctrl, state);
	if (ret != 0) {
		pr_err("failed to configure CHGRESET pin !\n");
		return -EIO;
	}

	return 0;
}

void GPIO_OTG_TOGGLE(bool toggle)
{
	struct pinctrl_state *state;
	if (toggle) {
		state = pinctrl_lookup_state(otg_enable_pinctrl, "otg_on");
		if (IS_ERR_OR_NULL(state))
			pr_err("could not get default pinstate\n");
		else if(pinctrl_select_state(otg_enable_pinctrl, state))
			pr_err("%d:could not set pins\n", __LINE__);
		else
			pr_info("set to OTG on\n");
	}
	else {
		state = pinctrl_lookup_state(otg_enable_pinctrl, "otg_off");
		if (IS_ERR_OR_NULL(state))
			pr_err("could not get sleep pinstate\n");
		else if(pinctrl_select_state(otg_enable_pinctrl, state))
			pr_err("%d:could not set pins\n", __LINE__);
		else
			pr_info("set to OTG off\n");
	}
}

void COVER_CHARGING_TOGGLE(bool toggle)
{
    struct pinctrl_state *state;
	if(toggle) {
	    state = pinctrl_lookup_state(ch_enable_pinctrl, "cover_chg_ena");
        if (IS_ERR_OR_NULL(state))
            pr_err("could not get default pinstate\n");
        else if(pinctrl_select_state(ch_enable_pinctrl, state))
            pr_err("%d:could not set pins\n", __LINE__);
        else
            BAT_DBG(" %s: set to PINCTRL_STATE_DEFAULT: ON\n", __func__);
    }
	else {
	    state = pinctrl_lookup_state(ch_enable_pinctrl, "cover_chg_dis");
        if (IS_ERR_OR_NULL(state))
            pr_err("could not get sleep pinstate\n");
        else if(pinctrl_select_state(ch_enable_pinctrl, state))
            pr_err("%d:could not set pins\n", __LINE__);
        else
            BAT_DBG(" %s: set to PINCTRL_STATE_SLEEP: OFF\n", __func__);
    }
}

static int smb345_enable_charging(struct smb345_charger *chrgr,
		bool enable, int ma);
static int smb345_i2c_read_reg(struct i2c_client *client,
		u8 reg_addr, u8 *data);

static int smb345_i2c_write_reg(struct i2c_client *client,
		u8 reg_addr, u8 data);

static struct charger_debug_data chrgr_dbg;

#if INTEL_STYLE
static struct power_supply_throttle smb345_dummy_throttle_states[] = {
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
	},
};

static char *smb345_supplied_to[] = {
		"battery",
};

static enum power_supply_property smb345_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_INLMT,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};
#else
static char *supply_list[] = {
	"battery",
	"pack_ac",
	"pack_bat",
};

static inline int get_battery_rsoc(int *rsoc);
static inline int get_battery_temperature(int *tempr);
static inline int get_battery_voltage(int *volt);
static inline int get_pack_bat_voltage(int *volt);
static inline int get_pack_bat_rsoc(int *rsoc);

static enum power_supply_property asus_power_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb345_power_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val);

static struct power_supply smb345_power_supplies[] = {
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = asus_power_properties,
		.num_properties = ARRAY_SIZE(asus_power_properties),
		.get_property = smb345_power_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = asus_power_properties,
		.num_properties = ARRAY_SIZE(asus_power_properties),
		.get_property = smb345_power_get_property,
	},
	{
		.name = "otg",
		.type = POWER_SUPPLY_TYPE_OTG,
		.properties = asus_power_properties,
		.num_properties = ARRAY_SIZE(asus_power_properties),
		.get_property = smb345_power_get_property,
	},
};
static inline struct power_supply *get_psy_battery(void)
{
    struct class_dev_iter iter;
    struct device *dev;
    static struct power_supply *pst;

    class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
    while ((dev = class_dev_iter_next(&iter))) {
        pst = (struct power_supply *)dev_get_drvdata(dev);
        if (pst->type == POWER_SUPPLY_TYPE_BATTERY) {
            class_dev_iter_exit(&iter);
            return pst;
        }
    }
    class_dev_iter_exit(&iter);

    return NULL;
}

static inline struct power_supply *get_psy_pack_bat(void)
{
    struct class_dev_iter iter;
    struct device *dev;
    static struct power_supply *pst;

    class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
    while ((dev = class_dev_iter_next(&iter))) {
        pst = (struct power_supply *)dev_get_drvdata(dev);
        if (pst->type == POWER_SUPPLY_TYPE_PACK_BATTERY) {
            class_dev_iter_exit(&iter);
            return pst;
        }
    }
    class_dev_iter_exit(&iter);

    return NULL;
}

static inline int get_battery_temperature(int *tempr)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_battery();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
    if (!ret)
        *tempr = val.intval;

    return ret;
}

static inline int get_battery_voltage(int *volt)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_battery();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
    if (!ret)
        *volt = val.intval / 1000;

    return ret;
}

static inline int get_battery_rsoc(int *rsoc)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_battery();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
    if (!ret)
        *rsoc = val.intval;

    return ret;
}

static inline int get_pack_bat_voltage(int *volt)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_pack_bat();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
    if (!ret)
        *volt = val.intval / 1000;

    return ret;
}

static inline int get_pack_bat_rsoc(int *rsoc)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_pack_bat();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
    if (!ret)
        *rsoc = val.intval;

    return ret;
}
#endif

#ifdef SYSFS_FAKE_VBUS_SUPPORT

/**
 * fake_vbus_show	Called when value queried from driver to sysfs
 * @dev			[in] pointer to device
 * @attr		[in] pointer to devices's attribute
 * @buf			[out] pointer to buffer that is to be filled with
 *			string. Passed from driver to sysfs
 *
 * Returns:		number of characters read
 */
static ssize_t fake_vbus_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct smb345_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));
	size_t size_copied;
	int value;

	value = chrgr->fake_vbus;
	size_copied = sprintf(buf, "%d\n", value);

	return size_copied;
}

/**
 * fake_vbus_store	Called when value written from sysfs to driver
 * @dev			[in] pointer to device
 * @attr		[in] pointer to devices's attribute
 * @buf			[in] pointer to buffer containing string passed
 *			from sysfs
 * @count		[in] number of characters in string
 *
 * Returns:		number of characters written
 */
static ssize_t fake_vbus_store(
			struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct smb345_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));
	int sysfs_val;
	int ret;
	size_t size_to_cpy;
	char strvalue[SYSFS_INPUT_VAL_LEN + 1];

	size_to_cpy = (count > SYSFS_INPUT_VAL_LEN) ?
				SYSFS_INPUT_VAL_LEN : count;

	strncpy(strvalue, buf, size_to_cpy);
	strvalue[size_to_cpy] = '\0';

	ret = kstrtoint(strvalue, 10, &sysfs_val);
	if (ret != 0)
		return ret;

	sysfs_val = (sysfs_val == 0) ? 0 : 1;

	down(&chrgr->prop_lock);

	chrgr->fake_vbus = -1;

	if (chrgr->state.vbus != 1) {
		pr_err("fake vbus event requested when USB cable removed !\n");
		up(&chrgr->prop_lock);
		return count;
	}

	if (sysfs_val == 0) {
		chrgr->fake_vbus = 0;
		atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
				USB_EVENT_VBUS, &chrgr->fake_vbus);

		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_FAKE_VBUS,
						chrgr->fake_vbus, 0);
		pr_info("fake vbus removal sent.\n");

	} else if (sysfs_val == 1 && !chrgr->state.charging_enabled) {
		chrgr->fake_vbus = 1;
		atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
				USB_EVENT_VBUS, &chrgr->fake_vbus);

		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_FAKE_VBUS,
						chrgr->fake_vbus, 0);
		pr_info("fake vbus connection sent\n");
	}

	up(&chrgr->prop_lock);

	return count;
}

static struct device_attribute smb345_fake_vbus_attr = {
	.attr = {
		.name = "fake_vbus_event",
		.mode = S_IRUSR | S_IWUSR,
	},
	.show = fake_vbus_show,
	.store = fake_vbus_store,
};

/**
 * smb345_setup_sysfs_attr	Sets up sysfs entries for smb345 i2c device
 * @chrgr			[in] pointer to charger driver internal
 *				structure
 */
static void smb345_setup_fake_vbus_sysfs_attr(struct smb345_charger *chrgr)
{
	struct device *dev = &chrgr->client->dev;
	int err;

	err = device_create_file(dev, &smb345_fake_vbus_attr);
	if (err)
		pr_err("Unable to create sysfs entry: '%s'\n",
				smb345_fake_vbus_attr.attr.name);
}

#else

static inline void smb345_setup_fake_vbus_sysfs_attr(
					struct smb345_charger *chrgr)
{
	(void) chrgr;
}

#endif /*SYSFS_FAKE_VBUS_SUPPORT*/

static int smb345_i2c_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret;
	struct smb345_charger *chrgr  = i2c_get_clientdata(client);

	wake_lock(&chrgr_data.wakelock);
	ret = i2c_smbus_read_byte_data(chrgr->client, reg);
	wake_unlock(&chrgr_data.wakelock);
	if (ret < 0) {
		dev_warn(&chrgr->client->dev,
				"i2c read fail: can't read reg 0x%02X: %d\n",
				reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int smb345_i2c_write_reg(struct i2c_client *client,
		u8 reg, u8 val)
{
	int ret;
	struct smb345_charger *chrgr = i2c_get_clientdata(client);

	wake_lock(&chrgr_data.wakelock);
	ret = i2c_smbus_write_byte_data(chrgr->client, reg, val);
	wake_unlock(&chrgr_data.wakelock);
	if (ret < 0) {
		dev_err(&chrgr->client->dev,
				"i2c write fail: can't write %02X to %02X: %d\n",
				val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb345_masked_read(struct i2c_client *client, int reg, u8 mask)
{
    s32 rc;
    u8 temp;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
        rc = smb345_i2c_read_reg(client, reg, &temp);
        if (rc) {
            retry_count--;
            pr_err("*smb345_i2c_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
            msleep(I2C_RETRY_DELAY);
        }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("smb345_i2c_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        return -1;
    }

    temp &= mask;
    return temp;
}

static int smb345_masked_write(struct i2c_client *client, int reg,
                u8 mask, u8 val)
{
    s32 rc;
    u8 temp;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
        rc = smb345_i2c_read_reg(client, reg, &temp);
        if (rc) {
            retry_count--;
            pr_err("*smb345_i2c_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
            msleep(I2C_RETRY_DELAY);
        }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("smb345_i2c_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        return rc;
    }

    temp &= ~mask;
    temp |= val & mask;

    retry_count = I2C_RETRY_COUNT;
    do
    {
        rc = smb345_i2c_write_reg(client, reg, temp);
        if (rc) {
            retry_count--;
            pr_err("*smb345_write failed: reg=%03X, rc=%d\n", reg, rc);
            msleep(I2C_RETRY_DELAY);
        }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("smb345_write failed: reg=%03X, rc=%d\n", reg, rc);
        return rc;
    }

    return 0;
}

static int smb345_set_writable(struct smb345_charger *chrgr, bool writable)
{
	int ret;
	u8 val = 0;

	ret = smb345_i2c_read_reg(chrgr->client, CMD_A, &val);
	if (ret != 0)
		return ret;

	if (writable)
		val |= CMD_A_ALLOW_WRITE;
	else
		val &= ~CMD_A_ALLOW_WRITE;

	return smb345_i2c_write_reg(chrgr->client, CMD_A, val);
}

int smb345_get_soc_control_float_vol(int bat_tempr, int bat_volt)
{
    int ret;

    /* read 03h[5:0]="011110" or "101010"*/
    ret = smb345_masked_read(chrgr_data.client,
                FLOAT_VOLTAGE_REG,
                FLOAT_VOLTAGE_MASK);

    BAT_DBG_E(" Charger get battery info T:%d, V:%d, 03h[5:0]=" BYTETOBINARYPATTERN "\n",
        bat_tempr, bat_volt, BYTETOBINARY(ret));
    return ret;
}

static int __maybe_unused smb345_set_voreg(struct smb345_charger *chrgr,
				int volt_to_set, int *volt_set, bool propagate)
{
	return 0;
}

static int __maybe_unused smb345_set_iocharge(
			struct smb345_charger *chrgr, int curr_to_set,
						int *curr_set, bool propagate)
{
	return 0;
}

static int __maybe_unused smb345_set_ibus_limit(struct smb345_charger *chrgr,
						int ilim_to_set, int *ilim_set)
{
	return 0;
}

static int __maybe_unused smb345_charger_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	return 0;
}

static int __maybe_unused smb345_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	return 0;
}

static void smb345_set_boost(struct work_struct *work)
{
	int ret;
	struct smb345_charger *chrgr = &chrgr_data;
	int on = chrgr->state.to_enable_boost;

	pr_info("%s(): %s\n", __func__, (on) ? "enable" : "disable");

	ret = smb345_set_writable(chrgr, true);
	if (ret < 0)
		goto boost_fail;


#if INTEL_STYLE
	if (on) {
		/* I2C disable OTG function */
		ret = smb345_masked_write(chrgr->client, CMD_A, BIT(4), 0);
		if (ret) {
			pr_err("fail to set OTG Disable bit ret=%d\n", ret);
			goto boost_fail;
		}

		/* Switching Frequency change 1.5Mhz 07h[7]="1" */
		ret = smb345_masked_write(chrgr->client, CFG_CHARGE_CURRENT_TC_MASK,
				BIT(7), BIT(7));
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			goto boost_fail;
		}

		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb345_masked_write(chrgr->client,
				OTG_TLIM_THERM_CNTRL_REG, OTG_CURRENT_LIMIT_AT_USBIN_MASK, 0);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			goto boost_fail;
		}

		/* Set OTG battery UVLO threshold to 2.7V: 0Ah[1:0]="00" */
		ret = smb345_masked_write(chrgr->client,
				OTG_TLIM_THERM_CNTRL_REG, OTG_BATTERY_UVLO_THRESHOLD_MASK, 0);
		if (ret) {
			pr_err("fail to set OTG battery UVLO threshold to 2.7V ret=%d\n",
					ret);
			goto boost_fail;
		}

		/* Toggle to enable OTG function: output low */
		gpio_direction_output(chrgr->chg_otg_en_gpio, 1);

		/* Set OTG current limit to 500mA: 0Ah[3:2]="01" */
		ret = smb345_masked_write(chrgr->client, OTG_TLIM_THERM_CNTRL_REG,
				BIT(2) | BIT(3), BIT(2));

		if (ret) {
			pr_err("fail to set OTG current limit 500mA ret=%d\n", ret);
			goto boost_fail;
		}
	} else {
		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb345_masked_write(chrgr->client,
				OTG_TLIM_THERM_CNTRL_REG, OTG_CURRENT_LIMIT_AT_USBIN_MASK, 0);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			goto boost_fail;
		}

		/* Toggle to enable OTG function: output high */
		gpio_direction_output(chrgr->chg_otg_en_gpio, 0);
	}
#else
	if (ret)
		goto boost_fail;
#endif

	return;
boost_fail:
	pr_err("%s: fail to boost\n", __func__);
}
/**
 * smb345_chgint_cb_work_func	function executed by
 *				smb345_charger::chgint_cb_work work
 * @work			[in] pointer to associated 'work' structure
 */
static void smb345_chgint_cb_work_func(struct work_struct *work)
{
	u32 regval;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	struct smb345_charger *chrgr = &chrgr_data;
	int ret, vbus_state_prev;
	u8 status;
	int i;

	down(&chrgr->prop_lock);
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_CHGINT_CB, 0, 0);

	vbus_state_prev = chrgr->state.vbus;

	pm_state_en = idi_peripheral_device_pm_get_state_handler(chrgr->ididev, "enable");

	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		goto chgint_fail;
	}

	pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(
				chrgr->ididev, "disable");

	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		goto chgint_fail;
	}

	ret = idi_set_power_state(chrgr->ididev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		goto chgint_fail;
	}

#if INTEL_STYLE
	ret = smb345_i2c_read_reg(chrgr->client, STAT_C, &status);
	chrgr->state.vbus = (status & STAT_C_CHG_ENABLED) ? VBUS_ON : VBUS_OFF;
#else
	chrgr->state.vbus = gpio_get_value(chrgr->vbus_in_det_n_gpio) ? VBUS_OFF : VBUS_ON;
#endif

	ret = idi_set_power_state(chrgr->ididev, pm_state_dis, false); /*FIXME DO I NEED THIS */
	if (ret)
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);

     /* If vbus status changed, then notify USB OTG */
	if (chrgr->state.vbus != vbus_state_prev) {
		if (chrgr->otg_handle) {
			pr_info("%s: vbus changed: %d\n", __func__, chrgr->state.vbus);
			atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
					USB_EVENT_VBUS, &chrgr->state.vbus);

			CHARGER_DEBUG_REL(
					chrgr_dbg, CHG_DBG_VBUS, chrgr->state.vbus, 0);
		}
	}

	/* XXX: CHGDET configuration */
	ret = idi_client_ioread(chrgr->ididev,
			CHARGER_CONTROL(chrgr->ctrl_io_res->start), &regval);

	/* charger detection CHARGER_CONTROL_CHDETLVL*/
	regval &= ~(CHARGER_CONTROL_CHDETLVL_M << CHARGER_CONTROL_CHDETLVL_O);

	if (chrgr->state.vbus) {
		regval |= (CHARGER_CONTROL_CHDETLVL_HIGH << CHARGER_CONTROL_CHDETLVL_O);
	} else {
		regval |= (CHARGER_CONTROL_CHDETLVL_LOW << CHARGER_CONTROL_CHDETLVL_O);
	}

	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL(chrgr->ctrl_io_res->start), regval);

	/* charger control WR strobe */
	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL_WR(chrgr->ctrl_io_res->start),
			BIT(CHARGER_CONTROL_WR_WS_O));

	/* This second triggering of the write strobe is intentional
	 * to make sure CHARGER_CTRL value is propagated to HW properly
	 */
	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL_WR(chrgr->ctrl_io_res->start),
			BIT(CHARGER_CONTROL_WR_WS_O));

chgint_fail:
	up(&chrgr->prop_lock);
}

static int unfreezable_bh_create(struct unfreezable_bh_struct *bh,
		const char *wq_name, const char *wakelock_name,
		void (*work_func)(struct work_struct *work))
{
	spin_lock_init(&bh->lock);

	/* Create private, single-threaded workqueue instead of using one of
	the system predefined workqueues to reduce latency */
	bh->wq = create_singlethread_workqueue(wq_name);

	if (NULL == bh->wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&bh->work, work_func);

	wake_lock_init(&bh->evt_wakelock,
			WAKE_LOCK_SUSPEND,
			wakelock_name);

	return 0;
}

static void unfreezable_bh_destroy(struct unfreezable_bh_struct *bh)
{
	cancel_delayed_work_sync(&bh->work);

	destroy_workqueue(bh->wq);

	wake_lock_destroy(&bh->evt_wakelock);
}

static void unfreezable_bh_schedule(struct unfreezable_bh_struct *bh)
{
	struct smb345_charger *chrgr = &chrgr_data;
	spin_lock(&bh->lock);

	if (!bh->in_suspend) {
		queue_delayed_work(bh->wq, &bh->work, chrgr->chgint_debounce);

		wake_lock_timeout(&bh->evt_wakelock, EVT_WAKELOCK_TIMEOUT);
	} else {
		bh->pending_evt = true;
	}

	spin_unlock(&bh->lock);
}

static void __maybe_unused unfreezable_bh_resume(struct unfreezable_bh_struct *bh)
{
	unsigned long flags;
	struct smb345_charger *chrgr = &chrgr_data;

	spin_lock_irqsave(&bh->lock, flags);

	bh->in_suspend = false;
	if (bh->pending_evt) {
		bh->pending_evt = false;

		queue_delayed_work(bh->wq, &bh->work, chrgr->chgint_debounce);

		wake_lock_timeout(&bh->evt_wakelock, EVT_WAKELOCK_TIMEOUT);
	}

	spin_unlock_irqrestore(&bh->lock, flags);
}

static void __maybe_unused unfreezable_bh_suspend(struct unfreezable_bh_struct *bh)
{
	unsigned long flags;

	spin_lock_irqsave(&bh->lock, flags);
	bh->in_suspend = true;
	spin_unlock_irqrestore(&bh->lock, flags);
}

static irqreturn_t smb345_charger_chgint_cb(int irq, void *dev)
{
#if INTEL_STYLE
	struct smb345_charger *chrgr = dev;

	unfreezable_bh_schedule(&chrgr->chgint_bh);
#endif
	return IRQ_HANDLED;
}

/**
 * smb345_configure_pmu_irq - function configuring PMU's Charger status IRQ
 * @chrgr		[in] pointer to charger driver internal structure
 */
static int smb345_configure_pmu_irq(struct smb345_charger *chrgr)
{
	int ret;

#if 0
	/* register callback with PMU for CHGINT irq */
	ret = request_irq(chrgr->chgint_irq,
		smb345_charger_chgint_cb,
			IRQF_NO_SUSPEND, SMB345_NAME, chrgr);
	if (ret != 0) {
		pr_err("Failed to register @PMU for GHGINT irq! ret=%d", ret);
		return ret;
	}
#endif
	/* register callback with PMU for CHGDET irq */
	ret = request_irq(chrgr->chgdet_irq,
		smb345_charger_chgint_cb,
			IRQF_NO_SUSPEND, SMB345_NAME, chrgr);
	if (ret != 0) {
		pr_err("Failed to register @PMU for GHGINT irq! ret=%d", ret);
		return ret;
	}

	return 0;
}

static int smb345_configure_pmu_regs(struct smb345_charger *chrgr)
{
	u32 regval;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	int ret;

	if (!chrgr->ctrl_io_res || !chrgr->ididev)
		return -EINVAL;

	pm_state_en =
		idi_peripheral_device_pm_get_state_handler(
				chrgr->ididev, "enable");
	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		return -EINVAL;
	}

	pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(
				chrgr->ididev, "disable");
	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		return -EINVAL;
	}

	pr_info("Getting PM state handlers: OK\n");

	ret = idi_set_power_state(chrgr->ididev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		return -EIO;
	}

	ret = idi_client_ioread(chrgr->ididev,
			CHARGER_CONTROL(chrgr->ctrl_io_res->start), &regval);


	/* ChargerResetLevel - CHARGER_CONTROL_CHGLVL_LOW */
	regval &= ~(CHARGER_CONTROL_CHGLVL_M << CHARGER_CONTROL_CHGLVL_O);
	regval |= (CHARGER_CONTROL_CHGLVL_LOW << CHARGER_CONTROL_CHGLVL_O);

	/* charger detection CHARGER_CONTROL_CDETSENS*/
	regval &= ~(CHARGER_CONTROL_CDETSENS_M << CHARGER_CONTROL_CDETSENS_O);
	regval |= (CHARGER_CONTROL_CDETSENS_EDGE << CHARGER_CONTROL_CDETSENS_O);

	/* charger detection CHARGER_CONTROL_CHDETLVL*/
	regval &= ~(CHARGER_CONTROL_CHDETLVL_M << CHARGER_CONTROL_CHDETLVL_O);
	regval |= (CHARGER_CONTROL_CHDETLVL_LOW << CHARGER_CONTROL_CHDETLVL_O);

	/* charger detection CHARGER_CONTROL_CHDWEN*/
	regval &= ~(CHARGER_CONTROL_CHDWEN_M << CHARGER_CONTROL_CHDWEN_O);
	regval |= (CHARGER_CONTROL_CHDWEN_EN << CHARGER_CONTROL_CHDWEN_O);

	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL(chrgr->ctrl_io_res->start), regval);

	/* charger control WR strobe */
	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL_WR(chrgr->ctrl_io_res->start),
			BIT(CHARGER_CONTROL_WR_WS_O));

	/* This second triggering of the write strobe is intentional
	 * to make sure CHARGER_CTRL value is propagated to HW properly
	 */
	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL_WR(chrgr->ctrl_io_res->start),
			BIT(CHARGER_CONTROL_WR_WS_O));

	ret = idi_set_power_state(chrgr->ididev, pm_state_dis, false);

	if (ret)
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);

	return 0;
}

static void do_charger(struct work_struct *work)
{
	struct smb345_charger *chrgr = &chrgr_data;

	down(&chrgr->prop_lock);

	pr_info("%s(), ma = %u, type = %d, connect = %d\n",
			__func__, chrgr->props->ma, chrgr->props->chrg_type,
			chrgr->props->chrg_evt);

	if (chrgr->props->chrg_evt == POWER_SUPPLY_CHARGER_EVENT_CONNECT) {
		chrgr->state.cable_type = chrgr->props->chrg_type;
		switch (chrgr->props->chrg_type) {
		case POWER_SUPPLY_CHARGER_TYPE_NONE:
			break;

		case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
			break;
		case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
			smb345_enable_charging(chrgr, true, chrgr->props->ma);

			break;

		default:
			break;

		}
	} else if (chrgr->props->chrg_evt
			== POWER_SUPPLY_CHARGER_EVENT_DISCONNECT) {
		smb345_enable_charging(chrgr, false, 0);
	}
	up(&chrgr->prop_lock);
}

static int smb345_write(struct smb345_charger *chrgr, u8 reg, u8 val)
{
	int ret;
	int retry_count = I2C_RETRY_COUNT;

	do {
		wake_lock(&chrgr_data.wakelock);
		ret = i2c_smbus_write_byte_data(chrgr->client, reg, val);
		wake_unlock(&chrgr_data.wakelock);
		if (ret < 0) {
			retry_count--;
			dev_warn(&chrgr->client->dev,
					"fail to write reg %02xh: %d\n",
					reg, ret);
			msleep(I2C_RETRY_DELAY);
		}
	} while (ret < 0 && retry_count > 0);

	return ret;
}

static int smb345_read(struct smb345_charger *chrgr, u8 reg)
{
	int ret;
	int retry_count = I2C_RETRY_COUNT;

	do {
		wake_lock(&chrgr_data.wakelock);
		ret = i2c_smbus_read_byte_data(chrgr->client, reg);
		wake_unlock(&chrgr_data.wakelock);
		if (ret < 0) {
			retry_count--;
			dev_warn(&chrgr->client->dev, "fail to read reg %02xh: %d\n",
					reg, ret);
			msleep(I2C_RETRY_DELAY);
		}
	} while (ret < 0 && retry_count > 0);

	return ret;
}
static int smb345_set_current_limits(struct smb345_charger *chrgr,
		int current_limit, bool is_twinsheaded)
{
	int ret;
	ret = smb345_set_writable(chrgr, true);
	if (ret < 0) {
		pr_info("%s() set writable as true failed, return %d\n",
				__func__, ret);
		return ret;
	}

	pr_info("%s()-cable_type = %d, current_limit = %d\n",
			__func__, chrgr->state.cable_type, current_limit);
	if (chrgr->state.cable_type == POWER_SUPPLY_CHARGER_TYPE_USB_SDP
			&& current_limit > 500) {
		pr_info("USB IN but current limit > 500, return\n");
		return 0;
	}

	switch (current_limit) {
	case 500:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_500);
		break;
	case 700:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_700);
		break;
	case 1000:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_1000);
		break;

	case 1200:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_1200);
		break;

	case 1500:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_1500);
		break;

	case 1800:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_1800);
		break;

	case 2000:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_2000);
		break;
	default:
		break;
	}
	return ret;
}

static int smb345_enable_charging(struct smb345_charger *chrgr,
		bool enable, int ma)
{
	int ret = 0;

#if !INTEL_STYLE
	return ret;
#endif

	ret = smb345_set_writable(chrgr, true);
	if (ret < 0) {
		pr_err("%s() set writable as true failed, return %d\n",
				__func__, ret);
		goto fail;
	}

	pr_info("%s(), enable = %d for type=%d, ma=%d\n",
			__func__, enable, chrgr->state.cable_type, ma);
	if (enable == true) {
		if (ma < 500)
			return 0;

		/* config charge voltage as 4.3v */
		ret = smb345_masked_write(chrgr->client,
					FLOAT_VOLTAGE_REG,
					BIT(0) | BIT(1) | BIT(2) |
					BIT(3) | BIT(4) | BIT(5),
					BIT(0) | BIT(1) | BIT(5));


		/* charger enable */
		ret = smb345_read(chrgr, CFG_PIN);
		if (ret < 0)
			goto fail;

		ret &= ~CFG_PIN_EN_CTRL_MASK;
		if (enable) {
			/*
			 * Set Pin active low
			 * (ME371MG connect EN to GROUND)
			 */
			ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
		} else {
		}
		ret = smb345_write(chrgr, CFG_PIN, ret);
		if (ret < 0)
			goto fail;


		if (smb345_set_current_limits(chrgr, ma, false) < 0) {
			dev_err(&chrgr->client->dev,
				"%s: fail to set max current limits\n",
				__func__);
			goto fail;
		}

#if INTEL_STYLE
		if (chrgr->state.cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_SDP) {
			power_supply_changed(&chrgr->usb_psy);
		} else if (chrgr->state.cable_type
				== POWER_SUPPLY_CHARGER_TYPE_USB_CDP ||
				chrgr->state.cable_type
				== POWER_SUPPLY_CHARGER_TYPE_USB_DCP) {
			power_supply_changed(&chrgr->ac_psy);
		}
#endif

	} else {
#if INTEL_STYLE
		power_supply_changed(&chrgr->ac_psy);
		power_supply_changed(&chrgr->usb_psy);
#endif
	}

fail:
	return ret;
}

static int smb345_otg_notification_handler(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct smb345_charger *chrgr = &chrgr_data;
	chrgr->props = (struct power_supply_cable_props *)data;

	switch (event) {
	case USB_EVENT_CHARGER:
		queue_delayed_work(charger_work_queue, &charger_work, 0);
		break;
	case INTEL_USB_DRV_VBUS:
		pr_info("%s() INTEL_USB_DRV_VBUS\n", __func__);
		if (!data)
			return NOTIFY_BAD;
		chrgr->state.to_enable_boost = *((bool *)data);
		unfreezable_bh_schedule(&chrgr->boost_op_bh);
		break;

	default:
		break;
	}
	return NOTIFY_OK;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/* USBIN suspend mode */
static bool suspend_mode(bool toggle)
{
	int ret;

	if (!chrgr_data.client) {
		pr_err("Warning: I2C client is null\n");
		return false;
	}

	pr_info("%s:", __func__);
	ret = smb345_set_writable(&chrgr_data, true);

	if (toggle) {
		/* enter suspend mode                                                 */
		/* Input to System FET (Suspend) On/Off Control 02h[7]="1"            */
		/* USBIN Suspend Mode 30h[2]="1"                                      */
		/* DCIN Suspend Mode 31h[2]="1"                                       */
		ret = smb345_masked_write(chrgr_data.client,
			0x02,
			BIT(7),
			BIT(7));
		if (ret) {
			pr_err("fail to set Input to System FET (Suspend) ret=%d\n", ret);
			return ret;
		}
		ret = smb345_masked_write(chrgr_data.client,
			0x30,
			BIT(2),
			BIT(2));
		if (ret) {
			pr_err("fail to set USBIN Suspend Mode ret=%d\n", ret);
			return ret;
		}
		ret = smb345_masked_write(chrgr_data.client,
			0x31,
			BIT(2),
			BIT(2));
		if (ret) {
			pr_err("fail to set DCIN Suspend Mode ret=%d\n", ret);
			return ret;
		}
	}
	else {
		/* leave suspend mode                                                 */
		/* Input to System FET (Suspend) On/Off Control 02h[7]="1"            */
		/* USBIN Suspend Mode 30h[2]="0"                                      */
		/* DCIN Suspend Mode 31h[2]="0"                                       */
		ret = smb345_masked_write(chrgr_data.client,
			0x02,
			BIT(7),
			BIT(7));
		if (ret) {
			pr_err("fail to set Input to System FET (Suspend) ret=%d\n", ret);
			return ret;
		}
		ret = smb345_masked_write(chrgr_data.client,
			0x30,
			BIT(2),
			0);
		if (ret) {
			pr_err("fail to set USBIN Suspend Mode ret=%d\n", ret);
			return ret;
		}
		ret = smb345_masked_write(chrgr_data.client,
			0x31,
			BIT(2),
			0);
		if (ret) {
			pr_err("fail to set DCIN Suspend Mode ret=%d\n", ret);
			return ret;
		}
	}
}


/* Convert register value to current using lookup table */
static int hw_to_current(const unsigned int *tbl, size_t size, unsigned int val)
{
	if (val >= size)
		return tbl[size-1];;
	return tbl[val];
}

/* Acquire the value of AICL Results in Status Register E (3Fh)
   return the current value (unit: mA)
*/
static int get_aicl_results(void)
{
    int ret;

    ret = smb345_read(&chrgr_data, STAT_E);
    if (ret < 0) {
        BAT_DBG_E(" %s: fail to read STAT_E reg\n", __func__);
        return ret;
    }

    ret &= 0x0F;
    return hw_to_current(aicl_results, ARRAY_SIZE(aicl_results), ret);
}

/* Acquire the value of AICL Results in Status Register E (3Fh) */
static ssize_t get_input_current(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct smb345_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));

	ret = smb345_read(chrgr, STAT_E);
	if (ret < 0) {
		pr_info("%s: ERROR: i2c read error\n", __func__);
		return sprintf(buf, "%d\n", -EIO);
	}

	ret &= 0x0F;
	return sprintf(buf, "%d\n",
			hw_to_current(aicl_results, ARRAY_SIZE(aicl_results), ret));
}

/* Acquire the charging status */
static ssize_t get_charge_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = smb345_get_charging_status();
	if (ret == POWER_SUPPLY_STATUS_CHARGING || ret == POWER_SUPPLY_STATUS_FULL)
		ret = 1;
	else
		ret = 0;
	return sprintf(buf, "%d\n", ret);
}

static DEVICE_ATTR(input_current, S_IRUGO, get_input_current, NULL);
static DEVICE_ATTR(charge_status, S_IRUGO, get_charge_status, NULL);
static struct attribute *dev_attrs[] = {
	&dev_attr_input_current.attr,
	&dev_attr_charge_status.attr,
	NULL,
};
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

int smb345_charging_toggle(charging_toggle_level_t level, bool on)
{
    int ret = 0;
    int charging_toggle;
    static bool jeita_want_charging = true;
    static bool balan_want_charging = true;
    static charging_toggle_level_t old_lvl = JEITA;
    char *level_str[] = {
        "BALANCE",
        "JEITA",
        "FLAGS",
    };
	struct smb345_charger *chrgr = &chrgr_data;
    charging_toggle = g_charging_toggle;

    BAT_DBG(" %s: old_lvl:%s, charging_toggle:%s, level:%s, on:%s\n",
        __func__,
        level_str[old_lvl],
        charging_toggle ? "YES" : "NO",
        level_str[level],
        on ? "YES" : "NO");

    /* reset to default if AudioCover/PowerBank doesn't attached */
    if (!COVER_ATTACHED_UPI()) {
        balan_want_charging = true;
    }

    /* do charging or not? */
    if (level != FLAGS) {
        if (on) {
            /* want to start charging? */
            if (level == JEITA) {
                jeita_want_charging = true;
                if (!charging_toggle) {
                    /* want to restart charging? */
                    if (old_lvl == FLAGS) {
                        /* reject the request! someone stop charging before */
                        BAT_DBG_E("%s: * reject RESTART charging to phone! *"
                            "(old_lvl != JEITA)\n",
                            __func__);
                        return -1;
                    }
                    else if (!balan_want_charging) {
                        /* reject the request! someone stop charging before */
                        BAT_DBG_E("%s: * reject RESTART charging to phone! *"
                            "(!balan_want_charging)\n",
                            __func__);
                        return -1;
                    }
                }
            }
            else if (level == BALANCE) {
                balan_want_charging = true;
                if (!charging_toggle) {
                    /* want to restart charging? */
                    if (old_lvl == FLAGS) {
                        /* reject the request! someone stop charging before */
                        BAT_DBG_E("%s: * reject RESTART charging to phone! *"
                            "(old_lvl != BALANCE)\n",
                            __func__);
                        return -1;
                    }                    
                    else if (!jeita_want_charging) {
                        /* reject the request! someone stop charging before */
                        BAT_DBG_E("%s: * reject RESTART charging to phone! *"
                            "(!jeita_want_charging)\n",
                            __func__);
                        return -1;
                    }
                }
            }
            else {
                /* what the hell are you? */
            }
        }
        else {
            /* FLAGS stop charging before. reject RESTOP charging to Cover again */
            if (old_lvl == FLAGS && !charging_toggle) {
                BAT_DBG_E(" %s: * reject STOP charging again to Cover! *\n",
                    __func__);
                return -1;
            }
            /* want to stop charging? just do it! */
            if (level == JEITA)
                jeita_want_charging = false;
            if (level == BALANCE)
                balan_want_charging = false;
        }
    }
    else {
        /* it's the highest level. just do it! */
    }

    /* level value assignment */
    old_lvl = level;

    if (!on)
        BAT_DBG_E(" %s: *** charging toggle: OFF ***\n",
            __func__);
    else
        BAT_DBG(" %s: --------------- charging toggle: ON ---------------\n",
            __func__);

    ret = smb345_set_writable(chrgr, true);
    if (ret < 0)
        return ret;

    /* Config CFG_PIN register */
    ret = smb345_read(chrgr, CFG_PIN);
    if (ret < 0)
        goto out;

    /*
     * Make the charging functionality controllable by a write to the
     * command register unless pin control is specified in the platform
     * data.
     */
    ret &= ~CFG_PIN_EN_CTRL_MASK;
    if (on) {
        /* set Pin Controls - active low (ME371MG connect EN to GROUND) */
        ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
    } else {
        /* Do nothing, 0 means i2c control
            . I2C Control - "0" in Command Register disables charger */
    }

    ret = smb345_write(chrgr, CFG_PIN, ret);
    if (ret < 0)
        goto out;

    g_charging_toggle = on;

out:
    return ret;
}

bool smb345_has_charger_error(void)
{
	int ret;
	struct smb345_charger *chrgr = &chrgr_data;

	ret = smb345_read(chrgr, STAT_C);
	if (ret < 0)
		return true;

	if (ret & STAT_C_CHARGER_ERROR)
		return true;

	return false;
}

int smb345_get_charging_status(void)
{
	int ret, status;
	int irqstat_c;
	struct smb345_charger *chrgr = &chrgr_data;

	ret = smb345_read(chrgr, STAT_C);
	if (ret < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	irqstat_c = smb345_read(chrgr, IRQSTAT_C);
	if (irqstat_c < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	BAT_DBG(" Charging Status: STAT_C:0x%x\n", ret);

	if ((ret & STAT_C_CHARGER_ERROR) ||
		(ret & STAT_C_HOLDOFF_STAT)) {
		/* set to NOT CHARGING upon charger error
		 * or charging has stopped.
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		if ((ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT) {
			/* set to charging if battery is in pre-charge,
			 * fast charge or taper charging mode.
			 */
			status = POWER_SUPPLY_STATUS_CHARGING;
		} else if (ret & STAT_C_CHG_TERM) {
			/* set the status to FULL if battery is not in pre
			 * charge, fast charge or taper charging mode AND
			 * charging is terminated at least once.
			 */
			status = POWER_SUPPLY_STATUS_FULL;
		} else {
			/* in this case no charger error or termination
			 * occured but charging is not in progress!!!
			 */
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
    }

	return status;
}
EXPORT_SYMBOL(smb345_get_charging_status);

static int smb345_power_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val)
{
	int ret = 0;
	int usb_state;
	int chrg_status;

	usb_state = g_usb_state;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			val->intval = (usb_state == USB_IN) ? 1 : 0;
		} else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = (usb_state == AC_IN) ? 1 : 0;
		} else if (psy->type == POWER_SUPPLY_TYPE_OTG) {
			val->intval = gpio_get_value(USB_HS_ID) ? 0 : 1;
		} else {
			ret = -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			/* for ATD test to acquire the status about charger ic */
			if (!smb345_has_charger_error()) {
				val->intval = 1;
				return 0;
			}

			chrg_status = smb345_get_charging_status();
			if (chrg_status == POWER_SUPPLY_STATUS_CHARGING) {
				val->intval = 1;
				return 0;
			} else if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
				val->intval = 1;
				return 0;
			}
			val->intval = 0;
		} else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			if (!smb345_has_charger_error()) {
				val->intval = 1;
				return 0;
			}

			chrg_status = smb345_get_charging_status();
			if (chrg_status == POWER_SUPPLY_STATUS_CHARGING) {
				val->intval = 1;
				return 0;
			}
			else if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
				val->intval = 1;
				return 0;
			}
			val->intval = 0;
		} else if (psy->type == POWER_SUPPLY_TYPE_OTG) {
			val->intval = gpio_get_value(USB_HS_ID) ? 0 : 1;
		} else
			ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

int request_power_supply_changed()
{
    int ret;

    if (!g_is_power_supply_registered)
        return -1;

    power_supply_changed(&smb345_power_supplies[CHARGER_AC-1]);
    power_supply_changed(&smb345_power_supplies[CHARGER_USB-1]);
    power_supply_changed(&smb345_power_supplies[2]);

    return ret;
}

static int smb345_register_power_supply(struct device *dev)
{
	int ret;

	ret = power_supply_register(dev, &smb345_power_supplies[CHARGER_USB-1]);
	if (ret) {
		BAT_DBG_E("Fail to register power supply USB\n");
		goto batt_err_reg_fail_usb;
	}

	ret = power_supply_register(dev, &smb345_power_supplies[CHARGER_AC-1]);
	if (ret) {
		BAT_DBG_E("Fail to register power supply AC\n");
		goto batt_err_reg_fail_ac;
	}

	ret = power_supply_register(dev, &smb345_power_supplies[2]);
	if (ret) {
		BAT_DBG_E("Fail to register power supply OTG\n");
		goto batt_err_reg_fail_ac;
	}

	g_is_power_supply_registered = true;
	BAT_DBG_E("%s(): done\n", __func__);
	return 0;

batt_err_reg_fail_ac:
	power_supply_unregister(&smb345_power_supplies[CHARGER_USB-1]);
batt_err_reg_fail_usb:
	return ret;
}

int smb345_dump_registers(struct seq_file *s)
{
	struct smb345_charger *smb;
	int ret;
	u8 reg;

	smb = &chrgr_data;

	BAT_DBG(" %s:\n", __func__);
	BAT_DBG(" Control registers:\n");
	BAT_DBG(" ==================\n");
	BAT_DBG(" #Addr\t#Value\n");

	for (reg = CFG_CHARGE_CURRENT; reg <= CFG_ADDRESS; reg++) {
		ret = smb345_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", reg, BYTETOBINARY(ret));
	}
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Command registers:\n");
	BAT_DBG(" ==================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Command registers:\n");
		seq_printf(s, "==================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}

	ret = smb345_read(smb, CMD_A);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
		"\n", CMD_A, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
			"\n", CMD_A, BYTETOBINARY(ret));
	ret = smb345_read(smb, CMD_B);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
		"\n", CMD_B, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
			"\n", CMD_B, BYTETOBINARY(ret));
	ret = smb345_read(smb, CMD_C);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
		"\n", CMD_C, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
			"\n", CMD_C, BYTETOBINARY(ret));
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Interrupt status registers:\n");
	BAT_DBG(" ===========================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Interrupt status registers:\n");
		seq_printf(s, "===========================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}
	for (reg = IRQSTAT_A; reg <= IRQSTAT_F; reg++) {
		ret = smb345_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", reg, BYTETOBINARY(ret));
	}
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Status registers:\n");
	BAT_DBG(" =================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Status registers:\n");
		seq_printf(s, "=================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}
	for (reg = STAT_A; reg <= STAT_E; reg++) {
		ret = smb345_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", reg, BYTETOBINARY(ret));
	}

	return 0;
}

static int smb345_debugfs_show(struct seq_file *s, void *data)
{
	seq_printf(s, "Control registers:\n");
	seq_printf(s, "==================\n");
	seq_printf(s, "#Addr\t#Value\n");

	smb345_dump_registers(s);

	return 0;
}

static int smb345_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, smb345_debugfs_show, inode->i_private);
}

static const struct file_operations smb345_debugfs_fops = {
	.open        = smb345_debugfs_open,
	.read        = seq_read,
	.llseek      = seq_lseek,
	.release     = single_release,
};

#if defined(CONFIG_Z380C)
int smb3xx_soc_control_jeita(void)
{
    int ret;

    BAT_DBG(" %s:\n", __func__);

    /* Set Hard Hot Limit as 72 Deg. C 0Bh[5:4]="11" */
    ret = smb345_masked_write(chrgr_data.client,
            0x0B,
            BIT(5) | BIT(4),
            BIT(5) | BIT(4));

    /* Set Soft Hot Limit Behavior as No Response 07h[1:0]="00" */
    ret = smb345_masked_write(chrgr_data.client,
            0x07,
            BIT(1) | BIT(0),
            0);

    /* Set Soft Cold Temperature Limit as No Response 07h[3:2]="00" */
    ret = smb345_masked_write(chrgr_data.client,
            0x07,
            BIT(3) | BIT(2),
            0);

    if (ret)
        pr_err("%s: i2c communication failure!", __func__);

    return ret;
}
#else
int smb3xx_soc_control_jeita(void) { return 0; }
#endif

#if defined(CONFIG_Z380C)
int smb3xx_charger_control_jeita(void)
{
    int ret;

    BAT_DBG(" %s:", __func__);

    /* Set Hard Hot Limit as 53 Deg. C 0Bh[5:4]="00" */
    ret = smb345_masked_write(chrgr_data.client,
            0x0B,
            BIT(5) | BIT(4),
            0);

    /* Set Soft Hot Limit Behavior as Float Voltage Compensation 07h[1:0]="10" */
    ret = smb345_masked_write(chrgr_data.client,
            0x07,
            BIT(1) | BIT(0),
            BIT(1));

    /* Set Soft Cold Temp Limit as Charger Current Compensation 07h[3:2]="01" */
    ret = smb345_masked_write(chrgr_data.client,
            0x07,
            BIT(3) | BIT(2),
            BIT(2));

    /* Charging Enable */
    smb345_charging_toggle(JEITA, true);

    if (ret)
        pr_err("%s: i2c communication failure!", __func__);

    return ret;
}
#else
int smb3xx_charger_control_jeita(void) { return 0; }
#endif

#if defined(CONFIG_Z380C)
int smb3xx_jeita_control(int bat_tempr, int bat_volt)
{
    static charger_jeita_status_t ori_charger_jeita_status = ROOM;
    int ret;

    if (!get_mipi_state()) {
        BAT_DBG(" %s: Screen Off. Charger Control HW JEITA\n", __func__);
        smb3xx_charger_control_jeita();
        goto others;
    }

    smb3xx_soc_control_jeita();

    ori_charger_jeita_status = chrgr_data.charger_jeita_status;

    /* condition judgement 1: */
    if (ori_charger_jeita_status == FREEZE) {
        if (bat_tempr >= 45)
            /* control diagram II: */
            goto control_diagram_II;
        else
            /* control diagram I: */
            goto control_diagram_I;
    }

    /* condition judgement 2: */
    if (ori_charger_jeita_status == COLD) {
        if (bat_tempr >= 180) {
            /* control diagram III: */
            goto control_diagram_III;
        }
        else if (bat_tempr < 15) {
            /* control diagram I: */
            goto control_diagram_I;
        }
        else {
            /* control diagram II: */
            goto control_diagram_II;
        }
    }

    /* condition judgement 3: */
    if (ori_charger_jeita_status == ROOM) {
        if (bat_tempr < 150) {
            /* control diagram II: */
            goto control_diagram_II;
        }
        else if (bat_tempr >= 500) {
            if (bat_volt >= 4100) {
                /* control diagram V: */
                goto control_diagram_V;
            }
            else {
                /* control diagram IV: */
                goto control_diagram_IV;
            }
        }
        else
            /* control diagram III: */
            goto control_diagram_III;
    }

    /* condition judgement 4: */
    if (ori_charger_jeita_status == HOT) {
        // state before is "Charging Disable"
        if (smb345_get_soc_control_float_vol(bat_tempr, bat_volt) == 0x2B) {
            if (bat_tempr < 470) {
                /* control diagram III: */
                goto control_diagram_III;
            }
            else if (bat_tempr > 550) {
                /* control diagram VI: */
                goto control_diagram_VI;
            }
            else {
                if (bat_volt >= 4100) {
                    /* control diagram V: */
                    goto control_diagram_V;
                }
                else {
                    /* control diagram IV: */
                    goto control_diagram_IV;
                }
            }
        }
        else { // state before is "Charging Enable"
            if (bat_tempr < 470) {
                /* control diagram III: */
                goto control_diagram_III;
            }
            else if (bat_tempr > 550) {
                /* control diagram VI: */
                goto control_diagram_VI;
            }
            else {
                /* control diagram IV: */
                goto control_diagram_IV;
            }
        }
    }

    /* condition judgement 5: */
    if (ori_charger_jeita_status == OVERHEAT) {
        if (bat_tempr < 520) {
            /* control diagram IV: */
            goto control_diagram_IV;
        }
        else {
            /* control diagram VI: */
            goto control_diagram_VI;
        }
    }

    BAT_DBG_E(" %s: *** ERROR ***\n", __func__);
    goto others;

control_diagram_I:
    ret = smb345_masked_write(chrgr_data.client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5)          | BIT(3)          | BIT(1));
    ret = smb345_masked_write(chrgr_data.client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(5));
    ret = smb345_charging_toggle(JEITA, false);
    chrgr_data.charger_jeita_status = FREEZE;
    if (ori_charger_jeita_status != chrgr_data.charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            chrgr_data.charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_DISCHARGING;

control_diagram_II:
    ret = smb345_masked_write(chrgr_data.client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5)          | BIT(3)          | BIT(1));
    ret = smb345_masked_write(chrgr_data.client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(5));
    ret = smb345_charging_toggle(JEITA, true);
    chrgr_data.charger_jeita_status = COLD;
    if (ori_charger_jeita_status != chrgr_data.charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            chrgr_data.charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_CHARGING;

control_diagram_III:
    ret = smb345_masked_write(chrgr_data.client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5)          | BIT(3)          | BIT(1));
    ret = smb345_masked_write(chrgr_data.client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(7)          | BIT(5));
    ret = smb345_charging_toggle(JEITA, true);
    chrgr_data.charger_jeita_status = ROOM;
    if (ori_charger_jeita_status != chrgr_data.charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            chrgr_data.charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_CHARGING;

control_diagram_IV:
    ret = smb345_masked_write(chrgr_data.client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
                     BIT(4) | BIT(3) | BIT(2) | BIT(1));
    ret = smb345_masked_write(chrgr_data.client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(7)          | BIT(5));
    ret = smb345_charging_toggle(JEITA, true);
    chrgr_data.charger_jeita_status = HOT;
    if (ori_charger_jeita_status != chrgr_data.charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            chrgr_data.charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_CHARGING;

control_diagram_V:
    ret = smb345_masked_write(chrgr_data.client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5)          | BIT(3)          | BIT(1));
    ret = smb345_masked_write(chrgr_data.client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(7)          | BIT(5));
    ret = smb345_charging_toggle(JEITA, false);
    chrgr_data.charger_jeita_status = HOT;
    if (ori_charger_jeita_status != chrgr_data.charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            chrgr_data.charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_DISCHARGING;

control_diagram_VI:
    ret = smb345_masked_write(chrgr_data.client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
                     BIT(4) | BIT(3) | BIT(2) | BIT(1));
    ret = smb345_masked_write(chrgr_data.client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(7)          | BIT(5));
    ret = smb345_charging_toggle(JEITA, false);
    chrgr_data.charger_jeita_status = OVERHEAT;
    if (ori_charger_jeita_status != chrgr_data.charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            chrgr_data.charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_DISCHARGING;

others:
    return POWER_SUPPLY_STATUS_CHARGING;
}
#else
int smb3xx_jeita_control(int bat_tempr, int bat_volt)
{
    return POWER_SUPPLY_STATUS_CHARGING;
}
#endif

int smb3xx_jeita_control_for_sw_gauge(int usb_state)
{
    int ret;
    int batt_tempr = 250;/* unit: C  */
    int batt_volt = 4000;/* unit: mV */

    if (usb_state != AC_IN && usb_state != USB_IN && !COVER_ATTACHED_UPI())
        return 0;

    /* acquire battery temperature here */
    ret = get_battery_temperature(&batt_tempr);
    if (ret) {
        BAT_DBG_E(" %s: fail to get battery temperature\n", __func__);
        return ret;
    }

    /* acquire battery voltage here */
    ret = get_battery_voltage(&batt_volt);
    if (ret) {
        BAT_DBG_E(" %s: fail to get battery voltage\n", __func__);
        return ret;
    }

    /* pad jeita function */
    if (usb_state == AC_IN ||
        (usb_state == USB_IN && !COVER_ATTACHED()) ||
        (COVER_ATTACHED_UPI() && IS_CB81()) ||
        (usb_state == USB_IN && COVER_ATTACHED_UPI() && g_Flag2 == 1))
        ret = smb3xx_jeita_control(batt_tempr, batt_volt);

    /* cover jeita */
    if (COVER_ATTACHED_UPI()) {
        if ((usb_state == AC_IN) || 
            (IS_CB81()) ||
            (usb_state == USB_IN && g_Flag2 == 0))
            ret = smb3xxc_jeita_control_for_sw_gauge(usb_state);
    }

    //request_power_supply_changed();
    return ret;
}

#if defined(CONFIG_Z380C)
static int config_otg_regs(int toggle, bool no_power_cover_test)
{
    int ret;

    if (toggle) {
		/* Toggle to enable OTG function: */
		if (!no_power_cover_test) GPIO_OTG_TOGGLE(1);

        /* Set OTG current limit to 250mA 0Ah[3:2]="01" */
        ret = smb345_masked_write(chrgr_data.client,
            0x0A,
            BIT(3) | BIT(2),
            BIT(2));
        if (ret) {
            pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
            return ret;
        }

        if (!no_power_cover_test) {
        /* set OTG to Pin Control 09h[7:6]="01" */
        ret = smb345_masked_write(chrgr_data.client,
            0x09,
            BIT(7) | BIT(6),
            BIT(6));
        if (ret) {
            pr_err("fail to set OTG enable bit 09h ret=%d\n", ret);
            return ret;
        }
        }
        else {
            /* set OTG to I2C Control 09h[7:6]="00" */
            ret = smb345_masked_write(chrgr_data.client,
                0x09,
                BIT(7) | BIT(6),
                0);
            if (ret) {
                pr_err("fail to set OTG I2C control ret=%d\n", ret);
                return ret;
            }

            /* OTG enable by I2C 30h[4]="1" */
            ret = smb345_masked_write(chrgr_data.client,
                0x30,
                BIT(4),
                BIT(4));
            if (ret) {
                pr_err("fail to set OTG enable by I2C ret=%d\n", ret);
                return ret;
            }
        }

        /* Set OTG current limit to 500mA 0Ah[3:2]="10" */
        ret = smb345_masked_write(chrgr_data.client,
            0x0A,
            BIT(3) | BIT(2),
            BIT(3));
        if (ret) {
            pr_err("fail to set OTG current limit 500mA ret=%d\n", ret);
            return ret;
        }
    }
    else {
        /* Set OTG current limit to 250mA 0Ah[3:2]="01" */
        ret = smb345_masked_write(chrgr_data.client,
            0x0A,
            BIT(3) | BIT(2),
            BIT(2));
        if (ret) {
            pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
            return ret;
        }

		/* Toggle to disable OTG function: */
		if (!no_power_cover_test) GPIO_OTG_TOGGLE(0);

        if (no_power_cover_test) {
            /* OTG disable by I2C 30h[4]="0" */
            ret = smb345_masked_write(chrgr_data.client,
                0x30,
                BIT(4),
                0);
            if (ret) {
                pr_err("fail to set OTG disable by I2C ret=%d\n", ret);
                return ret;
            }

            /* set OTG to Pin Control 09h[7:6]="01" */
            ret = smb345_masked_write(chrgr_data.client,
                0x09,
                BIT(7) | BIT(6),
                BIT(6));
            if (ret) {
                pr_err("fail to set OTG to Pin control ret=%d\n", ret);
                return ret;
            }
        }
    }

    BAT_DBG(" %s: %s\n", __func__, toggle ? "ON" : "OFF");
    return ret;
}
#else
static int config_otg_regs(int toggle, bool no_power_cover_test) { return 0; }
#endif
/*----------------------------------------------------------------------------*/

static int otg(int toggle, bool no_power_cover_test)
{
    int ret;

    ret = smb345_set_writable(&chrgr_data, true);
    if (ret < 0)
        return ret;


    ret = config_otg_regs(toggle, no_power_cover_test);
    if (ret < 0)
        return ret;

    return 0;
}

#if defined(CONFIG_Z380C)
static int aicl_cur_control(int usb_state)
{
    int aicl_result;
    static bool first_enter = true;

    if (!COVER_ATTACHED()) {
        /* ignore non-AC_IN */
        if (usb_state != AC_IN) {
            /* reset to default if non-AC_IN */
            first_enter = true;
            return 0;
        }
        else {
            /* AC_IN */
            if (first_enter) {
                /* don't AICL when first enter */
                first_enter = false;
                return 0;
            }
        }

        aicl_result = get_aicl_results();
        if (aicl_result < 1200) {
            dev_err(&chrgr_data.client->dev,
                "%s: * re-config AC_IN when aicl result(%dmA) < 1200mA.\n",
                __func__,
                aicl_result);
            smb3xx_config_max_current(usb_state, false);
        }
        else {
            BAT_DBG(" %s: execute AICL routine control work\n",
                __func__);
        }
    }
    else first_enter = true;

    return 0;
}
static int pad_cover_aicl_cur_control(int usb_state, bool rsoc_changed, int pad_bat_rsoc, int cover_rsoc)
{
    int aicl_result;
    static bool first_enter = true;

    if (COVER_ATTACHED()) {
        /* ignore non-AC_IN */
        if (usb_state != AC_IN) {
            /* reset to default if non-AC_IN */
            first_enter = true;
            return 0;
        }
        else {
            /* AC_IN */
            if (first_enter) {
                /* don't AICL when first enter */
                first_enter = false;
                return 0;
            }
        }

        aicl_result = get_aicl_results();
        if (aicl_result <= 500) {
            dev_err(&chrgr_data.client->dev,
                "%s: * re-config AC_IN & Cover when aicl result(%dmA) <= 500mA.\n",
                __func__,
                aicl_result);

            /* cover pre config */
            smb3xxc_pre_config(true, true);
            smb345_set_writable(&chrgr_data, true);

            /* disable cover otg */
            if (DCIN_VBUS()) disable_cover_otg();

            /* Enable Cover Charging with DCP */
            COVER_CHARGING_TOGGLE(true);

            /* Enable PAD Charger USBIN 30h[2]="0" */
            if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), 0))
                dev_err(&chrgr_data.client->dev,
                    "%s: fail to set max current limits for USB_IN\n",
                    __func__);

            /* Suspend PAD Charger DCIN 31h[2]="1" */
            if (smb345_masked_write(chrgr_data.client, 0x31, BIT(2), BIT(2)))
                dev_err(&chrgr_data.client->dev,
                    "%s: fail to set max current limits for USB_IN\n",
                    __func__);

            /* check PAD battery rsoc & Cover battery rsoc */
            pad_cover_aicl(pad_bat_rsoc, cover_rsoc);
#if defined(CONFIG_Z380C)
            usb_to_battery_callback(usb_state);
#endif
        }
        else {
            BAT_DBG(" %s: execute AICL routine control work\n",
                __func__);
            if (rsoc_changed) {
                BAT_DBG(" %s: rsoc changed -> A\n", __func__);
                /* check PAD battery rsoc & Cover battery rsoc */
                pad_cover_aicl(pad_bat_rsoc, cover_rsoc);
#if defined(CONFIG_Z380C)
                usb_to_battery_callback(usb_state);
#endif
            }
        }
    }
    else first_enter = true;

    return 0;
}
static int pad_cover_aicl_cur_control_sdp(int usb_state, bool rsoc_changed, int pad_rsoc, int cover_rsoc)
{
    static bool first_enter = true;

    if (COVER_ATTACHED()) {
        /* ignore non-USB_IN */
        if (usb_state != USB_IN) {
            /* reset to default if non-USB_IN */
            first_enter = true;
            return 0;
        }
        else {
            /* USB_IN */
            if (first_enter) {
                /* don't AICL when first enter */
                first_enter = false;
                return 0;
            }
        }

        if (rsoc_changed) {
            BAT_DBG(" %s: rsoc changed -> C\n", __func__);
            /* sdp_ */
            cover_with_sdp(pad_rsoc, cover_rsoc, rsoc_changed);
#if defined(CONFIG_Z380C)
            usb_to_battery_callback(usb_state);
#endif
        }
    }
    else first_enter = true;

    return 0;
}
static int pad_cover_aicl_cur_control_no_cable(int usb_state, bool rsoc_changed, int pad_rsoc, int cover_rsoc)
{
    int aicl_result;
    static bool first_enter = true;
    bool B_state_start = false;

    if (COVER_ATTACHED() && IS_CB81()) {
        if (usb_state != CABLE_OUT) {
            first_enter = true;
            return 0;
        }
        else {
            if (first_enter) {
                first_enter = false;
                return 0;
            }
        }

        if (cover_rsoc < 5 && !VBUS_IN()) {
            disable_cover_otg();
            BAT_DBG(" %s: disable cover otg due to cover rsoc < 5\n");
#if defined(CONFIG_Z380C)
            usb_to_battery_callback(NO_CABLE);
#endif
            msleep(1000);
            smb345_set_writable(&chrgr_data, true);
            return 0;
        }

        aicl_result = get_aicl_results();
        if (aicl_result <= 700) {
            if (g_Flag1 == 0 && pad_rsoc >= 70) {
                BAT_DBG("%s: * No need to re-config when aicl result(%dmA) <= 700mA."
                    " g_Flag1=0, pad_rsoc=%d\n" ,__func__, aicl_result, pad_rsoc);
                return 0;
            }
            BAT_DBG("%s: * re-config when aicl result(%dmA) <= 700mA.\n",
                __func__,
                aicl_result);

            /* cover pre config */
            smb3xxc_pre_config(true, true);

            COVER_CHARGING_TOGGLE(false);
            B_state_start = true;
        }
        else {
            BAT_DBG(" %s: execute AICL routine control work\n", __func__);
        }

        if (B_state_start || rsoc_changed) {
            BAT_DBG(" %s: rsoc changed -> B\n", __func__);
            if (pad_rsoc >= 90 && g_Flag1 == 1) {

                BAT_DBG(" %s: * Suspend Pad USBIN DCIN *\n", __func__);

                /* reset to default */
                g_Flag1 = 0;

                /* USB Suspend Mode: I2C Control 02h[7]="1" */
                if (smb345_masked_write(chrgr_data.client, 0x02, BIT(7), BIT(7)))
                    dev_err(&chrgr_data.client->dev,
                        "%s: fail to set USB Suspend Mode to I2C Control\n",
                        __func__);
                /* Suspend Pad Charger USBIN 30h[2]="1" */
                if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), BIT(2)))
                    dev_err(&chrgr_data.client->dev,
                        "%s: fail to Suspend Pad Charger USBIN\n",
                        __func__);
                /* Suspend Pad Charger DCIN 31h[2]="1" */
                if (smb345_masked_write(chrgr_data.client, 0x31, BIT(2), BIT(2)))
                    dev_err(&chrgr_data.client->dev,
                        "%s: fail to set Suspend Pad Charger DCIN\n",
                    __func__);

                cover_otg(1);
            }
            else if (pad_rsoc < 90 && g_Flag1 == 0) {
                cover_otg_control();
            }
        }
    }
    else first_enter = true;

    return 0;
}
#else
static int aicl_cur_control(int usb_state) { return 0; }
static int pad_cover_aicl_cur_control(int usb_state, bool rsoc_changed, int pad_bat_rsoc, int cover_rsoc) { return 0; }
static int pad_cover_aicl_cur_control_no_cable(int usb_state, bool rsoc_changed, int pad_rsoc, int cover_rsoc) { return 0; }
#endif

void cover_wakelock(void)
{
    if (IS_CB81() || (VBUS_IN() && COVER_ATTACHED_UPI())) {
        if (!wake_lock_active(&wlock_t)) {
            BAT_DBG(" %s: Cover_WakeLock: *LOCK*\n", __func__);
            wake_lock(&wlock_t);
        }
    }
    else {
        if (wake_lock_active(&wlock_t)) {
            BAT_DBG(" %s: Cover_WakeLock: *UNLOCK*\n", __func__);
            wake_unlock(&wlock_t);
        }
    }
}

void aicl_dete_worker(struct work_struct *dat)
{
    int usb_state;
    int rsoc;
    int cover_rsoc;
    static int old_rsoc = -1;
    static int old_cover_rsoc = -1;
    bool rsoc_changed = false;

    /* Do not i2c-5 transmit in the flow of system suspend */
    spin_lock(&spinlock_suspend);
    if (suspend_smb) {
        suspend_work_not_done = true;
        BAT_DBG(" %s: suspend_work_not_done = true!!!\n", __func__);
        spin_unlock(&spinlock_suspend);
        return;
    }
    spin_unlock(&spinlock_suspend);

    __pm_stay_awake(chrgr_data.ws);

    mutex_lock(&g_usb_state_lock);
    usb_state = g_usb_state;
    mutex_unlock(&g_usb_state_lock);

    /* acquire pad battery rsoc here */
    if (get_battery_rsoc(&rsoc)) {
        BAT_DBG(" %s: fail to get battery rsoc\n", __func__);
        goto final;
    }
    else {
        if (old_rsoc != rsoc) {
            BAT_DBG(" %s: * RSOC changed from %d%% to %d%% *\n", __func__, old_rsoc, rsoc);
            old_rsoc = rsoc;
            rsoc_changed = true;
        }
    }

    /* acquire cover battery rsoc */
    if (COVER_ATTACHED_UPI()) {
        if (get_pack_bat_rsoc(&cover_rsoc)) {
            BAT_DBG(" %s: fail to get cover battery rsoc\n", __func__);
            goto final;
        }
        else {
            if (old_cover_rsoc != cover_rsoc || CHARGING_FULL()) {
                BAT_DBG(" %s: * cover RSOC changed from %d%% to %d%% *\n",
                    __func__, old_cover_rsoc, cover_rsoc);
                    old_cover_rsoc = cover_rsoc;
                    rsoc_changed = true;
            }
        }
    }

    if (COVER_ATTACHED_UPI())
        BAT_DBG(" %s: ==========================Pad Rsoc: %d%%, Cover Rsoc: %d%%==========================\n",
            __func__, rsoc, cover_rsoc);
    else
        BAT_DBG(" %s: ==========================Pad Rsoc: %d%%==========================\n",
            __func__, rsoc);

    smb345_set_writable(&chrgr_data, true);
    aicl_cur_control(usb_state);
    pad_cover_aicl_cur_control(usb_state, rsoc_changed, rsoc, cover_rsoc);
    pad_cover_aicl_cur_control_sdp(usb_state, rsoc_changed, rsoc, cover_rsoc);
    pad_cover_aicl_cur_control_no_cable(usb_state, rsoc_changed, rsoc, cover_rsoc);
    smb3xx_jeita_control_for_sw_gauge(usb_state);

    if (rsoc==5 || rsoc==1)
        smb345_dump_registers(NULL);
    if (cover_rsoc==5 || cover_rsoc==1)
        smb345c_dump_registers(NULL);

    cover_wakelock();

    if (dat) {
        if (IS_CB81() && cover_rsoc<=5)
            schedule_delayed_work(&chrgr_data.aicl_dete_work, 5*HZ);
        else
            schedule_delayed_work(&chrgr_data.aicl_dete_work, 30*HZ);
    }

final:
    __pm_relax(chrgr_data.ws);
    spin_lock(&spinlock_suspend);
    suspend_work_not_done = false;
    spin_unlock(&spinlock_suspend);
}
EXPORT_SYMBOL(aicl_dete_worker);

#if defined(CONFIG_Z380C)
static int smb3xx_pre_config(bool cover_changed_cable_changed)
{
    int ret;

    smb345_set_writable(&chrgr_data, true);

    /* set fast charge current: 2000mA */
    /* set pre charge current: 250mA */
    /* set termination current: 200mA */
    /**/
    if (cover_changed_cable_changed)
    ret = smb345_write(&chrgr_data,
            0x00,
            0xbc);
    if (ret < 0)
        goto fail;

    /* set cold soft limit current: 900mA write 0Ah[7:6]="10"*/
    ret = smb345_masked_write(chrgr_data.client,
            0x0a,
            BIT(6) | BIT(7),
            BIT(7));
    if (ret < 0)
        goto fail;

    /* set Battery OV does not end charge cycle. 02h[5]="0", 02h[1]="0" */
    ret = smb345_masked_write(chrgr_data.client,
            0x02,
            BIT(5),
            0);
    if (ret < 0)
        goto fail;
    ret = smb345_masked_write(chrgr_data.client,
            0x02,
            BIT(1),
            0);
    if (ret < 0)
        goto fail;

    /* set Float Voltage: 4.34V. 03h[5:0]="101010" */
    if (cover_changed_cable_changed)
    ret = smb345_masked_write(chrgr_data.client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5) | BIT(3) | BIT(1));
    if (ret < 0)
        goto fail;

    /* set OTG/ID to Pin Control. 09h[7:6]="01" */
    ret = smb345_masked_write(chrgr_data.client,
            0x09,
            BIT(7) | BIT(6),
            BIT(6));
    if (ret < 0)
        goto fail;

    /* USB Suspend Mode: I2C control. 02h[7]="1" */
    ret = smb345_masked_write(chrgr_data.client,
            0x02,
            BIT(7),
            BIT(7));
    if (ret < 0)
        goto fail;

    /* Enable PAD Charger USBIN 30h[2]="0" */
    ret = smb345_masked_write(chrgr_data.client,
            0x30,
            BIT(2),
            0);
    if (ret < 0)
        goto fail;

    /* Enable PAD Charger USBIN 31h[2]="0" */
    ret = smb345_masked_write(chrgr_data.client,
            0x31,
            BIT(2),
            0);
    if (ret < 0)
        goto fail;

fail:
    return ret;
}
#else
static int smb3xx_pre_config(bool cover_changed_cable_changed) { return 0; }
#endif

/*----------------------------------------------------------------------------*/

#if defined(CONFIG_Z380C)
static void smb3xx_config_max_current(int usb_state, bool cover_changed_cable_changed)
{
    if (usb_state == AC_IN && !COVER_ATTACHED()) {
        /* check if AICL Result < 1200mA 3fh[3:0]="0100" */
        if (smb345_masked_read(chrgr_data.client, 0x3f, 0x0f) < 0x04 || cover_changed_cable_changed) {

            /* Disable AICL - Write 02h[4]="0" */
            if (smb345_masked_write(chrgr_data.client, 0x02, BIT(4), 0x00)) {
                dev_err(&chrgr_data.client->dev,
                "%s: fail to disable AICL\n", __func__);
                return;
            }

            /* Set I_USB_IN=1200mA - Write 01h[3:0]="0100" */
            if (smb345_masked_write(chrgr_data.client, 0x01, 0x0f, 0x04)) {
                dev_err(&chrgr_data.client->dev,
                "%s: fail to set max current limits for USB_IN\n",
                __func__);
                return;
            }

            /* Enable AICL - Write 02h[4]="1" */
            if (smb345_masked_write(chrgr_data.client, 0x02, BIT(4), 0x10)) {
                dev_err(&chrgr_data.client->dev,
                "%s: fail to enable AICL\n", __func__);
                return;
            }
        }
    }
}
#else
static void smb3xx_config_max_current(int usb_state) {}
#endif

static void smb345_config_max_current(int usb_state, bool cover_changed_cable_changed)
{
    if (usb_state != AC_IN && usb_state != USB_IN && !COVER_ATTACHED())
        return;

    /* Allow violate register can be written - Write 30h[7]="1" */
    if (smb345_set_writable(&chrgr_data, true) < 0) {
        dev_err(&chrgr_data.client->dev,
        "%s: smb345_set_writable failed!\n", __func__);
        return;
    }

    smb3xx_pre_config(cover_changed_cable_changed);

    pr_info("%s: charger type:%d done.\n", __func__, usb_state);
}

void pad_aicl_dcin(int I_DC_IN)
{
    int reg_val;

    if (I_DC_IN == 700)
        reg_val = BIT(5);
    else if (I_DC_IN == 900)
        reg_val = BIT(5) | BIT(4);
    else
        reg_val = BIT(5) | BIT(4);

    smb345_set_writable(&chrgr_data, true);

    /* Disable AICL - Write 02h[4]="0" */
    if (smb345_masked_write(chrgr_data.client, 0x02, BIT(4), 0)) {
        dev_err(&chrgr_data.client->dev,
            "%s: fail to disable AICL\n", __func__);
        return;
    }

    /* Set I_DC_IN=700mA,900mA - Write 01h[7:4]="0010","0011" */
    if (smb345_masked_write(chrgr_data.client, 0x01, 0xf0, reg_val)) {
        dev_err(&chrgr_data.client->dev,
            "%s: fail to set max current limits for DC_IN\n",
            __func__);
        return;
    }

    /* Enable AICL - Write 02h[4]="1" */
    if (smb345_masked_write(chrgr_data.client, 0x02, BIT(4), BIT(4))) {
        dev_err(&chrgr_data.client->dev,
            "%s: fail to enable AICL\n", __func__);
        return;
    }
}

void pad_aicl(int I_USB_IN)
{
    int reg_val;

    if (I_USB_IN == 700)
        reg_val = BIT(1);
    else if (I_USB_IN == 900)
        reg_val = BIT(0) | BIT(1);
    else if (I_USB_IN == 1200)
        reg_val = BIT(2);
    else
        reg_val = BIT(1);

    smb345_set_writable(&chrgr_data, true);

    /* Disable AICL - Write 02h[4]="0" */
    if (smb345_masked_write(chrgr_data.client, 0x02, BIT(4), 0)) {
        dev_err(&chrgr_data.client->dev,
            "%s: fail to disable AICL\n", __func__);
        return;
    }

    /* Set I_USB_IN=700mA,900mA - Write 01h[3:0]="0010","0011" */
    if (smb345_masked_write(chrgr_data.client, 0x01, 0x0f, reg_val)) {
        dev_err(&chrgr_data.client->dev,
            "%s: fail to set max current limits for USB_IN\n",
            __func__);
        return;
    }

    /* Enable AICL - Write 02h[4]="1" */
    if (smb345_masked_write(chrgr_data.client, 0x02, BIT(4), BIT(4))) {
        dev_err(&chrgr_data.client->dev,
            "%s: fail to enable AICL\n", __func__);
        return;
    }
}

void pad_cover_aicl(int pad_bat_rsoc, int cover_rsoc)
{
    if (g_Flag3 && !COVER_ATTACHED_UPI())
        goto final;

    if (pad_bat_rsoc == 100) {
        BAT_DBG(" %s: BALANCE 1.", __func__);
        /* cover config aicl */
        cover_aicl(1200);
        
        /* Suspend PAD Charger USBIN 30h[2]="1" */
        if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), BIT(2)))
            BAT_DBG("%s: fail to Suspend PAD Charger USBIN\n",
            __func__);

        /* enable Cover Charger USBIN */
        cover_usbin(true);

        /* Pad & Cover charging control */
        smb345_charging_toggle(BALANCE, false);
        smb345c_charging_toggle(BALANCE, true);
        return;
    }

    if (cover_rsoc == 100) {
        BAT_DBG(" %s: BALANCE 2.", __func__);
        /* pad aicl */
        pad_aicl(1200);

        /* enable PAD Charger USBIN 30h[2]="0" */
        if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), 0))
            BAT_DBG("%s: fail to Suspend PAD Charger USBIN\n",
            __func__);

        /* suspend Cover Charger USBIN */
        cover_usbin(false);

        /* Pad & Cover charging control */
        smb345_charging_toggle(BALANCE, true);
        smb345c_charging_toggle(BALANCE, false);
        return;
    }

    if (pad_bat_rsoc > 70 && cover_rsoc < 70) {
        if (pad_bat_rsoc > 90) {
            BAT_DBG(" %s: BALANCE 3.", __func__);
            /* cover config aicl */
            cover_aicl(1200);
        
            /* Suspend PAD Charger USBIN 30h[2]="1" */
            if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), BIT(2)))
                BAT_DBG("%s: fail to Suspend PAD Charger USBIN\n",
                __func__);

            /* enable Cover Charger USBIN */
            cover_usbin(true);

            /* Pad & Cover charging control */
            smb345_charging_toggle(BALANCE, false);
            smb345c_charging_toggle(BALANCE, true);
            return;
        }
        else {
            goto final;
        }
    }

    if (pad_bat_rsoc < 70 && cover_rsoc > 70) {
        if (cover_rsoc > 90) {
            BAT_DBG(" %s: BALANCE 4.", __func__);
            /* pad aicl */
            pad_aicl(1200);

            /* enable PAD Charger USBIN 30h[2]="0" */
            if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), 0))
                BAT_DBG("%s: fail to Suspend PAD Charger USBIN\n",
                __func__);

            /* suspend Cover Charger USBIN */
            cover_usbin(false);

            /* Pad & Cover charging control */
            smb345_charging_toggle(BALANCE, true);
            smb345c_charging_toggle(BALANCE, false);
            return;
        }
        else {
            if (pad_bat_rsoc < 30) {
                BAT_DBG(" %s: BALANCE 5.", __func__);
                /* pad aicl */
                pad_aicl(900);
                /* cover config aicl */
                cover_aicl(300);

                /* enable PAD Charger USBIN 30h[2]="0" */
                if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), 0))
                    BAT_DBG("%s: fail to Suspend PAD Charger USBIN\n",
                    __func__);

                /* eanble Cover Charger USBIN */
                cover_usbin(true);

                /* Pad & Cover charging control */
                smb345_charging_toggle(BALANCE, true);
                smb345c_charging_toggle(BALANCE, true);
                return;
            }
            else {
                goto final;
            }
        }
    }
    else {
            if (pad_bat_rsoc < 30) {
                BAT_DBG(" %s: BALANCE 5.", __func__);
                /* pad aicl */
                pad_aicl(900);
                /* cover config aicl */
                cover_aicl(300);

                /* enable PAD Charger USBIN 30h[2]="0" */
                if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), 0))
                    BAT_DBG("%s: fail to Suspend PAD Charger USBIN\n",
                    __func__);

                /* eanble Cover Charger USBIN */
                cover_usbin(true);

                /* Pad & Cover charging control */
                smb345_charging_toggle(BALANCE, true);
                smb345c_charging_toggle(BALANCE, true);
                return;
            }
            else {
                goto final;
            }
    }

final:
    BAT_DBG(" %s: BALANCE final", __func__);
    /* pad aicl */
    pad_aicl(700);
    /* cover config aicl */
    cover_aicl(500);

    /* enable PAD Charger USBIN 30h[2]="0" */
    if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), 0))
        BAT_DBG("%s: fail to Suspend PAD Charger USBIN\n",
        __func__);

    /* eanble Cover Charger USBIN */
    cover_usbin(true);

    /* Pad & Cover charging control */
    smb345_charging_toggle(BALANCE, true);
    smb345c_charging_toggle(BALANCE, true);
}

void cover_otg_control(void)
{
    BAT_DBG(" %s:", __func__);
    /* Enable PAD Charger DCIN 31h[2]="0" */
    if (smb345_masked_write(chrgr_data.client, 0x31, BIT(2), 0))
        dev_err(&chrgr_data.client->dev,
            "%s: fail to set max current limits for USB_IN\n",
            __func__);

    /* set Cover OTG output current = 900mA and Enable Cover OTG */
    cover_otg_current(900);
    cover_otg(1);

    /* Set Input Current Limit with AICL */
    pad_aicl_dcin(900);
    g_Flag1 = 1;
}

void sdp_charger_control_with_cover(void)
{
    BAT_DBG(" %s:\n", __func__);

    /* Disable Cover Charging with SDP */
    COVER_CHARGING_TOGGLE(false);

    /* Enable PAD Charger USBIN 30h[2]="0" */
    if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), 0))
        BAT_DBG("%s: fail to set max current limits for USB_IN\n",
        __func__);
    /* Enable PAD Charger DCIN 31h[2]="0" */
    if (smb345_masked_write(chrgr_data.client, 0x31, BIT(2), 0))
        BAT_DBG("%s: fail to Enable PAD Charger DCIN\n",
        __func__);

    /* aicl when cover attache with SDP */
    if (smb345_masked_write(chrgr_data.client, 0x02, BIT(4), 0))
        BAT_DBG("%s: fail to Disable AICL\n",
        __func__);
    if (smb345_masked_write(chrgr_data.client, 0x01, 0x0f, BIT(0)))
        BAT_DBG("%s: fail to set Input Current Limit 500mA\n",
        __func__);
    if (smb345_masked_write(chrgr_data.client, 0x02, BIT(4), BIT(4)))
        BAT_DBG("%s: fail to Enable AICL\n",
        __func__);
}

void cover_with_sdp(int pad_bat_rsoc, int cover_rsoc, bool cover_changed_cable_changed)
{
    BAT_DBG(" %s:\n", __func__);

    if (pad_bat_rsoc >= 70) {
        if (g_Flag2 == 1) {
            if (pad_bat_rsoc >= 90) {
                if (cover_rsoc == 100) {
                    g_Flag2 = 1;
                    sdp_charger_control_with_cover();
                }
                else
                    g_Flag2 = 0;
            }
            else {
                // Pad JEITA rule
            }
        }
        else {
            if (cover_rsoc == 100) {
                g_Flag2 = 1;
                sdp_charger_control_with_cover();
            }
            else
                g_Flag2 = 0;
        }
    }
    else { // pad bat rsoc < 70%
        if (g_Flag2 == 0) {
            g_Flag2 = 1;
            sdp_charger_control_with_cover();

            // Pad JEITA rule
        }
        else {
            if (pad_bat_rsoc >= 90) {
                g_Flag2 = 0;
            }
            else {
                // Pad JEITA rule
            }
        }
    }

    if (g_Flag2 == 0) {
        BAT_DBG("%s: * SDP suspend Pad Charger USBIN,DCIN with Cover *\n",
            __func__);

        /* USB Suspend Mode: I2C Control 02h[7]="1" */
        if (smb345_masked_write(chrgr_data.client, 0x02, BIT(7), BIT(7)))
            BAT_DBG("%s: fail to set USB Suspend Mode to I2C Control\n",
                __func__);
        /* Pad Charger USBIN Suspend 30h[2]="1" */
        if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), BIT(2)))
            BAT_DBG("%s: fail to set Pad Charger Suspend\n",
                __func__);
        /* Pad Charger DCIN Suspend 31h[2]="1" */
        if (smb345_masked_write(chrgr_data.client, 0x31, BIT(2), BIT(2)))
            BAT_DBG("%s: fail to set Pad Charger DCIN Suspend\n",
                __func__);

        /* Enable Cover Charging with SDP */
        COVER_CHARGING_TOGGLE(true);

        /* Enable Cover Charger Setting with SDP */
        smb3xxc_pre_config(false, cover_changed_cable_changed);

        /* Cover AICL */
        cover_aicl(500);

        /* start upi */
        if (upi_ug31xx_attach_state == false && COVER_ATTACHED_UPI()) {
            upi_ug31xx_attach_state = true;
            upi_ug31xx_attach(true);
        }

        // cover JEITA rule
    }
}

int setSMB345Charger(int usb_state)
{
    int ret = 0;
    int cover_vbat = 0;
    int cover_rsoc = -1;
    int pad_bat_rsoc = -1;
    static int old_usb_state = CABLE_OUT;
    static int old_cover_state = COVER_DETTACH;
    static bool cover_changed_cable_changed;

    BAT_DBG_E(" =================================== %s"
              " ===================================\n", __func__);
    mutex_lock(&g_usb_state_lock);
    if (usb_state < COVER_ATTACH) {
        g_usb_state = usb_state;
    }
    if (usb_state >= COVER_ATTACH) {
        g_cover_state = usb_state;
    }

    /* Reset to 0 if Cover attach then dettach OR cable insert then remove */
    if (old_usb_state != CABLE_OUT && g_usb_state == CABLE_OUT) {
        g_Flag1 = 0;
        g_Flag2 = 0;
    }
    if (old_cover_state != COVER_DETTACH && g_cover_state == COVER_DETTACH) {
        g_Flag1 = 0;
        g_Flag2 = 0;
    }

    /* detect cable changed or cover changed for JEITA */
    if ((old_usb_state != g_usb_state) || (old_cover_state != g_cover_state))
        cover_changed_cable_changed = true;
    else
        cover_changed_cable_changed = false;
    mutex_unlock(&g_usb_state_lock);

    BAT_DBG("### old_usb_state:%s, g_usb_state:%s, old_cover_state:%s, g_cover_state:%s ###\n",
        getCableCoverString[old_usb_state],
        getCableCoverString[g_usb_state],
        getCableCoverString[old_cover_state],
        getCableCoverString[g_cover_state]);

    if (g_Flag3 && !COVER_ATTACHED_UPI()) {
        mutex_lock(&g_usb_state_lock);
        if (old_cover_state == COVER_DETTACH && g_cover_state == COVER_ATTACH) {
            if (!VBUS_IN()) {
            BAT_DBG("%s: NFC: return (No cable)\n", __func__);
            old_cover_state = g_cover_state;
            mutex_unlock(&g_usb_state_lock);
            return 0;
            }
        }
        else if (old_usb_state == g_usb_state) {
            BAT_DBG("%s: NFC: return (Cable does't change)\n", __func__);
            mutex_unlock(&g_usb_state_lock);
            return 0;
        }
        mutex_unlock(&g_usb_state_lock);
    }

    if (old_cover_state == COVER_DETTACH &&
        g_cover_state == COVER_ATTACH    &&
        COVER_ATTACHED_UPI()){
        cancel_delayed_work(&chrgr_data.aicl_dete_work);
        BAT_DBG(" sleep 4s for UPI gauge driver...\n");
        msleep(4000);
        schedule_delayed_work(&chrgr_data.aicl_dete_work, 10*HZ);
    }

    if (g_Flag3 != 1) {
    ret = get_pack_bat_voltage(&cover_vbat);
    if (ret) {
        /* fail to acquire vbat. reset value to default */
        dev_warn(&chrgr_data.client->dev,
            "%s: fail to acquire cover vbat. reset value to default\n",
            __func__);
        cover_vbat = 0;
        ret = 0;
    }
    ret = get_pack_bat_rsoc(&cover_rsoc);
    if (ret) {
        /* fail to acquire cover rsoc. reset value to default */
        dev_warn(&chrgr_data.client->dev,
            "%s: fail to acquire cover rsoc. reset value to default\n",
            __func__);
        cover_rsoc = -1;
        ret = 0;
    }
    ret = get_battery_rsoc(&pad_bat_rsoc);
    if (ret) {
        /* fail to acquire pad battery rsoc. reset value to default */
        dev_warn(&chrgr_data.client->dev,
            "%s: fail to acquire pad battery rsoc. reset value to default\n",
            __func__);
        pad_bat_rsoc = -1;
        ret = 0;
    }
    else {
        dev_warn(&chrgr_data.client->dev,
            "%s: cover_vbat(%dmV),cover_rsoc(%d%%),pad_bat_rsoc(%d%%)\n",
            __func__,
            cover_vbat, cover_rsoc, pad_bat_rsoc);
    }
    }

    switch (g_usb_state)
    {
    case USB_IN:
        BAT_DBG_E(" usb_state: USB_IN\n");
        smb3xx_pre_config(true);
    case DISABLE_5V:
        if (g_usb_state == DISABLE_5V) {
            BAT_DBG_E(" usb_state: DISABLE_5V\n");
            ret = otg(0, false);
            if (COVER_ATTACHED())
                BAT_DBG_E(" CA81: %s, CB81: %s\n", IS_CA81() ? "Y" : "N", IS_CB81() ? "Y" : "N");
            if (g_is_power_supply_registered)
                power_supply_changed(&smb345_power_supplies[2]);
        }
    case CABLE_OUT:
        if (g_usb_state == CABLE_OUT) {
            BAT_DBG_E(" usb_state: CABLE_OUT\n");
            smb345_charging_toggle(FLAGS, true);
            smb3xx_pre_config(true);
            if (g_cover_state == COVER_DETTACH) {
                COVER_CHARGING_TOGGLE(false);
                otg(0, false);
                break;
            }
        }
    case AC_IN:
        if (g_usb_state == AC_IN) {
            BAT_DBG_E(" usb_state: AC_IN\n");
            smb3xx_pre_config(true);
        }
        if (VBUS_IN() || COVER_ATTACHED()) {

            //mutex_lock(&g_usb_state_lock);
            //smb345_config_max_current(g_usb_state, cover_changed_cable_changed);
            //mutex_unlock(&g_usb_state_lock);

            if (COVER_ATTACHED()) {
                if (!VBUS_IN() && cover_rsoc != -1 && cover_vbat != 0 && cover_rsoc < 5) {
                    disable_cover_otg();
pr_info("10\n");
                }
                else {
                    disable_cover_otg();
pr_info("11\n");
                    smb3xxc_pre_config(true, cover_changed_cable_changed);
                    if (VBUS_IN() && gpio_get_value(USB_HS_ID)) {
                        if (g_Flag3 && !COVER_ATTACHED_UPI()) otg(0, true);
pr_info("12\n");
                        smb345_set_writable(&chrgr_data, true);

                        if (g_usb_state == AC_IN) {
                            if (old_cover_state == COVER_DETTACH && g_cover_state == COVER_ATTACH && IS_CB81()) {
                                pr_info("sleep 2s for DCP...\n");
                                msleep(2000);
                            }
                            /* re-judge again if sleep 2s */
                            if (VBUS_IN() && COVER_ATTACHED() && (g_usb_state == AC_IN)) {
                                /* Enable Cover Charging with DCP */
                                COVER_CHARGING_TOGGLE(true);
                                /* Enable PAD Charger USBIN 30h[2]="0" */
                                if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), 0))
                                    dev_err(&chrgr_data.client->dev,
                                        "%s: fail to set max current limits for USB_IN\n",
                                        __func__);
                                /* Suspend PAD Charger DCIN 31h[2]="1" */
                                if (smb345_masked_write(chrgr_data.client, 0x31, BIT(2), BIT(2)))
                                    dev_err(&chrgr_data.client->dev,
                                        "%s: fail to set max current limits for USB_IN\n",
                                        __func__);
                                /* check PAD battery rsoc & Cover battery rsoc */
                                pad_cover_aicl(pad_bat_rsoc, cover_rsoc);

                                /* start upi */
                                if (upi_ug31xx_attach_state == false && COVER_ATTACHED_UPI()) {
                                    upi_ug31xx_attach_state = true;
                                    upi_ug31xx_attach(true);
                                }
                            }
pr_info("13\n");
                        }
                        else if (g_usb_state == USB_IN) { // SDP
                            if (g_Flag3 == 1 && !COVER_ATTACHED_UPI()) {
pr_info("14a\n");
                                /* NFC device: turn on cable power to NFC Cover */
                                /* set USBIN Suspend Mode = I2C Control 02h[7]=1 */
                                if (smb345_masked_write(chrgr_data.client, 0x02, BIT(7), BIT(7)))
                                    dev_err(&chrgr_data.client->dev,
                                        "%s: fail to set USBIN Suspend Mode to I2C Control\n",
                                        __func__);
                                /* set Pad Charger Suspend 30h[2]=1 */
                                if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), BIT(2)))
                                    dev_err(&chrgr_data.client->dev,
                                        "%s: fail to set Pad Charger Suspend\n",
                                        __func__);
                                /* set Pad Charger DCIN Suspend 31h[2]=1 */
                                if (smb345_masked_write(chrgr_data.client, 0x31, BIT(2), BIT(2)))
                                    dev_err(&chrgr_data.client->dev,
                                        "%s: fail to set Pad Charger DCIN Suspend\n",
                                        __func__);
                                COVER_CHARGING_TOGGLE(true);
                            }
                            else {
                                cover_with_sdp(pad_bat_rsoc, cover_rsoc, cover_changed_cable_changed);
                            }
                        }
                        else {
                            // CABLE_OUT ? VBUS_IN detect
pr_info("23\n");
                            msleep(2000);
                            if (!VBUS_IN() && g_usb_state == CABLE_OUT) {
                                if (COVER_ATTACHED() && IS_CB81()) {
                                    if (pad_bat_rsoc < 90)
                                        cover_otg_control();
                                    else {
pr_info("23a\n");
                                        /* reset to default */
                                        g_Flag1 = 0;

                                        /* USB Suspend Mode: I2C Control 02h[7]="1" */
                                        if (smb345_masked_write(chrgr_data.client, 0x02, BIT(7), BIT(7)))
                                            dev_err(&chrgr_data.client->dev,
                                                "%s: fail to set USB Suspend Mode to I2C Control\n",
                                                __func__);
                                        /* Suspend Pad Charger USBIN 30h[2]="1" */
                                        if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), BIT(2)))
                                            dev_err(&chrgr_data.client->dev,
                                                "%s: fail to Suspend Pad Charger USBIN\n",
                                                __func__);
                                        /* Suspend Pad Charger DCIN 31h[2]="1" */
                                        if (smb345_masked_write(chrgr_data.client, 0x31, BIT(2), BIT(2)))
                                            dev_err(&chrgr_data.client->dev,
                                                "%s: fail to set Suspend Pad Charger DCIN\n",
                                            __func__);

                                        cover_otg(1);
                                    }
pr_info("24\n");
                                }
                            }
                            else {
                                pr_info("*********************************************************\n");
                                pr_info("                           BUG                           \n");
                                pr_info("*********************************************************\n");
                            }
                        }
                    }
                    else {
                        COVER_CHARGING_TOGGLE(false);
                        if (IS_CB81()) {
pr_info("18\n");
                            msleep(2000);
                            /* must confirm there is no plugged-in OTG cable */
                            if (!VBUS_IN() && COVER_ATTACHED_UPI() && IS_CB81() && gpio_get_value(USB_HS_ID)) {
                                if (pad_bat_rsoc < 90)
                                    cover_otg_control();
                                else {
pr_info("18a\n");
                                    /* reset to default */
                                    g_Flag1 = 0;

                                    /* USB Suspend Mode: I2C Control 02h[7]="1" */
                                    if (smb345_masked_write(chrgr_data.client, 0x02, BIT(7), BIT(7)))
                                        dev_err(&chrgr_data.client->dev,
                                            "%s: fail to set USB Suspend Mode to I2C Control\n",
                                            __func__);
                                    /* Suspend Pad Charger USBIN 30h[2]="1" */
                                    if (smb345_masked_write(chrgr_data.client, 0x30, BIT(2), BIT(2)))
                                        dev_err(&chrgr_data.client->dev,
                                            "%s: fail to Suspend Pad Charger USBIN\n",
                                            __func__);
                                    /* Suspend Pad Charger DCIN 31h[2]="1" */
                                    if (smb345_masked_write(chrgr_data.client, 0x31, BIT(2), BIT(2)))
                                        dev_err(&chrgr_data.client->dev,
                                            "%s: fail to set Suspend Pad Charger DCIN\n",
                                            __func__);

                                    cover_otg(1);
                                }
                            }
                        }
                        else if (g_Flag3 && !COVER_ATTACHED_UPI() && gpio_get_value(USB_HS_ID) && !VBUS_IN()) {
pr_info("19\n");
                            COVER_CHARGING_TOGGLE(true);
                            otg(1, true);
                        }
pr_info("20\n");
                    }
                }
            }
            else {
pr_info("22\n");
                COVER_CHARGING_TOGGLE(false);

                smb3xx_config_max_current(usb_state, cover_changed_cable_changed);
            }
        }

        /* regard OTG CABLE OUT as CABLE OUT */
        mutex_lock(&g_usb_state_lock);
        if (g_usb_state == DISABLE_5V) {
            old_usb_state = CABLE_OUT;
            g_usb_state = CABLE_OUT;
        }
        mutex_unlock(&g_usb_state_lock);

        request_power_supply_changed();
        break;

    case ENABLE_5V:
        BAT_DBG_E(" usb_state: ENABLE_5V\n");
        BAT_DBG_E(" USB_HS_ID: %s\n", gpio_get_value(USB_HS_ID) ? "H" : "L");

        if (COVER_ATTACHED())
            disable_cover_otg();
        COVER_CHARGING_TOGGLE(false);

        mutex_lock(&g_usb_state_lock);
        if (old_usb_state != ENABLE_5V && g_usb_state == ENABLE_5V) {
            msleep(500);
            ret = otg(1, false);
        }
        mutex_unlock(&g_usb_state_lock);

        if (g_is_power_supply_registered)
            power_supply_changed(&smb345_power_supplies[2]);
        break;

    default:
        BAT_DBG_E(" ERROR: wrong usb state value = %d\n", usb_state);
        ret = 1;
        break;
    }

#if defined(CONFIG_Z380C)
    switch (g_usb_state) {
    case USB_IN:
        usb_to_battery_callback(USB_PC);
        break;
    case CABLE_OUT:
        usb_to_battery_callback(NO_CABLE);
        break;
    case AC_IN:
        usb_to_battery_callback(USB_ADAPTER);
        break;
    }
#endif

    mutex_lock(&g_usb_state_lock);
    /* assign value to old variable */
    old_usb_state = g_usb_state;
    old_cover_state = g_cover_state;
    mutex_unlock(&g_usb_state_lock);

    smb345_dump_registers(NULL);
    smb345c_dump_registers(NULL);

    cover_wakelock();
    return ret;
}

void chrgr_type_work_func(struct work_struct *work)
{ setSMB345Charger(g_extern_device); }

static int charger_type_report(struct notifier_block *nb, unsigned long event, void *ptr)
{
    switch (event) {
    case POWER_SUPPLY_CHARGER_TYPE_NONE:
        BAT_DBG_E("-----------------------------------------------TYPE_NONE\n");
        /* wake lock to prevent system instantly
           enter S3 while it's in resuming flow */
        wake_lock_timeout(&chrgr_data.suspend_lock, HZ);
        g_extern_device = CABLE_OUT;
        schedule_work(&chrgr_data.chrgr_type_work);
        break;
    case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
        BAT_DBG_E("-----------------------------------------------USB_SDP\n");
        g_extern_device = USB_IN;
        schedule_work(&chrgr_data.chrgr_type_work);
        break;
    case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
    case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
    case POWER_SUPPLY_CHARGER_TYPE_USB_ACA:
    case POWER_SUPPLY_CHARGER_TYPE_AC:
        BAT_DBG_E("-----------------------------------------------USB_DCP\n");
        g_extern_device = AC_IN;
        schedule_work(&chrgr_data.chrgr_type_work);
        break;
    case POWER_SUPPLY_CHARGER_TYPE_OTG:
        BAT_DBG_E("-----------------------------------------------OTG\n");
        g_extern_device = ENABLE_5V;
        setSMB345Charger(g_extern_device);
        break;
    case POWER_SUPPLY_CHARGER_TYPE_OTG_ASYNC:
        BAT_DBG_E("-----------------------------------------------OTG-ASYNC\n");
        g_extern_device = ENABLE_5V;
        schedule_work(&chrgr_data.chrgr_type_work);
        break;
    case POWER_SUPPLY_CHARGER_TYPE_OTG_OUT:
        BAT_DBG_E("-----------------------------------------------OTG_OUT\n");
        g_extern_device = DISABLE_5V;
        setSMB345Charger(g_extern_device);
        break;
    case POWER_SUPPLY_CHARGER_TYPE_OTG_OUT_ASYNC:
        BAT_DBG_E("-----------------------------------------------OTG_OUT_ASYNC\n");
        g_extern_device = DISABLE_5V;
        schedule_work(&chrgr_data.chrgr_type_work);
        break;
    default:
        BAT_DBG_E("-----------------------------------------------DAMN *\n");
        break;
    }

    return NOTIFY_OK;
}

static irqreturn_t vbus_in_det_n_interrupt(int irq, void *dev)
{
    struct smb345_charger *chrgr = dev;

	pr_info("*********************** USB Connector ***********************\n");
	unfreezable_bh_schedule(&chrgr->chgint_bh);

    /* wake lock to prevent system instantly
       enter S3 while it's in resuming flow */
    wake_lock_timeout(&chrgr->suspend_lock, HZ);

    pm_runtime_get_sync(&chrgr->client->dev);

    if (gpio_get_value(chrgr->vbus_in_det_n_gpio)) {
        dev_warn(&chrgr->client->dev,
            "%s: >>> VBUS_IN_DET_N pin (HIGH) <<<\n", __func__);

    }
    else {
        dev_warn(&chrgr->client->dev,
            "%s: >>> VBUS_IN_DET_N pin (LOW) <<<\n", __func__);
        if (COVER_ATTACHED_UPI()) {
            disable_cover_otg();
            COVER_CHARGING_TOGGLE(false);
        }
    }

    pm_runtime_put_sync(&chrgr->client->dev);
	return IRQ_HANDLED;
}


static int vbus_in_det_n_gpio_init(struct smb345_charger *smb)
{
    int ret, irq;

    /* request it and configure gpio as input */
    ret = gpio_request(smb->vbus_in_det_n_gpio,"vbus_in_det_n");
    if (ret) {
        dev_err(&smb->client->dev, "gpio_request failed for %d ret=%d\n",
            smb->vbus_in_det_n_gpio, ret);
        return ret;
    }

    if (gpio_get_value(smb->vbus_in_det_n_gpio))
        dev_info(&smb->client->dev, ">>> VBUS_IN_DET_N (HIGH) <<<\n");
    else
        dev_info(&smb->client->dev, ">>> VBUS_IN_DET_N (LOW) <<<\n");

    return 0;

fail_gpio:
    gpio_free(smb->vbus_in_det_n_gpio);
    return ret;
}

static irqreturn_t smb345_inok_interrupt(int irq, void *data)
{
    struct smb345_charger *smb = data;

    /* wake lock to prevent system instantly
       enter S3 while it's in resuming flow */
    wake_lock_timeout(&smb->suspend_lock, HZ);

    if (gpio_get_value(smb->usb_acok)) {
        dev_warn(&smb->client->dev,
            "%s: >>> INOK pin (HIGH) <<<\n", __func__);
    }
    else {
        dev_warn(&smb->client->dev,
            "%s: >>> INOK pin (LOW) <<<\n", __func__);
    }

    return IRQ_HANDLED;
}

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

static void cover_detect_work_func(struct work_struct *work)
{
    if (COVER_ATTACHED_UPI()) {
        disable_cover_otg();

        /* inform UPI for cover attach only if gpio109 dectect low */
        upi_ug31xx_attach_state = true;
        upi_ug31xx_attach(true);
    }
    else { // gpio109 is High
        /* re-schedule it if gpio109 detect High && gpio0 dectect High:
           It's No power cover or Cover with low low battery. We cannot
           inform UPI gauge when Cover attached Due to that there is no
           UPI gauge on No Power Cover(may cause lots of I2C failure).
           If Cover with low low battery, we also cannot inform UPI gauge
           driver when Cover attached due to that UPI gauge is supplied
           by Vbat. It may cause lots of I2C failure if UPI gauge isn't
           power on.
        */
        if (COVER_ATTACHED()) {
            if (!VBUS_IN()) {
                /* It means No cable. Need Pad to power supply to Cover */
                /* enable OTG 5V boost from Pad Battery to Cover */
                BAT_DBG_E(" * turn on OTG 5V boost from Pad to Cover *\n");
                COVER_CHARGING_TOGGLE(true);
                otg(1, true);

                msleep(2000);

                if (COVER_ATTACHED_UPI()) {
                    BAT_DBG_E(" CA81: %s, CB81: %s\n",
                        IS_CA81() ? "Y" : "N",
                        IS_CB81() ? "Y" : "N");

                    BAT_DBG_E(" * turn off OTG 5V boost from Pad to Cover *\n");
                    COVER_CHARGING_TOGGLE(false);
                    otg(0, true);
                }
                else {
                    if (COVER_ATTACHED()) {
                        g_Flag3 = 1;
                        BAT_DBG(" ********************** NFC **********************\n");
                    }
                    else {
                        COVER_CHARGING_TOGGLE(false);
                        otg(0, true);
                    }
                }
            }
            else {
                BAT_DBG(" power from USB cable will supply to Cover");

                /* confirm there is no OTG cable when vbus present */
                if (gpio_get_value(USB_HS_ID)) {
                    /* turn off otg_en */
                    GPIO_OTG_TOGGLE(0);

                    /* turn on cover_chg_en */
                    BAT_DBG_E(" * turn on COVER_CHG_EN from Pad to Cover *\n");
                    COVER_CHARGING_TOGGLE(true);
                    msleep(2000);

                    if (COVER_ATTACHED_UPI()) {
                        BAT_DBG_E(" CA81: %s, CB81: %s\n",
                            IS_CA81() ? "Y" : "N",
                            IS_CB81() ? "Y" : "N");

                        BAT_DBG_E(" * turn off COVER_CHG_EN from Pad to Cover *\n");
                        COVER_CHARGING_TOGGLE(false);
                        msleep(500);
                    }
                    else {
                        BAT_DBG_E(" CA81: %s, CB81: %s\n",
                            IS_CA81() ? "Y" : "N",
                            IS_CB81() ? "Y" : "N");

                        if (COVER_ATTACHED()) {
                            g_Flag3 = 1;
                            BAT_DBG(" ********************** NFC **********************\n");
                        }
                        else {
                            COVER_CHARGING_TOGGLE(false);
                        }
                    }
                }
            }
        }
        else {
            upi_ug31xx_attach_state = false;
            upi_ug31xx_attach(false);
        }
    }
    setSMB345Charger(COVER_ATTACHED() ? COVER_ATTACH : COVER_DETTACH);
}

static int cover_type_report(struct notifier_block *nb, unsigned long event, void *ptr)
{
    switch (event) {
    case COVER_IN:
        BAT_DBG_E("-----------------------------------------------COVER_IN\n");
        /* wait UPI gauge driver ready */
        wake_lock_timeout(&chrgr_data.suspend_lock, 10*HZ);
        cancel_delayed_work(&chrgr_data.cover_detect_work);
        schedule_delayed_work(&chrgr_data.cover_detect_work, 0);
        break;
    case COVER_OUT:
        BAT_DBG_E("-----------------------------------------------COVER_OUT\n");
        g_Flag1 = 0;
        g_Flag2 = 0;
        if (g_Flag3) {
            COVER_CHARGING_TOGGLE(false);
            otg(0, true);
            BAT_DBG_E(" * turn off OTG 5V boost from Pad to Cover *\n");
        }
        g_Flag3 = 0;
        smb345c_charging_toggle(FLAGS, true);
        cancel_delayed_work(&chrgr_data.cover_detect_work);
        schedule_delayed_work(&chrgr_data.cover_detect_work, 0);
        break;
#if 0
    case COVER_ENUMERATION:
        BAT_DBG_E("-----------------------------------------------COVER_ENUMERATION\n");
        break;
#endif
    default:
        BAT_DBG_E("-----------------------------------------------COVER_EMPTY *\n");
        break;
    }

    return NOTIFY_OK;
}

static int smb345_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct usb_phy *otg_handle;
	struct smb345_charger *chrgr = &chrgr_data;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	int ret;
	int hwid;
	u32 val;
	int IRQ;

	pr_info("++++++++++++++++++++++++++++ [%s] ++++++++++++++++++++++++++++\n",
		__func__);
	INIT_CHARGER_DEBUG_ARRAY(chrgr_dbg);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_PROBE, 0, 0);

	/* Get pinctrl configurations */
	chrgr->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(chrgr->pinctrl))
		BUG();

	chrgr->pins_default = pinctrl_lookup_state(chrgr->pinctrl,
						PINCTRL_STATE_DEFAULT);
	if (IS_ERR(chrgr->pins_default))
		dev_err(dev, "could not get default pinstate\n");

	chrgr->pins_sleep = pinctrl_lookup_state(chrgr->pinctrl,
						PINCTRL_STATE_SLEEP);
	if (IS_ERR(chrgr->pins_sleep))
		dev_err(dev, "could not get sleep pinstate\n");

	chrgr->pins_inactive = pinctrl_lookup_state(chrgr->pinctrl,
						"inactive");
	if (IS_ERR(chrgr->pins_inactive))
		dev_err(dev, "could not get inactive pinstate\n");

	/* Register i2c clientdata before calling pinctrl */
	i2c_set_clientdata(client, chrgr);

	pr_info("%s\n", __func__);

	chrgr->client = client;
	wake_lock_init(&chrgr->wakelock, WAKE_LOCK_SUSPEND, "smb347");
	spin_lock_init(&spinlock_suspend);

    /* Special Cover Wake Lock */
    wake_lock_init(&wlock_t, WAKE_LOCK_SUSPEND, "Cover_WakeLock");

    chrgr->ws = wakeup_source_register("smb347_ws");

	otg_handle = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg_handle == NULL) {
		pr_err("ERROR!: getting OTG transceiver failed\n");
		return -EINVAL;
	}
	chrgr->otg_handle = otg_handle;

	/*
	 * Get interrupt from device tree
	 */
	chrgr->chgint_irq = irq_of_parse_and_map(np, 0);
	if (!chrgr->chgint_irq) {
		ret = -EINVAL;
		pr_err("can't get irq\n");
		goto remap_fail;
	}
	client->irq = chrgr->chgint_irq;

	chrgr->chgdet_irq = irq_of_parse_and_map(np, 1);
	if(!chrgr->chgdet_irq) {
		ret = -EINVAL;
		pr_err("Unable to retrieve IRQ for charger detection\n");
		goto remap_fail;
	}

    otg_enable_pinctrl = devm_pinctrl_get(&client->dev);
    if (IS_ERR_OR_NULL(otg_enable_pinctrl))
        dev_err(&client->dev, "could not get otg_enable pinctrl\n");
    GPIO_OTG_TOGGLE(0);

	chrgr->chg_otg_en_gpio = of_get_named_gpio(np, "intel,otg-enable", 0);
	hwid = Read_HW_ID();
#if defined(CONFIG_Z380C)
	if (hwid == HW_ID_SR)
		chrgr->chg_otg_en_gpio = of_get_named_gpio(np, "intel,otg-enable-SR", 0);
#endif

	if (!gpio_is_valid(chrgr->chg_otg_en_gpio)) {
		pr_err("Unable to retrieve intel,otg-enable pin\n");
	} else {
		ret = gpio_request(chrgr->chg_otg_en_gpio, "smb346_otg");
		if (ret < 0)
			pr_err("%s: request CHG_OTG gpio fail!\n", __func__);
	}

#if !INTEL_STYLE
	if (ret < 0)
		goto pre_fail;
	/* disable OTG function initially */
	gpio_direction_output(chrgr->chg_otg_en_gpio, 0);
#endif

#if defined(CONFIG_Z300C) || defined(CONFIG_Z300CG)
	chrgr->usb_acok = of_get_named_gpio_flags(np, "intel,usb_acok", 0, 0);
	if (!gpio_is_valid(chrgr->usb_acok)) {
		pr_err("%s(): can't get gpio: intel,usb_acok\n",
			__func__);
		goto fail;
	}

	pr_info("chrgr->usb_acok: %d\n", chrgr->usb_acok);

	ret = gpio_request_one(chrgr->usb_acok, GPIOF_IN, "usb_acok");
	if (ret < 0) {
		pr_err("%s: request usb_acok gpio fail!\n", __func__);
		goto fail;
	}

	chrgr->acok_irq = irq_of_parse_and_map(np, 2);
	if (IS_ERR_VALUE(chrgr->acok_irq)) {
		pr_err("%s: request acok_irq fail!\n", __func__);
		goto fail;
	}

	ret = devm_request_irq(dev, chrgr->acok_irq,
					smb345_inok_interrupt,
					IRQF_SHARED | IRQF_NO_SUSPEND,
					"usbin_det",
					chrgr);
	if (ret < 0) {
		pr_err("%s: config usb_acok gpio as IRQ fail!\n", __func__);
		goto fail;
	}
#endif

	chrgr->dcin_ok = of_get_named_gpio_flags(np, "intel,dcin_ok", 0, 0);
	if (!gpio_is_valid(chrgr->dcin_ok)) {
		pr_err("%s(): can't get gpio: intel,dcin_ok\n",
			__func__);
		goto fail;
	}

	pr_info("chrgr->dcin_ok: %d\n", chrgr->dcin_ok);

	ret = gpio_request_one(chrgr->dcin_ok, GPIOF_IN, "dcin_ok");
	if (ret < 0) {
		pr_err("%s: request dcin_ok gpio fail!\n", __func__);
		goto fail;
	}

#if 0
	chrgr->dcin_irq = irq_of_parse_and_map(np, 3);
	if (IS_ERR_VALUE(chrgr->dcin_irq)) {
		pr_err("%s: request dcin_irq fail!\n", __func__);
		goto fail;
	}

	ret = devm_request_irq(dev, chrgr->dcin_irq,
					smb345_inok_interrupt,
					IRQF_SHARED | IRQF_NO_SUSPEND,
					"dcin_det",
					chrgr);
	if (ret < 0) {
		pr_err("%s: config dcin_ok gpio as IRQ fail!\n", __func__);
		goto fail;
	}
#endif

    chrgr->vbus_in_det_n_gpio = of_get_named_gpio_flags(np, "intel,vbus-det", 0, 0);
    pr_info("intel,vbus-det = %d.\n", chrgr->vbus_in_det_n_gpio);

    if (!gpio_is_valid(chrgr->vbus_in_det_n_gpio)) {
        pr_err("gpio is not valid: vbus_in_det_n_gpio\n");
        return -EINVAL;
    }

    ret = vbus_in_det_n_gpio_init(chrgr);
    if (ret < 0) {
        pr_err("fail to initialize VBUS_IN_DET_N gpio: %d\n", ret);
        return ret;
    }

    IRQ = irq_of_parse_and_map(np, 2);
    pr_info("***************IRQ : %d*****************\n", IRQ);

    ret = request_threaded_irq(IRQ, NULL, vbus_in_det_n_interrupt,
                    IRQF_PERCPU | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
                    IRQF_ONESHOT |
                    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                    "vbus_in_det",
                    chrgr);
    if (ret < 0)
        dev_info(&chrgr->client->dev, "config VBUS_IN_DET_N gpio as IRQ fail!\n");

	ret = of_property_read_u32(np, "intel,chgint-debounce", &val);
	if (ret) {
		pr_err("Unable to retrieve intel,chgint-debounce\n");
		val = 0;
	}
	chrgr->chgint_debounce = msecs_to_jiffies(val);

	/* Setup the PMU registers. The charger IC reset line
	   is deasserted at this point. From this point onwards
	   the charger IC will act upon the values stored in its
	   registers instead of displaying default behaviour  */
	ret = smb345_configure_pmu_regs(chrgr);
	if (ret != 0)
		goto pre_fail;

	ret = smb345_set_writable(chrgr, true);

	if (ret != 0)
		pr_err("failed to enable writable\n");

	smb345_dump_registers(NULL);

	/* Refer to SMB345 Application Note 72 to solve serious problems */
	ret = smb345_masked_write(chrgr->client,
			OTG_TLIM_THERM_CNTRL_REG,
			OTG_CURRENT_LIMIT_AT_USBIN_MASK,
			OTG_CURRENT_LIMIT_250mA);
	if (ret < 0)
		return ret;

	sema_init(&chrgr->prop_lock, 1);

	/* Set up the wake lock to prevent suspend when charging. */
	wake_lock_init(&chrgr->suspend_lock,
			WAKE_LOCK_SUSPEND,
			"smb345_wake_lock");

	/* It is OK to setup the wake lock here, because it is
	before the interrupt is later enabled with the call:
	smb345_configure_pmu_irq(). */
	if (unfreezable_bh_create(&chrgr->chgint_bh, "chrgr_wq",
			"smb345_evt_lock", smb345_chgint_cb_work_func)) {
		ret = -ENOMEM;
		goto wq_creation_fail;
	}

	ret = smb345_set_pinctrl_state(client, chrgr->pins_default);
	if (ret != 0)
		return ret;

#if INTEL_STYLE
	chrgr->usb_psy.name           = "usb_charger";
	chrgr->usb_psy.type           = POWER_SUPPLY_TYPE_USB;
	chrgr->usb_psy.properties     = smb345_power_props;
	chrgr->usb_psy.num_properties = ARRAY_SIZE(smb345_power_props);
	chrgr->usb_psy.get_property   = smb345_charger_get_property;
	chrgr->usb_psy.set_property   = smb345_charger_set_property;
	chrgr->usb_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrgr->usb_psy.supplied_to = smb345_supplied_to;
	chrgr->usb_psy.num_supplicants = ARRAY_SIZE(smb345_supplied_to);
	chrgr->usb_psy.throttle_states = smb345_dummy_throttle_states;
	chrgr->usb_psy.num_throttle_states =
				ARRAY_SIZE(smb345_dummy_throttle_states);

	chrgr->current_psy = &chrgr->usb_psy;

	ret = power_supply_register(&client->dev, &chrgr->usb_psy);
	if (ret)
		goto fail;

	chrgr->ac_psy.name           = "ac_charger";
	chrgr->ac_psy.type           = POWER_SUPPLY_TYPE_MAINS;
	chrgr->ac_psy.properties     = smb345_power_props;
	chrgr->ac_psy.num_properties = ARRAY_SIZE(smb345_power_props);
	chrgr->ac_psy.get_property   = smb345_charger_get_property;
	chrgr->ac_psy.set_property   = smb345_charger_set_property;
	chrgr->ac_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrgr->ac_psy.supplied_to = smb345_supplied_to;
	chrgr->ac_psy.num_supplicants = ARRAY_SIZE(smb345_supplied_to);
	chrgr->ac_psy.throttle_states = smb345_dummy_throttle_states;
	chrgr->ac_psy.num_throttle_states =
				ARRAY_SIZE(smb345_dummy_throttle_states);

	ret = power_supply_register(&client->dev, &chrgr->ac_psy);
	if (ret)
		goto fail_ac_registr;
#else
	ret = smb345_register_power_supply(&client->dev);
	if (ret < 0)
		return ret;
#endif


	chrgr->ack_time = jiffies;

	ret = smb345_configure_pmu_irq(chrgr);
	if (ret != 0)
		goto pmu_irq_fail;

	i2c_set_clientdata(client, chrgr);

	smb345_setup_fake_vbus_sysfs_attr(chrgr);
	charger_work_queue = create_singlethread_workqueue("charger_workqueue");

	if (unfreezable_bh_create(&chrgr->boost_op_bh, "boost_op_wq",
				"smb345_boost_lock", smb345_set_boost)) {
		ret = -ENOMEM;
		goto pmu_irq_fail;
	}

#if INTEL_STYLE
	ret = usb_register_notifier(otg_handle, &chrgr->otg_nb);
	if (ret) {
		pr_err("ERROR!: registration for OTG notifications failed\n");
		goto boost_reg_fail;
	}
	INIT_DELAYED_WORK(&charger_work, do_charger);
#endif
	device_init_wakeup(&client->dev, true);

    INIT_WORK(&chrgr->chrgr_type_work, chrgr_type_work_func);
    INIT_DEFERRABLE_WORK(&chrgr->aicl_dete_work, aicl_dete_worker);
    INIT_DEFERRABLE_WORK(&chrgr->cover_detect_work, cover_detect_work_func);
    cover_cable_status_register_client(&cover_type_notifier);
    cable_status_register_client(&charger_type_notifier);

	/* create /sys/kernel/debug/smb */
	dentry = debugfs_create_file("smb", S_IRUGO,
				NULL, &chrgr, &smb345_debugfs_fops);

	/* create /sys/bus/idi/devices/i2c_5-0/i2c-5/5-006a/ interface */
	sysfs_create_group(&client->dev.kobj, &dev_attr_grp);

	ret = init_asus_charging_limit_toggle();
	if (ret) {
		BAT_DBG_E("Unable to create proc file\n");
		return ret;
	}

	/* register callback function */
	//smb345_tbl.soc_control_float_vol = smb345_soc_control_float_vol;
	//smb345_tbl.get_soc_control_float_vol = smb345_get_soc_control_float_vol;
	smb345_tbl.charging_toggle = smb345_charging_toggle;
	smb345_tbl.dump_registers = smb345_dump_registers;
	//smb345_tbl.jeita_control = smb3xx_jeita_control;
	//smb345_tbl.aicl_control = aicl_cur_control;
	asus_register_power_supply_charger(dev, &smb345_tbl);

    schedule_delayed_work(&chrgr->aicl_dete_work, HZ);
	pr_info("---------------------------- [%s] ----------------------------\n",
		__func__);

	/* Read the VBUS presence status for initial update by
	making a dummy interrupt bottom half invocation */
	queue_delayed_work(chrgr->chgint_bh.wq, &chrgr->chgint_bh.work, msecs_to_jiffies(1000));

	return 0;

boost_reg_fail:
	unfreezable_bh_destroy(&chrgr->boost_op_bh);
pmu_irq_fail:
#if INTEL_STYLE
	power_supply_unregister(&chrgr->ac_psy);
#endif
fail_ac_registr:
#if INTEL_STYLE
	power_supply_unregister(&chrgr->usb_psy);
#endif
fail:
	unfreezable_bh_destroy(&chrgr->chgint_bh);
	if (!IS_ERR_OR_NULL(dentry))
		debugfs_remove(dentry);
pre_fail:
wq_creation_fail:
remap_fail:
	usb_put_phy(otg_handle);
	return ret;
}

static int __exit smb345_i2c_remove(struct i2c_client *client)
{
	int ret = 0;
	struct smb345_charger *chrgr = &chrgr_data;
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_REMOVE, 0, 0);

	free_irq(client->irq, chrgr);
#if INTEL_STYLE
	power_supply_unregister(&chrgr_data.usb_psy);
	power_supply_unregister(&chrgr_data.ac_psy);
#else
	power_supply_unregister(&smb345_power_supplies[CHARGER_USB-1]);
	power_supply_unregister(&smb345_power_supplies[CHARGER_AC-1]);
#endif
	wake_lock_destroy(&chrgr_data.suspend_lock);

	unfreezable_bh_destroy(&chrgr_data.chgint_bh);
	unfreezable_bh_destroy(&chrgr_data.boost_op_bh);

	cancel_delayed_work_sync(&chrgr_data.charging_work);
	if (chrgr_data.otg_handle)
		usb_put_phy(chrgr_data.otg_handle);

	if (!IS_ERR_OR_NULL(dentry))
		debugfs_remove(dentry);

	ret = smb345_set_pinctrl_state(client, chrgr->pins_inactive);
	if (ret != 0)
		return ret;
    return 0;
}

static int smb345_idi_probe(struct idi_peripheral_device *ididev,
					const struct idi_device_id *id)
{
	struct resource *res;
	struct smb345_charger *chrgr = &chrgr_data;
	int ret = 0;

	spin_lock_init(&chrgr_dbg.lock);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_IDI_PROBE, 0, 0);
	pr_info("%s\n", __func__);

	res = idi_get_resource_byname(&ididev->resources,
			IORESOURCE_MEM, "registers");

	if (res == NULL) {
		pr_err("getting pmu_chgr_ctrl resources failed!\n");
		return -EINVAL;
	}

	chrgr->ctrl_io_res = res;

	chrgr->ididev = ididev;

	ret = idi_device_pm_set_class(ididev);
	if (ret) {
		pr_err("%s: Unable to register for generic pm class\n",
								__func__);
		return ret;
	}

	return 0;
}

static int __exit smb345_idi_remove(struct idi_peripheral_device *ididev)
{
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_IDI_REMOVE, 0, 0);
	pr_info("%s\n", __func__);

	return 0;
}

static int smb345_suspend(struct device *dev)
{
	struct smb345_charger *chrgr = &chrgr_data;
	unsigned long flags;

	spin_lock_irqsave(&spinlock_suspend, flags);
	suspend_smb = true;
	spin_unlock_irqrestore(&spinlock_suspend, flags);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SUSPEND_OK, 0, 0);
	if (device_may_wakeup(dev)) {
		pr_info("%s: enable wakeirq\n", __func__);
		enable_irq_wake(chrgr->chgdet_irq);
	}
	unfreezable_bh_suspend(&chrgr_data.chgint_bh);
	unfreezable_bh_suspend(&chrgr_data.boost_op_bh);

	return 0;
}

static int smb345_resume(struct device *dev)
{
	struct smb345_charger *chrgr = &chrgr_data;
	unsigned long flags;

	spin_lock_irqsave(&spinlock_suspend, flags);
	suspend_smb = false;
	if (suspend_work_not_done) {
		wake_lock_timeout(&chrgr->suspend_lock, msecs_to_jiffies(500));
		schedule_delayed_work(&chrgr_data.aicl_dete_work, 1);
	}
	spin_unlock_irqrestore(&spinlock_suspend, flags);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_RESUME, 0, 0);

	unfreezable_bh_resume(&chrgr_data.chgint_bh);
	unfreezable_bh_resume(&chrgr_data.boost_op_bh);

	if (device_may_wakeup(dev)) {
		pr_info("%s: disable wakeirq\n", __func__);
		disable_irq_wake(chrgr->chgdet_irq);
	}
	return 0;
}

#ifdef CONFIG_PM
static int smb345_prepare(struct device *dev)
{
    struct smb345_charger *smb = dev_get_drvdata(dev);

    /* cover charger control jeita */
    //smb3xx_charger_control_jeita();

    dev_info(&smb->client->dev, "smb347 prepare\n");

    return 0;
}

static void smb345_complete(struct device *dev)
{
    struct smb345_charger *smb = dev_get_drvdata(dev);

    dev_info(&smb->client->dev, "smb347 complete\n");
}
#else
#define smb345_prepare NULL
#define smb345_complete NULL
#endif

static void smb345_shutdown(struct i2c_client *client)
{
    dev_info(&client->dev, "%s\n", __func__);

    /* charger control jeita */
	smb3xx_charger_control_jeita();

    /* registers dump */
    smb345_dump_registers(NULL);

    /* turn off OTG */
    otg(0, false);

    /* turn off Cover CHG EN */
    COVER_CHARGING_TOGGLE(false);
}

const struct dev_pm_ops smb345_pm = {
	.suspend = smb345_suspend,
	.resume = smb345_resume,
	.prepare = smb345_prepare,
	.complete = smb345_complete,
};


static const struct i2c_device_id smb345_id[] = {
	{"smb345_charger", 0}, { }
};

MODULE_DEVICE_TABLE(i2c, smb345_id);

static struct i2c_driver smb345_i2c_driver = {
	.probe          = smb345_i2c_probe,
	.remove         = smb345_i2c_remove,
	.shutdown    = smb345_shutdown,
	.id_table       = smb345_id,
	.driver = {
		.name   = SMB345_NAME,
		.owner  = THIS_MODULE,
		.pm = &smb345_pm,
	},
};

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_CHG,
	},

	{ /* end: all zeroes */},
};

static struct idi_peripheral_driver smb345_idi_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "smb345_idi",
		.pm = NULL,
	},
	.p_type = IDI_CHG,
	.id_table = idi_ids,
	.probe  = smb345_idi_probe,
	.remove = smb345_idi_remove,
};

static int __init smb345_init(void)
{
	int ret;

	pr_info("++++++++++++++++++++++++++++ [%s] ++++++++++++++++++++++++++++\n",
		__func__);

	ret = idi_register_peripheral_driver(&smb345_idi_driver);
	if (ret)
		return ret;

	ret = i2c_add_driver(&smb345_i2c_driver);
	if (ret)
		return ret;

	return 0;
}

module_init(smb345_init);

static void __exit smb345_exit(void)
{
	i2c_del_driver(&smb345_i2c_driver);
}
module_exit(smb345_exit);

#if defined(CONFIG_Z380C)
static int en_chg_pinctrl_probe(struct platform_device *pdev)
{
    pr_info("++++++++++++++++++++++++++++ [%s] ++++++++++++++++++++++++++++\n",
		__func__);
    ch_enable_pinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR_OR_NULL(ch_enable_pinctrl))
        dev_err(&pdev->dev, "could not get ddr_mode pinctrl\n");
    COVER_CHARGING_TOGGLE(false);

    return 0;
}

static const struct of_device_id en_chg_pinctrl_of_match[] = {
        { .compatible = "CHARG_ENABLE"},
        {}
};
static struct platform_driver en_chg_driver = {
        .driver = {
             .name  = "en_chg",
             .owner = THIS_MODULE,
             //.pm    = &en_chg_pinctrl_dev_pm_ops,
             .of_match_table = en_chg_pinctrl_of_match,
        },
        .probe      = en_chg_pinctrl_probe,
};

static int __init en_chg_init(void)
{
	int ret;

	pr_info("++++++++++++++++++++++++++++ [%s] ++++++++++++++++++++++++++++\n",
		__func__);

	ret = platform_driver_register(&en_chg_driver);
	if (ret)
		return ret;

	return 0;
}
subsys_initcall(en_chg_init);
#endif
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Charger Driver for SMB345 charger IC");
