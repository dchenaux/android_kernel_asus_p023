/*
 * Asus USB Audio stand driver
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 */
#define DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/usb.h>

#define DRIVER_AUTHOR "JenChiu-Chiang, hank1_chiang@asus.com"
#define DRIVER_DESC "USB Audio Stand Driver"

static int interrupt_in_interval = 2;
module_param(interrupt_in_interval, int, 0);
MODULE_PARM_DESC(interrupt_in_interval, "Interrupt in interval in ms");

static int interrupt_out_interval = 8;
module_param(interrupt_out_interval, int, 0);
MODULE_PARM_DESC(interrupt_out_interval, "Interrupt out interval in ms");

static int battery_poll_interval = 5;
module_param(battery_poll_interval, int, 5);
MODULE_PARM_DESC(battery_poll_interval, "Battery poll interval in seconds");

/* table of devices that work with this driver */
static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(0x0b05, 0x7797), },
	{ },
};
MODULE_DEVICE_TABLE(usb, id_table);

enum {
    STAND_IN,
    STAND_CONFIGURED,
    STAND_RUNNING,
    STAND_DISCONNECT,
    STAND_REMOVAL,
};

#define CTRL_CMD_EC_STATUS    0x02
struct audio_stand_status {
    u8 opcode;
    u8 reserved[5];
    u8 fw_ver_h;
    u8 fw_ver_l;
} __attribute__ ((packed));
#define get_fw_ver(s) ((s)->fw_ver_h << 8 | (s)->fw_ver_l)

#define CTRL_CMD_BOOTLOADER   0x03
#define CTRL_CMD_GPIO_STATUS  0x04
/* GPI bits */
#define CTRL_CMD_GPI_ACOK 0

struct audio_stand_gpio_status {
    u8 opcode;
    u16 gpi;
    u16 gpo;
    u8 reserved[3];
} __attribute__ ((packed));

#define CTRL_CMD_GPO_SETTING  0x05
/* GPO bits */
#define CTRL_CMD_GPO_MOS_QB 0

struct audio_stand_gpo_ctrl {
    u8 opcode;
    u16 gpo_clr;                /* write 1 to clear the associated GPO */
    u16 gpo_set;                /* write 1 to set the associate GPO */
    u8 reserved[3];
} __attribute__ ((packed));

#define CTRL_CMD_BATT_INFO    0x10
struct audio_stand_batt_status {
    u8 opcode;
    u8 percentage;
    u8 temperature;
    u16 voltage;
    u8 reserved[2];
    u8 status;
} __attribute__ ((packed));
#define get_batt_voltage(s) ((s)->voltage)

#define CTRL_CMD_CHARGET_CTRL 0x20


struct usb_audio_stand {
	struct usb_device    *udev;
    struct usb_interface *interface;

    struct usb_endpoint_descriptor*  interrupt_in_endpoint;
    struct urb*                      interrupt_in_urb;
    int                              interrupt_in_interval;
    char*                            interrupt_in_buffer;
    int interrupt_in_buffer_lenght;

    struct usb_endpoint_descriptor*  interrupt_out_endpoint;
    struct urb*                      interrupt_out_urb;
    int                              interrupt_out_interval;
    char*                            interrupt_out_buffer;
    int interrupt_out_buffer_length;

    struct delayed_work status_poll_work;
    int status;
    char *read_buffer;
    int rb_length;
    spinlock_t rb_lock;

    char *write_buffer;
    int wr_length;
    u16 fw_ver;
};

#define CTRL_BLK_LENGTH 8
int fill_ctrl_blk(u8 *buf, int length, u8 opcode)
{
    if (length < CTRL_BLK_LENGTH)
        return 0;

    switch (opcode) {
        case CTRL_CMD_EC_STATUS:
            memcpy(buf, "\x02\0\0\0\0\0\0\0", CTRL_BLK_LENGTH);
            break;
        case CTRL_CMD_BOOTLOADER:
            memcpy(buf, "\x03\0xaa\0\0\0\0\0\0", CTRL_BLK_LENGTH);
            break;
        case CTRL_CMD_GPIO_STATUS:
            memcpy(buf, "\x04\0\0\0\0\0\0\0", CTRL_BLK_LENGTH);
            break;
        case CTRL_CMD_GPO_SETTING:
            memcpy(buf, "\x05\0\0\0\0\0\0\0", CTRL_BLK_LENGTH);
            break;
        case CTRL_CMD_BATT_INFO:
            memcpy(buf, "\x10\0\0\0\0\0\0\0", CTRL_BLK_LENGTH);
            break;
        case CTRL_CMD_CHARGET_CTRL:
            memcpy(buf, "\x20\0\0\0\0\0\0\0", CTRL_BLK_LENGTH);
            break;
        default:
            return 0;
    }
    return CTRL_BLK_LENGTH;
}

/* If successful, it returns the wanna object or NULL for exception.
 */
struct audio_stand_status* audio_stand_asking_status(
    struct usb_audio_stand *dev)
{
    struct usb_device *udev = dev->udev;
    struct usb_interface *interface = dev->interface;
    struct audio_stand_status *ctrl_blk;
    int ctrl_blk_length;
    int retval;

    ctrl_blk = kmalloc (sizeof(struct audio_stand_status), GFP_KERNEL);
    if (!ctrl_blk)
        return NULL;
    ctrl_blk_length = sizeof(struct audio_stand_status);

    retval = fill_ctrl_blk((u8*)ctrl_blk, ctrl_blk_length, CTRL_CMD_EC_STATUS);
    if (retval == 0) {
        dev_err(&interface->dev, "buffer length mismatch\n");
        kfree(ctrl_blk);
        return NULL;
    }

    retval = usb_control_msg(udev,
                             usb_sndctrlpipe(udev, 0),
                             0x09,
                             0x21,
                             0x300,
                             0,
                             ctrl_blk,
                             ctrl_blk_length,
                             2000);
    if (retval < 0) {
        dev_err(&interface->dev, "unable to initial send status command. err:%d\n", retval);
        kfree(ctrl_blk);
        return NULL;
    }

    retval = usb_control_msg(udev,
                             usb_rcvctrlpipe(udev, 0),
                             0x01,
                             0xa1,
                             0x300,
                             0,
                             ctrl_blk,
                             ctrl_blk_length,
                             2000);
    if (retval < 0) {
        dev_err(&interface->dev, "unable to get status. err:%d\n", retval);
        kfree(ctrl_blk);
        return NULL;
    }

	dev_dbg(&interface->dev, "%s got response: %02x:-:-:-:-:-:%02x:%02x\n",
            __func__, ctrl_blk->opcode,
            ctrl_blk->fw_ver_h, ctrl_blk->fw_ver_l);
    if (ctrl_blk->opcode != CTRL_CMD_EC_STATUS) {
        dev_err(&interface->dev, "invalid opcode: %02x\n", ctrl_blk->opcode);
        kfree(ctrl_blk);
        ctrl_blk = NULL;
    }
    return ctrl_blk;
}

/* If successful, it returns the wanna object or NULL for exception.
 */
struct audio_stand_batt_status* audio_stand_asking_battinfo(
    struct usb_audio_stand *dev)
{
    struct usb_device *udev = dev->udev;
    struct usb_interface *interface = dev->interface;
    struct audio_stand_batt_status *ctrl_blk;
    int ctrl_blk_length;
    int retval;

    ctrl_blk = kmalloc (sizeof(struct audio_stand_batt_status), GFP_KERNEL);
    if (!ctrl_blk)
        return NULL;
    ctrl_blk_length = sizeof(struct audio_stand_batt_status);
    retval = fill_ctrl_blk((u8*)ctrl_blk, ctrl_blk_length, CTRL_CMD_BATT_INFO);
    if (retval == 0) {
        dev_err(&interface->dev, "buffer length mismatch\n");
        kfree(ctrl_blk);
        return NULL;
    }

    retval = usb_control_msg(udev,
                             usb_sndctrlpipe(udev, 0),
                             0x09,
                             0x21,
                             0x300,
                             0,
                             ctrl_blk,
                             ctrl_blk_length,
                             2000);
    if (retval < 0) {
        dev_err(&interface->dev, "unable to initial send battinfo command. err:%d\n", retval);
        kfree(ctrl_blk);
        return NULL;
    }

    retval = usb_control_msg(udev,
                             usb_rcvctrlpipe(udev, 0),
                             0x01,
                             0xa1,
                             0x300,
                             0,
                             ctrl_blk,
                             ctrl_blk_length,
                             2000);
    if (retval < 0) {
        dev_err(&interface->dev, "unable to get batt info. err:%d\n", retval);
        kfree(ctrl_blk);
        return NULL;
    }

	dev_dbg(&interface->dev, "%s got response: %02x:%02x:%02x:%04x:-:-:%02x\n",
            __func__, ctrl_blk->opcode,
            ctrl_blk->percentage, ctrl_blk->temperature,
            ctrl_blk->voltage, ctrl_blk->status);

    if (ctrl_blk->opcode != CTRL_CMD_BATT_INFO) {
        dev_err(&interface->dev, "invalid opcode: %02x\n", ctrl_blk->opcode);
        kfree(ctrl_blk);
        ctrl_blk = NULL;
    }
    return ctrl_blk;
}

/* If successful, it returns the wanna object or NULL for exception.
 */
struct audio_stand_gpio_status* audio_stand_asking_gpio_status(
    struct usb_audio_stand *dev)
{
    struct usb_device *udev = dev->udev;
    struct usb_interface *interface = dev->interface;
    struct audio_stand_gpio_status *ctrl_blk;
    int ctrl_blk_length;
    int retval;

    ctrl_blk = kmalloc (sizeof(struct audio_stand_gpio_status), GFP_KERNEL);
    if (!ctrl_blk)
        return NULL;
    ctrl_blk_length = sizeof(struct audio_stand_gpio_status);
    retval = fill_ctrl_blk((u8*)ctrl_blk, ctrl_blk_length, CTRL_CMD_GPIO_STATUS);
    if (retval == 0) {
        dev_err(&interface->dev, "buffer length mismatch\n");
        kfree(ctrl_blk);
        return NULL;
    }

    retval = usb_control_msg(udev,
                             usb_sndctrlpipe(udev, 0),
                             0x09,
                             0x21,
                             0x300,
                             0,
                             ctrl_blk,
                             ctrl_blk_length,
                             2000);
    if (retval < 0) {
        dev_err(&interface->dev, "unable to initial send gpio status command. err:%d\n", retval);
        kfree(ctrl_blk);
        return NULL;
    }

    retval = usb_control_msg(udev,
                             usb_rcvctrlpipe(udev, 0),
                             0x01,
                             0xa1,
                             0x300,
                             0,
                             ctrl_blk,
                             ctrl_blk_length,
                             2000);
    if (retval < 0) {
        dev_err(&interface->dev, "unable to get gpio status. err:%d\n", retval);
        kfree(ctrl_blk);
        return NULL;
    }

	dev_dbg(&interface->dev, "%s got response: %02x:%04x:%04x:-:-:-\n",
            __func__, ctrl_blk->opcode,
            ctrl_blk->gpi, ctrl_blk->gpo);

    if (ctrl_blk->opcode != CTRL_CMD_GPIO_STATUS) {
        dev_err(&interface->dev, "invalid opcode: %02x\n", ctrl_blk->opcode);
        kfree(ctrl_blk);
        ctrl_blk = NULL;
    }
    return ctrl_blk;
}

void audio_stand_interrupt_out_callback(struct urb *rb)
{
    struct usb_audio_stand *dev = rb->context;
    dev_info(&dev->interface->dev, "%s: enter", __func__);
}

/* Before calling, caller should fill their data into 'dev->write_buffer' */
int audio_stand_intr_out_write(struct usb_audio_stand *dev)
{
    int retval;

    /* intr-out URB */
    usb_fill_int_urb(dev->interrupt_out_urb,
                     dev->udev,
                     usb_sndintpipe(dev->udev, dev->interrupt_out_endpoint->bEndpointAddress),
                     dev->interrupt_out_buffer,
                     usb_endpoint_maxp(dev->interrupt_out_endpoint),
                     audio_stand_interrupt_out_callback,
                     dev,
                     dev->interrupt_out_interval);
    wmb();

    retval = usb_submit_urb (dev->interrupt_out_urb, GFP_ATOMIC);
    if (retval) {
        dev_err(&dev->interface->dev,
                "Couldn't submit interrupt_out_urb %d\n", retval);
    }
    return retval;
}

void audio_stand_interrupt_in_callback(struct urb *rb)
{
	struct usb_audio_stand *dev = rb->context;
	int status = rb->status;
	int retval;
    int copy_length;

	dev_info(&dev->interface->dev, "%s: enter, status %d", __func__, status);

	if (status) {
		if (status == -ENOENT ||
		    status == -ECONNRESET ||
		    status == -ESHUTDOWN) {
			goto exit;
		} else {
			dev_info(&dev->interface->dev, "%s: nonzero status received: %d",
                     __func__, status);
			goto resubmit;
		}
	}

	/* if (rb->actual_length > 0) { */
    /*     copy_length = rb->actual_length; */
    /*     if (rb->actual_length > dev->rb_length) { */
    /*         dev_err(&dev->interface->dev, "urb actual_length is great than buffer length\n"); */
    /*         copy_length = dev->rb_length; */
    /*     } */
	/* 	spin_lock(&dev->rb_lock); */
    /*     /\* */
    /*     memcpy (dev->read_buffer, */
	/* 			dev->interrupt_in_buffer, */
    /*             copy_length); */
    /*     *\/ */
    /*     dev_info(&dev->interface, "%s: received %d bytes", __func__, rb->actual_length); */
	/* 	spin_unlock(&dev->rb_lock); */
	/* } */
resubmit:
	/* still running. resubmit */
	if (dev->status <= STAND_RUNNING && dev->udev) {
		retval = usb_submit_urb (dev->interrupt_in_urb, GFP_ATOMIC);
		if (retval)
			dev_err(&dev->interface->dev,
                    "%s: usb_submit_urb failed (%d)\n",
                    __func__, retval);
	}
exit:
	dev_info(&dev->interface->dev, "%s: leave, status %d", __func__, status);
}

int Mos_Qb_on(struct usb_audio_stand *stand_dev)
{
    struct usb_device *udev;
    struct audio_stand_gpo_ctrl *ctrl_blk;
	uint32_t val = simple_strtoul("00010000", NULL, 16);
    int retval, ctrl_blk_length;

    if (!stand_dev)
        return 0;
    udev = stand_dev->udev;
    ctrl_blk = kmalloc (sizeof(struct audio_stand_gpo_ctrl), GFP_KERNEL);
    if (!ctrl_blk)
        return 0;
    ctrl_blk_length = sizeof(struct audio_stand_gpo_ctrl);

    retval = fill_ctrl_blk((u8*)ctrl_blk, ctrl_blk_length, CTRL_CMD_GPO_SETTING);
    if (retval == 0) {
        dev_err(&stand_dev->interface->dev, "buffer length mismatch\n");
        kfree(ctrl_blk);
        return 0;
    }
    ctrl_blk->gpo_clr = (u16)(val & 0xffff);
    ctrl_blk->gpo_set = (u16)(val >> 16);

    retval = usb_control_msg(udev,
                             usb_sndctrlpipe(udev, 0),
                             0x09,
                             0x21,
                             0x300,
                             0,
                             ctrl_blk,
                             ctrl_blk_length,
                             2000);
    if (retval < 0) {
        dev_err(&stand_dev->interface->dev, "unable to set gpio. err:%d\n", retval);
        kfree(ctrl_blk);
        return 0;
    }

	return 0;
}

static void audio_stand_status_poll(struct work_struct *w)
{
    struct usb_audio_stand *dev = container_of(w, struct usb_audio_stand,
                                               status_poll_work.work);
    struct usb_interface *interface = dev->interface;
    struct audio_stand_batt_status *battinfo;
    int poll_seconds = battery_poll_interval;
    int retval;

	dev_info(&interface->dev, "%s: %d\n", __func__, dev->status);

    switch (dev->status) {
        case STAND_IN:
            /* intr-in URB */
            /* TODO. The following code is not test yet. because EC isn't ready.
            usb_fill_int_urb (dev->interrupt_in_urb,
                              dev->udev,
                              usb_rcvintpipe(dev->udev, dev->interrupt_in_endpoint->bEndpointAddress),
                              dev->interrupt_in_buffer,
                              usb_endpoint_maxp(dev->interrupt_in_endpoint),
                              audio_stand_interrupt_in_callback,
                              dev,
                              dev->interrupt_in_interval);
            mb();
            retval = usb_submit_urb (dev->interrupt_in_urb, GFP_ATOMIC);
            if (retval) {
                dev_err(&dev->udev->dev,
                        "Couldn't submit interrupt_in_urb %d\n", retval);
                dev->status = STAND_IN;
                break;
            }
            */
            dev->status = STAND_CONFIGURED;
            poll_seconds = 1;
            break;
        case STAND_CONFIGURED:
            dev->status = STAND_RUNNING;
        case STAND_RUNNING:
            /* asking batter infomation */
            battinfo = audio_stand_asking_battinfo(dev);
            if (!battinfo)
                break;
            dev_info(&interface->dev, "<UBATT> P:%u%%, V:%umV, T:%uC, status: %02x\n",
                     battinfo->percentage, get_batt_voltage(battinfo),
                     battinfo->temperature, battinfo->status);
            kfree(battinfo);
            /* TODO. resend intr request */

            //memset(dev->write_buffer, 0 , dev->wr_length);
            //audio_stand_intr_out_write(dev);
            break;
        default:
            //usb_kill_urb(dev->interrupt_in_urb);
            //usb_kill_urb(dev->interrupt_out_urb);
            break;
    }
    if (dev->status > STAND_RUNNING) {
        dev_err(&dev->udev->dev, "stopping poll work\n");
        return;
    }
    /* reschedule us */
    queue_delayed_work(system_nrt_wq, &dev->status_poll_work, poll_seconds*HZ);
}

static ssize_t fwver_show(struct device *dev,
                          struct device_attribute *attr, char *buf)
{
    struct usb_audio_stand *stand_dev = dev_get_drvdata(dev);

    if(stand_dev)
        return snprintf(buf, PAGE_SIZE, "%04x\n", stand_dev->fw_ver);
    else
        return snprintf(buf, PAGE_SIZE, "unknown\n");
}
static ssize_t gpio_show(struct device *dev,
                         struct device_attribute *attr, char *buf)
{
    struct usb_audio_stand *stand_dev = dev_get_drvdata(dev);
    struct audio_stand_gpio_status *status;

    if(!stand_dev)
        return snprintf(buf, PAGE_SIZE, "unknown\n");

    status = audio_stand_asking_gpio_status(stand_dev);
    if (!status)
        return snprintf(buf, PAGE_SIZE, "unknown\n");

    return snprintf(buf, PAGE_SIZE, "GPI:%04x, GPO: %04x\n",
                    status->gpi, status->gpo);
}
static ssize_t gpio_store(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    struct usb_audio_stand *stand_dev = dev_get_drvdata(dev);
    struct usb_device *udev;
    struct audio_stand_gpo_ctrl *ctrl_blk;
	uint32_t val = simple_strtoul(buf, NULL, 16);
    int retval, ctrl_blk_length;

    if (!stand_dev)
        return 0;
    udev = stand_dev->udev;
    ctrl_blk = kmalloc (sizeof(struct audio_stand_gpo_ctrl), GFP_KERNEL);
    if (!ctrl_blk)
        return 0;
    ctrl_blk_length = sizeof(struct audio_stand_gpo_ctrl);

    retval = fill_ctrl_blk((u8*)ctrl_blk, ctrl_blk_length, CTRL_CMD_GPO_SETTING);
    if (retval == 0) {
        dev_err(dev, "buffer length mismatch\n");
        kfree(ctrl_blk);
        return 0;
    }
    ctrl_blk->gpo_clr = (u16)(val & 0xffff);
    ctrl_blk->gpo_set = (u16)(val >> 16);

    retval = usb_control_msg(udev,
                             usb_sndctrlpipe(udev, 0),
                             0x09,
                             0x21,
                             0x300,
                             0,
                             ctrl_blk,
                             ctrl_blk_length,
                             2000);
    if (retval < 0) {
        dev_err(dev, "unable to set gpio. err:%d\n", retval);
        kfree(ctrl_blk);
        return 0;
    }

	return count;
}

static DEVICE_ATTR(fw_ver, S_IRUGO, fwver_show, NULL);
static DEVICE_ATTR(gpio, S_IRUGO | S_IWUSR, gpio_show, gpio_store);
static struct attribute *dev_attrs[] = {
    &dev_attr_fw_ver.attr,
    &dev_attr_gpio.attr,
    NULL,
};
static struct attribute_group dev_attr_grp = {
    .attrs = dev_attrs,
};

static int stand_probe(struct usb_interface *interface,
		     const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct usb_audio_stand *dev = NULL;
    struct usb_host_interface *iface_desc;
    struct usb_endpoint_descriptor *endpoint;
    struct audio_stand_status *status;
	int retval = -ENOMEM;
    int i;

	dev = kzalloc(sizeof(struct usb_audio_stand), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&interface->dev, "out of memory\n");
		goto error;
	}

	dev->udev = usb_get_dev(udev);
    dev->interface = interface;
    dev->interrupt_in_endpoint = NULL;
    dev->interrupt_in_urb = NULL;
    dev->interrupt_in_buffer = NULL;

    dev->interrupt_out_endpoint = NULL;
    dev->interrupt_out_urb = NULL;
    dev->interrupt_out_buffer = NULL;

    dev->read_buffer = NULL;

    /* setup endpoints */
    iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;
        /* we only interested in interrupt endpoints */
		if (usb_endpoint_xfer_int(endpoint)) {
			if (usb_endpoint_dir_in(endpoint))
				dev->interrupt_in_endpoint = endpoint;
			else
				dev->interrupt_out_endpoint = endpoint;
		}
	}
    if (unlikely(dev->interrupt_in_endpoint == NULL)) {
        dev_err(&interface->dev, "missing intr-in endpont\n");
        goto error;
    }
    if (unlikely(dev->interrupt_out_endpoint == NULL)) {
        dev_err(&interface->dev, "missing intr-out endpont\n");
        goto error;
    }
    /* allocate foo for intr-in */
	dev->interrupt_in_interval = interrupt_in_interval > dev->interrupt_in_endpoint->bInterval ?
        interrupt_in_interval : dev->interrupt_in_endpoint->bInterval;

    dev->interrupt_in_buffer = kmalloc (usb_endpoint_maxp(dev->interrupt_in_endpoint), GFP_KERNEL);
	if (!dev->interrupt_in_buffer) {
		dev_err(&interface->dev, "Couldn't allocate intr-in buffer\n");
		goto error;
	}
    dev->interrupt_in_buffer = kmalloc (usb_endpoint_maxp(dev->interrupt_in_endpoint), GFP_KERNEL);
	if (!dev->interrupt_in_buffer) {
		dev_err(&interface->dev, "Couldn't allocate intr-in buffer\n");
		goto error_mem;
	}
	dev->interrupt_in_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->interrupt_in_urb) {
		dev_err(&interface->dev, "Couldn't allocate intr-in urb\n");
		goto error_mem;
	}
	dev->read_buffer = kmalloc (usb_endpoint_maxp(dev->interrupt_in_endpoint), GFP_KERNEL);
	if (!dev->read_buffer) {
		dev_err(&interface->dev, "Couldn't allocate read buffer\n");
		goto error_mem;
	}
    dev->rb_length = usb_endpoint_maxp(dev->interrupt_in_endpoint);
    spin_lock_init(&dev->rb_lock);
    /* allocate foo for intr-out */
	dev->interrupt_out_interval = interrupt_out_interval > dev->interrupt_out_endpoint->bInterval ?
        interrupt_out_interval : dev->interrupt_out_endpoint->bInterval;

	dev->interrupt_out_buffer = kmalloc (usb_endpoint_maxp(dev->interrupt_out_endpoint), GFP_KERNEL);
	if (!dev->interrupt_out_buffer) {
		dev_err(&interface->dev, "Couldn't allocate interrupt_out_buffer\n");
		goto error_mem;
	}

	dev->write_buffer = kmalloc (usb_endpoint_maxp(dev->interrupt_in_endpoint), GFP_KERNEL);
	if (!dev->interrupt_out_buffer) {
		dev_err(&interface->dev, "Couldn't allocate interrupt_out_buffer\n");
		goto error_mem;
	}
    dev->wr_length = usb_endpoint_maxp(dev->interrupt_in_endpoint);

	dev->interrupt_out_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->interrupt_out_urb) {
		dev_err(&interface->dev, "Couldn't allocate interrupt_out_urb\n");
		goto error_mem;
	}

	usb_set_intfdata(interface, dev);

    /* asking status */
    status = audio_stand_asking_status(dev);
    if (!status)
        goto error_mem;
    dev->fw_ver = get_fw_ver(status);
    kfree(status);
    dev_info(&interface->dev, "USB Audio Stand now attached, fw_ver: %04x\n",
             dev->fw_ver);

#ifdef _ENG_BUILD_
    Mos_Qb_on(dev);
#endif

    /* finally, we setup a timer and free running */
    dev->status = STAND_IN;
    INIT_DELAYED_WORK(&dev->status_poll_work, audio_stand_status_poll);
    queue_delayed_work(system_nrt_wq, &dev->status_poll_work, HZ);

    usb_set_intfdata(interface, dev);
    if (sysfs_create_group(&interface->dev.kobj, &dev_attr_grp))
        dev_err(&interface->dev, "unable to create ATD interface\n");

	return 0;

error_mem:
    if (dev->interrupt_in_urb)
        usb_free_urb(dev->interrupt_in_urb);
    if (dev->interrupt_in_buffer)
        kfree(dev->interrupt_in_buffer);

    if (dev->interrupt_out_urb)
        usb_free_urb(dev->interrupt_out_urb);
    if (dev->interrupt_out_buffer)
        kfree(dev->interrupt_out_buffer);

    if (dev->read_buffer)
        kfree(dev->read_buffer);
    if (dev->write_buffer)
        kfree(dev->write_buffer);
error:
	usb_set_intfdata(interface, NULL);
	usb_put_dev(dev->udev);
	kfree(dev);
    return retval;
}

static void stand_disconnect(struct usb_interface *interface)
{
	struct usb_audio_stand *dev;

	dev = usb_get_intfdata(interface);
    sysfs_remove_group(&interface->dev.kobj, &dev_attr_grp);
    /* TOTO. Kill intr urbs */
    cancel_delayed_work(&dev->status_poll_work);

	usb_set_intfdata(interface, NULL);
    /* free allocations */
    usb_free_urb(dev->interrupt_in_urb);
    kfree(dev->interrupt_in_buffer);

    usb_free_urb(dev->interrupt_out_urb);
    kfree(dev->interrupt_out_buffer);

    kfree(dev->read_buffer);
    kfree(dev->write_buffer);

	usb_put_dev(dev->udev);

	kfree(dev);

	dev_info(&interface->dev, "USB Audio Stand now disconnected\n");
}

static struct usb_driver stand_driver = {
	.name =		"asus_audio_stand",
	.probe =	stand_probe,
	.disconnect =	stand_disconnect,
	.id_table =	id_table,
};

module_usb_driver(stand_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
