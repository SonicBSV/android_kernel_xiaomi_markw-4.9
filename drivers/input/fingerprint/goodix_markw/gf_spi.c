/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *     Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 * Copyright (C) 2018 XiaoMi, Inc.
 * Copyright (C) 2024 Optimized version
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

#define GF_SPIDEV_NAME      "goodix,fingerprint"
#define GF_DEV_NAME         "goodix_fp"
#define GF_INPUT_NAME       "gf3208"
#define CHRD_DRIVER_NAME    "goodix_fp_spi"
#define CLASS_NAME          "goodix_fp"
#define N_SPI_MINORS        32

/* Оптимизированные тайминги */
#define GF_TTW_HOLD_TIME    1000
#define GF_RESET_DELAY_US   100
#define GF_RESET_HOLD_US    3000
#define GF_POWER_DELAY_MS   5

/* PM QoS для снижения латентности */
static struct pm_qos_request gf_pm_qos_req;
static bool pm_qos_enabled = false;

static const struct gf_key_map key_map[] = {
    { "POWER",  KEY_POWER  },
    { "HOME",   KEY_HOME   },
    { "MENU",   KEY_MENU   },
    { "BACK",   KEY_BACK   },
    { "UP",     KEY_UP     },
    { "DOWN",   KEY_DOWN   },
    { "LEFT",   KEY_LEFT   },
    { "RIGHT",  KEY_RIGHT  },
    { "CAMERA", KEY_CAMERA },
    { "ENTER",  KEY_SELECT },
    { "FORCE",  KEY_F9     },
    { "CLICK",  KEY_F19    },
};

#define GF_KEY_MAP_SIZE ARRAY_SIZE(key_map)

/* Debug control */
#ifdef GF_DEBUG
#define gf_dbg(fmt, args...) pr_debug("gf: " fmt, ##args)
#define FUNC_ENTRY() pr_debug("gf: %s entry\n", __func__)
#define FUNC_EXIT()  pr_debug("gf: %s exit\n", __func__)
#else
#define gf_dbg(fmt, args...) do {} while (0)
#define FUNC_ENTRY() do {} while (0)
#define FUNC_EXIT()  do {} while (0)
#endif

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static struct gf_dev gf;
static struct class *gf_class;
static int gf_major = -1;

/* Forward declarations */
static int driver_init_partial(struct gf_dev *gf_dev);
static void gf_pm_qos_update(bool enable);

/* Оптимизированное управление PM QoS */
static void gf_pm_qos_update(bool enable)
{
    if (enable && !pm_qos_enabled) {
        pm_qos_update_request(&gf_pm_qos_req, 100);
        pm_qos_enabled = true;
    } else if (!enable && pm_qos_enabled) {
        pm_qos_update_request(&gf_pm_qos_req, PM_QOS_DEFAULT_VALUE);
        pm_qos_enabled = false;
    }
}

static inline void gf_enable_irq(struct gf_dev *gf_dev)
{
    if (!gf_dev->irq_enabled) {
        enable_irq_wake(gf_dev->irq);
        gf_dev->irq_enabled = 1;
        gf_dbg("IRQ enabled\n");
    }
}

static inline void gf_disable_irq(struct gf_dev *gf_dev)
{
    if (gf_dev->irq_enabled) {
        gf_dev->irq_enabled = 0;
        disable_irq_wake(gf_dev->irq);
        gf_dbg("IRQ disabled\n");
    }
}

#ifdef AP_CONTROL_CLK
static long spi_clk_max_rate(struct clk *clk, unsigned long rate)
{
    long lowest_available, nearest_low, step_size, cur;
    long step_direction = -1;
    long guess = rate;
    int max_steps = 10;

    if (unlikely(!clk))
        return -EINVAL;

    cur = clk_round_rate(clk, rate);
    if (cur == rate)
        return rate;

    lowest_available = clk_round_rate(clk, 0);
    if (lowest_available > rate)
        return -EINVAL;

    step_size = (rate - lowest_available) >> 1;
    nearest_low = lowest_available;

    while (max_steps-- && step_size) {
        guess += step_size * step_direction;
        cur = clk_round_rate(clk, guess);

        if ((cur < rate) && (cur > nearest_low))
            nearest_low = cur;

        if (((cur > rate) && (step_direction > 0)) ||
            ((cur < rate) && (step_direction < 0))) {
            step_direction = -step_direction;
            step_size >>= 1;
        }
    }
    return nearest_low;
}

static int spi_clock_set(struct gf_dev *gf_dev, int speed)
{
    long rate;
    int rc;

    if (unlikely(!gf_dev->core_clk))
        return -EINVAL;

    rate = spi_clk_max_rate(gf_dev->core_clk, speed);
    if (rate < 0) {
        pr_warn("%s: no match for clock frequency: %d\n", __func__, speed);
        return rate;
    }

    rc = clk_set_rate(gf_dev->core_clk, rate);
    if (rc)
        pr_err("%s: failed to set clock rate: %d\n", __func__, rc);

    return rc;
}

static int gfspi_ioctl_clk_init(struct gf_dev *data)
{
    data->clk_enabled = 0;
    
    data->core_clk = clk_get(&data->spi->dev, "core_clk");
    if (IS_ERR_OR_NULL(data->core_clk)) {
        pr_err("%s: failed to get core_clk\n", __func__);
        return -EPERM;
    }
    
    data->iface_clk = clk_get(&data->spi->dev, "iface_clk");
    if (IS_ERR_OR_NULL(data->iface_clk)) {
        pr_err("%s: failed to get iface_clk\n", __func__);
        clk_put(data->core_clk);
        data->core_clk = NULL;
        return -ENOENT;
    }
    
    return 0;
}

static int gfspi_ioctl_clk_enable(struct gf_dev *data)
{
    int err;

    if (data->clk_enabled)
        return 0;

    err = clk_prepare_enable(data->core_clk);
    if (err) {
        pr_err("%s: failed to enable core_clk\n", __func__);
        return -EPERM;
    }

    err = clk_prepare_enable(data->iface_clk);
    if (err) {
        pr_err("%s: failed to enable iface_clk\n", __func__);
        clk_disable_unprepare(data->core_clk);
        return -ENOENT;
    }

    data->clk_enabled = 1;
    return 0;
}

static int gfspi_ioctl_clk_disable(struct gf_dev *data)
{
    if (!data->clk_enabled)
        return 0;

    clk_disable_unprepare(data->core_clk);
    clk_disable_unprepare(data->iface_clk);
    data->clk_enabled = 0;
    return 0;
}

static void gfspi_ioctl_clk_uninit(struct gf_dev *data)
{
    if (data->clk_enabled)
        gfspi_ioctl_clk_disable(data);

    if (!IS_ERR_OR_NULL(data->core_clk)) {
        clk_put(data->core_clk);
        data->core_clk = NULL;
    }

    if (!IS_ERR_OR_NULL(data->iface_clk)) {
        clk_put(data->iface_clk);
        data->iface_clk = NULL;
    }
}
#endif /* AP_CONTROL_CLK */

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct gf_dev *gf_dev = &gf;
    struct gf_key gf_key;
    int retval = 0;
    int i;
#ifdef AP_CONTROL_CLK
    unsigned int speed = 0;
#endif

    if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
        return -ENODEV;

    if (_IOC_DIR(cmd) & _IOC_READ)
        retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    if (!retval && (_IOC_DIR(cmd) & _IOC_WRITE))
        retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if (retval)
        return -EFAULT;

    if (!gf_dev->device_available) {
        if (cmd != GF_IOC_POWER_ON && cmd != GF_IOC_POWER_OFF && 
            cmd != GF_IOC_ENABLE_GPIO) {
            pr_info("Sensor is powered off\n");
            return -ENODEV;
        }
    }

    switch (cmd) {
    case GF_IOC_ENABLE_GPIO:
        retval = driver_init_partial(gf_dev);
        break;

    case GF_IOC_RELEASE_GPIO:
        gf_cleanup(gf_dev);
        break;

    case GF_IOC_DISABLE_IRQ:
        gf_disable_irq(gf_dev);
        break;

    case GF_IOC_ENABLE_IRQ:
        gf_enable_irq(gf_dev);
        break;

    case GF_IOC_SETSPEED:
#ifdef AP_CONTROL_CLK
        if (!get_user(speed, (u32 __user *)arg)) {
            if (speed > 12000000) {
                pr_warn("Speed %d exceeds 12Mbps limit\n", speed);
            } else {
                spi_clock_set(gf_dev, speed);
            }
        } else {
            pr_warn("Failed to get speed from user\n");
            retval = -EFAULT;
        }
#endif
        break;

    case GF_IOC_RESET:
        gf_hw_reset(gf_dev, 3);
        break;

    case GF_IOC_COOLBOOT:
        gf_power_off(gf_dev);
        usleep_range(GF_POWER_DELAY_MS * 1000, GF_POWER_DELAY_MS * 1000 + 100);
        gf_power_on(gf_dev);
        break;

    case GF_IOC_SENDKEY:
        if (copy_from_user(&gf_key, (struct gf_key *)arg, sizeof(gf_key))) {
            pr_warn("Failed to copy data from user space\n");
            retval = -EFAULT;
            break;
        }

        for (i = 0; i < GF_KEY_MAP_SIZE; i++) {
            if (key_map[i].val == gf_key.key) {
                int report_key = (gf_key.key == KEY_CAMERA) ? KEY_SELECT : gf_key.key;
                input_report_key(gf_dev->input, report_key, gf_key.value);
                input_sync(gf_dev->input);
                break;
            }
        }
        if (i == GF_KEY_MAP_SIZE) {
            pr_warn("Key %d not supported\n", gf_key.key);
            retval = -EINVAL;
        }
        break;

    case GF_IOC_CLK_READY:
#ifdef AP_CONTROL_CLK
        gfspi_ioctl_clk_enable(gf_dev);
#endif
        break;

    case GF_IOC_CLK_UNREADY:
#ifdef AP_CONTROL_CLK
        gfspi_ioctl_clk_disable(gf_dev);
#endif
        break;

    case GF_IOC_PM_FBCABCK:
        if (put_user(gf_dev->fb_black, (u8 __user *)arg))
            retval = -EFAULT;
        break;

    case GF_IOC_POWER_ON:
        if (!gf_dev->device_available) {
            gf_power_on(gf_dev);
            gf_dev->device_available = 1;
        }
        break;

    case GF_IOC_POWER_OFF:
        if (gf_dev->device_available) {
            gf_power_off(gf_dev);
            gf_dev->device_available = 0;
        }
        break;

    default:
        gf_dbg("Unsupported cmd: 0x%x\n", cmd);
        retval = -ENOTTY;
        break;
    }

    return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

/* Оптимизированный обработчик прерываний */
static irqreturn_t gf_irq(int irq, void *handle)
{
    struct gf_dev *gf_dev = &gf;
    
    FUNC_ENTRY();

    /* Активируем wake lock с минимальным таймаутом */
    __pm_wakeup_event(&gf_dev->ttw_wl, GF_TTW_HOLD_TIME);
    
    /* Обновляем PM QoS для быстрой обработки */
    gf_pm_qos_update(true);

#if defined(GF_NETLINK_ENABLE)
    {
        char temp = GF_NET_EVENT_IRQ;
        sendnlmsg(&temp);
    }
#elif defined(GF_FASYNC)
    if (gf_dev->async)
        kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif

    return IRQ_HANDLED;
}

static int driver_init_partial(struct gf_dev *gf_dev)
{
    int ret = 0;
    
    FUNC_ENTRY();

    gf_dev->device_available = 1;

    ret = gf_parse_dts(gf_dev);
    if (ret)
        goto error;

    gf_dev->irq = gf_irq_num(gf_dev);
    
    ret = devm_request_threaded_irq(&gf_dev->spi->dev,
            gf_dev->irq,
            NULL,
            gf_irq,
            IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_SUSPEND,
            "gf", gf_dev);
    if (ret) {
        pr_err("Failed to request irq %d\n", gf_dev->irq);
        goto error;
    }

    gf_enable_irq(gf_dev);
    gf_disable_irq(gf_dev);
    gf_hw_reset(gf_dev, 360);

    FUNC_EXIT();
    return 0;

error:
    gf_cleanup(gf_dev);
    gf_dev->device_available = 0;
    return -EPERM;
}

static int gf_open(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev;
    int status = -ENXIO;

    FUNC_ENTRY();
    mutex_lock(&device_list_lock);

    list_for_each_entry(gf_dev, &device_list, device_entry) {
        if (gf_dev->devt == inode->i_rdev) {
            gf_dbg("Device found\n");
            status = 0;
            break;
        }
    }

    if (status == 0) {
        gf_dev->users++;
        filp->private_data = gf_dev;
        nonseekable_open(inode, filp);
        gf_dev->device_available = 1;
        gf_dbg("Device opened, irq = %d\n", gf_dev->irq);
    } else {
        gf_dbg("No device for minor %d\n", iminor(inode));
    }

    mutex_unlock(&device_list_lock);
    FUNC_EXIT();
    return status;
}

#ifdef GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
    struct gf_dev *gf_dev = filp->private_data;
    return fasync_helper(fd, filp, mode, &gf_dev->async);
}
#endif

static int gf_release(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev;
    int status = 0;

    FUNC_ENTRY();
    mutex_lock(&device_list_lock);
    
    gf_dev = filp->private_data;
    filp->private_data = NULL;

    if (--gf_dev->users == 0) {
        gf_disable_irq(gf_dev);
        devm_free_irq(&gf_dev->spi->dev, gf_dev->irq, gf_dev);
        gf_dev->device_available = 0;
        gf_power_off(gf_dev);
        gf_pm_qos_update(false);
    }

    mutex_unlock(&device_list_lock);
    FUNC_EXIT();
    return status;
}

static const struct file_operations gf_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = gf_compat_ioctl,
#endif
    .open = gf_open,
    .release = gf_release,
#ifdef GF_FASYNC
    .fasync = gf_fasync,
#endif
};

/* Оптимизированный обработчик FB событий */
static int goodix_fb_state_chg_callback(struct notifier_block *nb,
                    unsigned long val, void *data)
{
    struct gf_dev *gf_dev;
    struct fb_event *evdata = data;
    unsigned int blank;
#if defined(GF_NETLINK_ENABLE)
    char temp = 0;
#endif

    if (val != FB_EARLY_EVENT_BLANK || !evdata || !evdata->data)
        return NOTIFY_DONE;

    gf_dev = container_of(nb, struct gf_dev, gf_notifier);
    if (!gf_dev || !gf_dev->device_available)
        return NOTIFY_DONE;

    blank = *(int *)(evdata->data);

    switch (blank) {
    case FB_BLANK_POWERDOWN:
        gf_dev->fb_black = 1;
        gf_pm_qos_update(false);
#if defined(GF_NETLINK_ENABLE)
        temp = GF_NET_EVENT_FB_BLACK;
        sendnlmsg(&temp);
#elif defined(GF_FASYNC)
        if (gf_dev->async)
            kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif
        break;

    case FB_BLANK_UNBLANK:
        gf_dev->fb_black = 0;
#if defined(GF_NETLINK_ENABLE)
        temp = GF_NET_EVENT_FB_UNBLACK;
        sendnlmsg(&temp);
#elif defined(GF_FASYNC)
        if (gf_dev->async)
            kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
#endif
        break;
    }

    return NOTIFY_OK;
}

static void gf_reg_key_kernel(struct gf_dev *gf_dev)
{
    int i;

    set_bit(EV_KEY, gf_dev->input->evbit);
    for (i = 0; i < GF_KEY_MAP_SIZE; i++)
        set_bit(key_map[i].val, gf_dev->input->keybit);

    gf_dev->input->name = GF_INPUT_NAME;
    if (input_register_device(gf_dev->input))
        pr_warn("Failed to register input device\n");
}

#if defined(USE_SPI_BUS)
static int gf_probe(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_probe(struct platform_device *pdev)
#endif
{
    struct gf_dev *gf_dev = &gf;
    int status = -EINVAL;
    unsigned long minor;

    FUNC_ENTRY();
    pr_info("gf_probe start\n");

    INIT_LIST_HEAD(&gf_dev->device_entry);

#if defined(USE_SPI_BUS)
    gf_dev->spi = spi;
#elif defined(USE_PLATFORM_BUS)
    gf_dev->spi = pdev;
#endif

    /* Инициализация значений по умолчанию */
    gf_dev->irq_gpio = -EINVAL;
    gf_dev->reset_gpio = -EINVAL;
    gf_dev->pwr_gpio = -EINVAL;
    gf_dev->device_available = 0;
    gf_dev->fb_black = 0;
    gf_dev->irq_enabled = 0;
    gf_dev->fingerprint_pinctrl = NULL;
    gf_dev->users = 0;

    /* Инициализация PM QoS */
    pm_qos_add_request(&gf_pm_qos_req, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);

    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
        struct device *dev;
        gf_dev->devt = MKDEV(gf_major, minor);
        dev = device_create(gf_class, &gf_dev->spi->dev, gf_dev->devt,
                    gf_dev, GF_DEV_NAME);
        status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    } else {
        dev_dbg(&gf_dev->spi->dev, "No minor number available\n");
        status = -ENODEV;
    }

    if (status == 0) {
        set_bit(minor, minors);
        list_add(&gf_dev->device_entry, &device_list);
    } else {
        gf_dev->devt = 0;
    }
    mutex_unlock(&device_list_lock);

    if (status == 0) {
        gf_dev->input = input_allocate_device();
        if (!gf_dev->input) {
            dev_err(&gf_dev->spi->dev, "Failed to allocate input device\n");
            status = -ENOMEM;
            goto error;
        }

#ifdef AP_CONTROL_CLK
        if (gfspi_ioctl_clk_init(gf_dev))
            goto gfspi_probe_clk_init_failed;

        if (gfspi_ioctl_clk_enable(gf_dev))
            goto gfspi_probe_clk_enable_failed;

        spi_clock_set(gf_dev, 4800000);
#endif

#ifdef CONFIG_FB
        gf_dev->gf_notifier.notifier_call = goodix_fb_state_chg_callback;
        if (fb_register_client(&gf_dev->gf_notifier))
            pr_err("Failed to register fb notifier\n");
#endif

        gf_reg_key_kernel(gf_dev);
        wakeup_source_init(&gf_dev->ttw_wl, "goodix_ttw_wl");
    }

    pr_info("gf_probe OK\n");
    return status;

error:
    gf_cleanup(gf_dev);
    gf_dev->device_available = 0;
    if (gf_dev->devt != 0) {
        mutex_lock(&device_list_lock);
        list_del(&gf_dev->device_entry);
        device_destroy(gf_class, gf_dev->devt);
        clear_bit(MINOR(gf_dev->devt), minors);
        mutex_unlock(&device_list_lock);

#ifdef AP_CONTROL_CLK
gfspi_probe_clk_enable_failed:
        gfspi_ioctl_clk_uninit(gf_dev);
gfspi_probe_clk_init_failed:
#endif
        if (gf_dev->input)
            input_unregister_device(gf_dev->input);
    }

    pm_qos_remove_request(&gf_pm_qos_req);
    return status;
}

#if defined(USE_SPI_BUS)
static int gf_remove(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_remove(struct platform_device *pdev)
#endif
{
    struct gf_dev *gf_dev = &gf;
    
    FUNC_ENTRY();

    pm_qos_remove_request(&gf_pm_qos_req);

    if (gf_dev->irq)
        free_irq(gf_dev->irq, gf_dev);

    if (gf_dev->input) {
        input_unregister_device(gf_dev->input);
        input_free_device(gf_dev->input);
    }

    mutex_lock(&device_list_lock);
    list_del(&gf_dev->device_entry);
    device_destroy(gf_class, gf_dev->devt);
    clear_bit(MINOR(gf_dev->devt), minors);
    mutex_unlock(&device_list_lock);

    wakeup_source_trash(&gf_dev->ttw_wl);

    FUNC_EXIT();
    return 0;
}

#if defined(USE_SPI_BUS)
static int gf_suspend(struct spi_device *spi, pm_message_t mesg)
#elif defined(USE_PLATFORM_BUS)
static int gf_suspend(struct platform_device *pdev, pm_message_t state)
#endif
{
    gf_pm_qos_update(false);
    gf_dbg("gf_suspend\n");
    return 0;
}

#if defined(USE_SPI_BUS)
static int gf_resume(struct spi_device *spi)
#elif defined(USE_PLATFORM_BUS)
static int gf_resume(struct platform_device *pdev)
#endif
{
    gf_dbg("gf_resume\n");
    return 0;
}

static struct of_device_id gx_match_table[] = {
    { .compatible = GF_SPIDEV_NAME, },
    { },
};

#if defined(USE_SPI_BUS)
static struct spi_driver gf_driver = {
#elif defined(USE_PLATFORM_BUS)
static struct platform_driver gf_driver = {
#endif
    .driver = {
        .name = GF_DEV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = gx_match_table,
    },
    .probe = gf_probe,
    .remove = gf_remove,
    .suspend = gf_suspend,
    .resume = gf_resume,
};

static int __init gf_init(void)
{
    int status;
    
    FUNC_ENTRY();
    pr_info("gf_init start\n");

    BUILD_BUG_ON(N_SPI_MINORS > 256);

    gf_major = register_chrdev(0, CHRD_DRIVER_NAME, &gf_fops);
    if (gf_major < 0) {
        pr_warn("Failed to register char device\n");
        return gf_major;
    }

    gf_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(gf_class)) {
        unregister_chrdev(gf_major, gf_driver.driver.name);
        pr_warn("Failed to create class\n");
        return PTR_ERR(gf_class);
    }

#if defined(USE_PLATFORM_BUS)
    status = platform_driver_register(&gf_driver);
#elif defined(USE_SPI_BUS)
    status = spi_register_driver(&gf_driver);
#endif

    if (status < 0) {
        class_destroy(gf_class);
        unregister_chrdev(gf_major, gf_driver.driver.name);
        pr_warn("Failed to register driver\n");
        return status;
    }

#ifdef GF_NETLINK_ENABLE
    netlink_init();
#endif

    pr_info("gf_init OK\n");
    FUNC_EXIT();
    return 0;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
#ifdef CONFIG_FB
    struct gf_dev *gf_dev = &gf;
#endif

    FUNC_ENTRY();

#ifdef CONFIG_FB
    fb_unregister_client(&gf_dev->gf_notifier);
#endif

#ifdef GF_NETLINK_ENABLE
    netlink_exit();
#endif

#if defined(USE_PLATFORM_BUS)
    platform_driver_unregister(&gf_driver);
#elif defined(USE_SPI_BUS)
    spi_unregister_driver(&gf_driver);
#endif

    class_destroy(gf_class);
    if (gf_major >= 0)
        unregister_chrdev(gf_major, gf_driver.driver.name);

    FUNC_EXIT();
}
module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("Goodix Fingerprint SPI device interface - Optimized");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf-spi");
