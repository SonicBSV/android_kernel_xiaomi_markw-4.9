/*
 * FPC1020 Fingerprint sensor device driver
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 * Copyright (C) 2018 XiaoMi, Inc.
 * Copyright (C) 2024 Optimized version
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <soc/qcom/scm.h>

/* Оптимизированные тайминги (микросекунды) */
#define FPC1020_RESET_LOW_US      1000
#define FPC1020_RESET_HIGH1_US    100
#define FPC1020_RESET_HIGH2_US    1250
#define PWR_ON_STEP_SLEEP         100
#define PWR_ON_STEP_RANGE1        100
#define PWR_ON_STEP_RANGE2        900
#define FPC_TTW_HOLD_TIME         1000
#define NUM_PARAMS_REG_ENABLE_SET 2

static const char * const pctl_names[] = {
    "fpc1020_reset_reset",
    "fpc1020_reset_active",
    "fpc1020_irq_active",
};

struct fpc1020_data {
    struct device *dev;
    struct pinctrl *fingerprint_pinctrl;
    struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
#ifdef LINUX_CONTROL_SPI_CLK
    u32 max_speed_hz;
    struct clk *iface_clk;
    struct clk *core_clk;
    bool clocks_enabled;
    bool clocks_suspended;
#endif
    struct wakeup_source ttw_wl;
    struct pm_qos_request pm_qos_req;
    int irq_gpio;
    int rst_gpio;
    struct mutex lock;
    bool prepared;
    atomic_t wakeup_enabled;
    bool compatible_enabled;
    bool pm_qos_enabled;
};

static struct kernfs_node *soc_symlink;

/* Forward declarations */
static irqreturn_t fpc1020_irq_handler(int irq, void *handle);
static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
                                       const char *label, int *gpio);
static int hw_reset(struct fpc1020_data *fpc1020);

/* PM QoS helper */
static inline void fpc_pm_qos_update(struct fpc1020_data *fpc1020, bool enable)
{
    if (enable && !fpc1020->pm_qos_enabled) {
        pm_qos_update_request(&fpc1020->pm_qos_req, 100);
        fpc1020->pm_qos_enabled = true;
    } else if (!enable && fpc1020->pm_qos_enabled) {
        pm_qos_update_request(&fpc1020->pm_qos_req, PM_QOS_DEFAULT_VALUE);
        fpc1020->pm_qos_enabled = false;
    }
}

#ifdef LINUX_CONTROL_SPI_CLK
static int set_clks(struct fpc1020_data *fpc1020, bool enable)
{
    int rc = 0;

    mutex_lock(&fpc1020->lock);

    if (enable == fpc1020->clocks_enabled)
        goto out;

    if (enable) {
        rc = clk_set_rate(fpc1020->core_clk, fpc1020->max_speed_hz);
        if (rc) {
            dev_err(fpc1020->dev, "%s: Error setting clk_rate: %u, %d\n",
                    __func__, fpc1020->max_speed_hz, rc);
            goto out;
        }

        rc = clk_prepare_enable(fpc1020->core_clk);
        if (rc) {
            dev_err(fpc1020->dev, "%s: Error enabling core clk: %d\n",
                    __func__, rc);
            goto out;
        }

        rc = clk_prepare_enable(fpc1020->iface_clk);
        if (rc) {
            dev_err(fpc1020->dev, "%s: Error enabling iface clk: %d\n",
                    __func__, rc);
            clk_disable_unprepare(fpc1020->core_clk);
            goto out;
        }

        dev_dbg(fpc1020->dev, "%s: clk rate %u hz\n", __func__,
                fpc1020->max_speed_hz);
        fpc1020->clocks_enabled = true;
    } else {
        clk_disable_unprepare(fpc1020->iface_clk);
        clk_disable_unprepare(fpc1020->core_clk);
        fpc1020->clocks_enabled = false;
    }

out:
    mutex_unlock(&fpc1020->lock);
    return rc;
}

static ssize_t clk_enable_set(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
    int rc = set_clks(fpc1020, (*buf == '1'));
    return rc ? rc : count;
}
static DEVICE_ATTR(clk_enable, S_IWUSR, NULL, clk_enable_set);
#endif /* LINUX_CONTROL_SPI_CLK */

static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name)
{
    size_t i;
    int rc;
    struct device *dev = fpc1020->dev;

    for (i = 0; i < ARRAY_SIZE(pctl_names); i++) {
        const char *n = pctl_names[i];
        if (!strncmp(n, name, strlen(n))) {
            rc = pinctrl_select_state(fpc1020->fingerprint_pinctrl,
                                      fpc1020->pinctrl_state[i]);
            if (rc)
                dev_err(dev, "cannot select '%s'\n", name);
            else
                dev_dbg(dev, "Selected '%s'\n", name);
            return rc;
        }
    }
    
    dev_err(dev, "%s: '%s' not found\n", __func__, name);
    return -EINVAL;
}

static ssize_t pinctl_set(struct device *dev,
                          struct device_attribute *attr,
                          const char *buf, size_t count)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
    int rc = select_pin_ctl(fpc1020, buf);
    return rc ? rc : count;
}
static DEVICE_ATTR(pinctl_set, S_IWUSR, NULL, pinctl_set);

static int hw_reset(struct fpc1020_data *fpc1020)
{
    int rc;
    int irq_gpio;
    struct device *dev = fpc1020->dev;

    rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
    if (rc)
        return rc;
    usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 50);

    rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
    if (rc)
        return rc;
    usleep_range(FPC1020_RESET_LOW_US, FPC1020_RESET_LOW_US + 50);

    rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
    if (rc)
        return rc;
    usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 50);

    irq_gpio = gpio_get_value(fpc1020->irq_gpio);
    dev_dbg(dev, "IRQ after reset: %d\n", irq_gpio);

    return 0;
}

static ssize_t hw_reset_set(struct device *dev,
                            struct device_attribute *attr,
                            const char *buf, size_t count)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
    int rc;

    if (!strncmp(buf, "reset", strlen("reset")))
        rc = hw_reset(fpc1020);
    else
        return -EINVAL;

    return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

static int device_prepare(struct fpc1020_data *fpc1020, bool enable)
{
    int rc = 0;

    mutex_lock(&fpc1020->lock);

    if (enable && !fpc1020->prepared) {
        fpc1020->prepared = true;
        select_pin_ctl(fpc1020, "fpc1020_reset_reset");
        usleep_range(PWR_ON_STEP_SLEEP, PWR_ON_STEP_RANGE2);
        select_pin_ctl(fpc1020, "fpc1020_reset_active");
        usleep_range(PWR_ON_STEP_SLEEP, PWR_ON_STEP_RANGE1);
    } else if (!enable && fpc1020->prepared) {
        select_pin_ctl(fpc1020, "fpc1020_reset_reset");
        usleep_range(PWR_ON_STEP_SLEEP, PWR_ON_STEP_RANGE2);
        fpc1020->prepared = false;
    }

    mutex_unlock(&fpc1020->lock);
    return rc;
}

static ssize_t spi_prepare_set(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t count)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
    int rc;

    if (!strncmp(buf, "enable", strlen("enable")))
        rc = device_prepare(fpc1020, true);
    else if (!strncmp(buf, "disable", strlen("disable")))
        rc = device_prepare(fpc1020, false);
    else
        return -EINVAL;

    return rc ? rc : count;
}
static DEVICE_ATTR(spi_prepare, S_IWUSR, NULL, spi_prepare_set);

static ssize_t wakeup_enable_set(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

    if (!strncmp(buf, "enable", strlen("enable")))
        atomic_set(&fpc1020->wakeup_enabled, 1);
    else if (!strncmp(buf, "disable", strlen("disable")))
        atomic_set(&fpc1020->wakeup_enabled, 0);
    else
        return -EINVAL;

    return count;
}
static DEVICE_ATTR(wakeup_enable, S_IWUSR, NULL, wakeup_enable_set);

static ssize_t irq_get(struct device *device,
                       struct device_attribute *attribute,
                       char *buffer)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
    int irq = gpio_get_value(fpc1020->irq_gpio);
    return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}

static ssize_t irq_ack(struct device *device,
                       struct device_attribute *attribute,
                       const char *buffer, size_t count)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
    dev_dbg(fpc1020->dev, "%s\n", __func__);
    return count;
}
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

static ssize_t compatible_all_set(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
    int rc;
    int i;
    int irqf;

    dev_dbg(dev, "compatible_all: enabled=%d\n", fpc1020->compatible_enabled);

    if (!strncmp(buf, "enable", strlen("enable")) && 
        !fpc1020->compatible_enabled) {
        
        rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_irq",
                                         &fpc1020->irq_gpio);
        if (rc)
            goto exit;

        rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_rst",
                                         &fpc1020->rst_gpio);
        if (rc)
            goto exit;

        fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
        if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
            rc = PTR_ERR(fpc1020->fingerprint_pinctrl);
            if (rc == -EPROBE_DEFER)
                dev_info(dev, "pinctrl not ready\n");
            else
                dev_err(dev, "Target does not use pinctrl\n");
            fpc1020->fingerprint_pinctrl = NULL;
            goto exit;
        }

        for (i = 0; i < ARRAY_SIZE(pctl_names); i++) {
            const char *n = pctl_names[i];
            struct pinctrl_state *state;
            
            state = pinctrl_lookup_state(fpc1020->fingerprint_pinctrl, n);
            if (IS_ERR(state)) {
                dev_err(dev, "cannot find '%s'\n", n);
                rc = PTR_ERR(state);
                goto exit;
            }
            dev_dbg(dev, "found pin control %s\n", n);
            fpc1020->pinctrl_state[i] = state;
        }

        rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
        if (rc)
            goto exit;

        rc = select_pin_ctl(fpc1020, "fpc1020_irq_active");
        if (rc)
            goto exit;

        irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_SUSPEND;
        if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup"))
            device_init_wakeup(dev, 1);

        rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
                                        NULL, fpc1020_irq_handler, irqf,
                                        dev_name(dev), fpc1020);
        if (rc) {
            dev_err(dev, "could not request irq %d\n",
                    gpio_to_irq(fpc1020->irq_gpio));
            goto exit;
        }
        dev_dbg(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

        enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));
        fpc1020->compatible_enabled = true;

        if (of_property_read_bool(dev->of_node, "fpc,enable-on-boot")) {
            dev_info(dev, "Enabling hardware\n");
            device_prepare(fpc1020, true);
#ifdef LINUX_CONTROL_SPI_CLK
            set_clks(fpc1020, false);
#endif
        }
        
    } else if (!strncmp(buf, "disable", strlen("disable")) && 
               fpc1020->compatible_enabled) {
        
        if (gpio_is_valid(fpc1020->irq_gpio)) {
            devm_gpio_free(dev, fpc1020->irq_gpio);
            pr_debug("irq_gpio freed\n");
        }
        
        if (gpio_is_valid(fpc1020->rst_gpio)) {
            devm_gpio_free(dev, fpc1020->rst_gpio);
            pr_debug("rst_gpio freed\n");
        }
        
        devm_free_irq(dev, gpio_to_irq(fpc1020->irq_gpio), fpc1020);
        fpc1020->compatible_enabled = false;
    } else {
        goto exit;
    }

    hw_reset(fpc1020);
    return count;

exit:
    return -EINVAL;
}
static DEVICE_ATTR(compatible_all, S_IWUSR, NULL, compatible_all_set);

static struct attribute *attributes[] = {
    &dev_attr_pinctl_set.attr,
    &dev_attr_spi_prepare.attr,
    &dev_attr_hw_reset.attr,
    &dev_attr_wakeup_enable.attr,
    &dev_attr_compatible_all.attr,
#ifdef LINUX_CONTROL_SPI_CLK
    &dev_attr_clk_enable.attr,
#endif
    &dev_attr_irq.attr,
    NULL
};

static const struct attribute_group attribute_group = {
    .attrs = attributes,
};

/* Оптимизированный обработчик прерываний */
static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
    struct fpc1020_data *fpc1020 = handle;

    dev_dbg(fpc1020->dev, "%s\n", __func__);

    /* Активируем PM QoS для быстрой обработки */
    fpc_pm_qos_update(fpc1020, true);

    if (atomic_read(&fpc1020->wakeup_enabled))
        __pm_wakeup_event(&fpc1020->ttw_wl, FPC_TTW_HOLD_TIME);

    sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);

    return IRQ_HANDLED;
}

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
                                       const char *label, int *gpio)
{
    struct device *dev = fpc1020->dev;
    struct device_node *np = dev->of_node;
    int rc;

    rc = of_get_named_gpio(np, label, 0);
    if (rc < 0) {
        dev_err(dev, "failed to get '%s'\n", label);
        return rc;
    }
    
    *gpio = rc;
    
    rc = devm_gpio_request(dev, *gpio, label);
    if (rc) {
        dev_err(dev, "failed to request gpio %d\n", *gpio);
        return rc;
    }
    
    dev_dbg(dev, "%s: gpio %d\n", label, *gpio);
    return 0;
}

static int fpc1020_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct fpc1020_data *fpc1020;
    struct device_node *np = dev->of_node;
    struct device *platform_dev;
    struct kobject *soc_kobj;
    struct kernfs_node *devices_node, *soc_node;
    int rc = 0;

    fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020), GFP_KERNEL);
    if (!fpc1020) {
        dev_err(dev, "failed to allocate memory\n");
        return -ENOMEM;
    }

    fpc1020->dev = dev;
    fpc1020->pm_qos_enabled = false;
    dev_set_drvdata(dev, fpc1020);

    if (!np) {
        dev_err(dev, "no of node found\n");
        return -EINVAL;
    }

#ifdef LINUX_CONTROL_SPI_CLK
    if (of_property_read_u32(np, "fpc,spi-max-frequency", 
                              &fpc1020->max_speed_hz)) {
        fpc1020->max_speed_hz = 4800000;
        dev_warn(dev, "Using default spi-max-frequency\n");
    }

    fpc1020->iface_clk = clk_get(dev, "iface_clk");
    if (IS_ERR(fpc1020->iface_clk)) {
        dev_err(dev, "Failed to get iface_clk\n");
        return -EINVAL;
    }

    fpc1020->core_clk = clk_get(dev, "core_clk");
    if (IS_ERR(fpc1020->core_clk)) {
        dev_err(dev, "Failed to get core_clk\n");
        clk_put(fpc1020->iface_clk);
        return -EINVAL;
    }

    fpc1020->clocks_enabled = false;
    fpc1020->clocks_suspended = false;
#endif

    atomic_set(&fpc1020->wakeup_enabled, 1);
    fpc1020->compatible_enabled = false;

    mutex_init(&fpc1020->lock);
    wakeup_source_init(&fpc1020->ttw_wl, "fpc_ttw_wl");

    /* Инициализация PM QoS */
    pm_qos_add_request(&fpc1020->pm_qos_req, PM_QOS_CPU_DMA_LATENCY, 
                       PM_QOS_DEFAULT_VALUE);

    rc = sysfs_create_group(&dev->kobj, &attribute_group);
    if (rc) {
        dev_err(dev, "could not create sysfs\n");
        goto err_sysfs;
    }

    /* Создание symlink для совместимости */
    if (dev->parent && dev->parent->parent) {
        platform_dev = dev->parent->parent;
        if (!strcmp(kobject_name(&platform_dev->kobj), "platform")) {
            devices_node = platform_dev->kobj.sd->parent;
            soc_kobj = &dev->parent->kobj;
            soc_node = soc_kobj->sd;
            kernfs_get(soc_node);

            soc_symlink = kernfs_create_link(devices_node, 
                                              kobject_name(soc_kobj), 
                                              soc_node);
            kernfs_put(soc_node);
            if (IS_ERR(soc_symlink))
                dev_warn(dev, "Unable to create soc symlink\n");
        }
    }

    dev_info(dev, "%s: OK\n", __func__);
    return 0;

err_sysfs:
    pm_qos_remove_request(&fpc1020->pm_qos_req);
    wakeup_source_trash(&fpc1020->ttw_wl);
    mutex_destroy(&fpc1020->lock);
    return rc;
}

static int fpc1020_remove(struct platform_device *pdev)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(&pdev->dev);

    if (!IS_ERR_OR_NULL(soc_symlink))
        kernfs_remove_by_name(soc_symlink->parent, soc_symlink->name);

    sysfs_remove_group(&pdev->dev.kobj, &attribute_group);
    pm_qos_remove_request(&fpc1020->pm_qos_req);
    wakeup_source_trash(&fpc1020->ttw_wl);
    mutex_destroy(&fpc1020->lock);

    dev_info(&pdev->dev, "%s\n", __func__);
    return 0;
}

#ifdef LINUX_CONTROL_SPI_CLK
static int fpc1020_suspend(struct device *dev)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

    fpc_pm_qos_update(fpc1020, false);
    fpc1020->clocks_suspended = fpc1020->clocks_enabled;
    set_clks(fpc1020, false);

    return 0;
}

static int fpc1020_resume(struct device *dev)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

    if (fpc1020->clocks_suspended)
        set_clks(fpc1020, true);

    return 0;
}

static const struct dev_pm_ops fpc1020_pm_ops = {
    .suspend = fpc1020_suspend,
    .resume = fpc1020_resume,
};
#endif

static struct of_device_id fpc1020_of_match[] = {
    { .compatible = "soc:fpc1020", },
    { }
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
    .driver = {
        .name = "fpc1020",
        .owner = THIS_MODULE,
        .of_match_table = fpc1020_of_match,
#ifdef LINUX_CONTROL_SPI_CLK
        .pm = &fpc1020_pm_ops,
#endif
    },
    .probe = fpc1020_probe,
    .remove = fpc1020_remove,
};

static int __init fpc1020_init(void)
{
    int rc = platform_driver_register(&fpc1020_driver);
    
    if (!rc)
        pr_info("%s: OK\n", __func__);
    else
        pr_err("%s: failed (%d)\n", __func__, rc);
    
    return rc;
}

static void __exit fpc1020_exit(void)
{
    pr_info("%s\n", __func__);
    platform_driver_unregister(&fpc1020_driver);
}

module_init(fpc1020_init);
module_exit(fpc1020_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver - Optimized");
