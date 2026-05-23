#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

/* Оптимизированные константы */
#define GF_RESET_DELAY_US   3000
#define GF_RESET_HOLD_US    100

#define gf_dbg(fmt, args...) pr_debug("gf: " fmt, ##args)

static int gf3208_request_named_gpio(struct gf_dev *gf_dev, 
                                      const char *label, int *gpio)
{
    struct device *dev = &gf_dev->spi->dev;
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

static int select_pin_ctl(struct gf_dev *gf_dev, const char *name)
{
    size_t i;
    int rc;
    struct device *dev = &gf_dev->spi->dev;

    for (i = 0; i < ARRAY_SIZE(pctl_names); i++) {
        const char *n = pctl_names[i];
        if (!strncmp(n, name, strlen(n))) {
            rc = pinctrl_select_state(gf_dev->fingerprint_pinctrl,
                                      gf_dev->pinctrl_state[i]);
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

int gf_parse_dts(struct gf_dev *gf_dev)
{
    int rc;
    int i;
    struct device *dev = &gf_dev->spi->dev;

    pr_info("gf_parse_dts start\n");

    rc = gf3208_request_named_gpio(gf_dev, "goodix,gpio_reset", 
                                    &gf_dev->reset_gpio);
    if (rc) {
        gf_dbg("Failed to request RESET GPIO, rc = %d\n", rc);
        return -EPERM;
    }

    rc = gf3208_request_named_gpio(gf_dev, "goodix,gpio_irq", 
                                    &gf_dev->irq_gpio);
    if (rc) {
        gf_dbg("Failed to request IRQ GPIO, rc = %d\n", rc);
        return -EPERM;
    }

    gf_dev->fingerprint_pinctrl = devm_pinctrl_get(dev);
    if (IS_ERR(gf_dev->fingerprint_pinctrl)) {
        rc = PTR_ERR(gf_dev->fingerprint_pinctrl);
        dev_err(dev, "Failed to get pinctrl, rc = %d\n", rc);
        gf_dev->fingerprint_pinctrl = NULL;
        return rc;
    }

    for (i = 0; i < ARRAY_SIZE(pctl_names); i++) {
        const char *n = pctl_names[i];
        struct pinctrl_state *state;
        
        state = pinctrl_lookup_state(gf_dev->fingerprint_pinctrl, n);
        if (IS_ERR(state)) {
            pr_err("cannot find '%s'\n", n);
            rc = PTR_ERR(state);
            goto err_pinctrl;
        }
        pr_debug("found pin control %s\n", n);
        gf_dev->pinctrl_state[i] = state;
    }

    rc = select_pin_ctl(gf_dev, "goodixfp_reset_active");
    if (rc)
        goto err_pinctrl;

    rc = select_pin_ctl(gf_dev, "goodixfp_irq_active");
    if (rc)
        goto err_pinctrl;

    pr_info("gf_parse_dts OK\n");
    return 0;

err_pinctrl:
    if (gf_dev->fingerprint_pinctrl) {
        devm_pinctrl_put(gf_dev->fingerprint_pinctrl);
        gf_dev->fingerprint_pinctrl = NULL;
    }
    return rc;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
    struct device *dev = &gf_dev->spi->dev;

    gf_dbg("gf_cleanup\n");

    if (gpio_is_valid(gf_dev->irq_gpio)) {
        devm_gpio_free(dev, gf_dev->irq_gpio);
        gf_dev->irq_gpio = -EINVAL;
        gf_dbg("irq_gpio freed\n");
    }

    if (gpio_is_valid(gf_dev->reset_gpio)) {
        devm_gpio_free(dev, gf_dev->reset_gpio);
        gf_dev->reset_gpio = -EINVAL;
        gf_dbg("reset_gpio freed\n");
    }

    if (gf_dev->fingerprint_pinctrl) {
        devm_pinctrl_put(gf_dev->fingerprint_pinctrl);
        gf_dev->fingerprint_pinctrl = NULL;
        gf_dbg("pinctrl released\n");
    }
}

int gf_power_on(struct gf_dev *gf_dev)
{
    usleep_range(10000, 10100);
    pr_debug("gf power on\n");
    return 0;
}

int gf_power_off(struct gf_dev *gf_dev)
{
    pr_debug("gf power off\n");
    return 0;
}

static int hw_reset(struct gf_dev *gf_dev)
{
    int rc;
    int irq_gpio;
    struct device *dev = &gf_dev->spi->dev;

    rc = select_pin_ctl(gf_dev, "goodixfp_reset_reset");
    if (rc)
        return rc;

    usleep_range(GF_RESET_DELAY_US, GF_RESET_DELAY_US + 100);

    rc = select_pin_ctl(gf_dev, "goodixfp_reset_active");
    if (rc)
        return rc;

    usleep_range(GF_RESET_HOLD_US, GF_RESET_HOLD_US + 50);

    irq_gpio = gpio_get_value(gf_dev->irq_gpio);
    dev_dbg(dev, "IRQ after reset: %d\n", irq_gpio);

    return 0;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
    int rc;

    if (!gf_dev) {
        pr_err("gf_hw_reset: NULL pointer\n");
        return -EINVAL;
    }

    rc = hw_reset(gf_dev);
    if (!rc && delay_ms)
        usleep_range(delay_ms * 1000, delay_ms * 1000 + 100);

    return rc;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
    if (!gf_dev) {
        pr_err("gf_irq_num: NULL pointer\n");
        return -EINVAL;
    }
    
    return gpio_to_irq(gf_dev->irq_gpio);
}
