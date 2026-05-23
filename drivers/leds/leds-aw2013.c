/*
 * drivers/leds/leds-aw2013.c
 * AW2013 RGB LED driver for Linux 4.9
 *
 * Sysfs per LED:
 *  - brightness (0..255)
 *  - blink (0/1)
 *  - on_off_ms ("on off" in ms)
 *
 * DTS:
 *  aw2013@45 { ... child nodes with aw2013,name aw2013,id aw2013,max-current ... }
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

/* AW2013 registers */
#define AW_REG_RESET        0x00
#define AW_REG_GLOBAL_CTRL  0x01
#define AW_REG_LED_ENABLE   0x30
#define AW_REG_LED_CFG_BASE 0x31
#define AW_REG_PWM_BASE     0x34
#define AW_REG_T0_BASE      0x37
#define AW_REG_T1_BASE      0x38
#define AW_REG_T2_BASE      0x39

/* bits */
#define AW2013_CHIPID       0x33
#define AW_RESET_MAGIC      0x55
#define AW_GCR_ENABLE       0x01

/* LED_CFG bits (per channel) */
#define AW_CFG_BREATH       0x10
#define AW_CFG_FADE_IN      0x20
#define AW_CFG_FADE_OUT     0x40

#define AW2013_MAX_CHANNELS 3

/* timing table indices:
 * 0=0ms, 1=130, 2=260, 3=380, 4=510, 5=770, 6=1040, 7=1600
 */
static u8 aw_time_ms_to_reg(int ms)
{
    if (ms <= 0) return 0;
    if (ms <= 130) return 1;
    if (ms <= 260) return 2;
    if (ms <= 380) return 3;
    if (ms <= 510) return 4;
    if (ms <= 770) return 5;
    if (ms <= 1040) return 6;
    return 7;
}

struct aw2013_chip;

struct aw2013_led {
    struct aw2013_chip *chip;
    struct led_classdev cdev;
    struct work_struct work;

    u8 id;              /* 0..2 */
    u8 max_current;     /* 0..3 */
    u8 brightness;      /* cached 0..255 */

    bool blinking;
    int on_ms;
    int off_ms;
};

struct aw2013_chip {
    struct i2c_client *client;
    struct mutex lock;

    struct regulator *vdd;
    struct regulator *vcc;
    bool powered;

    u32 power_on_delay_us;

    int num_leds;
    struct aw2013_led *leds;
};

static int aw2013_write(struct aw2013_chip *chip, u8 reg, u8 val)
{
    int ret = i2c_smbus_write_byte_data(chip->client, reg, val);
    if (ret < 0)
        dev_err(&chip->client->dev, "i2c write reg=0x%02x val=0x%02x ret=%d\n",
                reg, val, ret);
    return ret;
}

static int aw2013_read(struct aw2013_chip *chip, u8 reg, u8 *val)
{
    int ret = i2c_smbus_read_byte_data(chip->client, reg);
    if (ret < 0) {
        dev_err(&chip->client->dev, "i2c read reg=0x%02x ret=%d\n", reg, ret);
        return ret;
    }
    *val = (u8)ret;
    return 0;
}

static int aw2013_power_set(struct aw2013_chip *chip, bool on)
{
    int ret;

    if (on) {
        if (chip->powered)
            return 0;

        if (chip->vdd && !IS_ERR(chip->vdd)) {
            ret = regulator_enable(chip->vdd);
            if (ret) {
                dev_err(&chip->client->dev, "enable vdd failed %d\n", ret);
                return ret;
            }
        }

        if (chip->vcc && !IS_ERR(chip->vcc)) {
            ret = regulator_enable(chip->vcc);
            if (ret) {
                dev_err(&chip->client->dev, "enable vcc failed %d\n", ret);
                if (chip->vdd && !IS_ERR(chip->vdd))
                    regulator_disable(chip->vdd);
                return ret;
            }
        }

        if (chip->power_on_delay_us)
            usleep_range(chip->power_on_delay_us, chip->power_on_delay_us + 500);
        else
            usleep_range(2000, 2500);

        ret = aw2013_write(chip, AW_REG_GLOBAL_CTRL, AW_GCR_ENABLE);
        if (ret < 0)
            return ret;

        chip->powered = true;
        return 0;
    }

    if (!chip->powered)
        return 0;

    /* disable chip */
    aw2013_write(chip, AW_REG_GLOBAL_CTRL, 0);

    if (chip->vcc && !IS_ERR(chip->vcc))
        regulator_disable(chip->vcc);
    if (chip->vdd && !IS_ERR(chip->vdd))
        regulator_disable(chip->vdd);

    chip->powered = false;
    return 0;
}

static bool aw2013_any_led_active_locked(struct aw2013_chip *chip)
{
    int i;
    for (i = 0; i < chip->num_leds; i++) {
        if (chip->leds[i].brightness > 0)
            return true;
    }
    return false;
}

/* apply one LED settings to HW (chip lock must be held, power must be on) */
static void aw2013_apply_one_locked(struct aw2013_led *led)
{
    struct aw2013_chip *chip = led->chip;
    u8 cfg;
    u8 enable_mask;
    u8 t_rise, t_hold, t_fall, t_off;
    int ret;

    /* current in low bits (0..3) */
    cfg = (led->max_current & 0x03);

    if (led->brightness == 0) {
        /* just disable bit later in enable pass */
        ret = aw2013_write(chip, AW_REG_PWM_BASE + led->id, 0);
        (void)ret;
        /* set cfg to static */
        ret = aw2013_write(chip, AW_REG_LED_CFG_BASE + led->id, cfg);
        (void)ret;
        return;
    }

    if (led->blinking && led->on_ms > 0 && led->off_ms > 0) {
        cfg |= (AW_CFG_BREATH | AW_CFG_FADE_IN | AW_CFG_FADE_OUT);

        /* split on_ms into rise/fall, keep hold minimal (0) */
        t_rise = aw_time_ms_to_reg(led->on_ms / 2);
        t_hold = 0;
        t_fall = aw_time_ms_to_reg(led->on_ms / 2);
        t_off  = aw_time_ms_to_reg(led->off_ms);

        /* write timings */
        aw2013_write(chip, AW_REG_T0_BASE + led->id * 3, (t_rise << 4) | (t_hold & 0x0F));
        aw2013_write(chip, AW_REG_T1_BASE + led->id * 3, (t_fall << 4) | (t_off & 0x0F));
        aw2013_write(chip, AW_REG_T2_BASE + led->id * 3, 0);
    }

    aw2013_write(chip, AW_REG_LED_CFG_BASE + led->id, cfg);
    aw2013_write(chip, AW_REG_PWM_BASE + led->id, led->brightness);

    /* enable bit computed later */
    (void)enable_mask;
}

static void aw2013_update_enable_locked(struct aw2013_chip *chip)
{
    int i;
    u8 en = 0;

    for (i = 0; i < chip->num_leds; i++) {
        if (chip->leds[i].brightness > 0)
            en |= (1U << chip->leds[i].id);
    }

    aw2013_write(chip, AW_REG_LED_ENABLE, en);
}

/* Work handler per LED: apply whole chip state to avoid races */
static void aw2013_work_fn(struct work_struct *work)
{
    struct aw2013_led *led = container_of(work, struct aw2013_led, work);
    struct aw2013_chip *chip = led->chip;
    int i;
    bool any_on;
    int ret;

    mutex_lock(&chip->lock);

    any_on = aw2013_any_led_active_locked(chip);

    ret = aw2013_power_set(chip, any_on);
    if (ret) {
        mutex_unlock(&chip->lock);
        return;
    }

    if (any_on) {
        for (i = 0; i < chip->num_leds; i++)
            aw2013_apply_one_locked(&chip->leds[i]);

        aw2013_update_enable_locked(chip);
    } else {
        aw2013_update_enable_locked(chip);
        aw2013_power_set(chip, false);
    }

    mutex_unlock(&chip->lock);
}

static void aw2013_brightness_set(struct led_classdev *cdev,
                                 enum led_brightness brightness)
{
    struct aw2013_led *led = container_of(cdev, struct aw2013_led, cdev);

    led->brightness = (u8)brightness;
    if (led->brightness == 0)
        led->blinking = false;

    schedule_work(&led->work);
}

/* Sysfs: blink */
static ssize_t aw2013_blink_store(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t count)
{
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw2013_led *led = container_of(cdev, struct aw2013_led, cdev);
    unsigned long v;

    if (kstrtoul(buf, 10, &v))
        return -EINVAL;

    led->blinking = (v > 0);
    schedule_work(&led->work);
    return count;
}

static ssize_t aw2013_blink_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw2013_led *led = container_of(cdev, struct aw2013_led, cdev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", led->blinking ? 1 : 0);
}

static DEVICE_ATTR(blink, 0664, aw2013_blink_show, aw2013_blink_store);

/* Sysfs: on_off_ms */
static ssize_t aw2013_on_off_ms_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw2013_led *led = container_of(cdev, struct aw2013_led, cdev);
    int on_ms, off_ms;

    if (sscanf(buf, "%d %d", &on_ms, &off_ms) != 2)
        return -EINVAL;

    if (on_ms < 0) on_ms = 0;
    if (off_ms < 0) off_ms = 0;

    led->on_ms = on_ms;
    led->off_ms = off_ms;

    /* do not force blink on here; only apply if blink=1 */
    schedule_work(&led->work);
    return count;
}

static ssize_t aw2013_on_off_ms_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf)
{
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw2013_led *led = container_of(cdev, struct aw2013_led, cdev);
    return scnprintf(buf, PAGE_SIZE, "%d %d\n", led->on_ms, led->off_ms);
}

static DEVICE_ATTR(on_off_ms, 0664, aw2013_on_off_ms_show, aw2013_on_off_ms_store);

static struct attribute *aw2013_led_attrs[] = {
    &dev_attr_blink.attr,
    &dev_attr_on_off_ms.attr,
    NULL,
};

static const struct attribute_group aw2013_led_attr_group = {
    .attrs = aw2013_led_attrs,
};

static int aw2013_check_chipid(struct aw2013_chip *chip)
{
    u8 val = 0;
    int ret;

    ret = aw2013_write(chip, AW_REG_RESET, AW_RESET_MAGIC);
    if (ret < 0)
        return ret;

    usleep_range(2000, 2500);

    ret = aw2013_read(chip, AW_REG_RESET, &val);
    if (ret < 0)
        return ret;

    if (val != AW2013_CHIPID) {
        dev_err(&chip->client->dev, "wrong chipid 0x%02x\n", val);
        return -ENODEV;
    }

    return 0;
}

static int aw2013_parse_child(struct aw2013_chip *chip,
                             struct device_node *np,
                             int idx)
{
    struct aw2013_led *led;
    const char *name;
    u32 tmp;
    int ret;

    led = &chip->leds[idx];
    led->chip = chip;

    ret = of_property_read_string(np, "aw2013,name", &name);
    if (ret < 0)
        return ret;

    led->cdev.name = name;

    tmp = idx;
    (void)of_property_read_u32(np, "aw2013,id", &tmp);
    if (tmp >= AW2013_MAX_CHANNELS)
        return -EINVAL;
    led->id = (u8)tmp;

    tmp = 255;
    (void)of_property_read_u32(np, "aw2013,max-brightness", &tmp);
    led->cdev.max_brightness = tmp;

    tmp = 1;
    (void)of_property_read_u32(np, "aw2013,max-current", &tmp);
    if (tmp > 3) tmp = 3;
    led->max_current = (u8)tmp;

    led->brightness = 0;
    led->blinking = false;
    led->on_ms = 1000;
    led->off_ms = 1000;

    INIT_WORK(&led->work, aw2013_work_fn);
    led->cdev.brightness_set = aw2013_brightness_set;

    ret = led_classdev_register(&chip->client->dev, &led->cdev);
    if (ret)
        return ret;

    ret = sysfs_create_group(&led->cdev.dev->kobj, &aw2013_led_attr_group);
    if (ret) {
        led_classdev_unregister(&led->cdev);
        return ret;
    }

    return 0;
}

static void aw2013_unregister_all(struct aw2013_chip *chip, int count)
{
    int i;
    for (i = 0; i < count; i++) {
        sysfs_remove_group(&chip->leds[i].cdev.dev->kobj, &aw2013_led_attr_group);
        led_classdev_unregister(&chip->leds[i].cdev);
        cancel_work_sync(&chip->leds[i].work);
    }
}

static int aw2013_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    struct device_node *np;
    struct device_node *child;
    struct aw2013_chip *chip;
    int num, ret, idx;

    np = client->dev.of_node;
    if (!np)
        return -EINVAL;

    num = of_get_child_count(np);
    if (num <= 0)
        return -EINVAL;
    if (num > AW2013_MAX_CHANNELS)
        num = AW2013_MAX_CHANNELS;

    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip)
        return -ENOMEM;

    chip->client = client;
    mutex_init(&chip->lock);
    chip->powered = false;

    chip->power_on_delay_us = 0;
    (void)of_property_read_u32(np, "aw2013,power-on-delay-us", &chip->power_on_delay_us);

    chip->vdd = devm_regulator_get_optional(&client->dev, "vdd");
    if (IS_ERR(chip->vdd))
        chip->vdd = NULL;

    chip->vcc = devm_regulator_get_optional(&client->dev, "vcc");
    if (IS_ERR(chip->vcc))
        chip->vcc = NULL;

    chip->leds = devm_kcalloc(&client->dev, num, sizeof(*chip->leds), GFP_KERNEL);
    if (!chip->leds)
        return -ENOMEM;

    chip->num_leds = num;
    i2c_set_clientdata(client, chip);

    /* power on to check chip */
    ret = aw2013_power_set(chip, true);
    if (ret)
        return ret;

    ret = aw2013_check_chipid(chip);
    if (ret) {
        aw2013_power_set(chip, false);
        return ret;
    }

    /* power off until used */
    aw2013_power_set(chip, false);

    idx = 0;
    for_each_child_of_node(np, child) {
        if (idx >= num)
            break;
        ret = aw2013_parse_child(chip, child, idx);
        if (ret) {
            aw2013_unregister_all(chip, idx);
            return ret;
        }
        idx++;
    }

    dev_info(&client->dev, "AW2013 probed, leds=%d\n", idx);
    return 0;
}

static int aw2013_remove(struct i2c_client *client)
{
    struct aw2013_chip *chip = i2c_get_clientdata(client);
    if (!chip)
        return 0;

    aw2013_unregister_all(chip, chip->num_leds);
    aw2013_power_set(chip, false);
    return 0;
}

static void aw2013_shutdown(struct i2c_client *client)
{
    struct aw2013_chip *chip = i2c_get_clientdata(client);
    if (!chip)
        return;

    mutex_lock(&chip->lock);
    aw2013_write(chip, AW_REG_LED_ENABLE, 0);
    aw2013_power_set(chip, false);
    mutex_unlock(&chip->lock);
}

static const struct i2c_device_id aw2013_i2c_id[] = {
    { "aw2013_led", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, aw2013_i2c_id);

static const struct of_device_id aw2013_of_match[] = {
    { .compatible = "awinic,aw2013" },
    { }
};
MODULE_DEVICE_TABLE(of, aw2013_of_match);

static struct i2c_driver aw2013_driver = {
    .probe = aw2013_probe,
    .remove = aw2013_remove,
    .shutdown = aw2013_shutdown,
    .driver = {
        .name = "aw2013_led",
        .of_match_table = aw2013_of_match,
    },
    .id_table = aw2013_i2c_id,
};

module_i2c_driver(aw2013_driver);

MODULE_DESCRIPTION("AW2013 RGB LED driver (markw)");
MODULE_LICENSE("GPL v2");
