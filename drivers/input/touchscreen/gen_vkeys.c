/* Copyright (c) 2013, 2018 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/input/gen_vkeys.h>

#define MAX_BUF_SIZE	256
#define VKEY_VER_CODE	"0x01"

#define HEIGHT_SCALE_NUM 8
#define HEIGHT_SCALE_DENOM 10

#define VKEY_Y_OFFSET_DEFAULT 0

/* numerator and denomenator for border equations */
#define BORDER_ADJUST_NUM 3
#define BORDER_ADJUST_DENOM 4

struct vkey_sysfs_data {
	struct list_head	entry;
	struct device		*dev;
	struct kobj_attribute	attr;
	char			buf[MAX_BUF_SIZE];
};

static LIST_HEAD(vkey_list_head);
static DEFINE_MUTEX(vkey_mutex);

static struct kobject *vkey_obj = NULL;
static struct attribute *vkey_attr = NULL;
static struct attribute_group vkey_grp = {
	.attrs = &vkey_attr,
};

static ssize_t vkey_show(struct kobject  *obj,
		struct kobj_attribute *attr, char *buf)
{
	struct vkey_sysfs_data *sysfs_data;

	sysfs_data = container_of(attr, struct vkey_sysfs_data, attr);
	strlcpy(buf, sysfs_data->buf, MAX_BUF_SIZE);
	return strnlen(buf, MAX_BUF_SIZE);
}

static struct vkey_sysfs_data * find_dev(struct device *dev)
{
	struct list_head *pos;
	struct vkey_sysfs_data *sysfs_data;

	list_for_each(pos, &vkey_list_head) {
		sysfs_data = list_entry(pos, struct vkey_sysfs_data, entry);
		if (sysfs_data->dev == dev)
			return sysfs_data;
	}
	return NULL;
}

static int create_sysfs_dir(struct device *dev)
{
	int ret = 0;

	mutex_lock(&vkey_mutex);

	if (vkey_obj == NULL) {
		vkey_obj = kobject_create_and_add("board_properties", NULL);
		if (!vkey_obj) {
			ret = -ENOMEM;
			dev_err(dev, "unable to create kobject\n");
			goto end;
		}

		ret = sysfs_create_group(vkey_obj, &vkey_grp);
		if (ret) {
			kobject_put(vkey_obj);
			vkey_obj = NULL;
			dev_err(dev, "failed to create attributes\n");
		}
	}

    end:
	mutex_unlock(&vkey_mutex);
	return ret;
}

static int add_sysfs_file(struct device *dev, struct vkey_sysfs_data *sysfs_data)
{
	int ret = 0;

	mutex_lock(&vkey_mutex);

	if (find_dev(dev) != NULL) {
		ret = -EEXIST;
		dev_err(dev, "gen-vkeys device is registered already\n");
		goto end;
	}

	ret = sysfs_add_file_to_group(vkey_obj, &sysfs_data->attr.attr, NULL);
	if (ret) {
		dev_err(dev, "failed to create attributes\n");
		goto end;
	}

	sysfs_data->dev = dev;
	list_add(&sysfs_data->entry, &vkey_list_head);

    end:
	mutex_unlock(&vkey_mutex);
	return ret;
}

static int remove_sysfs_file(struct device *dev)
{
	struct vkey_sysfs_data *sysfs_data;

	mutex_lock(&vkey_mutex);

	sysfs_data = find_dev(dev);
	if (sysfs_data != NULL) {
		list_del(&sysfs_data->entry);
		sysfs_remove_file_from_group(vkey_obj, &sysfs_data->attr.attr, NULL);
	}

	if (list_empty(&vkey_list_head)) {
		sysfs_remove_group(vkey_obj, &vkey_grp);
		kobject_put(vkey_obj);
		vkey_obj = NULL;
	}

	mutex_unlock(&vkey_mutex);
	return 0;
}

static int vkey_parse_dt(struct device *dev,
			struct vkeys_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	struct property *prop;
	int rc, val;

	rc = of_property_read_string(np, "label", &pdata->name);
	if (rc) {
		dev_err(dev, "Failed to read label\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "qcom,disp-maxx", &pdata->disp_maxx);
	if (rc) {
		dev_err(dev, "Failed to read display max x\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "qcom,disp-maxy", &pdata->disp_maxy);
	if (rc) {
		dev_err(dev, "Failed to read display max y\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "qcom,panel-maxx", &pdata->panel_maxx);
	if (rc) {
		dev_err(dev, "Failed to read panel max x\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "qcom,panel-maxy", &pdata->panel_maxy);
	if (rc) {
		dev_err(dev, "Failed to read panel max y\n");
		return -EINVAL;
	}

	prop = of_find_property(np, "qcom,key-codes", NULL);
	if (prop) {
		pdata->num_keys = prop->length / sizeof(u32);
		pdata->keycodes = devm_kzalloc(dev,
			sizeof(u32) * pdata->num_keys, GFP_KERNEL);
		if (!pdata->keycodes)
			return -ENOMEM;
		rc = of_property_read_u32_array(np, "qcom,key-codes",
				pdata->keycodes, pdata->num_keys);
		if (rc) {
			dev_err(dev, "Failed to read key codes\n");
			return -EINVAL;
		}
	}

	pdata->y_offset = VKEY_Y_OFFSET_DEFAULT;
	rc = of_property_read_u32(np, "qcom,y-offset", &val);
	if (!rc)
		pdata->y_offset = val;
	else if (rc != -EINVAL) {
		dev_err(dev, "Failed to read y position offset\n");
		return rc;
	}
	return 0;
}

static int vkeys_probe(struct platform_device *pdev)
{
	struct vkey_sysfs_data *sysfs_data;
	struct vkeys_platform_data *pdata;
	int width, height, center_x, center_y;
	int x1 = 0, x2 = 0, i, c = 0, ret, border;
	char *name;

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct vkeys_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&pdev->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = vkey_parse_dt(&pdev->dev, pdata);
		if (ret) {
			dev_err(&pdev->dev, "Parsing DT failed(%d)", ret);
			return ret;
		}
	} else
		pdata = pdev->dev.platform_data;

	if (!pdata || !pdata->name || !pdata->keycodes || !pdata->num_keys ||
		!pdata->disp_maxx || !pdata->disp_maxy || !pdata->panel_maxy) {
		dev_err(&pdev->dev, "pdata is invalid\n");
		return -EINVAL;
	}

	sysfs_data = devm_kzalloc(&pdev->dev, sizeof(struct vkey_sysfs_data), GFP_KERNEL);
	if (!sysfs_data)
		return -ENOMEM;

	border = (pdata->panel_maxx - pdata->disp_maxx) * 2;
	width = ((pdata->disp_maxx - (border * (pdata->num_keys - 1)))
			/ pdata->num_keys);
	height = (pdata->panel_maxy - pdata->disp_maxy);
	center_y = pdata->disp_maxy + (height / 2) + pdata->y_offset;
	height = height * HEIGHT_SCALE_NUM / HEIGHT_SCALE_DENOM;

	x2 -= border * BORDER_ADJUST_NUM / BORDER_ADJUST_DENOM;

	for (i = 0; i < pdata->num_keys; i++) {
		x1 = x2 + border;
		x2 = x2 + border + width;
		center_x = x1 + (x2 - x1) / 2;
		c += snprintf(sysfs_data->buf + c, MAX_BUF_SIZE - c,
				"%s:%d:%d:%d:%d:%d\n",
				VKEY_VER_CODE, pdata->keycodes[i],
				center_x, center_y, width, height);
	}

	sysfs_data->buf[c] = '\0';

	name = devm_kzalloc(&pdev->dev, sizeof(*name) * MAX_BUF_SIZE,
					GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	snprintf(name, MAX_BUF_SIZE,
				"virtualkeys.%s", pdata->name);

	sysfs_data->dev = &pdev->dev;
	sysfs_data->attr.attr.name = name;
	sysfs_data->attr.attr.mode = 0444;
	sysfs_data->attr.show = vkey_show;

	ret = create_sysfs_dir(&pdev->dev);
	if (ret)
		return ret;

	ret = add_sysfs_file(&pdev->dev, sysfs_data);
	if (ret) {
		dev_err(&pdev->dev, "failed to create attributes\n");
		remove_sysfs_file(NULL);
		return ret;
	}
	return 0;
}

static int vkeys_remove(struct platform_device *pdev)
{
	return remove_sysfs_file(&pdev->dev);
}

static struct of_device_id vkey_match_table[] = {
	{ .compatible = "qcom,gen-vkeys",},
	{ },
};

static struct platform_driver vkeys_driver = {
	.probe = vkeys_probe,
	.remove = vkeys_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "gen_vkeys",
		.of_match_table = vkey_match_table,
	},
};

module_platform_driver(vkeys_driver);
MODULE_LICENSE("GPL v2");
