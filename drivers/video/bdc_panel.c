/*
 * Copyright (c) 2016 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <backlight.h>
#include <dm.h>
#include <panel.h>
#include <asm/gpio.h>
#include <power/regulator.h>

DECLARE_GLOBAL_DATA_PTR;

struct bdc_panel_priv {
	struct udevice *reg;
	struct udevice *backlight;
	struct gpio_desc enable;
};

static int bdc_panel_enable_backlight(struct udevice *dev)
{
	struct bdc_panel_priv *priv = dev_get_priv(dev);
	int ret;

	printf("%s: start, backlight = '%s'\n", __func__, priv->backlight->name);
	dm_gpio_set_value(&priv->enable, 1);
	ret = backlight_enable(priv->backlight);
	printf("%s: done, ret = %d\n", __func__, ret);
	if (ret)
		return ret;

	return 0;
}

static int bdc_panel_ofdata_to_platdata(struct udevice *dev)
{
	struct bdc_panel_priv *priv = dev_get_priv(dev);
	int ret;

	if (IS_ENABLED(CONFIG_DM_REGULATOR)) {
		ret = uclass_get_device_by_phandle(UCLASS_REGULATOR, dev,
						   "power-supply", &priv->reg);
		if (ret) {
			printf("%s: Warning: cannot get power supply: ret=%d\n",
			      __func__, ret);
			if (ret != -ENOENT)
				return ret;
		}
	}
	ret = uclass_get_device_by_phandle(UCLASS_PANEL_BACKLIGHT, dev,
					   "backlight", &priv->backlight);
	if (ret) {
		printf("%s: Cannot get backlight: ret=%d\n", __func__, ret);
		return ret;
	}
	ret = gpio_request_by_name(dev, "enable-gpios", 0, &priv->enable,
				   GPIOD_IS_OUT);
	if (ret) {
		printf("%s: Warning: cannot get enable GPIO: ret=%d\n",
		      __func__, ret);
		if (ret != -ENOENT)
			return ret;
	}

	return 0;
}

static int bdc_panel_probe(struct udevice *dev)
{
	struct bdc_panel_priv *priv = dev_get_priv(dev);
	int ret;

	if (IS_ENABLED(CONFIG_DM_REGULATOR) && priv->reg) {
		printf("%s: Enable regulator '%s'\n", __func__, priv->reg->name);
		ret = regulator_set_enable(priv->reg, true);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct panel_ops bdc_panel_ops = {
	.enable_backlight	= bdc_panel_enable_backlight,
};

static const struct udevice_id bdc_panel_ids[] = {
	{ .compatible = "bdc,wvga" },
	{ }
};

U_BOOT_DRIVER(bdc_panel) = {
	.name	= "bdc_panel",
	.id	= UCLASS_PANEL,
	.of_match = bdc_panel_ids,
	.ops	= &bdc_panel_ops,
	.ofdata_to_platdata	= bdc_panel_ofdata_to_platdata,
	.probe		= bdc_panel_probe,
	.priv_auto_alloc_size	= sizeof(struct bdc_panel_priv),
};