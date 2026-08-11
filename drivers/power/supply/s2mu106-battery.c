// SPDX-License-Identifier: GPL-2.0
/*
 * Battery Fuel Gauge Driver for Samsung S2MU106 PMIC.
 *
 * Modelled on the Samsung S2MU005 fuel gauge driver, which uses the same
 * register layout. The S2MU106 differs only in the MONOUT_SEL codes used to
 * route the average voltage/current measurements, and requires a short
 * settling delay before reading them back.
 *
 * Copyright (C) 2015 Samsung Electronics
 * Copyright (C) 2026 <your name here>
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/units.h>

#define S2MU106_FG_REG_STATUS		0x00
#define S2MU106_FG_REG_IRQ		0x02
#define S2MU106_FG_REG_RVBAT		0x04
#define S2MU106_FG_REG_RCURCC		0x06
#define S2MU106_FG_REG_RSOC		0x08
#define S2MU106_FG_REG_MONOUT		0x0a
#define S2MU106_FG_REG_MONOUTSEL	0x0c
#define S2MU106_FG_REG_RBATCAP		0x0e
#define S2MU106_FG_REG_IRQLVL		0x1a
#define S2MU106_FG_REG_START		0x1e
/* Enables the analogue VBAT measurement path before averaging. */
#define S2MU106_FG_REG_VBATCAL		0x40

#define S2MU106_FG_MONOUTSEL_AVGVOLTAGE	0x16
#define S2MU106_FG_MONOUTSEL_AVGCURRENT	0x17
#define S2MU106_FG_MONOUTSEL_OFF	0x10

#define S2MU106_FG_VBATCAL_ENABLE	0x08

/* The MONOUT mux needs time to settle after selection before being read. */
#define S2MU106_FG_MONOUT_SETTLE_MS	50

struct s2mu106_fg {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct mutex monout_mutex;
};

static const struct regmap_config s2mu106_fg_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static irqreturn_t s2mu106_handle_irq(int irq, void *data)
{
	struct s2mu106_fg *priv = data;

	msleep(100);
	power_supply_changed(priv->psy);

	return IRQ_HANDLED;
}

static int s2mu106_fg_get_voltage_now(struct s2mu106_fg *priv, int *value)
{
	u32 val;
	int ret;

	ret = regmap_read(priv->regmap, S2MU106_FG_REG_RVBAT, &val);
	if (ret < 0) {
		dev_err(priv->dev, "failed to read voltage register (%d)\n", ret);
		return ret;
	}

	*value = (val * MICRO) >> 13;

	return 0;
}

static int s2mu106_fg_get_voltage_avg(struct s2mu106_fg *priv, int *value)
{
	struct regmap *regmap = priv->regmap;
	u32 val;
	int ret;

	mutex_lock(&priv->monout_mutex);

	ret = regmap_write(regmap, S2MU106_FG_REG_VBATCAL,
			   S2MU106_FG_VBATCAL_ENABLE);
	if (ret < 0) {
		dev_err(priv->dev, "failed to enable VBAT measurement path (%d)\n",
			ret);
		goto unlock;
	}

	ret = regmap_write(regmap, S2MU106_FG_REG_MONOUTSEL,
			   S2MU106_FG_MONOUTSEL_AVGVOLTAGE);
	if (ret < 0) {
		dev_err(priv->dev, "failed to enable average voltage monitoring (%d)\n",
			ret);
		goto unlock;
	}

	msleep(S2MU106_FG_MONOUT_SETTLE_MS);

	ret = regmap_read(regmap, S2MU106_FG_REG_MONOUT, &val);
	if (ret < 0) {
		dev_err(priv->dev, "failed to read voltage register (%d)\n", ret);
		goto unlock;
	}

	*value = (val * MICRO) >> 12;

	regmap_write(regmap, S2MU106_FG_REG_MONOUTSEL, S2MU106_FG_MONOUTSEL_OFF);

unlock:
	mutex_unlock(&priv->monout_mutex);

	return ret;
}

static int s2mu106_fg_get_current_now(struct s2mu106_fg *priv, int *value)
{
	u32 val;
	int ret;

	ret = regmap_read(priv->regmap, S2MU106_FG_REG_RCURCC, &val);
	if (ret < 0) {
		dev_err(priv->dev, "failed to read current register (%d)\n", ret);
		return ret;
	}

	/* Positive while charging, negative while discharging. */
	*value = -((s16)val * MICRO) >> 12;

	return 0;
}

static int s2mu106_fg_get_current_avg(struct s2mu106_fg *priv, int *value)
{
	struct regmap *regmap = priv->regmap;
	u32 val;
	int ret;

	mutex_lock(&priv->monout_mutex);

	ret = regmap_write(regmap, S2MU106_FG_REG_MONOUTSEL,
			   S2MU106_FG_MONOUTSEL_AVGCURRENT);
	if (ret < 0) {
		dev_err(priv->dev, "failed to enable average current monitoring (%d)\n",
			ret);
		goto unlock;
	}

	msleep(S2MU106_FG_MONOUT_SETTLE_MS);

	ret = regmap_read(regmap, S2MU106_FG_REG_MONOUT, &val);
	if (ret < 0) {
		dev_err(priv->dev, "failed to read current register (%d)\n", ret);
		goto unlock;
	}

	*value = -((s16)val * MICRO) >> 12;

	regmap_write(regmap, S2MU106_FG_REG_MONOUTSEL, S2MU106_FG_MONOUTSEL_OFF);

unlock:
	mutex_unlock(&priv->monout_mutex);

	return ret;
}

static int s2mu106_fg_get_capacity(struct s2mu106_fg *priv, int *value)
{
	u32 val;
	int ret;

	ret = regmap_read(priv->regmap, S2MU106_FG_REG_RSOC, &val);
	if (ret < 0) {
		dev_err(priv->dev, "failed to read capacity register (%d)\n", ret);
		return ret;
	}

	*value = (val * CENTI) >> 14;

	return 0;
}

static int s2mu106_fg_get_status(struct s2mu106_fg *priv, int *value)
{
	int current_now, current_avg, capacity;
	int ret;

	ret = s2mu106_fg_get_current_now(priv, &current_now);
	if (ret < 0)
		return ret;

	ret = s2mu106_fg_get_current_avg(priv, &current_avg);
	if (ret < 0)
		return ret;

	/*
	 * Verify both current values reported to reduce inaccuracies due to
	 * internal hysteresis.
	 */
	if (current_now < 0 && current_avg < 0) {
		*value = POWER_SUPPLY_STATUS_DISCHARGING;
	} else if (current_now == 0) {
		*value = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		*value = POWER_SUPPLY_STATUS_CHARGING;

		ret = s2mu106_fg_get_capacity(priv, &capacity);
		if (!ret && capacity > 98)
			*value = POWER_SUPPLY_STATUS_FULL;
		return ret;
	}

	return 0;
}

static const enum power_supply_property s2mu106_fg_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_STATUS,
};

static int s2mu106_fg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct s2mu106_fg *priv = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		/*
		 * The gauge itself has no battery-detect bit (downstream's
		 * driver doesn't expose one either) - presence detection
		 * only exists in the charger's STATUS3 register. The pack
		 * is non-removable on the A40, so report it as always
		 * present, same as other fixed-battery drivers do.
		 */
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return s2mu106_fg_get_voltage_now(priv, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		return s2mu106_fg_get_voltage_avg(priv, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return s2mu106_fg_get_current_now(priv, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		return s2mu106_fg_get_current_avg(priv, &val->intval);
	case POWER_SUPPLY_PROP_CAPACITY:
		return s2mu106_fg_get_capacity(priv, &val->intval);
	case POWER_SUPPLY_PROP_STATUS:
		return s2mu106_fg_get_status(priv, &val->intval);
	default:
		return -EINVAL;
	}
}

static const struct power_supply_desc s2mu106_fg_desc = {
	.name = "s2mu106-fuel-gauge",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = s2mu106_fg_properties,
	.num_properties = ARRAY_SIZE(s2mu106_fg_properties),
	.get_property = s2mu106_fg_get_property,
};

static int s2mu106_fg_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s2mu106_fg *priv;
	struct power_supply_config psy_cfg = {};
	const struct power_supply_desc *psy_desc;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);
	priv->dev = dev;

	priv->regmap = devm_regmap_init_i2c(client, &s2mu106_fg_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap),
				     "failed to initialize regmap\n");

	ret = devm_mutex_init(dev, &priv->monout_mutex);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize MONOUT mutex\n");

	psy_desc = device_get_match_data(dev);

	psy_cfg.drv_data = priv;
	psy_cfg.fwnode = dev_fwnode(dev);
	priv->psy = devm_power_supply_register(priv->dev, psy_desc, &psy_cfg);
	if (IS_ERR(priv->psy))
		return dev_err_probe(dev, PTR_ERR(priv->psy),
				     "failed to register power supply subsystem\n");

	ret = devm_request_threaded_irq(priv->dev, client->irq, NULL,
					s2mu106_handle_irq, IRQF_ONESHOT,
					psy_desc->name, priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	return 0;
}

static const struct of_device_id s2mu106_fg_of_match_table[] = {
	{
		.compatible = "samsung,s2mu106-fuel-gauge",
		.data = &s2mu106_fg_desc,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, s2mu106_fg_of_match_table);

static struct i2c_driver s2mu106_fg_i2c_driver = {
	.probe = s2mu106_fg_i2c_probe,
	.driver = {
		.name = "s2mu106-fuel-gauge",
		.of_match_table = s2mu106_fg_of_match_table,
	},
};
module_i2c_driver(s2mu106_fg_i2c_driver);

MODULE_DESCRIPTION("Samsung S2MU106 PMIC Battery Fuel Gauge Driver");
MODULE_LICENSE("GPL");
