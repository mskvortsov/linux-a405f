// SPDX-License-Identifier: GPL-2.0+
//
// Copyright (c) 2012-2014 Samsung Electronics Co., Ltd
//              http://www.samsung.com

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/samsung/s2mpu08.h>
#include <linux/mfd/samsung/core.h>

struct s2mpu08_info {
	struct regulator_dev *rdev[S2MPU08_REGULATOR_MAX];
	unsigned int opmode[S2MPU08_REGULATOR_MAX];
	struct s2mpu08_dev *iodev;
	struct mutex lock;
	struct i2c_client *i2c;
};

static int s2mpu08_get_ramp_delay(int ramp_delay)
{
	unsigned char cnt = 0;

	ramp_delay /= 6000;

	while (true) {
		ramp_delay = ramp_delay >> 1;
		if (ramp_delay == 0)
			break;
		cnt++;
	}
	return cnt;
}

static int s2mpu08_set_ramp_delay(struct regulator_dev *rdev, int ramp_delay)
{
	unsigned int ramp_value, ramp_shift, ramp_reg;
	int rdev_id = rdev_get_id(rdev);

	ramp_value = s2mpu08_get_ramp_delay(ramp_delay);
	if (ramp_value > 4) {
		pr_warn("%s: ramp_delay: %d not supported\n",
			rdev->desc->name, ramp_delay);
	}

	switch (rdev_id) {
	case S2MPU08_BUCK1:
	case S2MPU08_BUCK6:
	case S2MPU08_BUCK7:
		ramp_shift = S2MPU08_BUCK1_RAMP_SHIFT;
		break;
	case S2MPU08_BUCK2:
	case S2MPU08_BUCK3:
		ramp_shift = S2MPU08_BUCK2_RAMP_SHIFT;
		break;
	case S2MPU08_BUCK4:
	case S2MPU08_BUCK5:
		ramp_shift = S2MPU08_BUCK4_RAMP_SHIFT;
		break;
	case S2MPU08_BUCK8:
		ramp_shift = S2MPU08_BUCK8_RAMP_SHIFT;
		break;
	default:
		return 0;
	}
	ramp_reg = S2MPU08_REG_BUCKRAMP;

	return regmap_update_bits(rdev->regmap, ramp_reg,
				  S2MPU08_BUCK_RAMP_MASK << ramp_shift,
				  ramp_value << ramp_shift);
}

static int s2mpu08_regulator_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	unsigned int val;
	int ret;
	struct s2mpu08_info *s2mpu08 = rdev_get_drvdata(rdev);
	int rdev_id = rdev_get_id(rdev);
	val = mode << S2MPU08_ENABLE_SHIFT;
	ret = regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
				rdev->desc->enable_mask,
				val);
	if (ret)
		return ret;
	s2mpu08->opmode[rdev_id] = val;
	return 0;
}

static int s2mpu08_regulator_is_enabled(struct regulator_dev *rdev)
{
	int ret;
	struct s2mpu08_info *s2mpu08 = rdev_get_drvdata(rdev);
	int rdev_id = rdev_get_id(rdev);
	unsigned int val, val2, val3;

	/* if pmic_rev is not 0 w/a is unnecessary */
	if (s2mpu08->iodev->pmic_rev == 0x0 && rdev_id == S2MPU08_LDO35)
		goto ldo35_36_workaround;

	if (rdev_id == S2MPU08_BUCK5)
		return 0;

	else {
		ret = regmap_read(rdev->regmap,
				rdev->desc->enable_reg, &val);
		if (ret)
			return ret;
	}

	if (rdev->desc->enable_is_inverted)
		return (val & rdev->desc->enable_mask) == 0;
	else
		return (val & rdev->desc->enable_mask) != 0;

ldo35_36_workaround:

	regmap_read(rdev->regmap, 0xFF, &val);

	if ((val & 0x08) == 0x08)
		return 1;

	else {
		regmap_read(rdev->regmap, S2MPU08_REG_L36CTRL, &val2);
		regmap_read(rdev->regmap, 0x75, &val3);

		if (((val2 & 0xC0) != 0x00) && ((val3 & 0xF0) == 0x00))
			return 1;
		else
			return 0;
	}
}

static int s2mpu08_regulator_enable(struct regulator_dev *rdev)
{
	struct s2mpu08_info *s2mpu08 = rdev_get_drvdata(rdev);
	int rdev_id = rdev_get_id(rdev);
	unsigned int val;

	/* if pmic_rev is not 0 w/a is unnecessary */
	if (s2mpu08->iodev->pmic_rev == 0x0 &&
			(rdev_id == S2MPU08_LDO35 || rdev_id == S2MPU08_LDO36))
				goto ldo35_36_workaround;

	/*
	 * The BUCKs are SoC DVFS rails owned by ACPM. On mainline there is no
	 * DVFS framework to coordinate with, and the live rails sit in a
	 * different on-mode than the driver's default opmode (e.g. vdd_mif is
	 * in mode 0b01 while enable would write 0b11) - so issuing the enable
	 * flips a live rail's control mode and browns it out (hard hang).
	 * Leave every BUCK to ACPM/the bootloader by making enable a no-op
	 * (the downstream already does this for BUCK5 alone).
	 */
	if (rdev_id >= S2MPU08_BUCK1)
		return 0;

	return regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
				  rdev->desc->enable_mask,
				  s2mpu08->opmode[rdev_id]);

ldo35_36_workaround:

	if (rdev_id == S2MPU08_LDO35) {
		regmap_read(rdev->regmap, S2MPU08_REG_L36CTRL, &val);

		/* if LDO36 is on */
		if ((val & 0xC0) != 0x00) {
			regmap_update_bits(rdev->regmap, 0x75, 0xF0, 0x00);
			regmap_update_bits(rdev->regmap, 0xFF, 0x08, 0x00);
			return 0;
		}

		else {
			regmap_update_bits(rdev->regmap, 0x75, 0xF0, 0xF0);
			regmap_update_bits(rdev->regmap, 0xFF, 0x08, 0x08);
			return 0;
		}
	}

	/* rdev_id == S2MPU08_LDO36 */
	else {
		regmap_read(rdev->regmap, 0xFF, &val);

		/* if LDO35 is on */
		if ((val & 0x08) == 0x08) {
			regmap_update_bits(rdev->regmap, 0x75, 0xF0, 0x00);
			regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
				rdev->desc->enable_mask, 0xC0);
			/* LDO35 should be off when LDO36 is on */
			regmap_update_bits(rdev->regmap, 0xFF, 0x08, 0x00);
			return 0;
		}

		else {
			regmap_update_bits(rdev->regmap, 0x75, 0xF0, 0xF0);
			regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
				rdev->desc->enable_mask, 0xC0);
			return 0;
		}
	}
}

static int s2mpu08_regulator_disable(struct regulator_dev *rdev) {
	struct s2mpu08_info *s2mpu08 = rdev_get_drvdata(rdev);
	int val;
	int rdev_id = rdev_get_id(rdev);

	/* The BUCKs belong to ACPM; see s2mpu08_regulator_enable(). */
	if (rdev_id >= S2MPU08_BUCK1)
		return 0;

	if (s2mpu08->iodev->pmic_rev == 0x0 &&
			(rdev_id == S2MPU08_LDO35 || rdev_id == S2MPU08_LDO36))
				goto ldo35_36_workaround;

	if (rdev->desc->enable_is_inverted)
		val = rdev->desc->enable_mask;
	else
		val = 0;

	return regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
				  rdev->desc->enable_mask, val);

ldo35_36_workaround:

	if (rdev_id == S2MPU08_LDO35) {
		regmap_read(rdev->regmap, S2MPU08_REG_L36CTRL, &val);

		/* if LDO36 is on */
		if ((val & 0xC0) != 0x00) {
			regmap_update_bits(rdev->regmap, 0x75, 0xF0, 0xF0);
			regmap_update_bits(rdev->regmap, 0xFF, 0x08, 0x00);
			return 0;
		}

		else {
			regmap_update_bits(rdev->regmap, 0x75, 0xF0, 0x00);
			regmap_update_bits(rdev->regmap, 0xFF, 0x08, 0x00);
			return 0;
		}
	}

	/* rdev_id == S2MPU08_LDO36 */
	else {
		regmap_read(rdev->regmap, 0x75, &val);

		/* if LDO35 is on */
		if ((val & 0xF0) == 0x00) {
			regmap_update_bits(rdev->regmap, 0xFF, 0x08, 0x08);
			regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
				rdev->desc->enable_mask, 0);
			regmap_update_bits(rdev->regmap, 0x75, 0xF0, 0xF0);
			return 0;
		}
		else {
			regmap_update_bits(rdev->regmap, 0xFF, 0x08, 0x00);
			regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
				rdev->desc->enable_mask, 0x00);
			return 0;
		}
	}
}

static int s2mpu08_dummy_operation(struct regulator_dev *rdev) {
	return 0;
}

static const struct regulator_ops s2mpu08_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= s2mpu08_regulator_is_enabled,
	.enable			= s2mpu08_regulator_enable,
	.disable		= s2mpu08_regulator_disable,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
	.set_mode		= s2mpu08_regulator_set_mode,
};

static const struct regulator_ops s2mpu08_buck_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= s2mpu08_regulator_is_enabled,
	.enable			= s2mpu08_regulator_enable,
	.disable		= s2mpu08_regulator_disable,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
	.set_mode		= s2mpu08_regulator_set_mode,
	.set_ramp_delay		= s2mpu08_set_ramp_delay,
};
static const struct regulator_ops s2mpu08_buck5_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= s2mpu08_dummy_operation,
	.enable			= s2mpu08_dummy_operation,
	.disable		= s2mpu08_dummy_operation,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
	.set_mode		= s2mpu08_regulator_set_mode,
	.set_ramp_delay		= s2mpu08_set_ramp_delay,
};

#define regulator_desc_s2mpu08_ldo1 {			\
	.name		= "LDO1",			\
	.id		= S2MPU08_LDO1,			\
	.ops		= &s2mpu08_ldo_ops,		\
	.type		= REGULATOR_VOLTAGE,		\
	.owner		= THIS_MODULE,			\
	.min_uV		= MIN_700_MV,			\
	.uV_step	= S2MPU08_LDO_STEP1,		\
	.n_voltages	= S2MPU08_LDO_N_VOLTAGES,	\
	.ramp_delay	= RAMP_DELAY_12_MVUS,		\
	.vsel_reg	= S2MPU08_REG_L1CTRL,		\
	.vsel_mask	= S2MPU08_LDO_VSEL_MASK,	\
	.enable_reg	= S2MPU08_REG_L1CTRL,		\
	.enable_mask	= S2MPU08_ENABLE_MASK,		\
	.enable_time	= S2MPU08_ENABLE_TIME_LDO	\
}

#define regulator_desc_s2mpu08_ldo2 {			\
	.name		= "LDO2",			\
	.id		= S2MPU08_LDO2,			\
	.ops		= &s2mpu08_ldo_ops,		\
	.type		= REGULATOR_VOLTAGE,		\
	.owner		= THIS_MODULE,			\
	.min_uV		= MIN_1800_MV,			\
	.uV_step	= S2MPU08_LDO_STEP2,		\
	.n_voltages	= S2MPU08_LDO_N_VOLTAGES,	\
	.ramp_delay	= RAMP_DELAY_12_MVUS,		\
	.vsel_reg	= S2MPU08_REG_L2CTRL1,		\
	.vsel_mask	= S2MPU08_LDO_VSEL_MASK,	\
	.enable_reg	= S2MPU08_REG_L2CTRL1,		\
	.enable_mask	= S2MPU08_ENABLE_MASK,		\
	.enable_time	= S2MPU08_ENABLE_TIME_LDO	\
}

#define regulator_desc_s2mpu08_ldo(num, min, step) {	\
	.name		= "LDO"#num,			\
	.id		= S2MPU08_LDO##num,		\
	.ops		= &s2mpu08_ldo_ops,		\
	.type		= REGULATOR_VOLTAGE,		\
	.owner		= THIS_MODULE,			\
	.min_uV		= min,				\
	.uV_step	= step,				\
	.n_voltages	= S2MPU08_LDO_N_VOLTAGES,	\
	.ramp_delay	= RAMP_DELAY_12_MVUS,		\
	.vsel_reg	= S2MPU08_REG_L3CTRL + num - 3,	\
	.vsel_mask	= S2MPU08_LDO_VSEL_MASK,	\
	.enable_reg	= S2MPU08_REG_L3CTRL + num - 3,	\
	.enable_mask	= S2MPU08_ENABLE_MASK,		\
	.enable_time	= S2MPU08_ENABLE_TIME_LDO	\
}

/*
 * LDO33-37 are not contiguous with LDO3-14 in the register map (L33CTRL
 * starts at 0x4D, after a gap), so they need their own L33CTRL base
 * instead of the L3CTRL + num - 3 formula above.
 */
#define regulator_desc_s2mpu08_ldo_h(num, min, step) {	\
	.name		= "LDO"#num,			\
	.id		= S2MPU08_LDO##num,		\
	.ops		= &s2mpu08_ldo_ops,		\
	.type		= REGULATOR_VOLTAGE,		\
	.owner		= THIS_MODULE,			\
	.min_uV		= min,				\
	.uV_step	= step,				\
	.n_voltages	= S2MPU08_LDO_N_VOLTAGES,	\
	.ramp_delay	= RAMP_DELAY_12_MVUS,		\
	.vsel_reg	= S2MPU08_REG_L33CTRL + num - 33, \
	.vsel_mask	= S2MPU08_LDO_VSEL_MASK,	\
	.enable_reg	= S2MPU08_REG_L33CTRL + num - 33, \
	.enable_mask	= S2MPU08_ENABLE_MASK,		\
	.enable_time	= S2MPU08_ENABLE_TIME_LDO	\
}

#define regulator_desc_s2mpu08_buck12346(num) {			\
	.name		= "BUCK"#num,				\
	.id		= S2MPU08_BUCK##num,			\
	.ops		= &s2mpu08_buck_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= MIN_500_MV,				\
	.uV_step	= S2MPU08_BUCK_STEP1,			\
	.n_voltages	= S2MPU08_BUCK_N_VOLTAGES,		\
	.ramp_delay	= RAMP_DELAY_12_MVUS,			\
	.vsel_reg	= S2MPU08_REG_B1CTRL2 + (num - 1) * 2,	\
	.vsel_mask	= S2MPU08_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPU08_REG_B1CTRL1 + (num - 1) * 2,	\
	.enable_mask	= S2MPU08_ENABLE_MASK,			\
	.enable_time	= S2MPU08_ENABLE_TIME_BUCK##num		\
}

#define regulator_desc_s2mpu08_buck5(num) {			\
	.name		= "BUCK"#num,				\
	.id		= S2MPU08_BUCK##num,			\
	.ops		= &s2mpu08_buck5_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= MIN_500_MV,				\
	.uV_step	= S2MPU08_BUCK_STEP1,			\
	.n_voltages	= S2MPU08_BUCK_N_VOLTAGES,		\
	.ramp_delay	= RAMP_DELAY_12_MVUS,			\
	.vsel_reg	= S2MPU08_REG_B1CTRL2 + (num - 1) * 2,	\
	.vsel_mask	= S2MPU08_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPU08_REG_B1CTRL1 + (num - 1) * 2,	\
	.enable_mask	= S2MPU08_ENABLE_MASK,			\
	.enable_time	= S2MPU08_ENABLE_TIME_BUCK##num		\
}

#define regulator_desc_s2mpu08_buck78(num, min) {		\
	.name		= "BUCK"#num,				\
	.id		= S2MPU08_BUCK##num,			\
	.ops		= &s2mpu08_buck_ops,			\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= min,					\
	.uV_step	= S2MPU08_BUCK_STEP2,			\
	.n_voltages	= S2MPU08_BUCK_N_VOLTAGES,		\
	.ramp_delay	= RAMP_DELAY_12_MVUS,			\
	.vsel_reg	= S2MPU08_REG_B7CTRL2 + (num - 7) * 3,	\
	.vsel_mask	= S2MPU08_BUCK_VSEL_MASK,		\
	.enable_reg	= S2MPU08_REG_B7CTRL1 + (num - 7) * 3,	\
	.enable_mask	= S2MPU08_ENABLE_MASK,			\
	.enable_time	= S2MPU08_ENABLE_TIME_BUCK##num		\
}

static const struct regulator_desc s2mpu08_regulators[] = {
	regulator_desc_s2mpu08_ldo1,
	regulator_desc_s2mpu08_ldo2,
	regulator_desc_s2mpu08_ldo(3,  MIN_800_MV,  S2MPU08_LDO_STEP2),
	regulator_desc_s2mpu08_ldo(4,  MIN_500_MV,  S2MPU08_LDO_STEP1),
	regulator_desc_s2mpu08_ldo(5,  MIN_800_MV,  S2MPU08_LDO_STEP1),
	regulator_desc_s2mpu08_ldo(6,  MIN_800_MV,  S2MPU08_LDO_STEP1),
	regulator_desc_s2mpu08_ldo(7,  MIN_800_MV,  S2MPU08_LDO_STEP2),
	regulator_desc_s2mpu08_ldo(8,  MIN_500_MV,  S2MPU08_LDO_STEP2),
	regulator_desc_s2mpu08_ldo(9,  MIN_500_MV,  S2MPU08_LDO_STEP2),
	regulator_desc_s2mpu08_ldo(10, MIN_500_MV,  S2MPU08_LDO_STEP2),
	regulator_desc_s2mpu08_ldo(11, MIN_500_MV,  S2MPU08_LDO_STEP2),
	regulator_desc_s2mpu08_ldo(12, MIN_800_MV,  S2MPU08_LDO_STEP1),
	regulator_desc_s2mpu08_ldo(13, MIN_800_MV,  S2MPU08_LDO_STEP2),
	regulator_desc_s2mpu08_ldo(14, MIN_1800_MV, S2MPU08_LDO_STEP2),
	regulator_desc_s2mpu08_ldo_h(33, MIN_800_MV,  S2MPU08_LDO_STEP2),
	regulator_desc_s2mpu08_ldo_h(34, MIN_1800_MV, S2MPU08_LDO_STEP2),
	regulator_desc_s2mpu08_ldo_h(35, MIN_1800_MV, S2MPU08_LDO_STEP2),
	regulator_desc_s2mpu08_ldo_h(36, MIN_800_MV,  S2MPU08_LDO_STEP2),
	regulator_desc_s2mpu08_ldo_h(37, MIN_1800_MV, S2MPU08_LDO_STEP2),
	regulator_desc_s2mpu08_buck12346(1),
	regulator_desc_s2mpu08_buck12346(2),
	regulator_desc_s2mpu08_buck12346(3),
	regulator_desc_s2mpu08_buck12346(4),
	regulator_desc_s2mpu08_buck5(5),
	regulator_desc_s2mpu08_buck12346(6),
	regulator_desc_s2mpu08_buck78(7, MIN_1200_MV),
	regulator_desc_s2mpu08_buck78(8, MIN_1800_MV),
};

static int s2mpu08_pmic_dt_parse(struct platform_device *pdev,
		struct of_regulator_match *rdata, struct s2mpu08_info *s2mpu08,
		unsigned int rdev_num)
{
	struct device_node *reg_np;
	unsigned int i;

	reg_np = of_get_child_by_name(pdev->dev.parent->of_node, "regulators");
	if (!reg_np) {
		dev_err(&pdev->dev, "could not find regulators sub-node\n");
		return -EINVAL;
	}

	/* of_regulator_match() matches the DT nodes against rdata[].name. */
	for (i = 0; i < rdev_num; i++)
		rdata[i].name = s2mpu08_regulators[i].name;

	of_regulator_match(&pdev->dev, reg_np, rdata, rdev_num);

	of_node_put(reg_np);

	return 0;
}

/*
 * The S2MPU08 registers are reached over ACPM. Bridge regmap to the ACPM
 * register helpers so the regulator core can use the standard regmap ops.
 * The context is the PMIC sub-block handle (iodev->pmic).
 */
static int s2mpu08_acpm_regmap_read(void *context, unsigned int reg,
				    unsigned int *val)
{
	u8 v;
	int ret;

	ret = s2mpu08_read_reg(context, reg, &v);
	if (ret)
		return ret;

	*val = v;
	return 0;
}

static int s2mpu08_acpm_regmap_write(void *context, unsigned int reg,
				     unsigned int val)
{
	return s2mpu08_write_reg(context, reg, val);
}

static const struct regmap_config s2mpu08_regmap_config = {
	.name = "s2mpu08",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.reg_read = s2mpu08_acpm_regmap_read,
	.reg_write = s2mpu08_acpm_regmap_write,
};

static int s2mpu08_pmic_probe(struct platform_device *pdev)
{
	struct s2mpu08_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct of_regulator_match *rdata = NULL;
	struct regulator_config config = { };
	struct s2mpu08_info *s2mpu08;
	unsigned int rdev_num = 0;
	int i, ret = 0;

	s2mpu08 = devm_kzalloc(&pdev->dev, sizeof(struct s2mpu08_info),
				GFP_KERNEL);
	if (!s2mpu08)
		return -ENOMEM;

	s2mpu08->iodev = iodev;
	s2mpu08->i2c = iodev->pmic;

	mutex_init(&s2mpu08->lock);

	platform_set_drvdata(pdev, s2mpu08);

	config.regmap = devm_regmap_init(&pdev->dev, NULL, iodev->pmic,
					 &s2mpu08_regmap_config);
	if (IS_ERR(config.regmap))
		return PTR_ERR(config.regmap);

	rdev_num = ARRAY_SIZE(s2mpu08_regulators);

	rdata = kcalloc(rdev_num, sizeof(*rdata), GFP_KERNEL);
	if (!rdata)
		return -ENOMEM;

	ret = s2mpu08_pmic_dt_parse(pdev, rdata, s2mpu08, rdev_num);
	if (ret)
		goto out;

	for (i = 0; i < rdev_num; i++) {
		config.dev = &pdev->dev;
		config.driver_data = s2mpu08;
		config.init_data = rdata[i].init_data;
		config.of_node = rdata[i].of_node;
		s2mpu08->opmode[i] = S2MPU08_ENABLE_MASK;

		s2mpu08->rdev[i] = devm_regulator_register(&pdev->dev,
						&s2mpu08_regulators[i], &config);
		if (IS_ERR(s2mpu08->rdev[i])) {
			ret = PTR_ERR(s2mpu08->rdev[i]);
			dev_err(&pdev->dev, "regulator init failed for %d\n", i);
			goto out;
		}
	}

	/*
	 * Downstream also sets RTCBUF bit 1 and rewrites the SELMIF routing
	 * (SEL_CTRL1 = 0x7E, SEL_CTRL2[6:0] = 0: LDO2,4-8,33-35 follow
	 * PWREN_MIF, LDO1,10-14 follow PWREN). Those only matter for
	 * sleep-mode rail control, which mainline does not use yet, so leave
	 * whatever the bootloader programmed and just report it.
	 */
	{
		unsigned int sel1 = 0, sel2 = 0;

		regmap_read(config.regmap, S2MPU08_REG_SEL_CTRL1, &sel1);
		regmap_read(config.regmap, S2MPU08_REG_SEL_CTRL2, &sel2);
		dev_info(&pdev->dev, "SEL_CTRL1 0x%02x SEL_CTRL2 0x%02x\n",
			 sel1, sel2);
	}

	/* initialize LDO35,36 for revision 0 (pmic_rev is read by the MFD) */
	if (iodev->pmic_rev == 0x00) {
		regmap_update_bits(config.regmap, 0x75, 0xF0, 0xF0);
		regmap_update_bits(config.regmap, 0x7C, 0x08, 0x08);
		regmap_update_bits(config.regmap, 0x8B, 0x60, 0x00);
	}

	/* changed water out THD in codec side */
	//regmap_write(iodev->close, 0x83, 0x74);
	//regmap_write(iodev->close, 0x84, 0x0E);
	/* changed water jack in THD in codec side */
	//regmap_read(iodev->close, 0x7C, &flag_wtp);
	//if ((flag_wtp & BIT(7)) == false) {
	//	regmap_update_bits(iodev->close, 0x7C, BIT(7), BIT(7));
	//	regmap_read(iodev->close, 0x82, &lowr_wtp);
	//	lowr_wtp -= 15;
	//	regmap_write(iodev->close, 0x82, lowr_wtp);
	//}
out:
	kfree(rdata);

	return ret;
}

static const struct platform_device_id s2mpu08_pmic_id[] = {
	{ "s2mpu08-regulator", S2MPU08X},
	{ },
};
MODULE_DEVICE_TABLE(platform, s2mpu08_pmic_id);

static struct platform_driver s2mpu08_pmic_driver = {
	.driver = {
		.name = "s2mpu08-regulator",
		.owner = THIS_MODULE,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = s2mpu08_pmic_probe,
	.id_table = s2mpu08_pmic_id,
};

module_platform_driver(s2mpu08_pmic_driver);

/* Module information */
MODULE_AUTHOR("CHANGE ME <changeme@gmail.com>");
MODULE_DESCRIPTION("Samsung S2MPU08 Regulator Driver");
MODULE_LICENSE("GPL");
