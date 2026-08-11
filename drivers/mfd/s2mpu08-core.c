
/*
 * s2mpu08.c - mfd core driver for the s2mpu08
 *
 * Copyright (C) 2016 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/err.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mfd/core.h>
#include <linux/mfd/samsung/s2mpu08.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/rtc.h>
#include <linux/slab.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#endif /* CONFIG_OF */

#define I2C_ADDR_TOP	0x00
#define I2C_ADDR_PMIC	0x01
#define I2C_ADDR_RTC	0x02
#define I2C_ADDR_CODEC 	0x03
#define I2C_ADDR_CODECA 0x07
#define I2C_ADDR_CLOSE	0x0F

#define MFD_DEV_NAME "s2mpu08"

struct device_node *acpm_mfd_node;

struct sec_acpm_shared_bus_context {
	const struct acpm_handle *acpm;
	unsigned int acpm_chan_id;
	u8 speedy_channel;
};

struct sec_acpm_bus_context {
	struct sec_acpm_shared_bus_context *shared;
	u8 type;
#define SEC_ACPM_TYPE_COMMON  0x00
#define SEC_ACPM_TYPE_PMIC    0x01
#define SEC_ACPM_TYPE_RTC     0x02
#define SEC_ACPM_TYPE_METER   0x0a
#define SEC_ACPM_TYPE_WLWP    0x0b
#define SEC_ACPM_TYPE_TRIM    0x0f
};


static const struct mfd_cell s2mpu08_devs[] = {
	{ .name = "s2mpu08-regulator", },
//	{ .name = "s2mpu08-rtc", },
};

#if 1
int s2mpu08_read_codec_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	int ret;

	ret = i2c_smbus_read_byte_data(i2c, reg);

	if (ret < 0) {
		pr_info("%s:%s reg(0x%x), ret(%d)\n",
				MFD_DEV_NAME, __func__, reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_EXYNOS_ACPM_PROTOCOL)
int s2mpu08_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct s2mpu08_dev *s2mpu08 = i2c_get_clientdata(i2c);
	struct acpm_pmic_ops ops = s2mpu08->acpm->ops->pmic;
	int ret;

	ret = ops.read_reg(s2mpu08->acpm, s2mpu08->acpm_channel_id,
				i2c->addr, reg, s2mpu08->speedy_channel,
				dest);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return 0;
}

int s2mpu08_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpu08_dev *s2mpu08 = i2c_get_clientdata(i2c);
	struct acpm_pmic_ops ops = s2mpu08->acpm->ops->pmic;
	int ret;

	ret = ops.bulk_read(s2mpu08->acpm, s2mpu08->acpm_channel_id,
				i2c->addr, reg, count, s2mpu08->speedy_channel,
				buf);
//	ret = acpm_pmic_bulk_read(i2c->addr, reg, count, buf);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return 0;
}

int s2mpu08_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct s2mpu08_dev *s2mpu08 = i2c_get_clientdata(i2c);
	struct acpm_pmic_ops ops = s2mpu08->acpm->ops->pmic;
	int ret;

	ret = ops.write_reg(s2mpu08->acpm, s2mpu08->acpm_channel_id,
				i2c->addr, reg, s2mpu08->speedy_channel,
				value);
//	ret = acpm_pmic_write_reg(i2c->addr, reg, value);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return ret;
}

int s2mpu08_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpu08_dev *s2mpu08 = i2c_get_clientdata(i2c);
	struct acpm_pmic_ops ops = s2mpu08->acpm->ops->pmic;
	int ret;

	ret = ops.bulk_write(s2mpu08->acpm, s2mpu08->acpm_channel_id,
				i2c->addr, reg, count, s2mpu08->speedy_channel,
				buf);
//	ret = acpm_pmic_bulk_write(i2c->addr, reg, count, buf);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return ret;
}

int s2mpu08_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct s2mpu08_dev *s2mpu08 = i2c_get_clientdata(i2c);
	struct acpm_pmic_ops ops = s2mpu08->acpm->ops->pmic;
	int ret;

	ret = ops.update_reg(s2mpu08->acpm, s2mpu08->acpm_channel_id,
				i2c->addr, reg, s2mpu08->speedy_channel,
				val, mask);
//	ret = acpm_pmic_update_reg(i2c->addr, reg, val, mask);
	if (ret) {
		pr_err("[%s] acpm ipc fail!\n", __func__);
		return ret;
	}
	return ret;
}
#else
int s2mpu08_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct s2mpu08_dev *s2mpu08 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&s2mpu08->i2c_lock);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	mutex_unlock(&s2mpu08->i2c_lock);
	if (ret < 0) {
		pr_info("%s:%s reg(0x%x), ret(%d)\n",
			 MFD_DEV_NAME, __func__, reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}

int s2mpu08_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpu08_dev *s2mpu08 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&s2mpu08->i2c_lock);
	ret = i2c_smbus_read_i2c_block_data(i2c, reg, count, buf);
	mutex_unlock(&s2mpu08->i2c_lock);
	if (ret < 0)
		return ret;

	return 0;
}

int s2mpu08_read_word(struct i2c_client *i2c, u8 reg)
{
	struct s2mpu08_dev *s2mpu08 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&s2mpu08->i2c_lock);
	ret = i2c_smbus_read_word_data(i2c, reg);
	mutex_unlock(&s2mpu08->i2c_lock);
	if (ret < 0)
		return ret;

	return ret;
}

int s2mpu08_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct s2mpu08_dev *s2mpu08 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&s2mpu08->i2c_lock);
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	mutex_unlock(&s2mpu08->i2c_lock);
	if (ret < 0)
		pr_info("%s:%s reg(0x%x), ret(%d)\n",
				MFD_DEV_NAME, __func__, reg, ret);

	return ret;
}

int s2mpu08_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2mpu08_dev *s2mpu08 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&s2mpu08->i2c_lock);
	ret = i2c_smbus_write_i2c_block_data(i2c, reg, count, buf);
	mutex_unlock(&s2mpu08->i2c_lock);
	if (ret < 0)
		return ret;

	return 0;
}

int s2mpu08_write_word(struct i2c_client *i2c, u8 reg, u16 value)
{
	struct s2mpu08_dev *s2mpu08 = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&s2mpu08->i2c_lock);
	ret = i2c_smbus_write_word_data(i2c, reg, value);
	mutex_unlock(&s2mpu08->i2c_lock);
	if (ret < 0)
		return ret;
	return 0;
}

int s2mpu08_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct s2mpu08_dev *s2mpu08 = i2c_get_clientdata(i2c);
	int ret;
	u8 old_val, new_val;

	mutex_lock(&s2mpu08->i2c_lock);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret >= 0) {
		old_val = ret & 0xff;
		new_val = (val & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
	}
	mutex_unlock(&s2mpu08->i2c_lock);
	return ret;
}

#endif

EXPORT_SYMBOL_GPL(s2mpu08_read_reg);
EXPORT_SYMBOL_GPL(s2mpu08_write_reg);

#if defined(CONFIG_OF)
static int of_s2mpu08_dt(struct device *dev,
			 struct s2mpu08_platform_data *pdata,
			 struct s2mpu08_dev *s2mpu08)
{
	struct device_node *np = dev->of_node;
//	struct device_node *codec_np;
//	struct i2c_client *i2c_codec;
	int strlen;
	const char *status;
//	u32 val;

	if (!np)
		return -EINVAL;

	acpm_mfd_node = np;

	/*
	 * of_get_named_gpio() and <linux/of_gpio.h> were removed upstream.
	 * pdata->irq_gpio was only legacy bookkeeping (never used to obtain
	 * the interrupt, which comes from the I2C client - i2c->irq), so it
	 * is left at its devm_kzalloc() default of 0 on mainline.
	 */

	status = of_get_property(np, "s2mpu08,wakeup", &strlen);
	if (status == NULL)
		return -EINVAL;
	if (strlen > 0) {
		if (!strcmp(status, "enabled") || !strcmp(status, "okay"))
			pdata->wakeup = true;
		else
			pdata->wakeup = false;
	}

#if 0
	codec_np = of_parse_phandle(np, "samsung,codec-interrupt", 0);
	if(codec_np) {
		i2c_codec = of_find_i2c_device_by_node(codec_np);
		s2mpu08->codec = i2c_codec;
		pr_err("[DEBUG] %s: codec node exist, name: %s\n", __func__, i2c_codec->name);
	}

	if (of_get_property(np, "i2c-speedy-address", NULL))
		pdata->use_i2c_speedy = true;

	pr_info("%s: irq-gpio: %u \n", __func__, pdata->irq_gpio);

	/* WTSR, SMPL */
	pdata->wtsr_smpl = devm_kzalloc(dev, sizeof(*pdata->wtsr_smpl),
			GFP_KERNEL);
	if (!pdata->wtsr_smpl)
		return -ENOMEM;

	status = of_get_property(np, "wtsr_en", &strlen);
	if (status == NULL)
		return -EINVAL;
	if (strlen > 0) {
		if (!strcmp(status, "enabled") || !strcmp(status, "okay"))
			pdata->wtsr_smpl->wtsr_en = true;
		else
			pdata->wtsr_smpl->wtsr_en = false;
	}

	status = of_get_property(np, "smpl_en", &strlen);
	if (status == NULL)
		return -EINVAL;
	if (strlen > 0) {
		if (!strcmp(status, "enabled") || !strcmp(status, "okay"))
			pdata->wtsr_smpl->smpl_en = true;
		else
			pdata->wtsr_smpl->smpl_en = false;
	}

	ret = of_property_read_u32(np, "wtsr_timer_val",
			&pdata->wtsr_smpl->wtsr_timer_val);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "smpl_timer_val",
			&pdata->wtsr_smpl->smpl_timer_val);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "check_jigon", &val);
	if (ret)
		return -EINVAL;
	pdata->wtsr_smpl->check_jigon = !!val;

	/* init time */
	pdata->init_time = devm_kzalloc(dev, sizeof(*pdata->init_time),
			GFP_KERNEL);
	if (!pdata->init_time)
		return -ENOMEM;

//	ret = of_property_read_u32(np, "init_time,msec",
//			&pdata->init_time->tm_msec);
//	if (ret)
//		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,sec",
			&pdata->init_time->tm_sec);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,min",
			&pdata->init_time->tm_min);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,hour",
			&pdata->init_time->tm_hour);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,mday",
			&pdata->init_time->tm_mday);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,mon",
			&pdata->init_time->tm_mon);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,year",
			&pdata->init_time->tm_year);
	if (ret)
		return -EINVAL;

	ret = of_property_read_u32(np, "init_time,wday",
			&pdata->init_time->tm_wday);
	if (ret)
		return -EINVAL;

	/* rtc optimize */
	ret = of_property_read_u32(np, "osc-bias-up", &val);
	if (!ret)
		pdata->osc_bias_up = val;
	else
		pdata->osc_bias_up = -1;

	ret = of_property_read_u32(np, "rtc_cap_sel", &val);
	if (!ret)
		pdata->cap_sel = val;
	else
		pdata->cap_sel = -1;

	ret = of_property_read_u32(np, "rtc_osc_xin", &val);
	if (!ret)
		pdata->osc_xin = val;
	else
		pdata->osc_xin = -1;

	ret = of_property_read_u32(np, "rtc_osc_xout", &val);
	if (!ret)
		pdata->osc_xout = val;
	else
		pdata->osc_xout = -1;
#endif /* RTC DISABLE */
	return 0;
}
#else
static int of_s2mpu08_dt(struct device *dev,
			 struct s2mpu08_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

/*
static const struct acpm_ops s2mpu08_acpm_ops = {
	{
		.read_reg = s2mpu08_read_reg,
		.bulk_read = s2mpu08_bulk_read,
		.write_reg = s2mpu08_write_reg,
		.bulk_write = s2mpu08_bulk_write,
		.update_reg = s2mpu08_update_reg,
	},
};
*/

/*
 * The S2MPU08 is reached over ACPM, not a real I2C/SPEEDY bus. The
 * s2mpu08_{read,write,update}_reg() helpers only use the i2c_client as a
 * carrier for the sub-block address (i2c->addr) and a back-pointer to the
 * s2mpu08_dev (i2c_get_clientdata()). Allocate a bare client - never
 * registered on any bus - to play that role for each sub-block.
 */
static struct i2c_client *s2mpu08_reg_handle(struct device *dev,
					     struct s2mpu08_dev *s2mpu08,
					     u8 addr)
{
	struct i2c_client *c;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return NULL;

	c->addr = addr;
	c->dev.parent = dev;
	i2c_set_clientdata(c, s2mpu08);

	return c;
}

static int s2mpu08_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpu08_dev *s2mpu08;
	struct s2mpu08_platform_data *pdata;
	struct acpm_handle *acpm;
	struct i2c_client *top;
	u8 reg_data;
	int ret;

	s2mpu08 = devm_kzalloc(dev, sizeof(*s2mpu08), GFP_KERNEL);
	if (!s2mpu08)
		return -ENOMEM;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	ret = of_s2mpu08_dt(dev, pdata, s2mpu08);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to parse device tree\n");

	/* The ACPM handle comes from the parent &acpm_ipc node. */
	acpm = devm_acpm_get_by_node(dev, dev->of_node->parent);
	if (IS_ERR(acpm))
		return dev_err_probe(dev, PTR_ERR(acpm),
				     "failed to get ACPM handle\n");

	s2mpu08->dev = dev;
	s2mpu08->device_type = S2MPU08X;
	s2mpu08->acpm = acpm;
	s2mpu08->acpm_channel_id = 2;
	s2mpu08->speedy_channel = 0;
	s2mpu08->pdata = pdata;
	s2mpu08->wakeup = pdata->wakeup;
	s2mpu08->irq_gpio = pdata->irq_gpio;

	pdata->irq_base = devm_irq_alloc_descs(dev, -1, 0, S2MPU08_IRQ_NR, -1);
	if (pdata->irq_base < 0)
		return dev_err_probe(dev, pdata->irq_base,
				     "irq_alloc_descs() failed\n");
	s2mpu08->irq_base = pdata->irq_base;

	mutex_init(&s2mpu08->i2c_lock);
	platform_set_drvdata(pdev, s2mpu08);

	/* Per sub-block register-access handles (address selects the block). */
	s2mpu08->pmic = s2mpu08_reg_handle(dev, s2mpu08, I2C_ADDR_PMIC);
	s2mpu08->rtc = s2mpu08_reg_handle(dev, s2mpu08, I2C_ADDR_RTC);
	s2mpu08->close = s2mpu08_reg_handle(dev, s2mpu08, I2C_ADDR_CLOSE);
	s2mpu08->codeca = s2mpu08_reg_handle(dev, s2mpu08, I2C_ADDR_CODECA);
	if (!s2mpu08->pmic || !s2mpu08->rtc || !s2mpu08->close ||
	    !s2mpu08->codeca)
		return -ENOMEM;
	s2mpu08->i2c = s2mpu08->pmic;

	/*
	 * The chip ID is register 0x00 of the TOP block. Register 0x00 of
	 * the PMIC block is INT1 (clear-on-read), which reads back 0 and
	 * would select the revision-0 LDO35/36 workarounds.
	 */
	top = s2mpu08_reg_handle(dev, s2mpu08, I2C_ADDR_TOP);
	if (!top)
		return -ENOMEM;
	ret = s2mpu08_read_reg(top, S2MPU08_PMIC_REG_PMICID, &reg_data);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "failed to read PMIC ID over ACPM\n");
	s2mpu08->pmic_rev = reg_data;
	dev_info(dev, "S2MPU08 PMIC revision 0x%02x\n", s2mpu08->pmic_rev);

	s2mpu08->irq = platform_get_irq_optional(pdev, 0);
	if (s2mpu08->irq > 0) {
		ret = s2mpu08_irq_init(s2mpu08);
		if (ret < 0)
			dev_warn(dev,
				 "IRQ init failed (%d); continuing without PMIC IRQ\n",
				 ret);
	} else {
		dev_warn(dev, "no PMIC IRQ; continuing without it\n");
	}

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE, s2mpu08_devs,
				   ARRAY_SIZE(s2mpu08_devs), NULL, 0, NULL);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to add MFD devices\n");

	device_init_wakeup(dev, pdata->wakeup);

	return 0;
}

static void s2mpu08_remove(struct platform_device *pdev)
{
	/* All resources are devm-managed. */
}

static const struct of_device_id s2mpu08_of_match[] = {
	{ .compatible = "samsung,s2mpu08" },
	{ },
};
MODULE_DEVICE_TABLE(of, s2mpu08_of_match);

#ifdef CONFIG_PM
static int s2mpu08_suspend(struct device *dev)
{
	struct s2mpu08_dev *s2mpu08 = dev_get_drvdata(dev);

	if (s2mpu08->irq > 0) {
		if (device_may_wakeup(dev))
			enable_irq_wake(s2mpu08->irq);
		disable_irq(s2mpu08->irq);
	}

	return 0;
}

static int s2mpu08_resume(struct device *dev)
{
	struct s2mpu08_dev *s2mpu08 = dev_get_drvdata(dev);

	if (s2mpu08->irq > 0) {
		if (device_may_wakeup(dev))
			disable_irq_wake(s2mpu08->irq);
		enable_irq(s2mpu08->irq);
	}

	return 0;
}
#else
#define s2mpu08_suspend	NULL
#define s2mpu08_resume	NULL
#endif /* CONFIG_PM */

static const struct dev_pm_ops s2mpu08_pm = {
	LATE_SYSTEM_SLEEP_PM_OPS(s2mpu08_suspend, s2mpu08_resume)
};

static struct platform_driver s2mpu08_driver = {
	.driver = {
		.name = "s2mpu08",
		.pm = &s2mpu08_pm,
		.of_match_table = s2mpu08_of_match,
	},
	.probe = s2mpu08_probe,
	.remove = s2mpu08_remove,
};
module_platform_driver(s2mpu08_driver);

MODULE_DESCRIPTION("s2mpu08 multi-function core driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
