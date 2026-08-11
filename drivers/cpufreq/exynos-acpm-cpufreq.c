// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos ACPM CPU frequency scaling driver
 *
 * CPU frequency and voltage are controlled by the ACPM (Always-on Co-Processor
 * Management) co-processor via mailbox IPC. This driver reads OPP tables from
 * the device tree and delegates rate changes to ACPM, which adjusts both the
 * PLL dividers and the PMIC BUCK voltages atomically.
 *
 * Copyright (c) 2025 Mikhael Skvortsov
 */

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>

struct exynos_acpm_cpufreq_policy {
	u32 dvfs_id;
	cpumask_var_t cpus;
	struct device *cpu_dev;
	struct cpufreq_frequency_table *freq_table;
};

static struct acpm_handle *acpm_handle;
static u32 acpm_dvfs_chan;

static int exynos_acpm_cpufreq_init(struct cpufreq_policy *policy)
{
	struct exynos_acpm_cpufreq_policy *data;
	struct device *cpu_dev;
	int ret;

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev)
		return -ENODEV;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (!zalloc_cpumask_var(&data->cpus, GFP_KERNEL)) {
		ret = -ENOMEM;
		goto err_free;
	}

	data->cpu_dev = cpu_dev;
	cpumask_set_cpu(policy->cpu, data->cpus);

	ret = of_property_read_u32(cpu_dev->of_node, "samsung,acpm-dvfs-id",
				   &data->dvfs_id);
	if (ret) {
		dev_err(cpu_dev, "missing samsung,acpm-dvfs-id: %d\n", ret);
		goto err_cpumask;
	}

	ret = dev_pm_opp_of_get_sharing_cpus(cpu_dev, data->cpus);
	if (ret) {
		dev_err(cpu_dev, "failed to get OPP sharing CPUs: %d\n", ret);
		goto err_cpumask;
	}

	ret = dev_pm_opp_of_cpumask_add_table(data->cpus);
	if (ret) {
		dev_err(cpu_dev, "failed to add OPP table: %d\n", ret);
		goto err_cpumask;
	}

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &data->freq_table);
	if (ret) {
		dev_err(cpu_dev, "failed to init cpufreq table: %d\n", ret);
		goto err_remove_table;
	}

	cpumask_copy(policy->cpus, data->cpus);
	policy->driver_data = data;
	policy->freq_table = data->freq_table;
	policy->cpuinfo.transition_latency = CPUFREQ_DEFAULT_TRANSITION_LATENCY_NS;
	policy->dvfs_possible_from_any_cpu = true;

	return 0;

err_remove_table:
	dev_pm_opp_of_cpumask_remove_table(data->cpus);
err_cpumask:
	free_cpumask_var(data->cpus);
err_free:
	kfree(data);
	return ret;
}

static void exynos_acpm_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct exynos_acpm_cpufreq_policy *data = policy->driver_data;

	dev_pm_opp_free_cpufreq_table(data->cpu_dev, &data->freq_table);
	dev_pm_opp_of_cpumask_remove_table(data->cpus);
	free_cpumask_var(data->cpus);
	kfree(data);
}

static int exynos_acpm_cpufreq_target_index(struct cpufreq_policy *policy,
					    unsigned int index)
{
	struct exynos_acpm_cpufreq_policy *data = policy->driver_data;
	unsigned long freq_hz = (unsigned long)policy->freq_table[index].frequency * 1000;

	return acpm_handle->ops->dvfs.set_rate(acpm_handle, acpm_dvfs_chan,
					       data->dvfs_id, freq_hz);
}

static struct cpufreq_driver exynos_acpm_cpufreq_driver = {
	.name		= "exynos-acpm",
	.flags		= CPUFREQ_IS_COOLING_DEV,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= exynos_acpm_cpufreq_target_index,
	.init		= exynos_acpm_cpufreq_init,
	.exit		= exynos_acpm_cpufreq_exit,
	.register_em	= cpufreq_register_em_with_opp,
	.suspend	= cpufreq_generic_suspend,
};

static int exynos_acpm_cpufreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct acpm_handle *acpm;
	u32 chan;
	int ret;

	acpm = devm_acpm_get_by_node(dev, dev->parent->of_node);
	if (IS_ERR(acpm))
		return dev_err_probe(dev, PTR_ERR(acpm),
				     "failed to get ACPM handle\n");

	if (of_property_read_u32(dev->of_node, "samsung,acpm-dvfs-channel", &chan))
		chan = 1;

	acpm_handle = acpm;
	acpm_dvfs_chan = chan;

	ret = cpufreq_register_driver(&exynos_acpm_cpufreq_driver);
	if (ret) {
		acpm_handle = NULL;
		return dev_err_probe(dev, ret,
				     "failed to register cpufreq driver\n");
	}

	return 0;
}

static void exynos_acpm_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&exynos_acpm_cpufreq_driver);
	acpm_handle = NULL;
}

static const struct of_device_id exynos_acpm_cpufreq_of_match[] = {
	{ .compatible = "samsung,exynos-acpm-cpufreq" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_acpm_cpufreq_of_match);

static struct platform_driver exynos_acpm_cpufreq_platdrv = {
	.driver = {
		.name		= "exynos-acpm-cpufreq",
		.of_match_table	= exynos_acpm_cpufreq_of_match,
	},
	.probe	= exynos_acpm_cpufreq_probe,
	.remove	= exynos_acpm_cpufreq_remove,
};
module_platform_driver(exynos_acpm_cpufreq_platdrv);

MODULE_DESCRIPTION("Samsung Exynos ACPM CPU frequency scaling driver");
MODULE_LICENSE("GPL");
