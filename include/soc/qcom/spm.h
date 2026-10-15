/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 * Copyright (c) 2014,2015, Linaro Ltd.
 */

#ifndef __SPM_H__
#define __SPM_H__

enum pm_sleep_mode {
	PM_SLEEP_MODE_STBY,
	PM_SLEEP_MODE_RET,
	PM_SLEEP_MODE_SPC,
	PM_SLEEP_MODE_PC,
	PM_SLEEP_MODE_NR,
};

struct spm_driver_data;

#if IS_BUILTIN(CONFIG_QCOM_SPM)
void spm_set_low_power_mode(struct spm_driver_data *drv,
			    enum pm_sleep_mode mode);
struct spm_driver_data *spm_get_cpu_drv(unsigned int cpu);
#else
static inline void spm_set_low_power_mode(struct spm_driver_data *drv,
					  enum pm_sleep_mode mode)
{
}

static inline struct spm_driver_data *spm_get_cpu_drv(unsigned int cpu)
{
	return NULL;
}
#endif

#endif /* __SPM_H__ */
