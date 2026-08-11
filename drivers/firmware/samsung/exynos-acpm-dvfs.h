/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2020 Samsung Electronics Co., Ltd.
 * Copyright 2020 Google LLC.
 * Copyright 2025 Linaro Ltd.
 */
#ifndef __EXYNOS_ACPM_DVFS_H__
#define __EXYNOS_ACPM_DVFS_H__

#include <linux/types.h>

struct acpm_handle;

int acpm_dvfs_set_init_freq(struct acpm_handle *handle,
			    unsigned int acpm_chan_id, unsigned int clk_id,
			    unsigned long rate);
int acpm_dvfs_set_rate(struct acpm_handle *handle,
		       unsigned int acpm_chan_id, unsigned int id,
		       unsigned long rate);
unsigned long acpm_dvfs_get_rate(struct acpm_handle *handle,
				 unsigned int acpm_chan_id,
				 unsigned int clk_id);
int acpm_dvfs_set_flag(struct acpm_handle *handle, unsigned int acpm_chan_id,
		       unsigned int id, unsigned int flag);

#endif /* __EXYNOS_ACPM_DVFS_H__ */
