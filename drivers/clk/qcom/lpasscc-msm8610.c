// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8610 LPASS clock controller
 *
 * Only the clocks of the digital audio codec are described: the rest of the
 * LPASS clocks are managed by the ADSP firmware.
 */

#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/qcom,msm8610-lpasscc.h>

#include "clk-branch.h"
#include "clk-rcg.h"
#include "clk-regmap.h"
#include "common.h"

enum {
	P_XO,
};

static const struct parent_map lpass_cc_xo_map[] = {
	{ P_XO, 0 },
};

static const struct clk_parent_data lpass_cc_xo_data[] = {
	{ .fw_name = "xo" },
};

static const struct freq_tbl ftbl_digcodec_clk_src[] = {
	F(9600000, P_XO, 2, 0, 0),
	F(19200000, P_XO, 1, 0, 0),
	{ }
};

static struct clk_rcg2 digcodec_clk_src = {
	.cmd_rcgr = 0x2c000,
	.mnd_width = 8,
	.hid_width = 5,
	.parent_map = lpass_cc_xo_map,
	.freq_tbl = ftbl_digcodec_clk_src,
	.clkr.hw.init = &(struct clk_init_data) {
		.name = "digcodec_clk_src",
		.parent_data = lpass_cc_xo_data,
		.num_parents = ARRAY_SIZE(lpass_cc_xo_data),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_branch digcodec_clk = {
	.halt_reg = 0x2c014,
	.clkr = {
		.enable_reg = 0x2c014,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "digcodec_clk",
			.parent_hws = (const struct clk_hw *[]) {
				&digcodec_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch digcodec_ahb_clk = {
	.halt_reg = 0x2c018,
	.clkr = {
		.enable_reg = 0x2c018,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data) {
			.name = "digcodec_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_regmap *lpass_cc_msm8610_clocks[] = {
	[DIGCODEC_CLK_SRC] = &digcodec_clk_src.clkr,
	[DIGCODEC_CLK] = &digcodec_clk.clkr,
	[DIGCODEC_AHB_CLK] = &digcodec_ahb_clk.clkr,
};

static const struct regmap_config lpass_cc_msm8610_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0x3fffc,
	.fast_io = true,
};

static const struct qcom_cc_desc lpass_cc_msm8610_desc = {
	.config = &lpass_cc_msm8610_regmap_config,
	.clks = lpass_cc_msm8610_clocks,
	.num_clks = ARRAY_SIZE(lpass_cc_msm8610_clocks),
};

static int lpass_cc_msm8610_probe(struct platform_device *pdev)
{
	return qcom_cc_probe(pdev, &lpass_cc_msm8610_desc);
}

static const struct of_device_id lpass_cc_msm8610_match_table[] = {
	{ .compatible = "qcom,msm8610-lpasscc" },
	{ }
};
MODULE_DEVICE_TABLE(of, lpass_cc_msm8610_match_table);

static struct platform_driver lpass_cc_msm8610_driver = {
	.probe = lpass_cc_msm8610_probe,
	.driver = {
		.name = "msm8610-lpasscc",
		.of_match_table = lpass_cc_msm8610_match_table,
	},
};
module_platform_driver(lpass_cc_msm8610_driver);

MODULE_DESCRIPTION("Qualcomm MSM8610 LPASS Clock Controller Driver");
MODULE_LICENSE("GPL");
