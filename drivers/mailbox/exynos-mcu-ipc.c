// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos MCU_IPC mailbox: the AP side of the block the AP shares
 * with a co-processor such as the Shannon modem (CP).
 *
 * The block has 16 doorbells in each direction and 64 shared registers
 * (ISSR0..63) that neither side's writes signal on their own. Downstream
 * (drivers/misc/mcu_ipc/) wraps all of it in an ad-hoc exported API; here
 * the two halves map onto existing frameworks instead:
 *
 *  - doorbells are mailbox channels, #mbox-cells = <1> selecting the
 *    doorbell number. send_data() rings AP->CP doorbell n (INTGR1 bit n)
 *    and ignores its payload; a CP->AP doorbell n (INTSR0 bit n + 16)
 *    calls the channel's rx_callback with a NULL message.
 *  - the shared registers are a syscon regmap registered for this node,
 *    so clients look them up with syscon_regmap_lookup_by_phandle() and
 *    get regmap's locking for read-modify-write of the status words.
 *
 * Doorbells carry no data and need no acknowledgement, so the controller
 * has no TX-done mechanism: clients set knows_txdone.
 *
 * With a "regulator" child node, the block also provides the vote that
 * keeps the external regulator shared by the CP, GNSS and WLBT cores up
 * (downstream exynos_pmu_shared_reg_enable()/_disable()): the CP_WAKEUP
 * bit in AP2CP status (ISSR2 bit 0) plus the PMU's
 * EXT_REGULATOR_SHARED_OPTION. As a regulator its users get the
 * reference count downstream keeps in shared_reg_usage_cnt.
 */

#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox/exynos-mcu-ipc.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define MCU_IPC_INTCR0		0x0c	/* clear CP->AP pending, bits 31:16 */
#define MCU_IPC_INTMR0		0x10	/* mask CP->AP, bits 31:16 */
#define MCU_IPC_INTSR0		0x14	/* CP->AP status, bits 31:16 */
#define MCU_IPC_INTGR1		0x1c	/* raise AP->CP, bits 15:0 */

#define MCU_IPC_AP2CP_STATUS	EXYNOS_MCU_IPC_ISSR(2)
#define MCU_IPC_CP_WAKEUP	BIT(0)

/* PMU_ALIVE */
#define EXT_REGULATOR_SHARED_OPTION	0x3648
#define EXT_REGULATOR_SHARED_ENABLE	BIT(2)

#define MCU_IPC_NUM_CHANS	16
#define MCU_IPC_RX_SHIFT	16
#define MCU_IPC_RX_ALL		GENMASK(31, 16)

struct exynos_mcu_ipc {
	struct device *dev;
	void __iomem *base;
	struct regmap *regmap;
	struct regmap *pmu;
	struct mbox_controller mbox;
	struct mbox_chan chans[MCU_IPC_NUM_CHANS];
};

static unsigned int mcu_ipc_chan_id(struct mbox_chan *chan)
{
	struct exynos_mcu_ipc *ipc = container_of(chan->mbox,
						  struct exynos_mcu_ipc, mbox);

	return chan - ipc->chans;
}

static int exynos_mcu_ipc_send_data(struct mbox_chan *chan, void *data)
{
	struct exynos_mcu_ipc *ipc = container_of(chan->mbox,
						  struct exynos_mcu_ipc, mbox);

	return regmap_write(ipc->regmap, MCU_IPC_INTGR1,
			    BIT(mcu_ipc_chan_id(chan)));
}

static int exynos_mcu_ipc_startup(struct mbox_chan *chan)
{
	struct exynos_mcu_ipc *ipc = container_of(chan->mbox,
						  struct exynos_mcu_ipc, mbox);
	u32 bit = BIT(mcu_ipc_chan_id(chan) + MCU_IPC_RX_SHIFT);

	/* drop whatever was pending before anyone listened, then unmask */
	regmap_write(ipc->regmap, MCU_IPC_INTCR0, bit);

	return regmap_clear_bits(ipc->regmap, MCU_IPC_INTMR0, bit);
}

static void exynos_mcu_ipc_shutdown(struct mbox_chan *chan)
{
	struct exynos_mcu_ipc *ipc = container_of(chan->mbox,
						  struct exynos_mcu_ipc, mbox);

	regmap_set_bits(ipc->regmap, MCU_IPC_INTMR0,
			BIT(mcu_ipc_chan_id(chan) + MCU_IPC_RX_SHIFT));
}

static const struct mbox_chan_ops exynos_mcu_ipc_ops = {
	.send_data = exynos_mcu_ipc_send_data,
	.startup = exynos_mcu_ipc_startup,
	.shutdown = exynos_mcu_ipc_shutdown,
};

static irqreturn_t exynos_mcu_ipc_irq(int irq, void *data)
{
	struct exynos_mcu_ipc *ipc = data;
	unsigned long pending;
	u32 stat, mask;
	unsigned int n;

	regmap_read(ipc->regmap, MCU_IPC_INTSR0, &stat);
	regmap_read(ipc->regmap, MCU_IPC_INTMR0, &mask);
	stat &= MCU_IPC_RX_ALL & ~mask;
	if (!stat)
		return IRQ_NONE;

	regmap_write(ipc->regmap, MCU_IPC_INTCR0, stat);

	pending = stat >> MCU_IPC_RX_SHIFT;
	for_each_set_bit(n, &pending, MCU_IPC_NUM_CHANS)
		mbox_chan_received_data(&ipc->chans[n], NULL);

	return IRQ_HANDLED;
}

static struct mbox_chan *exynos_mcu_ipc_xlate(struct mbox_controller *mbox,
					      const struct of_phandle_args *sp)
{
	if (sp->args_count != 1 || sp->args[0] >= MCU_IPC_NUM_CHANS)
		return ERR_PTR(-EINVAL);

	return &mbox->chans[sp->args[0]];
}

static int exynos_mcu_ipc_shared_enable(struct regulator_dev *rdev)
{
	struct exynos_mcu_ipc *ipc = rdev_get_drvdata(rdev);

	regmap_set_bits(ipc->regmap, MCU_IPC_AP2CP_STATUS, MCU_IPC_CP_WAKEUP);

	return regmap_set_bits(ipc->pmu, EXT_REGULATOR_SHARED_OPTION,
			       EXT_REGULATOR_SHARED_ENABLE);
}

static int exynos_mcu_ipc_shared_disable(struct regulator_dev *rdev)
{
	struct exynos_mcu_ipc *ipc = rdev_get_drvdata(rdev);

	regmap_clear_bits(ipc->pmu, EXT_REGULATOR_SHARED_OPTION,
			  EXT_REGULATOR_SHARED_ENABLE);

	return regmap_clear_bits(ipc->regmap, MCU_IPC_AP2CP_STATUS,
				 MCU_IPC_CP_WAKEUP);
}

static int exynos_mcu_ipc_shared_is_enabled(struct regulator_dev *rdev)
{
	struct exynos_mcu_ipc *ipc = rdev_get_drvdata(rdev);
	u32 val;
	int ret;

	ret = regmap_read(ipc->pmu, EXT_REGULATOR_SHARED_OPTION, &val);
	if (ret)
		return ret;

	return !!(val & EXT_REGULATOR_SHARED_ENABLE);
}

static const struct regulator_ops exynos_mcu_ipc_shared_ops = {
	.enable = exynos_mcu_ipc_shared_enable,
	.disable = exynos_mcu_ipc_shared_disable,
	.is_enabled = exynos_mcu_ipc_shared_is_enabled,
};

static const struct regulator_desc exynos_mcu_ipc_shared_desc = {
	.name = "shared",
	.ops = &exynos_mcu_ipc_shared_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
};

static int exynos_mcu_ipc_register_shared(struct exynos_mcu_ipc *ipc)
{
	struct device *dev = ipc->dev;
	struct regulator_config cfg = { };
	struct regulator_dev *rdev;
	struct device_node *np;

	np = of_get_child_by_name(dev->of_node, "regulator");
	if (!np)
		return 0;

	ipc->pmu = syscon_regmap_lookup_by_phandle(dev->of_node,
						   "samsung,pmu-syscon");
	if (IS_ERR(ipc->pmu)) {
		of_node_put(np);
		return dev_err_probe(dev, PTR_ERR(ipc->pmu),
				     "regulator needs samsung,pmu-syscon\n");
	}

	cfg.dev = dev;
	cfg.driver_data = ipc;
	cfg.of_node = np;
	cfg.init_data = of_get_regulator_init_data(dev, np,
						   &exynos_mcu_ipc_shared_desc);

	rdev = devm_regulator_register(dev, &exynos_mcu_ipc_shared_desc, &cfg);
	of_node_put(np);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev),
				     "failed to register the shared regulator\n");

	return 0;
}

static const struct regmap_config exynos_mcu_ipc_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = EXYNOS_MCU_IPC_ISSR(EXYNOS_MCU_IPC_NUM_ISSR - 1),
	.fast_io = true,
};

static int exynos_mcu_ipc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_mcu_ipc *ipc;
	int irq, ret;

	ipc = devm_kzalloc(dev, sizeof(*ipc), GFP_KERNEL);
	if (!ipc)
		return -ENOMEM;

	ipc->dev = dev;

	ipc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ipc->base))
		return PTR_ERR(ipc->base);

	ipc->regmap = devm_regmap_init_mmio(dev, ipc->base,
					    &exynos_mcu_ipc_regmap_config);
	if (IS_ERR(ipc->regmap))
		return dev_err_probe(dev, PTR_ERR(ipc->regmap),
				     "failed to create regmap\n");

	/* first: the PMU lookup can defer, the syscon below cannot be undone */
	ret = exynos_mcu_ipc_register_shared(ipc);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* no listeners yet: mask and clear every CP->AP doorbell */
	regmap_set_bits(ipc->regmap, MCU_IPC_INTMR0, MCU_IPC_RX_ALL);
	regmap_write(ipc->regmap, MCU_IPC_INTCR0, MCU_IPC_RX_ALL);

	ret = devm_request_irq(dev, irq, exynos_mcu_ipc_irq, 0,
			       dev_name(dev), ipc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	ipc->mbox.dev = dev;
	ipc->mbox.ops = &exynos_mcu_ipc_ops;
	ipc->mbox.chans = ipc->chans;
	ipc->mbox.num_chans = MCU_IPC_NUM_CHANS;
	ipc->mbox.of_xlate = exynos_mcu_ipc_xlate;

	ret = devm_mbox_controller_register(dev, &ipc->mbox);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register mailbox\n");

	/*
	 * syscon keeps this regmap for good -- there is no way to take it
	 * back -- which is why the driver cannot be unbound or unloaded.
	 */
	ret = of_syscon_register_regmap(dev->of_node, ipc->regmap);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register syscon\n");

	platform_set_drvdata(pdev, ipc);

	return 0;
}

static const struct of_device_id exynos_mcu_ipc_of_match[] = {
	{ .compatible = "samsung,exynos7885-mcu-ipc" },
	{ }
};

static struct platform_driver exynos_mcu_ipc_driver = {
	.probe = exynos_mcu_ipc_probe,
	.driver = {
		.name = "exynos-mcu-ipc",
		.of_match_table = exynos_mcu_ipc_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(exynos_mcu_ipc_driver);
