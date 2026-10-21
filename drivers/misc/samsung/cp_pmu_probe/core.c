// SPDX-License-Identifier: GPL-2.0-only
/*
 * CP (Shannon SS310AP modem) PMU-only bring-up probe.
 *
 * Not a modem driver. No mailbox IPC channel, no firmware load, no SIPC.
 * It does now issue one SMC: CP_CTRL_NS/S are secure-only, so the power
 * sequence reaches them through the same firmware call downstream uses. The only thing this exists to test: whether
 * CENTRAL_SEQ_CP_STATUS (PMU_ALIVE 0x0284) is unreachable because
 * nothing on this kernel has ever issued the CP domain's cal_cp_init()
 * equivalent -- one half of the split hypothesis in gnss.md's
 * "Downstream sources for CP init and firmware load" entry, the half
 * that doesn't require touching firmware or EL3 at all.
 *
 * Sequences below are ported directly from the downstream cal-if table
 * this device's kernel actually compiles and runs --
 * drivers/soc/samsung/cal-if/exynos7885/pmucal_cp_exynos7885.h in
 * /w/linux-samsung-a30, referenced from modem_ctrl_ss310ap.c's
 * ss310ap_on()/ss310ap_reset(). Unlike the GNSS side (CONFIG_GNSS_PMUCAL=n
 * downstream, so pmucal_gnss_exynos7885.h is dead code there and had to
 * be cross-checked against a hand-written fallback path), CONFIG_CP_PMUCAL=y
 * on this defconfig: cp_init[]/cp_reset_assert[]/cp_reset_release[] below
 * are the live, compiled, actually-running downstream sequence, not a
 * fallback -- so this port has no separate "hand-written path" to
 * reconcile against.
 *
 * SAFETY: none of CP_CTRL_NS/S, CENTRAL_SEQ_CP_CONFIGURATION/STATUS, the
 * CP SYS_PWR_REG block, or CLEANY_BUS_CP_* have been individually
 * confirmed safe to read on this hardware. They were only ever touched
 * as part of a 183-offset batched sweep (gnss_kepler's pmu_alive_known_offsets[])
 * that hung the device -- see gnss.md, "Tried a brute-force PMU_ALIVE
 * register diff". Which specific offset caused that hang was never
 * isolated. So: `regs` below deliberately reads only registers already
 * confirmed safe by hundreds of prior reads elsewhere in this
 * investigation (CP_STAT, the EXT_REGULATOR_SHARED_* pair, the CP
 * mailbox ISSR2/3) -- it does NOT touch anything CP-domain-specific.
 * `probe_offset` and `central_seq_status` are separate, explicitly
 * opt-in single-register reads for exactly this reason: touch one thing,
 * check the device is still alive, then touch the next. `start`/`reset`/
 * `release` are the real experiment and are inherently first-contact
 * with these registers -- that's the point of this driver, not an
 * oversight, but treat each one as a live hardware test, not a routine
 * poke, and check liveness (a plain `ssh a40 true`, or read `regs`)
 * immediately after each.
 */

#include <linux/arm-smccc.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>

/*
 * PMU_ALIVE offsets, relative to the pmu_system_controller syscon base
 * (0x11c80000) -- same syscon gnss_kepler and the WLBT driver use.
 * CP's own register block, from pmucal_cp_exynos7885.h.
 */
/*
 * CP_CTRL_NS/S are secure-only on this SoC: a direct regmap access takes
 * a synchronous external abort and can take the board down. Downstream's
 * pmucal_is_cp_smc_regs() singles out exactly these two offsets and
 * routes them through a firmware call instead, which is what
 * cp_ctrl_smc_read()/_write() below reimplement.
 *
 * Convention, from pmucal_smc_read()/pmucal_smc_write():
 *   read:  a0 = exynos_smc(SMC_ID, READ_CTRL, half, reg)
 *          half 0 = low 16 bits, 1 = high 16; on success a0 is
 *          (value_half << 16) and the low 16 bits are 0, otherwise the
 *          low 16 bits are an error code.
 *   write: ret = exynos_smc(SMC_ID, WRITE_CTRL, value32, reg), ret > 0
 *          means failure.
 * `reg` is an enum index, NOT the PMU offset: CP_CTRL_S = 0, NS = 1.
 */
#define CP_SMC_ID			0x82000700
#define CP_SMC_READ_CTRL		0x3
#define CP_SMC_WRITE_CTRL		0x4
#define CP_SMC_REG_CTRL_S		0
#define CP_SMC_REG_CTRL_NS		1

/*
 * The same firmware call also carries CP secure boot, as sub-functions
 * 0..2 in the slot where READ_CTRL/WRITE_CTRL go. From downstream
 * modem_v1 (drivers/misc/modem_v1/link_device_shmem.c,
 * shmem_security_request()) -- note modem_if/ has a near-identical copy
 * but is not the built stack on this SoC; CONFIG_SEC_SIPC_MODEM_IF
 * selects modem_v1/, whose modemctl is ss310ap.
 *
 *   exynos_smc(SMC_ID_CLK, SSS_CLK_ENABLE, 0, 0);
 *   if (use_cp_memory_map && mode == 0)
 *           mode |= (u32)cp_phys_base;
 *   err = exynos_smc(SMC_ID, mode, param2, param3);
 *   exynos_smc(SMC_ID_CLK, SSS_CLK_DISABLE, 0, 0);
 *
 * with param2 = boot-loader size and param3 = main image size for
 * mode 0 (shm_get_security_param2()/_param3() in mcu_ipc/shm_ipc.c).
 * The Security Sub System clock has to be running across the call --
 * that bracket is the part this driver was missing when it first tried
 * a bare CP power-on and got 0x000b back.
 */
#define CP_SMC_CLK_ID			0xc2001011
#define SSS_CLK_ENABLE			0
#define SSS_CLK_DISABLE			1

#define CP_BOOT_MODE_NORMAL		0
#define CP_BOOT_MODE_DUMP		1
#define CP_BOOT_RE_INIT			2

/*
 * smc_err_string[] from the same file. Worth having in full: the return
 * value is the only channel EL3 gives us, and the difference between
 * "CP_NOT_WORKING" (a precondition complaint -- the CP simply is not up)
 * and e.g. "CP_NOT_VALID_MAGIC" or "CP_INVALID_CP_BASE" (EL3 read our
 * memory and disliked what it found) is exactly what this probe is for.
 */
static const char * const cp_smc_err_string[] = {
	"CP_NO_ERROR",			"CP_NOT_ALIGN_64KB",
	"CP_MEM_TOO_BIG",		"CP_FLAG_OUT_RANGE",
	"CP_WRONG_TZASC_REGION_NUM",	"CP_WRONG_BL_SIZE",
	"CP_MEM_OUT_OF_RANGE",		"CP_NOT_ALIGN_16B",
	"CP_MEM_IN_SECURE_DRAM",	"CP_ASP_ENABLE_FAIL",
	"CP_ASP_DISABLE_FAIL",		"CP_NOT_WORKING",
	"CP_ALREADY_WORKING",		"CP_ALREADY_DUMP_MODE",
	"CP_NOT_VALID_MAGIC",		"CP_SHOULD_BE_DISABLE",
	"CP_ALREADY_ENABLE_CPMEM_ON",	"CP_ALREADY_SET_WND",
	"CP_FAIL_TO_SET_WND",		"CP_INVALID_CP_BASE",
	"CP_CORRUPTED_CP_MEM_INFO",
};

static const char *cp_smc_err_name(unsigned long err)
{
	return err < ARRAY_SIZE(cp_smc_err_string) ? cp_smc_err_string[err]
						  : "UNKNOWN";
}

#define CP_CTRL_NS			0x0030
#define CP_CTRL_S			0x0034
#define CENTRAL_SEQ_CP_CONFIGURATION	0x0280
#define CENTRAL_SEQ_CP_STATUS		0x0284
#define RESET_SEQUENCER_STATUS		0x0504	/* shared with GNSS, different bitfield */
#define RESET_AHEAD_CP_SYS_PWR_REG	0x1320
#define CLEANY_BUS_CP_SYS_PWR_REG	0x1324
#define LOGIC_RESET_CP_SYS_PWR_REG	0x1328
#define TCXO_GATE_CP_SYS_PWR_REG	0x132c
#define CP_DISABLE_ISO_SYS_PWR_REG	0x1330
#define CP_RESET_ISO_SYS_PWR_REG	0x1334
#define CP_DUMP_PC_SYS_PWR_REG		0x1338
#define CLEANY_BUS_CP_CONFIGURATION	0x3920
#define CLEANY_BUS_CP_STATUS		0x3924

/* Already confirmed safe -- read constantly by gnss_kepler throughout
 * the investigation with no incident (e.g. "CP_STAT = 0x01" in gnss.md).
 */
#define CP_STAT				0x0038
#define EXT_REGULATOR_SHARED_STATUS	0x3644
#define EXT_REGULATOR_SHARED_OPTION	0x3648
#define EXT_REGULATOR_SHARED_ENABLE	BIT(2)

#define CP_PWRON			BIT(1)
#define CP_START			BIT(3)
#define SWEEPER_BYPASS_DATA_EN		BIT(16)

/*
 * Hypothesis, not documentation: downstream defines TCXO_ENABLE_SW as
 * BIT(11) of GNSS_CTRL_NS (pmu-gnss.h), and CP_CTRL_NS (0x0030) is the
 * parallel register to GNSS_CTRL_NS (0x0040). There is no CP-side
 * TCXO_ENABLE_SW anywhere in the downstream tree, so this is an analogy
 * from the register layout alone.
 *
 * Worth testing because CP_CTRL_NS reads 0x000046aa live -- bit 11
 * clear, exactly the state the GNSS TCXO bit was in -- and because both
 * cp_init[] and the GNSS sequences gate on TCXO_GATE_*_SYS_PWR_REG.
 * If holding this makes RESET_SEQUENCER_STATUS reach 5, one missing
 * clock explains both the CP never leaving reset and the GNSS RF dying
 * ~12 s into acquisition. Off by default; opt in with
 *     echo 1 > /sys/kernel/debug/cp_pmu_probe/tcxo_sw
 * before writing `start`, so the A/B stays explicit.
 */
#define CP_TCXO_ENABLE_SW		BIT(11)

/* CP's field within the shared RESET_SEQUENCER_STATUS register: bits[2:0],
 * target 0x5. GNSS's own field in the same register is bits[6:4] -- see
 * gnss_kepler's GNSS_RESET_SEQ_STAT_MASK/_DONE.
 */
#define CP_RESET_SEQ_STAT_MASK		GENMASK(2, 0)
#define CP_RESET_SEQ_STAT_DONE		(0x5 << 0)

#define CENTRAL_SEQ_CP_STATES_MASK	GENMASK(23, 16)
#define CENTRAL_SEQ_CP_STATES_DOWN	(0x80 << 16)
#define CENTRAL_SEQ_CP_STATES_RUN	(0x0 << 16)

#define CLEANY_BUS_CP_REQ		BIT(0)
#define CLEANY_BUS_CP_STATUS_MASK	GENMASK(17, 16)
#define CLEANY_BUS_CP_STATUS_DONE	(0x0 << 16)

/* Not this device's own resource -- same fixed SoC address gnss_kepler
 * already maps for the same reason (no modem_ctrl_ss310ap port on this
 * kernel to own it). Only used here for the shared-regulator handshake
 * below, same as gnss_pmu_release_reset() downstream and
 * gnss_kepler_shared_reg_enable() in this tree.
 */
#define CP_MBOX_PHYS_BASE	0x12080000
#define CP_MBOX_SIZE		0x180
#define CP_MBOX_ISSR2		0x88	/* MCU_IPC_ISSR0 + 4*2 */
#define CP_MBOX_ISSR3		0x8c
#define CP_WAKEUP_REQ		BIT(0)

struct cp_pmu_probe {
	struct device *dev;
	struct regmap *pmureg;
	void __iomem *cp_mbox;
	struct dentry *debugfs_dir;
	u32 last_probe_offset;
	u32 last_probe_value;
	int last_probe_ret;

	/* lazily-mapped window onto the CP reserved region, for cp_mem */
	void *cp_mem;

	/* lazily-mapped non-cacheable window onto the shared IPC region */
	void *ipc_mem;

	/* set CP_TCXO_ENABLE_SW in cp_power_on(); see its definition */
	bool tcxo_sw;

	/* last CP secure-boot firmware call, for read-back via sec_boot */
	bool sec_boot_done;
	unsigned long sec_boot_mode;
	unsigned long sec_boot_bl_size;
	unsigned long sec_boot_main_size;
	unsigned long sec_boot_base;
	unsigned long sec_boot_ret;
};

/*
 * exynos_pmu_shared_reg_enable()/_disable() (mcu_ipc.c downstream).
 * ss310ap_on() calls enable() before cal_cp_init() on a first boot (the
 * "CP first Init!" branch) -- same mechanism gnss_pmu_release_reset()
 * uses, and the same simplification gnss_kepler_shared_reg_enable()
 * already made: no refcounting (shared_reg_usage_cnt), since this probe
 * only ever has one caller. See gnss.md.
 */
static void cp_shared_reg_enable(struct cp_pmu_probe *cp)
{
	u32 val = readl(cp->cp_mbox + CP_MBOX_ISSR2);

	writel(val | CP_WAKEUP_REQ, cp->cp_mbox + CP_MBOX_ISSR2);
	regmap_update_bits(cp->pmureg, EXT_REGULATOR_SHARED_OPTION,
			    EXT_REGULATOR_SHARED_ENABLE,
			    EXT_REGULATOR_SHARED_ENABLE);

	regmap_read(cp->pmureg, EXT_REGULATOR_SHARED_STATUS, &val);
	dev_info(cp->dev, "shared_reg_enable: EXT_REGULATOR_SHARED_STATUS=0x%08x\n",
		 val);
}

static void cp_shared_reg_disable(struct cp_pmu_probe *cp)
{
	u32 val;

	regmap_update_bits(cp->pmureg, EXT_REGULATOR_SHARED_OPTION,
			    EXT_REGULATOR_SHARED_ENABLE, 0);

	val = readl(cp->cp_mbox + CP_MBOX_ISSR2);
	writel(val & ~CP_WAKEUP_REQ, cp->cp_mbox + CP_MBOX_ISSR2);
}

static int cp_wait_bits(struct cp_pmu_probe *cp, u32 offset, u32 mask,
			 u32 target, const char *what)
{
	u32 val = 0;
	int i;

	for (i = 0; i < 200; i++) {
		regmap_read(cp->pmureg, offset, &val);
		if ((val & mask) == target) {
			dev_info(cp->dev, "%s done after %d us (0x%08x)\n",
				 what, i * 50, val);
			return 0;
		}
		usleep_range(50, 60);
	}

	dev_err(cp->dev, "timeout waiting for %s (last read 0x%08x)\n",
		what, val);
	return -ETIMEDOUT;
}

static unsigned long cp_smc(unsigned long fn, unsigned long arg, unsigned long reg)
{
	struct arm_smccc_res res;

	arm_smccc_smc(CP_SMC_ID, fn, arg, reg, 0, 0, 0, 0, &res);

	return res.a0;
}

/* -> 0 on success, with *out set to the full 32-bit register value. */
static int cp_ctrl_smc_read(struct cp_pmu_probe *cp, unsigned long reg, u32 *out)
{
	unsigned long lo, hi;

	lo = cp_smc(CP_SMC_READ_CTRL, 0, reg);
	if (lo & 0xffff) {
		dev_err(cp->dev, "smc read reg%lu low half failed: 0x%04lx\n",
			reg, lo & 0xffff);
		return -EIO;
	}

	hi = cp_smc(CP_SMC_READ_CTRL, 1, reg);
	if (hi & 0xffff) {
		dev_err(cp->dev, "smc read reg%lu high half failed: 0x%04lx\n",
			reg, hi & 0xffff);
		return -EIO;
	}

	*out = (u32)(((hi >> 16) << 16) | (lo >> 16));

	return 0;
}

static int cp_ctrl_smc_write(struct cp_pmu_probe *cp, unsigned long reg, u32 val)
{
	unsigned long ret = cp_smc(CP_SMC_WRITE_CTRL, val, reg);

	if ((long)ret > 0) {
		dev_err(cp->dev, "smc write reg%lu = 0x%08x failed: %ld\n",
			reg, val, (long)ret);
		return -EIO;
	}

	return 0;
}

/* The secure-world stand-in for regmap_update_bits() on these two. */
static int cp_ctrl_smc_update_bits(struct cp_pmu_probe *cp, unsigned long reg,
				    u32 mask, u32 val)
{
	u32 old, new;
	int ret = cp_ctrl_smc_read(cp, reg, &old);

	if (ret)
		return ret;

	new = (old & ~mask) | (val & mask);
	ret = cp_ctrl_smc_write(cp, reg, new);
	dev_info(cp->dev, "smc reg%lu: 0x%08x -> 0x%08x (mask 0x%08x)%s\n",
		 reg, old, new, mask, ret ? " FAILED" : "");

	return ret;
}

/*
 * cp_init[] from pmucal_cp_exynos7885.h -- the live downstream sequence,
 * not dead code (CONFIG_CP_PMUCAL=y). First-ever deliberate write to
 * CP_CTRL_NS/S on this kernel: this and the wait below are the actual
 * experiment this driver exists to run.
 */
static int cp_power_on(struct cp_pmu_probe *cp)
{
	int ret;

	/*
	 * Downstream brackets cal_cp_init() with this at the *caller*
	 * (ss310ap_on()'s "CP first Init!" branch), not inside pmucal, so
	 * porting cp_init[] alone missed it -- cp_release_reset() below
	 * happened to get it because its downstream counterpart is bracketed
	 * too. Without it EL3 answers CP_CTRL accesses with 0x000b
	 * (CP_NOT_WORKING), which is exactly what we measured here while the
	 * same registers were readable on the downstream kernel.
	 */
	cp_shared_reg_enable(cp);

	if (cp->tcxo_sw) {
		dev_info(cp->dev, "power_on: CP_CTRL_NS__TCXO_ENABLE_SW\n");
		ret = cp_ctrl_smc_update_bits(cp, CP_SMC_REG_CTRL_NS,
					       CP_TCXO_ENABLE_SW,
					       CP_TCXO_ENABLE_SW);
		if (ret)
			goto done;
	}

	dev_info(cp->dev, "power_on: CP_CTRL_NS__CP_PWRON\n");
	ret = cp_ctrl_smc_update_bits(cp, CP_SMC_REG_CTRL_NS, CP_PWRON, CP_PWRON);
	if (ret)
		goto done;

	dev_info(cp->dev, "power_on: CP_CTRL_S__CP_START\n");
	ret = cp_ctrl_smc_update_bits(cp, CP_SMC_REG_CTRL_S, CP_START, CP_START);
	if (ret)
		goto done;

	ret = cp_wait_bits(cp, RESET_SEQUENCER_STATUS, CP_RESET_SEQ_STAT_MASK,
			    CP_RESET_SEQ_STAT_DONE, "RESET_SEQUENCER_STATUS (CP)");

done:
	cp_shared_reg_disable(cp);

	return ret;
}

/*
 * cp_reset_assert[] from pmucal_cp_exynos7885.h. Each SYS_PWR_REG write
 * below is a plain regmap_write of the sequence's target value rather
 * than a masked update -- same simplification gnss_kepler_hold_reset()
 * already makes for the equivalent GNSS registers (each is a small
 * dedicated register where the full-register write is the update).
 */
static int cp_hold_reset(struct cp_pmu_probe *cp)
{
	dev_info(cp->dev, "hold_reset\n");

	regmap_update_bits(cp->pmureg, CENTRAL_SEQ_CP_CONFIGURATION, BIT(16), 0);
	regmap_write(cp->pmureg, RESET_AHEAD_CP_SYS_PWR_REG, 0);
	regmap_write(cp->pmureg, CLEANY_BUS_CP_SYS_PWR_REG, 0);
	regmap_write(cp->pmureg, CP_DUMP_PC_SYS_PWR_REG, 0);
	regmap_write(cp->pmureg, LOGIC_RESET_CP_SYS_PWR_REG, 0);
	regmap_write(cp->pmureg, TCXO_GATE_CP_SYS_PWR_REG, 0);
	regmap_write(cp->pmureg, CP_DISABLE_ISO_SYS_PWR_REG, 1);
	regmap_write(cp->pmureg, CP_RESET_ISO_SYS_PWR_REG, 0);

	cp_ctrl_smc_update_bits(cp, CP_SMC_REG_CTRL_NS, CP_PWRON, 0);

	return cp_wait_bits(cp, CENTRAL_SEQ_CP_STATUS, CENTRAL_SEQ_CP_STATES_MASK,
			     CENTRAL_SEQ_CP_STATES_DOWN, "CENTRAL_SEQ_CP_STATUS (down)");
}

/*
 * cp_reset_release[] from pmucal_cp_exynos7885.h. Structurally the same
 * shape as gnss_kepler_release_reset() -- SWEEPER_BYPASS_DATA_EN bracket,
 * PWRON+START, wait for the central sequencer RUN state, then the
 * CLEANY_BUS handshake.
 */
static int cp_release_reset(struct cp_pmu_probe *cp)
{
	int ret;

	dev_info(cp->dev, "release_reset\n");

	cp_shared_reg_enable(cp);

	cp_ctrl_smc_update_bits(cp, CP_SMC_REG_CTRL_NS, SWEEPER_BYPASS_DATA_EN,
				 SWEEPER_BYPASS_DATA_EN);
	cp_ctrl_smc_update_bits(cp, CP_SMC_REG_CTRL_NS, CP_PWRON, CP_PWRON);
	cp_ctrl_smc_update_bits(cp, CP_SMC_REG_CTRL_S, CP_START, CP_START);

	ret = cp_wait_bits(cp, CENTRAL_SEQ_CP_STATUS, CENTRAL_SEQ_CP_STATES_MASK,
			    CENTRAL_SEQ_CP_STATES_RUN, "CENTRAL_SEQ_CP_STATUS (run)");
	if (ret)
		goto done;

	regmap_update_bits(cp->pmureg, CLEANY_BUS_CP_CONFIGURATION,
			    CLEANY_BUS_CP_REQ, CLEANY_BUS_CP_REQ);
	ret = cp_wait_bits(cp, CLEANY_BUS_CP_STATUS, CLEANY_BUS_CP_STATUS_MASK,
			    CLEANY_BUS_CP_STATUS_DONE, "CLEANY_BUS_CP_STATUS");

done:
	cp_ctrl_smc_update_bits(cp, CP_SMC_REG_CTRL_NS, SWEEPER_BYPASS_DATA_EN, 0);
	cp_shared_reg_disable(cp);

	return ret;
}

/*
 * Measured 2026-08-13: the CP power sequence could not be run as it stood.
 * Its very first access -- the read side of the CP_CTRL_NS
 * read-modify-write -- took a synchronous external abort
 * (esr 0x96000210) in regmap_mmio_read32le(), killing the calling task;
 * a subsequent PMU read then took the whole board down. (It recovered on
 * its own, which does at least prove the panic/watchdog reset path these
 * configs were added for.)
 *
 * The reason was in downstream itself: pmucal_is_cp_smc_regs() singles
 * out exactly these two offsets, 0x30 (CP_CTRL_NS) and 0x34 (CP_CTRL_S),
 * and routes them through exynos_smc() rather than touching them
 * directly -- they are secure-only on this SoC. Every other CP register
 * here (CP_STAT 0x38, CENTRAL_SEQ_CP_STATUS 0x284, the shared-regulator
 * pair) is readable and fine.
 *
 * 2026-08-14: cp_ctrl_smc_read()/_write() above implement that call, so
 * these three are live again. Read `cp_ctrl` first -- it exercises the
 * SMC read path alone, and if the firmware rejects the call you learn
 * that without having written anything.
 */
static int cp_ctrl_show(struct seq_file *s, void *unused)
{
	struct cp_pmu_probe *cp = s->private;
	u32 val;
	int ret;

	ret = cp_ctrl_smc_read(cp, CP_SMC_REG_CTRL_S, &val);
	seq_printf(s, "CP_CTRL_S  (0x%04x) ", CP_CTRL_S);
	if (ret)
		seq_printf(s, "SMC READ FAILED (%d)\n", ret);
	else
		seq_printf(s, "0x%08x  PWRON=%d START=%d\n", val,
			   !!(val & CP_PWRON), !!(val & CP_START));

	ret = cp_ctrl_smc_read(cp, CP_SMC_REG_CTRL_NS, &val);
	seq_printf(s, "CP_CTRL_NS (0x%04x) ", CP_CTRL_NS);
	if (ret)
		seq_printf(s, "SMC READ FAILED (%d)\n", ret);
	else
		seq_printf(s, "0x%08x  PWRON=%d START=%d\n", val,
			   !!(val & CP_PWRON), !!(val & CP_START));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(cp_ctrl);

/*
 * Ask EL3 to run CP secure boot, with the SSS clock bracket downstream
 * wraps around it.
 *
 * Nothing has been loaded into CP DRAM at this point, and that is the
 * whole idea: we are not trying to boot the modem yet, only to find out
 * whether EL3 will engage at all. A bare CP power-on (no SSS clock, no
 * boot mode) previously returned 0x000b = CP_NOT_WORKING, which says
 * nothing about whether the image would be accepted. If this call still
 * reports CP_NOT_WORKING then EL3 is refusing before it looks at
 * anything; if it moves to CP_NOT_VALID_MAGIC / CP_INVALID_CP_BASE /
 * CP_CORRUPTED_CP_MEM_INFO then it did go and read the base we handed
 * it, and loading a real image is worth doing.
 *
 * Note the return value is *not* a Linux errno -- 0 is CP_NO_ERROR and
 * anything else is an index into cp_smc_err_string[].
 */
static unsigned long cp_sec_boot(struct cp_pmu_probe *cp, unsigned long mode,
				  unsigned long bl_size, unsigned long main_size,
				  unsigned long base)
{
	struct arm_smccc_res res;
	unsigned long x1 = mode;

	/*
	 * shm_get_use_cp_memory_map_flag() is a downstream DT property we do
	 * not have; fold the base in whenever the caller supplies one, which
	 * matches what that flag does on a normal boot.
	 */
	if (mode == CP_BOOT_MODE_NORMAL && base)
		x1 |= (u32)base;

	dev_info(cp->dev,
		 "sec_boot: mode=%lu x1=0x%08lx bl_size=0x%lx main_size=0x%lx\n",
		 mode, x1, bl_size, main_size);

	arm_smccc_smc(CP_SMC_CLK_ID, SSS_CLK_ENABLE, 0, 0, 0, 0, 0, 0, &res);
	arm_smccc_smc(CP_SMC_ID, x1, bl_size, main_size, 0, 0, 0, 0, &res);
	cp->sec_boot_ret = res.a0;
	arm_smccc_smc(CP_SMC_CLK_ID, SSS_CLK_DISABLE, 0, 0, 0, 0, 0, 0, &res);

	cp->sec_boot_done = true;
	cp->sec_boot_mode = mode;
	cp->sec_boot_bl_size = bl_size;
	cp->sec_boot_main_size = main_size;
	cp->sec_boot_base = base;

	dev_info(cp->dev, "sec_boot: -> 0x%08lx (%s)\n", cp->sec_boot_ret,
		 cp_smc_err_name(cp->sec_boot_ret));

	return cp->sec_boot_ret;
}

/*
 * A plain window onto the CP reserved region (modem_rmem in the a40 DT:
 * 0xf5400000 + 0x8800000, already carved out of the linear map's
 * allocator by /reserved-memory).
 *
 * Staging the radio image is normally the CP boot daemon's job, working
 * through the modem_v1 ioctls we have not ported. All that path really
 * does for our purposes is copy the partition into this region, so a
 * seekable file lets a shell do it instead:
 *
 *   dd if=/dev/mmcblk0p19 of=cp_mem bs=4096 count=2
 *
 * The layout matters. Downstream reads its memory map table from
 * cp_base + 0x200 + 0xa0 with the comment "0x200: TOC, 0xa0: cp memory
 * map offset", i.e. the raw partition image goes in at offset 0 --
 * a 0x200-byte TOC followed by the BOOT section, whose own 0xa0 offset
 * holds a "MEM"-tagged cp_reserved_map_table. Without a valid table
 * there EL3 has nothing to validate the base against.
 *
 * Not no-map, so memremap()'s RAM-aware path is the right tool, exactly
 * as for gnss_rmem.
 */
#define CP_MEM_PHYS_BASE	0xf5400000
#define CP_MEM_SIZE		0x8800000

static void *cp_mem_get(struct cp_pmu_probe *cp)
{
	if (!cp->cp_mem) {
		cp->cp_mem = memremap(CP_MEM_PHYS_BASE, CP_MEM_SIZE, MEMREMAP_WB);
		if (!cp->cp_mem)
			dev_err(cp->dev, "failed to map CP region 0x%x+0x%x\n",
				CP_MEM_PHYS_BASE, CP_MEM_SIZE);
	}

	return cp->cp_mem;
}

static ssize_t cp_mem_read(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	struct cp_pmu_probe *cp = file->private_data;
	void *mem = cp_mem_get(cp);

	if (!mem)
		return -ENOMEM;

	return simple_read_from_buffer(buf, count, ppos, mem, CP_MEM_SIZE);
}

static ssize_t cp_mem_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct cp_pmu_probe *cp = file->private_data;
	void *mem = cp_mem_get(cp);
	loff_t pos = *ppos;
	dma_addr_t dma;
	ssize_t ret;

	if (!mem)
		return -ENOMEM;

	ret = simple_write_to_buffer(mem, CP_MEM_SIZE, ppos, buf, count);

	/*
	 * The CP, and EL3's secure-world view used by CP secure boot, read
	 * this region from DRAM, not through our cacheable mapping, so
	 * clean what we just staged to the point of coherency. arm64 does
	 * not export dcache_clean_poc() to modules; a DMA_TO_DEVICE mapping
	 * on this non-coherent device performs exactly that clean.
	 */
	if (ret > 0) {
		dma = dma_map_single(cp->dev, mem + pos, ret, DMA_TO_DEVICE);
		if (dma_mapping_error(cp->dev, dma)) {
			dev_err(cp->dev, "cp_mem: cache clean failed at +0x%llx\n",
				(unsigned long long)pos);
			return -EIO;
		}
		dma_unmap_single(cp->dev, dma, ret, DMA_TO_DEVICE);
	}

	return ret;
}

static const struct file_operations cp_mem_fops = {
	.open = simple_open,
	.read = cp_mem_read,
	.write = cp_mem_write,
	.llseek = default_llseek,
};

/*
 * The shared IPC region (shmem,ipc_offset 0x7000000, ipc_size 0x800000
 * in the downstream a40 DT): magic/access words and the FMT/RAW circular
 * queues the CP bootloader and firmware poll. Both sides update head and
 * tail words that share cache lines, so a cacheable window plus cache
 * maintenance would race the CP; map it write-combine instead, exactly
 * as downstream shm_request_region() does (vmap + pgprot_writecombine).
 * ioremap_wc() refuses it because the region is RAM in the linear map.
 */
#define CP_IPC_PHYS_BASE	(CP_MEM_PHYS_BASE + 0x7000000)
#define CP_IPC_SIZE		0x800000

static void *ipc_mem_get(struct cp_pmu_probe *cp)
{
	unsigned int i, n = CP_IPC_SIZE >> PAGE_SHIFT;
	struct page **pages;

	if (cp->ipc_mem)
		return cp->ipc_mem;

	pages = kvmalloc_array(n, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return NULL;

	for (i = 0; i < n; i++)
		pages[i] = pfn_to_page(PHYS_PFN(CP_IPC_PHYS_BASE) + i);

	cp->ipc_mem = vmap(pages, n, VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	kvfree(pages);
	if (!cp->ipc_mem)
		dev_err(cp->dev, "failed to map IPC region 0x%x+0x%x\n",
			CP_IPC_PHYS_BASE, CP_IPC_SIZE);

	return cp->ipc_mem;
}

static ssize_t ipc_mem_read(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	struct cp_pmu_probe *cp = file->private_data;
	void *mem = ipc_mem_get(cp);

	if (!mem)
		return -ENOMEM;

	return simple_read_from_buffer(buf, count, ppos, mem, CP_IPC_SIZE);
}

static ssize_t ipc_mem_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct cp_pmu_probe *cp = file->private_data;
	void *mem = ipc_mem_get(cp);
	ssize_t ret;

	if (!mem)
		return -ENOMEM;

	ret = simple_write_to_buffer(mem, CP_IPC_SIZE, ppos, buf, count);
	if (ret > 0)
		wmb();	/* drain the write-combine buffer before the CP looks */

	return ret;
}

static const struct file_operations ipc_mem_fops = {
	.open = simple_open,
	.read = ipc_mem_read,
	.write = ipc_mem_write,
	.llseek = default_llseek,
};

static int sec_boot_show(struct seq_file *s, void *unused)
{
	struct cp_pmu_probe *cp = s->private;

	if (!cp->sec_boot_done) {
		seq_puts(s, "no call made yet\n");
		seq_puts(s, "usage: echo <mode> <bl_size> <main_size> [base] > sec_boot\n");
		seq_puts(s, "       (all values hex; mode 0=boot 1=dump 2=re_init)\n");
		return 0;
	}

	seq_printf(s, "mode      0x%lx\n", cp->sec_boot_mode);
	seq_printf(s, "bl_size   0x%lx\n", cp->sec_boot_bl_size);
	seq_printf(s, "main_size 0x%lx\n", cp->sec_boot_main_size);
	seq_printf(s, "base      0x%lx\n", cp->sec_boot_base);
	seq_printf(s, "return    0x%08lx (%s)\n", cp->sec_boot_ret,
		   cp_smc_err_name(cp->sec_boot_ret));

	return 0;
}

static int sec_boot_open(struct inode *inode, struct file *file)
{
	return single_open(file, sec_boot_show, inode->i_private);
}

static ssize_t sec_boot_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct cp_pmu_probe *cp = ((struct seq_file *)file->private_data)->private;
	unsigned long mode, bl_size, main_size, base = 0;
	char kbuf[64];
	int n;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;
	kbuf[count] = '\0';

	n = sscanf(kbuf, "%lx %lx %lx %lx", &mode, &bl_size, &main_size, &base);
	if (n < 3)
		return -EINVAL;
	if (mode > CP_BOOT_RE_INIT)
		return -EINVAL;

	cp_sec_boot(cp, mode, bl_size, main_size, base);

	return count;
}

static const struct file_operations sec_boot_fops = {
	.open = sec_boot_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = sec_boot_write,
};

static ssize_t start_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	cp_power_on(file->private_data);
	return count;
}

static const struct file_operations start_fops = {
	.open = simple_open,
	.write = start_write,
	.llseek = noop_llseek,
};

static ssize_t reset_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	cp_hold_reset(file->private_data);
	return count;
}

static const struct file_operations reset_fops = {
	.open = simple_open,
	.write = reset_write,
	.llseek = noop_llseek,
};

static ssize_t release_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	cp_release_reset(file->private_data);
	return count;
}

static const struct file_operations release_fops = {
	.open = simple_open,
	.write = release_write,
	.llseek = noop_llseek,
};

/*
 * The deliberately opt-in, one-register-at-a-time tool: write a hex
 * PMU_ALIVE offset (e.g. `echo 0284 > probe_offset`), get exactly one
 * regmap_read of it, logged via dev_info. Nothing here is looped or
 * bundled -- each write is one live hardware test. Check the device is
 * still reachable before probing the next offset.
 */
static ssize_t probe_offset_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct cp_pmu_probe *cp = file->private_data;
	char kbuf[16];
	u32 offset;
	int ret;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;
	kbuf[count] = '\0';

	ret = kstrtou32(kbuf, 16, &offset);
	if (ret)
		return ret;
	if (offset > 0xffff)
		return -EINVAL;

	dev_info(cp->dev, "probe_offset: about to read PMU_ALIVE+0x%04x -- "
		 "if this is the last line you see, that offset just hung "
		 "the bus\n", offset);

	cp->last_probe_offset = offset;
	cp->last_probe_ret = regmap_read(cp->pmureg, offset, &cp->last_probe_value);

	if (cp->last_probe_ret)
		dev_err(cp->dev, "probe_offset: 0x%04x -> err %d\n", offset,
			cp->last_probe_ret);
	else
		dev_info(cp->dev, "probe_offset: 0x%04x -> 0x%08x\n", offset,
			 cp->last_probe_value);

	return count;
}

static const struct file_operations probe_offset_fops = {
	.open = simple_open,
	.write = probe_offset_write,
	.llseek = noop_llseek,
};

static int last_probe_show(struct seq_file *s, void *unused)
{
	struct cp_pmu_probe *cp = s->private;

	if (cp->last_probe_ret)
		seq_printf(s, "0x%04x: <err %d>\n", cp->last_probe_offset,
			   cp->last_probe_ret);
	else
		seq_printf(s, "0x%04x: 0x%08x\n", cp->last_probe_offset,
			   cp->last_probe_value);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(last_probe);

/*
 * Convenience wrapper around the one register this whole driver exists
 * to test -- equivalent to `echo 0284 > probe_offset; cat last_probe`,
 * named for what it is. Still a single, deliberate, opt-in read: `cat`
 * on this file is itself a live hardware test, same as probe_offset.
 */
static int central_seq_cp_status_show(struct seq_file *s, void *unused)
{
	struct cp_pmu_probe *cp = s->private;
	u32 val;
	int ret;

	dev_info(cp->dev, "central_seq_cp_status: about to read PMU_ALIVE+0x0284\n");
	ret = regmap_read(cp->pmureg, CENTRAL_SEQ_CP_STATUS, &val);
	if (ret) {
		seq_printf(s, "<err %d>\n", ret);
		return 0;
	}

	seq_printf(s, "0x%08x (states=0x%02x)\n", val,
		   (u32)(val & CENTRAL_SEQ_CP_STATES_MASK) >> 16);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(central_seq_cp_status);

/* Registers already confirmed safe elsewhere in this investigation --
 * deliberately does NOT include anything CP-domain-specific. See the
 * file header comment.
 */
static int regs_show(struct seq_file *s, void *unused)
{
	struct cp_pmu_probe *cp = s->private;
	u32 val;

	regmap_read(cp->pmureg, CP_STAT, &val);
	seq_printf(s, "CP_STAT (0x%04x)                    = 0x%08x\n",
		   CP_STAT, val);
	regmap_read(cp->pmureg, EXT_REGULATOR_SHARED_STATUS, &val);
	seq_printf(s, "EXT_REGULATOR_SHARED_STATUS (0x%04x) = 0x%08x\n",
		   EXT_REGULATOR_SHARED_STATUS, val);
	regmap_read(cp->pmureg, EXT_REGULATOR_SHARED_OPTION, &val);
	seq_printf(s, "EXT_REGULATOR_SHARED_OPTION (0x%04x) = 0x%08x\n",
		   EXT_REGULATOR_SHARED_OPTION, val);
	/*
	 * The register cp_init[]'s final wait hangs on. Both cores' fields
	 * are shown because they share it and 0x5 means "done" for each:
	 * on the downstream kernel this read 0x3 in the CP field for every
	 * cp_init attempt, so the CP CPU was never released and its mailbox
	 * stayed silent. See kepler.md.
	 */
	regmap_read(cp->pmureg, RESET_SEQUENCER_STATUS, &val);
	seq_printf(s,
		   "RESET_SEQUENCER_STATUS (0x%04x)     = 0x%08x  CP=%lu GNSS=%lu (done=5)\n",
		   RESET_SEQUENCER_STATUS, val,
		   (val & CP_RESET_SEQ_STAT_MASK),
		   (val & GENMASK(6, 4)) >> 4);

	seq_printf(s, "CP_MBOX ISSR2 (CP_WAKEUP)    = 0x%08x\n",
		   readl(cp->cp_mbox + CP_MBOX_ISSR2));
	seq_printf(s, "CP_MBOX ISSR3                = 0x%08x\n",
		   readl(cp->cp_mbox + CP_MBOX_ISSR3));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(regs);

static void cp_pmu_probe_debugfs_init(struct cp_pmu_probe *cp)
{
	struct dentry *root = debugfs_create_dir("cp_pmu_probe", NULL);

	cp->debugfs_dir = root;

	debugfs_create_file("regs", 0400, root, cp, &regs_fops);
	debugfs_create_file("probe_offset", 0200, root, cp, &probe_offset_fops);
	debugfs_create_file("last_probe", 0400, root, cp, &last_probe_fops);
	debugfs_create_file("central_seq_cp_status", 0400, root, cp,
			     &central_seq_cp_status_fops);
	debugfs_create_file("cp_ctrl", 0400, root, cp, &cp_ctrl_fops);
	debugfs_create_file("sec_boot", 0600, root, cp, &sec_boot_fops);
	debugfs_create_file("cp_mem", 0600, root, cp, &cp_mem_fops);
	debugfs_create_file("ipc_mem", 0600, root, cp, &ipc_mem_fops);
	debugfs_create_bool("tcxo_sw", 0600, root, &cp->tcxo_sw);
	debugfs_create_file("start", 0200, root, cp, &start_fops);
	debugfs_create_file("reset", 0200, root, cp, &reset_fops);
	debugfs_create_file("release", 0200, root, cp, &release_fops);
}

static int cp_pmu_probe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cp_pmu_probe *cp;

	cp = devm_kzalloc(dev, sizeof(*cp), GFP_KERNEL);
	if (!cp)
		return -ENOMEM;

	cp->dev = dev;
	platform_set_drvdata(pdev, cp);

	cp->pmureg = syscon_regmap_lookup_by_phandle(dev->of_node,
						      "samsung,syscon-phandle");
	if (IS_ERR(cp->pmureg))
		return dev_err_probe(dev, PTR_ERR(cp->pmureg),
				      "failed to look up PMU syscon\n");

	cp->cp_mbox = devm_ioremap(dev, CP_MBOX_PHYS_BASE, CP_MBOX_SIZE);
	if (!cp->cp_mbox)
		return dev_err_probe(dev, -ENOMEM,
				      "failed to map CP mailbox registers\n");

	cp_pmu_probe_debugfs_init(cp);

	dev_info(dev, "probed (PMU-only, no mailbox IPC, no firmware)\n");

	return 0;
}

static void cp_pmu_probe_remove(struct platform_device *pdev)
{
	struct cp_pmu_probe *cp = platform_get_drvdata(pdev);

	debugfs_remove_recursive(cp->debugfs_dir);

	if (cp->cp_mem)
		memunmap(cp->cp_mem);
	if (cp->ipc_mem)
		vunmap(cp->ipc_mem);
}

static const struct of_device_id cp_pmu_probe_of_match[] = {
	{ .compatible = "samsung,exynos7885-cp-pmu-probe" },
	{}
};
MODULE_DEVICE_TABLE(of, cp_pmu_probe_of_match);

static struct platform_driver cp_pmu_probe_driver = {
	.probe = cp_pmu_probe_probe,
	.remove = cp_pmu_probe_remove,
	.driver = {
		.name = "cp_pmu_probe",
		.of_match_table = cp_pmu_probe_of_match,
	},
};
module_platform_driver(cp_pmu_probe_driver);

MODULE_DESCRIPTION("Samsung CP (modem) PMU-only bring-up probe");
MODULE_LICENSE("GPL");
