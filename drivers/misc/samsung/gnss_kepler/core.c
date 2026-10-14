// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung "Kepler" GNSS companion chip bring-up driver.
 *
 * Milestone 1 (see /w/linux/gnss.md): PMU power-on sequencing and
 * fault-region readback. Milestone 2: a minimal MCU_IPC mailbox
 * implementation, just enough to send one BCMD the same way the
 * downstream faultdump tool does (zeroed cmd_id/flags/param1/param2) and
 * see whether that reaches the same fault. No firmware load, no ipc_msg
 * ring-buffer channel, no full ioctl UAPI -- those are later milestones,
 * if any of them turn out to matter.
 *
 * The goal throughout is to find out whether a from-scratch mainline
 * port reaches the same "MEM_BLC RV:1" fault the downstream (4.4.177)
 * vendor driver does, with every downstream fix found during the
 * hardware investigation applied from the start rather than
 * rediscovered.
 *
 * PMU_ALIVE and MCU_IPC offsets/bits and the fault-region layout are
 * taken from the downstream drivers/misc/gnss_if/pmu-gnss.[ch],
 * gnss_keplerctl_device.c, gnss_link_device_shmem.c, and
 * drivers/misc/mcu_ipc/{mcu_ipc.c,regs-mcu_ipc.h} -- see gnss.md for the
 * full writeup and the hardware investigation that produced these
 * values.
 */

#include <linux/completion.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/remoteproc.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

/*
 * PMU_ALIVE offsets, relative to the pmu_system_controller syscon base
 * (0x11c80000) -- same syscon the WLBT driver uses.
 */
#define GNSS_CTRL_NS			0x0040
#define GNSS_CTRL_S			0x0044
#define GNSS_STAT			0x0048
#define GNSS_DEBUG			0x004c
#define CENTRAL_SEQ_GNSS_CONFIGURATION	0x02c0
#define CENTRAL_SEQ_GNSS_STATUS		0x02c4
#define RESET_SEQUENCER_STATUS		0x0504
#define RESET_AHEAD_GNSS_SYS_PWR_REG	0x1340
#define CLEANY_BUS_SYS_PWR_REG		0x1344
#define LOGIC_RESET_GNSS_SYS_PWR_REG	0x1348
#define TCXO_GATE_GNSS_SYS_PWR_REG	0x134c
#define GNSS_DISABLE_ISO_SYS_PWR_REG	0x1350
#define GNSS_RESET_ISO_SYS_PWR_REG	0x1354

/*
 * The GNSS->AP memory window, from gnss_pmu_init_conf() and pmu-gnss.h
 * downstream. This is GNSS's counterpart to the BAAW windows WLBT
 * (platform_mif.c) and CP (modem_ctrl_ss310ap.c) each program for
 * themselves: MEM_CONFIG0/1 tell the Kepler core where in AP DRAM its
 * shared memory actually is. Nothing else on this kernel writes them,
 * and they do not power up pointing at our reserved region -- read back
 * live on this board, GNSS_MEM_BA0 held 0x00060000 (base 0x60000000)
 * while gnss_rmem is at 0xffc00000. See gnss.md, "The missing step".
 */
#define GNSS2AP_MEM_CONFIG0		0x7250	/* GNSS_MEM_SIZE0 */
#define GNSS2AP_MEM_CONFIG1		0x7254	/* GNSS_MEM_BA0 */
#define GNSS2AP_MIF_ACCESS_WIN0		0x7260
#define GNSS2AP_PERI_ACCESS_WIN0	0x7278
#define GNSS2AP_PERI_ACCESS_WIN1	0x727c

#define GNSS_MEMSIZE_MASK		GENMASK(19, 0)
#define GNSS_MEMSIZE_RES		SZ_4K
#define GNSS_MEMBASE_ADDR_MASK		GENMASK(23, 0)
#define GNSS_MEMBASE_ADDR_SHIFT		12

#define GNSS_PWRON			BIT(1)
/*
 * GNSS_CTRL_NS bit 11, TCXO_ENABLE_SW in downstream's pmu-gnss.h. No .c
 * file downstream ever writes it: on stock the TCXO is shared with the CP
 * (hence EXT_REGULATOR_SHARED_* and the TCXO_SHARED_MODE ioctl), and the
 * running CP holds it up. Under postmarketOS there is no CP firmware, so
 * nothing does -- and the GNSS firmware dies on its first RF access with
 * "SPDY RD FAIL" (see kepler.md 13.9). This bit is the GNSS side's own
 * software enable for that clock. Off by default; opt in with
 *     echo 1 > /sys/kernel/debug/gnss_kepler/tcxo_sw
 * before powering the core, so the A/B stays explicit.
 */
#define GNSS_TCXO_ENABLE_SW		BIT(11)
#define GNSS_RESET_SET			BIT(2)
#define GNSS_START			BIT(3)
#define GNSS_ACTIVE_REQ_CLR		BIT(6)
#define GNSS_RESET_REQ_CLR		BIT(8)

/*
 * GNSS's field within the shared RESET_SEQUENCER_STATUS register.
 * See gnss.md, "Confirmed -- the wait was the fix, and the core is
 * alive": without this wait after PWRON+START, the core never produces
 * its first interrupt. This is the downstream fix, applied here from
 * the start rather than rediscovered.
 */
#define GNSS_RESET_SEQ_STAT_MASK	GENMASK(6, 4)
#define GNSS_RESET_SEQ_STAT_DONE	(0x5 << 4)

/*
 * CENTRAL_SEQ_GNSS_STATUS (0x02c4) "STATES" field, and the separate
 * CLEANY_BUS_GNSS_CONFIGURATION/STATUS request/status pair at
 * 0x3a20/0x3a24 -- not SYS_PWR_REG SFRs, only appear in
 * pmucal_gnss_exynos7885.h's gnss_reset_release[] downstream. Same
 * fix already found and verified against the downstream kernel this
 * session (see gnss.md, "gnss_pmu_release_reset() is missing three
 * steps"), ported here rather than rediscovered.
 */
#define CENTRAL_SEQ_GNSS_STATES_MASK	GENMASK(23, 16)
#define CENTRAL_SEQ_GNSS_STATES_RUN	(0x0 << 16)
#define CLEANY_BUS_GNSS_CONFIGURATION	0x3a20
#define CLEANY_BUS_GNSS_STATUS		0x3a24
#define CLEANY_BUS_GNSS_REQ		BIT(0)
#define CLEANY_BUS_GNSS_STATUS_MASK	GENMASK(17, 16)
#define CLEANY_BUS_GNSS_STATUS_DONE	(0x0 << 16)
#define SWEEPER_BYPASS_DATA_EN		BIT(16)

/*
 * CP_STAT / EXT_REGULATOR_SHARED_{OPTION,STATUS} -- also PMU_ALIVE SFRs,
 * not gnss_if-owned ones, read here for the regs dump only.
 * gnss_pmu_release_reset() (the CONFIG_GNSS_PMUCAL=n branch, the one
 * actually compiled downstream) calls exynos_pmu_shared_reg_enable()/
 * _disable() from mcu_ipc.c right around the PMU sequence below. That
 * function sets EXT_REGULATOR_SHARED_OPTION bit 2 *and* a CP_WAKEUP
 * request bit in the CP mailbox's own ISSR2 (a separate IP block at
 * 0x12080000 -- distinct from the GNSS mailbox at 0x120d0000 this driver
 * maps), i.e. GNSS asks CP's firmware to keep a shared regulator up for
 * the duration of the PMU release-reset sequence. See gnss.md. The CP
 * mailbox driver provides that vote as the "shared" regulator, counted
 * against the CP driver's own.
 */
#define EXT_REGULATOR_SHARED_STATUS	0x3644
#define EXT_REGULATOR_SHARED_OPTION	0x3648
#define CP_STAT				0x0038

/*
 * Fault region: offset 0x200000 into the reserved memory-region, DT
 * property `fault_info = <1 0x200000 0x180000>` downstream. In
 * practice only ~40 bytes are ever real data (a 16-byte type+string
 * record plus 24 bytes of PC/address-shaped fields), the rest is a
 * fixed 0xcc fill -- see gnss.md, "The raw MEM_BLC fault dump" and
 * "The fault region is 1.5 MB, but REQ_FAULT_INFO only ever returns 16
 * bytes". Dump the first 0x40 bytes; that is everything real.
 */
#define GNSS_FAULT_OFFSET	0x200000
#define GNSS_FAULT_DUMP_SIZE	0x40

/*
 * MCU_IPC mailbox window at 0x120d0000 (drivers/misc/mcu_ipc/
 * regs-mcu_ipc.h downstream). INTGR1/INTCR1 generate/clear an
 * interrupt to the *remote* (GNSS) side; INTGR0/INTCR0/INTSR0/INTMR0
 * are the local (AP) side's own generate/clear/status/mask for
 * interrupts GNSS raises back. ISSRn are 32-bit scratch registers
 * shared by both sides -- the only "payload" this IP has.
 */
#define MCU_IPC_MCUCTLR		0x00
#define MCU_IPC_INTGR0		0x08
#define MCU_IPC_INTCR0		0x0c
#define MCU_IPC_INTMR0		0x10
#define MCU_IPC_INTSR0		0x14
#define MCU_IPC_INTGR1		0x1c
#define MCU_IPC_INTCR1		0x20
#define MCU_IPC_ISSR0		0x80
#define MCU_IPC_ISSR(n)		(MCU_IPC_ISSR0 + 4 * (n))

#define MCU_IPC_MSWRST		BIT(0)

/*
 * BCMD wire format, from kepler_req_bcmd() (gnss_keplerctl_device.c):
 * ISSR[CTRL0] = (flags << 16) | cmd_id, ISSR[CTRL1] = param1,
 * ISSR[CTRL2] = param2, ISSR[CTRL3] = 0xff (a "not yet answered by
 * Kepler" sentinel -- the core overwrites it with the real return
 * value), then ring the AP2GNSS_BCMD doorbell. Downstream's DT sets
 * mbx,reg_bcmd_ctrl = <0>,<1>,<2>,<3> and
 * mbx,int_ap2gnss_bcmd = <0> -- both are fixed per-product constants
 * in this driver's design, not discovered per-board, so they're
 * `#define`s here rather than DT properties.
 */
#define GNSS_MBOX_BCMD_CTRL0	MCU_IPC_ISSR(0)
#define GNSS_MBOX_BCMD_CTRL1	MCU_IPC_ISSR(1)
#define GNSS_MBOX_BCMD_CTRL2	MCU_IPC_ISSR(2)
#define GNSS_MBOX_BCMD_CTRL3	MCU_IPC_ISSR(3)
#define GNSS_MBOX_BCMD_CTRL3_PENDING	0xff
#define GNSS_MBOX_INT_AP2GNSS_BCMD	0

/* REQ_BCMD_TIMEOUT downstream (gnss_keplerctl_device.c). */
#define REQ_BCMD_TIMEOUT		200
#define REQ_BCMD_POLL_INTERVAL		50	/* us */

/*
 * The fault-info doorbell: a *different* interrupt from the BCMD one
 * (mbx,int_ap2gnss_req_fault_info = <1> downstream) with its own, much
 * longer timeout. This matters -- on 4.4, `cmd_id=0` on int 0 times out
 * exactly as it does here, but the core *does* answer on int 1: it
 * raises the gnss2ap fault-info interrupt and writes the payload size
 * into ISSR3. It is the only channel proven live on this silicon, and
 * the one this port had never rung. See gnss.md, "The 4.4 register
 * diff".
 */
#define GNSS_MBOX_INT_AP2GNSS_REQ_FAULT_INFO	1
#define REQ_FAULT_INFO_TIMEOUT		1000

/*
 * Command IDs, recovered from the vendor daemon's own BCMD call sites
 * (aarch64 disassembly of vendor/bin/hw/gpsd -- see gnss.md). Only
 * BLC_BRANCH is named downstream, as the one command with no reply.
 */
#define GNSS_BCMD_READ32		0x05

/* Kepler's EXE-address register, and the aperture that must not be read. */
#define GNSS_EXE_ADDR_UNSAFE		0x40043004
#define GNSS_EXE_ADDR_SAFE		0x80043004
#define GNSS_BCMD_WRITE32		0x06
#define GNSS_BCMD_BLC_BRANCH		0x04

/*
 * shmem,boot_without_mbox = <1> downstream (this board): LOAD_FIRMWARE
 * at offset 0 doesn't write firmware there -- it writes a 16-byte
 * autorun command (shmem_set_autorun_cmd()), and firmware goes at
 * offset AUTO_RUN_CMD_SIZE instead. cmd[0]=GNSS_AUTO_RUN magic,
 * cmd[1]=fw start address (0, i.e. right after this header),
 * cmd[2]=size, cmd[3]=reserved. Verified byte-exact round-trip against
 * the downstream driver earlier this session -- see gnss.md, "LOAD_FIRMWARE
 * with a synthetic image" / "A real patch blob does exist".
 */
#define GNSS_AUTO_RUN_MAGIC	0x00040001
#define AUTO_RUN_CMD_SIZE	0x10

/*
 * The ipc_msg ring layer. Layout from the downstream DT
 * (exynos7885-*_common.dtsi, gnss_pdata):
 *
 *	shmem,ipc_offset  = <0x380000>
 *	shmem,ipc_size    = <0x80000>
 *	shmem,ipc_reg_cnt = <32>
 *
 * and shmem_remap_ipc_region() (gnss_link_device_shmem.c:636), which
 * carves that region up as:
 *
 *	+0x00000  RXQ ring buffer, ipc_size / 2
 *	+0x40000  TXQ ring buffer, ipc_size / 2 - 32 * 4
 *	+0x7ff80  32 u32 "shared registers"
 *
 * Note the asymmetry is real, not a rounding artifact: the register
 * array is carved out of the TXQ half only, and the TXQ buffer abuts it
 * exactly. Kepler sees the whole thing at 0x50380000 (see gnss.md,
 * "The ROM, disassembled" for that mapping).
 */
#define GNSS_IPC_OFFSET		0x380000
#define GNSS_IPC_SIZE		0x80000
#define GNSS_IPC_REG_CNT	32
#define GNSS_IPC_REG_BYTES	(GNSS_IPC_REG_CNT * sizeof(u32))

/*
 * Shared-register indices, from the downstream DT. Each property is
 * <device index>, and device 1 is GNSS_IPC_SHMEM (gnss.h), so on this
 * board every ring pointer lives in shared memory rather than in a
 * mailbox ISSR -- which is why the mailbox ISSRs stayed empty through
 * all the earlier BCMD work.
 *
 *	reg_tx_tail = <1 0>	reg_tx_head    = <1 1>
 *	reg_rx_tail = <1 2>	reg_rx_head    = <1 3>
 *	reg_tx_ipc_msg = <1 4>	reg_rx_ipc_msg = <1 5>
 */
#define GNSS_IPC_REG_TX_TAIL	0
#define GNSS_IPC_REG_TX_HEAD	1
#define GNSS_IPC_REG_RX_TAIL	2
#define GNSS_IPC_REG_RX_HEAD	3
#define GNSS_IPC_REG_TX_MSG	4
#define GNSS_IPC_REG_RX_MSG	5

/* mbx,int_ap2gnss_ipc_msg = <2>, mbx,irq_gnss2ap_ipc_msg = <2>. */
#define GNSS_MBOX_INT_AP2GNSS_IPC_MSG	2

/*
 * send_int2gnss(shmd, 0x82) -- the only value downstream ever puts in
 * reg_tx_ipc_msg (gnss_link_device_shmem.c:551). Ported as the opaque
 * constant it is there; the bits are undocumented.
 */
#define GNSS_IPC_TX_MSG_MASK	0x82

/*
 * EXYNOS link-layer frame header, 12 bytes (exynos_ipc.h). The struct
 * declared there omits the start field even though the offset macros
 * include it, and the macros are what the code actually uses, so the
 * true wire layout is:
 *
 *	+0  u16 start = 0xabcd
 *	+2  u16 seq
 *	+4  u16 frag config (single frame = 0xc0 in the high byte)
 *	+6  u16 len -- total frame length, header included
 *	+8  u8  ch_id
 *	+9  u8  ch_seq
 *	+10 u16 reserved
 *
 * Frames are padded to a 4-byte boundary; the padding is part of what
 * the ring pointers advance over but not of the length field.
 */
#define EXYNOS_HEADER_SIZE	12
#define EXYNOS_START_MAGIC	0xabcd
#define EXYNOS_START_OFFSET	0
#define EXYNOS_SEQ_OFFSET	2
#define EXYNOS_FRAG_OFFSET	4
#define EXYNOS_LEN_OFFSET	6
#define EXYNOS_CH_ID_OFFSET	8
#define EXYNOS_CH_SEQ_OFFSET	9
#define EXYNOS_SINGLE_MASK	0xc0

/* How many received frames to keep for the ipc_rx debugfs file. */
#define GNSS_IPC_RX_LOG_FRAMES	8
#define GNSS_IPC_RX_LOG_BYTES	256

struct gnss_kepler {
	struct device *dev;
	struct regmap *pmureg;
	void __iomem *dram;
	void __iomem *mbox;
	bool bcmd_keep_loop;
	bool hold_shared_reg;
	/*
	 * The shared-regulator vote and the modem's power reference, each
	 * held at most once; power_lock protects both flags.
	 */
	struct mutex power_lock;
	struct regulator *shared_reg;
	bool shared_reg_on;
	struct rproc *modem;		/* NULL without samsung,modem */
	bool modem_held;
	bool need_modem;
	bool load_fw_autorun;
	bool tcxo_sw;
	enum {
		GNSS_STATE_OFFLINE,
		GNSS_STATE_HOLD_RESET,
		GNSS_STATE_ONLINE,
	} state;
	struct miscdevice miscdev;
	wait_queue_head_t ipc_rx_wait;
	unsigned int ipc_rx_log_out;
	u8 tx_channel;
	u32 reset_settle_ms;
	u32 power_off_ms;
	phys_addr_t mem_base;
	size_t mem_size;
	int irq_active;
	int irq_watchdog;
	int irq_mbox;
	struct completion bcmd_cmpl;
	struct completion fault_cmpl;
	struct dentry *debugfs_dir;
	unsigned int active_count;
	unsigned int watchdog_count;
	unsigned int mbox_irq_count;
	size_t fw_offset;

	/* ipc_msg ring layer */
	u8 __iomem *rxq_buff;
	u8 __iomem *txq_buff;
	u32 rxq_size;
	u32 txq_size;
	u32 __iomem *ipc_reg;
	spinlock_t tx_lock;
	u16 tx_seq;
	u8 tx_ch_seq;
	struct work_struct ipc_rx_work;
	unsigned int ipc_rx_frames;
	unsigned int ipc_rx_bad;
	/* Last few RX frames, for the ipc_rx debugfs file. */
	struct {
		u32 len;
		u8 data[GNSS_IPC_RX_LOG_BYTES];
	} ipc_rx_log[GNSS_IPC_RX_LOG_FRAMES];
	unsigned int ipc_rx_log_in;
	spinlock_t ipc_rx_log_lock;
};

/*
 * Both lines are level-triggered and stay asserted at the PMU until
 * explicitly cleared there -- matching downstream's
 * gnss_pmu_clear_interrupt(). Learned the hard way: an earlier version
 * of this handler only logged and returned, which left the line
 * asserted and caused an immediate infinite re-fire storm (100k+
 * interrupts in under a millisecond) that livelocked the whole device
 * solid enough that even ICMP stopped responding -- this board also
 * has its hardware watchdog permanently disabled
 * ("s3c2410-wdt: watchdog inactive, reset disabled"), so there was no
 * automatic recovery; it needed a physical power cycle. See gnss.md.
 */
static irqreturn_t gnss_kepler_active_irq(int irq, void *data)
{
	struct gnss_kepler *gk = data;

	gk->active_count++;
	dev_info(gk->dev, "ACTIVE interrupt (#%u)\n", gk->active_count);

	regmap_update_bits(gk->pmureg, GNSS_CTRL_NS, GNSS_ACTIVE_REQ_CLR,
			    GNSS_ACTIVE_REQ_CLR);

	return IRQ_HANDLED;
}

static irqreturn_t gnss_kepler_watchdog_irq(int irq, void *data)
{
	struct gnss_kepler *gk = data;

	gk->watchdog_count++;
	dev_info(gk->dev, "WATCHDOG interrupt (#%u)\n", gk->watchdog_count);

	regmap_update_bits(gk->pmureg, GNSS_CTRL_NS, GNSS_RESET_REQ_CLR,
			    GNSS_RESET_REQ_CLR);

	return IRQ_HANDLED;
}

/*
 * One shared GIC line (GIC_SPI 56) demuxed through INTSR0's upper 16
 * bits, matching mcu_ipc_handler() downstream. Milestone 2 doesn't
 * need per-bit dispatch (that's the ipc_msg ring-buffer machinery,
 * still out of scope) -- just log which bits fired and clear them all,
 * enough to see whether a BCMD send gets any response at all.
 */
static irqreturn_t gnss_kepler_mbox_irq(int irq, void *data)
{
	struct gnss_kepler *gk = data;
	u32 stat = readl(gk->mbox + MCU_IPC_INTSR0) & 0xffff0000;

	writel(stat, gk->mbox + MCU_IPC_INTCR0);
	gk->mbox_irq_count++;
	dev_info(gk->dev, "mbox interrupt (#%u), INTSR0 bits 0x%04x\n",
		 gk->mbox_irq_count, stat >> 16);

	/*
	 * int 0 (bit 16) is mbx,irq_gnss2ap_bcmd -- the BCMD reply. Same
	 * signal kepler_req_bcmd() waits on downstream.
	 */
	if (stat & BIT(16))
		complete(&gk->bcmd_cmpl);

	/* int 1 = mbx,irq_gnss2ap_rsp_fault_info -- what gc->fault_cmpl
	 * waits on downstream.
	 */
	if (stat & BIT(17))
		complete(&gk->fault_cmpl);

	/*
	 * int 2 = mbx,irq_gnss2ap_ipc_msg. Downstream defers to a tasklet
	 * (shmem_irq_msg_handler -> ipc_rx_task); a workqueue is enough
	 * here and lets the RX path sleep for its per-frame allocation.
	 */
	if (stat & BIT(18))
		schedule_work(&gk->ipc_rx_work);

	return IRQ_HANDLED;
}

/*
 * The core answers a BCMD by writing ISSR3 and *nothing else* -- it
 * never raises the int 0 reply interrupt on this silicon. Downstream
 * (and this driver until 2026-08-13) waits on a completion fed by that
 * interrupt, so it reports a 200 ms timeout while the answer is already
 * sitting in the register. Poll ISSR3 instead; gk->bcmd_cmpl and the
 * ISR's complete() are kept only to record whether int 0 ever fires.
 *
 * Two subtleties, both measured:
 *
 *  - A legitimate return value can be 0, so "changed from the 0xff
 *    sentinel" is the only sound completion test.
 *  - While the core is in hold_reset, ISSR3 reads back 0 no matter what
 *    was written (ISSR0..2 latch fine). Polling for "not 0xff" would
 *    then instantly succeed with a bogus 0, so verify the sentinel
 *    actually stuck before polling at all. It doesn't stick in exactly
 *    the case that matters: a BCMD staged before release_reset, where
 *    the answer only appears once the core comes out of reset and the
 *    caller has to read ISSR3 afterwards.
 */
static void gnss_kepler_send_bcmd(struct gnss_kepler *gk, u16 cmd_id,
				   u16 flags, u32 param1, u32 param2)
{
	u32 val;
	int ret;

	dev_info(gk->dev,
		 "send_bcmd cmd_id=0x%x flags=0x%x param1=0x%x param2=0x%x\n",
		 cmd_id, flags, param1, param2);

	reinit_completion(&gk->bcmd_cmpl);

	/*
	 * ctrl[CTRL0] = (flags << 16) + cmd_id, exactly as
	 * kepler_req_bcmd() packs it downstream -- cmd_id in the low half.
	 * (gpsd's struct kepler_bcmd_args holds the two the other way
	 * round; the downstream ioctl handler swaps them on the way
	 * through, so this is the packing the core actually sees.)
	 */
	writel(((u32)flags << 16) | cmd_id, gk->mbox + GNSS_MBOX_BCMD_CTRL0);
	writel(param1, gk->mbox + GNSS_MBOX_BCMD_CTRL1);
	writel(param2, gk->mbox + GNSS_MBOX_BCMD_CTRL2);
	writel(GNSS_MBOX_BCMD_CTRL3_PENDING, gk->mbox + GNSS_MBOX_BCMD_CTRL3);
	val = readl(gk->mbox + GNSS_MBOX_BCMD_CTRL3);

	writel(BIT(GNSS_MBOX_INT_AP2GNSS_BCMD), gk->mbox + MCU_IPC_INTGR1);

	/* BLC_Branch has no return value -- downstream skips the wait. */
	if (cmd_id == GNSS_BCMD_BLC_BRANCH)
		return;

	if (val != GNSS_MBOX_BCMD_CTRL3_PENDING) {
		dev_info(gk->dev,
			 "bcmd cmd_id=0x%x staged while held (ISSR3 reads 0x%08x, sentinel did not stick) -- read ISSR3 after release_reset\n",
			 cmd_id, val);
		return;
	}

	ret = readl_poll_timeout(gk->mbox + GNSS_MBOX_BCMD_CTRL3, val,
				 val != GNSS_MBOX_BCMD_CTRL3_PENDING,
				 REQ_BCMD_POLL_INTERVAL,
				 REQ_BCMD_TIMEOUT * USEC_PER_MSEC);
	if (ret) {
		dev_err(gk->dev,
			"bcmd cmd_id=0x%x NO ANSWER after %d ms (ISSR3 still 0x%08x)\n",
			cmd_id, REQ_BCMD_TIMEOUT, val);
		return;
	}

	dev_info(gk->dev, "bcmd cmd_id=0x%x returned 0x%08x%s\n", cmd_id, val,
		 completion_done(&gk->bcmd_cmpl) ? "" : " (no int 0 raised)");
}

/*
 * The GNSS shared memory is accessed 16 bits at a time. Downstream does
 * this unconditionally for every IPC transfer (gnss_memcpy16_*_io(),
 * gnss_circ_*16_*_io() in gnss_link_device_memory.c) and never explains
 * why, so it is ported as-is rather than "simplified" to memcpy. The
 * firmware payload area demonstrably tolerates byte writes -- that is
 * how the K148 image gets in, verified byte-exact in Kepler RAM -- so
 * whatever the constraint is, it is specific to this region or to
 * concurrent access by the running core. Not worth finding out the hard
 * way.
 *
 * Counts are always even here: frames are padded to 4 bytes.
 */
static void gnss_kepler_memcpy16_to_io(void __iomem *to, const void *from,
					u32 count)
{
	const u16 *s = from;
	u16 __iomem *d = to;

	for (count >>= 1; count; count--)
		iowrite16(*s++, d++);
}

static void gnss_kepler_memcpy16_from_io(void *to, const void __iomem *from,
					  u32 count)
{
	const u16 __iomem *s = from;
	u16 *d = to;

	for (count >>= 1; count; count--)
		*d++ = ioread16(s++);
}

/* circ_get_space()/circ_get_usage()/circ_new_pointer(), memory.h. */
static u32 circ_space(u32 qsize, u32 in, u32 out)
{
	return (in < out) ? (out - in - 1) : (qsize + out - in - 1);
}

static u32 circ_usage(u32 qsize, u32 in, u32 out)
{
	return (in >= out) ? (in - out) : (qsize - out + in);
}

static u32 circ_advance(u32 qsize, u32 p, u32 len)
{
	p += len;

	return (p < qsize) ? p : (p - qsize);
}

static void circ_write16(void __iomem *dst, const void *src, u32 qsize, u32 in,
			  u32 len)
{
	u32 first = min(len, qsize - in);

	gnss_kepler_memcpy16_to_io(dst + in, src, first);
	if (len > first)
		gnss_kepler_memcpy16_to_io(dst, src + first, len - first);
}

static void circ_read16(void *dst, const void __iomem *src, u32 qsize, u32 out,
			 u32 len)
{
	u32 first = min(len, qsize - out);

	gnss_kepler_memcpy16_from_io(dst, src + out, first);
	if (len > first)
		gnss_kepler_memcpy16_from_io(dst + first, src, len - first);
}

static u32 ipc_reg_read(struct gnss_kepler *gk, unsigned int idx)
{
	return ioread32(gk->ipc_reg + idx);
}

static void ipc_reg_write(struct gnss_kepler *gk, unsigned int idx, u32 val)
{
	iowrite32(val, gk->ipc_reg + idx);
}

/*
 * shmem_remap_ipc_region() + clear_shmem_map(): lay out the ring
 * pointers, then zero the entire IPC region. Downstream sets the four
 * pointers to 0 and *then* memsets the whole region (which zeroes them
 * again), so a single memset is equivalent -- but the region must be
 * clean before the core boots, because the firmware finds a scheduled
 * port with a stale head/tail and asserts. That assert
 * (`ScheduledPortInQueue 145`) is what this whole layer exists to fix.
 */
static void gnss_kepler_ipc_reset(struct gnss_kepler *gk)
{
	memset_io(gk->dram + GNSS_IPC_OFFSET, 0, GNSS_IPC_SIZE);

	gk->tx_seq = 0;
	gk->tx_ch_seq = 0;
}

static void gnss_kepler_ipc_init(struct gnss_kepler *gk)
{
	void __iomem *ipc = gk->dram + GNSS_IPC_OFFSET;

	gk->rxq_buff = ipc;
	gk->rxq_size = GNSS_IPC_SIZE / 2;

	gk->txq_buff = ipc + gk->rxq_size;
	gk->txq_size = GNSS_IPC_SIZE / 2 - GNSS_IPC_REG_BYTES;

	gk->ipc_reg = ipc + GNSS_IPC_SIZE - GNSS_IPC_REG_BYTES;

	dev_info(gk->dev,
		 "ipc: rxq %u @ +0x%06x, txq %u @ +0x%06x, regs @ +0x%06x\n",
		 gk->rxq_size, GNSS_IPC_OFFSET, gk->txq_size,
		 GNSS_IPC_OFFSET + gk->rxq_size,
		 GNSS_IPC_OFFSET + GNSS_IPC_SIZE - (u32)GNSS_IPC_REG_BYTES);

	gnss_kepler_ipc_reset(gk);
}

/*
 * xmit_ipc_msg() + write_ipc_to_txq() + send_int2gnss(). One frame per
 * call, always a "single frame" (no fragmentation) -- the multi-frame
 * path exists downstream for payloads above the queue size, which
 * nothing here generates.
 */
static int gnss_kepler_ipc_send(struct gnss_kepler *gk, u8 ch,
				 const void *data, u32 len)
{
	u32 frame_len = EXYNOS_HEADER_SIZE + len;
	u32 pad = (4 - (frame_len & 0x3)) & 0x3;
	u32 total = frame_len + pad;
	u8 hdr[EXYNOS_HEADER_SIZE] = {};
	unsigned long flags;
	u32 qsize, in, out;
	void *frame;
	int ret = 0;

	if (frame_len > U16_MAX)
		return -EMSGSIZE;

	frame = kzalloc(total, GFP_KERNEL);
	if (!frame)
		return -ENOMEM;

	put_unaligned_le16(EXYNOS_START_MAGIC, hdr + EXYNOS_START_OFFSET);
	put_unaligned_le16(gk->tx_seq, hdr + EXYNOS_SEQ_OFFSET);
	put_unaligned_le16(EXYNOS_SINGLE_MASK << 8, hdr + EXYNOS_FRAG_OFFSET);
	put_unaligned_le16(frame_len, hdr + EXYNOS_LEN_OFFSET);
	hdr[EXYNOS_CH_ID_OFFSET] = ch;
	hdr[EXYNOS_CH_SEQ_OFFSET] = gk->tx_ch_seq;

	memcpy(frame, hdr, EXYNOS_HEADER_SIZE);
	memcpy(frame + EXYNOS_HEADER_SIZE, data, len);

	spin_lock_irqsave(&gk->tx_lock, flags);

	qsize = gk->txq_size;
	in = ipc_reg_read(gk, GNSS_IPC_REG_TX_HEAD);
	out = ipc_reg_read(gk, GNSS_IPC_REG_TX_TAIL);

	/* circ_valid(): a pointer past the end means the ring is garbage. */
	if (in >= qsize || out >= qsize) {
		dev_err(gk->dev, "ipc: TXQ invalid {qsize %u in %u out %u}\n",
			qsize, in, out);
		ret = -EIO;
		goto unlock;
	}

	if (circ_space(qsize, in, out) < total) {
		dev_err(gk->dev, "ipc: TXQ full {space %u need %u}\n",
			circ_space(qsize, in, out), total);
		ret = -ENOSPC;
		goto unlock;
	}

	circ_write16(gk->txq_buff, frame, qsize, in, total);
	ipc_reg_write(gk, GNSS_IPC_REG_TX_HEAD, circ_advance(qsize, in, total));

	gk->tx_seq++;
	gk->tx_ch_seq++;

	/* send_int2gnss(): the mask goes in a register, then the doorbell. */
	ipc_reg_write(gk, GNSS_IPC_REG_TX_MSG, GNSS_IPC_TX_MSG_MASK);
	writel(BIT(GNSS_MBOX_INT_AP2GNSS_IPC_MSG), gk->mbox + MCU_IPC_INTGR1);

unlock:
	spin_unlock_irqrestore(&gk->tx_lock, flags);
	kfree(frame);

	if (!ret)
		dev_info(gk->dev,
			 "ipc: sent ch %u, %u bytes (frame %u + pad %u), head %u -> %u\n",
			 ch, len, frame_len, pad, in,
			 circ_advance(qsize, in, total));

	return ret;
}

static void gnss_kepler_ipc_log_rx(struct gnss_kepler *gk, const u8 *frame,
				    u32 len)
{
	unsigned long flags;
	unsigned int slot;

	spin_lock_irqsave(&gk->ipc_rx_log_lock, flags);

	slot = gk->ipc_rx_log_in % GNSS_IPC_RX_LOG_FRAMES;
	gk->ipc_rx_log[slot].len = min_t(u32, len, GNSS_IPC_RX_LOG_BYTES);
	memcpy(gk->ipc_rx_log[slot].data, frame, gk->ipc_rx_log[slot].len);
	gk->ipc_rx_log_in++;

	wake_up_interruptible(&gk->ipc_rx_wait);

	spin_unlock_irqrestore(&gk->ipc_rx_log_lock, flags);
}

/*
 * rx_ipc_frames() + msg_handler(). Walks whole frames out of the RXQ,
 * validating the 0xabcd start of each; on a bad frame downstream gives
 * up on the batch entirely (-EBADMSG, tail left alone) rather than
 * trying to resynchronise, and so do we -- a desynchronised ring is a
 * bug to see in the log, not something to paper over.
 */
static void gnss_kepler_ipc_rx(struct gnss_kepler *gk)
{
	u32 qsize = gk->rxq_size;
	u32 in, out, rest;

	in = ipc_reg_read(gk, GNSS_IPC_REG_RX_HEAD);
	out = ipc_reg_read(gk, GNSS_IPC_REG_RX_TAIL);

	if (in == out)
		return;

	if (in >= qsize || out >= qsize) {
		dev_err(gk->dev, "ipc: RXQ invalid {qsize %u in %u out %u}\n",
			qsize, in, out);
		return;
	}

	rest = circ_usage(qsize, in, out);
	dev_info(gk->dev, "ipc: RX {in %u out %u, %u bytes}\n", in, out, rest);

	while (rest > 0) {
		u8 hdr[EXYNOS_HEADER_SIZE];
		u32 frame_len, total;
		u8 *frame;

		if (rest < EXYNOS_HEADER_SIZE) {
			dev_err(gk->dev, "ipc: RX runt, %u bytes left\n", rest);
			gk->ipc_rx_bad++;
			return;
		}

		circ_read16(hdr, gk->rxq_buff, qsize, out, EXYNOS_HEADER_SIZE);

		if (get_unaligned_le16(hdr + EXYNOS_START_OFFSET) !=
		    EXYNOS_START_MAGIC) {
			dev_err(gk->dev,
				"ipc: RX bad start 0x%04x at out %u -- giving up on batch\n",
				get_unaligned_le16(hdr + EXYNOS_START_OFFSET),
				out);
			gk->ipc_rx_bad++;
			return;
		}

		frame_len = get_unaligned_le16(hdr + EXYNOS_LEN_OFFSET);
		total = frame_len + ((4 - (frame_len & 0x3)) & 0x3);

		if (frame_len < EXYNOS_HEADER_SIZE || total > rest) {
			dev_err(gk->dev,
				"ipc: RX bad len %u (total %u, rest %u)\n",
				frame_len, total, rest);
			gk->ipc_rx_bad++;
			return;
		}

		frame = kmalloc(total, GFP_KERNEL);
		if (!frame) {
			/* Leave the tail here so the frame is retried. */
			ipc_reg_write(gk, GNSS_IPC_REG_RX_TAIL, out);
			return;
		}

		circ_read16(frame, gk->rxq_buff, qsize, out, total);

		dev_info(gk->dev,
			 "ipc: RX frame ch %u seq %u len %u\n",
			 frame[EXYNOS_CH_ID_OFFSET],
			 get_unaligned_le16(frame + EXYNOS_SEQ_OFFSET),
			 frame_len);

		/*
		 * frame_len, not total: `total` includes the padding that
		 * rounds the frame up to 4 bytes, and that padding is not part
		 * of the message. Handing it up made read() return payload +
		 * pad, which the daemon's byte-stream parser then had to skip
		 * -- "BETP Helper: Discarded 1/2/3 bytes before frame", once
		 * per frame, exactly the range a 4-byte pad produces.
		 */
		gnss_kepler_ipc_log_rx(gk, frame, frame_len);
		gk->ipc_rx_frames++;
		kfree(frame);

		out = circ_advance(qsize, out, total);
		rest -= total;
	}

	/* Emptied the queue: tail catches up with the head we sampled. */
	ipc_reg_write(gk, GNSS_IPC_REG_RX_TAIL, in);
}

static void gnss_kepler_ipc_rx_work(struct work_struct *work)
{
	struct gnss_kepler *gk = container_of(work, struct gnss_kepler,
					       ipc_rx_work);

	gnss_kepler_ipc_rx(gk);
}

/*
 * One read32 BCMD, no logging -- the bulk-dump primitive behind the
 * kmem file below. Kept separate from gnss_kepler_send_bcmd() because
 * that one exists to narrate a single command to the log, which is
 * useless at 32768 reads per ROM dump.
 *
 * Returns -EIO if the core does not answer, which is also how "not in
 * the ROM command loop" and "unmapped address" present themselves.
 */
static int gnss_kepler_read32(struct gnss_kepler *gk, u32 addr, u32 *out)
{
	u32 val;
	int ret;

	writel(GNSS_BCMD_READ32, gk->mbox + GNSS_MBOX_BCMD_CTRL0);
	writel(addr, gk->mbox + GNSS_MBOX_BCMD_CTRL1);
	writel(0, gk->mbox + GNSS_MBOX_BCMD_CTRL2);
	writel(GNSS_MBOX_BCMD_CTRL3_PENDING, gk->mbox + GNSS_MBOX_BCMD_CTRL3);

	writel(BIT(GNSS_MBOX_INT_AP2GNSS_BCMD), gk->mbox + MCU_IPC_INTGR1);

	ret = readl_poll_timeout(gk->mbox + GNSS_MBOX_BCMD_CTRL3, val,
				 val != GNSS_MBOX_BCMD_CTRL3_PENDING,
				 REQ_BCMD_POLL_INTERVAL,
				 REQ_BCMD_TIMEOUT * USEC_PER_MSEC);
	if (ret)
		return -EIO;

	*out = val;

	return 0;
}

/*
 * Kepler's own address space, read a word at a time over BCMD, exposed
 * so the on-die mask ROM can be dumped and disassembled -- it appears
 * in no file on the device, yet the firmware calls into it constantly
 * and its addresses are what the fault records point at.
 *
 * The file offset *is* the Kepler address, so:
 *
 *   echo 1 > start; echo 1 > reset
 *   echo "0x05 0 0 0" > bcmd     # stage a read32 to enter the ROM loop
 *   echo 1 > release
 *   dd if=kmem bs=4096 count=32 > rom.bin
 *
 * Staging that read32 before release_reset is what puts the ROM into
 * its command-service loop; without it nothing answers and every read
 * here returns -EIO. Note a word whose true value is 0x000000ff is
 * indistinguishable from "no answer" -- the sentinel is the core's own
 * protocol, not something we can pick around.
 */
static ssize_t kmem_read(struct file *file, char __user *buf, size_t count,
			  loff_t *ppos)
{
	struct gnss_kepler *gk = file->private_data;
	u32 addr = *ppos & ~3U;
	size_t done = 0;
	int ret;

	while (done + sizeof(u32) <= count) {
		u32 val;

		ret = gnss_kepler_read32(gk, addr + done, &val);
		if (ret)
			return done ? done : ret;

		if (copy_to_user(buf + done, &val, sizeof(val)))
			return -EFAULT;

		done += sizeof(val);
		cond_resched();
	}

	*ppos = addr + done;

	return done;
}

static int ipc_regs_show(struct seq_file *s, void *unused)
{
	struct gnss_kepler *gk = s->private;
	u32 th = ipc_reg_read(gk, GNSS_IPC_REG_TX_HEAD);
	u32 tt = ipc_reg_read(gk, GNSS_IPC_REG_TX_TAIL);
	u32 rh = ipc_reg_read(gk, GNSS_IPC_REG_RX_HEAD);
	u32 rt = ipc_reg_read(gk, GNSS_IPC_REG_RX_TAIL);
	unsigned int i;

	seq_printf(s, "TXQ  size %u  head %u  tail %u  space %u  usage %u\n",
		   gk->txq_size, th, tt, circ_space(gk->txq_size, th, tt),
		   circ_usage(gk->txq_size, th, tt));
	seq_printf(s, "RXQ  size %u  head %u  tail %u  space %u  usage %u\n",
		   gk->rxq_size, rh, rt, circ_space(gk->rxq_size, rh, rt),
		   circ_usage(gk->rxq_size, rh, rt));
	seq_printf(s, "tx_ipc_msg 0x%08x  rx_ipc_msg 0x%08x\n",
		   ipc_reg_read(gk, GNSS_IPC_REG_TX_MSG),
		   ipc_reg_read(gk, GNSS_IPC_REG_RX_MSG));
	seq_printf(s, "rx frames %u  bad %u  tx_seq %u\n",
		   gk->ipc_rx_frames, gk->ipc_rx_bad, gk->tx_seq);

	seq_puts(s, "raw regs:");
	for (i = 0; i < GNSS_IPC_REG_CNT; i++) {
		if (i % 8 == 0)
			seq_printf(s, "\n  [%2u]", i);
		seq_printf(s, " %08x", ipc_reg_read(gk, i));
	}
	seq_puts(s, "\n");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ipc_regs);

/*
 * echo "<ch> <payload>" > ipc_tx -- everything after the first space is
 * sent verbatim as the frame payload on channel <ch>.
 */
static ssize_t ipc_tx_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct gnss_kepler *gk = file->private_data;
	size_t consumed = count;
	char *kbuf, *payload;
	unsigned int ch;
	int ret;

	kbuf = memdup_user_nul(buf, count);
	if (IS_ERR(kbuf))
		return PTR_ERR(kbuf);

	payload = strchr(kbuf, ' ');
	if (!payload) {
		ret = -EINVAL;
		goto out;
	}
	*payload++ = '\0';

	ret = kstrtouint(kbuf, 0, &ch);
	if (ret)
		goto out;
	if (ch > U8_MAX) {
		ret = -EINVAL;
		goto out;
	}

	/* Drop one trailing newline from echo, keep everything else. */
	count = strlen(payload);
	if (count && payload[count - 1] == '\n')
		count--;

	ret = gnss_kepler_ipc_send(gk, ch, payload, count);
out:
	kfree(kbuf);

	/*
	 * Report the whole write as consumed, not the payload length --
	 * a short count makes the shell retry with the remainder, which
	 * then fails to parse as "<ch> <payload>".
	 */
	return ret ? ret : (ssize_t)consumed;
}

static const struct file_operations ipc_tx_fops = {
	.open = simple_open,
	.write = ipc_tx_write,
	.llseek = noop_llseek,
};

static int ipc_rx_show(struct seq_file *s, void *unused)
{
	struct gnss_kepler *gk = s->private;
	unsigned long flags;
	unsigned int n, i;

	spin_lock_irqsave(&gk->ipc_rx_log_lock, flags);

	n = min_t(unsigned int, gk->ipc_rx_log_in, GNSS_IPC_RX_LOG_FRAMES);
	seq_printf(s, "%u frames received (%u bad), showing last %u\n",
		   gk->ipc_rx_frames, gk->ipc_rx_bad, n);

	for (i = 0; i < n; i++) {
		unsigned int slot = (gk->ipc_rx_log_in - n + i) %
				    GNSS_IPC_RX_LOG_FRAMES;
		u32 len = gk->ipc_rx_log[slot].len;
		const u8 *p = gk->ipc_rx_log[slot].data;
		u32 off;

		seq_printf(s, "\nframe %u, %u bytes%s\n",
			   gk->ipc_rx_log_in - n + i, len,
			   len == GNSS_IPC_RX_LOG_BYTES ? " (truncated)" : "");

		for (off = 0; off < len; off += 16) {
			u32 j, end = min(off + 16, len);

			seq_printf(s, "  %04x: ", off);
			for (j = off; j < end; j++)
				seq_printf(s, "%02x ", p[j]);
			for (; j < off + 16; j++)
				seq_puts(s, "   ");
			seq_puts(s, " |");
			for (j = off; j < end; j++)
				seq_putc(s, isprint(p[j]) ? p[j] : '.');
			seq_puts(s, "|\n");
		}
	}

	spin_unlock_irqrestore(&gk->ipc_rx_log_lock, flags);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ipc_rx);

/*
 * Test hook, not part of the protocol: moves whatever is pending in the
 * TXQ into the RXQ and runs the receive path over it. The firmware
 * asserts before it ever drives the RXQ (see gnss.md,
 * "ScheduledPortInQueue"), so without this the RX parser would ship
 * having never executed. Exercises circ_read16(), the frame walk and
 * the tail update against frames this driver itself framed.
 */
static ssize_t ipc_loopback_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct gnss_kepler *gk = file->private_data;
	u32 th, tt, rh, len;
	void *tmp;

	th = ipc_reg_read(gk, GNSS_IPC_REG_TX_HEAD);
	tt = ipc_reg_read(gk, GNSS_IPC_REG_TX_TAIL);
	rh = ipc_reg_read(gk, GNSS_IPC_REG_RX_HEAD);

	if (th >= gk->txq_size || tt >= gk->txq_size || rh >= gk->rxq_size)
		return -EIO;

	len = circ_usage(gk->txq_size, th, tt);
	if (!len)
		return -ENODATA;

	if (circ_space(gk->rxq_size, rh, ipc_reg_read(gk, GNSS_IPC_REG_RX_TAIL))
	    < len)
		return -ENOSPC;

	tmp = kmalloc(len, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	circ_read16(tmp, gk->txq_buff, gk->txq_size, tt, len);
	circ_write16(gk->rxq_buff, tmp, gk->rxq_size, rh, len);
	kfree(tmp);

	/* Consume from the TXQ and publish into the RXQ. */
	ipc_reg_write(gk, GNSS_IPC_REG_TX_TAIL, th);
	ipc_reg_write(gk, GNSS_IPC_REG_RX_HEAD,
		      circ_advance(gk->rxq_size, rh, len));

	dev_info(gk->dev, "ipc_loopback: moved %u bytes TXQ -> RXQ\n", len);
	gnss_kepler_ipc_rx(gk);

	return count;
}

static const struct file_operations ipc_loopback_fops = {
	.open = simple_open,
	.write = ipc_loopback_write,
	.llseek = noop_llseek,
};

/*
 * echo "<pmu-offset> <value>" > win -- program one of the GNSS2AP access
 * windows by hand.
 *
 * gpsd runs the firmware *in place* at Kepler 0x60000000 (its RUN param1
 * decodes to patch_addr + 0x60000001, and our K148 record carries
 * addr 0) -- see gnss.md. This driver instead lets the ROM autorun copy
 * the image to internal RAM 0x00200000 and branches there, so the code
 * runs but its absolute data references still point into 0x6xxxxxxx,
 * which reads as 0 because no window maps the reserved region there.
 * These three registers are written 0 verbatim from downstream and their
 * encoding is undocumented, so this exists to try candidate values.
 *
 * Restricted to the three window registers: a stray write elsewhere in
 * PMU_ALIVE is how this board gets taken down hard.
 */
static ssize_t win_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *ppos)
{
	struct gnss_kepler *gk = file->private_data;
	u32 offset, val, readback;
	char kbuf[64];
	int ret;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;
	kbuf[count] = '\0';

	if (sscanf(kbuf, "%i %i", &offset, &val) != 2)
		return -EINVAL;

	if (offset != GNSS2AP_MIF_ACCESS_WIN0 &&
	    offset != GNSS2AP_PERI_ACCESS_WIN0 &&
	    offset != GNSS2AP_PERI_ACCESS_WIN1) {
		dev_err(gk->dev, "win: refusing to write PMU offset 0x%04x\n",
			offset);
		return -EPERM;
	}

	ret = regmap_write(gk->pmureg, offset, val);
	if (ret)
		return ret;

	regmap_read(gk->pmureg, offset, &readback);
	dev_info(gk->dev, "win: 0x%04x <- 0x%08x (read back 0x%08x)\n",
		 offset, val, readback);

	return count;
}

static const struct file_operations win_fops = {
	.open = simple_open,
	.write = win_write,
	.llseek = noop_llseek,
};

static ssize_t ipc_reset_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct gnss_kepler *gk = file->private_data;

	dev_info(gk->dev, "ipc_reset\n");
	gnss_kepler_ipc_reset(gk);

	return count;
}

static const struct file_operations ipc_reset_fops = {
	.open = simple_open,
	.write = ipc_reset_write,
	.llseek = noop_llseek,
};

static const struct file_operations kmem_fops = {
	.open = simple_open,
	.read = kmem_read,
	.llseek = default_llseek,
};

/*
 * kepler_req_fault_info() downstream (gnss_keplerctl_device.c:275).
 * Rings int 1, waits a full second, then reads ISSR3 -- which the core
 * writes with the fault payload size. Downstream reads ISSR3 without
 * pre-seeding it; this version writes the same 0xff sentinel send_bcmd
 * uses first, so a leftover value cannot be mistaken for an answer.
 */
static void gnss_kepler_req_fault_info(struct gnss_kepler *gk)
{
	u32 size;

	dev_info(gk->dev, "req_fault_info: ringing int %d, waiting %d ms\n",
		 GNSS_MBOX_INT_AP2GNSS_REQ_FAULT_INFO, REQ_FAULT_INFO_TIMEOUT);

	reinit_completion(&gk->fault_cmpl);
	writel(GNSS_MBOX_BCMD_CTRL3_PENDING, gk->mbox + GNSS_MBOX_BCMD_CTRL3);

	writel(BIT(GNSS_MBOX_INT_AP2GNSS_REQ_FAULT_INFO),
	       gk->mbox + MCU_IPC_INTGR1);

	if (!wait_for_completion_timeout(&gk->fault_cmpl,
					  msecs_to_jiffies(REQ_FAULT_INFO_TIMEOUT))) {
		dev_err(gk->dev,
			"req_fault_info TIMEOUT (ISSR3 still 0x%08x)\n",
			readl(gk->mbox + GNSS_MBOX_BCMD_CTRL3));
		return;
	}

	size = readl(gk->mbox + GNSS_MBOX_BCMD_CTRL3);
	dev_info(gk->dev,
		 "req_fault_info: CORE ANSWERED, ISSR3=0x%08x (%u bytes)\n",
		 size, size);
}

static ssize_t fault_req_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	gnss_kepler_req_fault_info(file->private_data);

	return count;
}

static const struct file_operations fault_req_fops = {
	.open = simple_open,
	.write = fault_req_write,
	.llseek = noop_llseek,
};

static int gnss_kepler_wait_bits(struct gnss_kepler *gk, u32 offset,
				  u32 mask, u32 target, const char *what)
{
	u32 val = 0;
	int i;

	for (i = 0; i < 200; i++) {
		regmap_read(gk->pmureg, offset, &val);
		if ((val & mask) == target) {
			dev_info(gk->dev, "%s done after %d us (0x%08x)\n",
				 what, i * 50, val);
			return 0;
		}
		usleep_range(50, 60);
	}

	dev_err(gk->dev, "timeout waiting for %s (last read 0x%08x)\n",
		what, val);
	return -ETIMEDOUT;
}

/*
 * gnss_init[] from the downstream cal-if reference sequence
 * (pmucal_gnss_exynos7885.h, dead code there under
 * CONFIG_GNSS_PMUCAL=n): PWRON, START, then a hard wait for
 * RESET_SEQUENCER_STATUS before anything else touches the domain.
 * Deliberately not implementing gnss_pmu_release_reset()'s longer
 * sequence here -- milestone 1 only needs one clean power-on per module
 * load, matching the very first RESET+BCMD cycle after a downstream
 * reboot (the state that reaches this exact function there).
 */
/*
 * gnss_pmu_init_conf() downstream. Must run before the first power_on:
 * MEM_CONFIG1 is where the core gets the base of its shared DRAM, so
 * without this it fetches from whatever the register happened to hold
 * and never executes -- no fault record, no BCMD reply, but a perfectly
 * normal-looking ACTIVE interrupt, since that is sequencer-level and
 * does not depend on the core running correct code.
 *
 * Both fields sit at bit 0 (MEMSIZE_OFFSET/MEMBASE_ADDR_OFFSET are 0
 * downstream), so there is no shift beyond the /4K and >>12 scaling.
 * The three ACCESS_WIN registers are written 0x0 verbatim from
 * downstream, whose entire documentation for them is the comment "set
 * access window for GNSS" -- ported rather than guessed at.
 */
static void gnss_kepler_init_conf(struct gnss_kepler *gk)
{
	u32 memsz = gk->mem_size / GNSS_MEMSIZE_RES;
	u32 base = gk->mem_base >> GNSS_MEMBASE_ADDR_SHIFT;
	u32 rb_size, rb_base;

	/*
	 * Downstream masks without checking. Silently truncating either
	 * value is precisely the failure this function exists to fix, so
	 * say so rather than write a plausible-looking wrong address.
	 */
	if (memsz & ~GNSS_MEMSIZE_MASK || base & ~GNSS_MEMBASE_ADDR_MASK) {
		dev_err(gk->dev,
			"init_conf: region %pa+%zx does not fit the MEM_CONFIG fields (size0=0x%x base0=0x%x) -- not programming the window\n",
			&gk->mem_base, gk->mem_size, memsz, base);
		return;
	}

	regmap_update_bits(gk->pmureg, GNSS2AP_MEM_CONFIG0,
			    GNSS_MEMSIZE_MASK, memsz);
	regmap_update_bits(gk->pmureg, GNSS2AP_MEM_CONFIG1,
			    GNSS_MEMBASE_ADDR_MASK, base);

	regmap_write(gk->pmureg, GNSS2AP_MIF_ACCESS_WIN0, 0);
	regmap_write(gk->pmureg, GNSS2AP_PERI_ACCESS_WIN0, 0);
	regmap_write(gk->pmureg, GNSS2AP_PERI_ACCESS_WIN1, 0);

	/*
	 * Read back: this register was found holding a stale non-zero base
	 * on this board, so "we wrote it" is worth less than "it now reads
	 * what we wrote".
	 */
	regmap_read(gk->pmureg, GNSS2AP_MEM_CONFIG0, &rb_size);
	regmap_read(gk->pmureg, GNSS2AP_MEM_CONFIG1, &rb_base);

	dev_info(gk->dev,
		 "init_conf: shared mem %pa+%zx -> size0=0x%08x base0=0x%08x (read back 0x%08x/0x%08x)\n",
		 &gk->mem_base, gk->mem_size, memsz, base, rb_size, rb_base);

	if ((rb_size & GNSS_MEMSIZE_MASK) != memsz ||
	    (rb_base & GNSS_MEMBASE_ADDR_MASK) != base)
		dev_err(gk->dev, "init_conf: window did not take\n");
}

static int gnss_kepler_power_on(struct gnss_kepler *gk)
{
	dev_info(gk->dev, "power_on\n");

	regmap_update_bits(gk->pmureg, GNSS_CTRL_NS, GNSS_PWRON, GNSS_PWRON);

	if (gk->tcxo_sw) {
		regmap_update_bits(gk->pmureg, GNSS_CTRL_NS,
				    GNSS_TCXO_ENABLE_SW, GNSS_TCXO_ENABLE_SW);
		dev_info(gk->dev, "power_on: TCXO_ENABLE_SW set\n");
	}
	regmap_update_bits(gk->pmureg, GNSS_CTRL_S, GNSS_START, GNSS_START);

	return gnss_kepler_wait_bits(gk, RESET_SEQUENCER_STATUS,
				      GNSS_RESET_SEQ_STAT_MASK,
				      GNSS_RESET_SEQ_STAT_DONE,
				      "RESET_SEQUENCER_STATUS");
}

/*
 * Ask the core what it is, so a test script can assert the state it
 * expects instead of assuming it. Intrusive by nature: it sends a real
 * BCMD (0x21, ROM-SW version -- no side effects, unlike every other
 * command we have), with a short timeout so probing a running core is
 * cheap.
 *
 * The three outcomes are distinguishable because ISSR3 is not writable
 * while the core is held: the sentinel not sticking *is* the held
 * indication, and is checked before the doorbell is rung.
 */
#define GNSS_BCMD_ROM_SW_VERSION	0x21
#define BCMD_PROBE_TIMEOUT		50	/* ms */

static int gnss_kepler_bcmd_probe(struct gnss_kepler *gk, u32 *out)
{
	u32 val;
	int ret;

	writel(GNSS_BCMD_ROM_SW_VERSION, gk->mbox + GNSS_MBOX_BCMD_CTRL0);
	writel(0, gk->mbox + GNSS_MBOX_BCMD_CTRL1);
	writel(0, gk->mbox + GNSS_MBOX_BCMD_CTRL2);
	writel(GNSS_MBOX_BCMD_CTRL3_PENDING, gk->mbox + GNSS_MBOX_BCMD_CTRL3);

	if (readl(gk->mbox + GNSS_MBOX_BCMD_CTRL3) !=
	    GNSS_MBOX_BCMD_CTRL3_PENDING)
		return -EBUSY;

	writel(BIT(GNSS_MBOX_INT_AP2GNSS_BCMD), gk->mbox + MCU_IPC_INTGR1);

	ret = readl_poll_timeout(gk->mbox + GNSS_MBOX_BCMD_CTRL3, val,
				 val != GNSS_MBOX_BCMD_CTRL3_PENDING,
				 REQ_BCMD_POLL_INTERVAL,
				 BCMD_PROBE_TIMEOUT * USEC_PER_MSEC);
	if (ret)
		return -ETIMEDOUT;

	*out = val;

	return 0;
}

/*
 * The counterpart to power_on, which downstream has no equivalent of --
 * it never needs one, because on stock the core is always in a state
 * that answers hold_reset.
 *
 * Measured 2026-08-13: hold_reset does *not* stop a core that is
 * executing firmware. Its powerdown path ends at
 * CENTRAL_SEQ_GNSS_CONFIGURATION = 0, which asks the sequencer to take
 * the domain down and waits on the core to quiesce; a running firmware
 * never does, so the domain stays up and the core keeps servicing
 * mailbox interrupts. That produced hours of A/B tests where "the core
 * answered while held" and results carried over between runs -- see the
 * carry-over note in gnss.md.
 *
 * Clearing PWRON is the only thing that reliably ends a cycle. Order
 * matters: hold_reset first so the sequencer registers are already
 * staged, then drop START and PWRON.
 */
static void gnss_kepler_shared_reg_disable(struct gnss_kepler *gk);

static void gnss_kepler_power_off(struct gnss_kepler *gk)
{
	unsigned int stat;

	dev_info(gk->dev, "power_off\n");

	regmap_update_bits(gk->pmureg, GNSS_CTRL_S, GNSS_START, 0);
	regmap_update_bits(gk->pmureg, GNSS_CTRL_NS, GNSS_PWRON, 0);

	/*
	 * The domain needs real time down for this to be worth anything.
	 * What has to clear is the EXE-address register (0x80043004): once
	 * a codeload with flags 4 (or a cmd 2) has set it, the ROM jumps
	 * there at every subsequent power-on, so the core auto-starts the
	 * firmware and never enters its command loop -- which is what made
	 * results carry between runs.
	 *
	 * Measured: 1 ms is not enough and leaves the register armed; 200
	 * ms, 1 s and 3 s all come back to a clean ROM command loop
	 * (verified with cmd 0x21 answering after a release). 300 ms for
	 * margin -- the true threshold is somewhere in 1..200 ms and has
	 * not been narrowed further.
	 */
	msleep(gk->power_off_ms);

	regmap_read(gk->pmureg, GNSS_STAT, &stat);
	dev_info(gk->dev, "power_off: GNSS_STAT=0x%08x\n", stat);

	gnss_kepler_shared_reg_disable(gk);
}

/*
 * gnss_pmu_hold_reset() downstream (exynos_sys_powerdown_conf_gnss()
 * + GNSS_RESET_SET). Needed because PMU_ALIVE genuinely is the
 * always-on domain: its state (confirmed via GNSS_CTRL_NS's PWRON bit)
 * survives a plain `reboot` of this device, so without an explicit
 * reset step every test after the first one runs against a domain
 * that's already up rather than a comparable fresh state -- found
 * when a second `start` read back "RESET_SEQUENCER_STATUS done after
 * 0 us" and produced no ACTIVE interrupt at all. See gnss.md.
 */
static void gnss_kepler_hold_reset(struct gnss_kepler *gk)
{
	dev_info(gk->dev, "hold_reset\n");

	regmap_write(gk->pmureg, RESET_AHEAD_GNSS_SYS_PWR_REG, 0);
	regmap_write(gk->pmureg, CLEANY_BUS_SYS_PWR_REG, 0);
	regmap_write(gk->pmureg, LOGIC_RESET_GNSS_SYS_PWR_REG, 0);
	regmap_write(gk->pmureg, TCXO_GATE_GNSS_SYS_PWR_REG, 0);
	regmap_write(gk->pmureg, GNSS_DISABLE_ISO_SYS_PWR_REG, 1);
	regmap_write(gk->pmureg, GNSS_RESET_ISO_SYS_PWR_REG, 0);
	regmap_write(gk->pmureg, CENTRAL_SEQ_GNSS_CONFIGURATION, 0);

	regmap_update_bits(gk->pmureg, GNSS_CTRL_NS, GNSS_RESET_SET,
			    GNSS_RESET_SET);

	usleep_range(80, 100);

	/*
	 * kepler_hold_reset() downstream calls mbox_sw_reset() right
	 * after the PMU hold_reset -- resets the mailbox IP itself
	 * (MCUCTLR's MSWRST bit), separate from the GNSS PMU domain
	 * reset above.
	 */
	dev_info(gk->dev, "mbox_sw_reset\n");
	writel(readl(gk->mbox + MCU_IPC_MCUCTLR) | MCU_IPC_MSWRST,
	       gk->mbox + MCU_IPC_MCUCTLR);
	udelay(5);
}

/*
 * exynos_pmu_shared_reg_enable()/_disable() (mcu_ipc.c downstream),
 * called directly from gnss_pmu_release_reset(). The CP mailbox driver
 * provides it as a regulator, so the reference count downstream keeps
 * across GNSS/CP/WLBT/FM-radio callers (shared_reg_usage_cnt) is the
 * regulator core's. The real function also computes a need_delay from
 * the CP mailbox's ISSR3 and stalls 3ms on the *next* CP doorbell ring
 * if set -- gnss_pmu_release_reset() discards that return value, so it's
 * not reproduced here.
 */
static void gnss_kepler_shared_reg_enable(struct gnss_kepler *gk)
{
	u32 val;
	int ret = 0;

	mutex_lock(&gk->power_lock);
	if (!gk->shared_reg_on) {
		ret = regulator_enable(gk->shared_reg);
		gk->shared_reg_on = !ret;
	}
	mutex_unlock(&gk->power_lock);

	regmap_read(gk->pmureg, EXT_REGULATOR_SHARED_STATUS, &val);
	dev_info(gk->dev, "shared_reg_enable: %d, EXT_REGULATOR_SHARED_STATUS=0x%08x\n",
		 ret, val);
}

static void gnss_kepler_shared_reg_disable(struct gnss_kepler *gk)
{
	mutex_lock(&gk->power_lock);
	if (gk->shared_reg_on) {
		regulator_disable(gk->shared_reg);
		gk->shared_reg_on = false;
	}
	mutex_unlock(&gk->power_lock);
}

/*
 * The core needs the CP running: with the CP down its first RF access
 * fails (SPDY RD FAIL, kepler.md 13.9 and 14.22). Hold the modem's power
 * reference from the first power-up until the domain is switched off for
 * good (debugfs start=0, or unbinding the driver), so a remoteproc "stop"
 * only drops its own reference and leaves the CP up. The reset cycles
 * gpsd drives (RESET, then REQ_BCMD) keep it: they are not "GNSS off".
 *
 * This only joins a CP that is already running and never boots one:
 * the CP cannot be restarted on the same boot yet, so booting it here
 * and dropping it at power_off would lose it for the rest of the boot.
 * For experiments without the CP:
 *     echo 0 > /sys/kernel/debug/gnss_kepler/need_modem
 */
static int gnss_kepler_modem_hold(struct gnss_kepler *gk)
{
	int ret = 0;

	if (!gk->modem || !gk->need_modem)
		return 0;

	mutex_lock(&gk->power_lock);
	if (gk->modem_held)
		goto out;

	if (READ_ONCE(gk->modem->state) != RPROC_RUNNING) {
		dev_err(gk->dev, "the modem (%s) is not running, start it first\n",
			gk->modem->name);
		ret = -ENOLINK;
		goto out;
	}

	/* already running, so this only takes a power reference */
	ret = rproc_boot(gk->modem);
	gk->modem_held = !ret;
out:
	mutex_unlock(&gk->power_lock);

	return ret;
}

static void gnss_kepler_modem_release(struct gnss_kepler *gk)
{
	mutex_lock(&gk->power_lock);
	if (gk->modem_held) {
		rproc_shutdown(gk->modem);
		gk->modem_held = false;
	}
	mutex_unlock(&gk->power_lock);
}

/*
 * gnss_reset_release[] downstream: the sequence that actually recovers
 * the domain after hold_reset(), as opposed to gnss_kepler_power_on()
 * (gnss_init[]) which only takes it from genuinely OFFLINE. Confirmed
 * necessary here too: without this, CENTRAL_SEQ_GNSS_STATUS stayed at
 * states=0x80 ("powered down") after a hold_reset+power_on cycle, and
 * no ACTIVE interrupt ever arrived. See gnss.md.
 */
static int gnss_kepler_release_reset(struct gnss_kepler *gk)
{
	int ret;

	dev_info(gk->dev, "release_reset\n");

	/*
	 * ld->reset_buffers(ld) -- kepler_req_bcmd() calls this immediately
	 * before kepler_release_reset() whenever the core is in
	 * STATE_HOLD_RESET, so the rings are clean every time the core
	 * boots. Only touches GNSS_IPC_OFFSET upwards, so a firmware image
	 * already staged at dram+0x10 survives.
	 */
	gnss_kepler_ipc_reset(gk);

	/*
	 * kepler_release_reset() downstream clears all pending mailbox
	 * interrupts (mcu_ipc_clear_all_interrupt(), INTCR1=0xffff)
	 * before touching the PMU.
	 */
	writel(0xffff, gk->mbox + MCU_IPC_INTCR1);

	gnss_kepler_shared_reg_enable(gk);

	if (gk->tcxo_sw) {
		regmap_update_bits(gk->pmureg, GNSS_CTRL_NS,
				    GNSS_TCXO_ENABLE_SW, GNSS_TCXO_ENABLE_SW);
		dev_info(gk->dev, "release_reset: TCXO_ENABLE_SW set\n");
	}

	regmap_update_bits(gk->pmureg, GNSS_CTRL_NS, SWEEPER_BYPASS_DATA_EN,
			    SWEEPER_BYPASS_DATA_EN);
	regmap_update_bits(gk->pmureg, GNSS_CTRL_NS, GNSS_PWRON, GNSS_PWRON);
	regmap_update_bits(gk->pmureg, GNSS_CTRL_S, GNSS_START, GNSS_START);

	ret = gnss_kepler_wait_bits(gk, CENTRAL_SEQ_GNSS_STATUS,
				     CENTRAL_SEQ_GNSS_STATES_MASK,
				     CENTRAL_SEQ_GNSS_STATES_RUN,
				     "CENTRAL_SEQ_GNSS_STATUS");
	if (ret)
		goto release_reset_done;

	regmap_update_bits(gk->pmureg, CLEANY_BUS_GNSS_CONFIGURATION,
			    CLEANY_BUS_GNSS_REQ, CLEANY_BUS_GNSS_REQ);
	ret = gnss_kepler_wait_bits(gk, CLEANY_BUS_GNSS_STATUS,
				     CLEANY_BUS_GNSS_STATUS_MASK,
				     CLEANY_BUS_GNSS_STATUS_DONE,
				     "CLEANY_BUS_GNSS_STATUS");

release_reset_done:
	regmap_update_bits(gk->pmureg, GNSS_CTRL_NS, SWEEPER_BYPASS_DATA_EN, 0);

	/*
	 * Downstream's gnss_pmu_release_reset() ends with
	 * exynos_pmu_shared_reg_disable() unconditionally, and this port
	 * copied it. On stock Android that is harmless: the running CP
	 * holds the shared regulator on its own account, so GNSS dropping
	 * its reference changes nothing. Without CP firmware, GNSS is the
	 * only holder, and the TCXO goes away the instant this returns --
	 * microseconds after the core starts.
	 *
	 * Switchable rather than assumed; with it set, GNSS keeps its vote
	 * until power_off:
	 *
	 *     echo 1 > /sys/kernel/debug/gnss_kepler/hold_shared_reg
	 */
	if (!gk->hold_shared_reg)
		gnss_kepler_shared_reg_disable(gk);
	else
		dev_info(gk->dev, "release_reset: holding the shared regulator\n");

	return ret;
}

/*
 * One line per fact a test script needs before it trusts a measurement.
 * `core` is the interesting one; see gnss_kepler_bcmd_probe().
 */
static int state_show(struct seq_file *s, void *unused)
{
	struct gnss_kepler *gk = s->private;
	unsigned int ctrl, stat;
	u32 ver = 0;
	int ret;

	regmap_read(gk->pmureg, GNSS_CTRL_NS, &ctrl);
	regmap_read(gk->pmureg, GNSS_STAT, &stat);

	seq_printf(s, "power        %s\n", ctrl & GNSS_PWRON ? "on" : "off");
	seq_printf(s, "GNSS_CTRL_NS 0x%08x\n", ctrl);
	seq_printf(s, "GNSS_STAT    0x%08x\n", stat);

	ret = gnss_kepler_bcmd_probe(gk, &ver);
	if (ret == -EBUSY)
		seq_puts(s, "core         HELD (ISSR3 not writable)\n");
	else if (ret == -ETIMEDOUT)
		seq_puts(s, "core         SILENT (no reply -- executing firmware, or down)\n");
	else
		seq_printf(s, "core         ROM COMMAND LOOP (rom-sw 0x%08x)\n",
			   ver);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(state);

/*
 * ---------------------------------------------------------------------
 * /dev/gnss_ipc -- the ABI the vendor daemon speaks
 *
 * ioctl numbers and structs are downstream's (gnss_prj.h), independently
 * confirmed against gpsd's own call sites: it builds a
 * kepler_bcmd_args with cmd_id at +2 and reads the reply back from
 * ret_val at +0xc, and passes LOAD_FIRMWARE {size, offset, buf}. See
 * gnss.md, "gpsd's real boot sequence".
 * ---------------------------------------------------------------------
 */
#define GNSS_IOC_MAGIC			'K'
#define GNSS_IOCTL_RESET		_IO(GNSS_IOC_MAGIC, 0x00)
#define GNSS_IOCTL_LOAD_FIRMWARE	_IO(GNSS_IOC_MAGIC, 0x01)
#define GNSS_IOCTL_REQ_FAULT_INFO	_IO(GNSS_IOC_MAGIC, 0x02)
#define GNSS_IOCTL_REQ_BCMD		_IO(GNSS_IOC_MAGIC, 0x03)
#define GNSS_IOCTL_READ_FIRMWARE	_IO(GNSS_IOC_MAGIC, 0x04)
#define GNSS_IOCTL_CHANGE_SENSOR_GPIO	_IO(GNSS_IOC_MAGIC, 0x05)
#define GNSS_IOCTL_CHANGE_TCXO_MODE	_IO(GNSS_IOC_MAGIC, 0x06)
#define GNSS_IOCTL_SET_SENSOR_POWER	_IO(GNSS_IOC_MAGIC, 0x07)
#define GNSS_IOCTL_RELEASE_RESET	_IO(GNSS_IOC_MAGIC, 0x08)

struct kepler_bcmd_args {
	u16 flags;
	u16 cmd_id;
	u32 param1;
	u32 param2;
	u32 ret_val;
};

struct kepler_firmware_args {
	u32 firmware_size;
	u32 offset;
	char __user *firmware_bin;
};

struct kepler_fault_args {
	u32 dump_size;
	char __user *dumped_data;
};

/*
 * kepler_req_bcmd() downstream. Where the debugfs `bcmd` file only
 * stages and leaves sequencing to the caller, this drives the
 * power/reset transition itself off the tracked state, which is what
 * gpsd depends on: it issues RESET once and then expects every
 * REQ_BCMD to imply the release that lets the ROM read the command.
 *
 * Returns the reply value, or -EIO on timeout. Downstream returns the
 * reply as an int too, so a reply with bit 31 set would be
 * indistinguishable from an errno; kept as-is for ABI fidelity, and no
 * command we know of returns one.
 */
static int gnss_kepler_power_on(struct gnss_kepler *gk);
static void gnss_kepler_power_off(struct gnss_kepler *gk);
static void gnss_kepler_hold_reset(struct gnss_kepler *gk);
static int gnss_kepler_release_reset(struct gnss_kepler *gk);

static int gnss_kepler_req_bcmd(struct gnss_kepler *gk, u16 cmd_id, u16 flags,
				 u32 param1, u32 param2)
{
	u32 val;
	int ret;

	/*
	 * gpsd's EXE-address check reads the register through the
	 * 0x4xxxxxxx aperture (cmd 5, param1 = 0x40043004), and a read of
	 * that address takes this SoC down hard -- a rule we learned the
	 * expensive way well before gpsd ever ran here, and which the
	 * daemon walks straight into on every startup for chip id >= 0xd.
	 * The 0x8xxxxxxx view of the same register reads fine and is what
	 * the ROM's own cmd 1 handler writes, so redirect rather than fail
	 * the command: gpsd gets the answer it is actually asking for.
	 */
	if (cmd_id == GNSS_BCMD_READ32 && param1 == GNSS_EXE_ADDR_UNSAFE) {
		dev_warn(gk->dev,
			 "req_bcmd: redirecting read32 0x%08x -> 0x%08x (the 0x4xxxxxxx aperture hangs the SoC)\n",
			 param1, GNSS_EXE_ADDR_SAFE);
		param1 = GNSS_EXE_ADDR_SAFE;
	}

	/*
	 * Do the reset cycle here, not in the RESET ioctl. The core stays in
	 * its ROM command loop only ~1.4 ms after hold_reset -- ACTIVE fires
	 * at that point and it is off executing -- so the command must be
	 * staged inside that window. Driving the steps from a shell wins the
	 * race (measured 6/6); splitting them across two ioctls loses it
	 * every time (0/6), the process spawn alone costing more than the
	 * window. So hold_reset and the staging below must share a syscall.
	 */
	if (gk->state != GNSS_STATE_ONLINE) {
		gnss_kepler_power_off(gk);
		gnss_kepler_power_on(gk);
		if (gk->reset_settle_ms)
			msleep(gk->reset_settle_ms);
		gnss_kepler_hold_reset(gk);
	}

	writel(((u32)flags << 16) | cmd_id, gk->mbox + GNSS_MBOX_BCMD_CTRL0);
	writel(param1, gk->mbox + GNSS_MBOX_BCMD_CTRL1);
	writel(param2, gk->mbox + GNSS_MBOX_BCMD_CTRL2);
	writel(GNSS_MBOX_BCMD_CTRL3_PENDING, gk->mbox + GNSS_MBOX_BCMD_CTRL3);

	/*
	 * Read the staging back before going anywhere near the PMU. The
	 * ROM samples ISSR0 as it comes out of reset, and the release
	 * below is a regmap write to a different bus -- without this
	 * barrier it can land first and the core reads stale ISSRs. The
	 * debugfs path never hit this because its 200 ms poll sits between
	 * the two; here they are microseconds apart. Measured: without the
	 * read-back the core never answers, with it the reply arrives.
	 */
	readl(gk->mbox + GNSS_MBOX_BCMD_CTRL3);

	writel(BIT(GNSS_MBOX_INT_AP2GNSS_BCMD), gk->mbox + MCU_IPC_INTGR1);
	readl(gk->mbox + MCU_IPC_INTGR1);

	/*
	 * If the core is already sitting in its ROM command loop it answers
	 * straight away, and driving the PMU underneath it would only knock
	 * it out of that loop. Downstream never needs this check because on
	 * stock the core really is held at this point; here, power_on alone
	 * frequently leaves it serving commands. Measured: the debugfs path
	 * gets its reply during staging, before any release_reset -- and
	 * the ioctl path failed precisely because it released first.
	 */
	ret = readl_poll_timeout(gk->mbox + GNSS_MBOX_BCMD_CTRL3, val,
				 val != GNSS_MBOX_BCMD_CTRL3_PENDING,
				 REQ_BCMD_POLL_INTERVAL,
				 BCMD_PROBE_TIMEOUT * USEC_PER_MSEC);
	if (!ret) {
		dev_info(gk->dev,
			 "req_bcmd: cmd_id 0x%x -> 0x%08x (core already serving)\n",
			 cmd_id, val);
		/*
		 * This path returns without ever calling release_reset, so it
		 * also never latched GNSS_STATE_ONLINE -- which meant the
		 * guard at the top stayed true and *every* subsequent command
		 * power-cycled the domain first. Reads still worked, so this
		 * was easy to miss; writes silently did not survive to the
		 * next command, and every value read back was a reset default
		 * rather than live state.
		 *
		 * Latching here is opt-in, because it trades that away: with
		 * the reset cycle skipped, a core that later stops serving
		 * cannot be recovered by another command. gpsd wants the old
		 * behaviour. Multi-step register experiments want this:
		 *     echo 1 > /sys/kernel/debug/gnss_kepler/bcmd_keep_loop
		 */
		if (gk->bcmd_keep_loop)
			gk->state = GNSS_STATE_ONLINE;

		return val;
	}

	if (gk->state != GNSS_STATE_ONLINE) {
		gnss_kepler_release_reset(gk);
		gk->state = GNSS_STATE_ONLINE;
	}

	/* BLC_Branch has no reply -- downstream returns 0 without waiting. */
	if (cmd_id == GNSS_BCMD_BLC_BRANCH)
		return 0;

	ret = readl_poll_timeout(gk->mbox + GNSS_MBOX_BCMD_CTRL3, val,
				 val != GNSS_MBOX_BCMD_CTRL3_PENDING,
				 REQ_BCMD_POLL_INTERVAL,
				 REQ_BCMD_TIMEOUT * USEC_PER_MSEC);
	if (ret) {
		dev_err(gk->dev, "req_bcmd: cmd_id 0x%x TIMEOUT\n", cmd_id);
		return -EIO;
	}

	dev_info(gk->dev, "req_bcmd: cmd_id 0x%x flags 0x%x -> 0x%08x\n",
		 cmd_id, flags, val);

	return val;
}

static int gnss_ipc_open(struct inode *inode, struct file *file)
{
	file->private_data = container_of(file->private_data,
					   struct gnss_kepler, miscdev);

	return 0;
}

static long gnss_ipc_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	struct gnss_kepler *gk = file->private_data;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case GNSS_IOCTL_RESET:
		/*
		 * Only power the domain down and record it. The reset cycle
		 * proper happens inside req_bcmd, because the staging has to
		 * follow hold_reset within about 1.4 ms and a userspace round
		 * trip does not fit in that window -- see the comment there.
		 */
		gnss_kepler_power_off(gk);
		gk->state = GNSS_STATE_HOLD_RESET;
		return 0;

	case GNSS_IOCTL_RELEASE_RESET: {
		int ret = gnss_kepler_modem_hold(gk);

		if (ret)
			return ret;
		gnss_kepler_release_reset(gk);
		gk->state = GNSS_STATE_ONLINE;
		return 0;
	}

	case GNSS_IOCTL_REQ_BCMD: {
		struct kepler_bcmd_args a;
		int ret;

		if (copy_from_user(&a, argp, sizeof(a)))
			return -EFAULT;

		ret = gnss_kepler_modem_hold(gk);
		if (ret)
			return ret;

		ret = gnss_kepler_req_bcmd(gk, a.cmd_id, a.flags, a.param1,
					    a.param2);
		if (ret == -EIO)
			return -EIO;

		a.ret_val = ret;

		return copy_to_user(argp, &a, sizeof(a)) ? -EFAULT : 0;
	}

	case GNSS_IOCTL_LOAD_FIRMWARE: {
		struct kepler_firmware_args a;
		u32 off;

		if (copy_from_user(&a, argp, sizeof(a)))
			return -EFAULT;

		off = a.offset;

		/*
		 * gnss_load_firmware()'s boot_without_mbox case downstream.
		 * gpsd's device type 0xa -- which is what exynos7885 resolves
		 * to, via chipset "K148" -- takes a path that issues no BCMDs
		 * at all: it loads the firmware and calls RELEASE_RESET, and
		 * the only thing that can boot the core from there is the ROM
		 * autorun header. So an offset of 0 means "put the image
		 * behind a header", exactly as downstream does.
		 *
		 * Note this is inference for *this* board: the property is a
		 * DT one and no a30/a40 DT exists in the downstream tree we
		 * have, where it appears only on 7883/7884 boards. Hence the
		 * toggle rather than an unconditional change.
		 */
		if (gk->load_fw_autorun && off == 0) {
			off = AUTO_RUN_CMD_SIZE;
			writel(GNSS_AUTO_RUN_MAGIC, gk->dram + 0x0);
			writel(0, gk->dram + 0x4);
			writel(a.firmware_size, gk->dram + 0x8);
			writel(0, gk->dram + 0xc);
		}

		if (off > gk->mem_size || a.firmware_size > gk->mem_size - off)
			return -EINVAL;
		if (copy_from_user(gk->dram + off, a.firmware_bin,
				    a.firmware_size))
			return -EFAULT;

		dev_info(gk->dev,
			 "load_firmware: %u bytes at region offset 0x%x%s\n",
			 a.firmware_size, off,
			 off != a.offset ? " (autorun header written)" : "");

		return 0;
	}

	case GNSS_IOCTL_READ_FIRMWARE: {
		struct kepler_firmware_args a;

		if (copy_from_user(&a, argp, sizeof(a)))
			return -EFAULT;
		if (a.offset > gk->mem_size ||
		    a.firmware_size > gk->mem_size - a.offset)
			return -EINVAL;
		if (copy_to_user(a.firmware_bin, gk->dram + a.offset,
				  a.firmware_size))
			return -EFAULT;

		return 0;
	}

	case GNSS_IOCTL_REQ_FAULT_INFO: {
		struct kepler_fault_args a;
		u32 len;

		if (copy_from_user(&a, argp, sizeof(a)))
			return -EFAULT;

		gnss_kepler_req_fault_info(gk);

		len = min_t(u32, a.dump_size, GNSS_FAULT_DUMP_SIZE);
		if (copy_to_user(a.dumped_data, gk->dram + GNSS_FAULT_OFFSET,
				  len))
			return -EFAULT;

		return 0;
	}

	/*
	 * Sensor and TCXO plumbing this port has no hardware description
	 * for. gpsd calls the sensor pair after a successful codeload and
	 * only logs the result, so succeeding quietly keeps it moving; if
	 * that turns out to matter, it will show up as a later failure.
	 */
	case GNSS_IOCTL_CHANGE_SENSOR_GPIO:
	case GNSS_IOCTL_CHANGE_TCXO_MODE:
	case GNSS_IOCTL_SET_SENSOR_POWER:
		dev_info(gk->dev, "ioctl 0x%x stubbed, returning success\n",
			 cmd);
		return 0;

	default:
		return -ENOTTY;
	}
}

/*
 * One IPC frame per read(). The header is stripped: the rings carry the
 * 12-byte EXYNOS framing, which is this driver's business, and the
 * daemon above wants only the BETP payload.
 */
static ssize_t gnss_ipc_read(struct file *file, char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct gnss_kepler *gk = file->private_data;
	u8 frame[GNSS_IPC_RX_LOG_BYTES];
	unsigned long flags;
	unsigned int slot;
	u32 len;
	int ret;

	for (;;) {
		spin_lock_irqsave(&gk->ipc_rx_log_lock, flags);

		if (gk->ipc_rx_log_out != gk->ipc_rx_log_in) {
			slot = gk->ipc_rx_log_out % GNSS_IPC_RX_LOG_FRAMES;
			len = gk->ipc_rx_log[slot].len;
			memcpy(frame, gk->ipc_rx_log[slot].data, len);
			gk->ipc_rx_log_out++;
			spin_unlock_irqrestore(&gk->ipc_rx_log_lock, flags);
			break;
		}

		spin_unlock_irqrestore(&gk->ipc_rx_log_lock, flags);

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(gk->ipc_rx_wait,
					        gk->ipc_rx_log_out !=
					        gk->ipc_rx_log_in);
		if (ret)
			return ret;
	}

	if (len <= EXYNOS_HEADER_SIZE)
		return 0;

	len = min_t(size_t, len - EXYNOS_HEADER_SIZE, count);
	if (copy_to_user(buf, frame + EXYNOS_HEADER_SIZE, len))
		return -EFAULT;

	return len;
}

static ssize_t gnss_ipc_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct gnss_kepler *gk = file->private_data;
	void *kbuf;
	int ret;

	if (!count || count > SZ_4K)
		return -EINVAL;

	kbuf = memdup_user(buf, count);
	if (IS_ERR(kbuf))
		return PTR_ERR(kbuf);

	ret = gnss_kepler_ipc_send(gk, gk->tx_channel, kbuf, count);
	kfree(kbuf);

	return ret ? ret : count;
}

static __poll_t gnss_ipc_poll(struct file *file, poll_table *wait)
{
	struct gnss_kepler *gk = file->private_data;

	poll_wait(file, &gk->ipc_rx_wait, wait);

	return gk->ipc_rx_log_out != gk->ipc_rx_log_in ?
	       EPOLLIN | EPOLLRDNORM : 0;
}

static const struct file_operations gnss_ipc_fops = {
	.owner = THIS_MODULE,
	.open = gnss_ipc_open,
	.read = gnss_ipc_read,
	.write = gnss_ipc_write,
	.poll = gnss_ipc_poll,
	.unlocked_ioctl = gnss_ipc_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.llseek = noop_llseek,
};


/* "1" powers the domain up, "0" takes it down -- see power_off(). */
static ssize_t start_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct gnss_kepler *gk = file->private_data;
	u32 on;
	int ret = kstrtou32_from_user(buf, count, 0, &on);

	if (ret)
		return ret;

	if (on) {
		ret = gnss_kepler_modem_hold(gk);
		if (ret)
			return ret;
		gnss_kepler_power_on(gk);
	} else {
		gnss_kepler_power_off(gk);
		gnss_kepler_modem_release(gk);
	}

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
	struct gnss_kepler *gk = file->private_data;

	gnss_kepler_hold_reset(gk);

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
	struct gnss_kepler *gk = file->private_data;
	int ret = gnss_kepler_modem_hold(gk);

	if (ret)
		return ret;
	gnss_kepler_release_reset(gk);

	return count;
}

static const struct file_operations release_fops = {
	.open = simple_open,
	.write = release_write,
	.llseek = noop_llseek,
};

/*
 * Matches the downstream faultdump tool's own test: a plain
 * GNSS_IOCTL_REQ_BCMD with a zeroed struct kepler_bcmd_args (cmd_id=0,
 * flags=0, param1=0, param2=0) -- not the special cmd_id=0x4
 * "BLC_Branch" case downstream skips waiting for a reply on.
 */
static ssize_t bcmd_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	struct gnss_kepler *gk = file->private_data;

	char kbuf[64];
	u32 cmd_id, flags = 0, param1 = 0, param2 = 0;
	int n;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;
	kbuf[count] = '\0';

	/*
	 * "cmd_id [flags [param1 [param2]]]", each 0x-prefixed or decimal.
	 * A bare "1" (what every earlier session echoed into this file)
	 * still means cmd_id 0x1 with zero arguments, so old habits do not
	 * silently send something different -- but note that is no longer
	 * the all-zero probe it used to be. Use "0" for that.
	 */
	n = sscanf(kbuf, "%i %i %i %i", &cmd_id, &flags, &param1, &param2);
	if (n < 1)
		return -EINVAL;
	if (cmd_id > U16_MAX || flags > U16_MAX)
		return -EINVAL;

	gnss_kepler_send_bcmd(gk, cmd_id, flags, param1, param2);

	return count;
}

static const struct file_operations bcmd_fops = {
	.open = simple_open,
	.write = bcmd_write,
	.llseek = noop_llseek,
};

/*
 * Firmware payload, written to gk->dram + AUTO_RUN_CMD_SIZE. Tracks its
 * own offset in gk->fw_offset rather than relying on VFS *ppos
 * bookkeeping, so a plain `cat blob > .../fw` works regardless of the
 * file's open/seek semantics -- reset on open so repeat loads land at
 * the start again.
 */
static int fw_open(struct inode *inode, struct file *file)
{
	struct gnss_kepler *gk = inode->i_private;

	gk->fw_offset = 0;
	file->private_data = gk;

	return 0;
}

static ssize_t fw_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	struct gnss_kepler *gk = file->private_data;

	if (AUTO_RUN_CMD_SIZE + gk->fw_offset + count > gk->mem_size)
		return -ENOSPC;

	if (copy_from_user(gk->dram + AUTO_RUN_CMD_SIZE + gk->fw_offset, buf,
			    count))
		return -EFAULT;

	gk->fw_offset += count;

	return count;
}

static const struct file_operations fw_fops = {
	.open = fw_open,
	.write = fw_write,
	.llseek = noop_llseek,
};

/*
 * Same payload, staged at region offset 0 instead of behind the autorun
 * header. This is what gpsd's GNSS_IOCTL_LOAD_FIRMWARE does: the image
 * descriptor carries addr 0, and the codeload BCMD then passes
 * param1 = addr + 0x50000000, i.e. the very start of the window as Kepler
 * sees it. The ROM's cmd 1 handler copies from there to 0x00200000
 * itself, so no header is involved and none must be left behind -- clear
 * it with `echo 0 > fw_commit` first. See gnss.md.
 */
static ssize_t fw_raw_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct gnss_kepler *gk = file->private_data;

	if (gk->fw_offset + count > gk->mem_size)
		return -ENOSPC;

	if (copy_from_user(gk->dram + gk->fw_offset, buf, count))
		return -EFAULT;

	gk->fw_offset += count;

	return count;
}

static const struct file_operations fw_raw_fops = {
	.open = fw_open,
	.write = fw_raw_write,
	.llseek = noop_llseek,
};

/*
 * shmem_set_autorun_cmd() downstream: write the 16-byte autorun header
 * once the payload write above is done, with the real size taken as
 * the number of bytes actually written (echo <bytes-written> > this
 * file, or just the shell's own $(wc -c) of what was cat'd into fw).
 */
static ssize_t fw_commit_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct gnss_kepler *gk = file->private_data;
	u32 size;
	int ret = kstrtou32_from_user(buf, count, 0, &size);

	if (ret)
		return ret;

	dev_info(gk->dev, "fw_commit size=%u\n", size);

	/*
	 * size 0 clears the header instead of writing it. The ROM checks
	 * for the autorun magic before anything else, so a stale header
	 * (DRAM survives a module reload, and even a warm reboot) wins
	 * over the cmd 2 "RUN" path, which records an entry address in
	 * hardware and sets mode 0xa for the boot path to act on. To test
	 * that path the magic has to be gone. See gnss.md.
	 */
	if (!size) {
		memset_io(gk->dram, 0, AUTO_RUN_CMD_SIZE);
		dev_info(gk->dev, "fw_commit: autorun header cleared\n");
		return count;
	}

	writel(GNSS_AUTO_RUN_MAGIC, gk->dram + 0x0);
	writel(0, gk->dram + 0x4);
	writel(size, gk->dram + 0x8);
	writel(0, gk->dram + 0xc);

	return count;
}

static const struct file_operations fw_commit_fops = {
	.open = simple_open,
	.write = fw_commit_write,
	.llseek = noop_llseek,
};

static int regs_show(struct seq_file *s, void *unused)
{
	struct gnss_kepler *gk = s->private;
	static const struct { u32 offset; const char *name; } regs[] = {
		{ GNSS_CTRL_NS,			"GNSS_CTRL_NS                " },
		{ GNSS_CTRL_S,			"GNSS_CTRL_S                 " },
		{ GNSS_STAT,			"GNSS_STAT                   " },
		{ GNSS_DEBUG,			"GNSS_DEBUG                  " },
		{ CENTRAL_SEQ_GNSS_CONFIGURATION, "CENTRAL_SEQ_GNSS_CONFIG     " },
		{ CENTRAL_SEQ_GNSS_STATUS,	"CENTRAL_SEQ_GNSS_STATUS     " },
		{ RESET_SEQUENCER_STATUS,	"RESET_SEQUENCER_STATUS      " },
		{ TCXO_GATE_GNSS_SYS_PWR_REG,	"TCXO_GATE_GNSS_SYS_PWR_REG  " },
		{ GNSS_DISABLE_ISO_SYS_PWR_REG, "GNSS_DISABLE_ISO_SYS_PWR_REG" },
		{ GNSS_RESET_ISO_SYS_PWR_REG,	"GNSS_RESET_ISO_SYS_PWR_REG  " },
		{ GNSS2AP_MEM_CONFIG0,		"GNSS2AP_MEM_CONFIG0 (size)  " },
		{ GNSS2AP_MEM_CONFIG1,		"GNSS2AP_MEM_CONFIG1 (base)  " },
		{ GNSS2AP_MIF_ACCESS_WIN0,	"GNSS2AP_MIF_ACCESS_WIN0     " },
		{ GNSS2AP_PERI_ACCESS_WIN0,	"GNSS2AP_PERI_ACCESS_WIN0    " },
		{ GNSS2AP_PERI_ACCESS_WIN1,	"GNSS2AP_PERI_ACCESS_WIN1    " },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		u32 val;
		int ret = regmap_read(gk->pmureg, regs[i].offset, &val);

		if (ret)
			seq_printf(s, "%s (0x%04x) = <err %d>\n",
				   regs[i].name, regs[i].offset, ret);
		else
			seq_printf(s, "%s (0x%04x) = 0x%08x\n",
				   regs[i].name, regs[i].offset, val);
	}

	seq_printf(s, "ACTIVE interrupts: %u\n", gk->active_count);
	seq_printf(s, "WATCHDOG interrupts: %u\n", gk->watchdog_count);
	seq_printf(s, "mbox interrupts: %u\n", gk->mbox_irq_count);

	/*
	 * MCUCTLR bit 0 is MSWRST. mbox_sw_reset() sets it and never
	 * clears it -- downstream's mbox_sw_reset() doesn't either -- so
	 * whether it is self-clearing in hardware decides whether the
	 * mailbox is usable after a hold_reset at all. See gnss.md.
	 */
	seq_printf(s, "MBOX MCUCTLR                 = 0x%08x (MSWRST=%u)\n",
		   readl(gk->mbox + MCU_IPC_MCUCTLR),
		   !!(readl(gk->mbox + MCU_IPC_MCUCTLR) & MCU_IPC_MSWRST));
	seq_printf(s, "MBOX ISSR0 (BCMD ctrl0)      = 0x%08x\n",
		   readl(gk->mbox + GNSS_MBOX_BCMD_CTRL0));
	seq_printf(s, "MBOX ISSR1 (BCMD param1)     = 0x%08x\n",
		   readl(gk->mbox + GNSS_MBOX_BCMD_CTRL1));
	seq_printf(s, "MBOX ISSR2 (BCMD param2)     = 0x%08x\n",
		   readl(gk->mbox + GNSS_MBOX_BCMD_CTRL2));
	seq_printf(s, "MBOX ISSR3 (BCMD ctrl3/ret)  = 0x%08x\n",
		   readl(gk->mbox + GNSS_MBOX_BCMD_CTRL3));
	seq_printf(s, "MBOX INTSR0                  = 0x%08x\n",
		   readl(gk->mbox + MCU_IPC_INTSR0));
	seq_printf(s, "MBOX INTMR0                  = 0x%08x\n",
		   readl(gk->mbox + MCU_IPC_INTMR0));

	{
		u32 val;

		regmap_read(gk->pmureg, CP_STAT, &val);
		seq_printf(s, "CP_STAT (0x%04x)                    = 0x%08x\n",
			   CP_STAT, val);
		regmap_read(gk->pmureg, EXT_REGULATOR_SHARED_STATUS, &val);
		seq_printf(s, "EXT_REGULATOR_SHARED_STATUS (0x%04x) = 0x%08x\n",
			   EXT_REGULATOR_SHARED_STATUS, val);
		regmap_read(gk->pmureg, EXT_REGULATOR_SHARED_OPTION, &val);
		seq_printf(s, "EXT_REGULATOR_SHARED_OPTION (0x%04x) = 0x%08x\n",
			   EXT_REGULATOR_SHARED_OPTION, val);
	}
	seq_printf(s, "shared regulator vote        = %s\n",
		   gk->shared_reg_on ? "held" : "not held");
	seq_printf(s, "modem                        = %s\n",
		   !gk->modem ? "none" :
		   gk->modem_held ? "held" : "not held");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(regs);

/*
 * Brute-force diff aid, take two. A blind word-by-word sweep of the
 * *entire* PMU_ALIVE syscon range (0x11c80000, size 0x10000 per its DT
 * node in exynos7885.dtsi) rebooted the device once (with no pstore
 * record of why) and hard-hung it a second time (needing a physical
 * power cycle, the hardware watchdog on this board being permanently
 * disabled) -- see gnss.md. Most of that 64KB is evidently *not*
 * backed by plain registers; reading the wrong offset there isn't
 * safe, it wedges the bus. That approach is abandoned.
 *
 * This is the safer version of the same idea: only ever read offsets
 * Samsung's own cal-if PMUCAL tables reference for base 0x11C80000
 * (`pmucal_system_exynos7885.h`, `pmucal_cp_exynos7885.h`,
 * `pmucal_gnss_exynos7885.h`, downstream) -- a finite, vendor-curated
 * list of real SFRs their own driver reads/writes at every boot, so
 * reading them back can't hit the same reserved/power-gated holes the
 * blind sweep did. Extracted mechanically (every
 * `PMUCAL_SEQ_DESC(..., 0x11C80000, <offset>, ...)` argument, deduped,
 * sorted) rather than hand-picked, so it's not just re-deriving the
 * dozen registers already named above -- the point is coverage of
 * everything *known safe to touch*, still diffable against an
 * equivalent downstream capture.
 */
static const u16 pmu_alive_known_offsets[] = {
	0x0024, 0x0030, 0x0034, 0x0040, 0x0044, 0x00d0, 0x0120, 0x0200,
	0x0208, 0x0240, 0x0280, 0x0284, 0x02c4, 0x0418, 0x0420, 0x0504,
	0x0628, 0x064c, 0x0dcc, 0x1000, 0x1008, 0x1010, 0x1018, 0x1020,
	0x1028, 0x1030, 0x1038, 0x1040, 0x1048, 0x1050, 0x1058, 0x1060,
	0x1068, 0x1070, 0x1078, 0x1080, 0x1084, 0x1088, 0x10a4, 0x10a8,
	0x10ac, 0x10c0, 0x10c4, 0x10c8, 0x10d0, 0x1100, 0x1104, 0x110c,
	0x1114, 0x1118, 0x111c, 0x1120, 0x1124, 0x112c, 0x1130, 0x113c,
	0x1140, 0x1148, 0x1158, 0x115c, 0x1160, 0x1164, 0x116c, 0x1180,
	0x1190, 0x119c, 0x11a0, 0x11a8, 0x11ac, 0x11b0, 0x11b8, 0x11bc,
	0x11c0, 0x11cc, 0x11dc, 0x11e8, 0x11f0, 0x1200, 0x1204, 0x1208,
	0x120c, 0x1210, 0x1214, 0x121c, 0x1220, 0x1230, 0x1234, 0x1240,
	0x1250, 0x1260, 0x12c0, 0x12c4, 0x1300, 0x1304, 0x1308, 0x1320,
	0x1324, 0x1328, 0x132c, 0x1330, 0x1334, 0x1338, 0x1340, 0x1344,
	0x1348, 0x134c, 0x1350, 0x1354, 0x1404, 0x1408, 0x140c, 0x1410,
	0x1414, 0x1418, 0x2008, 0x2088, 0x2108, 0x2188, 0x2208, 0x2288,
	0x2308, 0x2388, 0x2408, 0x2428, 0x2448, 0x2608, 0x2628, 0x2648,
	0x2c08, 0x2c48, 0x2c88, 0x2cc8, 0x2e08, 0x2f28, 0x3008, 0x3028,
	0x3068, 0x3088, 0x30a8, 0x30e8, 0x3108, 0x3188, 0x31a8, 0x330c,
	0x361c, 0x363c, 0x3648, 0x365c, 0x371c, 0x3920, 0x3924, 0x39d8,
	0x3a20, 0x3a24, 0x4024, 0x4028, 0x4044, 0x4048, 0x4064, 0x4068,
	0x4084, 0x4088, 0x40a4, 0x40a8, 0x40c8, 0x4a28, 0x4a48, 0x4a68,
	0x4a88, 0x4aa8, 0x4ac0, 0x8000, 0x8084, 0x8088, 0x808c, 0x8094,
	0x80a0, 0x80a4, 0x80bc, 0x80c8, 0x8228, 0x82c8, 0x8400,
};

static int regs_raw_show(struct seq_file *s, void *unused)
{
	struct gnss_kepler *gk = s->private;
	int i;

	for (i = 0; i < ARRAY_SIZE(pmu_alive_known_offsets); i++) {
		u32 off = pmu_alive_known_offsets[i];
		u32 val;
		int ret = regmap_read(gk->pmureg, off, &val);

		if (ret)
			continue;
		seq_printf(s, "0x%04x: 0x%08x\n", off, val);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(regs_raw);

static int fault_show(struct seq_file *s, void *unused)
{
	struct gnss_kepler *gk = s->private;
	const u8 *p = gk->dram + GNSS_FAULT_OFFSET;
	int i;

	for (i = 0; i < GNSS_FAULT_DUMP_SIZE; i += 16) {
		int j;

		seq_printf(s, "%04x: ", i);
		for (j = i; j < i + 16; j++)
			seq_printf(s, "%02x ", p[j]);
		seq_puts(s, " |");
		for (j = i; j < i + 16; j++)
			seq_putc(s, isprint(p[j]) ? p[j] : '.');
		seq_puts(s, "|\n");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(fault);

/*
 * Raw read of the whole shared region, so any offset can be inspected
 * with dd rather than only the fixed fault window above. The mapping is
 * a vmap() of normal pages (see gnss_kepler_map_wc()), so it is a plain
 * kernel VA despite the __iomem cast, and a direct copy is what the
 * fault dump above already relies on.
 *
 * This is the only way to see whether Kepler is doing anything once it
 * stops faulting: with firmware loaded there is no fault record to read.
 */
static ssize_t dram_read(struct file *file, char __user *buf, size_t count,
			  loff_t *ppos)
{
	struct gnss_kepler *gk = file->private_data;

	if (*ppos < 0 || *ppos >= gk->mem_size)
		return 0;

	count = min_t(size_t, count, gk->mem_size - *ppos);
	if (copy_to_user(buf, (void *)gk->dram + *ppos, count))
		return -EFAULT;

	*ppos += count;

	return count;
}

static const struct file_operations dram_fops = {
	.open = simple_open,
	.read = dram_read,
	.llseek = default_llseek,
};

static void gnss_kepler_debugfs_init(struct gnss_kepler *gk)
{
	struct dentry *root = debugfs_create_dir("gnss_kepler", NULL);
	struct dentry *start_file;

	gk->debugfs_dir = root;

	start_file = debugfs_create_file("start", 0200, root, gk,
					  &start_fops);
	if (IS_ERR(start_file))
		dev_warn(gk->dev, "debugfs: failed to create start\n");

	debugfs_create_bool("hold_shared_reg", 0600, root,
			     &gk->hold_shared_reg);
	gk->need_modem = true;
	debugfs_create_bool("need_modem", 0600, root, &gk->need_modem);
	gk->load_fw_autorun = true;
	debugfs_create_bool("bcmd_keep_loop", 0600, root, &gk->bcmd_keep_loop);
	debugfs_create_bool("tcxo_sw", 0600, root, &gk->tcxo_sw);
	debugfs_create_bool("load_fw_autorun", 0600, root,
			     &gk->load_fw_autorun);
	debugfs_create_file("state", 0400, root, gk, &state_fops);
	debugfs_create_file("regs", 0400, root, gk, &regs_fops);
	debugfs_create_file("regs_raw", 0400, root, gk, &regs_raw_fops);
	debugfs_create_file("fault", 0400, root, gk, &fault_fops);
	debugfs_create_file("dram", 0400, root, gk, &dram_fops);
	debugfs_create_file("kmem", 0400, root, gk, &kmem_fops);
	debugfs_create_file("ipc_regs", 0400, root, gk, &ipc_regs_fops);
	debugfs_create_file("ipc_tx", 0200, root, gk, &ipc_tx_fops);
	debugfs_create_file("ipc_rx", 0400, root, gk, &ipc_rx_fops);
	debugfs_create_file("ipc_reset", 0200, root, gk, &ipc_reset_fops);
	debugfs_create_file("win", 0200, root, gk, &win_fops);
	debugfs_create_file("ipc_loopback", 0200, root, gk,
			     &ipc_loopback_fops);
	debugfs_create_file("fault_req", 0200, root, gk, &fault_req_fops);
	debugfs_create_file("bcmd", 0200, root, gk, &bcmd_fops);
	debugfs_create_file("reset", 0200, root, gk, &reset_fops);
	debugfs_create_file("release", 0200, root, gk, &release_fops);
	debugfs_create_file("fw", 0200, root, gk, &fw_fops);
	debugfs_create_file("fw_raw", 0200, root, gk, &fw_raw_fops);
	debugfs_create_file("fw_commit", 0200, root, gk, &fw_commit_fops);
}

/*
 * memremap(MEMREMAP_WB) gives a *cached* alias into the direct map,
 * which is wrong for memory a non-coherent bus master (the Kepler
 * core) writes into directly: reads here could keep serving a stale
 * cache line from before the core ever wrote anything, instead of
 * what's actually in DRAM now. memremap(MEMREMAP_WC) would be the
 * fix, but it explicitly refuses any range recognized as System RAM
 * (gnss_rmem is deliberately left mapped that way -- see its DT node
 * comment), so it's not usable here either. This mirrors what
 * downstream's shm_request_region() actually does instead: build a
 * page array by hand and vmap() it with an explicit write-combining
 * pgprot, which bypasses memremap()'s RAM safety check at a lower
 * level. Found by chasing a real symptom: `fault` stayed all-0xcc
 * indefinitely after a BCMD send that should have produced content
 * (see gnss.md).
 */
static void __iomem *gnss_kepler_map_wc(phys_addr_t base, size_t size)
{
	unsigned int num_pages = size >> PAGE_SHIFT;
	struct page **pages;
	void *addr;
	unsigned int i;

	pages = kmalloc_array(num_pages, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return NULL;

	for (i = 0; i < num_pages; i++)
		pages[i] = phys_to_page(base + i * PAGE_SIZE);

	addr = vmap(pages, num_pages, VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	kfree(pages);

	return (void __iomem *)addr;
}

static void gnss_kepler_put_modem(void *data)
{
	rproc_put(data);
}

/* samsung,modem: the CP remoteproc the core depends on, optional */
static int gnss_kepler_get_modem(struct gnss_kepler *gk)
{
	phandle ph;

	if (of_property_read_u32(gk->dev->of_node, "samsung,modem", &ph))
		return 0;

	gk->modem = rproc_get_by_phandle(ph);
	if (!gk->modem)
		return dev_err_probe(gk->dev, -EPROBE_DEFER,
				      "modem remoteproc not registered yet\n");

	return devm_add_action_or_reset(gk->dev, gnss_kepler_put_modem,
					gk->modem);
}

static int gnss_kepler_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *mem_np;
	struct reserved_mem *rmem;
	struct gnss_kepler *gk;
	int ret;

	gk = devm_kzalloc(dev, sizeof(*gk), GFP_KERNEL);
	if (!gk)
		return -ENOMEM;

	gk->dev = dev;
	mutex_init(&gk->power_lock);
	init_completion(&gk->bcmd_cmpl);
	init_completion(&gk->fault_cmpl);
	spin_lock_init(&gk->tx_lock);
	spin_lock_init(&gk->ipc_rx_log_lock);
	INIT_WORK(&gk->ipc_rx_work, gnss_kepler_ipc_rx_work);
	platform_set_drvdata(pdev, gk);

	gk->pmureg = syscon_regmap_lookup_by_phandle(dev->of_node,
						      "samsung,syscon-phandle");
	if (IS_ERR(gk->pmureg))
		return dev_err_probe(dev, PTR_ERR(gk->pmureg),
				      "failed to look up PMU syscon\n");

	mem_np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!mem_np)
		return dev_err_probe(dev, -ENODEV,
				      "no memory-region property\n");

	rmem = of_reserved_mem_lookup(mem_np);
	of_node_put(mem_np);
	if (!rmem)
		return dev_err_probe(dev, -ENODEV,
				      "failed to look up reserved-memory region\n");

	gk->mem_base = rmem->base;
	gk->mem_size = rmem->size;

	gk->dram = gnss_kepler_map_wc(gk->mem_base, gk->mem_size);
	if (!gk->dram)
		return dev_err_probe(dev, -ENOMEM,
				      "failed to map shared DRAM region\n");

	/*
	 * Before anything can power the core on -- including the `start`
	 * debugfs file, which is reachable the moment debugfs_init() runs
	 * below.
	 */
	gnss_kepler_init_conf(gk);

	gk->irq_active = platform_get_irq_byname(pdev, "active");
	if (gk->irq_active < 0)
		return gk->irq_active;

	ret = devm_request_irq(dev, gk->irq_active, gnss_kepler_active_irq,
				IRQF_TRIGGER_HIGH, "active", gk);
	if (ret)
		return dev_err_probe(dev, ret,
				      "failed to request active irq\n");

	gk->irq_watchdog = platform_get_irq_byname(pdev, "watchdog");
	if (gk->irq_watchdog < 0)
		return gk->irq_watchdog;

	ret = devm_request_irq(dev, gk->irq_watchdog,
				gnss_kepler_watchdog_irq, IRQF_TRIGGER_HIGH,
				"watchdog", gk);
	if (ret)
		return dev_err_probe(dev, ret,
				      "failed to request watchdog irq\n");

	gk->mbox = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gk->mbox))
		return dev_err_probe(dev, PTR_ERR(gk->mbox),
				      "failed to map mailbox registers\n");

	/* the vote for the regulator shared with the CP, see shared_reg_enable() */
	gk->shared_reg = devm_regulator_get(dev, "shared");
	if (IS_ERR(gk->shared_reg))
		return dev_err_probe(dev, PTR_ERR(gk->shared_reg),
				      "no shared regulator\n");

	ret = gnss_kepler_get_modem(gk);
	if (ret)
		return ret;

	gk->irq_mbox = platform_get_irq_byname(pdev, "mbox");
	if (gk->irq_mbox < 0)
		return gk->irq_mbox;

	ret = devm_request_irq(dev, gk->irq_mbox, gnss_kepler_mbox_irq,
				IRQF_TRIGGER_HIGH, "mbox", gk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request mbox irq\n");

	/*
	 * Unmask the response bits we handle: int 0
	 * (mbx,irq_gnss2ap_bcmd downstream), int 1
	 * (mbx,irq_gnss2ap_rsp_fault_info) and int 2
	 * (mbx,irq_gnss2ap_ipc_msg). INTMR0 mirrors INTSR0's
	 * upper-16-bits-per-int layout; clearing a bit unmasks it,
	 * matching mbox_enable_irq() downstream.
	 */
	/*
	 * Unmask *all* sixteen GNSS->AP interrupts, not just the three we
	 * know. Downstream has a handler for a "WAKE interrupt(Mbox15)"
	 * that acks with int_ap2gnss_ack_wake_set, so the core has at
	 * least one doorbell beyond our three -- and a firmware waiting on
	 * an ack we never send because the interrupt is masked would look
	 * exactly like the one we have: initialised, ring descriptor
	 * correct, and silent. Masked interrupts cost us the evidence; the
	 * ISR logs whatever arrives.
	 */
	writel(readl(gk->mbox + MCU_IPC_INTMR0) & ~0xffff0000,
	       gk->mbox + MCU_IPC_INTMR0);

	gnss_kepler_ipc_init(gk);
	gnss_kepler_debugfs_init(gk);

	init_waitqueue_head(&gk->ipc_rx_wait);
	gk->state = GNSS_STATE_OFFLINE;
	/*
	 * Channel 0. Downstream is explicit about it in two places --
	 * exynos_build_header() does
	 *   buff[EXYNOS_CH_ID_OFFSET] = 0;  / * single channel, should be 0 * /
	 * and gnss_main.c sets iodev->id = 0 / * Fixed channel 0 * / -- so the
	 * EXYNOS frame's ch_id is not a routing field here at all.
	 *
	 * This used to default to 145, guessed from the port ids 0x91..0x94
	 * the firmware registers in its own table. That was wrong: those are
	 * internal to the firmware and have nothing to do with the frame
	 * header, so every frame we sent was stamped with a channel the
	 * receiver had no reason to accept. Kept settable for experiments.
	 */
	gk->tx_channel = 0;
	debugfs_create_u8("tx_channel", 0600, gk->debugfs_dir, &gk->tx_channel);
	gk->power_off_ms = 300;
	debugfs_create_u32("power_off_ms", 0600, gk->debugfs_dir,
			   &gk->power_off_ms);
	gk->reset_settle_ms = 2;
	debugfs_create_u32("reset_settle_ms", 0600, gk->debugfs_dir,
			   &gk->reset_settle_ms);

	gk->miscdev.minor = MISC_DYNAMIC_MINOR;
	gk->miscdev.name = "gnss_ipc";
	gk->miscdev.fops = &gnss_ipc_fops;
	ret = misc_register(&gk->miscdev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register /dev/gnss_ipc\n");

	dev_info(dev,
		 "probed: mem %pa+%zx, active irq %d, watchdog irq %d, mbox irq %d\n",
		 &gk->mem_base, gk->mem_size, gk->irq_active,
		 gk->irq_watchdog, gk->irq_mbox);

	return 0;
}

static void gnss_kepler_remove(struct platform_device *pdev)
{
	struct gnss_kepler *gk = platform_get_drvdata(pdev);

	misc_deregister(&gk->miscdev);
	gnss_kepler_shared_reg_disable(gk);
	gnss_kepler_modem_release(gk);
	debugfs_remove_recursive(gk->debugfs_dir);
	cancel_work_sync(&gk->ipc_rx_work);
	vunmap(gk->dram);
}

static const struct of_device_id gnss_kepler_of_match[] = {
	{ .compatible = "samsung,exynos7885-gnss-kepler" },
	{}
};
MODULE_DEVICE_TABLE(of, gnss_kepler_of_match);

static struct platform_driver gnss_kepler_driver = {
	.probe = gnss_kepler_probe,
	.remove = gnss_kepler_remove,
	.driver = {
		.name = "gnss_kepler",
		.of_match_table = gnss_kepler_of_match,
	},
};
module_platform_driver(gnss_kepler_driver);

MODULE_DESCRIPTION("Samsung Kepler GNSS companion chip bring-up driver");
MODULE_LICENSE("GPL");
