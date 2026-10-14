// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos7885 modem (Shannon CP) remoteproc driver.
 *
 * Boots the CP the way the vendor stack (cbd + modem_v1 + mcu_ipc) does,
 * up to the point where the CP main firmware asks for the IPC link
 * (CMD_INIT_START):
 *
 *   load:  copy BOOT, MAIN and VSS from the radio image, and the NV data,
 *          into the CP carve-out at the offsets the image's TOC gives
 *   start: CP secure boot (EL3), mailbox init, "BOOT" magic in the IPC
 *          region, CP power-on through the PMU, bootloader handshake,
 *          wait for INIT_START
 *
 * Once INIT_START is in, the SBD link (drivers/net/wwan/samsung_sbd.c)
 * takes over as a subdevice: ring layout, PIF_INIT_DONE, PHONE_START ack,
 * then data on WWAN ports. Its ports exist from probe on, so daemons
 * can open them before the CP is started.
 *
 * The firmware is the raw contents of the "radio" partition: a 0x200-byte
 * TOC followed by the sections it describes. The NV data (nv_data.bin
 * from efs) is a second firmware file, the second "firmware-name" entry.
 *
 * Restarting follows the vendor stack (cbd's start_shannon310_boot and
 * modem_ctrl_ss310ap): stop() asserts the CP reset; a later load() asks
 * EL3 for CP_BOOT_RE_INIT before staging again (the carve-out is
 * presumably firewalled after a secure boot, and cbd always issues
 * RE_INIT before transferring images), and start() releases the reset
 * (cp_reset_release[]) instead of the first-boot cp_init[] when the PMU
 * reset sequencer says the CP has been initialised before.
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox/exynos-mcu-ipc.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/remoteproc.h>
#include <linux/samsung-sbd.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "remoteproc_internal.h"

/* EL3 CP service: secure boot (modes 0..2) and CP_CTRL_S/NS access */
#define CP_SMC_ID			0x82000700
#define CP_SMC_BOOT_NORMAL		0x0
#define CP_SMC_BOOT_RE_INIT		0x2
#define CP_SMC_READ_CTRL		0x3
#define CP_SMC_WRITE_CTRL		0x4
#define CP_SMC_REG_CTRL_S		0
#define CP_SMC_REG_CTRL_NS		1

/* the Security Sub System clock must run across a secure-boot call */
#define CP_SMC_CLK_ID			0xc2001011
#define SSS_CLK_ENABLE			0
#define SSS_CLK_DISABLE			1

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

/* PMU_ALIVE, through the pmu syscon. CP_CTRL_NS/S only via the SMC above. */
#define CP_STAT				0x0038
#define CENTRAL_SEQ_CP_CONFIGURATION	0x0280
#define CENTRAL_SEQ_CP_STATUS		0x0284
#define RESET_SEQUENCER_STATUS		0x0504
#define RESET_AHEAD_CP_SYS_PWR_REG	0x1320
#define CLEANY_BUS_CP_SYS_PWR_REG	0x1324
#define LOGIC_RESET_CP_SYS_PWR_REG	0x1328
#define TCXO_GATE_CP_SYS_PWR_REG	0x132c
#define CP_DISABLE_ISO_SYS_PWR_REG	0x1330
#define CP_RESET_ISO_SYS_PWR_REG	0x1334
#define CP_DUMP_PC_SYS_PWR_REG		0x1338

#define CP_PWRON			BIT(1)
#define CP_START			BIT(3)
#define CP_RESET_SEQ_STAT_MASK		GENMASK(2, 0)
#define CP_RESET_SEQ_STAT_DONE		0x5
#define CENTRAL_SEQ_CP_STATES		GENMASK(23, 16)
#define CENTRAL_SEQ_CP_STATES_DOWN	0x80
#define CENTRAL_SEQ_CP_STATES_RUN	0x00
#define CP_SWEEPER_BYPASS_DATA_EN	BIT(16)
#define CLEANY_BUS_CP_CONFIGURATION	0x3920
#define CLEANY_BUS_CP_STATUS		0x3924
#define CLEANY_BUS_CP_REQ		BIT(0)
#define CLEANY_BUS_CP_STATUS_MASK	GENMASK(17, 16)

/*
 * Mailbox shared registers (ISSRn) and their fields, from the stock
 * device tree's modem node (mbx_* / sbi_* properties).
 */
#define MBX_AP2CP_MSG			0
#define MBX_CP2AP_MSG			1
#define MBX_AP2CP_STATUS		2
#define MBX_CP2AP_STATUS		3

#define AP2CP_CP_WAKEUP			BIT(0)
#define AP2CP_AP_STATUS			GENMASK(4, 1)
#define AP2CP_PDA_ACTIVE		BIT(5)
#define AP2CP_SYS_REV			GENMASK(13, 6)
#define AP2CP_DS_DET			GENMASK(15, 14)
#define CP2AP_CP_STATUS			GENMASK(4, 1)

/* messages: MASK_INT_VALID | MASK_CMD_VALID | CMD_* */
#define CP_MSG_INT_VALID		BIT(7)
#define CP_MSG_CMD_VALID		BIT(6)
#define CP_MSG_CMD			GENMASK(5, 0)
#define CMD_INIT_START			0x01
#define CMD_CRASH_RESET			0x07
#define CMD_CRASH_EXIT			0x09
#define CP_MSG(cmd)			(CP_MSG_INT_VALID | CP_MSG_CMD_VALID | (cmd))

/*
 * Legacy layout at the start of the IPC region (struct
 * shmem_4mb_phys_map), used by the CP bootloader for the boot handshake.
 */
#define IPC_MAGIC			0x00
#define IPC_RAW_TX_HEAD			0x18
#define IPC_RAW_TX_TAIL			0x1c
#define IPC_RAW_RX_HEAD			0x20
#define IPC_RAW_RX_TAIL			0x24
#define IPC_BOOTLOG			0xc00	/* u32 length, then text */
#define IPC_RAW_TX_BUF			0x3000
#define IPC_RAW_TX_SIZE			0x1fd000
#define IPC_RAW_RX_BUF			0x200000
#define IPC_RAW_RX_SIZE			0x200000

#define IPC_MAGIC_BOOT			0x424f4f54	/* "BOOT" */

/* boot channel frame: SIPC5 START | PADDING, channel BOOT_0, u32 payload */
#define BOOT_FRAME_CFG			0xfc
#define BOOT_FRAME_CH			0xd7
#define BOOT_FRAME_LEN			8

#define BOOT_REQ_900D			0x900d
#define BOOT_REQ_9F00			0x9f00

#define TOC_ENTRY_SIZE			32
#define TOC_SIZE			0x200

struct cp_toc_entry {
	char name[12];
	__le32 offset;		/* in the image */
	__le32 load;		/* CP address */
	__le32 size;
	__le32 crc;
	__le32 index;
};

struct exynos_cp_data {
	phys_addr_t cp_base;	/* CP address of the start of the carve-out */
	u32 ipc_offset;
	u32 ipc_size;
};

struct exynos_cp {
	struct device *dev;
	struct rproc *rproc;
	const struct exynos_cp_data *data;

	struct regmap *pmu;
	struct regmap *mbox_regs;
	struct regulator *shared_reg;
	bool shared_reg_kept;
	struct mbox_client mbox_cl;
	struct mbox_chan *mbox_msg;
	struct mbox_client mbox_wakeup_cl;
	struct mbox_chan *mbox_wakeup;	/* optional */

	phys_addr_t mem_phys;
	size_t mem_size;
	void *ipc;		/* write-combine view of the IPC region */

	const char *nv_name;
	u32 sys_rev;
	u32 ds_det;

	/* from the TOC, for the secure-boot call */
	u32 boot_size;
	u32 main_size;

	atomic_t rx_count;

	struct sbd_link *link;
	struct rproc_subdev link_subdev;
	/* serializes AP->CP messages */
	struct mutex msg_lock;
};

static const char *cp_smc_err_name(unsigned long err)
{
	return err < ARRAY_SIZE(cp_smc_err_string) ? cp_smc_err_string[err]
						  : "UNKNOWN";
}

static unsigned long cp_smc(unsigned long fn, unsigned long arg,
			    unsigned long reg)
{
	struct arm_smccc_res res;

	arm_smccc_smc(CP_SMC_ID, fn, arg, reg, 0, 0, 0, 0, &res);

	return res.a0;
}

/*
 * READ_CTRL returns one 16-bit half per call, in the upper half of a0;
 * a non-zero low half is an error code.
 */
static int cp_ctrl_read(struct exynos_cp *cp, unsigned long reg, u32 *out)
{
	unsigned long lo, hi;

	lo = cp_smc(CP_SMC_READ_CTRL, 0, reg);
	if (lo & 0xffff)
		return -EIO;

	hi = cp_smc(CP_SMC_READ_CTRL, 1, reg);
	if (hi & 0xffff)
		return -EIO;

	*out = (u32)(((hi >> 16) << 16) | (lo >> 16));

	return 0;
}

static int cp_ctrl_update_bits(struct exynos_cp *cp, unsigned long reg,
			       u32 mask, u32 val)
{
	unsigned long ret;
	u32 old;

	if (cp_ctrl_read(cp, reg, &old)) {
		dev_err(cp->dev, "CP_CTRL reg%lu: read failed\n", reg);
		return -EIO;
	}

	ret = cp_smc(CP_SMC_WRITE_CTRL, (old & ~mask) | (val & mask), reg);
	if ((long)ret > 0) {
		dev_err(cp->dev, "CP_CTRL reg%lu: write failed: %ld\n",
			reg, (long)ret);
		return -EIO;
	}

	return 0;
}

/*
 * exynos_pmu_shared_reg_enable()/_disable() from downstream mcu_ipc.c:
 * the regulator shared with the GNSS and WLBT cores, plus the CP_WAKEUP
 * bit. The mailbox provides it as a regulator, so the vote is counted
 * against gnss_kepler's.
 */
static int cp_shared_reg_enable(struct exynos_cp *cp)
{
	int ret = regulator_enable(cp->shared_reg);

	if (ret)
		dev_err(cp->dev, "shared regulator: %d\n", ret);

	return ret;
}

static void cp_shared_reg_disable(struct exynos_cp *cp)
{
	regulator_disable(cp->shared_reg);
}

/*
 * EL3 only allows CP_CTRL access once a CP image has been secure-booted
 * (it answers CP_NOT_WORKING before that), so a readable CP_CTRL_NS means
 * this boot has already used up its one secure boot.
 */
static bool cp_already_booted(struct exynos_cp *cp)
{
	u32 val;
	int ret;

	if (cp_shared_reg_enable(cp))
		return true;
	ret = cp_ctrl_read(cp, CP_SMC_REG_CTRL_NS, &val);
	cp_shared_reg_disable(cp);

	return !ret;
}

/*
 * cp_reset_release[] from downstream pmucal_cp_exynos7885.h: what
 * ss310ap_on() runs instead of cp_init[] for a CP that was initialised
 * before (pmucal_cp_status(): RESET_SEQUENCER_STATUS[2:0] == 5) and has
 * since been through cp_reset_assert[].
 */
static int cp_reset_release(struct exynos_cp *cp)
{
	u32 val;
	int ret, err;

	ret = cp_ctrl_update_bits(cp, CP_SMC_REG_CTRL_NS,
				  CP_SWEEPER_BYPASS_DATA_EN,
				  CP_SWEEPER_BYPASS_DATA_EN);
	if (ret)
		return ret;

	ret = cp_ctrl_update_bits(cp, CP_SMC_REG_CTRL_NS, CP_PWRON, CP_PWRON);
	if (!ret)
		ret = cp_ctrl_update_bits(cp, CP_SMC_REG_CTRL_S, CP_START,
					  CP_START);
	if (ret)
		goto out;

	ret = regmap_read_poll_timeout(cp->pmu, CENTRAL_SEQ_CP_STATUS, val,
				       FIELD_GET(CENTRAL_SEQ_CP_STATES, val) ==
				       CENTRAL_SEQ_CP_STATES_RUN,
				       50, 50 * USEC_PER_MSEC);
	if (ret) {
		dev_err(cp->dev, "CP did not leave reset (0x%08x)\n", val);
		goto out;
	}

	regmap_set_bits(cp->pmu, CLEANY_BUS_CP_CONFIGURATION, CLEANY_BUS_CP_REQ);
	ret = regmap_read_poll_timeout(cp->pmu, CLEANY_BUS_CP_STATUS, val,
				       !(val & CLEANY_BUS_CP_STATUS_MASK),
				       50, 50 * USEC_PER_MSEC);
	if (ret)
		dev_err(cp->dev, "CP bus not clean (0x%08x)\n", val);

out:
	err = cp_ctrl_update_bits(cp, CP_SMC_REG_CTRL_NS,
				  CP_SWEEPER_BYPASS_DATA_EN, 0);

	return ret ?: err;
}

/* cp_init[], or cp_reset_release[] for a CP that ran before */
static int cp_power_on(struct exynos_cp *cp)
{
	u32 val;
	int ret;

	ret = cp_shared_reg_enable(cp);
	if (ret)
		return ret;

	regmap_read(cp->pmu, RESET_SEQUENCER_STATUS, &val);
	if ((val & CP_RESET_SEQ_STAT_MASK) == CP_RESET_SEQ_STAT_DONE) {
		dev_info(cp->dev, "CP initialised before, releasing its reset\n");
		ret = cp_reset_release(cp);
		goto out;
	}

	ret = cp_ctrl_update_bits(cp, CP_SMC_REG_CTRL_NS, CP_PWRON, CP_PWRON);
	if (ret)
		goto out;

	ret = cp_ctrl_update_bits(cp, CP_SMC_REG_CTRL_S, CP_START, CP_START);
	if (ret)
		goto out;

	ret = regmap_read_poll_timeout(cp->pmu, RESET_SEQUENCER_STATUS, val,
				       (val & CP_RESET_SEQ_STAT_MASK) ==
				       CP_RESET_SEQ_STAT_DONE,
				       50, 50 * USEC_PER_MSEC);
	if (ret)
		dev_err(cp->dev, "reset sequencer stuck (0x%08x)\n", val);

out:
	cp_shared_reg_disable(cp);

	return ret;
}

static int cp_secure_boot(struct exynos_cp *cp)
{
	struct arm_smccc_res res;

	arm_smccc_smc(CP_SMC_CLK_ID, SSS_CLK_ENABLE, 0, 0, 0, 0, 0, 0, &res);
	arm_smccc_smc(CP_SMC_ID, CP_SMC_BOOT_NORMAL, cp->boot_size,
		      cp->main_size, 0, 0, 0, 0, &res);
	arm_smccc_smc(CP_SMC_CLK_ID, SSS_CLK_DISABLE, 0, 0, 0, 0, 0, 0,
		      &(struct arm_smccc_res){});

	if (res.a0) {
		dev_err(cp->dev, "secure boot failed: 0x%lx (%s)\n", res.a0,
			cp_smc_err_name(res.a0));
		return -EIO;
	}

	return 0;
}

/*
 * CP_BOOT_RE_INIT: what cbd asks EL3 for before every image transfer. On
 * a first boot EL3 answers CP_NOT_WORKING (nothing to re-init); after a
 * secure boot it has to succeed before the carve-out may be written.
 */
static int cp_secure_reinit(struct exynos_cp *cp)
{
	struct arm_smccc_res res;

	arm_smccc_smc(CP_SMC_CLK_ID, SSS_CLK_ENABLE, 0, 0, 0, 0, 0, 0, &res);
	arm_smccc_smc(CP_SMC_ID, CP_SMC_BOOT_RE_INIT, 0, 0, 0, 0, 0, 0, &res);
	arm_smccc_smc(CP_SMC_CLK_ID, SSS_CLK_DISABLE, 0, 0, 0, 0, 0, 0,
		      &(struct arm_smccc_res){});

	if (res.a0) {
		dev_err(cp->dev, "secure re-init failed: 0x%lx (%s)\n", res.a0,
			cp_smc_err_name(res.a0));
		return -EIO;
	}

	dev_info(cp->dev, "secure re-init done\n");

	return 0;
}

static u32 ipc_read(struct exynos_cp *cp, u32 off)
{
	return READ_ONCE(*(u32 *)(cp->ipc + off));
}

static void ipc_write(struct exynos_cp *cp, u32 off, u32 val)
{
	WRITE_ONCE(*(u32 *)(cp->ipc + off), val);
}

static void cp_dump_bootlog(struct exynos_cp *cp)
{
	u32 len = ipc_read(cp, IPC_BOOTLOG);
	char *buf;

	len = min_t(u32, len, 512);
	buf = kmemdup_nul(cp->ipc + IPC_BOOTLOG + 4, len, GFP_KERNEL);
	if (!buf)
		return;

	dev_err(cp->dev, "CP bootloader log: %s\n", buf);
	kfree(buf);
}

/* one request/response round on the bootloader's raw boot queues */
static int cp_boot_request(struct exynos_cp *cp, u32 req, u32 resp)
{
	u32 head, tail, hdr, val, cfg, len;
	unsigned long timeout;

	head = ipc_read(cp, IPC_RAW_TX_HEAD);
	if (head >= IPC_RAW_TX_SIZE || head % 4)
		return -EIO;

	hdr = BOOT_FRAME_CFG | BOOT_FRAME_CH << 8 | BOOT_FRAME_LEN << 16;
	ipc_write(cp, IPC_RAW_TX_BUF + head, hdr);
	ipc_write(cp, IPC_RAW_TX_BUF + (head + 4) % IPC_RAW_TX_SIZE, req);
	wmb();	/* frame before head */
	ipc_write(cp, IPC_RAW_TX_HEAD, (head + BOOT_FRAME_LEN) % IPC_RAW_TX_SIZE);
	wmb();	/* drain the write-combine buffer */

	timeout = jiffies + 5 * HZ;
	for (;;) {
		head = ipc_read(cp, IPC_RAW_RX_HEAD);
		tail = ipc_read(cp, IPC_RAW_RX_TAIL);
		if (head != tail)
			break;
		if (time_after(jiffies, timeout)) {
			dev_err(cp->dev, "boot request 0x%04x: no response\n",
				req);
			return -ETIMEDOUT;
		}
		usleep_range(1000, 2000);
	}

	if (head >= IPC_RAW_RX_SIZE || tail >= IPC_RAW_RX_SIZE || tail % 4)
		return -EIO;

	rmb();	/* head before frame */
	hdr = ipc_read(cp, IPC_RAW_RX_BUF + tail);
	val = ipc_read(cp, IPC_RAW_RX_BUF + (tail + 4) % IPC_RAW_RX_SIZE);
	cfg = hdr & 0xff;
	len = hdr >> 16;
	if (len < 4 || len > SZ_64K)
		return -EIO;
	if (cfg & 0x04)		/* padding to 4 bytes */
		len = ALIGN(len, 4);
	ipc_write(cp, IPC_RAW_RX_TAIL, (tail + len) % IPC_RAW_RX_SIZE);
	wmb();

	if (val != resp) {
		dev_err(cp->dev, "boot request 0x%04x: got 0x%08x, want 0x%04x\n",
			req, val, resp);
		return -EIO;
	}

	return 0;
}

static void exynos_cp_rx(struct mbox_client *cl, void *msg)
{
	struct exynos_cp *cp = container_of(cl, struct exynos_cp, mbox_cl);
	u32 val, cmd;

	atomic_inc(&cp->rx_count);
	regmap_read(cp->mbox_regs, EXYNOS_MCU_IPC_ISSR(MBX_CP2AP_MSG), &val);
	dev_dbg(cp->dev, "CP message 0x%02x\n", val);

	sbd_link_notify(cp->link, val);

	if ((val & (CP_MSG_INT_VALID | CP_MSG_CMD_VALID)) !=
	    (CP_MSG_INT_VALID | CP_MSG_CMD_VALID))
		return;

	cmd = val & CP_MSG_CMD;
	if (cmd == CMD_CRASH_RESET || cmd == CMD_CRASH_EXIT) {
		dev_err(cp->dev, "CP crashed (message 0x%02x)\n", val);
		rproc_report_crash(cp->rproc, RPROC_FATAL_ERROR);
	}
}

/*
 * mcu_ipc_send_command() downstream: message register, then the doorbell,
 * inside the shared-regulator bracket. A sleeping CP needs the regulator
 * to stay up for 3 ms after the doorbell.
 */
static void exynos_cp_send_msg(void *ctx, u32 msg)
{
	struct exynos_cp *cp = ctx;
	u32 status;
	int ret;

	mutex_lock(&cp->msg_lock);

	if (cp_shared_reg_enable(cp)) {
		mutex_unlock(&cp->msg_lock);
		return;
	}
	regmap_read(cp->mbox_regs, EXYNOS_MCU_IPC_ISSR(MBX_CP2AP_STATUS),
		    &status);

	regmap_write(cp->mbox_regs, EXYNOS_MCU_IPC_ISSR(MBX_AP2CP_MSG), msg);
	ret = mbox_send_message(cp->mbox_msg, NULL);
	if (ret < 0)
		dev_err(cp->dev, "failed to ring the doorbell: %d\n", ret);
	else
		mbox_client_txdone(cp->mbox_msg, 0);

	if (!FIELD_GET(CP2AP_CP_STATUS, status))
		usleep_range(3000, 3500);
	cp_shared_reg_disable(cp);

	mutex_unlock(&cp->msg_lock);
}

static const struct sbd_link_ops exynos_cp_link_ops = {
	.send_msg = exynos_cp_send_msg,
};

static int exynos_cp_link_start(struct rproc_subdev *subdev)
{
	struct exynos_cp *cp = container_of(subdev, struct exynos_cp,
					    link_subdev);

	return sbd_link_start(cp->link);
}

static void exynos_cp_link_stop(struct rproc_subdev *subdev, bool crashed)
{
	struct exynos_cp *cp = container_of(subdev, struct exynos_cp,
					    link_subdev);

	sbd_link_stop(cp->link);
}

static int exynos_cp_start(struct rproc *rproc)
{
	struct exynos_cp *cp = rproc->priv;
	u32 val;
	int ret, n;

	ret = cp_secure_boot(cp);
	if (ret)
		return ret;

	/*
	 * init_mailbox_regs(): all shared registers cleared, then the AP
	 * status word. ap_status = 1 is what the CP bootloader waits for.
	 * CP_WAKEUP belongs to the shared regulator, whose other users may
	 * hold it: leave it alone.
	 */
	for (n = 0; n < EXYNOS_MCU_IPC_NUM_ISSR; n++)
		if (n != MBX_AP2CP_STATUS)
			regmap_write(cp->mbox_regs, EXYNOS_MCU_IPC_ISSR(n), 0);
	regmap_update_bits(cp->mbox_regs, EXYNOS_MCU_IPC_ISSR(MBX_AP2CP_STATUS),
			   (u32)~AP2CP_CP_WAKEUP,
			   FIELD_PREP(AP2CP_AP_STATUS, 1) | AP2CP_PDA_ACTIVE |
			   FIELD_PREP(AP2CP_SYS_REV, cp->sys_rev) |
			   FIELD_PREP(AP2CP_DS_DET, cp->ds_det));

	/*
	 * The bootloader reads the magic as soon as it runs, so it has to be
	 * in place before power-on: DRAM still holds whatever the previous
	 * boot left there.
	 */
	for (n = 0x08; n <= IPC_RAW_RX_TAIL; n += 4)
		ipc_write(cp, n, 0);
	ipc_write(cp, IPC_MAGIC, IPC_MAGIC_BOOT);
	wmb();

	ret = cp_power_on(cp);
	if (ret)
		return ret;

	ret = cp_boot_request(cp, BOOT_REQ_900D, 0xa00d);
	if (!ret)
		ret = cp_boot_request(cp, BOOT_REQ_9F00, 0xaf00);
	if (ret) {
		cp_dump_bootlog(cp);
		return ret;
	}

	/* the main firmware is up once it asks for the IPC link */
	ret = regmap_read_poll_timeout(cp->mbox_regs,
				       EXYNOS_MCU_IPC_ISSR(MBX_CP2AP_MSG), val,
				       val == CP_MSG(CMD_INIT_START),
				       10 * USEC_PER_MSEC, 10 * USEC_PER_SEC);
	if (ret) {
		dev_err(cp->dev, "no INIT_START from the CP (last message 0x%02x)\n",
			val);
		cp_dump_bootlog(cp);
		return ret;
	}

	dev_info(cp->dev, "CP main firmware running, %d mailbox interrupts so far\n",
		 atomic_read(&cp->rx_count));

	return 0;
}

/* cp_reset_assert[] from downstream pmucal_cp_exynos7885.h */
static int cp_power_off(struct exynos_cp *cp)
{
	u32 val;
	int ret;

	regmap_clear_bits(cp->pmu, CENTRAL_SEQ_CP_CONFIGURATION, BIT(16));
	regmap_clear_bits(cp->pmu, RESET_AHEAD_CP_SYS_PWR_REG, GENMASK(1, 0));
	regmap_clear_bits(cp->pmu, CLEANY_BUS_CP_SYS_PWR_REG, BIT(0));
	regmap_clear_bits(cp->pmu, CP_DUMP_PC_SYS_PWR_REG, BIT(0));
	regmap_clear_bits(cp->pmu, LOGIC_RESET_CP_SYS_PWR_REG, GENMASK(1, 0));
	regmap_clear_bits(cp->pmu, TCXO_GATE_CP_SYS_PWR_REG, BIT(0));
	regmap_set_bits(cp->pmu, CP_DISABLE_ISO_SYS_PWR_REG, BIT(0));
	regmap_clear_bits(cp->pmu, CP_RESET_ISO_SYS_PWR_REG, BIT(0));

	ret = cp_ctrl_update_bits(cp, CP_SMC_REG_CTRL_NS, CP_PWRON, 0);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(cp->pmu, CENTRAL_SEQ_CP_STATUS, val,
				       FIELD_GET(CENTRAL_SEQ_CP_STATES, val) ==
				       CENTRAL_SEQ_CP_STATES_DOWN,
				       50, 50 * USEC_PER_MSEC);
	if (ret)
		dev_err(cp->dev, "CP did not power down (0x%08x)\n", val);

	return ret;
}

static void cp_dump_power(struct exynos_cp *cp, const char *when)
{
	u32 stat, seq, rst, ns = 0, s = 0;
	int ns_ret, s_ret;

	regmap_read(cp->pmu, CP_STAT, &stat);
	regmap_read(cp->pmu, CENTRAL_SEQ_CP_STATUS, &seq);
	regmap_read(cp->pmu, RESET_SEQUENCER_STATUS, &rst);
	ns_ret = cp_ctrl_read(cp, CP_SMC_REG_CTRL_NS, &ns);
	s_ret = cp_ctrl_read(cp, CP_SMC_REG_CTRL_S, &s);

	dev_dbg(cp->dev,
		"%s: CP_STAT 0x%08x CENTRAL_SEQ_CP_STATUS 0x%08x RESET_SEQUENCER_STATUS 0x%08x CP_CTRL_NS %s0x%08x CP_CTRL_S %s0x%08x\n",
		when, stat, seq, rst, ns_ret ? "(err) " : "", ns,
		s_ret ? "(err) " : "", s);
}

/*
 * Take a running CP down the way downstream's ss310ap_reset() does: wake
 * it, then cp_reset_assert[] with the shared regulator on. Downstream
 * leaves the regulator on afterwards; so does this, by keeping one vote.
 */
static int exynos_cp_power_down(struct exynos_cp *cp)
{
	int ret;

	sbd_link_stop(cp->link);

	if (!cp->shared_reg_kept) {
		ret = cp_shared_reg_enable(cp);
		if (ret)
			return ret;
		cp->shared_reg_kept = true;
	}
	cp_dump_power(cp, "before power-down");

	if (cp->mbox_wakeup) {
		mbox_send_message(cp->mbox_wakeup, NULL);
		mbox_client_txdone(cp->mbox_wakeup, 0);
		msleep(5);
	}

	ret = cp_power_off(cp);
	cp_dump_power(cp, "after power-down");

	return ret;
}

/* Powers the CP down; see the header comment for starting it again. */
static int exynos_cp_stop(struct rproc *rproc)
{
	return exynos_cp_power_down(rproc->priv);
}

static int exynos_cp_sanity_check(struct rproc *rproc,
				  const struct firmware *fw)
{
	if (fw->size < TOC_SIZE || memcmp(fw->data, "TOC", 4)) {
		dev_err(&rproc->dev, "%s: no TOC, not a radio image\n",
			rproc->firmware);
		return -EINVAL;
	}

	return 0;
}

static const struct cp_toc_entry *cp_toc_find(const struct firmware *fw,
					      const char *name)
{
	const struct cp_toc_entry *e = (const void *)fw->data;
	int i;

	for (i = 1; i < TOC_SIZE / TOC_ENTRY_SIZE; i++)
		if (!strncmp(e[i].name, name, sizeof(e[i].name)))
			return &e[i];

	return NULL;
}

/* -> offset of a TOC section in the carve-out, after bounds checks */
static int cp_section_offset(struct exynos_cp *cp,
			     const struct cp_toc_entry *e, size_t *off)
{
	u64 load = le32_to_cpu(e->load), size = le32_to_cpu(e->size);

	if (load < cp->data->cp_base ||
	    load - cp->data->cp_base + size > cp->data->ipc_offset) {
		dev_err(cp->dev, "TOC %.12s: 0x%llx+0x%llx outside the carve-out\n",
			e->name, load, size);
		return -EINVAL;
	}

	*off = load - cp->data->cp_base;

	return 0;
}

static int cp_copy_section(struct exynos_cp *cp, void *mem,
			   const struct cp_toc_entry *e,
			   const void *src, size_t len)
{
	dma_addr_t dma;
	size_t off;
	int ret;

	ret = cp_section_offset(cp, e, &off);
	if (ret)
		return ret;

	memcpy(mem + off, src, len);

	/*
	 * EL3 and the CP read DRAM, not this cacheable mapping: clean what was
	 * written to the point of coherency. A DMA_TO_DEVICE mapping on this
	 * non-coherent device does exactly that.
	 */
	dma = dma_map_single(cp->dev, mem + off, len, DMA_TO_DEVICE);
	if (dma_mapping_error(cp->dev, dma))
		return -EIO;
	dma_unmap_single(cp->dev, dma, len, DMA_TO_DEVICE);

	dev_dbg(cp->dev, "%.12s: 0x%zx bytes at +0x%zx\n", e->name, len, off);

	return 0;
}

static int exynos_cp_load(struct rproc *rproc, const struct firmware *fw)
{
	static const char * const sections[] = { "BOOT", "MAIN", "VSS" };
	struct exynos_cp *cp = rproc->priv;
	const struct cp_toc_entry *e;
	const struct firmware *nv;
	void *mem;
	int i, ret;

	/*
	 * Secure-booted before on this boot: the carve-out must not be
	 * written until EL3 has re-initialised it. If it refuses, stop here
	 * rather than fault on the first write.
	 */
	if (cp_already_booted(cp)) {
		ret = cp_secure_reinit(cp);
		if (ret)
			return ret;
	}

	mem = memremap(cp->mem_phys, cp->data->ipc_offset, MEMREMAP_WB);
	if (!mem)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(sections); i++) {
		u32 off, size;

		e = cp_toc_find(fw, sections[i]);
		if (!e) {
			dev_err(cp->dev, "TOC has no %s\n", sections[i]);
			ret = -EINVAL;
			goto out;
		}

		off = le32_to_cpu(e->offset);
		size = le32_to_cpu(e->size);
		if ((u64)off + size > fw->size) {
			dev_err(cp->dev, "%s: 0x%x+0x%x past the end of %s\n",
				sections[i], off, size, rproc->firmware);
			ret = -EINVAL;
			goto out;
		}

		ret = cp_copy_section(cp, mem, e, fw->data + off, size);
		if (ret)
			goto out;

		if (i == 0)
			cp->boot_size = size;
		else if (i == 1)
			cp->main_size = size;
	}

	/* the NV section is not in the image: its TOC entry has offset 0 */
	e = cp_toc_find(fw, "NV");
	if (!e) {
		dev_err(cp->dev, "TOC has no NV\n");
		ret = -EINVAL;
		goto out;
	}

	ret = request_firmware(&nv, cp->nv_name, cp->dev);
	if (ret) {
		dev_err(cp->dev, "failed to load NV data %s: %d\n",
			cp->nv_name, ret);
		goto out;
	}

	ret = cp_copy_section(cp, mem, e, nv->data,
			      min_t(size_t, nv->size, le32_to_cpu(e->size)));
	release_firmware(nv);

out:
	memunmap(mem);

	return ret;
}

static const struct rproc_ops exynos_cp_ops = {
	.start = exynos_cp_start,
	.stop = exynos_cp_stop,
	.load = exynos_cp_load,
	.sanity_check = exynos_cp_sanity_check,
};

/*
 * Debugging: raw access to the IPC region, e.g. to dump ring pointers
 * while the link runs.
 */
static ssize_t ipc_mem_read(struct file *file, char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct exynos_cp *cp = file->private_data;

	return simple_read_from_buffer(buf, count, ppos, cp->ipc,
				       cp->data->ipc_size);
}

static ssize_t ipc_mem_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct exynos_cp *cp = file->private_data;
	ssize_t ret;

	ret = simple_write_to_buffer(cp->ipc, cp->data->ipc_size, ppos, buf,
				     count);
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

/*
 * The IPC region is polled by the CP and both sides update words that
 * share cache lines, so map it write-combine like downstream does. It is
 * RAM in the linear map, which ioremap_wc() refuses, hence vmap().
 */
static void *exynos_cp_map_ipc(struct exynos_cp *cp)
{
	unsigned int i, n = cp->data->ipc_size >> PAGE_SHIFT;
	unsigned long pfn = PHYS_PFN(cp->mem_phys + cp->data->ipc_offset);
	struct page **pages;
	void *va;

	pages = kvmalloc_array(n, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return NULL;

	for (i = 0; i < n; i++)
		pages[i] = pfn_to_page(pfn + i);

	va = vmap(pages, n, VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	kvfree(pages);

	return va;
}

static void exynos_cp_unmap_ipc(void *va)
{
	vunmap(va);
}

/* the vote power-down keeps (see exynos_cp_power_down()) */
static void exynos_cp_drop_kept_vote(void *data)
{
	struct exynos_cp *cp = data;

	if (cp->shared_reg_kept)
		cp_shared_reg_disable(cp);
}

static void exynos_cp_free_mbox(void *chan)
{
	mbox_free_channel(chan);
}

static void exynos_cp_destroy_link(void *link)
{
	sbd_link_destroy(link);
}

static int exynos_cp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct of_phandle_args args;
	struct reserved_mem *rmem;
	struct device_node *mem_np;
	struct exynos_cp *cp;
	const char *fw_name;
	struct rproc *rproc;
	int ret;

	ret = rproc_of_parse_firmware(dev, 0, &fw_name);
	if (ret)
		return dev_err_probe(dev, ret, "no firmware-name\n");

	rproc = devm_rproc_alloc(dev, "cp", &exynos_cp_ops, fw_name,
				 sizeof(*cp));
	if (!rproc)
		return -ENOMEM;

	/* started by userspace (shannond), once the RFS server runs */
	rproc->auto_boot = false;
	rproc->recovery_disabled = true;

	cp = rproc->priv;
	cp->dev = dev;
	cp->rproc = rproc;
	cp->data = of_device_get_match_data(dev);
	atomic_set(&cp->rx_count, 0);
	mutex_init(&cp->msg_lock);

	ret = of_property_read_string_index(np, "firmware-name", 1,
					    &cp->nv_name);
	if (ret)
		return dev_err_probe(dev, ret, "no NV firmware-name\n");

	of_property_read_u32(np, "samsung,sys-rev", &cp->sys_rev);
	of_property_read_u32(np, "samsung,ds-det", &cp->ds_det);

	mem_np = of_parse_phandle(np, "memory-region", 0);
	if (!mem_np)
		return dev_err_probe(dev, -EINVAL, "no memory-region\n");
	rmem = of_reserved_mem_lookup(mem_np);
	of_node_put(mem_np);
	if (!rmem)
		return dev_err_probe(dev, -EINVAL, "memory-region not reserved\n");

	cp->mem_phys = rmem->base;
	cp->mem_size = rmem->size;
	if ((u64)cp->data->ipc_offset + cp->data->ipc_size > cp->mem_size)
		return dev_err_probe(dev, -EINVAL, "carve-out too small\n");

	cp->pmu = syscon_regmap_lookup_by_phandle(np, "samsung,syscon-phandle");
	if (IS_ERR(cp->pmu))
		return dev_err_probe(dev, PTR_ERR(cp->pmu), "no PMU syscon\n");

	cp->shared_reg = devm_regulator_get(dev, "shared");
	if (IS_ERR(cp->shared_reg))
		return dev_err_probe(dev, PTR_ERR(cp->shared_reg),
				     "no shared regulator\n");
	/* runs before the regulator is put: devm actions unwind in reverse */
	ret = devm_add_action_or_reset(dev, exynos_cp_drop_kept_vote, cp);
	if (ret)
		return ret;

	/* the mailbox's shared registers: a syscon on the mailbox node */
	ret = of_parse_phandle_with_args(np, "mboxes", "#mbox-cells", 0, &args);
	if (ret)
		return dev_err_probe(dev, ret, "no mboxes\n");
	cp->mbox_regs = syscon_node_to_regmap(args.np);
	of_node_put(args.np);
	if (IS_ERR(cp->mbox_regs))
		return dev_err_probe(dev, PTR_ERR(cp->mbox_regs),
				     "no mailbox registers\n");

	cp->ipc = exynos_cp_map_ipc(cp);
	if (!cp->ipc)
		return dev_err_probe(dev, -ENOMEM, "failed to map IPC region\n");
	ret = devm_add_action_or_reset(dev, exynos_cp_unmap_ipc, cp->ipc);
	if (ret)
		return ret;

	cp->mbox_cl.dev = dev;
	cp->mbox_cl.rx_callback = exynos_cp_rx;
	cp->mbox_cl.knows_txdone = true;
	cp->mbox_msg = mbox_request_channel_byname(&cp->mbox_cl, "msg");
	if (IS_ERR(cp->mbox_msg))
		return dev_err_probe(dev, PTR_ERR(cp->mbox_msg),
				     "no msg mailbox channel\n");
	ret = devm_add_action_or_reset(dev, exynos_cp_free_mbox, cp->mbox_msg);
	if (ret)
		return ret;

	cp->mbox_wakeup_cl.dev = dev;
	cp->mbox_wakeup_cl.knows_txdone = true;
	cp->mbox_wakeup = mbox_request_channel_byname(&cp->mbox_wakeup_cl,
						      "wakeup");
	if (IS_ERR(cp->mbox_wakeup)) {
		if (PTR_ERR(cp->mbox_wakeup) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		cp->mbox_wakeup = NULL;
	} else {
		ret = devm_add_action_or_reset(dev, exynos_cp_free_mbox,
					       cp->mbox_wakeup);
		if (ret)
			return ret;
	}

	cp->link = sbd_link_create(dev, cp->ipc, cp->data->ipc_size,
				   &exynos_cp_link_ops, cp);
	if (IS_ERR(cp->link))
		return dev_err_probe(dev, PTR_ERR(cp->link),
				     "failed to create the SBD link\n");
	ret = devm_add_action_or_reset(dev, exynos_cp_destroy_link, cp->link);
	if (ret)
		return ret;

	cp->link_subdev.start = exynos_cp_link_start;
	cp->link_subdev.stop = exynos_cp_link_stop;
	rproc_add_subdev(rproc, &cp->link_subdev);

	platform_set_drvdata(pdev, rproc);

	ret = devm_rproc_add(dev, rproc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add rproc\n");

	debugfs_create_file("ipc_mem", 0600, rproc->dbg_dir, cp, &ipc_mem_fops);

	return 0;
}

static const struct exynos_cp_data exynos7885_cp_data = {
	.cp_base = 0x40000000,
	.ipc_offset = 0x7000000,
	.ipc_size = 0x800000,
};

static const struct of_device_id exynos_cp_of_match[] = {
	{ .compatible = "samsung,exynos7885-cp", .data = &exynos7885_cp_data },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_cp_of_match);

static struct platform_driver exynos_cp_driver = {
	.probe = exynos_cp_probe,
	.driver = {
		.name = "exynos-cp-rproc",
		.of_match_table = exynos_cp_of_match,
	},
};
module_platform_driver(exynos_cp_driver);

MODULE_DESCRIPTION("Samsung Exynos7885 modem (CP) remoteproc driver");
MODULE_LICENSE("GPL");
