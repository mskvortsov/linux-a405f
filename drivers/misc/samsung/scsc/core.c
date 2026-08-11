// SPDX-License-Identifier: GPL-2.0-only
/*
 * Core driver for the Samsung SCSC "Maxwell" WLBT (WiFi/BT combo)
 * companion chip found on several Exynos-based Samsung phones.
 *
 * Bring-up state -- the firmware boots and the management handshake
 * completes, so the chip is up and idle waiting for a service to be
 * started. No WLAN or BT service exists yet.
 *
 * What works: every resource is resolved and mapped (R4/M4 mailbox SFR
 * windows, PMU syscon, the three chip IRQs, the shared DRAM region);
 * the TZASC window is opened to WLBT via an EL3 SiP call; mx140.bin is
 * parsed and copied into the DRAM window with its entry point handed to
 * the R4 boot ROM via the mailbox scratchpad; the PMU power-on and
 * power-off sequences run, ported from the vendor
 * platform_mif_pmu_reset_release() and platform_mif_pmu_reset(); the
 * infrastructure config (mxconf) is built with all four transports; and
 * the firmware runs to completion, announces itself with MM_START_IND
 * over the management transport, and is halted with MM_HALT_REQ.
 *
 * Two hard-won facts worth keeping:
 *
 *  - PMU_ALIVE state survives a warm reboot. A domain left wedged by a
 *    bad write stays wedged until the battery is actually pulled, and
 *    reports WIFI_STAT ACCESS_MIF=1 while doing so. Do not read that
 *    bit as a sign of life.
 *
 *  - A power-on applied to an already-powered domain does not take,
 *    which is why scsc_mx_power_off() exists and why probe calls it
 *    when it finds WIFI_PWRON already set.
 *
 * Two things here are bring-up shortcuts rather than a final design:
 * the WLBT clocks are poked through raw ioremaps of CMU_CORE/CMU_APM
 * behind the clock driver's back (they belong in clk-exynos7885.c as
 * real gates), and the probe path is heavy with diagnostic logging.
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/ieee80211.h>
#include <linux/ctype.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/random.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#include "bhcs.h"
#include "bsmhcp.h"
#include "bt.h"
#include "fapi.h"
#include "hip4.h"
#include "mbulk.h"
#include "wlan.h"

/*
 * The vendor exynos_acpm_set_flag() always sends this exact id/flag
 * pair on the DVFS ACPM channel, right before WLBT reset-release. See
 * the comment on acpm_dvfs_set_flag() for what little is known about
 * its semantics.
 */
#define SCSC_ACPM_SET_FLAG_ID	7
#define SCSC_ACPM_SET_FLAG_ON	1

/*
 * PMU_ALIVE offsets, relative to the pmu_system_controller syscon base
 * (0x11c80000). Names and bit positions from the vendor
 * mif_reg_S5E7885.h (drivers/misc/samsung/scsc/mif_reg_S5E7885.h in
 * the Samsung 4.14 BSP for this SoC).
 */
#define WIFI_CTRL_NS		0x0140
#define WIFI_PWRON		BIT(1)
#define WIFI_RESET_SET		BIT(2)
/*
 * Acknowledges the wdog interrupt line: set to clear WIFI_RESET_REQ,
 * which the vendor's own comment calls auto-clearing on a direct write
 * (i.e. this bit is pulsed, not held). Named for what it does rather
 * than for a WIFI_RESET_REQ bit this driver has no other use for.
 */
#define WIFI_RESET_REQ_CLR	BIT(8)
/*
 * The counterpart on resume: WLBT can raise WIFI_ACTIVE_REQ to ask the
 * AP to wake up, and this is how the host acknowledges it, same
 * write-to-pulse convention as WIFI_RESET_REQ_CLR. Nothing here gives
 * WLBT a reason to raise it -- there is no WoWLAN, no wake-on-anything
 * -- so this is cheap hygiene on the vendor's own resume path rather
 * than a response to a request this driver expects.
 */
#define WIFI_ACTIVE_CLR		BIT(6)
/*
 * CLEANY bypass control on the WIFI2AP memory path. Note the polarity:
 * per the vendor header, 1 means CLEANY *operates*, 0 means bypass. It
 * comes up set (WIFI_CTRL_NS reads 0x100ea2), so CLEANY is live and
 * will hold the power-down until the WLBT side of the bus drains --
 * which a wedged R4 never lets happen.
 */
#define SWEEPER_BYPASS_DATA_EN	BIT(16)

#define WIFI_CTRL_S		0x0144
#define WIFI_START		BIT(3)

/*
 * External regulator shared between CP (modem), GNSS, the FM radio and
 * WLBT. The vendor wraps its *entire* WLBT power-on sequence in
 * exynos_pmu_shared_reg_enable()/disable() (drivers/misc/mcu_ipc/
 * mcu_ipc.c), and this is exynos7885-specific -- other SoCs in the same
 * driver don't do it. Nothing in our boot chain claims this regulator,
 * which would leave the WLBT domain without its supply no matter how
 * correctly WIFI_CTRL_NS/S is driven.
 *
 * Note the vendor also pokes the CP mailbox (a CP wakeup request) in
 * the same function, since CP nominally owns this rail. We have no CP
 * driver, so only the PMU side is done here.
 */
#define EXT_REGULATOR_SHARED_STATUS	0x3644
#define EXT_REGULATOR_SHARED_OPTION	0x3648
#define EXT_REGULATOR_SHARED_EN		BIT(2)

/*
 * "Check WIFI power-down status" -- WIFI_PWRDN_DONE latches when the
 * domain has confirmed it finished powering down; WIFI_ACCESS_MIF
 * reports whether WIFI is currently accessing the MIF domain. Neither
 * is written anywhere in the vendor driver; read-only diagnostic.
 */
#define WIFI_STAT		0x0148
#define WIFI_PWRDN_DONE		BIT(0)
#define WIFI_ACCESS_MIF		BIT(4)

/*
 * MIF sleep/wakeup debug control. MASK_CLKREQ_WIFI is the interesting
 * one: when set, ALIVE *ignores* WLBT's clock request, which would
 * leave the domain powered and requesting MIF access -- exactly what we
 * observe -- while the core itself never gets a clock to fetch with.
 */
#define WIFI_DEBUG		0x014c
#define EN_MIF_REQ		BIT(0)
#define EN_WIFI_ACTIVE		BIT(2)
#define EN_MIF_RESET_REQ	BIT(3)
#define MASK_CLKREQ_WIFI	BIT(8)

/*
 * Per-domain power state config, "1 - on, 0 - down", applied when the
 * domain is taken through a low-power transition. The vendor only ever
 * clears these on the way *down* (platform_mif_pmu_reset) and never
 * writes them on the way up, so their up-path state is whatever S-Boot
 * left -- which we have never actually looked at. WIFI_DISABLE_ISO and
 * WIFI_RESET_ISO control isolation; if those are configured off, the
 * core stays isolated no matter what WIFI_CTRL_NS/S says.
 */
#define RESET_AHEAD_WIFI_PWR_REG	0x1360
#define CLEANY_BUS_WIFI_SYS_PWR_REG	0x1364
#define LOGIC_RESET_WIFI_SYS_PWR_REG	0x1368
#define TCXO_GATE_WIFI_SYS_PWR_REG	0x136c
#define WIFI_DISABLE_ISO_SYS_PWR_REG	0x1370
#define WIFI_RESET_ISO_SYS_PWR_REG	0x1374

/* Field widths the vendor uses when clearing the group above. */
#define SYS_PWR_CFG		BIT(0)
#define SYS_PWR_CFG_2		GENMASK(1, 0)

/*
 * The state the central sequencer settles in once the WIFI domain has
 * finished powering down. The vendor polls for exactly this value, for
 * up to 500ms, at the end of its reset sequence.
 */
#define CENTRAL_SEQ_STATE_DOWN	0x80

/*
 * PMU central sequencer state for the WIFI domain. Logged as a
 * diagnostic only: the vendor polls this for 0x80 ("up") solely in its
 * *shutdown* path (platform_mif_pmu_reset), never on power-up, and
 * with low-power mode disabled the sequencer isn't driving this domain
 * at all -- so it reads 0 whether or not the core is running. Do not
 * treat 0 here as a power-on failure.
 */
#define CENTRAL_SEQ_WIFI_STATUS	0x0384
#define CENTRAL_SEQ_STATES	GENMASK(23, 16)

/*
 * bit 16: HIGH = system-level low-power mode DISABLED (domain stays
 * under direct/manual WIFI_CTRL_NS/S control); LOW = low-power mode
 * ENABLED (PMU's central sequencer can auto-transition the domain).
 * Vendor's *shutdown* path clears this bit to re-enable automatic
 * low-power sequencing on the way down -- it is not a "take manual
 * control" bit, that reading was backwards. A live read showed bit
 * 16 already set (1) at boot, i.e. already in the correct state for
 * manual power-on; not written here.
 */
#define CENTRAL_SEQ_WIFI_CONFIGURATION	0x0380
#define CENTRAL_SEQ_CFG_LOWPWR		BIT(16)

/*
 * BAAW (Base Address Aliasing Window): tells the chip which AP-DRAM
 * window it's allowed to master into. SECTION_0 only -- CONFIG2/3
 * (SECTION_1) are for a separate BT-ABOX shared region we don't wire
 * up here.
 */
#define WIFI2AP_MEM_CONFIG0	0x7300	/* mem size, SECTION_0 */
#define WIFI2AP_MEM_CONFIG1	0x7304	/* base address, SECTION_0 */
#define WIFI2AP_MEM_CONFIG2	0x7308	/* mem size, SECTION_1 */
#define WIFI2AP_MEM_CONFIG3	0x730c	/* base address, SECTION_1 */

/*
 * R4 mailbox SFR window. Vendor mif_reg_S5E7885.h. Same block layout as
 * drivers/mailbox/exynos-mailbox.c's exynos7885 variant, but this is a
 * separate IP instance dedicated to WLBT.
 */
#define MCUCTRL			0x000
#define INTCR0			0x00c	/* interrupt clear 0 (w) */
#define INTMR0			0x010	/* interrupt mask, r/w */
#define INTSR0			0x014	/* interrupt status 0 (r) */
#define INTGR1			0x01c	/* interrupt generate, AP -> WLBT (w) */
#define INTCR1			0x020	/* interrupt clear 1 (w) */
#define INTMR1			0x024	/* interrupt mask 1, r/w */
#define INTSR1			0x028	/* interrupt status 1 (r) */
/*
 * "IS_Shared_Register" scratchpad, 8 words. These carry no hardware
 * semantics at all, which makes ISSR(0) the cleanest possible probe of
 * whether this SFR block is actually clocked: if a pure scratchpad
 * won't hold a value, nothing in the window will.
 *
 * The R4 boot ROM reads its entry point out of ISSR(0) and a magic
 * number out of ISSR(2) as it comes out of reset -- see mbox_init() in
 * the vendor mxman.c.
 */
#define ISSR_BASE		0x080
#define ISSR(r)			(ISSR_BASE + 4 * (r))
#define MBOX2_MAGIC_NUMBER	0xbcdeedcb

/*
 * Boot source select: 0 = boot from external DRAM (the BAAW window
 * programmed above), non-zero = boot from internal IRAM test image.
 * Both vendor ports write 0 here before releasing reset.
 */
#define WLBT_BOOT_TEST_RST_CFG	0x7330

/*
 * TZASC (TrustZone Address Space Controller) window for WLBT.
 *
 * WLBT is a separate bus master, so the AP mapping the carveout says
 * nothing about whether WLBT may read it: that is arbitrated in the
 * TZASC, which only EL3 can program. The vendor issues this SiP call
 * unconditionally at probe on every SoC from "Katmai" onward, with a
 * comment warning that without it "DRAM could be NOT accessible in
 * secure mode" (platform_mif.c, EXYNOS_SMC_WLBT_TZASC_CMD in
 * mif_reg_S5E7885.h).
 *
 * Nothing in a mainline boot makes this call, which fits what we see:
 * the domain powers up and asserts WIFI_ACCESS_MIF -- it is trying to
 * fetch -- yet executes nothing and leaves an empty panic record,
 * because the panic handler lives in the image it cannot read.
 *
 * We still run the stock S-Boot EL3_MON here, so the call should be
 * implemented. A return of SMCCC_RET_NOT_SUPPORTED (-1) means it is
 * not, which rules the theory out just as usefully.
 */
#define EXYNOS_SMC_WLBT_TZASC_CMD	0x82000710
#define WLBT_TZASC			0

/*
 * WLBT clocks. The vendor cal-if tables (drivers/soc/samsung/cal-if/
 * exynos7885/cmucal-qch.c) list exactly four WLBT quality channels:
 * three in CMU_CORE and MAILBOX_APM2WLBT in CMU_APM. Offsets and bit
 * positions from cmucal-sfr.c in the same directory.
 *
 * Mainline's clk-exynos7885.c models no WLBT clocks at all, and neither
 * vendor driver enables them explicitly -- downstream relies on the
 * cal-if/cmucal layer (and S-Boot) to set up HWACG. Nothing in our boot
 * chain does, which is why the mailbox SFR window read back as zero
 * until QCH_CON_MAILBOX_WLBT was enabled here.
 *
 * Two kinds of control per block:
 *   QCH_CON     - HWACG quality channel; ENABLE lets the block request
 *                 its clock at all.
 *   CLK_CON_GAT - the gate itself; MANUAL=1 + CG_VAL=1 forces it on,
 *                 bypassing automatic gating.
 *
 * LHM_AXI_D_WLBT is the AXI *data* path -- the R4's own bus master port
 * into DRAM. With that gated the core cannot fetch its image no matter
 * how correctly the BAAW window and entry point are programmed, which
 * matches what we see.
 *
 * Poked directly here as bring-up; these belong in the clock driver as
 * real gates rather than an ioremap behind the CMU's back.
 */
#define CMU_CORE_BASE			0x12000000
#define CMU_CORE_SIZE			0x4000
/*
 * Note this is CMU_APM, not the PMU_ALIVE syscon at 0x11c80000 that
 * mx->pmureg covers -- different block, adjacent address.
 */
#define CMU_APM_BASE			0x11c00000
#define CMU_APM_SIZE			0x4000

#define QCH_CON_LHM_AXI_D_WLBT		0x305c
#define QCH_CON_MAILBOX_WLBT_CM4	0x30c0
#define QCH_CON_MAILBOX_WLBT		0x30c4
#define QCH_CON_ENABLE			BIT(0)
#define QCH_CON_CLOCK_REQ		BIT(1)

/* CMU_APM: the APM (power-management M0) side mailbox to WLBT. */
#define QCH_CON_MAILBOX_APM2WLBT	0x3044

#define CLK_CON_GAT_LHM_AXI_D_WLBT	0x2098
#define CLK_CON_GAT_MAILBOX_WLBT_CM4	0x20fc
#define CLK_CON_GAT_MAILBOX_WLBT	0x2100
#define CLK_CON_GAT_MAILBOX_APM2WLBT	0x2050
#define CLK_CON_GAT_MANUAL		BIT(20)
#define CLK_CON_GAT_CG_VAL		BIT(21)

#define SCSC_FW_NAME		"mx140.bin"

/*
 * Maxwell firmware header, format SC-505846-SW. Offsets from the vendor
 * fwhdr.c; the v0.2 and v1.0 layouts are identical. mx140.bin from this
 * device's vendor partition parses as hdr v1.0, fwapi 0.2.
 */
#define FWHDR_MAGIC_OFFSET		8
#define FWHDR_MAGIC			"smxf"
#define FWHDR_LENGTH_OFFSET		16
#define FWHDR_RUNTIME_LENGTH_OFFSET	36
#define FWHDR_ENTRY_POINT_OFFSET	40
#define FWHDR_BUILD_ID_OFFSET		48
#define FWHDR_BUILD_ID_MAX		128
#define FWHDR_R4_PANIC_RECORD_OFFSET	176
/* Header is only guaranteed to carry panic-record offsets past 188. */
#define FWHDR_MIN_LENGTH		192

/*
 * mx140 infrastructure config structure (vendor mxconf.h), handed to the
 * firmware by R4-relative address in ISSR(1). Magic is checked by the R4
 * to trap endian mismatches, version to trap layout mismatches.
 *
 * Do NOT compute these offsets from mxconf.h. That header marks the
 * structures __packed, which gives a 22-byte mxstreamconf and a
 * 162-byte mxconf -- but the ABI the firmware actually implements pads
 * mxstreamconf to a 4-byte boundary, making it 24, mxtransconf 48 and
 * mxconf 176. Verified by dumping the serialised structure out of a
 * running vendor driver (4.4 downstream, /dev/mx_0_mmap): its stream
 * descriptors sit at +8, +32, +56, +80, +104, +128, +152.
 *
 * Getting this wrong is silent: the firmware reads magic, version and
 * the first transport correctly, then walks into misaligned garbage and
 * gives up without touching a single ring.
 */
#define MXCONF_MAGIC		0x79828486
#define MXCONF_VERSION_MAJOR	0
#define MXCONF_VERSION_MINOR	1
#define MXCONF_SIZE		176
#define MXCONF_ALLOC_SIZE	512

/*
 * Field offsets within the packed structures in mxconf.h. Written a
 * field at a time rather than as C structs because the destination is
 * __iomem: all "*_loc" fields are R4-relative offsets, i.e. offsets
 * from the base of the BAAW window (mem_base), which is also what the
 * entry point is relative to.
 *
 *   mxcbufconf   (20 bytes)  buffer_loc, num_packets, packet_size,
 *                            read_index_loc, write_index_loc
 *   mxstreamconf (24 bytes)  mxcbufconf + read_bit_idx + write_bit_idx,
 *                            padded to 4 bytes
 *   mxtransconf  (48 bytes)  to_ap_stream_conf + from_ap_stream_conf
 *   mxconf      (176 bytes)  magic, version, mgmt + gdb_r4 + gdb_m4
 *                            transports, mxlogconf
 */
#define MXCBUFCONF_BUFFER_LOC		0
#define MXCBUFCONF_NUM_PACKETS		4
#define MXCBUFCONF_PACKET_SIZE		8
#define MXCBUFCONF_READ_INDEX_LOC	12
#define MXCBUFCONF_WRITE_INDEX_LOC	16
#define MXCBUFCONF_SIZE			20

#define MXSTREAMCONF_READ_BIT_IDX	20
#define MXSTREAMCONF_WRITE_BIT_IDX	21
#define MXSTREAMCONF_SIZE		24

#define MXTRANSCONF_TO_AP		0
#define MXTRANSCONF_FROM_AP		24
#define MXTRANSCONF_SIZE		48

#define MXCONF_OFF_MAGIC		0
#define MXCONF_OFF_VERSION_MAJOR	4
#define MXCONF_OFF_VERSION_MINOR	6
#define MXCONF_OFF_MGMT			8
#define MXCONF_OFF_GDB_R4		56
#define MXCONF_OFF_GDB_M4		104
#define MXCONF_OFF_MXLOG		152

/*
 * Transport geometry, from the vendor's *_transport_init(). The
 * management packet size is sizeof(struct mxmgr_message) -- a 1-byte
 * channel id plus a 7-byte payload.
 */
#define MXMGMT_BUF_LENGTH	512
#define MXMGMT_PACKET_SIZE	8
#define GDB_BUF_LENGTH		(2 * 1024)
#define GDB_PACKET_SIZE		4
#define MXLOG_BUF_LENGTH	(16 * 1024)
#define MXLOG_PACKET_SIZE	4

/*
 * Management transport channels (vendor mxmgmt_transport.h). Every
 * packet on the transport starts with one of these; only channel 0 is
 * handled here, the other two belong to the service manager and to
 * mxlogger, neither of which exists yet.
 */
#define MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT	0
#define MMTRANS_CHAN_ID_SERVICE_MANAGEMENT	1
#define MMTRANS_CHAN_ID_MAXWELL_LOGGING		2

/*
 * Maxwell agent messages carried on channel 0 (vendor mxman.c "ma_msg").
 * The numbering is firmware ABI, not a local choice.
 *
 * The payload the vendor puts on the wire is a __packed { u8 ma_msg;
 * u32 arg; } appended to the channel id, so a message occupies six of
 * the eight bytes in a packet and the argument is unaligned. Only
 * to-host messages carry a meaningful argument.
 */
#define MM_START_IND	0
#define MM_HALT_REQ	1
#define MM_FORCE_PANIC	2
#define MM_HOST_SUSPEND	3
#define MM_HOST_RESUME	4
#define MM_FW_CONFIG	5
#define MM_HALT_RSP	6

#define MM_MSG_OFF_CHANNEL	0
#define MM_MSG_OFF_MSG		1
#define MM_MSG_OFF_ARG		2
#define MM_MSG_SIZE		6

/*
 * Service management on channel 1 (vendor servman_messages.h). The
 * payload is a __packed { u8 service_id; u8 msg; u32 optional_data; },
 * so seven bytes with the channel id -- a different layout from channel
 * 0, not a variation on it.
 */
#define SM_MSG_START_REQ	0
#define SM_MSG_START_CFM	1
#define SM_MSG_STOP_REQ		2
#define SM_MSG_STOP_CFM		3

#define SM_MSG_OFF_SERVICE_ID	1
#define SM_MSG_OFF_MSG		2
#define SM_MSG_OFF_DATA		3
#define SM_MSG_SIZE		7

/*
 * Service ids (vendor include/scsc/scsc_mx.h). Note that client_test.c's
 * own module-parameter help string has WLAN and BT the wrong way round;
 * this enum is what goes on the wire.
 */
#define SCSC_SERVICE_ID_NULL		0
#define SCSC_SERVICE_ID_WLAN		1
#define SCSC_SERVICE_ID_BT		2
#define SCSC_SERVICE_ID_ANT		3
#define SCSC_SERVICE_ID_R4DBG		4
#define SCSC_SERVICE_ID_ECHO		5
#define SCSC_SERVICE_ID_DBG_SAMPLER	6
#define SCSC_SERVICE_ID_CLK20MHZ	7
#define SCSC_SERVICE_ID_INVALID		0xff

/* Vendor sm_completion_timeout_ms, for both START_CFM and STOP_CFM. */
#define SM_CFM_TIMEOUT_MS	1000

/*
 * Vendor timeouts: mm_completion_timeout_ms for MM_START_IND and
 * MM_HALT_RSP_TIMEOUT_MS for the reply to MM_HALT_REQ. The latter is
 * short on purpose -- not every firmware build implements MM_HALT_RSP,
 * and its absence is not an error.
 */
#define MM_START_IND_TIMEOUT_MS	2000
#define MM_HALT_RSP_TIMEOUT_MS	100

/*
 * How long to wait for a FAPI confirm. The vendor's mlme layer allows
 * ten seconds, but that covers signals that go on the air (scan, join);
 * a MIB read is answered in microseconds or not at all.
 */
#define SCSC_HIP_CFM_TIMEOUT_MS	1000

/*
 * mxlog record framing (vendor mxlog.c mxlog_header_parser). Each
 * record starts with a u32 header: a sync value in the top half, the
 * firmware debug level in bits 15:8, a length in bits 7:0.
 *
 * Phase 4 records carry a plain string and the length counts chars.
 * Phase 5 records are tokenised -- the length counts 32-bit words and
 * the payload indexes a firmware-side string table shipped separately
 * as common/log-strings.bin, which is not among the blobs we pulled off
 * the vendor partition. Those get dumped raw.
 */
#define MXLOG_SYNC_PHASE_4	0xa55a
#define MXLOG_SYNC_PHASE_5	0x9669
#define MXLOG_MAX_RECORD	256

/*
 * MIF interrupt bits, 16 in each direction. Bit 0 of both from-host
 * registers is reserved for forcing a firmware panic, so dynamic
 * allocation starts at 1; to-host bits have no reservation.
 */
#define MIFINTRBIT_NUM_INT		16
#define MIFINTRBIT_RESERVED_PANIC	0

/*
 * Bring-up diagnostics: register dumps either side of power-on, the
 * DRAM change detector, the HIP/MIB setup detail, per-step MLME and
 * power-off chatter, and the firmware's own log (mxlog). All of it was needed to get the firmware running and none
 * of it is news on a healthy boot, so it is off unless asked for.
 * Writable at runtime so a misbehaving link can be watched without a
 * reload; the probe-time part needs it set at load.
 */
static bool scsc_mx_verbose;
module_param_named(verbose, scsc_mx_verbose, bool, 0644);
MODULE_PARM_DESC(verbose,
		 "log bring-up diagnostics and the firmware log: register dumps, DRAM diffs, HIP/MIB, MLME and power-off detail, mxlog (default off)");

#define scsc_mx_vinfo(mx, fmt, ...)					\
	do {								\
		if (scsc_mx_verbose)					\
			dev_info((mx)->dev, fmt, ##__VA_ARGS__);	\
	} while (0)

/* Interrupt storm guard: any firmware signal is news, a flood is not. */
#define SCSC_IRQ_LOG_MAX	16

/*
 * Granularity of the DRAM change detector. The R4's stack, heap and BSS
 * all live in this carveout, so any instruction it actually executes
 * perturbs the region -- which makes a before/after comparison the one
 * test that separates "never fetched" from "ran and died somewhere we
 * can't see". 64K chunks over 4M is 64 CRCs: fine to hold, and coarse
 * enough to read at a glance while still localising what moved.
 */
/*
 * firmware_startup_flags, handed over in ISSR(3).
 *
 * The vendor header documents only bit 0 ("spin in CRT0 for a debugger
 * to attach") and calls the rest reserved, but the image disassembles
 * otherwise: the startup function loads ISSR(3) into r5 and then gates
 * individual init steps on its bits, e.g.
 *
 *	lsls r3, r5, #0x1e	; bit 1
 *	bmi  <skip>		; skips the stage-7 step (GDB transports)
 *
 * That makes it a way to bisect startup: turn a step off and read the
 * stage marker to see whether the hang moves. Kept as a module
 * parameter so a value can be tried from /etc/modprobe.d without a
 * rebuild. 0 is the normal boot and the shipping default.
 */
static unsigned int scsc_mx_startup_flags;
module_param_named(startup_flags, scsc_mx_startup_flags, uint, 0444);
MODULE_PARM_DESC(startup_flags,
		 "firmware_startup_flags in ISSR(3); bit1 skips GDB transport init (default 0)");

#define SCSC_DRAM_CHUNK		SZ_64K
/* Enough to characterise a small write burst without flooding dmesg. */
#define SCSC_DRAM_DIFF_WORDS	192

/* Vendor caps both v1 and v2 panic records at 256 words. */
#define R4_PANIC_RECORD_MAX_WORDS	256

/*
 * R4 panic record v2 field indices; see scsc_mx_dump_panic_record().
 * Word 1 is the length in bytes, which is what shifts everything below
 * by one relative to the v1 layout in panic_record_r4_defs.h.
 */
#define R4_PANIC_RECORD_VERSION_2	2
#define R4_PANIC_RECORD_V2_WORDS	53
#define R4_PANIC_RECORD_V2_STRING	2
#define R4_PANIC_RECORD_V2_SUBCODE	3
#define R4_PANIC_RECORD_V2_REGS		8
#define R4_PANIC_RECORD_V2_SYMPATHETIC	51
#define PANIC_RECORD_CKSUM_SEED		0xa5a5a5a5
#define SCSC_PANIC_SUBCODE_MASK		0x7fff

/*
 * Host-side view of one simplex stream, kept because everything the
 * firmware needs is in mxconf but everything *we* need to work the ring
 * has to be remembered: where the buffer and its two indices are (all
 * R4-relative, so offsets into the DRAM mapping), the geometry, and
 * which interrupt bit each side raises.
 *
 * write_bit is raised by whoever wrote a packet, read_bit by whoever
 * consumed one -- so for a stream we read from, write_bit is the
 * to-host bit the firmware kicks us with and read_bit is the from-host
 * bit we kick back. Naming them by role rather than direction keeps the
 * two streams of a transport symmetric.
 */
struct scsc_mx_stream {
	u32 buffer;
	u32 ridx_loc;
	u32 widx_loc;
	u32 num_packets;
	u32 packet_size;
	u8 read_bit;
	u8 write_bit;
};

/* Bump-allocator cursor over the shared-DRAM heap; see scsc_mx_ram_alloc(). */
struct scsc_mx_ram {
	void __iomem *base;
	u32 next;
	u32 end;
};

/* MIF interrupt-bit counters; see scsc_mx_intr_alloc(). */
struct scsc_mx_intr {
	u8 tohost;
	u8 fromhost_r4;
	u8 fromhost_m4;
};

/* A transport is a pair of streams, named from the firmware's side. */
struct scsc_mx_transport {
	struct scsc_mx_stream in;	/* firmware -> AP */
	struct scsc_mx_stream out;	/* AP -> firmware */
};

/*
 * Host-side bookkeeping for one mbulk pool.
 *
 * The free list itself lives in shared memory -- each free segment
 * points at the next by signed relative offset -- so all we keep here is
 * the head and the geometry. The firmware never allocates from a host
 * pool, but it does walk the descriptors, which is why the list has to
 * be maintained in place rather than mirrored.
 */
struct scsc_mx_mbulk_pool {
	u32 base;		/* R4-relative, as everything shared is */
	u32 size;
	u32 block;		/* per segment, including struct mbulk */
	u32 head;		/* free-list head, 0 when exhausted */
	unsigned int free;
};

/* Largest signal and bulk payload this driver will handle in one go. */
#define SCSC_HIP_SIG_MAX	256
#define SCSC_HIP_DATA_MAX	2048

/*
 * Room for the information elements of an association request. The ones
 * this firmware sends run to a couple of hundred bytes -- the vendor
 * element alone can be over a hundred -- so this is generous rather than
 * tight, and over-long ones are truncated rather than refused.
 */
#define SCSC_ASSOC_IE_MAX	512

/*
 * HIP4 transport state.
 *
 * One outstanding request at a time, matched by signal id: enough for
 * bring-up, where every exchange is a request the probe path issues and
 * waits for. A real driver keys confirms off the vif and host tag and
 * lets several run at once.
 */
struct scsc_mx_hip {
	bool ready;
	u32 ctl;		/* struct hip4_hip_control */
	u32 scbrd;		/* its scoreboard */
	u8 fh_bit;		/* from-host: we kick this after queueing */
	u8 th_bit;		/* to-host: the firmware kicks us on this */
	u32 region;		/* the WLAN region, for bounds checks */
	u32 region_size;
	u8 headroom;		/* both firmware-dictated, see the config */
	u8 tailroom;
	struct scsc_mx_mbulk_pool pool[2];	/* [0] data, [1] control */
	/*
	 * Serialises the pools, the queues and the scoreboard.
	 *
	 * A mutex, not a spinlock: both users are process context -- the
	 * bottom half is a work item and signals are sent from the probe
	 * path -- and the interrupt handler only schedules the work. It
	 * was a spinlock held with interrupts off across the whole drain,
	 * logging included, which is fine for a handful of signals and
	 * not at all fine for the hundreds a scan delivers.
	 */
	struct mutex lock;
	struct work_struct work;
	/* The one outstanding request. */
	u16 pid;		/* rolling sender_pid, see fapi.h */
	u16 host_tag;		/* rolling id for fire-and-forget frames */
	u16 cfm_id;
	struct completion cfm;
	u8 cfm_sig[SCSC_HIP_SIG_MAX];
	u8 cfm_data[SCSC_HIP_DATA_MAX];
	u16 cfm_siglen;
	u16 cfm_datalen;
	/*
	 * Results counted between the confirm and MLME-SCAN-DONE. The
	 * indication is not waited for -- cfg80211 issues a scan and is
	 * told later, so it is reported rather than blocked on.
	 */
	unsigned int scan_results;
	/*
	 * Set when MLME-SCAN-DONE arrives and acted on after the
	 * transport lock is dropped: cfg80211_scan_done() wants the wiphy
	 * mutex, and taking that under this one would invent a lock
	 * ordering for no reason.
	 */
	bool scan_done;
	/*
	 * Set when the from-host return queue gave buffers back and the
	 * transmit path had been stopped for want of them. Acted on after
	 * the lock is dropped, for the same reason as scan_done: waking a
	 * netdev queue re-enters ndo_start_xmit.
	 */
	bool tx_resume;
	bool tx_stopped;
	/*
	 * The information elements of the association request the firmware
	 * actually transmitted, kept from MLME-PROCEDURE-STARTED until the
	 * connect result can report them. Not a copy of what we asked for:
	 * the firmware rewrites parts of it -- see scsc_mx_assoc_ies().
	 */
	u8 assoc_req_ie[SCSC_ASSOC_IE_MAX];
	u16 assoc_req_ie_len;
	/*
	 * Vifs whose association still needs acknowledging, one bit each.
	 * A bitmask rather than a flag because the request can come from
	 * the bottom half, which already holds the lock this would need to
	 * take -- see scsc_mx_mlme_connect_resp().
	 */
	unsigned long connect_resp_pending;
	/*
	 * The AP-mode counterpart, one bit per peer_index (1..16, so a
	 * peer_index is also a bit position) rather than per vif -- an AP
	 * vif can have several stations finishing their handshakes at
	 * once. Indexed by vif; slot 0 unused, like scsc_wlan::ndev.
	 */
	unsigned long connected_resp_pending[SCSC_WLAN_MAX_VIFS + 1];
	/*
	 * The roam-mode counterpart of connect_resp_pending -- one bit per
	 * vif, since a station has only the one peer. Split from it rather
	 * than reused because the two acknowledge different signals
	 * (MLME_CONNECT_RES vs MLME_ROAMED_RES) and scsc_wlan_add_key()
	 * needs to know which one a pairwise key install is completing.
	 */
	unsigned long roamed_resp_pending;
	/*
	 * A channel switch to report. cfg80211_ch_switch_notify() wants the
	 * wiphy mutex, which the transport lock is not, so this is captured
	 * here and sent from scsc_mx_hip_work() once the lock is dropped --
	 * same reason as scan_done above. Indexed by vif; slot 0 unused.
	 */
	struct {
		bool pending;
		u32 freq_mhz;
		u32 center_freq1_mhz;
		u8 width_mhz;
	} chsw[SCSC_WLAN_MAX_VIFS + 1];
	/* (first channel, count) pairs, as the firmware reports them. */
	u8 chanranges[32];
	u8 nchanranges;
	/*
	 * HT/VHT support, read from the firmware's own capability MIBs
	 * (UnifiHtCapabilities/UnifiVhtCapabilities) during
	 * scsc_mx_wlan_probe_mib() rather than assumed -- see
	 * scsc_wlan_build_bands(), which turns these into
	 * ieee80211_sta_ht_cap/ieee80211_sta_vht_cap. Each blob is the raw
	 * 802.11 HT/VHT Capabilities element body the firmware would put
	 * on air, which is why it can mostly be copied rather than built.
	 */
	bool ht_activated;
	bool vht_activated;
	u8 ht_cap[21];
	u8 vht_cap[12];
};

/*
 * BT (BSMHCP) data path state, live once the BT service has started.
 *
 * The info_* bits scsc_mx_bt_start() also allocates are still unused
 * (FW_INFO/AVDTP/IQ-report, none of which this driver implements --
 * see bt.md).
 *
 * ACL is a two-hop path in a way HCI command/event is not: an L2CAP
 * PDU larger than one ACL packet arrives from hci_core as several
 * fragments (a START one carrying the L2CAP header's length/CID, then
 * CONT ones that do not), and BSMHCP wants every fragment's TX
 * descriptor to carry the CID and an explicit "this fragment completes
 * the L2CAP PDU" flag regardless of which fragment it is. conn[] is
 * where the CID and remaining-length of an in-progress reassembly are
 * kept between one fragment and the next, indexed by the connection
 * handle exactly as the vendor's connection_handle_list is (SC-505753-DD
 * gives no smaller bound than the 12-bit handle space, so neither does
 * this).
 */
#define SCSC_BT_MAX_CONN_HANDLES	4096

struct scsc_mx_bt_conn {
	u16 l2cap_cid;
	u16 remaining;		/* L2CAP octets still expected, mid-reassembly */
};

struct scsc_mx_bt {
	bool ready;
	u32 proto;		/* BSMHCP_PROTOCOL offset in DRAM */
	u8 ap_to_bg;		/* from-host: kicked after posting a command */
	u8 ap_to_fg;		/* from-host: kicked after posting ACL data */
	u8 bg_to_ap;		/* to-host: the firmware kicks this on an event */
	struct work_struct work;
	/* Serialises concurrent hci_dev sends against the TX rings. */
	struct mutex lock;
	/* Which of the 30 acl_tx_buffer slots are currently lent out. */
	unsigned long tx_buf_used;
	struct scsc_mx_bt_conn conn[SCSC_BT_MAX_CONN_HANDLES];
};

struct scsc_mx {
	struct device *dev;
	void __iomem *r4_regs;
	void __iomem *m4_regs;
	struct regmap *pmureg;
	struct acpm_handle *acpm;
	unsigned int acpm_chan;
	phys_addr_t mem_base;
	size_t mem_size;
	/* BT-ABOX shared window, BAAW SECTION_1 (see scsc_mx_alloc_abox()). */
	void *abox_cpu;
	dma_addr_t abox_dma;
	phys_addr_t abox_base;
	size_t abox_size;
	u32 r4_panic_record_offset;
	size_t fw_size;
	atomic_t irq_count;
	int irq_mbox;
	int irq_alive;
	int irq_wdog;
	/*
	 * Whether the wdog handler is currently enabled, so remove() knows
	 * whether it owes it a matching disable_irq() -- see
	 * scsc_mx_wdog_irq() and its enable site in probe().
	 *
	 * Doubles as "the firmware is confirmed up and has not since died",
	 * which is exactly the same window: both flip true at the same
	 * successful scsc_mx_wait_start_ind(), and false at the same two
	 * places (a fired watchdog, or remove()). See its use gating
	 * MM_HOST_SUSPEND/RESUME in scsc_mx_pm_suspend()/resume().
	 */
	bool wdog_armed;
	u32 *dram_crc;			/* pre-reset-release snapshot */
	void *dram_shadow;		/* ...and the bytes behind it */
	unsigned int dram_chunks;
	/*
	 * The shared window, mapped for the life of the device: the
	 * interrupt path reaches into the rings, so it cannot be an
	 * ioremap around each use as the diagnostics used to be.
	 */
	void __iomem *dram;
	/* Management transport, and whether it is safe to walk. */
	struct scsc_mx_transport mgmt;
	bool mgmt_ready;
	struct work_struct mgmt_work;
	struct completion mm_start_ind;
	struct completion mm_halt_rsp;
	u32 rf_hw_ver;
	/* Shared-RAM cursor and interrupt-bit counters, see build_mxconf(). */
	struct scsc_mx_ram ram;
	struct scsc_mx_intr intr;
	/*
	 * Which services this driver has running, and their confirmations.
	 * Two slots -- WLAN and BT -- rather than one: scsc_mx_service_start()
	 * and scsc_mx_service_stop() are synchronous (each sends its request
	 * and blocks for the matching CFM before returning), so only one is
	 * ever outstanding at a time even with two services active, which is
	 * what lets service_pending stay a scalar rather than needing its own
	 * per-service set.
	 */
	bool service_active[SCSC_SERVICE_ID_CLK20MHZ + 1];	/* index by SCSC_SERVICE_ID_*, [0] unused */
	u8 service_pending;
	struct completion sm_start_cfm;
	struct completion sm_stop_cfm;
	/* WLAN data path, live once the WLAN service has started. */
	struct scsc_mx_hip hip;
	/* The cfg80211 side, once it is registered. */
	struct scsc_wlan *wlan;
	/* BT data path, live once the BT service has started. */
	struct scsc_mx_bt bt;
	/* The hci_dev side, once it is registered. */
	struct scsc_bt *bt_drv;
	/* Phase-5 format strings, indexed by byte offset (see mxlog dump). */
	const struct firmware *logstr;
	/* Firmware log stream, firmware -> AP only. */
	struct scsc_mx_stream mxlog;
	struct work_struct mxlog_work;
	bool mxlog_ready;
};

static void scsc_mx_mgmt_drain(struct scsc_mx *mx);
static void scsc_mx_mxlog_drain(struct scsc_mx *mx);
static const char *scsc_mx_result_name(u16 code);
static void scsc_mx_dump_panic_record(struct scsc_mx *mx, void __iomem *dram,
				      const char *when);

static irqreturn_t scsc_mx_irq(int irq, void *data)
{
	struct scsc_mx *mx = data;
	unsigned int n = atomic_inc_return(&mx->irq_count);
	u32 status;

	/*
	 * R0 group (bits 31:16) carries R4/M4 -> AP interrupts and is
	 * acknowledged via INTCR0; R1 is the other direction.
	 */
	status = readl(mx->r4_regs + INTSR0);
	writel(status, mx->r4_regs + INTCR0);

	if (scsc_mx_verbose && n <= SCSC_IRQ_LOG_MAX)
		dev_info(mx->dev, "irq %d from WLBT (#%u), INTSR0=0x%x\n",
			 irq, n, status);

	/*
	 * Log volume is capped, but the line stays enabled: these
	 * interrupts carry the management protocol now, so masking them
	 * after sixteen of anything would stall the transport. (It used
	 * to disable the irq here, back when every interrupt was noise.)
	 */
	if (scsc_mx_verbose && n == SCSC_IRQ_LOG_MAX)
		dev_warn(mx->dev, "silencing further WLBT irq logs\n");

	/*
	 * The firmware raises the in-stream's write bit when it posts a
	 * management message. Draining the ring reads DRAM and can log,
	 * so it happens in process context.
	 */
	if (mx->mgmt_ready && (status & BIT(16 + mx->mgmt.in.write_bit)))
		schedule_work(&mx->mgmt_work);

	/*
	 * The WLAN data path has a bit of its own, allocated when the
	 * service started. The vendor masks it here and unmasks it at the
	 * end of its bottom half, to bound an interrupt storm; we do not,
	 * because the only per-bit masking we have is the "unmask bits 0..n"
	 * write and undoing it would close the management transport too.
	 * The work item is singly queued, so a storm costs interrupts but
	 * cannot stack up bottom halves.
	 */
	if (mx->hip.ready && (status & BIT(16 + mx->hip.th_bit)))
		schedule_work(&mx->hip.work);

	/*
	 * The BT event/ACL-RX doorbell, allocated when that service
	 * started. Same singly-queued reasoning as mx->hip.work above.
	 */
	if (mx->bt.ready && (status & BIT(16 + mx->bt.bg_to_ap)))
		schedule_work(&mx->bt.work);

	/*
	 * The firmware raises this whenever it logs. Draining in process
	 * context because rendering a record reads DRAM and prints.
	 */
	if (mx->mxlog_ready && (status & BIT(16 + mx->mxlog.write_bit)))
		schedule_work(&mx->mxlog_work);

	return IRQ_HANDLED;
}

static void scsc_mx_mgmt_work(struct work_struct *work)
{
	struct scsc_mx *mx = container_of(work, struct scsc_mx, mgmt_work);

	scsc_mx_mgmt_drain(mx);
}

static void scsc_mx_mxlog_work(struct work_struct *work)
{
	struct scsc_mx *mx = container_of(work, struct scsc_mx, mxlog_work);

	scsc_mx_mxlog_drain(mx);
}

/* Close the transport: nothing may touch the rings after this. */
static void scsc_mx_stop_mgmt(void *data)
{
	struct scsc_mx *mx = data;

	mx->mgmt_ready = false;
	mx->hip.ready = false;
	mx->bt.ready = false;
	mx->mxlog_ready = false;
	cancel_work_sync(&mx->mgmt_work);
	cancel_work_sync(&mx->hip.work);
	cancel_work_sync(&mx->bt.work);
	cancel_work_sync(&mx->mxlog_work);
}

/*
 * Only "mbox" is enabled on request. The "wdog" line is already
 * asserted while the WLBT block is down -- enabling it before the
 * firmware has announced itself would produce an immediate screaming
 * level interrupt that has nothing to do with the firmware, so it stays
 * disabled here and is armed once scsc_mx_wait_start_ind() succeeds
 * (see scsc_mx_wdog_irq()). "alive" is left off, full stop: the vendor's
 * own handler for it (platform_alive_isr(), gated behind a config
 * option that is not default-enabled there either) does nothing but log
 * that it fired, so there is no meaningful action to wire up yet.
 */
static int scsc_mx_request_irq(struct platform_device *pdev,
				struct scsc_mx *mx, const char *name,
				irq_handler_t handler, unsigned long flags,
				int *irqp)
{
	int irq, ret;

	irq = platform_get_irq_byname(pdev, name);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(&pdev->dev, irq, handler, flags, name, mx);
	if (!ret && irqp)
		*irqp = irq;

	return ret;
}

/*
 * Report the raw state of an interrupt line without enabling its
 * handler. Used for "alive", which still has no handler worth enabling
 * (see scsc_mx_request_irq()), and for "wdog" during the probe-time
 * diagnostics below, before scsc_mx_wdog_irq() is armed -- enabling it
 * that early would catch it still asserted from the block being down
 * and scream immediately. Reading the irqchip state polls the GIC
 * directly instead, which is safe at any time.
 */
static void scsc_mx_report_irq_line(struct scsc_mx *mx, const char *name,
				    int irq)
{
	bool pending = false, level = false;
	int ret_p, ret_l;

	if (irq <= 0)
		return;

	ret_p = irq_get_irqchip_state(irq, IRQCHIP_STATE_PENDING, &pending);
	ret_l = irq_get_irqchip_state(irq, IRQCHIP_STATE_LINE_LEVEL, &level);

	dev_info(mx->dev, "irq line %s (%d): pending=%s level=%s\n", name, irq,
		 ret_p ? "?" : (pending ? "YES" : "no"),
		 ret_l ? "?" : (level ? "HIGH" : "low"));
}

/*
 * The R4 watchdog fired: it stopped petting its own timer, which is
 * what tells apart a hung core (parked on an outstanding bus
 * transaction, or in WFI with its VIC masked) from a merely quiet one
 * -- see scsc_mx_report_irq_line() above. The line is level-triggered
 * and, unlike the mailbox, cannot be acknowledged through anything
 * scsc_mx_irq() touches; the vendor clears it by writing
 * WIFI_RESET_REQ_CLR in the PMU instead (platform_wdog_isr(),
 * platform_mif.c), which is what this does too.
 *
 * Left disabled afterwards rather than re-armed: a fired watchdog means
 * this boot's firmware is dead, and the only way back is a fresh R4
 * boot -- a module reload, which requests this interrupt afresh. See
 * "one R4 boot per SoC reset" elsewhere in this file.
 */
static irqreturn_t scsc_mx_wdog_irq(int irq, void *data)
{
	struct scsc_mx *mx = data;
	int ret;

	disable_irq_nosync(irq);
	mx->wdog_armed = false;

	dev_err(mx->dev,
		"R4 watchdog fired -- the firmware has stopped responding\n");
	scsc_mx_dump_panic_record(mx, mx->dram, "watchdog");

	ret = regmap_update_bits(mx->pmureg, WIFI_CTRL_NS, WIFI_RESET_REQ_CLR,
				 WIFI_RESET_REQ_CLR);
	if (ret)
		dev_warn(mx->dev, "failed to clear WIFI_RESET_REQ: %d\n", ret);

	return IRQ_HANDLED;
}

/*
 * Ungate the WLBT mailbox SFR block's clock in CMU_CORE. See the
 * comment on CMU_CORE_QCH_MAILBOX_WLBT for why this is done behind the
 * clock driver's back for now.
 */
/*
 * Whether to force the four WLBT clock gates open.
 *
 * A working 4.4 system reads QCH_CON = 0x3f0002 and CLK_CON_GAT =
 * 0x200000 on all four -- exactly the state this driver was written to
 * "fix" -- while WLBT runs, and its mailbox ISSR registers read back
 * fine through /dev/mem in that state. So the ungating is a divergence
 * we introduce, and the early "the mailbox window is dead until
 * MAILBOX_WLBT is ungated" observation was almost certainly taken
 * against the poisoned CENTRAL_SEQ_WIFI_CONFIGURATION state that
 * invalidated that whole run of measurements.
 *
 * Tested both ways with the SYS_PWR fix in place: the firmware boots
 * fully either way, so this is an unnecessary poke at clock gates the
 * vendor leaves alone. Default off, kept as a parameter because the
 * "gated" reading is counter-intuitive enough to want to re-check.
 */
/*
 * Deliberately panic the firmware after the post-reset dump.
 *
 * This existed to answer "is the core alive or stopped?" back when it
 * hung mid-init and would not respond. The firmware now boots fully and
 * waits for the host, so poking it destroys a healthy run: the panic
 * record that used to appear afterwards was this, not a firmware fault.
 * Off by default; still useful if a future hang needs the same answer.
 */
static unsigned int scsc_mx_poke_panic_enabled;
module_param_named(poke_panic, scsc_mx_poke_panic_enabled, uint, 0444);
MODULE_PARM_DESC(poke_panic,
		 "force a firmware panic after the post-reset dump (default 0)");

static bool scsc_mx_fw_debug;
module_param_named(fw_debug, scsc_mx_fw_debug, bool, 0444);
MODULE_PARM_DESC(fw_debug,
		 "enable the firmware's debug output (unifiDebugEnable, default off)");

static unsigned int scsc_mx_fw_debug_level = 4;
module_param_named(fw_debug_level, scsc_mx_fw_debug_level, uint, 0444);
MODULE_PARM_DESC(fw_debug_level,
		 "per-module firmware debug level 0-255 when fw_debug is set (default 4)");

/*
 * A manual override for the priority in MA-UNITDATA.request, mainly
 * for testing a specific access category on demand. Left at its default
 * (negative), scsc_mx_tx_data()'s caller picks per packet instead:
 * FAPI_PRIORITY_CONTENTION for a non-QoS association, the DSCP/802.1p-
 * derived TID for a QoS one -- see scsc_wlan_ndo_xmit(). Negative means
 * "use the caller's own classification"; 0-7 pins every frame to that
 * TID regardless of what it would otherwise classify as.
 */
static int scsc_mx_tx_priority = -1;
module_param_named(tx_priority, scsc_mx_tx_priority, int, 0644);
MODULE_PARM_DESC(tx_priority,
		 "MA-UNITDATA priority override: <0 to classify per packet (default), 0-7 to pin every frame to a QoS TID");

static bool scsc_mx_ap_smoke;
module_param_named(ap_smoke, scsc_mx_ap_smoke, bool, 0444);
MODULE_PARM_DESC(ap_smoke,
		 "at probe, add and remove an AP vif to see whether the firmware allows one (default off)");

static bool scsc_mx_wide_2g4;
module_param_named(wide_2g4, scsc_mx_wide_2g4, bool, 0444);
MODULE_PARM_DESC(wide_2g4,
		 "allow 40 MHz channels in the 2.4 GHz band (unifi24G40MhzChannels, default off)");

static bool scsc_mx_hip_debug;
module_param_named(hip_debug, scsc_mx_hip_debug, bool, 0644);
MODULE_PARM_DESC(hip_debug,
		 "log the six HIP queue indices as data is sent (default off)");

static unsigned int scsc_mx_ungate_clocks;
module_param_named(ungate_clocks, scsc_mx_ungate_clocks, uint, 0444);
MODULE_PARM_DESC(ungate_clocks,
		 "force the WLBT CMU gates open (default 0; the vendor does not)");

/*
 * Whether to restore the six *_SYS_PWR_REG to their boot value before
 * powering on. Our power-off clears bits in them and nothing puts them
 * back; a working system has all six reading 0xffffffff.
 */
static unsigned int scsc_mx_restore_sys_pwr = 1;
module_param_named(restore_sys_pwr, scsc_mx_restore_sys_pwr, uint, 0444);
MODULE_PARM_DESC(restore_sys_pwr,
		 "restore *_SYS_PWR_REG to 0xffffffff before power-on (default 1)");

/*
 * Which service to start once the firmware is up. Default (the param
 * left unset, SCSC_SERVICE_ID_INVALID) is both WLAN and BT -- proven
 * to coexist on this firmware (see the WLAN+BT concurrency finding in
 * bt.md), so there is no reason to make every boot ask for one by
 * hand. Give an explicit 1 or 2 to bring up only one, e.g. to isolate
 * which service a problem is in.
 *
 * ECHO (5) was the obvious candidate for testing a bare service start
 * -- the vendor's do-nothing test service, started with ref 0 by
 * their own client_test.c -- but this firmware does not have it, and
 * says so:
 *
 *	servman.c:66:CMN: Attempt to start unknown service: id %d
 *	(ignoring) -- 0x5
 *
 * Asking for WLAN or BT produces no error at all, so the distinction is
 * the firmware's, not ours.
 *
 * Neither confirms with ref 0: a real service needs the reference to
 * point at its shared-memory layout. Left as a parameter because it is
 * how that will be developed.
 */
static unsigned int scsc_mx_service = SCSC_SERVICE_ID_INVALID;
module_param_named(service, scsc_mx_service, uint, 0444);
MODULE_PARM_DESC(service,
		 "service id to start after boot: 1=WLAN only, 2=BT only, 0xff/unset=both (default)");

enum scsc_mx_cmu {
	SCSC_CMU_CORE,
	SCSC_CMU_APM,
	SCSC_CMU_COUNT,
};

static const struct scsc_mx_cmu_bank {
	const char *name;
	phys_addr_t base;
	size_t size;
} scsc_mx_cmu_banks[SCSC_CMU_COUNT] = {
	[SCSC_CMU_CORE] = { "CMU_CORE", CMU_CORE_BASE, CMU_CORE_SIZE },
	[SCSC_CMU_APM]  = { "CMU_APM", CMU_APM_BASE, CMU_APM_SIZE },
};

static const struct scsc_mx_clk {
	const char *name;
	enum scsc_mx_cmu cmu;
	u32 offset;
	u32 set;
} scsc_mx_clks[] = {
	/* Gates first, then the quality channels that let blocks request them. */
	{ "CLK_CON_GAT_LHM_AXI_D_WLBT", SCSC_CMU_CORE, CLK_CON_GAT_LHM_AXI_D_WLBT,
	  CLK_CON_GAT_MANUAL | CLK_CON_GAT_CG_VAL },
	{ "CLK_CON_GAT_MAILBOX_WLBT", SCSC_CMU_CORE, CLK_CON_GAT_MAILBOX_WLBT,
	  CLK_CON_GAT_MANUAL | CLK_CON_GAT_CG_VAL },
	{ "CLK_CON_GAT_MAILBOX_WLBT_CM4", SCSC_CMU_CORE, CLK_CON_GAT_MAILBOX_WLBT_CM4,
	  CLK_CON_GAT_MANUAL | CLK_CON_GAT_CG_VAL },
	{ "CLK_CON_GAT_MAILBOX_APM2WLBT", SCSC_CMU_APM, CLK_CON_GAT_MAILBOX_APM2WLBT,
	  CLK_CON_GAT_MANUAL | CLK_CON_GAT_CG_VAL },
	{ "QCH_CON_LHM_AXI_D_WLBT", SCSC_CMU_CORE, QCH_CON_LHM_AXI_D_WLBT, QCH_CON_ENABLE },
	{ "QCH_CON_MAILBOX_WLBT", SCSC_CMU_CORE, QCH_CON_MAILBOX_WLBT, QCH_CON_ENABLE },
	{ "QCH_CON_MAILBOX_WLBT_CM4", SCSC_CMU_CORE, QCH_CON_MAILBOX_WLBT_CM4, QCH_CON_ENABLE },
	{ "QCH_CON_MAILBOX_APM2WLBT", SCSC_CMU_APM, QCH_CON_MAILBOX_APM2WLBT, QCH_CON_ENABLE },
};

static void scsc_mx_enable_clocks(struct scsc_mx *mx)
{
	void __iomem *cmu[SCSC_CMU_COUNT] = { };
	int i;

	if (!scsc_mx_ungate_clocks) {
		scsc_mx_vinfo(mx,
			      "leaving WLBT clock gates as found (vendor does not touch them)\n");
		return;
	}

	for (i = 0; i < SCSC_CMU_COUNT; i++) {
		cmu[i] = ioremap(scsc_mx_cmu_banks[i].base,
				 scsc_mx_cmu_banks[i].size);
		if (!cmu[i])
			dev_warn(mx->dev, "failed to map %s\n",
				 scsc_mx_cmu_banks[i].name);
	}

	for (i = 0; i < ARRAY_SIZE(scsc_mx_clks); i++) {
		const struct scsc_mx_clk *c = &scsc_mx_clks[i];
		u32 before, after;

		if (!cmu[c->cmu])
			continue;

		before = readl(cmu[c->cmu] + c->offset);
		writel(before | c->set, cmu[c->cmu] + c->offset);
		after = readl(cmu[c->cmu] + c->offset);

		scsc_mx_vinfo(mx, "%s: 0x%x -> 0x%x%s\n", c->name, before, after,
			      (after & c->set) == c->set ? "" : " (SET FAILED)");
	}

	for (i = 0; i < SCSC_CMU_COUNT; i++)
		if (cmu[i])
			iounmap(cmu[i]);
}

/*
 * PMU system-init writes that the downstream kernel performs at boot
 * via cal-if (pmucal_lpm_init[] in
 * drivers/soc/samsung/cal-if/exynos7885/pmucal_system_exynos7885.h) and
 * mainline's exynos-pmu does not.
 *
 * Diffing that table against live mainline registers showed 22 of its
 * 64 PMU_ALIVE writes missing. Only the ones plausibly on the WLBT/APM
 * path are applied here -- the WLBT firmware's first action is
 * negotiating its LDO32 rail with the APM, so APM power options and the
 * shared regulator are the relevant subset.
 *
 * Deliberately NOT touched, though also mismatched: PS_HOLD_CONTROL
 * (bit 31 is the power hold -- a wrong write powers the phone off),
 * WDTRESET_LPI, and the CPU/cluster/memory options, which belong to
 * subsystems mainline manages differently.
 *
 * These are PMU_ALIVE registers, so they survive a warm reboot; a bad
 * value here needs a battery pull to clear.
 */
#define UP_SCHEDULER			0x0120
#define UP_SCHEDULER_ENABLE		GENMASK(2, 0)
#define TOP_PWR_APM_OPTION		0x82c8
#define MEMORY_APM_TOP_OPTION		0x8228
#define PWR_OPTION_MODE			GENMASK(1, 0)
#define PWR_OPTION_MODE_VAL		0x2
#define EXT_REGULATOR_SHARED_INIT	BIT(0)

static void scsc_mx_pmu_init(struct scsc_mx *mx)
{
	static const struct {
		const char *name;
		u32 offset;
		u32 mask;
		u32 val;
	} seq[] = {
		{ "EXT_REGULATOR_SHARED_OPTION", EXT_REGULATOR_SHARED_OPTION,
		  EXT_REGULATOR_SHARED_INIT, EXT_REGULATOR_SHARED_INIT },
		{ "UP_SCHEDULER", UP_SCHEDULER,
		  UP_SCHEDULER_ENABLE, UP_SCHEDULER_ENABLE },
		{ "TOP_PWR_APM_OPTION", TOP_PWR_APM_OPTION,
		  PWR_OPTION_MODE, PWR_OPTION_MODE_VAL },
		{ "MEMORY_APM_TOP_OPTION", MEMORY_APM_TOP_OPTION,
		  PWR_OPTION_MODE, PWR_OPTION_MODE_VAL },
	};
	unsigned int i;
	u32 before, after;

	for (i = 0; i < ARRAY_SIZE(seq); i++) {
		if (regmap_read(mx->pmureg, seq[i].offset, &before))
			continue;
		if (regmap_update_bits(mx->pmureg, seq[i].offset,
				       seq[i].mask, seq[i].val)) {
			dev_warn(mx->dev, "pmu-init: %s write failed\n",
				 seq[i].name);
			continue;
		}
		regmap_read(mx->pmureg, seq[i].offset, &after);
		scsc_mx_vinfo(mx, "pmu-init: %-28s 0x%08x -> 0x%08x\n",
			      seq[i].name, before, after);
	}
}

/*
 * Ported from the vendor platform_mif_pmu_reset(rst_case=2) plus the
 * reset=true half of platform_mif_reset(): put the WIFI/BT domain back
 * into reset and close its DRAM window.
 *
 * This is what makes the driver reloadable. Without it the domain is
 * only ever powered on, never returned to a state it can be started
 * from, so the R4 boots exactly once per physical power cycle and every
 * later attempt lands on a wedged domain (which reports WIFI_STAT
 * ACCESS_MIF=1 -- a stuck-state marker, not a sign of life).
 *
 * Note the direction of the writes below: clearing these *_SYS_PWR_REG
 * bits and CENTRAL_SEQ_WIFI_CONFIGURATION bit 16 hands the domain back
 * to the PMU's low-power sequencer, which is what actually performs the
 * power-down -- the opposite of the power-on path, which keeps bit 16
 * set to hold the domain under manual control.
 *
 * Status: this does engage the sequencer -- CENTRAL_SEQ_WIFI_STATUS
 * moves off 0x0, where it sits whenever bit 16 is set -- but it stalls
 * at state 0x2d and never reaches 0x80, so a reload still does not get
 * the R4 booting again. Releasing the CMU gates first (on the theory
 * that MANUAL|CG_VAL keeps WLBT clocks alive and blocks the handshake)
 * was tried and is NOT the answer: clearing them mid-flight takes the
 * R4's own AXI master port away and hangs the bus hard enough to reset
 * the SoC. Whatever 0x2d is waiting on is still unidentified.
 */
/*
 * Poll the central sequencer down to CENTRAL_SEQ_STATE_DOWN.
 *
 * Every state it passes through is logged rather than just the last:
 * a sequencer advancing slowly and one pinned on a handshake look
 * identical from a single end-of-timeout sample. Returns 0 once down.
 */
static int scsc_mx_wait_down(struct scsc_mx *mx, const char *phase)
{
	u32 val, state, last_state = U32_MAX;
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(500);
	do {
		if (regmap_read(mx->pmureg, CENTRAL_SEQ_WIFI_STATUS, &val))
			break;

		state = FIELD_GET(CENTRAL_SEQ_STATES, val);
		if (state != last_state) {
			scsc_mx_vinfo(mx, "power-off/%s: sequencer state 0x%x\n",
				      phase, state);
			last_state = state;
		}

		if (state == CENTRAL_SEQ_STATE_DOWN) {
			regmap_read(mx->pmureg, WIFI_STAT, &val);
			scsc_mx_vinfo(mx,
				      "power-off/%s: DOWN reached, WIFI_STAT=0x%x\n",
				      phase, val);
			return 0;
		}

		usleep_range(1000, 2000);
	} while (time_before(jiffies, timeout));

	/*
	 * Not a warning: power_off() escalates on this, and on a reload the
	 * domain gets stuck here at every step without harm to the next
	 * power-on.
	 */
	regmap_read(mx->pmureg, WIFI_STAT, &val);
	scsc_mx_vinfo(mx,
		      "power-off/%s: stuck at 0x%x (want 0x%x), WIFI_STAT=0x%x\n",
		      phase, last_state, CENTRAL_SEQ_STATE_DOWN, val);

	return -ETIMEDOUT;
}

static int scsc_mx_power_off(struct scsc_mx *mx)
{
	static const struct {
		u32 offset;
		u32 mask;
		u32 val;
	} seq[] = {
		{ RESET_AHEAD_WIFI_PWR_REG,	SYS_PWR_CFG_2,	0 },
		{ CLEANY_BUS_WIFI_SYS_PWR_REG,	SYS_PWR_CFG,	0 },
		{ LOGIC_RESET_WIFI_SYS_PWR_REG,	SYS_PWR_CFG_2,	0 },
		{ TCXO_GATE_WIFI_SYS_PWR_REG,	SYS_PWR_CFG,	0 },
		{ WIFI_DISABLE_ISO_SYS_PWR_REG,	SYS_PWR_CFG,	SYS_PWR_CFG },
		{ WIFI_RESET_ISO_SYS_PWR_REG,	SYS_PWR_CFG,	0 },
		{ CENTRAL_SEQ_WIFI_CONFIGURATION, CENTRAL_SEQ_CFG_LOWPWR, 0 },
	};
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(seq); i++) {
		ret = regmap_update_bits(mx->pmureg, seq[i].offset,
					 seq[i].mask, seq[i].val);
		if (ret) {
			dev_warn(mx->dev,
				 "power-off: failed to update PMU 0x%x: %d\n",
				 seq[i].offset, ret);
			return ret;
		}
	}

	/*
	 * rst_case 2: hold the domain in reset rather than dropping
	 * WIFI_PWRON entirely, which is what the vendor does when it
	 * expects to start the chip again later.
	 */
	ret = regmap_update_bits(mx->pmureg, WIFI_CTRL_NS, WIFI_RESET_SET,
				 WIFI_RESET_SET);
	if (ret) {
		dev_warn(mx->dev, "power-off: failed to assert WIFI_RESET_SET: %d\n",
			 ret);
		return ret;
	}

	if (!scsc_mx_wait_down(mx, "reset-hold"))
		goto close_window;

	/*
	 * Escalate. The vendor never gets here -- a 4.4 cycle reaches 0x80
	 * inside its 500ms budget every time -- because it powers down a
	 * firmware that has just acknowledged SM_MSG_STOP_CFM. Ours is still
	 * running, so the sequencer waits on a handshake that never comes.
	 *
	 * rst_case 2 only holds the domain in reset, leaving WIFI_PWRON and
	 * WIFI_START set. Per the datasheet those two are what put the block
	 * in UP state, so drop them: this is the vendor's rst_case 1 (dead
	 * code in their tree) plus WIFI_START, and it asks the PMU to remove
	 * the domain rather than politely request that it stop.
	 *
	 * Worth having even beyond tidiness. Every measurement since the
	 * mxconf fix has started from a domain the previous run left wedged,
	 * and PMU_ALIVE survives a warm reboot, so nothing short of a battery
	 * pull has cleared it. If this works it buys both a clean start per
	 * module reload and the cold-boot measurement we cannot otherwise
	 * take.
	 */
	scsc_mx_vinfo(mx, "power-off: escalating to WIFI_START/WIFI_PWRON clear\n");

	ret = regmap_update_bits(mx->pmureg, WIFI_CTRL_S, WIFI_START, 0);
	if (ret)
		dev_warn(mx->dev, "power-off: failed to clear WIFI_START: %d\n",
			 ret);

	ret = regmap_update_bits(mx->pmureg, WIFI_CTRL_NS, WIFI_PWRON, 0);
	if (ret)
		dev_warn(mx->dev, "power-off: failed to clear WIFI_PWRON: %d\n",
			 ret);

	if (!scsc_mx_wait_down(mx, "power-clear"))
		goto close_window;

	/*
	 * Last resort: stop waiting for the domain to co-operate and tell
	 * the PMU to ignore it.
	 *
	 * MASK_CLKREQ_WIFI makes ALIVE ignore CLKREQ from WLBT, and clearing
	 * SWEEPER_BYPASS_DATA_EN puts CLEANY on the WIFI2AP memory path into
	 * bypass instead of waiting for it to drain. Both are exactly the
	 * "proceed anyway" controls the datasheet provides, and both are in
	 * the state that would keep a sequencer waiting on a core that is
	 * still running.
	 *
	 * (An earlier note in wifi.md ruled CLEANY out on the grounds that
	 * SWEEPER_BYPASS_DATA_EN was clear. It is set -- 0x100ea2 has bit 16
	 * -- and the polarity is inverted from what the name suggests.)
	 */
	scsc_mx_vinfo(mx, "power-off: forcing -- mask CLKREQ, bypass CLEANY\n");

	regmap_update_bits(mx->pmureg, WIFI_DEBUG, MASK_CLKREQ_WIFI,
			   MASK_CLKREQ_WIFI);
	regmap_update_bits(mx->pmureg, WIFI_CTRL_NS, SWEEPER_BYPASS_DATA_EN, 0);

	ret = scsc_mx_wait_down(mx, "force") ? -ETIMEDOUT : 0;

	/*
	 * Put both back if that did not work. These live in PMU_ALIVE, which
	 * survives a warm reboot, so leaving them changed would silently
	 * alter every later run -- the same way a stray write to
	 * CENTRAL_SEQ_WIFI_CONFIGURATION poisoned the domain early on and
	 * invalidated a long run of measurements.
	 */
	if (ret) {
		regmap_update_bits(mx->pmureg, WIFI_DEBUG, MASK_CLKREQ_WIFI, 0);
		regmap_update_bits(mx->pmureg, WIFI_CTRL_NS,
				   SWEEPER_BYPASS_DATA_EN,
				   SWEEPER_BYPASS_DATA_EN);
		scsc_mx_vinfo(mx, "power-off: force failed, override bits restored\n");
	}

close_window:
	/*
	 * Vendor closes the BAAW unconditionally once WLBT is down, even if
	 * the sequencer timed out -- leaving a live DRAM window pointing at
	 * our carveout is worse than the timeout.
	 */
	regmap_write(mx->pmureg, WIFI2AP_MEM_CONFIG0, 0);
	regmap_write(mx->pmureg, WIFI2AP_MEM_CONFIG1, 0);
	regmap_write(mx->pmureg, WIFI2AP_MEM_CONFIG2, 0);
	regmap_write(mx->pmureg, WIFI2AP_MEM_CONFIG3, 0);

	return ret;
}

/*
 * Open the to-host interrupt bits we allocated.
 *
 * INTMR0's upper half masks firmware -> AP interrupts, and a 1 masks.
 * We were writing 0xffff0000 during init and never clearing anything,
 * so every to-host bit stayed masked for the whole run -- meaning the
 * mailbox interrupt could never have reached us no matter what the
 * firmware did, and the "irqs=0" line we have been quoting as evidence
 * could not have read anything else.
 *
 * Found by rebuilding the 4.4 kernel with CONFIG_SCSC_DEBUG and
 * watching the vendor do it: it unmasks each bit as it registers a
 * handler ("UNMASK Setting INTMR0: 0xfffe0000 bit 0", then 0xfffc0000
 * bit 1, and so on), finishing at 0xff020000.
 */
static void scsc_mx_unmask_tohost(struct scsc_mx *mx, unsigned int nbits)
{
	u32 mask = GENMASK(15, 0) & ~GENMASK(nbits - 1, 0);
	u32 val = mask << 16;

	writel(val, mx->r4_regs + INTMR0);
	scsc_mx_vinfo(mx, "INTMR0 <- 0x%08x (to-host bits 0..%u unmasked)\n",
		      val, nbits - 1);
}

/*
 * Ported from the vendor platform_mif_pmu_reset_release(): power on the
 * WIFI/BT domain, release it from reset, then start it. This is the
 * sequence the vendor driver runs before it expects the R4/M4 cores to
 * respond on the mailbox.
 */
static int scsc_mx_power_on(struct scsc_mx *mx)
{
	static const u32 sys_pwr_regs[] = {
		RESET_AHEAD_WIFI_PWR_REG, CLEANY_BUS_WIFI_SYS_PWR_REG,
		LOGIC_RESET_WIFI_SYS_PWR_REG, TCXO_GATE_WIFI_SYS_PWR_REG,
		WIFI_DISABLE_ISO_SYS_PWR_REG, WIFI_RESET_ISO_SYS_PWR_REG,
	};
	u32 ctrl_ns, ctrl_s, val, state, seq_cfg;
	unsigned int i_restore;
	int ret;

	/*
	 * Put the six *_SYS_PWR_REG back to their boot value. A working 4.4
	 * system has all six reading 0xffffffff; ours are left with bits
	 * cleared by the power-off we run at probe, and nothing restores
	 * them -- the vendor never notices because it does not power down
	 * before its first start.
	 */
	if (scsc_mx_restore_sys_pwr) {
		for (i_restore = 0; i_restore < ARRAY_SIZE(sys_pwr_regs); i_restore++)
			regmap_write(mx->pmureg, sys_pwr_regs[i_restore],
				     0xffffffff);
		scsc_mx_vinfo(mx, "restored six *_SYS_PWR_REG to 0xffffffff\n");
	}

	regmap_read(mx->pmureg, CENTRAL_SEQ_WIFI_CONFIGURATION, &seq_cfg);
	scsc_mx_vinfo(mx, "CENTRAL_SEQ_WIFI_CONFIGURATION=0x%x (LOWPWR bit16=%lu, %s)\n",
		      seq_cfg, FIELD_GET(CENTRAL_SEQ_CFG_LOWPWR, seq_cfg),
		      FIELD_GET(CENTRAL_SEQ_CFG_LOWPWR, seq_cfg) ? "manual, correct for power-on"
								  : "auto-lowpower, unexpected");

	/*
	 * A previous bring-up iteration left this cleared (wrong direction,
	 * see comment above the register defines) and it survives module
	 * reload since it's a physical PMU register, not driver state.
	 * Force it back to the correct value defensively on every probe.
	 */
	ret = regmap_update_bits(mx->pmureg, CENTRAL_SEQ_WIFI_CONFIGURATION,
				 CENTRAL_SEQ_CFG_LOWPWR, CENTRAL_SEQ_CFG_LOWPWR);
	if (ret)
		return dev_err_probe(mx->dev, ret,
				      "failed to set CENTRAL_SEQ_WIFI_CONFIGURATION LOWPWR\n");

	/*
	 * Claim the shared external regulator before driving the domain.
	 * Vendor brackets the whole sequence below in this.
	 */
	regmap_read(mx->pmureg, EXT_REGULATOR_SHARED_STATUS, &val);
	scsc_mx_vinfo(mx, "EXT_REGULATOR_SHARED_STATUS before=0x%x\n", val);

	ret = regmap_update_bits(mx->pmureg, EXT_REGULATOR_SHARED_OPTION,
				 EXT_REGULATOR_SHARED_EN, EXT_REGULATOR_SHARED_EN);
	if (ret)
		return dev_err_probe(mx->dev, ret,
				      "failed to claim shared external regulator\n");

	/* Vendor allows for a settling delay when CP hasn't already run. */
	usleep_range(1000, 2000);
	regmap_read(mx->pmureg, EXT_REGULATOR_SHARED_STATUS, &val);
	scsc_mx_vinfo(mx, "EXT_REGULATOR_SHARED_STATUS after=0x%x\n", val);

	/*
	 * 36-bit mask: this field takes a physical address wider than 32
	 * bits, and the low 14 bits must be clear (16K alignment) before
	 * the >>12 that gives the 4K-granular window base.
	 */
	val = (mx->mem_base & 0xFFFFFC000ULL) >> 12;
	ret = regmap_write(mx->pmureg, WIFI2AP_MEM_CONFIG1, val);
	if (ret)
		return dev_err_probe(mx->dev, ret, "failed to set WIFI2AP_MEM_CONFIG1\n");

	val = mx->mem_size >> 12;
	ret = regmap_write(mx->pmureg, WIFI2AP_MEM_CONFIG0, val);
	if (ret)
		return dev_err_probe(mx->dev, ret, "failed to set WIFI2AP_MEM_CONFIG0\n");

	/*
	 * SECTION_1 is the BT-ABOX shared window. Left alone it comes out of
	 * S-Boot as a 4MB window based at 0x70000000 -- below DRAM, in
	 * peripheral space -- so it must not simply be left as found.
	 *
	 * Zeroing it looked like the safe choice, but a known-good 4.4 boot
	 * programs it on every reset release, and the combo firmware image
	 * has no way to know BT is not in use: an unmapped alias faults into
	 * nothing, which matches the symptom of an R4 that runs briefly and
	 * halts without leaving a panic record. So mirror the vendor and
	 * point it at a real carve-out when we have one.
	 */
	if (mx->abox_size) {
		val = (mx->abox_base & 0xFFFFFC000ULL) >> 12;
		regmap_write(mx->pmureg, WIFI2AP_MEM_CONFIG3, val);
		val = mx->abox_size >> 12;
		regmap_write(mx->pmureg, WIFI2AP_MEM_CONFIG2, val);
	} else {
		regmap_write(mx->pmureg, WIFI2AP_MEM_CONFIG2, 0);
		regmap_write(mx->pmureg, WIFI2AP_MEM_CONFIG3, 0);
	}

	ret = mx->acpm->ops->dvfs.set_flag(mx->acpm, mx->acpm_chan,
					   SCSC_ACPM_SET_FLAG_ID,
					   SCSC_ACPM_SET_FLAG_ON);
	if (ret)
		return dev_err_probe(mx->dev, ret, "failed to set ACPM flag\n");

	ret = regmap_update_bits(mx->pmureg, WIFI_CTRL_NS, WIFI_PWRON, WIFI_PWRON);
	if (ret)
		return dev_err_probe(mx->dev, ret, "failed to set WIFI_PWRON\n");

	ret = regmap_update_bits(mx->pmureg, WIFI_CTRL_NS, WIFI_RESET_SET, 0);
	if (ret)
		return dev_err_probe(mx->dev, ret, "failed to release WIFI_RESET_SET\n");

	ret = regmap_update_bits(mx->pmureg, WIFI_CTRL_S, WIFI_START, WIFI_START);
	if (ret)
		return dev_err_probe(mx->dev, ret, "failed to set WIFI_START\n");

	/*
	 * Release the shared regulator claim. The vendor brackets only the
	 * reset release itself -- platform_mif_pmu_reset_release() calls
	 * exynos_pmu_shared_reg_enable() on the way in and
	 * exynos_pmu_shared_reg_disable() on the way out -- whereas we were
	 * setting the bit and leaving it set forever. The claim exists to
	 * stop BUCK2 dipping while WLBT draws inrush current; holding it
	 * afterwards is not what the working configuration does.
	 */
	ret = regmap_update_bits(mx->pmureg, EXT_REGULATOR_SHARED_OPTION,
				 EXT_REGULATOR_SHARED_EN, 0);
	if (ret)
		return dev_err_probe(mx->dev, ret,
				      "failed to release shared external regulator\n");

	regmap_read(mx->pmureg, EXT_REGULATOR_SHARED_STATUS, &val);
	scsc_mx_vinfo(mx, "EXT_REGULATOR_SHARED_STATUS released=0x%x (0x20001 is the vendor's normal)\n",
		      val);

	regmap_read(mx->pmureg, WIFI_CTRL_NS, &ctrl_ns);
	regmap_read(mx->pmureg, WIFI_CTRL_S, &ctrl_s);
	scsc_mx_vinfo(mx, "power-on sequence done: WIFI_CTRL_NS=0x%x WIFI_CTRL_S=0x%x\n",
		      ctrl_ns, ctrl_s);

	regmap_read(mx->pmureg, CENTRAL_SEQ_WIFI_STATUS, &val);
	state = FIELD_GET(CENTRAL_SEQ_STATES, val);
	scsc_mx_vinfo(mx, "CENTRAL_SEQ_WIFI_STATUS=0x%x (state=0x%x, %s)\n",
		      val, state, state == 0x80 ? "up" : "not up");

	regmap_read(mx->pmureg, WIFI_STAT, &val);
	scsc_mx_vinfo(mx, "WIFI_STAT=0x%x (PWRDN_DONE=%lu ACCESS_MIF=%lu)\n",
		      val, FIELD_GET(WIFI_PWRDN_DONE, val), FIELD_GET(WIFI_ACCESS_MIF, val));

	return 0;
}


/*
 * Probe whether the mailbox SFR window is actually live, using ISSR(0)
 * as a pure scratchpad (no hardware semantics, so a failed read-back
 * can only mean the block isn't clocked/powered).
 */
static bool scsc_mx_mailbox_is_live(struct scsc_mx *mx)
{
	static const u32 pattern = 0xa5a5f00f;
	u32 val;

	writel(pattern, mx->r4_regs + ISSR(0));
	val = readl(mx->r4_regs + ISSR(0));
	scsc_mx_vinfo(mx, "R4 ISSR(0) scratchpad test: wrote 0x%x read 0x%x (%s)\n",
		      pattern, val, val == pattern ? "live" : "DEAD");

	return val == pattern;
}

/*
 * Mailbox reset, mirroring the vendor platform_mif_map() on its
 * exynos7885 path exactly: zero both scratchpads, mask everything, then
 * clear any latched interrupts. Runs before the R4 leaves reset.
 *
 * Note this masks all interrupts rather than unmasking them -- the
 * vendor unmasks individual bits later as transports are registered.
 * (An earlier revision of this driver copied TizenRT's INTMR=0 here,
 * but that is a different SoC's port; 7885 masks.)
 *
 * The M4 window gets the same treatment: on 7885 the vendor mirrors
 * every ISSR write into the M4 register block and masks/clears its
 * INTMR1/INTCR1 too.
 */
static void scsc_mx_mailbox_init(struct scsc_mx *mx)
{
	int i;

	for (i = 0; i < 8; i++) {
		writel(0, mx->r4_regs + ISSR(i));
		writel(0, mx->m4_regs + ISSR(i));
	}

	/*
	 * Mask every to-host bit for now; scsc_mx_unmask_tohost() opens the
	 * ones we actually hand out once mxconf is built.
	 */
	writel(0xffff0000, mx->r4_regs + INTMR0);
	writel(0x0000ffff, mx->r4_regs + INTMR1);
	writel(0x0000ffff, mx->m4_regs + INTMR1);

	writel(0xffff0000, mx->r4_regs + INTCR0);
	writel(0x0000ffff, mx->r4_regs + INTCR1);
	writel(0x0000ffff, mx->m4_regs + INTCR1);
}

/*
 * Walk the firmware's log ring and print whatever it managed to say
 * before dying. This is the firmware describing its own failure, which
 * beats inferring one from an undocumented panic record.
 *
 * The ring is a circular buffer of 4-byte packets; the firmware is the
 * only writer, and we have never advanced the read index, so everything
 * from 0 up to the write index is unread.
 */
#define SCSC_LOG_STRINGS_NAME	"log-strings.bin"
/* Longest format string we will render; the table's are far shorter. */
#define SCSC_LOGSTR_MAX		192

/*
 * Resolve a phase-5 format-string offset against log-strings.bin.
 *
 * The file is a flat table of NUL-separated strings indexed by byte
 * offset, which is exactly what a record carries. Copies into the
 * caller's buffer rather than returning a pointer, because the blob is
 * not guaranteed to be NUL-terminated at its end and the result is
 * firmware-controlled data that must never reach printk as a format
 * string -- it is always printed with %s.
 */
static const char *scsc_mx_logstr(struct scsc_mx *mx, u32 off, char *buf,
				  size_t buflen)
{
	const char *data;
	size_t avail, len;

	if (!mx->logstr)
		return NULL;

	if (off >= mx->logstr->size) {
		snprintf(buf, buflen, "<offset 0x%x past end of table>", off);
		return buf;
	}

	data = (const char *)mx->logstr->data + off;
	avail = mx->logstr->size - off;
	len = strnlen(data, min(avail, buflen - 1));

	memcpy(buf, data, len);
	buf[len] = '\0';
	strreplace(buf, '\n', ' ');

	return buf;
}

/*
 * Decode one mxlog record, advancing *idxp past it.
 *
 * Returns false if the ring no longer makes sense, which must stop the
 * caller: a desynchronised ring will otherwise produce nonsense
 * forever.
 */
static bool scsc_mx_mxlog_record(struct scsc_mx *mx, u32 *idxp)
{
	struct scsc_mx_stream *s = &mx->mxlog;
	char buf[MXLOG_MAX_RECORD + 1];
	u32 idx = *idxp;
	u32 header = readl(mx->dram + s->buffer + idx * 4);
	u16 sync = header >> 16;
	u8 level = (header >> 8) & 0xff;
	u32 len = header & 0xff;
	u32 bytes, words, i;

	if (sync == MXLOG_SYNC_PHASE_4) {
		bytes = len;
		words = DIV_ROUND_UP(len, 4);
	} else if (sync == MXLOG_SYNC_PHASE_5) {
		words = len;
		bytes = len * 4;
	} else {
		scsc_mx_vinfo(mx,
			      "mxlog: lost sync at packet %u (header 0x%08x), stopping\n",
			      idx, header);
		return false;
	}

	idx = (idx + 1) % s->num_packets;

	if (!words || bytes > MXLOG_MAX_RECORD) {
		scsc_mx_vinfo(mx,
			      "mxlog: implausible record length %u, stopping\n", bytes);
		return false;
	}

	/* Payload may wrap, so copy it a packet at a time. */
	for (i = 0; i < words; i++) {
		u32 w = readl(mx->dram + s->buffer +
			      ((idx + i) % s->num_packets) * 4);

		memcpy(buf + i * 4, &w, 4);
	}

	if (sync == MXLOG_SYNC_PHASE_4) {
		buf[bytes] = '\0';
		strreplace(buf, '\n', ' ');
		scsc_mx_vinfo(mx, "mxlog[%u]: %s\n", level, buf);
	} else if (words >= 2) {
		/*
		 * Phase 5 is tokenised: timestamp, then a byte offset into
		 * log-strings.bin, then the format arguments. Arguments are
		 * printed as hex rather than substituted -- rendering them
		 * properly needs the vendor's per-conversion casting, and
		 * passing a firmware-supplied string to printk as a format
		 * is not something to do for cosmetics.
		 */
		u32 tstamp = get_unaligned_le32(buf);
		u32 off = get_unaligned_le32(buf + 4);
		u32 nargs = words - 2;
		char fmtbuf[SCSC_LOGSTR_MAX];
		char args[3 + 11 * 4] = "";
		const char *fmt;

		fmt = scsc_mx_logstr(mx, off, fmtbuf, sizeof(fmtbuf));

		for (i = 0; i < min(nargs, 4u); i++)
			snprintf(args + strlen(args), sizeof(args) - strlen(args),
				 " 0x%x", get_unaligned_le32(buf + 8 + i * 4));

		if (fmt)
			scsc_mx_vinfo(mx, "mxlog[%u] %08x: %s%s%s\n",
				      level, tstamp, fmt, nargs ? " --" : "", args);
		else
			scsc_mx_vinfo(mx,
				      "mxlog[%u] %08x: fmt@0x%x%s%s (no %s)\n",
				      level, tstamp, off, nargs ? " --" : "", args,
				      SCSC_LOG_STRINGS_NAME);
	} else {
		scsc_mx_vinfo(mx, "mxlog[%u]: runt phase5 record, %u words\n",
			      level, words);
	}

	*idxp = (idx + words) % s->num_packets;

	return true;
}

/*
 * Consume whatever the firmware has logged since we last looked.
 *
 * This used to start at packet 0 and read up to the write index, which
 * was fine for a dump at a fixed point in probe but reprints the whole
 * ring every time and cannot survive a wrap. Following the read index
 * instead makes it a consumer: it can be driven from the interrupt the
 * firmware raises when it logs, which turns the log from a post-mortem
 * into a running commentary -- and the firmware says considerably more
 * about what it is refusing to do than any host-side signal does.
 */
#define MXLOG_DRAIN_MAX		256

static void scsc_mx_mxlog_drain(struct scsc_mx *mx)
{
	struct scsc_mx_stream *s = &mx->mxlog;
	u32 ridx, widx;
	unsigned int n;

	if (!s->widx_loc || !s->num_packets)
		return;

	ridx = readl(mx->dram + s->ridx_loc);
	widx = readl(mx->dram + s->widx_loc);

	if (ridx >= s->num_packets || widx >= s->num_packets) {
		dev_warn(mx->dev, "mxlog: indices out of range (r %u w %u)\n",
			 ridx, widx);
		return;
	}

	for (n = 0; n < MXLOG_DRAIN_MAX && ridx != widx; n++)
		if (!scsc_mx_mxlog_record(mx, &ridx))
			break;

	if (!n)
		return;

	/* Consumed slots visible before the interrupt that announces them. */
	writel(ridx, mx->dram + s->ridx_loc);
	wmb();
	writel(BIT(s->read_bit), mx->r4_regs + INTGR1);
}

/*
 * Dump the R4 panic record. Word 0 is the version, word 1 the length in
 * *bytes*; for version 1 words 2-3 are timestamps followed by 18 saved
 * registers (panic_record_r4_defs.h).
 *
 * The firmware on this device writes version 2, 212 bytes / 53 words.
 * The vendor's own fw_parse_r4_panic_record_v2() only checksums the
 * record and names the sympathetic-panic flag at word 51, but the rest
 * is recoverable by combining three sources:
 *
 *   - v2 inserts a *length* word at index 1 (their
 *     R4_PANIC_RECORD_LENGTH_INDEX_V2), which shifts the v1 layout by
 *     one. That is why decoding the registers at the v1 index produces
 *     nonsense -- pc reads 0 and cpsr an address. They start at 8.
 *   - mxman.c takes the panic subcode from word 3 (masked 0x7fff).
 *   - word 2 is a byte offset into log-strings.bin naming the panic,
 *     which is what the v1 header means by "key/index of panic_string".
 *
 * The checksum is xor32 over all words but the last, seeded with
 * 0xa5a5a5a5, and it validates -- so this is a real decode rather than
 * a plausible-looking one. Words past the register block are still
 * printed raw: they carry stack residue and are not worth guessing at.
 *
 * Called both before overwriting DRAM (to recover the *previous* run's
 * record, which reloading the module would otherwise destroy) and after
 * reset release.
 */
static void scsc_mx_dump_panic_record(struct scsc_mx *mx, void __iomem *dram,
				      const char *when)
{
	u32 words[R4_PANIC_RECORD_MAX_WORDS];
	u32 version, length, count;
	unsigned int i;

	if (!mx->r4_panic_record_offset ||
	    mx->r4_panic_record_offset + 8 > mx->mem_size)
		return;

	version = readl(dram + mx->r4_panic_record_offset);
	length = readl(dram + mx->r4_panic_record_offset + 4);

	if (!version) {
		/*
		 * "Empty" is a judgement about word 0 only, and the one 64K
		 * chunk the R4 dirties is the one this record lives in -- so
		 * show the surrounding bytes rather than asserting nothing is
		 * there. A record written at a different offset, or with a
		 * layout we guessed wrong, looks identical to no record at
		 * all from word 0.
		 */
		if (!scsc_mx_verbose)
			return;
		memcpy_fromio(words, dram + mx->r4_panic_record_offset, 64);
		dev_info(mx->dev, "%s: R4 panic record @0x%x reads empty (word 0 zero)\n",
			 when, mx->r4_panic_record_offset);
		print_hex_dump(KERN_INFO, "scsc_mx: panic area: ",
			       DUMP_PREFIX_OFFSET, 16, 4, words, 64, false);
		return;
	}

	count = length / 4;
	if (count > R4_PANIC_RECORD_MAX_WORDS)
		count = R4_PANIC_RECORD_MAX_WORDS;
	if (mx->r4_panic_record_offset + count * 4 > mx->mem_size)
		count = (mx->mem_size - mx->r4_panic_record_offset) / 4;

	memcpy_fromio(words, dram + mx->r4_panic_record_offset, count * 4);

	dev_info(mx->dev, "%s: R4 PANIC RECORD @0x%x: version %u, %u bytes (%u words)\n",
		 when, mx->r4_panic_record_offset, version, length, count);

	if (version == R4_PANIC_RECORD_VERSION_2 &&
	    count == R4_PANIC_RECORD_V2_WORDS) {
		static const char *const regs[] = {
			"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
			"r8", "r9", "r10", "r11", "r12", "sp", "lr",
			"spsr", "pc", "cpsr",
		};
		char fmtbuf[SCSC_LOGSTR_MAX];
		const char *reason;
		u32 cksum = PANIC_RECORD_CKSUM_SEED;

		for (i = 0; i < count - 1; i++)
			cksum ^= words[i];

		reason = scsc_mx_logstr(mx, words[R4_PANIC_RECORD_V2_STRING],
					fmtbuf, sizeof(fmtbuf));

		dev_info(mx->dev, "%s:   checksum 0x%08x %s\n", when,
			 words[count - 1],
			 cksum == words[count - 1] ? "VALID" : "MISMATCH -- decode below is unreliable");
		/*
		 * The offset matters as well as the string: the table
		 * holds a separate copy per call site, so two panics with
		 * identical text are distinguishable by offset, and the
		 * strings either side of it in the table name the module
		 * the call site lives in.
		 */
		dev_info(mx->dev, "%s:   PANIC: [str@0x%x] %s (subcode 0x%x)%s\n",
			 when, words[R4_PANIC_RECORD_V2_STRING],
			 reason ?: "<no string table>",
			 words[R4_PANIC_RECORD_V2_SUBCODE] & SCSC_PANIC_SUBCODE_MASK,
			 words[R4_PANIC_RECORD_V2_SYMPATHETIC] ?
				" [sympathetic -- the other core panicked first]" : "");
		/*
		 * Words 4-7 are the panic info block, which is where the
		 * arguments to the panic string's own %x conversions live
		 * -- printed rather than substituted, for the same reason
		 * mxlog arguments are.
		 */
		dev_info(mx->dev, "%s:   info: %08x %08x %08x %08x\n", when,
			 words[4], words[5], words[6], words[7]);

		/* Three per line: 18 registers divides exactly. */
		for (i = 0; i < ARRAY_SIZE(regs); i += 3)
			dev_info(mx->dev, "%s:   %-4s %08x  %-4s %08x  %-4s %08x\n",
				 when,
				 regs[i], words[R4_PANIC_RECORD_V2_REGS + i],
				 regs[i + 1], words[R4_PANIC_RECORD_V2_REGS + i + 1],
				 regs[i + 2], words[R4_PANIC_RECORD_V2_REGS + i + 2]);

		i = R4_PANIC_RECORD_V2_REGS + ARRAY_SIZE(regs);
	} else {
		i = 4;
	}

	for (; i < count; i += 4)
		dev_info(mx->dev, "%s:   [%2u] %08x %08x %08x %08x\n", when, i,
			 words[i],
			 i + 1 < count ? words[i + 1] : 0,
			 i + 2 < count ? words[i + 2] : 0,
			 i + 3 < count ? words[i + 3] : 0);
}

/*
 * Bump allocator over the shared-DRAM heap, which starts where the
 * firmware's runtime image ends.
 *
 * The vendor uses a 64-byte-block bitmap allocator (miframman) because
 * services come and go at runtime. Nothing here ever frees: everything
 * is allocated once per probe and torn down wholesale on reload, so a
 * bump pointer is all that is warranted.
 *
 * Offsets returned are R4-relative. Zero is never a valid allocation --
 * it is the base of the firmware image -- so it doubles as failure.
 */
/* miframman's block size; see scsc_mx_ram_alloc(). */
#define SCSC_MIFRAM_BLOCK	64

static u32 scsc_mx_ram_alloc(struct scsc_mx *mx, struct scsc_mx_ram *ram,
			     u32 size, u32 align)
{
	u32 off = ALIGN(ram->next, align);

	if (off + size > ram->end) {
		dev_err(mx->dev, "shared RAM exhausted (need %u at 0x%x, end 0x%x)\n",
			size, off, ram->end);
		return 0;
	}

	/*
	 * Charge each allocation a whole number of 64-byte blocks, which is
	 * what the vendor's miframman does. Read out of a working 4.4
	 * system, every allocation consumes ALIGN(size + 8, 64): a 512-byte
	 * buffer advances the cursor by 0x240, 2048 by 0x840, 16384 by
	 * 0x4040, and a 4-byte index by 0x40. (The +8 is miframman's own
	 * per-block header; we do not need the header itself, only the
	 * spacing it produces.)
	 *
	 * The point is cache-line separation. This memory is shared with a
	 * core that is not coherent with us, so a ring's read and write
	 * indices must not share a line -- otherwise a write-back of one
	 * side's dirty line silently reverts the other side's update. We
	 * were packing them 4 bytes apart, in the same line.
	 *
	 * Note this cannot explain the stage-9 hang, which is firmware-
	 * internal mutex creation that never touches this memory. It is
	 * fixed because it is a real bug that would bite later, and because
	 * it is the last layout difference from the known-good reference.
	 */
	ram->next = off + ALIGN(size + 8, SCSC_MIFRAM_BLOCK);
	memset_io(ram->base + off, 0, size);

	return off;
}

/*
 * MIF interrupt bit allocation. The indices are only meaningful because
 * both sides agree on them via mxconf, so what matters is that we hand
 * out each bit once, in the same order the vendor does.
 */
static int scsc_mx_intr_alloc(struct scsc_mx *mx, u8 *counter)
{
	if (*counter >= MIFINTRBIT_NUM_INT) {
		dev_err(mx->dev, "out of MIF interrupt bits\n");
		return -ENOSPC;
	}

	return (*counter)++;
}

/*
 * Build one simplex stream: a circular packet buffer plus its read and
 * write indices, and the pair of interrupt bits that signal them.
 *
 * Direction is from the firmware's point of view of who writes. For an
 * IN stream (firmware -> AP) the firmware writes and signals us on a
 * to-host bit, so read_bit_idx is the from-host bit and write_bit_idx
 * the to-host one; an OUT stream is the mirror image. Getting this pair
 * backwards is silent -- both sides just wait forever -- so it follows
 * mif_stream_init() exactly.
 */
static int scsc_mx_stream_build(struct scsc_mx *mx, struct scsc_mx_ram *ram,
				struct scsc_mx_intr *intr, u32 conf,
				bool in, bool m4, bool panic_bit,
				u32 buf_length, u32 packet_size,
				struct scsc_mx_stream *rec)
{
	u32 num_packets = buf_length / packet_size;
	u32 buffer, ridx, widx;
	int tohost, fromhost;
	u8 *fromhost_ctr = m4 ? &intr->fromhost_m4 : &intr->fromhost_r4;

	buffer = scsc_mx_ram_alloc(mx, ram, buf_length, 4);
	ridx = scsc_mx_ram_alloc(mx, ram, sizeof(u32), 4);
	widx = scsc_mx_ram_alloc(mx, ram, sizeof(u32), 4);
	if (!buffer || !ridx || !widx)
		return -ENOMEM;

	/*
	 * Vendor fills an inbound buffer with 0xff rather than leaving it
	 * zeroed; keep that, since a zeroed packet is a valid-looking
	 * channel 0 message.
	 */
	if (in)
		memset_io(ram->base + buffer, 0xff, buf_length);

	tohost = scsc_mx_intr_alloc(mx, &intr->tohost);
	if (tohost < 0)
		return tohost;

	/*
	 * The gdb out-streams deliberately reuse the reserved panic bit
	 * instead of allocating, which is how the host forces a firmware
	 * panic through the gdb channel.
	 */
	fromhost = panic_bit ? MIFINTRBIT_RESERVED_PANIC
			     : scsc_mx_intr_alloc(mx, fromhost_ctr);
	if (fromhost < 0)
		return fromhost;

	writel(buffer, ram->base + conf + MXCBUFCONF_BUFFER_LOC);
	writel(num_packets, ram->base + conf + MXCBUFCONF_NUM_PACKETS);
	writel(packet_size, ram->base + conf + MXCBUFCONF_PACKET_SIZE);
	writel(ridx, ram->base + conf + MXCBUFCONF_READ_INDEX_LOC);
	writel(widx, ram->base + conf + MXCBUFCONF_WRITE_INDEX_LOC);

	writeb(in ? fromhost : tohost, ram->base + conf + MXSTREAMCONF_READ_BIT_IDX);
	writeb(in ? tohost : fromhost, ram->base + conf + MXSTREAMCONF_WRITE_BIT_IDX);

	if (rec) {
		rec->buffer = buffer;
		rec->ridx_loc = ridx;
		rec->widx_loc = widx;
		rec->num_packets = num_packets;
		rec->packet_size = packet_size;
		rec->read_bit = in ? fromhost : tohost;
		rec->write_bit = in ? tohost : fromhost;
	}

	dev_dbg(mx->dev,
		"stream @0x%x: buf 0x%x %ux%u, idx r 0x%x w 0x%x, bits r %d w %d\n",
		conf, buffer, num_packets, packet_size, ridx, widx,
		in ? fromhost : tohost, in ? tohost : fromhost);

	return 0;
}

/* A transport is a pair of streams: firmware->AP then AP->firmware. */
static int scsc_mx_transport_build(struct scsc_mx *mx, struct scsc_mx_ram *ram,
				   struct scsc_mx_intr *intr, u32 conf,
				   bool m4, bool gdb,
				   u32 buf_length, u32 packet_size,
				   struct scsc_mx_transport *rec)
{
	int ret;

	ret = scsc_mx_stream_build(mx, ram, intr, conf + MXTRANSCONF_TO_AP,
				   true, m4, false, buf_length, packet_size,
				   rec ? &rec->in : NULL);
	if (ret)
		return ret;

	return scsc_mx_stream_build(mx, ram, intr, conf + MXTRANSCONF_FROM_AP,
				    false, m4, gdb, buf_length, packet_size,
				    rec ? &rec->out : NULL);
}

/*
 * Read the serialised config back out of DRAM and print it decoded.
 *
 * This is a self-check on the most error-prone code here: a swapped
 * read/write bit index, a field written at the wrong offset, or the two
 * streams of a transport in the wrong order are all invisible from the
 * AP -- the firmware just refuses to start -- so the bytes are verified
 * against the layout rather than trusted.
 */
static void scsc_mx_dump_mxconf(struct scsc_mx *mx, void __iomem *dram, u32 conf)
{
	static const struct {
		const char *name;
		u32 off;
		bool pair;
	} parts[] = {
		{ "mgmt",   MXCONF_OFF_MGMT,   true  },
		{ "gdb_r4", MXCONF_OFF_GDB_R4, true  },
		{ "gdb_m4", MXCONF_OFF_GDB_M4, true  },
		{ "mxlog",  MXCONF_OFF_MXLOG,  false },
	};
	unsigned int i, j;

	scsc_mx_vinfo(mx, "mxconf verify: magic 0x%08x version %u.%u\n",
		      readl(dram + conf + MXCONF_OFF_MAGIC),
		      readw(dram + conf + MXCONF_OFF_VERSION_MAJOR),
		      readw(dram + conf + MXCONF_OFF_VERSION_MINOR));

	for (i = 0; i < ARRAY_SIZE(parts); i++) {
		for (j = 0; j < (parts[i].pair ? 2u : 1u); j++) {
			u32 s = conf + parts[i].off +
				(j ? MXTRANSCONF_FROM_AP : MXTRANSCONF_TO_AP);

			scsc_mx_vinfo(mx,
				      "mxconf verify: %-6s %-7s buf 0x%06x n=%-4u sz=%-2u ridx 0x%06x widx 0x%06x rbit %u wbit %u\n",
				      parts[i].name, j ? "from_ap" : "to_ap",
				      readl(dram + s + MXCBUFCONF_BUFFER_LOC),
				      readl(dram + s + MXCBUFCONF_NUM_PACKETS),
				      readl(dram + s + MXCBUFCONF_PACKET_SIZE),
				      readl(dram + s + MXCBUFCONF_READ_INDEX_LOC),
				      readl(dram + s + MXCBUFCONF_WRITE_INDEX_LOC),
				      readb(dram + s + MXSTREAMCONF_READ_BIT_IDX),
				      readb(dram + s + MXSTREAMCONF_WRITE_BIT_IDX));
		}
	}
}

/*
 * Lay out the infrastructure config the firmware picks up from ISSR(1):
 * the management transport it reports readiness on, a gdb transport per
 * core, and the log stream.
 *
 * Only the management transport is consumed by this driver so far, but
 * all four have to be present and valid -- the firmware walks the whole
 * structure at startup.
 */
static u32 scsc_mx_build_mxconf(struct scsc_mx *mx, void __iomem *dram,
				u32 heap_start)
{
	/*
	 * Allocator and interrupt-bit state live in mx rather than on the
	 * stack: a service started later carves its shared memory out of
	 * the same heap and its interrupt bits out of the same space, and
	 * both have to continue where this left off rather than restart.
	 */
	struct scsc_mx_intr *intr = &mx->intr;
	struct scsc_mx_ram *ram = &mx->ram;
	u32 conf;
	int ret;

	intr->tohost = 0;
	/* Bit 0 of each from-host register is the panic bit. */
	intr->fromhost_r4 = MIFINTRBIT_RESERVED_PANIC + 1;
	intr->fromhost_m4 = MIFINTRBIT_RESERVED_PANIC + 1;

	ram->base = dram;
	ram->next = heap_start;
	ram->end = mx->mem_size;

	/*
	 * Over-allocate. MXCONF_SIZE is the structure as it stood in the
	 * 4.14 BSP, but this firmware is from 2022 and the structure may
	 * have grown since. The allocator packs allocations back to back,
	 * so a firmware reading past 162 bytes would otherwise be parsing
	 * the management ring buffer as config; zeros are at least a
	 * defined value. scsc_mx_ram_alloc() clears what it hands out.
	 */
	conf = scsc_mx_ram_alloc(mx, ram, MXCONF_ALLOC_SIZE, 4);
	if (!conf)
		return 0;

	writel(MXCONF_MAGIC, dram + conf + MXCONF_OFF_MAGIC);
	writew(MXCONF_VERSION_MAJOR, dram + conf + MXCONF_OFF_VERSION_MAJOR);
	writew(MXCONF_VERSION_MINOR, dram + conf + MXCONF_OFF_VERSION_MINOR);

	ret = scsc_mx_transport_build(mx, ram, intr, conf + MXCONF_OFF_MGMT,
				      false, false, MXMGMT_BUF_LENGTH,
				      MXMGMT_PACKET_SIZE, &mx->mgmt);
	if (ret)
		return 0;

	ret = scsc_mx_transport_build(mx, ram, intr, conf + MXCONF_OFF_GDB_R4,
				      false, true, GDB_BUF_LENGTH,
				      GDB_PACKET_SIZE, NULL);
	if (ret)
		return 0;

	ret = scsc_mx_transport_build(mx, ram, intr, conf + MXCONF_OFF_GDB_M4,
				      true, true, GDB_BUF_LENGTH,
				      GDB_PACKET_SIZE, NULL);
	if (ret)
		return 0;

	/* mxlogconf is a bare mxstreamconf, firmware -> AP only. */
	ret = scsc_mx_stream_build(mx, ram, intr, conf + MXCONF_OFF_MXLOG,
				   true, false, false, MXLOG_BUF_LENGTH,
				   MXLOG_PACKET_SIZE, &mx->mxlog);
	if (ret)
		return 0;

	scsc_mx_dump_mxconf(mx, dram, conf);

	scsc_mx_vinfo(mx,
		      "mxconf @0x%x, heap 0x%x-0x%x used, %u to-host / %u+%u from-host irq bits\n",
		      conf, heap_start, ram->next, intr->tohost, intr->fromhost_r4,
		      intr->fromhost_m4);

	/* Now that we know how many we handed out, open them. */
	scsc_mx_unmask_tohost(mx, intr->tohost);

	/*
	 * From here the interrupt handler may walk the management ring,
	 * so it must not be published before the rings are laid out --
	 * and before the to-host bits are unmasked there is nothing to
	 * publish it for. The firmware log's ring is laid out by the same
	 * pass and is armed with it.
	 */
	mx->mgmt_ready = true;
	mx->mxlog_ready = true;

	return conf;
}

/*
 * The management protocol.
 *
 * Once the firmware has finished starting it announces itself with
 * MM_START_IND on channel 0 of the management transport, and from then
 * on the transport is the only way to talk to it: every service the
 * chip offers is negotiated over rings in shared DRAM, signalled by MIF
 * mailbox interrupts, with no doorbell registers involved.
 *
 * MM_START_IND needs no reply -- it is an indication, not a request --
 * so the "handshake" is really: wait for it, and answer MM_HALT_REQ /
 * MM_HALT_RSP on the way down. Waiting for it matters anyway, because
 * it is the point at which the chip is known good, and halting matters
 * because it lets the firmware stop itself rather than having the power
 * pulled from under it.
 */
static const char *scsc_mx_chip_version(u32 rf_hw_ver)
{
	switch (rf_hw_ver & 0xff) {
	case 0xb0:
		return (rf_hw_ver & 0xff00) > 0x1000 ? "S610/S611" : "S610";
	case 0xb1:
		return "S612";
	case 0xb2:
		return "S620";
	case 0x00:
		return "error: check if RF chip is present";
	default:
		return "unknown";
	}
}

static const char *scsc_mx_mm_msg_name(u8 msg)
{
	static const char *const names[] = {
		[MM_START_IND]		= "MM_START_IND",
		[MM_HALT_REQ]		= "MM_HALT_REQ",
		[MM_FORCE_PANIC]	= "MM_FORCE_PANIC",
		[MM_HOST_SUSPEND]	= "MM_HOST_SUSPEND",
		[MM_HOST_RESUME]	= "MM_HOST_RESUME",
		[MM_FW_CONFIG]		= "MM_FW_CONFIG",
		[MM_HALT_RSP]		= "MM_HALT_RSP",
	};

	return msg < ARRAY_SIZE(names) && names[msg] ? names[msg] : "unknown";
}

static const char *scsc_mx_sm_msg_name(u8 msg)
{
	static const char *const names[] = {
		[SM_MSG_START_REQ]	= "SM_MSG_START_REQ",
		[SM_MSG_START_CFM]	= "SM_MSG_START_CFM",
		[SM_MSG_STOP_REQ]	= "SM_MSG_STOP_REQ",
		[SM_MSG_STOP_CFM]	= "SM_MSG_STOP_CFM",
	};

	return msg < ARRAY_SIZE(names) && names[msg] ? names[msg] : "unknown";
}

static const char *scsc_mx_service_name(u8 id)
{
	static const char *const names[] = {
		[SCSC_SERVICE_ID_NULL]		= "NULL",
		[SCSC_SERVICE_ID_WLAN]		= "WLAN",
		[SCSC_SERVICE_ID_BT]		= "BT",
		[SCSC_SERVICE_ID_ANT]		= "ANT",
		[SCSC_SERVICE_ID_R4DBG]		= "R4DBG",
		[SCSC_SERVICE_ID_ECHO]		= "ECHO",
		[SCSC_SERVICE_ID_DBG_SAMPLER]	= "DBG_SAMPLER",
		[SCSC_SERVICE_ID_CLK20MHZ]	= "CLK20MHZ",
	};

	return id < ARRAY_SIZE(names) && names[id] ? names[id] : "unknown";
}

static void scsc_mx_sm_deliver(struct scsc_mx *mx, const u8 *pkt)
{
	u8 service_id = pkt[SM_MSG_OFF_SERVICE_ID];
	u8 msg = pkt[SM_MSG_OFF_MSG];
	u32 data = get_unaligned_le32(pkt + SM_MSG_OFF_DATA);

	/*
	 * Two services can be active at once (see struct scsc_mx's
	 * service_active[]), but scsc_mx_service_start()/_stop() are
	 * synchronous, so only one start/stop is ever outstanding -- the
	 * id is checked against that one rather than used to look a
	 * service up.
	 */
	if (service_id != mx->service_pending) {
		dev_warn(mx->dev,
			 "srv: %s for %s (%u), but %s is pending\n",
			 scsc_mx_sm_msg_name(msg),
			 scsc_mx_service_name(service_id), service_id,
			 scsc_mx_service_name(mx->service_pending));
		return;
	}

	switch (msg) {
	case SM_MSG_START_CFM:
		scsc_mx_vinfo(mx, "srv: SM_MSG_START_CFM for %s (data 0x%x)\n",
			      scsc_mx_service_name(service_id), data);
		complete(&mx->sm_start_cfm);
		break;
	case SM_MSG_STOP_CFM:
		scsc_mx_vinfo(mx, "srv: SM_MSG_STOP_CFM for %s (data 0x%x)\n",
			      scsc_mx_service_name(service_id), data);
		complete(&mx->sm_stop_cfm);
		break;
	default:
		dev_warn(mx->dev, "srv: unhandled %s (%u) for %s, data 0x%x\n",
			 scsc_mx_sm_msg_name(msg), msg,
			 scsc_mx_service_name(service_id), data);
		break;
	}
}

static void scsc_mx_mgmt_deliver(struct scsc_mx *mx, const u8 *pkt)
{
	u8 channel = pkt[MM_MSG_OFF_CHANNEL];
	u8 msg = pkt[MM_MSG_OFF_MSG];
	u32 arg = get_unaligned_le32(pkt + MM_MSG_OFF_ARG);

	if (channel == MMTRANS_CHAN_ID_SERVICE_MANAGEMENT) {
		scsc_mx_sm_deliver(mx, pkt);
		return;
	}

	if (channel != MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT) {
		/*
		 * Channel 2 belongs to mxlogger, which does not exist
		 * yet, so a message on it is worth reporting rather than
		 * silently dropping.
		 */
		dev_warn(mx->dev,
			 "mgmt: message on unhandled channel %u (msg %u arg 0x%x)\n",
			 channel, msg, arg);
		return;
	}

	switch (msg) {
	case MM_START_IND:
		/*
		 * The argument is the RF hardware version, which is the
		 * first thing the chip has ever told us about itself.
		 */
		mx->rf_hw_ver = arg;
		dev_info(mx->dev,
			 "mgmt: MM_START_IND -- RF_CHIP_VERSION: 0x%04x: %s (0x%02x), EVT%x.%x\n",
			 arg, scsc_mx_chip_version(arg), arg & 0xff,
			 (arg >> 12) & 0xf, (arg >> 8) & 0xf);
		complete(&mx->mm_start_ind);
		break;
	case MM_HALT_RSP:
		scsc_mx_vinfo(mx, "mgmt: MM_HALT_RSP\n");
		complete(&mx->mm_halt_rsp);
		break;
	default:
		dev_warn(mx->dev, "mgmt: unhandled %s (%u), arg 0x%x\n",
			 scsc_mx_mm_msg_name(msg), msg, arg);
		break;
	}
}

/*
 * Drain the firmware -> AP ring.
 *
 * Indices are packet counts modulo the ring length (vendor
 * cpacket_buffer.c), with one slot always left free so that
 * read == write means empty rather than ambiguous. The read index is
 * advanced before the message is delivered, matching the vendor's
 * peek/peek_complete order: a handler is allowed to send a reply, and
 * that reply must not find the ring still claiming to be full.
 */
static void scsc_mx_mgmt_drain(struct scsc_mx *mx)
{
	struct scsc_mx_stream *in = &mx->mgmt.in;
	unsigned int drained = 0;
	u32 ridx, widx;

	if (!mx->mgmt_ready)
		return;

	ridx = readl(mx->dram + in->ridx_loc);
	widx = readl(mx->dram + in->widx_loc);

	if (ridx >= in->num_packets || widx >= in->num_packets) {
		dev_warn(mx->dev,
			 "mgmt: ring indices out of range (r %u w %u, %u packets)\n",
			 ridx, widx, in->num_packets);
		return;
	}

	while (ridx != widx) {
		u8 pkt[MXMGMT_PACKET_SIZE];

		memcpy_fromio(pkt, mx->dram + in->buffer + ridx * in->packet_size,
			      in->packet_size);

		ridx = (ridx + 1) % in->num_packets;
		writel(ridx, mx->dram + in->ridx_loc);
		/* The consumed slot must be visible before we signal. */
		wmb();

		scsc_mx_mgmt_deliver(mx, pkt);
		drained++;

		/* More may have arrived while we were in the handler. */
		widx = readl(mx->dram + in->widx_loc);
		if (widx >= in->num_packets)
			break;
	}

	/*
	 * Tell the firmware space came free. One kick for the whole batch
	 * rather than the vendor's one per message: the bit carries no
	 * count, so it says the same thing either way.
	 */
	if (drained)
		writel(BIT(in->read_bit), mx->r4_regs + INTGR1);
}

/*
 * Post one message on the AP -> firmware ring.
 *
 * The wire format is the vendor's scatter-write flattened: the channel
 * id, then that channel's payload, then padding to the packet size. The
 * payload layout is per channel and not interchangeable -- channel 0
 * puts a message byte at 1 and a u32 at 2, channel 1 puts a service id
 * at 1, a message byte at 2 and a u32 at 3 -- so the caller supplies
 * the bytes and this only handles the ring. The vendor leaves the
 * padding as whatever the ring held; zeroing it is free and makes a
 * capture reproducible.
 */
static int scsc_mx_mgmt_send(struct scsc_mx *mx, u8 channel,
			     const void *payload, size_t len,
			     const char *what)
{
	struct scsc_mx_stream *out = &mx->mgmt.out;
	u8 pkt[MXMGMT_PACKET_SIZE] = {};
	u32 ridx, widx, next;

	if (!mx->mgmt_ready)
		return -ENODEV;

	if (len + 1 > sizeof(pkt)) {
		dev_warn(mx->dev, "mgmt: %s payload too big (%zu bytes)\n",
			 what, len);
		return -EINVAL;
	}

	ridx = readl(mx->dram + out->ridx_loc);
	widx = readl(mx->dram + out->widx_loc);

	if (ridx >= out->num_packets || widx >= out->num_packets) {
		dev_warn(mx->dev,
			 "mgmt: send with ring indices out of range (r %u w %u)\n",
			 ridx, widx);
		return -EIO;
	}

	next = (widx + 1) % out->num_packets;
	if (next == ridx) {
		dev_warn(mx->dev, "mgmt: ring full, dropping %s\n", what);
		return -ENOSPC;
	}

	pkt[0] = channel;
	memcpy(pkt + 1, payload, len);

	memcpy_toio(mx->dram + out->buffer + widx * out->packet_size, pkt,
		    out->packet_size);
	/* Packet before index, index before interrupt. */
	wmb();
	writel(next, mx->dram + out->widx_loc);
	wmb();
	writel(BIT(out->write_bit), mx->r4_regs + INTGR1);

	scsc_mx_vinfo(mx, "mgmt: sent %s\n", what);

	return 0;
}

/* Channel 0: __packed { u8 ma_msg; u32 arg; }. */
static int scsc_mx_mm_send(struct scsc_mx *mx, u8 msg, u32 arg)
{
	u8 payload[MM_MSG_SIZE - 1];
	char what[48];

	payload[0] = msg;
	put_unaligned_le32(arg, payload + 1);

	snprintf(what, sizeof(what), "%s (arg 0x%x)",
		 scsc_mx_mm_msg_name(msg), arg);

	return scsc_mx_mgmt_send(mx, MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT,
				 payload, sizeof(payload), what);
}

/*
 * Channel 1: __packed { u8 service_id; u8 msg; u32 optional_data; }.
 *
 * optional_data is an R4-relative reference to whatever shared memory
 * the service needs. The vendor's own test client passes 0 -- services
 * that need no layout handed to them accept that -- which is what makes
 * a bare service start testable without building a data path first.
 */
static int scsc_mx_sm_send(struct scsc_mx *mx, u8 service_id, u8 msg, u32 ref)
{
	u8 payload[SM_MSG_SIZE - 1];
	char what[80];

	payload[0] = service_id;
	payload[1] = msg;
	put_unaligned_le32(ref, payload + 2);

	snprintf(what, sizeof(what), "%s for %s (ref 0x%x)",
		 scsc_mx_sm_msg_name(msg), scsc_mx_service_name(service_id),
		 ref);

	return scsc_mx_mgmt_send(mx, MMTRANS_CHAN_ID_SERVICE_MANAGEMENT,
				 payload, sizeof(payload), what);
}

/*
 * Wait for the firmware to announce itself.
 *
 * The completion is signalled from the interrupt path, but the ring is
 * also drained by hand on timeout: a message sitting in DRAM with no
 * interrupt to go with it is a transport bug worth telling apart from a
 * firmware that never started, and the distinction is invisible if we
 * only ever wait.
 */
static int scsc_mx_wait_start_ind(struct scsc_mx *mx)
{
	unsigned long left;

	left = wait_for_completion_timeout(&mx->mm_start_ind,
					   msecs_to_jiffies(MM_START_IND_TIMEOUT_MS));
	if (left) {
		dev_info(mx->dev, "firmware up: MM_START_IND after %u ms\n",
			 MM_START_IND_TIMEOUT_MS - jiffies_to_msecs(left));
		return 0;
	}

	scsc_mx_mgmt_drain(mx);

	if (try_wait_for_completion(&mx->mm_start_ind)) {
		dev_warn(mx->dev,
			 "MM_START_IND was in the ring but no interrupt arrived\n");
		return 0;
	}

	dev_warn(mx->dev, "no MM_START_IND within %u ms\n",
		 MM_START_IND_TIMEOUT_MS);

	return -ETIMEDOUT;
}

/*
 * Ask the firmware to stop, and give it a moment to say it has.
 *
 * This build does not answer with MM_HALT_RSP, which the vendor also
 * allows for ("not implemented in all versions of firmware, so don't
 * treat its non-arrival as an error"), and it does not advance the read
 * index either. It does act on the request: its own log says
 *
 *	Imminent WLBT shutdown notified...
 *	RFIC powered down
 *	WLBT shutting down...
 *
 * which is the whole point -- the RF front end gets powered down by the
 * firmware that owns it, rather than losing its supply mid-flight.
 */
static void scsc_mx_mgmt_halt(struct scsc_mx *mx)
{
	if (scsc_mx_mm_send(mx, MM_HALT_REQ, 0))
		return;

	if (!wait_for_completion_timeout(&mx->mm_halt_rsp,
					 msecs_to_jiffies(MM_HALT_RSP_TIMEOUT_MS)))
		scsc_mx_vinfo(mx,
			      "no MM_HALT_RSP within %u ms (not all builds send one)\n",
			      MM_HALT_RSP_TIMEOUT_MS);

	/*
	 * With neither a response nor a read index to go on, the log is
	 * the acknowledgement -- so dump it here rather than leaving the
	 * shutdown unwitnessed.
	 */
	scsc_mx_vinfo(mx, "mgmt ring after halt: from-AP r %u w %u\n",
		      readl(mx->dram + mx->mgmt.out.ridx_loc),
		      readl(mx->dram + mx->mgmt.out.widx_loc));

	scsc_mx_mxlog_drain(mx);
}

/*
 * Start a service, and stop it again.
 *
 * The vendor's scsc_mx_service_open() sends nothing -- it allocates a
 * service structure and boots the firmware, both of which have already
 * happened by the time we get here -- so a start is exactly one
 * SM_MSG_START_REQ and a wait for the confirmation.
 *
 * The reference passed as optional_data is where a real service would
 * point the firmware at its shared-memory layout. ECHO needs none, and
 * neither does the vendor's test client for any service it starts.
 */
static int scsc_mx_service_start(struct scsc_mx *mx, u8 service_id, u32 ref)
{
	int ret;

	mx->service_pending = service_id;
	reinit_completion(&mx->sm_start_cfm);

	ret = scsc_mx_sm_send(mx, service_id, SM_MSG_START_REQ, ref);
	if (ret)
		return ret;

	if (!wait_for_completion_timeout(&mx->sm_start_cfm,
					 msecs_to_jiffies(SM_CFM_TIMEOUT_MS))) {
		dev_warn(mx->dev, "srv: no SM_MSG_START_CFM for %s within %u ms\n",
			 scsc_mx_service_name(service_id), SM_CFM_TIMEOUT_MS);
		return -ETIMEDOUT;
	}

	mx->service_active[service_id] = true;
	dev_info(mx->dev, "srv: %s started\n", scsc_mx_service_name(service_id));

	return 0;
}

static void scsc_mx_service_stop(struct scsc_mx *mx, u8 service_id)
{
	if (!mx->service_active[service_id])
		return;

	/*
	 * Close the data path before asking the service to stop: the
	 * queues live in memory the firmware is about to stop servicing,
	 * and a bottom half arriving after the stop would walk them anyway.
	 */
	if (service_id == SCSC_SERVICE_ID_WLAN && mx->hip.ready) {
		mx->hip.ready = false;
		cancel_work_sync(&mx->hip.work);
	}
	if (service_id == SCSC_SERVICE_ID_BT && mx->bt.ready) {
		mx->bt.ready = false;
		cancel_work_sync(&mx->bt.work);
	}

	mx->service_pending = service_id;
	reinit_completion(&mx->sm_stop_cfm);

	if (scsc_mx_sm_send(mx, service_id, SM_MSG_STOP_REQ, 0))
		return;

	if (!wait_for_completion_timeout(&mx->sm_stop_cfm,
					 msecs_to_jiffies(SM_CFM_TIMEOUT_MS)))
		dev_warn(mx->dev, "srv: no SM_MSG_STOP_CFM for %s within %u ms\n",
			 scsc_mx_service_name(service_id),
			 SM_CFM_TIMEOUT_MS);
	else
		dev_info(mx->dev, "srv: %s stopped\n",
			 scsc_mx_service_name(service_id));

	mx->service_active[service_id] = false;
}

/*
 * Bluetooth service start.
 *
 * The reference handed over in SM_MSG_START_REQ points at a struct BHCS
 * -- 28 bytes describing where the real protocol structure lives. That
 * structure, BSMHCP_PROTOCOL, is ~57K of rings and buffers which only
 * matter once HCI traffic flows; what matters at start time is that its
 * header carries BSMHCP_PROTOCOL_MAGICVALUE, which the firmware checks.
 *
 * The magic is not a constant: it is built from BSMHCP_ACL_BUFFER_SIZE,
 * BSMHCP_CMD_EVT_BUFFER_SIZE, offsetof(BSMHCP_PROTOCOL, acl_tx_buffer)
 * and sizeof(BSMHCP_PROTOCOL). It is an ABI hash, and the reason bhcs.h
 * and bsmhcp.h are copied in verbatim rather than retyped: a structure
 * that differs from the firmware's by one field computes a different
 * magic and is rejected. (Contrast mxconf, where the vendor header's
 * __packed markings did *not* describe the real ABI -- there the header
 * was the trap. Here the header is the contract.)
 *
 * Five interrupt bits are allocated and named in the header.
 *
 * The configuration blob *is* needed, which the host side does not say:
 * scsc_bt_module.c zeroes configuration_offset/length when the file is
 * missing and starts anyway, but the firmware then logs
 *
 *	BT_PSCONF: Loading %u bytes of config from: -- 0x0
 *
 * and panics. It is the firmware that insists, not the driver. The
 * vendor's file is bluetooth/bt.hcf; ours is the same blob under the
 * flattened name it was dumped as.
 *
 * A Bluetooth address is required too. scsc_bt_module.c zeroes the
 * three address fields at the top of setup_bhcs(), which reads like
 * "optional" -- but sixty lines further down it fills them from
 * exynos_soc_info.unique_id. With them left at zero the firmware panics
 * with panic string
 *
 *	lc_init.c:304:BT: PANIC_NO_BLUETOOTH_ADDRESS_CONFIGURED(0x%x)
 *
 * scsc_mx_resolve_bt_addr() mirrors setup_bhcs()'s layered resolution
 * (persistent provisioning wins, then a manual override, then a
 * fallback) rather than its exact sources, since neither of the
 * downstream driver's real ones ports as-is: mainline's exynos-chipid
 * does not expose this part's SoC unique id, and there is no
 * equivalent here yet of its /csa/bluetooth/.bd_addr persistent file.
 */
#define SCSC_BT_CONF_NAME	"mx140_bt.hcf"

/*
 * 0 means "unset": scsc_mx_resolve_bt_addr() only honours this as an
 * override when it is nonzero, the same convention the downstream
 * driver's own bluetooth_address param uses.
 */
static u64 scsc_mx_bt_addr;
module_param_named(bt_addr, scsc_mx_bt_addr, ullong, 0444);
MODULE_PARM_DESC(bt_addr,
		 "Bluetooth device address override; 0 (default) resolves local-bd-address, then falls back to a random one");
#define BSMHCP_OFF(field)	offsetof(struct BSMHCP_PROTOCOL, header.field)
#define BHCS_OFF(field)		offsetof(struct BHCS, field)

/*
 * Resolve the address to hand the firmware, in priority order:
 *
 *  1. The 'local-bd-address' fwnode property on this device's own DT
 *     node -- the same property name/byte order mainline's own
 *     hci_dev_get_bd_addr_from_property() reads for other BT
 *     transports, so any future per-device provisioning (an NVMEM
 *     cell, a factory DT overlay) that sets it needs no driver change
 *     to be honoured. Not read via that generic mechanism, though:
 *     unlike a chip with its own address storage, this firmware has
 *     none of its own and has to be told at start time, so the
 *     resolved address has to reach BHCS before the service starts,
 *     not patched into hdev->public_addr afterward.
 *  2. scsc_mx_bt_addr, if the module param was given a nonzero value
 *     -- a manual override for bring-up/test, matching the downstream
 *     driver's bluetooth_address param.
 *  3. A random locally-administered address, generated fresh this
 *     boot. This is the one place this deliberately falls short of
 *     the downstream driver: its equivalent fallback derives from
 *     exynos_soc_info.unique_id, so unprovisioned devices still get a
 *     stable-across-reboots address that differs per physical unit.
 *     Nothing here can do that yet -- no unique id, no persistent
 *     storage to remember a generated one in -- so this fallback is
 *     only unique per boot, not per device, and a classic BT bond
 *     will not survive a reboot until this board has real per-device
 *     address storage to read a local-bd-address property from.
 */
static u64 scsc_mx_resolve_bt_addr(struct scsc_mx *mx)
{
	u8 prop[6], rnd[6];
	u64 addr;

	if (!device_property_read_u8_array(mx->dev, "local-bd-address",
					   prop, sizeof(prop))) {
		addr = (u64)prop[0] | (u64)prop[1] << 8 | (u64)prop[2] << 16 |
		       (u64)prop[3] << 24 | (u64)prop[4] << 32 |
		       (u64)prop[5] << 40;
		if (addr) {
			scsc_mx_vinfo(mx, "bt: address from local-bd-address property\n");
			return addr;
		}
	}

	if (scsc_mx_bt_addr) {
		scsc_mx_vinfo(mx, "bt: address from bt_addr module param\n");
		return scsc_mx_bt_addr;
	}

	get_random_bytes(rnd, sizeof(rnd));
	rnd[0] = (rnd[0] & ~0x01) | 0x02; /* unicast, locally administered */
	dev_warn(mx->dev,
		 "bt: no local-bd-address property or bt_addr param -- using a random address that will not survive reboot\n");
	return (u64)rnd[0] | (u64)rnd[1] << 8 | (u64)rnd[2] << 16 |
	       (u64)rnd[3] << 24 | (u64)rnd[4] << 32 | (u64)rnd[5] << 40;
}

static int scsc_mx_bt_start(struct scsc_mx *mx)
{
	u32 bhcs, proto, config = 0, config_len = 0;
	int bg_to_ap, ap_to_bg, ap_to_fg, info_bg_to_ap, info_ap_to_bg, ret;
	const struct firmware *conf;
	u64 bt_addr = scsc_mx_resolve_bt_addr(mx);

	bhcs = scsc_mx_ram_alloc(mx, &mx->ram, sizeof(struct BHCS),
				 BSMHCP_ALIGNMENT);
	proto = scsc_mx_ram_alloc(mx, &mx->ram, sizeof(struct BSMHCP_PROTOCOL),
				  BSMHCP_ALIGNMENT);
	if (!bhcs || !proto)
		return -ENOMEM;

	/* scsc_mx_ram_alloc() zeroes what it hands out, which is what the
	 * vendor does explicitly before setting the magic.
	 */
	writel(BSMHCP_PROTOCOL_MAGICVALUE, mx->dram + proto + BSMHCP_OFF(magic_value));

	bg_to_ap = scsc_mx_intr_alloc(mx, &mx->intr.tohost);
	info_bg_to_ap = scsc_mx_intr_alloc(mx, &mx->intr.tohost);
	ap_to_bg = scsc_mx_intr_alloc(mx, &mx->intr.fromhost_r4);
	ap_to_fg = scsc_mx_intr_alloc(mx, &mx->intr.fromhost_r4);
	info_ap_to_bg = scsc_mx_intr_alloc(mx, &mx->intr.fromhost_r4);
	if (bg_to_ap < 0 || info_bg_to_ap < 0 || ap_to_bg < 0 ||
	    ap_to_fg < 0 || info_ap_to_bg < 0)
		return -ENOSPC;

	writew(bg_to_ap, mx->dram + proto + BSMHCP_OFF(bg_to_ap_int_src));
	writew(ap_to_bg, mx->dram + proto + BSMHCP_OFF(ap_to_bg_int_src));
	writew(ap_to_fg, mx->dram + proto + BSMHCP_OFF(ap_to_fg_int_src));
	writew(info_bg_to_ap, mx->dram + proto + BSMHCP_OFF(info_bg_to_ap_int_src));
	writew(info_ap_to_bg, mx->dram + proto + BSMHCP_OFF(info_ap_to_bg_int_src));

	if (!request_firmware(&conf, SCSC_BT_CONF_NAME, mx->dev)) {
		config_len = conf->size;
		config = scsc_mx_ram_alloc(mx, &mx->ram, config_len,
					   BSMHCP_ALIGNMENT);
		if (config)
			memcpy_toio(mx->dram + config, conf->data, config_len);
		else
			config_len = 0;
		release_firmware(conf);
	} else {
		dev_warn(mx->dev,
			 "bt: no %s; the firmware panics without one\n",
			 SCSC_BT_CONF_NAME);
	}

	writel(BHCS_VERSION, mx->dram + bhcs + BHCS_OFF(version));
	writel(proto, mx->dram + bhcs + BHCS_OFF(bsmhcp_protocol_offset));
	writel(sizeof(struct BSMHCP_PROTOCOL),
	       mx->dram + bhcs + BHCS_OFF(bsmhcp_protocol_length));
	writel(config, mx->dram + bhcs + BHCS_OFF(configuration_offset));
	writel(config_len, mx->dram + bhcs + BHCS_OFF(configuration_length));

	writel(bt_addr & 0xffffff,
	       mx->dram + bhcs + BHCS_OFF(bluetooth_address_lap));
	writeb((bt_addr >> 24) & 0xff,
	       mx->dram + bhcs + BHCS_OFF(bluetooth_address_uap));
	writew((bt_addr >> 32) & 0xffff,
	       mx->dram + bhcs + BHCS_OFF(bluetooth_address_nap));

	/* The firmware must see all of that before the request. */
	wmb();

	/* Open the two to-host bits we just handed out. */
	scsc_mx_unmask_tohost(mx, mx->intr.tohost);

	scsc_mx_vinfo(mx,
		      "bt: BHCS v%u @0x%x -> BSMHCP 0x%x+0x%zx, addr 0x%012llx, config 0x%x+%u, magic 0x%08x, irqs bg2ap %d ap2bg %d ap2fg %d info %d/%d\n",
		      BHCS_VERSION, bhcs, proto, sizeof(struct BSMHCP_PROTOCOL),
		      bt_addr, config, config_len, (u32)BSMHCP_PROTOCOL_MAGICVALUE,
		      bg_to_ap, ap_to_bg, ap_to_fg, info_bg_to_ap, info_ap_to_bg);

	mx->bt.proto = proto;
	mx->bt.ap_to_bg = ap_to_bg;
	mx->bt.ap_to_fg = ap_to_fg;
	mx->bt.bg_to_ap = bg_to_ap;

	ret = scsc_mx_service_start(mx, SCSC_SERVICE_ID_BT, bhcs);
	if (ret)
		return ret;

	/*
	 * Publishing bt.ready last is what lets the interrupt handler
	 * start dispatching -- same ordering as mx->hip.ready in
	 * scsc_mx_wlan_start().
	 */
	mx->bt.ready = true;

	return scsc_bt_register(mx);
}

/*
 * Post one HCI command into hci_cmd_transfer_ring and kick the R4.
 *
 * mailbox_hci_cmd_write is ours to advance; mailbox_hci_cmd_read is the
 * R4's. BSMHCP_HAS_ROOM() against that pair is the same producer/
 * consumer check scsc_mx_hip_q_add() runs for the HIP4 queues, just
 * against these two mailbox words instead of a scoreboard byte pair.
 */
int scsc_mx_bt_cmd_transfer(struct scsc_mx *mx, const u8 *data, u16 len)
{
	u32 td, write, read;

	if (!mx->bt.ready)
		return -ENODEV;

	if (len > BSMHCP_CMD_EVT_BUFFER_SIZE)
		return -EMSGSIZE;

	mutex_lock(&mx->bt.lock);

	write = readl(mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_hci_cmd_write));
	read = readl(mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_hci_cmd_read));
	/* read is the R4's; order this load against its store to it. */
	rmb();

	if (!BSMHCP_HAS_ROOM(write, read, BSMHCP_TRANSFER_RING_CMD_SIZE)) {
		mutex_unlock(&mx->bt.lock);
		return -ENOSPC;
	}

	td = mx->bt.proto +
	     offsetof(struct BSMHCP_PROTOCOL, hci_cmd_transfer_ring) +
	     write * sizeof(struct BSMHCP_TD_CONTROL);

	writew(len, mx->dram + td + offsetof(struct BSMHCP_TD_CONTROL, length));
	memcpy_toio(mx->dram + td + offsetof(struct BSMHCP_TD_CONTROL, data),
		    data, len);

	/* The entry must be visible before the index that publishes it. */
	wmb();

	BSMHCP_INCREASE_INDEX(write, BSMHCP_TRANSFER_RING_CMD_SIZE);
	writel(write, mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_hci_cmd_write));
	writel(BIT(mx->bt.ap_to_bg), mx->r4_regs + INTGR1);

	mutex_unlock(&mx->bt.lock);

	return 0;
}

/* handle(2, low 12 bits) + PB/BC(high nibble of byte 1) + length(2). */
#define SCSC_BT_ACL_HDR_LEN	4
/* L2CAP PDU header: length(2) + channel id(2), right after the above. */
#define SCSC_L2CAP_HDR_LEN	4

/*
 * Post one HCI ACL data packet into acl_tx_data_transfer_ring.
 *
 * Unlike a command, a TX descriptor here does not carry its payload
 * inline -- it references a slot in the shared acl_tx_buffer pool by
 * buffer_index, so posting one also has to manage that pool (30
 * slots, tracked in bt.tx_buf_used; freed indices arrive later on
 * acl_tx_free_transfer_ring, drained in scsc_mx_bt_work()).
 *
 * BSMHCP also wants every fragment of an L2CAP PDU marked with the
 * channel id and an explicit "this fragment finishes the PDU" flag
 * (BSMHCP_ACL_L2CAP_FLAG_END). hci_core already fragments and flags
 * PB=START/CONT correctly, but the CID only appears in a START
 * fragment's L2CAP header, so a CONT fragment's descriptor has to
 * recover it from mx->bt.conn[handle], filled in when the START
 * fragment went by. Mirrors the downstream driver's
 * scsc_bt_shm_h4_acl_write() (scsc_shm.c, the ABI's actual reference
 * -- BSMHCP_TD_ACL_TX_DATA's fields do not explain this on their own).
 */
int scsc_mx_bt_acl_transfer(struct scsc_mx *mx, const u8 *data, u16 len)
{
	struct scsc_mx_bt_conn *conn;
	u32 td, write, read, buf;
	unsigned long idx;
	u16 handle, payload_len, l2cap_len;
	u8 flags;

	if (!mx->bt.ready)
		return -ENODEV;

	if (len < SCSC_BT_ACL_HDR_LEN)
		return -EINVAL;

	payload_len = len - SCSC_BT_ACL_HDR_LEN;
	if (payload_len > BSMHCP_ACL_BUFFER_SIZE)
		return -EMSGSIZE;

	handle = get_unaligned_le16(data) & 0x0fff;
	if (handle >= SCSC_BT_MAX_CONN_HANDLES)
		return -EINVAL;
	conn = &mx->bt.conn[handle];

	/* PB<<4 | BC<<6, straight out of the handle field's high byte --
	 * the same bits BSMHCP_TD_ACL_TX_DATA.flags wants, already in
	 * place; BSMHCP_ACL_L2CAP_FLAG_END (bit 0) is added below.
	 */
	flags = data[1] & 0xf0;

	mutex_lock(&mx->bt.lock);

	idx = find_first_zero_bit(&mx->bt.tx_buf_used, BSMHCP_DATA_BUFFER_TX_ACL_SIZE);
	if (idx >= BSMHCP_DATA_BUFFER_TX_ACL_SIZE) {
		mutex_unlock(&mx->bt.lock);
		return -ENOSPC;
	}

	write = readl(mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_acl_tx_write));
	read = readl(mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_acl_tx_read));
	/* read is the R4's; order this load against its store to it. */
	rmb();

	if (!BSMHCP_HAS_ROOM(write, read, BSMHCP_TRANSFER_RING_ACL_SIZE)) {
		mutex_unlock(&mx->bt.lock);
		return -ENOSPC;
	}

	switch (flags >> 4) {
	case 0: /* BSMHCP_ACL_PB_FLAG_START_NONFLUSH */
	case 2: /* BSMHCP_ACL_PB_FLAG_START_FLUSH */
		if (payload_len < SCSC_L2CAP_HDR_LEN) {
			mutex_unlock(&mx->bt.lock);
			return -EINVAL;
		}
		l2cap_len = get_unaligned_le16(data + SCSC_BT_ACL_HDR_LEN);
		conn->l2cap_cid = get_unaligned_le16(data + SCSC_BT_ACL_HDR_LEN + 2);
		if (l2cap_len <= payload_len - SCSC_L2CAP_HDR_LEN) {
			/* Whole PDU in this one fragment (or an over-long
			 * one -- tolerated rather than rejected, since a
			 * conformance tester is allowed to pad).
			 */
			flags |= BSMHCP_ACL_L2CAP_FLAG_END;
			conn->remaining = 0;
		} else {
			conn->remaining = l2cap_len - (payload_len - SCSC_L2CAP_HDR_LEN);
		}
		break;
	case 1: /* BSMHCP_ACL_PB_FLAG_CONT */
		if (conn->remaining <= payload_len) {
			flags |= BSMHCP_ACL_L2CAP_FLAG_END;
			conn->remaining = 0;
		} else {
			conn->remaining -= payload_len;
		}
		break;
	default: /* BSMHCP_ACL_PB_FLAG_RSVD_3 */
		mutex_unlock(&mx->bt.lock);
		return -EINVAL;
	}

	set_bit(idx, &mx->bt.tx_buf_used);

	td = mx->bt.proto +
	     offsetof(struct BSMHCP_PROTOCOL, acl_tx_data_transfer_ring) +
	     write * sizeof(struct BSMHCP_TD_ACL_TX_DATA);
	buf = mx->bt.proto +
	      offsetof(struct BSMHCP_PROTOCOL, acl_tx_buffer) +
	      idx * BSMHCP_ACL_BUFFER_SIZE;

	writew(payload_len, mx->dram + td + offsetof(struct BSMHCP_TD_ACL_TX_DATA, length));
	writeb(idx, mx->dram + td + offsetof(struct BSMHCP_TD_ACL_TX_DATA, buffer_index));
	writeb(flags, mx->dram + td + offsetof(struct BSMHCP_TD_ACL_TX_DATA, flags));
	writew(handle, mx->dram + td + offsetof(struct BSMHCP_TD_ACL_TX_DATA, hci_connection_handle));
	writew(conn->l2cap_cid, mx->dram + td + offsetof(struct BSMHCP_TD_ACL_TX_DATA, l2cap_cid));
	memcpy_toio(mx->dram + buf, data + SCSC_BT_ACL_HDR_LEN, payload_len);

	/* The entry and its buffer must be visible before the index that
	 * publishes them. */
	wmb();

	BSMHCP_INCREASE_INDEX(write, BSMHCP_TRANSFER_RING_ACL_SIZE);
	writel(write, mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_acl_tx_write));
	writel(BIT(mx->bt.ap_to_fg), mx->r4_regs + INTGR1);

	mutex_unlock(&mx->bt.lock);

	return 0;
}

/*
 * Drain hci_evt_transfer_ring, acl_rx_transfer_ring and
 * acl_tx_free_transfer_ring -- all three arrive on the same bg_to_ap
 * doorbell (see the comment on struct scsc_mx_bt), so one work item
 * checks all three mailboxes.
 *
 * Event and ACL-RX draining are the same shape: an R4-owned write
 * index and an AP-owned read index we bump as each entry is consumed,
 * structurally the same as scsc_mx_hip_work()'s HIP4 queue drain.
 * Unlike hci_evt_transfer_ring, whose BSMHCP_TD_HCI_EVT entries are
 * already framed as raw HCI event bytes, acl_rx_transfer_ring's
 * BSMHCP_TD_ACL_RX carries the handle/PB/BC fields unpacked, so they
 * have to be reassembled into the 4-byte hci_acl_hdr hci_core expects
 * -- the RX mirror of what scsc_mx_bt_acl_transfer() unpacks on the
 * way out. A `disconnected` marker carries no payload; it exists only
 * to flush this ring past a torn-down connection and is dropped here.
 *
 * acl_tx_free_transfer_ring is different in kind: it is not something
 * to deliver, but the buffer-pool free list scsc_mx_bt_acl_transfer()
 * waits on, plus the trigger for a synthesised Number-Of-Completed-
 * Packets event. That event is the standard way hci_core knows it may
 * send more ACL data (hdev->acl_cnt), and this firmware never sends
 * one on its own -- confirmed against the downstream driver
 * (scsc_shm.c's h4_hci_credit_entries), which synthesises the same
 * event from the same ring for the same reason. Skipping this would
 * leave ACL TX working exactly once, for whatever credit hci_core
 * started with.
 */
static void scsc_mx_bt_work(struct work_struct *work)
{
	struct scsc_mx_bt *bt = container_of(work, struct scsc_mx_bt, work);
	struct scsc_mx *mx = container_of(bt, struct scsc_mx, bt);
	u32 write, read;

	if (!mx->bt.ready)
		return;

	write = readl(mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_hci_evt_write));
	read = readl(mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_hci_evt_read));
	/* write is the R4's; order these loads against its stores. */
	rmb();

	while (read != write) {
		u32 td = mx->bt.proto +
			 offsetof(struct BSMHCP_PROTOCOL, hci_evt_transfer_ring) +
			 read * sizeof(struct BSMHCP_TD_HCI_EVT);
		u8 data[BSMHCP_CMD_EVT_BUFFER_SIZE];
		u16 evlen = readw(mx->dram + td +
				  offsetof(struct BSMHCP_TD_HCI_EVT, length));

		if (evlen > sizeof(data))
			evlen = sizeof(data);
		memcpy_fromio(data, mx->dram + td +
			      offsetof(struct BSMHCP_TD_HCI_EVT, data), evlen);

		scsc_bt_recv(mx, SCSC_BT_PKT_EVENT, data, evlen);

		BSMHCP_INCREASE_INDEX(read, BSMHCP_TRANSFER_RING_EVT_SIZE);
		writel(read, mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_hci_evt_read));
		wmb();
	}

	write = readl(mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_acl_rx_write));
	read = readl(mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_acl_rx_read));
	rmb();

	while (read != write) {
		u32 td = mx->bt.proto +
			 offsetof(struct BSMHCP_PROTOCOL, acl_rx_transfer_ring) +
			 read * sizeof(struct BSMHCP_TD_ACL_RX);
		u16 handle = readw(mx->dram + td +
				   offsetof(struct BSMHCP_TD_ACL_RX, hci_connection_handle));
		u16 rxlen = readw(mx->dram + td +
				  offsetof(struct BSMHCP_TD_ACL_RX, length));
		u8 bc = readb(mx->dram + td +
			      offsetof(struct BSMHCP_TD_ACL_RX, broadcast_flag));
		u8 pb = readb(mx->dram + td +
			      offsetof(struct BSMHCP_TD_ACL_RX, packet_boundary));
		u8 disconnected = readb(mx->dram + td +
					offsetof(struct BSMHCP_TD_ACL_RX, disconnected));

		if (!disconnected) {
			/* Too big for the stack -- same reasoning as
			 * scsc_mx_hip_work()'s sig/data buffers. */
			u8 *pkt = kmalloc(SCSC_BT_ACL_HDR_LEN + BSMHCP_ACL_BUFFER_SIZE,
					  GFP_KERNEL);

			if (rxlen > BSMHCP_ACL_BUFFER_SIZE)
				rxlen = BSMHCP_ACL_BUFFER_SIZE;

			if (pkt) {
				pkt[0] = handle & 0xff;
				pkt[1] = ((handle >> 8) & 0x0f) |
					 ((pb & 0x03) << 4) | ((bc & 0x03) << 6);
				pkt[2] = rxlen & 0xff;
				pkt[3] = (rxlen >> 8) & 0xff;
				memcpy_fromio(pkt + SCSC_BT_ACL_HDR_LEN, mx->dram + td +
					      offsetof(struct BSMHCP_TD_ACL_RX, data), rxlen);
				scsc_bt_recv(mx, SCSC_BT_PKT_ACL, pkt,
					     SCSC_BT_ACL_HDR_LEN + rxlen);
				kfree(pkt);
			}
		}

		BSMHCP_INCREASE_INDEX(read, BSMHCP_TRANSFER_RING_ACL_SIZE);
		writel(read, mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_acl_rx_read));
		wmb();
	}

	write = readl(mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_acl_free_write));
	read = readl(mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_acl_free_read));
	rmb();

	if (read != write) {
		struct { u16 handle; u16 credits; } ncp[BSMHCP_TRANSFER_RING_ACL_SIZE] = { };
		unsigned int n = 0, i;

		while (read != write) {
			u32 td = mx->bt.proto +
				 offsetof(struct BSMHCP_PROTOCOL, acl_tx_free_transfer_ring) +
				 read * sizeof(struct BSMHCP_TD_ACL_TX_FREE);
			u8 idx = readb(mx->dram + td +
				       offsetof(struct BSMHCP_TD_ACL_TX_FREE, buffer_index));
			u16 handle = readw(mx->dram + td +
					   offsetof(struct BSMHCP_TD_ACL_TX_FREE, hci_connection_handle)) &
				     0x0fff;

			clear_bit(idx, &mx->bt.tx_buf_used);

			for (i = 0; i < n; i++)
				if (ncp[i].handle == handle) {
					ncp[i].credits++;
					break;
				}
			if (i == n && n < ARRAY_SIZE(ncp)) {
				ncp[n].handle = handle;
				ncp[n].credits = 1;
				n++;
			}

			BSMHCP_INCREASE_INDEX(read, BSMHCP_TRANSFER_RING_ACL_SIZE);
			writel(read, mx->dram + mx->bt.proto + BSMHCP_OFF(mailbox_acl_free_read));
			wmb();
		}

		if (n) {
			u8 evt[3 + 4 * BSMHCP_TRANSFER_RING_ACL_SIZE];

			evt[0] = 0x13; /* Number_Of_Completed_Packets */
			evt[1] = 1 + 4 * n;
			evt[2] = n;
			for (i = 0; i < n; i++) {
				put_unaligned_le16(ncp[i].handle, &evt[3 + 4 * i]);
				put_unaligned_le16(ncp[i].credits, &evt[3 + 4 * i + 2]);
			}
			scsc_bt_recv(mx, SCSC_BT_PKT_EVENT, evt, 3 + 4 * n);
		}
	}
}

/*
 * WLAN service start.
 *
 * Structurally the same move as BT -- build a configuration structure in
 * shared memory, hand its R4-relative address over in SM_MSG_START_REQ
 * -- but the structure is the HIP4 control block, and it is the
 * transport rather than a preamble to one: an init header, two
 * configuration structures (the firmware picks a version), a 256-entry
 * scoreboard and six queue descriptors, in 8 KB exactly.
 *
 * "Exactly" is not a figure of speech: sizeof(struct hip4_hip_control)
 * is 8192, which is IMG_MGR_SEC_WLAN_CONFIG_SIZE. The BSP sized the
 * region's first section to hold precisely this structure, which is a
 * useful check that the header was copied faithfully.
 *
 * Unlike BHCS there is no magic computed from the layout, so a field in
 * the wrong place is not caught early -- which is the argument for
 * having done BT first. What we do get is a versioned handshake:
 * init.conf_hip4_ver and the sap_*_ver fields are firmware-owned, so
 * after the confirmation the firmware has written back which config
 * version it chose and which SAP versions it speaks. Those are read
 * back below; they are the first thing this driver has ever learned
 * about the WLAN firmware's own interface.
 *
 * The host's mbulk pools must be formatted before the request. Leaving
 * them zeroed gets as far as the start and then panics in the
 * firmware's own allocator:
 *
 *	.../fw/wlan/mac/mbulk/mbulk_def.h:182:WLAN:
 *	PANIC_COMMON_MBULK_GENERIC(0x%x): 0x%x
 *
 * so the firmware does walk the descriptors the host lays down, exactly
 * as mbulk_pool_add() writes them.
 */
#define SCSC_WLAN_MIB_NAME	"mx140_wlan.hcf"
#define SCSC_WLAN_REGION_SIZE	(2 * 1024 * 1024)
/* The .hcf file header, stripped before the MIB reaches the firmware. */
#define SCSC_WLAN_MIB_HDR_LEN	8
/* vif 0 is the device itself, so a station interface starts at 1. */
#define SCSC_WLAN_TEST_VIF	1
/*
 * Scan identifiers are (interface << 8) | scan slot, and the vendor
 * never uses interface 0 for one: wlan0 is SLSI_NET_INDEX_WLAN = 1 and
 * the hardware scan slot is 0. A scan id of 0 is rejected with
 * FAPI_RESULTCODE_INVALID_PARAMETERS.
 */
#define SCSC_WLAN_SCAN_ID	((SCSC_WLAN_TEST_VIF << 8) | 0)

#define HIP4_INIT_MAGIC		0xcaaa0400
#define HIP4_CONFIG_MAGIC	0xcaba0401

/*
 * Both config structures have identical layouts and are filled
 * identically apart from the version; the firmware chooses one. A macro
 * rather than a helper because they are distinct types.
 */
#define scsc_mx_fill_hip_config(cfg, ver, region, mib_len, fh, th)	       \
do {									       \
	(cfg)->magic_number = HIP4_CONFIG_MAGIC;			       \
	(cfg)->hip_config_ver = (ver);					       \
	(cfg)->config_len = sizeof(*(cfg));				       \
	(cfg)->host_cache_line = 64;					       \
	(cfg)->host_buf_loc = (region) + IMG_MGR_SEC_WLAN_TX_OFFSET;	       \
	(cfg)->host_buf_sz = IMG_MGR_SEC_WLAN_TX_SIZE;			       \
	(cfg)->fw_buf_loc = (region) + IMG_MGR_SEC_WLAN_RX_OFFSET;	       \
	(cfg)->fw_buf_sz = IMG_MGR_SEC_WLAN_RX_SIZE;			       \
	(cfg)->mib_loc = (mib_len) ?					       \
		(region) + IMG_MGR_SEC_WLAN_MIB_OFFSET : 0;		       \
	(cfg)->mib_sz = (mib_len);					       \
	(cfg)->log_config_loc = 0;					       \
	(cfg)->mif_fh_int_n = (fh);					       \
	(cfg)->mif_th_int_n = (th);					       \
	(cfg)->scbrd_loc = (region) +					       \
		offsetof(struct hip4_hip_control, scoreboard);		       \
	(cfg)->q_num = MIF_HIP_CFG_Q_NUM;				       \
	(cfg)->q_len = 256;						       \
	(cfg)->q_idx_sz = 1;						       \
	for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++)				       \
		(cfg)->q_loc[i] = (region) +				       \
			offsetof(struct hip4_hip_control, q[i].array);	       \
} while (0)

/*
 * Carve an mbulk pool into free segments, as the vendor's
 * mbulk_pool_add() does.
 *
 * Only the segment descriptors live in shared memory; the pool
 * structure itself is host-side bookkeeping we do not need yet, since
 * nothing allocates from these pools until there is a data path. What
 * the firmware sees is a chain of MBULK_F_FREE descriptors, each
 * pointing at the previous one by *signed relative offset* -- so the
 * list runs downward through memory and the last segment is its head.
 *
 * Block size is round_up(size / slots, MBULK_ALIGN), which for both
 * pools here works out at exactly 2048 bytes and divides the regions
 * evenly: 218 data segments and 32 control ones.
 */
static void scsc_mx_mbulk_pool_format(struct scsc_mx *mx, u8 pool_id, u32 base,
				      u32 size, u32 slots)
{
	struct scsc_mx_mbulk_pool *pool = &mx->hip.pool[pool_id];
	u32 block = round_up(size / slots, MBULK_ALIGN);
	u32 off, prev = 0;
	unsigned int n = 0;

	for (off = base; off + block <= base + size; off += block) {
		struct mbulk m = {};

		m.pid = pool_id;
		m.flag = MBULK_F_FREE;
		m.next_offset = prev ? (s32)prev - (s32)off : 0;

		memcpy_toio(mx->dram + off, &m, sizeof(m));
		prev = off;
		n++;
	}

	pool->base = base;
	pool->size = size;
	pool->block = block;
	/* The last segment written is the head: the list runs downward. */
	pool->head = prev;
	pool->free = n;

	scsc_mx_vinfo(mx,
		      "wlan: mbulk pool %u at 0x%x+0x%x: %u segments of %u bytes\n",
		      pool_id, base, size, n, block);
}

/*
 * Take a segment off a pool's free list and dress it for use.
 *
 * The vendor's mbulk_pool_get() plus mbulk_seg_generic_alloc(): pop the
 * head, follow next_offset to the new head, then zero the descriptor and
 * fill in the sizes. head is where the bulk data starts relative to the
 * end of the descriptor, so it begins at the end of the signal buffer.
 *
 * Returns the R4-relative address of the segment, or 0.
 */
static u32 scsc_mx_mbulk_alloc(struct scsc_mx *mx, u8 pool_id,
			       enum mbulk_class clas, u16 sig_bufsz,
			       u16 dat_bufsz)
{
	struct scsc_mx_mbulk_pool *pool = &mx->hip.pool[pool_id];
	struct mbulk m = {};
	u32 ref = pool->head;
	s32 next;

	lockdep_assert_held(&mx->hip.lock);

	if (!ref)
		return 0;

	if (sizeof(m) + sig_bufsz + dat_bufsz > pool->block) {
		dev_warn(mx->dev,
			 "wlan: mbulk pool %u cannot hold %u+%u bytes (block %u)\n",
			 pool_id, sig_bufsz, dat_bufsz, pool->block);
		return 0;
	}

	next = (s32)readl(mx->dram + ref + offsetof(struct mbulk, next_offset));
	pool->head = next ? ref + next : 0;
	pool->free--;

	m.pid = pool_id;
	m.clas = clas;
	m.sig_bufsz = sig_bufsz;
	m.head = sig_bufsz;
	m.dat_bufsz = dat_bufsz;
	if (sig_bufsz)
		m.flag = MBULK_F_SIG;
	if (dat_bufsz)
		m.refcnt = 1;

	memcpy_toio(mx->dram + ref, &m, sizeof(m));

	return ref;
}

/* Push a segment back, head first, as mbulk_pool_put() does. */
static void scsc_mx_mbulk_free(struct scsc_mx *mx, u32 ref)
{
	struct scsc_mx_mbulk_pool *pool;
	struct mbulk m;
	u8 pool_id;

	lockdep_assert_held(&mx->hip.lock);

	memcpy_fromio(&m, mx->dram + ref, sizeof(m));
	if (m.flag == MBULK_F_FREE)
		return;

	/*
	 * The low bit of pid is the pool; the rest carries the "colour" a
	 * full driver stuffs in there (vif and peer index), which comes
	 * back untouched from the firmware. We always send colour 0, so
	 * this is only masking off something that should be zero -- but it
	 * is how the vendor recovers the pool, and a returned buffer going
	 * to the wrong free list would be a slow, confusing corruption.
	 */
	pool_id = m.pid & 0x1;
	pool = &mx->hip.pool[pool_id];

	m.flag = MBULK_F_FREE;
	m.next_offset = pool->head ? (s32)pool->head - (s32)ref : 0;
	memcpy_toio(mx->dram + ref, &m, sizeof(m));

	pool->head = ref;
	pool->free++;
}

/*
 * Is this a buffer reference we are willing to dereference?
 *
 * Everything on the queues is an R4-relative address the firmware chose,
 * and following one blindly is a read anywhere in the 4 MB window -- or
 * past it, which faults. Nothing on the wire says how big the reference
 * space is, so the WLAN region is the bound: both sides' buffers live in
 * it by construction.
 */
static bool scsc_mx_hip_ref_ok(struct scsc_mx *mx, u32 ref, u32 len,
			       const char *what)
{
	struct scsc_mx_hip *hip = &mx->hip;

	if (ref >= hip->region && ref + len > ref &&
	    ref + len <= hip->region + hip->region_size)
		return true;

	dev_warn(mx->dev, "wlan: %s reference 0x%x+%u outside 0x%x+0x%x\n",
		 what, ref, len, hip->region, hip->region_size);

	return false;
}

/*
 * Scoreboard access.
 *
 * Six queues, each with a read and a write index, one byte apiece, laid
 * out so that each side only ever writes its own indices (SC-505612-DD,
 * reproduced in the BSP's hip4.c). The from-host queues are written by
 * us and read by the firmware, so our index sits in the first block and
 * theirs 64 bytes further on; the to-host queues are the mirror image.
 * That 64 is the cache line: the two sides are not coherent, so an index
 * pair sharing a line would lose updates.
 */
#define SCSC_HIP_FW_OWN_OFS	64

enum scsc_hip_rw { SCSC_HIP_WIDX, SCSC_HIP_RIDX };

static const u8 scsc_hip_idx_layout[MIF_HIP_CFG_Q_NUM][2] = {
	{			  0, SCSC_HIP_FW_OWN_OFS + 0 }, /* fh ctrl */
	{			  1, SCSC_HIP_FW_OWN_OFS + 1 }, /* fh dat  */
	{ SCSC_HIP_FW_OWN_OFS + 2,			2 }, /* fh rfb  */
	{ SCSC_HIP_FW_OWN_OFS + 3,			3 }, /* th ctrl */
	{ SCSC_HIP_FW_OWN_OFS + 4,			4 }, /* th dat  */
	{			  5, SCSC_HIP_FW_OWN_OFS + 5 }, /* th rfb  */
};

static u8 scsc_mx_hip_idx_read(struct scsc_mx *mx, enum hip4_hip_q_conf q,
			       enum scsc_hip_rw rw)
{
	u8 v = readb(mx->dram + mx->hip.scbrd + scsc_hip_idx_layout[q][rw]);

	/* Whatever we read was written by the other core; order the loads. */
	rmb();

	return v;
}

static void scsc_mx_hip_idx_write(struct scsc_mx *mx, enum hip4_hip_q_conf q,
				  enum scsc_hip_rw rw, u8 value)
{
	writeb(value, mx->dram + mx->hip.scbrd + scsc_hip_idx_layout[q][rw]);
	wmb();
}

/*
 * Put a buffer reference on a queue and tell the firmware.
 *
 * The queues are 256-entry arrays of R4-relative addresses with byte
 * indices, so they wrap for free; one slot is left unused to keep full
 * distinguishable from empty.
 */
static int scsc_mx_hip_q_add(struct scsc_mx *mx, enum hip4_hip_q_conf q,
			     u32 ref)
{
	u8 widx = scsc_mx_hip_idx_read(mx, q, SCSC_HIP_WIDX);
	u8 ridx = scsc_mx_hip_idx_read(mx, q, SCSC_HIP_RIDX);

	lockdep_assert_held(&mx->hip.lock);

	if (ridx == (u8)(widx + 1))
		return -ENOSPC;

	writel(ref, mx->dram + mx->hip.ctl +
	       offsetof(struct hip4_hip_control, q[q].array) + widx * 4);
	/* The entry must be visible before the index that publishes it. */
	wmb();

	scsc_mx_hip_idx_write(mx, q, SCSC_HIP_WIDX, widx + 1);
	writel(BIT(mx->hip.fh_bit), mx->r4_regs + INTGR1);

	return 0;
}

/*
 * All six queue index pairs, plus how much of each from-host pool is
 * left.
 *
 * The one measurement that separates "the firmware is not taking our
 * frames" from "the firmware is taking them and answering nothing":
 * a from-host read index that follows the write index means the other
 * side is consuming, and a to-host write index that never moves means
 * it has nothing to say.
 */
static void scsc_mx_hip_dump_queues(struct scsc_mx *mx)
{
	static const char * const name[MIF_HIP_CFG_Q_NUM] = {
		"fh-ctl", "fh-dat", "fh-rfb", "th-ctl", "th-dat", "th-rfb"
	};
	char line[128];
	int n = 0;
	unsigned int q;

	lockdep_assert_held(&mx->hip.lock);

	for (q = 0; q < MIF_HIP_CFG_Q_NUM; q++)
		n += scnprintf(line + n, sizeof(line) - n, "%s %u/%u ",
			       name[q],
			       scsc_mx_hip_idx_read(mx, q, SCSC_HIP_RIDX),
			       scsc_mx_hip_idx_read(mx, q, SCSC_HIP_WIDX));

	dev_info_ratelimited(mx->dev, "wlan: q %s(pool dat %u ctl %u)\n", line,
			     mx->hip.pool[MBULK_CLASS_FROM_HOST_DAT].free,
			     mx->hip.pool[MBULK_CLASS_FROM_HOST_CTL].free);
}

/*
 * Send one signal, with optional bulk data after it.
 *
 * The signal goes into the segment's in-line signal buffer, four bytes
 * in: the firmware expects a preamble there which the host does not
 * write. (The vendor leaves those four bytes as whatever the previous
 * user of the segment left behind, so the firmware demonstrably ignores
 * them on this direction; we zero them because a reproducible capture is
 * worth more than the three instructions.)
 *
 * Bulk data does not start where the signal buffer ends. The firmware
 * asks for headroom and tailroom around it in the HIP config -- 184 and
 * 18 bytes on this build -- so that it can push an 802.11 header in
 * front and a trailer behind without copying. hip4_skb_to_mbulk() honours
 * that for every signal carrying data, control ones included, so the
 * data offset is head = sig_bufsz + headroom and the buffer is sized for
 * all three parts.
 */
#define SCSC_HIP_SIG_PREAMBLE	4

static int scsc_mx_hip_send(struct scsc_mx *mx, bool ctrl, const void *sig,
			    u16 siglen, const void *data, u16 datalen)
{
	enum mbulk_class clas = ctrl ? MBULK_CLASS_FROM_HOST_CTL
				     : MBULK_CLASS_FROM_HOST_DAT;
	u16 sig_bufsz = round_up(siglen + SCSC_HIP_SIG_PREAMBLE, 4);
	u16 head = sig_bufsz;
	u16 dat_bufsz = 0;
	u8 pool_id = clas;
	u32 seg, ref;
	int ret;

	if (!mx->hip.ready)
		return -ENODEV;

	if (datalen) {
		head += mx->hip.headroom;
		dat_bufsz = mx->hip.headroom + datalen + mx->hip.tailroom;
	}

	mutex_lock(&mx->hip.lock);

	ref = scsc_mx_mbulk_alloc(mx, pool_id, clas, sig_bufsz, dat_bufsz);
	if (!ref) {
		mutex_unlock(&mx->hip.lock);
		dev_warn(mx->dev, "wlan: no free mbulk in pool %u\n", pool_id);
		return -ENOBUFS;
	}

	seg = ref + sizeof(struct mbulk);
	memset_io(mx->dram + seg, 0, SCSC_HIP_SIG_PREAMBLE);
	memcpy_toio(mx->dram + seg + SCSC_HIP_SIG_PREAMBLE, sig, siglen);
	if (datalen) {
		memcpy_toio(mx->dram + seg + head, data, datalen);
		writew(head, mx->dram + ref + offsetof(struct mbulk, head));
	}

	writew(datalen, mx->dram + ref + offsetof(struct mbulk, len));
	/* MBULK_F_OBOUND: the other core owns this buffer from now on. */
	writeb(MBULK_F_SIG | MBULK_F_OBOUND,
	       mx->dram + ref + offsetof(struct mbulk, flag));

	ret = scsc_mx_hip_q_add(mx, ctrl ? HIP4_MIF_Q_FH_CTRL : HIP4_MIF_Q_FH_DAT,
				ref);
	if (ret)
		scsc_mx_mbulk_free(mx, ref);

	/*
	 * Record the stall while still holding the lock. Setting it after
	 * the unlock would race the bottom half: it could drain the return
	 * queue in between, decide nothing was waiting, and leave the flag
	 * set with no wakeup to come.
	 */
	if (ret && !ctrl)
		mx->hip.tx_stopped = true;

	mutex_unlock(&mx->hip.lock);

	if (ret) {
		dev_warn_ratelimited(mx->dev, "wlan: from-host %s queue full\n",
				     ctrl ? "control" : "data");
	} else if (ctrl) {
		scsc_mx_vinfo(mx,
			      "wlan: sent signal 0x%04x, %u bytes + %u bulk, mbulk 0x%x (sig_bufsz %u dat_bufsz %u head %u)\n",
			      get_unaligned_le16(sig), siglen, datalen, ref,
			      sig_bufsz, dat_bufsz, head);
	} else {
		/* One line per packet is not something the log can carry. */
		dev_dbg(mx->dev, "wlan: sent %u bytes of data, mbulk 0x%x\n",
			datalen, ref);
	}

	if (!ctrl && scsc_mx_hip_debug)
		scsc_mx_hip_dump_queues(mx);

	return ret;
}

/*
 * The information elements of an association request or response, copied
 * out of the management frame the firmware reports.
 *
 * This matters more than it looks. The firmware does not transmit the RSN
 * element it was handed: it rewrites the capabilities field -- 0x000c
 * from wpa_supplicant goes out as 0x242c. The supplicant computes message
 * 2 of the 4-way handshake from its own copy, the AP compares that
 * against what it received in the association request, and the handshake
 * is refused with "WPA IE from (Re)AssocReq did not match with msg 2/4".
 *
 * Reporting these to cfg80211 in the connect result is what stops that:
 * the supplicant adopts the elements the driver says were sent.
 *
 * Returns a pointer into the frame and the length in ie_len, or NULL.
 */
static const u8 *scsc_mx_assoc_ies(const u8 *frame, u16 len, u16 *ie_len)
{
	const struct ieee80211_mgmt *mgmt = (const void *)frame;
	size_t hdrlen;

	*ie_len = 0;

	if (len < offsetof(struct ieee80211_mgmt, u.assoc_req.variable))
		return NULL;

	if (ieee80211_is_assoc_req(mgmt->frame_control))
		hdrlen = offsetof(struct ieee80211_mgmt, u.assoc_req.variable);
	else if (ieee80211_is_reassoc_req(mgmt->frame_control))
		hdrlen = offsetof(struct ieee80211_mgmt,
				  u.reassoc_req.variable);
	else if (ieee80211_is_assoc_resp(mgmt->frame_control))
		hdrlen = offsetof(struct ieee80211_mgmt, u.assoc_resp.variable);
	else if (ieee80211_is_reassoc_resp(mgmt->frame_control))
		hdrlen = offsetof(struct ieee80211_mgmt,
				  u.reassoc_resp.variable);
	else
		return NULL;

	if (len <= hdrlen)
		return NULL;

	*ie_len = len - hdrlen;

	return frame + hdrlen;
}

/*
 * One scan result.
 *
 * The bulk data is the whole management frame the radio received --
 * beacon or probe response, header and all -- so this is ordinary
 * 802.11 parsing rather than anything SCSC-specific. Beacons and probe
 * responses share a body layout: eight bytes of timestamp, the beacon
 * interval, the capability field, then information elements.
 */
static void scsc_mx_scan_result(struct scsc_mx *mx, const u8 *sig, u16 siglen,
				const u8 *data, u16 datalen)
{
	const struct fapi_signal *s = (const void *)sig;
	const struct ieee80211_mgmt *mgmt = (const void *)data;
	const u8 *ie, *end;
	unsigned int freq;
	char ssid[IEEE80211_MAX_SSID_LEN + 1] = "";
	s16 rssi;

	mx->hip.scan_results++;

	if (siglen < fapi_sig_size(mlme_scan_ind))
		return;

	/* The firmware counts in half-megahertz. */
	freq = le16_to_cpu(s->u.mlme_scan_ind.channel_frequency) / 2;
	rssi = (s16)le16_to_cpu(s->u.mlme_scan_ind.rssi);

	if (datalen < offsetof(struct ieee80211_mgmt, u.beacon.variable)) {
		dev_info(mx->dev, "wlan: scan result, %u MHz %d dBm, runt frame\n",
			 freq, rssi);
		return;
	}

	ie = mgmt->u.beacon.variable;
	end = data + datalen;

	while (ie + 2 <= end && ie + 2 + ie[1] <= end) {
		if (ie[0] == WLAN_EID_SSID) {
			u8 len = min_t(u8, ie[1], IEEE80211_MAX_SSID_LEN);

			memcpy(ssid, ie + 2, len);
			ssid[len] = '\0';
			break;
		}
		ie += 2 + ie[1];
	}

	dev_dbg(mx->dev, "wlan: scan result %pM  %4u MHz  %4d dBm  \"%s\"\n",
		mgmt->bssid, freq, rssi, ssid);

	/*
	 * cfg80211 wants the frame, not our reading of it; the parse above
	 * is only so the log says something useful. Reporting the whole
	 * management frame is what lets it build a BSS entry with the
	 * capabilities and IEs we do not look at.
	 */
	scsc_wlan_scan_result(mx, freq, rssi, data, datalen);
}

/*
 * A signal arriving from the firmware, already copied out of shared
 * memory. Bring-up handling: satisfy a waiter if this is the confirm it
 * asked for, otherwise say what came in.
 */
static void scsc_mx_hip_deliver(struct scsc_mx *mx, const u8 *sig, u16 siglen,
				const u8 *data, u16 datalen)
{
	const struct fapi_signal_header *hdr = (const void *)sig;
	const struct fapi_vif_signal_header *vhdr = (const void *)sig;
	const u8 *ies;
	u16 id, ie_len, pid = 0, peer_index;

	if (siglen < sizeof(*hdr)) {
		dev_warn(mx->dev, "wlan: runt signal, %u bytes\n", siglen);
		return;
	}

	id = le16_to_cpu(hdr->id);
	pid = le16_to_cpu(hdr->receiver_pid);

	/*
	 * A confirm is matched on both the signal id and the process id
	 * we put in the request, which is what receiver_pid carries back.
	 */
	if (id && id == mx->hip.cfm_id && pid == mx->hip.pid) {
		scsc_mx_vinfo(mx,
			      "wlan: confirm 0x%04x for pid 0x%04x, %u bytes + %u bulk\n",
			      id, pid, siglen, datalen);
		mx->hip.cfm_siglen = min_t(u16, siglen, SCSC_HIP_SIG_MAX);
		mx->hip.cfm_datalen = min_t(u16, datalen, SCSC_HIP_DATA_MAX);
		memcpy(mx->hip.cfm_sig, sig, mx->hip.cfm_siglen);
		memcpy(mx->hip.cfm_data, data, mx->hip.cfm_datalen);
		mx->hip.cfm_id = 0;
		complete(&mx->hip.cfm);
		return;
	}

	/*
	 * Scan results are unsolicited and unbounded in number, so they
	 * are consumed here rather than woken on.
	 */
	if (id == MLME_SCAN_IND) {
		scsc_mx_scan_result(mx, sig, siglen, data, datalen);
		return;
	}

	if (id == MLME_SCAN_DONE_IND) {
		scsc_mx_vinfo(mx, "wlan: scan done, %u results\n",
			      mx->hip.scan_results);
		mx->hip.scan_done = true;
		return;
	}

	/*
	 * Received 802.3 frames, which on this driver means EAPOL: the
	 * 4-way handshake arrives this way even though it is sent on the
	 * control path. Anything else is passed up too and dropped by the
	 * stack, since nothing has configured an address on the
	 * interface. priority and sequence_number are what block-ack
	 * reordering needs -- see scsc_wlan_rx_unitdata().
	 */
	if (id == MA_UNITDATA_IND && siglen >= fapi_sig_size(ma_unitdata_ind)) {
		const struct fapi_signal *r = (const void *)sig;

		scsc_wlan_rx_unitdata(mx, le16_to_cpu(vhdr->vif),
				      le16_to_cpu(r->u.ma_unitdata_ind.peer_index),
				      le16_to_cpu(r->u.ma_unitdata_ind.priority),
				      le16_to_cpu(r->u.ma_unitdata_ind.sequence_number),
				      data, datalen);
		return;
	}

	/*
	 * MA-BLOCKACK.indication -- starts, ends or resets a host-side
	 * reorder session for one peer/TID; see scsc_wlan_blockack(). The
	 * transmit direction only reports capabilities (A-MSDU-in-A-MPDU)
	 * this driver's TX path does not use, so only receive is passed on.
	 */
	if (id == MA_BLOCKACK_IND && siglen >= fapi_sig_size(ma_blockack_ind)) {
		const struct fapi_signal *r = (const void *)sig;
		u16 parameter_set =
			le16_to_cpu(r->u.ma_blockack_ind.blockack_parameter_set);

		if (le16_to_cpu(r->u.ma_blockack_ind.direction) ==
		    FAPI_DIRECTION_RECEIVE)
			scsc_wlan_blockack(mx, le16_to_cpu(vhdr->vif),
					   r->u.ma_blockack_ind.peer_qsta_address,
					   (parameter_set >> 2) & 0x000F,
					   (parameter_set >> 6) & 0x03FF,
					   le16_to_cpu(r->u.ma_blockack_ind.sequence_number),
					   le16_to_cpu(r->u.ma_blockack_ind.reason_code));
		return;
	}

	/*
	 * The other way received data arrives, and the one that carries
	 * most of it: the firmware sends 802.3 frames up as MLME
	 * indications on the *control* queue, not only as MA-UNITDATA on
	 * the data queue. A DHCP offer comes back this way. The same
	 * signal carries received management frames, which is what
	 * data_unit_descriptor distinguishes.
	 */
	if (id == MLME_RECEIVED_FRAME_IND &&
	    siglen >= fapi_sig_size(mlme_received_frame_ind)) {
		const struct fapi_signal *r = (const void *)sig;
		u16 dud =
			le16_to_cpu(r->u.mlme_received_frame_ind.data_unit_descriptor);

		if (dud == FAPI_DATAUNITDESCRIPTOR_IEEE802_3_FRAME) {
			scsc_wlan_rx_data(mx, le16_to_cpu(vhdr->vif), data,
					  datalen);
			return;
		}

		/*
		 * An 802.11 management frame -- authentication frames (SAE)
		 * and action frames, mostly. channel_frequency is the
		 * firmware's usual half-megahertz encoding.
		 */
		scsc_wlan_rx_mgmt(mx, le16_to_cpu(vhdr->vif),
				  le16_to_cpu(r->u.mlme_received_frame_ind.channel_frequency) / 2,
				  data, datalen);
		return;
	}

	/*
	 * Transmit status, one per frame we sent. Buffers come back on the
	 * return queue regardless, so this is purely diagnostic -- worth a
	 * line when the radio could not place a frame, and silence when it
	 * could, because there is one of these per packet.
	 */
	if (id == MA_UNITDATA_CFM && siglen >= fapi_sig_size(ma_unitdata_cfm)) {
		const struct fapi_signal *c = (const void *)sig;
		u16 status = le16_to_cpu(c->u.ma_unitdata_cfm.transmission_status);

		if (status != FAPI_TRANSMISSIONSTATUS_SUCCESSFUL)
			dev_warn_ratelimited(mx->dev,
					     "wlan: tx failed, status %u, host tag %u\n",
					     status,
					     le16_to_cpu(c->u.ma_unitdata_cfm.host_tag));
		return;
	}

	/*
	 * Association progress. The firmware reports it in three steps:
	 * the procedure starting, the result of the exchange, and then
	 * the link being usable. cfg80211's notifiers queue an event
	 * under wdev->event_lock rather than taking the wiphy mutex, so
	 * unlike the scan they are safe to call from here.
	 */
	if (id == MLME_PROCEDURE_STARTED_IND &&
	    siglen >= fapi_sig_size(mlme_procedure_started_ind)) {
		const struct fapi_signal *p = (const void *)sig;
		u16 procedure_type =
			le16_to_cpu(p->u.mlme_procedure_started_ind.procedure_type);

		/*
		 * FAPI_PROCEDURETYPE_DEVICE_DISCOVERED is P2P-only, which
		 * this driver does not offer as an interface type, so it
		 * should never arrive; ignore rather than mishandle it as
		 * an association.
		 */
		if (procedure_type != FAPI_PROCEDURETYPE_CONNECTION_STARTED &&
		    procedure_type != FAPI_PROCEDURETYPE_ROAMING_STARTED) {
			dev_dbg(mx->dev,
				"wlan: unexpected procedure type 0x%04x on vif %u\n",
				procedure_type, le16_to_cpu(vhdr->vif));
			return;
		}

		/*
		 * The bulk data is the (re)association request as
		 * transmitted, for a fresh connect and a firmware-driven
		 * roam alike. Keep its elements either way: the supplicant
		 * needs to be told what actually went out, not what it
		 * asked for, and the result that reports them --
		 * MLME_CONNECT_IND or MLME_ROAMED_IND -- is a separate
		 * signal that carries no request elements of its own.
		 */
		ies = scsc_mx_assoc_ies(data, datalen, &ie_len);
		if (ies && ie_len > SCSC_ASSOC_IE_MAX) {
			dev_warn(mx->dev,
				 "wlan: %u bytes of association elements, keeping %u\n",
				 ie_len, SCSC_ASSOC_IE_MAX);
			ie_len = SCSC_ASSOC_IE_MAX;
		}
		mx->hip.assoc_req_ie_len = ies ? ie_len : 0;
		if (ies)
			memcpy(mx->hip.assoc_req_ie, ies, ie_len);

		/*
		 * peer_index looks like the association id to quote back in
		 * data frames, and is not: the firmware only fills it in for
		 * an AP vif, where it allocates one per joining station. On
		 * a station vif -- a fresh connect or a roam alike -- it
		 * reads 0, which is not a peer.
		 */
		peer_index = le16_to_cpu(p->u.mlme_procedure_started_ind.peer_index);

		scsc_mx_vinfo(mx,
			      "wlan: %s procedure started on vif %u, peer index %u\n",
			      procedure_type == FAPI_PROCEDURETYPE_ROAMING_STARTED ?
				      "roaming" : "association",
			      le16_to_cpu(vhdr->vif), peer_index);

		/*
		 * ies points into the association request frame, which
		 * starts with an ordinary 802.11 header -- sa is the joining
		 * station's address, the only place it appears in this
		 * signal. ies/ie_len is the same association elements just
		 * saved above, including whatever RSN element hostapd needs
		 * to see to start the handshake. Only a fresh connect on an
		 * AP vif has a peer to add here; a roam is station-only and
		 * reported later, on MLME_ROAMED_IND, from the elements just
		 * saved above.
		 */
		if (procedure_type == FAPI_PROCEDURETYPE_CONNECTION_STARTED &&
		    peer_index && ies)
			scsc_wlan_peer_started(mx, le16_to_cpu(vhdr->vif),
					       peer_index,
					       ((const struct ieee80211_mgmt *)data)->sa,
					       ies, ie_len);
		return;
	}

	if (id == MLME_CONNECT_IND && siglen >= fapi_sig_size(mlme_connect_ind)) {
		const struct fapi_signal *c = (const void *)sig;
		u16 result = le16_to_cpu(c->u.mlme_connect_ind.result_code);
		const u8 *resp_ie;
		u16 resp_ie_len;

		/* The bulk data here is the association response. */
		resp_ie = scsc_mx_assoc_ies(data, datalen, &resp_ie_len);

		dev_info(mx->dev, "wlan: connect result 0x%04x (%s) on vif %u\n",
			 result, scsc_mx_result_name(result),
			 le16_to_cpu(vhdr->vif));
		scsc_wlan_connect_result(mx, le16_to_cpu(vhdr->vif),
					 result == FAPI_RESULTCODE_SUCCESS,
					 mx->hip.assoc_req_ie,
					 mx->hip.assoc_req_ie_len,
					 resp_ie, resp_ie_len);
		return;
	}

	/*
	 * The firmware roamed a station to a new BSS on its own -- no
	 * MLME-CONNECT.request from the host, and no MLME_CONNECT_IND
	 * either. This is that connect's counterpart for the roam case;
	 * the bulk data is the (re)association response, same as
	 * MLME_CONNECT_IND's.
	 */
	if (id == MLME_ROAMED_IND && siglen >= fapi_sig_size(mlme_roamed_ind)) {
		const struct fapi_signal *r = (const void *)sig;
		u16 temporal_keys_required =
			le16_to_cpu(r->u.mlme_roamed_ind.temporal_keys_required);
		const u8 *resp_ie;
		u16 resp_ie_len;

		resp_ie = scsc_mx_assoc_ies(data, datalen, &resp_ie_len);
		if (!resp_ie) {
			dev_warn(mx->dev,
				 "wlan: roamed-ind on vif %u, runt response frame\n",
				 le16_to_cpu(vhdr->vif));
			return;
		}

		dev_info(mx->dev, "wlan: roamed on vif %u%s\n",
			 le16_to_cpu(vhdr->vif),
			 temporal_keys_required ? ", new keys required" : "");
		scsc_wlan_roamed(mx, le16_to_cpu(vhdr->vif),
				 temporal_keys_required,
				 ((const struct ieee80211_mgmt *)data)->bssid,
				 mx->hip.assoc_req_ie, mx->hip.assoc_req_ie_len,
				 resp_ie, resp_ie_len);
		return;
	}

	/*
	 * The firmware's own roam scan reporting a result. Nothing here
	 * needs to act on it -- the outcome is what MLME_ROAMED_IND (or,
	 * on failure, simply staying put) reports -- but it is worth a
	 * line, since a station that keeps roam-scanning without ever
	 * roaming is a sign the candidate APs are not actually better.
	 */
	if (id == MLME_ROAM_IND && siglen >= fapi_sig_size(mlme_roam_ind)) {
		const struct fapi_signal *r = (const void *)sig;

		scsc_mx_vinfo(mx, "wlan: roam scan result 0x%04x on vif %u\n",
			      le16_to_cpu(r->u.mlme_roam_ind.result_code),
			      le16_to_cpu(vhdr->vif));
		return;
	}

	/*
	 * A channel move, either an AP vif under radar or a station
	 * following its AP's own CSA. channel_information packs the
	 * bandwidth in the low byte and, for 40/80 MHz, the primary
	 * channel's position within it in the high byte -- the same
	 * decoding the vendor driver does, offset by 10 MHz per position
	 * below the reported centre for 40 MHz and 30 MHz for 80 MHz.
	 * cfg80211_ch_switch_notify() wants the wiphy mutex, which the
	 * transport lock is not, so this is captured and sent from
	 * scsc_mx_hip_work() once the lock is dropped -- same reason as
	 * scan_done above.
	 */
	if (id == MLME_CHANNEL_SWITCHED_IND &&
	    siglen >= fapi_sig_size(mlme_channel_switched_ind)) {
		const struct fapi_signal *c = (const void *)sig;
		u16 vif = le16_to_cpu(vhdr->vif);
		u16 info = le16_to_cpu(c->u.mlme_channel_switched_ind.channel_information);
		u32 cf1_mhz = le16_to_cpu(c->u.mlme_channel_switched_ind.channel_frequency) / 2;
		u8 primary_pos = info >> 8;
		u8 width_mhz = info & 0xff;
		u32 freq_mhz;

		if (width_mhz == 40)
			freq_mhz = cf1_mhz + primary_pos * 20 - 10;
		else if (width_mhz == 80)
			freq_mhz = cf1_mhz + primary_pos * 20 - 30;
		else
			freq_mhz = cf1_mhz;

		dev_info(mx->dev, "wlan: channel switch on vif %u to %u MHz, %u MHz wide\n",
			 vif, freq_mhz, width_mhz);

		if (vif >= 1 && vif <= SCSC_WLAN_MAX_VIFS) {
			mx->hip.chsw[vif].freq_mhz = freq_mhz;
			mx->hip.chsw[vif].center_freq1_mhz = cf1_mhz;
			mx->hip.chsw[vif].width_mhz = width_mhz;
			mx->hip.chsw[vif].pending = true;
			schedule_work(&mx->hip.work);
		}
		return;
	}

	/*
	 * The firmware wants a WPA3-SAE authentication run by the host --
	 * see scsc_mx_mlme_connect()'s sae parameter. The bulk data is the
	 * frame the firmware is proposing to authenticate against, same
	 * shape as MLME_CONNECT_IND's; only the bssid is needed here.
	 */
	if (id == MLME_SYNCHRONISED_IND &&
	    siglen >= fapi_sig_size(mlme_synchronised_ind) &&
	    datalen >= offsetof(struct ieee80211_mgmt, u)) {
		scsc_mx_vinfo(mx, "wlan: host authentication needed on vif %u, bssid %pM\n",
			      le16_to_cpu(vhdr->vif),
			      ((const struct ieee80211_mgmt *)data)->bssid);
		scsc_wlan_synchronised(mx, le16_to_cpu(vhdr->vif),
				      ((const struct ieee80211_mgmt *)data)->bssid);
		return;
	}

	/*
	 * The outcome of a frame sent through
	 * scsc_mx_mlme_send_mgmt_frame() -- cfg80211's .mgmt_tx wants to
	 * know whether it was acknowledged.
	 */
	if (id == MLME_FRAME_TRANSMISSION_IND &&
	    siglen >= fapi_sig_size(mlme_frame_transmission_ind)) {
		const struct fapi_signal *f = (const void *)sig;
		u16 host_tag = le16_to_cpu(f->u.mlme_frame_transmission_ind.host_tag);
		u16 status =
			le16_to_cpu(f->u.mlme_frame_transmission_ind.transmission_status);

		scsc_wlan_frame_transmission(mx, le16_to_cpu(vhdr->vif), host_tag,
					     status == FAPI_TRANSMISSIONSTATUS_SUCCESSFUL);
		return;
	}

	if (id == MLME_CONNECTED_IND &&
	    siglen >= fapi_sig_size(mlme_connected_ind)) {
		const struct fapi_signal *c = (const void *)sig;

		/*
		 * 0 on a station vif -- there the link coming up is the
		 * connect result above, not this indication -- and the
		 * joining peer's id on an AP vif.
		 */
		peer_index = le16_to_cpu(c->u.mlme_connected_ind.peer_index);

		dev_info(mx->dev, "wlan: connected on vif %u, peer index %u\n",
			 le16_to_cpu(vhdr->vif), peer_index);
		scsc_wlan_connected(mx, le16_to_cpu(vhdr->vif), peer_index);
		return;
	}

	if (id == MLME_DISCONNECTED_IND &&
	    siglen >= fapi_sig_size(mlme_disconnected_ind)) {
		const struct fapi_signal *d = (const void *)sig;
		u16 reason = le16_to_cpu(d->u.mlme_disconnected_ind.reason_code);

		dev_info(mx->dev, "wlan: disconnected on vif %u, reason %u\n",
			 le16_to_cpu(vhdr->vif), reason);
		scsc_wlan_disconnected(mx, le16_to_cpu(vhdr->vif),
				      d->u.mlme_disconnected_ind.peer_sta_address,
				      reason);
		return;
	}

	/*
	 * The DEBUG SAP is the firmware's own trace, and it is not
	 * sparing: a scan produces two DEBUG_WORD12 indications per
	 * result, which buries everything else. Faults are worth a line,
	 * rate-limited because they arrive in bursts; the rest is only
	 * interesting with dynamic debug turned on.
	 */
	if (id == DEBUG_WORD12IND) {
		const struct fapi_signal *d = (const void *)sig;

		/*
		 * Firmware trace. Ordinarily there is a great deal of it --
		 * two per scan result -- so it stays at debug level unless
		 * it was asked for, in which case it is the whole point.
		 */
		if (scsc_mx_fw_debug &&
		    siglen >= fapi_sig_size(debug_word12_ind)) {
			u16 w[12];
			unsigned int i;

			for (i = 0; i < 12; i++)
				w[i] = le16_to_cpu(d->u.debug_word12_ind.debug_words[i]);

			dev_info(mx->dev,
				 "wlan: fw dbg mod %u.%u vif %u %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
				 le16_to_cpu(d->u.debug_word12_ind.module_id),
				 le16_to_cpu(d->u.debug_word12_ind.module_sub_id),
				 le16_to_cpu(vhdr->vif),
				 w[0], w[1], w[2], w[3], w[4], w[5],
				 w[6], w[7], w[8], w[9], w[10], w[11]);
		} else {
			dev_dbg(mx->dev, "wlan: fw trace, %u bytes\n", datalen);
		}
		return;
	}

	if (id == DEBUG_FAULT_IND && siglen >= fapi_sig_size(debug_fault_ind)) {
		const struct fapi_signal *f = (const void *)sig;
		u16 cpu = le16_to_cpu(f->u.debug_fault_ind.cpu);

		dev_warn_ratelimited(mx->dev,
				     "wlan: firmware fault on %s: id 0x%04x arg 0x%08x count %u\n",
				     cpu == 0x8000 ? "MAC" :
				     cpu == 0x4000 ? "PHY" : "?",
				     le16_to_cpu(f->u.debug_fault_ind.faultid),
				     le32_to_cpu(f->u.debug_fault_ind.arg),
				     le16_to_cpu(f->u.debug_fault_ind.count));
		return;
	}

	dev_info(mx->dev, "wlan: signal 0x%04x for pid 0x%04x, %u bytes + %u bulk\n",
		 id, pid, siglen, datalen);

	if (!scsc_mx_verbose)
		return;
	print_hex_dump(KERN_INFO, "scsc_mx: wlan sig: ", DUMP_PREFIX_OFFSET,
		       16, 1, sig, siglen, false);
	if (datalen)
		print_hex_dump(KERN_INFO, "scsc_mx: wlan dat: ",
			       DUMP_PREFIX_OFFSET, 16, 1, data,
			       min_t(u16, datalen, 128), false);
}

/*
 * Consume one to-host queue.
 *
 * Each entry references an mbulk the firmware filled: a signal in the
 * in-line buffer (past the same four-byte preamble, which the firmware
 * *does* write on this direction) and optionally bulk data, possibly
 * spread over a chain of further segments. Every segment has to go back
 * on the to-host free-buffer queue afterwards or the firmware runs out.
 */
static void scsc_mx_hip_drain_q(struct scsc_mx *mx, enum hip4_hip_q_conf q,
				u8 *sig, u8 *data)
{
	u8 ridx = scsc_mx_hip_idx_read(mx, q, SCSC_HIP_RIDX);
	u8 widx = scsc_mx_hip_idx_read(mx, q, SCSC_HIP_WIDX);
	unsigned int seen = 0;


	for (; ridx != widx; ridx++, seen++) {
		u32 ref = readl(mx->dram + mx->hip.ctl +
				offsetof(struct hip4_hip_control, q[q].array) +
				ridx * 4);
		u32 chain[MBULK_MAX_CHAIN + 1];
		unsigned int n = 0, i;
		u16 siglen = 0, datalen = 0;
		struct mbulk m;

		if (!scsc_mx_hip_ref_ok(mx, ref, sizeof(m), "queued"))
			continue;

		chain[n++] = ref;
		memcpy_fromio(&m, mx->dram + ref, sizeof(m));

		if (m.sig_bufsz > SCSC_HIP_SIG_PREAMBLE) {
			siglen = min_t(u16, m.sig_bufsz - SCSC_HIP_SIG_PREAMBLE,
				       SCSC_HIP_SIG_MAX);
			memcpy_fromio(sig, mx->dram + ref + sizeof(m) +
				      SCSC_HIP_SIG_PREAMBLE, siglen);
		}

		/*
		 * Walk the chain, gathering the payload.
		 *
		 * chain_next_offset is an *absolute* reference despite the
		 * name, while next_offset four bytes above it is relative to
		 * the segment holding it. Nothing says so: the two are only
		 * told apart by what the vendor does with them --
		 * mbulk_pool_get() adds next_offset to the current segment,
		 * hip4_mbulk_to_skb() hands mbulk_chain_next() straight to
		 * the ref-to-pointer conversion. Reading this one as a delta
		 * lands outside the region, which is what oopsed the bottom
		 * half before the bounds check went in.
		 *
		 * The entry condition needs both chain bits and the
		 * continuation only one, which is not symmetry lost in the
		 * copying -- it is what hip4_mbulk_to_skb() does.
		 */
		while (1) {
			u16 take = min_t(u16, m.len, SCSC_HIP_DATA_MAX - datalen);
			u8 need = n == 1 ? (MBULK_F_CHAIN_HEAD | MBULK_F_CHAIN)
					 : MBULK_F_CHAIN;
			u32 next = m.chain_next_offset;

			if (take)
				memcpy_fromio(data + datalen,
					      mx->dram + chain[n - 1] +
					      sizeof(m) + m.head, take);
			datalen += take;

			if ((m.flag & need) != need || !next ||
			    n > MBULK_MAX_CHAIN)
				break;

			if (!scsc_mx_hip_ref_ok(mx, next, sizeof(m), "chained"))
				break;

			chain[n] = next;
			memcpy_fromio(&m, mx->dram + next, sizeof(m));
			n++;
		}

		scsc_mx_hip_deliver(mx, sig, siglen, data, datalen);

		/* Hand every segment of the chain back to the firmware. */
		for (i = 0; i < n; i++)
			if (scsc_mx_hip_q_add(mx, HIP4_MIF_Q_TH_RFB, chain[i]))
				dev_warn(mx->dev,
					 "wlan: to-host free queue full, leaking mbulk 0x%x\n",
					 chain[i]);
	}

	if (seen)
		scsc_mx_hip_idx_write(mx, q, SCSC_HIP_RIDX, ridx);
}

/* Defined with the other MLME signals, which are built on the transport. */
static void scsc_mx_mlme_connect_resp_send(struct scsc_mx *mx, u16 vif);
static void scsc_mx_mlme_connected_resp_send(struct scsc_mx *mx, u16 vif,
					     u16 peer_index);
static void scsc_mx_mlme_roamed_resp_send(struct scsc_mx *mx, u16 vif);

static void scsc_mx_hip_work(struct work_struct *work)
{
	struct scsc_mx_hip *hip = container_of(work, struct scsc_mx_hip, work);
	struct scsc_mx *mx = container_of(hip, struct scsc_mx, hip);
	u8 ridx, widx;
	u8 *sig, *data;
	u16 vif;

	/* Too big for the stack, and the bottom half is not re-entered. */
	sig = kmalloc(SCSC_HIP_SIG_MAX, GFP_KERNEL);
	data = kmalloc(SCSC_HIP_DATA_MAX, GFP_KERNEL);
	if (!sig || !data)
		goto out;

	mutex_lock(&hip->lock);

	/*
	 * From-host return: buffers we sent, now finished with. Free them
	 * before reading the to-host queues, so a burst of receives has
	 * somewhere to allocate replies from.
	 */
	ridx = scsc_mx_hip_idx_read(mx, HIP4_MIF_Q_FH_RFB, SCSC_HIP_RIDX);
	widx = scsc_mx_hip_idx_read(mx, HIP4_MIF_Q_FH_RFB, SCSC_HIP_WIDX);
	if (ridx != widx) {
		for (; ridx != widx; ridx++) {
			u32 ref = readl(mx->dram + hip->ctl +
					offsetof(struct hip4_hip_control,
						 q[HIP4_MIF_Q_FH_RFB].array) +
					ridx * 4);

			if (scsc_mx_hip_ref_ok(mx, ref, sizeof(struct mbulk),
					       "returned"))
				scsc_mx_mbulk_free(mx, ref);
		}
		scsc_mx_hip_idx_write(mx, HIP4_MIF_Q_FH_RFB, SCSC_HIP_RIDX, ridx);

		/* There is room again; let the transmit path find out. */
		if (hip->tx_stopped) {
			hip->tx_stopped = false;
			hip->tx_resume = true;
		}
	}

	scsc_mx_hip_drain_q(mx, HIP4_MIF_Q_TH_CTRL, sig, data);
	scsc_mx_hip_drain_q(mx, HIP4_MIF_Q_TH_DAT, sig, data);

	mutex_unlock(&hip->lock);

	/*
	 * Deferred until the transport lock is dropped, because
	 * cfg80211_scan_done() takes the wiphy mutex and there is no
	 * reason to establish an ordering between the two.
	 */
	if (hip->scan_done) {
		hip->scan_done = false;
		scsc_wlan_scan_done(mx, false);
	}

	/* Also deferred: waking a netdev queue re-enters ndo_start_xmit. */
	if (hip->tx_resume) {
		hip->tx_resume = false;
		scsc_wlan_tx_resume(mx);
	}

	/*
	 * And the association acknowledgement, which was decided while the
	 * lock was held and needs to take it again to send.
	 */
	for (vif = 1; vif <= SCSC_WLAN_MAX_VIFS; vif++) {
		unsigned long pending;
		u16 peer_index;

		if (test_and_clear_bit(vif, &hip->connect_resp_pending))
			scsc_mx_mlme_connect_resp_send(mx, vif);

		pending = xchg(&hip->connected_resp_pending[vif], 0);
		for_each_set_bit(peer_index, &pending, BITS_PER_LONG)
			scsc_mx_mlme_connected_resp_send(mx, vif, peer_index);

		if (test_and_clear_bit(vif, &hip->roamed_resp_pending))
			scsc_mx_mlme_roamed_resp_send(mx, vif);

		/*
		 * The channel-switch notification, which needs the wiphy
		 * mutex that this work item -- unlike the bottom half -- is
		 * free to take.
		 */
		if (hip->chsw[vif].pending) {
			hip->chsw[vif].pending = false;
			scsc_wlan_channel_switched(mx, vif,
						   hip->chsw[vif].freq_mhz,
						   hip->chsw[vif].center_freq1_mhz,
						   hip->chsw[vif].width_mhz);
		}
	}
out:
	kfree(sig);
	kfree(data);
}

/*
 * Send a request and wait for its confirm.
 *
 * Nothing here is re-entrant: one request at a time, from the probe
 * path, which is all this driver does.
 */
static int scsc_mx_hip_request(struct scsc_mx *mx, u16 cfm_id, void *sig,
			       u16 siglen, const void *data, u16 datalen)
{
	struct fapi_signal_header *hdr = sig;
	int ret;

	if (siglen < sizeof(*hdr))
		return -EINVAL;

	/*
	 * Name ourselves as the sender: the confirm comes back addressed
	 * to this, and the firmware has no process zero to address it to.
	 */
	if (++mx->hip.pid > FAPI_PROCESS_ID_MAX)
		mx->hip.pid = FAPI_PROCESS_ID_MIN;
	hdr->sender_pid = cpu_to_le16(mx->hip.pid);

	reinit_completion(&mx->hip.cfm);
	mx->hip.cfm_id = cfm_id;

	ret = scsc_mx_hip_send(mx, true, sig, siglen, data, datalen);
	if (ret) {
		mx->hip.cfm_id = 0;
		return ret;
	}

	if (!wait_for_completion_timeout(&mx->hip.cfm,
					 msecs_to_jiffies(SCSC_HIP_CFM_TIMEOUT_MS))) {
		mx->hip.cfm_id = 0;
		dev_err(mx->dev, "wlan: no 0x%04x within %u ms\n", cfm_id,
			SCSC_HIP_CFM_TIMEOUT_MS);
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Start a signal. Every signal this driver sends carries a virtual
 * interface index, which is why the vendor's allocator writes it
 * through the vif header rather than through a per-signal member: it is
 * at the same offset in all of them.
 */
static void scsc_mx_fapi_init(struct fapi_signal *sig, u16 id, u16 vif)
{
	struct fapi_vif_signal_header *hdr = (struct fapi_vif_signal_header *)sig;

	memset(sig, 0, sizeof(*sig));
	hdr->id = cpu_to_le16(id);
	hdr->vif = cpu_to_le16(vif);
}

/*
 * The handful of FAPI result codes worth naming. There are more, but a
 * bare 0x800c in the log is the kind of thing that costs ten minutes
 * every time it appears, and these cover what this driver provokes.
 */
static const char *scsc_mx_result_name(u16 code)
{
	switch (code) {
	case FAPI_RESULTCODE_SUCCESS:			return "success";
	case FAPI_RESULTCODE_UNSPECIFIED_FAILURE:	return "unspecified failure";
	case FAPI_RESULTCODE_INVALID_PARAMETERS:	return "invalid parameters";
	case FAPI_RESULTCODE_TIMEOUT:			return "timeout";
	case FAPI_RESULTCODE_NOT_SUPPORTED:		return "not supported";
	case FAPI_RESULTCODE_INVALID_STATE:		return "invalid state";
	case FAPI_RESULTCODE_INSUFFICIENT_RESOURCE:	return "insufficient resource";
	case FAPI_RESULTCODE_MISSING_INFORMATION_ELEMENT: return "missing information element";
	case FAPI_RESULTCODE_INVALID_VIRTUAL_INTERFACE_INDEX: return "invalid vif";
	case FAPI_RESULTCODE_HOST_REQUEST_SUCCESS:	return "host request success";
	case FAPI_RESULTCODE_HOST_REQUEST_FAILED:	return "host request failed";
	case FAPI_RESULTCODE_INVALID_FREQUENCY:		return "invalid frequency";
	case FAPI_RESULTCODE_PROBE_TIMEOUT:		return "probe timeout";
	default:					return "?";
	}
}

/* The result code every confirm carries, at the same offset in each. */
static u16 scsc_mx_cfm_result(struct scsc_mx *mx)
{
	const struct fapi_signal *cfm = (const void *)mx->hip.cfm_sig;

	if (mx->hip.cfm_siglen < fapi_sig_size(mlme_add_vif_cfm))
		return 0xffff;

	return le16_to_cpu(cfm->u.mlme_add_vif_cfm.result_code);
}

/*
 * MIB values are encoded as a sequence of entries:
 *
 *	psid (le16) | length (le16) | value bytes | pad to even
 *
 * where the value bytes hold one or more variable-length data items. A
 * byte below 0x80 is a small unsigned value in itself; otherwise its low
 * five bits are a byte count and the flags say how to read what follows
 * -- 0x40 signed, 0x20 an octet string whose length is itself encoded in
 * those bytes, neither means a big-endian unsigned.
 *
 * An entry may carry index values before the value proper, so it is the
 * *last* item that is the value; that is how the .hcf files express
 * per-band and per-chain settings, and how mib.c's slsi_mib_decode()
 * reads them back. This is that function, minus the parts that build
 * the vendor's slsi_mib_entry.
 *
 * Returns the number of bytes the entry occupies, 0 if it is malformed.
 */
#define SCSC_MIB_MORE_MASK	0x80
#define SCSC_MIB_SIGN_MASK	0x40
#define SCSC_MIB_TYPE_MASK	0x20
#define SCSC_MIB_LENGTH_MASK	0x1f

struct scsc_mx_mib_value {
	u16 psid;
	bool octet;
	u32 value;		/* unsigned or signed, when !octet */
	const u8 *data;		/* the octet string, when octet */
	size_t datalen;
};

static size_t scsc_mx_mib_decode(const u8 *buf, size_t len,
				 struct scsc_mx_mib_value *out)
{
	size_t vlen, off = 4;

	if (len < 4)
		return 0;

	memset(out, 0, sizeof(*out));
	out->psid = get_unaligned_le16(buf);
	vlen = get_unaligned_le16(buf + 2);
	if (4 + vlen > len)
		return 0;

	while (off < 4 + vlen) {
		size_t n, i;

		out->octet = false;

		if (!(buf[off] & SCSC_MIB_MORE_MASK)) {
			out->value = buf[off] & 0x7f;
			off++;
			continue;
		}

		n = buf[off] & SCSC_MIB_LENGTH_MASK;
		if (off + 1 + n > 4 + vlen)
			return 0;

		if (buf[off] & SCSC_MIB_TYPE_MASK &&
		    !(buf[off] & SCSC_MIB_SIGN_MASK)) {
			size_t datalen = 0;

			for (i = 0; i < n; i++)
				datalen = (datalen << 8) | buf[off + 1 + i];
			if (off + 1 + n + datalen > 4 + vlen)
				return 0;

			out->octet = true;
			out->data = buf + off + 1 + n;
			out->datalen = datalen;
			off += 1 + n + datalen;
			continue;
		}

		for (i = 0, out->value = 0; i < n; i++)
			out->value = (out->value << 8) | buf[off + 1 + i];
		/*
		 * Sign-extend a negative value to the full 32 bits, so a
		 * caller expecting one (RSSI, in dBm) can just cast out->value
		 * to a signed type of its own choosing. Every PSID this driver
		 * has read until now has been unsigned, so this path was
		 * unexercised -- without it, e.g. a one-byte -60 (0xc4) would
		 * decode as 196 rather than -60.
		 */
		if (buf[off] & SCSC_MIB_SIGN_MASK && n &&
		    out->value & (1U << (n * 8 - 1)))
			out->value |= ~0U << (n * 8);
		off += 1 + n;
	}

	/* Entries are padded to an even length; the pad is not counted. */
	return 4 + vlen + (vlen & 1);
}

/*
 * The first thing this driver has ever asked the WLAN firmware.
 *
 * MLME-GET.request carries a list of PSIDs encoded as psid | 0 | 0 (no
 * index bytes, so four bytes each) and the confirm brings back the same
 * PSIDs with their values. Nothing is configured by this and no virtual
 * interface exists yet: it is a round trip that proves the transport,
 * and it returns identification the firmware has never told us before.
 */
struct scsc_mx_mib_name {
	u16 psid;
	const char *name;
};

/*
 * The BSP drop has no PSID table -- mgt.c names constants that no header
 * in it defines -- so these numbers come from the generated mib.h in the
 * TizenRT tree, which carries the same MIB for a different product.
 *
 * That numbering is confirmed rather than assumed: 0x07fd appears in
 * mx140_wlan.hcf set to 1, TizenRT calls it unifiVHTActivated, and the
 * firmware returns 1 for it here. A wrong table would have to be wrong
 * in the same way in three places to survive that.
 */
#define SCSC_PSID_SUPPORTED_CHANNELS	0x1f4c
#define SCSC_PSID_HT_ACTIVATED		0x07fe
#define SCSC_PSID_VHT_ACTIVATED	0x07fd
/*
 * The raw HT/VHT Capabilities element body the firmware would put on
 * air -- 21 and 12 bytes respectively, both confirmed against the same
 * TizenRT mib.h as the rest of this table (UnifiHtCapabilities /
 * UnifiVhtCapabilities, PSID 0x07f0 / 0x09ec).
 */
#define SCSC_PSID_HT_CAPABILITIES	0x07f0
#define SCSC_PSID_VHT_CAPABILITIES	0x09ec

static const struct scsc_mx_mib_name scsc_mx_mib_probe[] = {
	{ 0x07e6, "chip version" },
	{ 0x07e5, "firmware build id" },
	{ SCSC_PSID_HT_ACTIVATED, "HT activated" },
	{ SCSC_PSID_VHT_ACTIVATED, "VHT activated" },
	{ SCSC_PSID_HT_CAPABILITIES, "HT capabilities" },
	{ SCSC_PSID_VHT_CAPABILITIES, "VHT capabilities" },
	{ SCSC_PSID_SUPPORTED_CHANNELS, "supported channels" },
	/*
	 * The regulatory state the firmware believes it is in. A world
	 * domain ("00") here would explain a chip that receives happily
	 * and refuses to transmit, which is what the association failure
	 * looks like.
	 */
	{ 0x1f4d, "default country" },
	{ 0x1f4b, "regulatory params" },
};

static int scsc_mx_wlan_probe_mib(struct scsc_mx *mx)
{
	u8 mib[ARRAY_SIZE(scsc_mx_mib_probe) * 4] = {};
	struct fapi_signal req;
	size_t off = 0;
	unsigned int i;
	int ret;

	scsc_mx_fapi_init(&req, MLME_GET_REQ, 0);

	/* psid | length 0: no index bytes, so four bytes per request. */
	for (i = 0; i < ARRAY_SIZE(scsc_mx_mib_probe); i++)
		put_unaligned_le16(scsc_mx_mib_probe[i].psid, mib + i * 4);

	ret = scsc_mx_hip_request(mx, MLME_GET_CFM, &req,
				  fapi_sig_size(mlme_get_req), mib,
				  sizeof(mib));
	if (ret)
		return ret;

	scsc_mx_vinfo(mx, "wlan: MLME-GET.confirm, %u bytes of MIB\n",
		      mx->hip.cfm_datalen);

	while (off < mx->hip.cfm_datalen) {
		struct scsc_mx_mib_value v;
		const char *name = "?";
		size_t n;

		n = scsc_mx_mib_decode(mx->hip.cfm_data + off,
				       mx->hip.cfm_datalen - off, &v);
		if (!n) {
			dev_warn(mx->dev, "wlan: malformed MIB at offset %zu\n",
				 off);
			break;
		}
		off += n;

		for (i = 0; i < ARRAY_SIZE(scsc_mx_mib_probe); i++)
			if (scsc_mx_mib_probe[i].psid == v.psid)
				name = scsc_mx_mib_probe[i].name;

		if (v.octet) {
			scsc_mx_vinfo(mx, "wlan:   psid 0x%04x %-20s = %zu bytes\n",
				      v.psid, name, v.datalen);
			if (scsc_mx_verbose)
				print_hex_dump(KERN_INFO, "scsc_mx: wlan mib: ",
					       DUMP_PREFIX_OFFSET, 16, 1, v.data,
					       min_t(size_t, v.datalen, 64), false);

			/*
			 * Keep the channel list: it is what the scan
			 * below asks the radio to visit, and taking it
			 * from the firmware beats hardcoding a plan.
			 */
			if (v.psid == SCSC_PSID_SUPPORTED_CHANNELS) {
				mx->hip.nchanranges =
					min_t(size_t, v.datalen / 2,
					      ARRAY_SIZE(mx->hip.chanranges) / 2);
				memcpy(mx->hip.chanranges, v.data,
				       mx->hip.nchanranges * 2);
			}

			if (v.psid == SCSC_PSID_HT_CAPABILITIES &&
			    v.datalen >= sizeof(mx->hip.ht_cap))
				memcpy(mx->hip.ht_cap, v.data,
				       sizeof(mx->hip.ht_cap));

			if (v.psid == SCSC_PSID_VHT_CAPABILITIES &&
			    v.datalen >= sizeof(mx->hip.vht_cap))
				memcpy(mx->hip.vht_cap, v.data,
				       sizeof(mx->hip.vht_cap));
		} else {
			scsc_mx_vinfo(mx, "wlan:   psid 0x%04x %-20s = 0x%x (%u)\n",
				      v.psid, name, v.value, v.value);

			if (v.psid == SCSC_PSID_HT_ACTIVATED)
				mx->hip.ht_activated = v.value;
			if (v.psid == SCSC_PSID_VHT_ACTIVATED)
				mx->hip.vht_activated = v.value;
		}
	}

	return 0;
}

/*
 * UnifiRssi and UnifiTxDataRate, both PSIDs confirmed against the same
 * TizenRT mib.h as the rest of this table. Unlike scsc_mx_wlan_probe_mib()'s
 * boot-time batch, this is meant to be called on demand -- from
 * .get_station, whenever userspace asks -- since both values move
 * constantly while associated rather than being fixed capabilities.
 *
 * rate is the firmware's own encoding, not a plain kbit/s figure:
 * bits 14-15 pick legacy/HT/VHT, and the rest packs MCS/NSS/bandwidth/GI
 * -- decoding that belongs with the cfg80211 struct it feeds
 * (struct rate_info), so it is left to the caller, in wlan.c.
 */
#define SCSC_PSID_RSSI		0x0898
#define SCSC_PSID_TX_DATA_RATE	0x08a0

int scsc_mx_mlme_get_station_info(struct scsc_mx *mx, u16 vif, s16 *rssi,
				  u16 *rate)
{
	u8 mib[8] = {};
	struct fapi_signal req;
	size_t off = 0;
	int ret;

	*rssi = 0;
	*rate = 0;

	put_unaligned_le16(SCSC_PSID_RSSI, mib);
	put_unaligned_le16(SCSC_PSID_TX_DATA_RATE, mib + 4);

	scsc_mx_fapi_init(&req, MLME_GET_REQ, vif);

	ret = scsc_mx_hip_request(mx, MLME_GET_CFM, &req,
				  fapi_sig_size(mlme_get_req), mib, sizeof(mib));
	if (ret)
		return ret;

	while (off < mx->hip.cfm_datalen) {
		struct scsc_mx_mib_value v;
		size_t n;

		n = scsc_mx_mib_decode(mx->hip.cfm_data + off,
				       mx->hip.cfm_datalen - off, &v);
		if (!n)
			break;
		off += n;

		if (v.octet)
			continue;

		/*
		 * The vendor driver checks this same sign on this same PSID
		 * and treats a non-negative reading as invalid rather than a
		 * real (impossible) non-negative dBm value -- confirmed on
		 * this hardware, not just copied: with an excellent link to
		 * the AP under test, this MIB returned a small positive
		 * number rather than anything resembling dBm. Left unset
		 * rather than substituting a sentinel like the vendor's -1,
		 * which would look like a real, if very poor, reading.
		 */
		if (v.psid == SCSC_PSID_RSSI && (s16)v.value < 0)
			*rssi = (s16)v.value;
		else if (v.psid == SCSC_PSID_TX_DATA_RATE)
			*rate = (u16)v.value;
	}

	return 0;
}

/*
 * The address the radio puts in the probe requests it sends while
 * scanning. The vendor reads the real one out of /efs/wifi/.mac.info or
 * a MIB and falls back to a fixed value; there is no netdev here to own
 * one, so it is a parameter with a locally-administered default.
 */
static u64 scsc_mx_mac_addr = 0x020000000002ULL;
module_param_named(mac_addr, scsc_mx_mac_addr, ullong, 0444);
MODULE_PARM_DESC(mac_addr,
		 "MAC address used as the scan source address (default locally administered)");

static void scsc_mx_put_mac(u8 *dst, u64 addr)
{
	int i;

	for (i = 0; i < ETH_ALEN; i++)
		dst[i] = addr >> (8 * (ETH_ALEN - 1 - i));
}

/*
 * The 5 GHz band where radar detection applies, so we may only listen.
 * Channels 52 to 144, which is 5260 to 5720 MHz.
 */
static bool scsc_mx_freq_is_dfs(u32 mhz)
{
	return mhz >= 5260 && mhz <= 5720;
}

/*
 * Start a scan.
 *
 * MLME-ADD-SCAN.request runs on vif 0 -- the device interface, which
 * always exists -- so no virtual interface has to be created first;
 * slsi_mlme_add_scan() passes vif 0 and a NULL netdev for the same
 * reason. Two vendor-specific information elements come after the
 * signal: scan timing, and the list of channels to visit.
 *
 * This returns once the firmware has accepted the request. Results
 * arrive later as unsolicited MLME-SCAN.indications, an unknown number
 * of them, and MLME-SCAN-DONE.indication ends the run -- both are
 * handed to the cfg80211 side as they arrive rather than waited for
 * here, because cfg80211 wants to issue a scan and be told later.
 */
#define SCSC_WLAN_SCAN_CHAN_MAX	64

int scsc_mx_mlme_scan_start(struct scsc_mx *mx, const u8 *ssid, u8 ssid_len,
			    const u32 *freqs, unsigned int nfreqs)
{
	/*
	 * Vendor-specific IE, Samsung OUI 00:16:32, type 1 (scan
	 * parameters), subtype 1 (timing). All-zero periods mean a
	 * one-shot scan, which is what slsi_mlme_add_scan() sends for an
	 * ordinary foreground scan.
	 */
	static const u8 timing_ie[] = {
		0xdd, 0x11, 0x00, 0x16, 0x32, 0x01, 0x01,
		0x00, 0x00, 0x00, 0x00,		/* min period */
		0x00, 0x00, 0x00, 0x00,		/* max period */
		0x00,				/* exponent */
		0x00,				/* step count */
		0x00, 0x00,			/* skip first period */
	};
	/* Same OUI and type, subtype 2: the channel list. */
	static const u8 chanlist_ie[] = {
		0xdd, 0x05, 0x00, 0x16, 0x32, 0x01, 0x02,
	};
	u8 body[2 + IEEE80211_MAX_SSID_LEN + sizeof(timing_ie) +
		sizeof(chanlist_ie) + 3 * SCSC_WLAN_SCAN_CHAN_MAX];
	struct fapi_signal req;
	unsigned int i, n = 0;
	size_t len = 0;
	u8 *ielen;
	int ret;

	if (!nfreqs)
		return -EINVAL;

	/*
	 * An SSID element -- zero-length for the wildcard, which is what
	 * makes this an active scan. slsi_set_scan_params() appends one
	 * element per requested SSID; with none at all the vendor marks
	 * every channel passive instead, and asking for an active scan
	 * without one is the contradiction the firmware rejects with
	 * FAPI_RESULTCODE_INVALID_PARAMETERS.
	 */
	if (ssid_len > IEEE80211_MAX_SSID_LEN)
		return -EINVAL;
	body[len++] = WLAN_EID_SSID;
	body[len++] = ssid_len;
	if (ssid_len) {
		memcpy(body + len, ssid, ssid_len);
		len += ssid_len;
	}

	memcpy(body + len, timing_ie, sizeof(timing_ie));
	len += sizeof(timing_ie);

	memcpy(body + len, chanlist_ie, sizeof(chanlist_ie));
	ielen = body + len + 1;
	len += sizeof(chanlist_ie);

	/*
	 * Each descriptor is the frequency in half-megahertz and a policy
	 * byte. DFS channels are listen-only, which is a regulatory
	 * requirement rather than a firmware one.
	 */
	for (i = 0; i < nfreqs && n < SCSC_WLAN_SCAN_CHAN_MAX; i++) {
		put_unaligned_le16(freqs[i] * 2, body + len);
		body[len + 2] = scsc_mx_freq_is_dfs(freqs[i]) ?
				FAPI_SCANPOLICY_PASSIVE : 0;
		len += 3;
		*ielen += 3;
		n++;
	}

	scsc_mx_fapi_init(&req, MLME_ADD_SCAN_REQ, 0);
	req.u.mlme_add_scan_req.scan_id = cpu_to_le16(SCSC_WLAN_SCAN_ID);
	req.u.mlme_add_scan_req.scan_type = cpu_to_le16(FAPI_SCANTYPE_FULL_SCAN);
	req.u.mlme_add_scan_req.report_mode_bitmap =
		cpu_to_le16(FAPI_REPORTMODE_REAL_TIME);
	scsc_mx_put_mac(req.u.mlme_add_scan_req.device_address, scsc_mx_mac_addr);

	scsc_mx_vinfo(mx, "wlan: scanning %u channels as %pM\n", n,
		      req.u.mlme_add_scan_req.device_address);

	mx->hip.scan_results = 0;

	ret = scsc_mx_hip_request(mx, MLME_ADD_SCAN_CFM, &req,
				  fapi_sig_size(mlme_add_scan_req), body, len);
	if (ret)
		return ret;

	if (scsc_mx_cfm_result(mx) != FAPI_RESULTCODE_SUCCESS) {
		dev_err(mx->dev, "wlan: scan refused, result 0x%04x (%s)\n",
			scsc_mx_cfm_result(mx),
			scsc_mx_result_name(scsc_mx_cfm_result(mx)));
		return -EIO;
	}

	return 0;
}

int scsc_mx_mlme_scan_stop(struct scsc_mx *mx)
{
	struct fapi_signal req;

	scsc_mx_fapi_init(&req, MLME_DEL_SCAN_REQ, 0);
	req.u.mlme_del_scan_req.scan_id = cpu_to_le16(SCSC_WLAN_SCAN_ID);

	return scsc_mx_hip_request(mx, MLME_DEL_SCAN_CFM, &req,
				   fapi_sig_size(mlme_del_scan_req), NULL, 0);
}

/*
 * Create and destroy a virtual interface. This is what everything above
 * a scan hangs off -- association, keys, data -- and it is driven from
 * the netdev's open and stop, which is where the vendor does it too.
 */
int scsc_mx_mlme_vif_add(struct scsc_mx *mx, u16 vif, const u8 *addr, bool ap)
{
	struct fapi_signal req;
	int ret;

	/*
	 * The FAPI interface type is chosen here rather than by the caller,
	 * for the same reason as the key types: wlan.c does not include
	 * fapi.h, and "is this an access point" is the whole of what the
	 * cfg80211 side knows.
	 */
	scsc_mx_fapi_init(&req, MLME_ADD_VIF_REQ, vif);
	req.u.mlme_add_vif_req.virtual_interface_type =
		cpu_to_le16(ap ? FAPI_VIFTYPE_AP : FAPI_VIFTYPE_STATION);
	memcpy(req.u.mlme_add_vif_req.interface_address, addr, ETH_ALEN);
	/*
	 * The device address is the P2P device's, and an AP that is not a
	 * P2P group owner has none -- the vendor passes all ones there.
	 * A station uses its own, which is what has been tested.
	 */
	if (ap)
		eth_broadcast_addr(req.u.mlme_add_vif_req.device_address);
	else
		memcpy(req.u.mlme_add_vif_req.device_address, addr, ETH_ALEN);

	ret = scsc_mx_hip_request(mx, MLME_ADD_VIF_CFM, &req,
				  fapi_sig_size(mlme_add_vif_req), NULL, 0);
	if (ret)
		return ret;

	if (scsc_mx_cfm_result(mx) != FAPI_RESULTCODE_SUCCESS) {
		dev_err(mx->dev, "wlan: add %s vif refused, result 0x%04x (%s)\n",
			ap ? "ap" : "station", scsc_mx_cfm_result(mx),
			scsc_mx_result_name(scsc_mx_cfm_result(mx)));
		return -EIO;
	}

	dev_info(mx->dev, "wlan: %s vif %u up as %pM\n",
		 ap ? "ap" : "station", vif, addr);

	return 0;
}

/*
 * Does this firmware, started the way we start it, accept an AP vif at
 * all?
 *
 * Everything else in AP mode -- beacon elements, MLME-START, the peer
 * table -- is built on the answer being yes, so it is worth one signal
 * to find out before writing any of it. Adds an AP vif and immediately
 * removes it again.
 */
static void scsc_mx_ap_smoke_test(struct scsc_mx *mx)
{
	u8 addr[ETH_ALEN];

	scsc_mx_put_mac(addr, scsc_mx_mac_addr);

	dev_info(mx->dev, "wlan: ap smoke test: adding an AP vif\n");

	if (scsc_mx_mlme_vif_add(mx, 1, addr, true)) {
		dev_err(mx->dev, "wlan: ap smoke test FAILED, no AP vif\n");
		return;
	}

	dev_info(mx->dev, "wlan: ap smoke test PASSED, the firmware took an AP vif\n");
	scsc_mx_mlme_vif_del(mx, 1);
}

/*
 * Hand the firmware a block of information elements to put in the frames
 * it generates on our behalf. purpose selects which frame -- beacon,
 * probe response, association response -- and each call replaces the
 * previous block for that purpose rather than adding to it.
 *
 * An AP needs this because the firmware builds its beacons: everything
 * hostapd wants advertised that MLME-START does not have a field for
 * arrives this way.
 */
/* wlan.h names these separately, since wlan.c has no fapi.h. Keep them so. */
static_assert(SCSC_IE_PURPOSE_BEACON == FAPI_PURPOSE_BEACON);
static_assert(SCSC_IE_PURPOSE_PROBE_RESPONSE == FAPI_PURPOSE_PROBE_RESPONSE);
static_assert(SCSC_IE_PURPOSE_ASSOCIATION_RESPONSE ==
	      FAPI_PURPOSE_ASSOCIATION_RESPONSE);

int scsc_mx_mlme_add_info_elements(struct scsc_mx *mx, u16 vif, u16 purpose,
				   const u8 *ies, u16 ies_len)
{
	struct fapi_signal req;
	int ret;

	if (ies_len > SCSC_HIP_DATA_MAX)
		return -EMSGSIZE;

	scsc_mx_fapi_init(&req, MLME_ADD_INFO_ELEMENTS_REQ, vif);
	req.u.mlme_add_info_elements_req.purpose = cpu_to_le16(purpose);

	ret = scsc_mx_hip_request(mx, MLME_ADD_INFO_ELEMENTS_CFM, &req,
				  fapi_sig_size(mlme_add_info_elements_req),
				  ies, ies_len);
	if (ret)
		return ret;

	if (scsc_mx_cfm_result(mx) != FAPI_RESULTCODE_SUCCESS) {
		dev_err(mx->dev,
			"wlan: %u bytes of elements for purpose 0x%04x refused, result 0x%04x (%s)\n",
			ies_len, purpose, scsc_mx_cfm_result(mx),
			scsc_mx_result_name(scsc_mx_cfm_result(mx)));
		return -EIO;
	}

	scsc_mx_vinfo(mx, "wlan: %u bytes of elements for purpose 0x%04x on vif %u\n",
		      ies_len, purpose, vif);

	return 0;
}

/* wlan.h names these separately, since wlan.c has no fapi.h. Keep them so. */
static_assert(SCSC_POWERMGT_ACTIVE == FAPI_POWERMANAGEMENTMODE_ACTIVE_MODE);
static_assert(SCSC_POWERMGT_POWER_SAVE == FAPI_POWERMANAGEMENTMODE_POWER_SAVE);
static_assert(SCSC_MSGTYPE_IEEE80211_ACTION == FAPI_MESSAGETYPE_IEEE80211_ACTION);
static_assert(SCSC_MSGTYPE_IEEE80211_MGMT == FAPI_MESSAGETYPE_IEEE80211_MGMT);
static_assert(SCSC_PRIORITY_CONTENTION == FAPI_PRIORITY_CONTENTION);
static_assert(SCSC_BLOCKACK_START == FAPI_REASONCODE_START);
static_assert(SCSC_BLOCKACK_END == FAPI_REASONCODE_END);
static_assert(SCSC_BLOCKACK_RESET == FAPI_REASONCODE_UNSPECIFIED_REASON);

/*
 * Toggle 802.11 power-save on a station vif -- whether it sets the PM
 * bit and goes quiet between beacons, polling the AP for anything
 * buffered, or stays awake throughout. Nothing here decides this on its
 * own; it is exactly what cfg80211's .set_power_mgmt asks for, wired
 * straight through.
 */
int scsc_mx_mlme_powermgt(struct scsc_mx *mx, u16 vif, u16 mode)
{
	struct fapi_signal req;
	int ret;

	scsc_mx_fapi_init(&req, MLME_POWERMGT_REQ, vif);
	req.u.mlme_powermgt_req.power_management_mode = cpu_to_le16(mode);

	ret = scsc_mx_hip_request(mx, MLME_POWERMGT_CFM, &req,
				  fapi_sig_size(mlme_powermgt_req), NULL, 0);
	if (ret)
		return ret;

	if (scsc_mx_cfm_result(mx) != FAPI_RESULTCODE_SUCCESS) {
		dev_err(mx->dev,
			"wlan: power management mode 0x%04x refused on vif %u, result 0x%04x (%s)\n",
			mode, vif, scsc_mx_cfm_result(mx),
			scsc_mx_result_name(scsc_mx_cfm_result(mx)));
		return -EIO;
	}

	scsc_mx_vinfo(mx, "wlan: power management mode 0x%04x on vif %u\n",
		      mode, vif);

	return 0;
}

/*
 * The firmware's channel_information: the width in MHz in the low byte,
 * with a flag for a 40 MHz channel whose secondary half sits below the
 * primary, and for 80 MHz the primary's position within the block.
 *
 * Kept here rather than in wlan.c because it is a FAPI encoding, even
 * though every input comes from a cfg80211 chandef.
 */
#define SCSC_CHANN_INFO_HT_SCB	0x0100

static u16 scsc_mx_chann_info(struct scsc_mx *mx, u16 width_mhz,
			      u32 freq_mhz, u32 center_freq_mhz)
{
	switch (width_mhz) {
	case 20:
		return 20;
	case 40:
		return center_freq_mhz < freq_mhz ? (40 | SCSC_CHANN_INFO_HT_SCB)
						  : 40;
	case 80: {
		u16 pos = (30 + freq_mhz - center_freq_mhz) / 20;

		if (pos > 3) {
			dev_warn(mx->dev,
				 "wlan: primary %u MHz is not inside the 80 MHz block at %u MHz\n",
				 freq_mhz, center_freq_mhz);
			pos = 0;
		}

		return 80 | (pos << 8);
	}
	default:
		dev_warn(mx->dev, "wlan: unsupported channel width %u MHz\n",
			 width_mhz);
		return 20;
	}
}

/*
 * Start beaconing.
 *
 * The elements are the beacon body as cfg80211 supplies it, minus the
 * fixed fields the signal carries in its own right. A hidden SSID is the
 * one case where the SSID has to be passed separately, because it is
 * absent from the beacon by definition.
 */
int scsc_mx_mlme_start_ap(struct scsc_mx *mx, u16 vif, const u8 *addr,
			  u32 freq_mhz, u32 center_freq_mhz, u16 width_mhz,
			  u16 beacon_period, u16 dtim_period, u16 capability,
			  bool hidden_ssid, const u8 *ssid, u8 ssid_len,
			  const u8 *ies, u16 ies_len)
{
	u8 *body;
	struct fapi_signal req;
	size_t len = 0;
	int ret;

	if (ssid_len > IEEE80211_MAX_SSID_LEN)
		return -EINVAL;
	if (ies_len + 2 + ssid_len > SCSC_HIP_DATA_MAX)
		return -EMSGSIZE;

	body = kmalloc(SCSC_HIP_DATA_MAX, GFP_KERNEL);
	if (!body)
		return -ENOMEM;

	if (hidden_ssid) {
		body[len++] = WLAN_EID_SSID;
		body[len++] = ssid_len;
		memcpy(body + len, ssid, ssid_len);
		len += ssid_len;
	}

	memcpy(body + len, ies, ies_len);
	len += ies_len;

	scsc_mx_fapi_init(&req, MLME_START_REQ, vif);
	memcpy(req.u.mlme_start_req.bssid, addr, ETH_ALEN);
	req.u.mlme_start_req.beacon_period = cpu_to_le16(beacon_period);
	req.u.mlme_start_req.dtim_period = cpu_to_le16(dtim_period);
	req.u.mlme_start_req.capability_information = cpu_to_le16(capability);
	req.u.mlme_start_req.authentication_type =
		cpu_to_le16(FAPI_AUTHENTICATIONTYPE_OPEN_SYSTEM);
	req.u.mlme_start_req.hidden_ssid = cpu_to_le16(hidden_ssid ? 1 : 0);
	req.u.mlme_start_req.channel_frequency = cpu_to_le16(freq_mhz * 2);
	req.u.mlme_start_req.channel_information =
		cpu_to_le16(scsc_mx_chann_info(mx, width_mhz, freq_mhz,
					       center_freq_mhz));

	scsc_mx_vinfo(mx,
		      "wlan: starting ap on vif %u as %pM, %u MHz wide %u (chan info 0x%04x), capability 0x%04x, beacon %u dtim %u, %u bytes of elements\n",
		      vif, addr, freq_mhz, width_mhz,
		      le16_to_cpu(req.u.mlme_start_req.channel_information),
		      capability, beacon_period, dtim_period, (u16)len);

	ret = scsc_mx_hip_request(mx, MLME_START_CFM, &req,
				  fapi_sig_size(mlme_start_req), body, len);
	kfree(body);
	if (ret)
		return ret;

	if (scsc_mx_cfm_result(mx) != FAPI_RESULTCODE_SUCCESS) {
		dev_err(mx->dev, "wlan: start refused, result 0x%04x (%s)\n",
			scsc_mx_cfm_result(mx),
			scsc_mx_result_name(scsc_mx_cfm_result(mx)));
		return -EIO;
	}

	return 0;
}

int scsc_mx_mlme_vif_del(struct scsc_mx *mx, u16 vif)
{
	struct fapi_signal req;
	int ret;

	scsc_mx_fapi_init(&req, MLME_DEL_VIF_REQ, vif);
	ret = scsc_mx_hip_request(mx, MLME_DEL_VIF_CFM, &req,
				  fapi_sig_size(mlme_del_vif_req), NULL, 0);
	if (!ret)
		dev_info(mx->dev, "wlan: vif %u down\n", vif);

	return ret;
}

/*
 * Set the regulatory domain.
 *
 * The firmware boots in the world domain -- unifiDefaultCountry reads
 * "00 " and unifiRegulatoryParameters begins "00" -- which permits
 * receiving but not initiating radiation. That is consistent with a
 * chip that scans happily and refuses to associate.
 *
 * The vendor only *reads* the country during startup, because on a
 * retail device the .hcf and the EFS partition already agree on one.
 * Here nothing sets it, so this does, from a module parameter.
 *
 * The MIB entry is the encoding from mib.c: psid, length, then the
 * value as a variable-length data item. A three-byte octet string
 * encodes as 0xa1 0x03 and the bytes -- MORE and TYPE set, one length
 * byte -- and the entry is padded to an even length.
 */
static char *scsc_mx_country;
module_param_named(country, scsc_mx_country, charp, 0444);
MODULE_PARM_DESC(country,
		 "ISO 3166-1 alpha-2 regulatory domain to set, e.g. GB (default: leave the firmware's)");

#define SCSC_PSID_DEFAULT_COUNTRY	0x1f4d
#define SCSC_MIB_MORE_MASK_ENC		0x80
#define SCSC_MIB_TYPE_MASK_ENC		0x20

/*
 * Encode one MIB entry with an unsigned value, as mib.c does: a value
 * below 64 is a single byte that is the value, anything larger is a
 * count byte with the MORE bit set followed by big-endian bytes. The
 * entry is psid, length, value, and a pad to an even length.
 *
 * Returns the number of bytes written.
 */
static size_t scsc_mx_mib_encode_uint(u8 *buf, u16 psid, u32 value)
{
	size_t vlen = 0;

	if (value < 64) {
		buf[4] = value;
		vlen = 1;
	} else {
		u8 n = 0;
		int i;

		for (i = 3; i >= 0; i--)
			if (value >> (i * 8))
				break;
		n = i + 1;
		buf[4] = n | 0x80;
		for (i = 0; i < n; i++)
			buf[5 + i] = value >> ((n - 1 - i) * 8);
		vlen = 1 + n;
	}

	put_unaligned_le16(psid, buf);
	put_unaligned_le16(vlen, buf + 2);
	if ((4 + vlen) & 1) {
		buf[4 + vlen] = 0;
		return 5 + vlen;
	}

	return 4 + vlen;
}

static int scsc_mx_mlme_set_mib(struct scsc_mx *mx, const u8 *mib, u16 len)
{
	struct fapi_signal req;

	scsc_mx_fapi_init(&req, MLME_SET_REQ, 0);

	return scsc_mx_hip_request(mx, MLME_SET_CFM, &req,
				   fapi_sig_size(mlme_set_req), mib, len);
}

/*
 * Turn on the firmware's debug output.
 *
 * unifiDebugEnable is a boolean that defaults to FALSE -- "Debug to
 * host state. Debug is either is sent to the host or it isn't" -- and
 * unifiDebugControl sets the level per module, 0 to 255. With the
 * first off, the firmware's own log carries power and initialisation
 * messages and nothing from the MLME, which is exactly the gap when
 * trying to find out why it refuses an association.
 *
 * Note the output does not come back on the firmware log stream: it
 * arrives as DEBUG_WORD12 indications on the HIP control queue.
 */
#define SCSC_PSID_DEBUG_CONTROL		0x13a5
#define SCSC_PSID_DEBUG_ENABLE		0x17b7
#define SCSC_PSID_24G_40MHZ_CHANNELS	0x07f3

static void scsc_mx_enable_fw_debug(struct scsc_mx *mx)
{
	u8 mib[12];
	size_t len;

	len = scsc_mx_mib_encode_uint(mib, SCSC_PSID_DEBUG_ENABLE, 1);
	if (scsc_mx_mlme_set_mib(mx, mib, len)) {
		dev_warn(mx->dev, "wlan: could not enable firmware debug\n");
		return;
	}

	len = scsc_mx_mib_encode_uint(mib, SCSC_PSID_DEBUG_CONTROL,
				      scsc_mx_fw_debug_level & 0xff);
	if (scsc_mx_mlme_set_mib(mx, mib, len))
		dev_warn(mx->dev, "wlan: could not set firmware debug level\n");

	dev_info(mx->dev, "wlan: firmware debug enabled, level %u\n",
		 scsc_mx_fw_debug_level & 0xff);
}

static int scsc_mx_mlme_set_country(struct scsc_mx *mx, const char *alpha2)
{
	struct fapi_signal req;
	u8 mib[10];
	int ret;

	if (!alpha2 || strlen(alpha2) < 2)
		return -EINVAL;

	put_unaligned_le16(SCSC_PSID_DEFAULT_COUNTRY, mib);
	put_unaligned_le16(5, mib + 2);			/* value length */
	mib[4] = 1 | SCSC_MIB_MORE_MASK_ENC | SCSC_MIB_TYPE_MASK_ENC;
	mib[5] = 3;					/* octets that follow */
	mib[6] = toupper(alpha2[0]);
	mib[7] = toupper(alpha2[1]);
	mib[8] = ' ';
	mib[9] = 0;					/* pad to even */

	scsc_mx_fapi_init(&req, MLME_SET_REQ, 0);

	ret = scsc_mx_hip_request(mx, MLME_SET_CFM, &req,
				  fapi_sig_size(mlme_set_req), mib, sizeof(mib));
	if (ret)
		return ret;

	dev_info(mx->dev, "wlan: regulatory domain set to %c%c\n",
		 mib[6], mib[7]);

	return 0;
}

/*
 * Tell the firmware which action frames the host wants to see.
 *
 * The vendor sends this immediately after creating a station vif and
 * before connecting, every time. The bitmaps are SLSI_STA_ACTION_FRAME_
 * BITMAP and its suspend variant from dev.h: public, WMM, WNM, QoS and
 * protected-dual in both, plus radio measurement while awake.
 */
#define SCSC_ACTION_FRAME_QOS			BIT(1)
#define SCSC_ACTION_FRAME_PUBLIC		BIT(4)
#define SCSC_ACTION_FRAME_RADIO_MEASUREMENT	BIT(5)
#define SCSC_ACTION_FRAME_PROTECTED_DUAL	BIT(9)
#define SCSC_ACTION_FRAME_WNM			BIT(10)
#define SCSC_ACTION_FRAME_WMM			BIT(17)

#define SCSC_STA_ACTION_FRAMES		(SCSC_ACTION_FRAME_PUBLIC | \
					 SCSC_ACTION_FRAME_WMM | \
					 SCSC_ACTION_FRAME_WNM | \
					 SCSC_ACTION_FRAME_QOS | \
					 SCSC_ACTION_FRAME_PROTECTED_DUAL)

int scsc_mx_mlme_register_action_frames(struct scsc_mx *mx, u16 vif)
{
	struct fapi_signal req;
	int ret;

	scsc_mx_fapi_init(&req, MLME_REGISTER_ACTION_FRAME_REQ, vif);
	req.u.mlme_register_action_frame_req.action_frame_category_bitmap_active =
		cpu_to_le32(SCSC_STA_ACTION_FRAMES |
			    SCSC_ACTION_FRAME_RADIO_MEASUREMENT);
	req.u.mlme_register_action_frame_req.action_frame_category_bitmap_suspended =
		cpu_to_le32(SCSC_STA_ACTION_FRAMES);

	ret = scsc_mx_hip_request(mx, MLME_REGISTER_ACTION_FRAME_CFM, &req,
				  fapi_sig_size(mlme_register_action_frame_req),
				  NULL, 0);
	if (ret)
		return ret;

	if (scsc_mx_cfm_result(mx) != FAPI_RESULTCODE_SUCCESS) {
		dev_err(mx->dev,
			"wlan: action frame registration refused, result 0x%04x (%s)\n",
			scsc_mx_cfm_result(mx),
			scsc_mx_result_name(scsc_mx_cfm_result(mx)));
		return -EIO;
	}

	scsc_mx_vinfo(mx, "wlan: action frames registered on vif %u\n", vif);

	return 0;
}

/*
 * Associate.
 *
 * MLME-CONNECT.request names the BSS and the channel and carries the
 * SSID as an information element; the firmware does the authentication
 * and association exchange itself. The confirm only says the request
 * was accepted -- the outcome arrives later as MLME-CONNECT.indication,
 * and MLME-CONNECTED.indication after that when the link is usable.
 *
 * Frequencies are in half-megahertz here as everywhere else in FAPI.
 */
#define SCSC_WLAN_SEC_IE_MAX	256

int scsc_mx_mlme_connect(struct scsc_mx *mx, u16 vif, const u8 *bssid,
			 u32 freq_mhz, const u8 *ssid, u8 ssid_len,
			 const u8 *sec_ie, u16 sec_ie_len, bool sae)
{
	u8 body[2 + IEEE80211_MAX_SSID_LEN + SCSC_WLAN_SEC_IE_MAX];
	struct fapi_signal req;
	size_t len = 0;
	int ret;

	if (ssid_len > IEEE80211_MAX_SSID_LEN ||
	    sec_ie_len > SCSC_WLAN_SEC_IE_MAX)
		return -EINVAL;

	body[len++] = WLAN_EID_SSID;
	body[len++] = ssid_len;
	memcpy(body + len, ssid, ssid_len);
	len += ssid_len;

	/*
	 * The RSN element goes in the request itself, not through
	 * MLME-ADD-INFO-ELEMENTS: the firmware builds the association
	 * request from it and needs it before the exchange starts.
	 */
	if (sec_ie_len) {
		memcpy(body + len, sec_ie, sec_ie_len);
		len += sec_ie_len;
	}

	scsc_mx_fapi_init(&req, MLME_CONNECT_REQ, vif);
	memcpy(req.u.mlme_connect_req.bssid, bssid, ETH_ALEN);
	req.u.mlme_connect_req.authentication_type =
		cpu_to_le16(sae ? FAPI_AUTHENTICATIONTYPE_SAE :
				  FAPI_AUTHENTICATIONTYPE_OPEN_SYSTEM);
	req.u.mlme_connect_req.channel_frequency = cpu_to_le16(freq_mhz * 2);

	dev_info(mx->dev, "wlan: connecting vif %u to %pM on %u MHz\n", vif,
		 bssid, freq_mhz);

	ret = scsc_mx_hip_request(mx, MLME_CONNECT_CFM, &req,
				  fapi_sig_size(mlme_connect_req), body, len);
	if (ret)
		return ret;

	if (scsc_mx_cfm_result(mx) != FAPI_RESULTCODE_SUCCESS) {
		dev_err(mx->dev, "wlan: connect refused, result 0x%04x (%s)\n",
			scsc_mx_cfm_result(mx),
			scsc_mx_result_name(scsc_mx_cfm_result(mx)));
		return -EIO;
	}

	return 0;
}

int scsc_mx_mlme_disconnect(struct scsc_mx *mx, u16 vif, const u8 *bssid,
			    u16 reason)
{
	struct fapi_signal req;
	int ret;

	scsc_mx_fapi_init(&req, MLME_DISCONNECT_REQ, vif);
	memcpy(req.u.mlme_disconnect_req.peer_sta_address, bssid, ETH_ALEN);
	req.u.mlme_disconnect_req.reason_code = cpu_to_le16(reason);

	dev_info(mx->dev, "wlan: disconnecting vif %u from %pM, reason %u\n",
		 vif, bssid, reason);

	ret = scsc_mx_hip_request(mx, MLME_DISCONNECT_CFM, &req,
				  fapi_sig_size(mlme_disconnect_req), NULL, 0);
	if (ret)
		return ret;

	if (scsc_mx_cfm_result(mx) != FAPI_RESULTCODE_SUCCESS) {
		dev_warn(mx->dev, "wlan: disconnect result 0x%04x (%s)\n",
			 scsc_mx_cfm_result(mx),
			 scsc_mx_result_name(scsc_mx_cfm_result(mx)));
		return -EIO;
	}

	return 0;
}

/*
 * Send an 802.3 frame through the control path.
 *
 * This is how EAPOL goes out: the vendor's slsi_tx_eapol() hands the
 * frame to MLME-SEND-FRAME.request rather than MA-UNITDATA, so the
 * 4-way handshake works without a data path at all. Fire and forget --
 * there is a confirm, but the handshake is driven by what comes back
 * over the air, not by it.
 *
 * message_type matters. M4 is the last frame of the handshake and the
 * firmware treats it specially, because the pairwise key is installed
 * immediately afterwards and the frame has to leave before that. The
 * test for it is the vendor's: a key frame with MIC set, ACK clear and
 * the pairwise bit set.
 */
#define SCSC_EAPOL_IEEE8021X_TYPE	1
#define SCSC_EAPOL_DESCRIPTOR_TYPE	4
#define SCSC_EAPOL_KEY_INFO_HI		5
#define SCSC_EAPOL_KEY_INFO_LO		6
#define SCSC_IEEE8021X_TYPE_EAPOL_KEY	3
#define SCSC_EAPOL_TYPE_RSN_KEY		2
#define SCSC_EAPOL_TYPE_WPA_KEY		254
#define SCSC_EAPOL_KEY_TYPE_PAIRWISE	BIT(3)	/* in the low byte */
#define SCSC_EAPOL_KEY_ACK		BIT(7)	/* in the low byte */
#define SCSC_EAPOL_KEY_MIC		BIT(0)	/* in the high byte */
#define SCSC_EAPOL_KEY_SECURE		BIT(1)	/* in the high byte */

u16 scsc_mx_eapol_message_type(const u8 *frame, u16 len)
{
	const u8 *eapol = frame + ETH_HLEN;

	if (len < ETH_HLEN + 7)
		return FAPI_MESSAGETYPE_EAP_MESSAGE;

	if (eapol[SCSC_EAPOL_IEEE8021X_TYPE] != SCSC_IEEE8021X_TYPE_EAPOL_KEY)
		return FAPI_MESSAGETYPE_EAP_MESSAGE;

	if (eapol[SCSC_EAPOL_KEY_INFO_LO] & SCSC_EAPOL_KEY_ACK)
		return FAPI_MESSAGETYPE_EAPOL_KEY_M123;
	if (!(eapol[SCSC_EAPOL_KEY_INFO_LO] & SCSC_EAPOL_KEY_TYPE_PAIRWISE))
		return FAPI_MESSAGETYPE_EAPOL_KEY_M123;

	if ((eapol[SCSC_EAPOL_DESCRIPTOR_TYPE] == SCSC_EAPOL_TYPE_RSN_KEY &&
	     eapol[SCSC_EAPOL_KEY_INFO_HI] & SCSC_EAPOL_KEY_SECURE) ||
	    (eapol[SCSC_EAPOL_DESCRIPTOR_TYPE] == SCSC_EAPOL_TYPE_WPA_KEY &&
	     eapol[SCSC_EAPOL_KEY_INFO_HI] & SCSC_EAPOL_KEY_MIC))
		return FAPI_MESSAGETYPE_EAPOL_KEY_M4;

	return FAPI_MESSAGETYPE_EAPOL_KEY_M123;
}

/*
 * The next host tag.
 *
 * Not a plain counter: the firmware reads the bottom two bits as the
 * traffic queue the frame belongs to, so the sequence number lives in
 * bits 2..15 and the step is four. This driver uses one queue --
 * best effort, which is zero -- so the low bits stay clear.
 */
static u16 scsc_mx_host_tag(struct scsc_mx *mx)
{
	mx->hip.host_tag += 4;

	return mx->hip.host_tag;
}

int scsc_mx_mlme_send_frame(struct scsc_mx *mx, u16 vif, u16 msg_type,
			    const u8 *frame, u16 len)
{
	struct fapi_signal req;

	scsc_mx_fapi_init(&req, MLME_SEND_FRAME_REQ, vif);
	req.sender_pid = cpu_to_le16(FAPI_PROCESS_ID_MIN);
	req.u.mlme_send_frame_req.data_unit_descriptor =
		cpu_to_le16(FAPI_DATAUNITDESCRIPTOR_IEEE802_3_FRAME);
	req.u.mlme_send_frame_req.message_type = cpu_to_le16(msg_type);
	req.u.mlme_send_frame_req.host_tag = cpu_to_le16(scsc_mx_host_tag(mx));

	dev_dbg(mx->dev, "wlan: sending %u-byte frame, message type %u\n",
		len, msg_type);

	return scsc_mx_hip_send(mx, true, &req,
				fapi_sig_size(mlme_send_frame_req), frame, len);
}

/*
 * Send a whole 802.11 management frame -- cfg80211's .mgmt_tx, one
 * layer down. Unlike scsc_mx_mlme_send_frame()'s EAPOL, the frame
 * already has its own 802.11 header, so the firmware is told so via
 * FAPI_DATAUNITDESCRIPTOR_IEEE802_11_FRAME rather than the 802.3 one.
 * channel_frequency/dwell_time are left zero: this driver has no
 * off-channel vif, so a frame only ever goes out on whatever channel
 * the vif is already on.
 */
int scsc_mx_mlme_send_mgmt_frame(struct scsc_mx *mx, u16 vif, u16 msg_type,
				 const u8 *frame, u16 len, u16 *host_tag)
{
	struct fapi_signal req;

	scsc_mx_fapi_init(&req, MLME_SEND_FRAME_REQ, vif);
	req.sender_pid = cpu_to_le16(FAPI_PROCESS_ID_MIN);
	req.u.mlme_send_frame_req.data_unit_descriptor =
		cpu_to_le16(FAPI_DATAUNITDESCRIPTOR_IEEE802_11_FRAME);
	req.u.mlme_send_frame_req.message_type = cpu_to_le16(msg_type);
	*host_tag = scsc_mx_host_tag(mx);
	req.u.mlme_send_frame_req.host_tag = cpu_to_le16(*host_tag);

	dev_dbg(mx->dev, "wlan: sending %u-byte management frame on vif %u, message type %u\n",
		len, vif, msg_type);

	return scsc_mx_hip_send(mx, true, &req,
				fapi_sig_size(mlme_send_frame_req), frame, len);
}

/*
 * Send one 802.3 frame -- the data path proper.
 *
 * This is the only signal in the driver that goes on the from-host data
 * queue rather than the control one, and the only one with no confirm to
 * wait for. The firmware acknowledges by handing the buffer back on the
 * from-host return queue, which the bottom half already recycles, so a
 * successful return here means queued, not transmitted. MA-UNITDATA.cfm
 * exists but reports transmission status per host tag and is not needed
 * to reclaim anything.
 *
 * peer_index is the association id, and the caller's business: for a
 * station it is always 1 (the vendor derives it as queue set + 1, and a
 * station's single peer -- the AP -- is always set 0), and for an AP vif
 * it is whichever peer the frame's destination address maps to, kept in
 * wlan.c's peer table -- except for a broadcast or multicast destination,
 * where it is 0. That is not "no peer": the vendor's own queue-to-AID
 * mapping produces the same 0 for its broadcast/multicast queue (a
 * negative queue set plus one), so it is what the firmware expects there.
 * MLME-PROCEDURE-STARTED carries a field of the same name which is
 * tempting to reuse here and wrong; see its handler.
 *
 * -ENOBUFS is expected under load rather than exceptional: the pool has
 * HIP4_DAT_SLOTS segments and the firmware returns them asynchronously.
 * The caller is meant to stop its queue and wait for scsc_wlan_tx_resume().
 */
/*
 * Acknowledge the association.
 *
 * MLME-CONNECT.response has no fields beyond the vif and no confirm, so
 * it looks like a formality and is not: the firmware associates, reports
 * success, and then holds the data path shut until the host answers.
 * Without it a station is associated and mute -- frames are accepted,
 * their buffers come back, and nothing reaches the air.
 *
 * The timing carries the security state. For an open network the answer
 * goes out as soon as the connect succeeds; for a secured one the vendor
 * withholds it until the 4-way handshake has installed the pairwise key,
 * which is what stops unencrypted data escaping in between.
 *
 * Queued rather than sent, because the open case is decided in the
 * bottom half, which is already holding the lock the send would take.
 * The transport work does it once the lock is free.
 */
void scsc_mx_mlme_connect_resp(struct scsc_mx *mx, u16 vif)
{
	if (vif < 1 || vif > SCSC_WLAN_MAX_VIFS)
		return;

	set_bit(vif, &mx->hip.connect_resp_pending);
	schedule_work(&mx->hip.work);
}

static void scsc_mx_mlme_connect_resp_send(struct scsc_mx *mx, u16 vif)
{
	struct fapi_signal req;
	int ret;

	scsc_mx_fapi_init(&req, MLME_CONNECT_RES, vif);
	req.sender_pid = cpu_to_le16(FAPI_PROCESS_ID_MIN);

	ret = scsc_mx_hip_send(mx, true, &req, fapi_sig_size(mlme_connect_res),
			       NULL, 0);
	if (ret)
		dev_err(mx->dev,
			"wlan: could not acknowledge the association on vif %u: %d\n",
			vif, ret);
	else
		scsc_mx_vinfo(mx, "wlan: association acknowledged on vif %u\n",
			      vif);
}

/*
 * Accept a station on an AP vif. The AP-mode counterpart of
 * scsc_mx_mlme_connect_resp() above -- same reason to queue it (it can be
 * decided from the transport's bottom half, at MLME-CONNECTED.ind, which
 * already holds the lock this would need) and the same withholding
 * behaviour on a secured network, except the decision is per station
 * rather than per vif, hence the peer_index.
 */
void scsc_mx_mlme_connected_resp(struct scsc_mx *mx, u16 vif, u16 peer_index)
{
	if (vif < 1 || vif > SCSC_WLAN_MAX_VIFS)
		return;
	if (!peer_index || peer_index >= BITS_PER_LONG)
		return;

	set_bit(peer_index, &mx->hip.connected_resp_pending[vif]);
	schedule_work(&mx->hip.work);
}

static void scsc_mx_mlme_connected_resp_send(struct scsc_mx *mx, u16 vif,
					     u16 peer_index)
{
	struct fapi_signal req;
	int ret;

	scsc_mx_fapi_init(&req, MLME_CONNECTED_RES, vif);
	req.sender_pid = cpu_to_le16(FAPI_PROCESS_ID_MIN);
	req.u.mlme_connected_res.peer_index = cpu_to_le16(peer_index);

	ret = scsc_mx_hip_send(mx, true, &req,
			       fapi_sig_size(mlme_connected_res), NULL, 0);
	if (ret)
		dev_err(mx->dev,
			"wlan: could not accept peer %u on vif %u: %d\n",
			peer_index, vif, ret);
	else
		scsc_mx_vinfo(mx, "wlan: peer %u accepted on vif %u\n",
			      peer_index, vif);
}

/*
 * Acknowledge a roam. The roam-mode counterpart of
 * scsc_mx_mlme_connect_resp() -- same shape, same reason to queue it
 * (scsc_wlan_add_key() can decide this while holding the same lock the
 * send would need), and the same withholding behaviour when the new BSS
 * needs a fresh pairwise key: MLME_ROAMED_IND's temporal_keys_required
 * says so, and scsc_wlan_roamed() defers to here instead of
 * scsc_mx_mlme_connect_resp() so the right signal goes out once the key
 * lands.
 */
void scsc_mx_mlme_roamed_resp(struct scsc_mx *mx, u16 vif)
{
	if (vif < 1 || vif > SCSC_WLAN_MAX_VIFS)
		return;

	set_bit(vif, &mx->hip.roamed_resp_pending);
	schedule_work(&mx->hip.work);
}

static void scsc_mx_mlme_roamed_resp_send(struct scsc_mx *mx, u16 vif)
{
	struct fapi_signal req;
	int ret;

	scsc_mx_fapi_init(&req, MLME_ROAMED_RES, vif);
	req.sender_pid = cpu_to_le16(FAPI_PROCESS_ID_MIN);

	ret = scsc_mx_hip_send(mx, true, &req, fapi_sig_size(mlme_roamed_res),
			       NULL, 0);
	if (ret)
		dev_err(mx->dev,
			"wlan: could not acknowledge the roam on vif %u: %d\n",
			vif, ret);
	else
		scsc_mx_vinfo(mx, "wlan: roam acknowledged on vif %u\n", vif);
}

/*
 * Called from .external_auth, which runs from the cfg80211 op path
 * rather than the transport's bottom half, so there is no lock this
 * would need to take that is not already free -- unlike
 * scsc_mx_mlme_roamed_resp() and friends, this can send directly
 * rather than deferring through scsc_mx_hip_work().
 */
void scsc_mx_mlme_synchronised_resp(struct scsc_mx *mx, u16 vif,
				    const u8 *bssid, bool success)
{
	struct fapi_signal req;
	int ret;

	if (vif < 1 || vif > SCSC_WLAN_MAX_VIFS)
		return;

	scsc_mx_fapi_init(&req, MLME_SYNCHRONISED_RES, vif);
	req.sender_pid = cpu_to_le16(FAPI_PROCESS_ID_MIN);
	req.u.mlme_synchronised_res.result_code =
		cpu_to_le16(success ? FAPI_RESULTCODE_HOST_REQUEST_SUCCESS :
				      FAPI_RESULTCODE_HOST_REQUEST_FAILED);
	memcpy(req.u.mlme_synchronised_res.bssid, bssid, ETH_ALEN);

	ret = scsc_mx_hip_send(mx, true, &req,
			       fapi_sig_size(mlme_synchronised_res), NULL, 0);
	if (ret)
		dev_err(mx->dev,
			"wlan: could not report authentication result on vif %u: %d\n",
			vif, ret);
	else
		scsc_mx_vinfo(mx, "wlan: authentication %s reported on vif %u\n",
			      success ? "success" : "failure", vif);
}

int scsc_mx_tx_data(struct scsc_mx *mx, u16 vif, u16 peer_index,
		    const u8 *frame, u16 len, u16 priority)
{
	struct fapi_signal req;

	if (len > SCSC_HIP_DATA_MAX)
		return -EMSGSIZE;
	if (vif < 1 || vif > SCSC_WLAN_MAX_VIFS)
		return -EINVAL;

	scsc_mx_fapi_init(&req, MA_UNITDATA_REQ, vif);
	req.sender_pid = cpu_to_le16(FAPI_PROCESS_ID_MIN);
	req.u.ma_unitdata_req.host_tag = cpu_to_le16(scsc_mx_host_tag(mx));
	req.u.ma_unitdata_req.peer_index = cpu_to_le16(peer_index);
	/*
	 * scsc_mx_tx_priority is a manual override for testing; left at its
	 * default (-1) it does nothing and the caller's own classification
	 * (contention for a non-QoS association, the DSCP/802.1p-derived TID
	 * for a QoS one -- see scsc_wlan_ndo_xmit()) is what goes out.
	 */
	req.u.ma_unitdata_req.priority =
		cpu_to_le16(scsc_mx_tx_priority < 0 ? priority
						    : scsc_mx_tx_priority);
	req.u.ma_unitdata_req.data_unit_descriptor =
		cpu_to_le16(FAPI_DATAUNITDESCRIPTOR_IEEE802_3_FRAME);

	return scsc_mx_hip_send(mx, false, &req,
				fapi_sig_size(ma_unitdata_req), frame, len);
}

/*
 * Install a key.
 *
 * length is in bits, and the sequence number is a little-endian counter
 * spread over eight 16-bit fields. WEP is the odd one out: the key
 * index is prepended to the key material and counted in the length,
 * which is why slsi_mlme_set_key() adds one before multiplying.
 */
int scsc_mx_mlme_set_key(struct scsc_mx *mx, u16 vif, u16 key_id,
			 bool pairwise, const u8 *addr, u32 cipher,
			 const u8 *key, u16 key_len, const u8 *seq, u8 seq_len)
{
	struct fapi_signal req;
	unsigned int i;
	u16 key_type;
	int ret;

	if (key_len > SCSC_HIP_DATA_MAX)
		return -EINVAL;

	/*
	 * The FAPI key types are decided here rather than by the caller,
	 * so the cfg80211 side never has to include fapi.h. A management
	 * cipher is an IGTK whatever cfg80211 calls it.
	 */
	if (cipher == WLAN_CIPHER_SUITE_AES_CMAC)
		key_type = FAPI_KEYTYPE_IGTK;
	else if (pairwise)
		key_type = FAPI_KEYTYPE_PAIRWISE;
	else
		key_type = FAPI_KEYTYPE_GROUP;

	scsc_mx_fapi_init(&req, MLME_SETKEYS_REQ, vif);
	req.u.mlme_setkeys_req.length = cpu_to_le16(key_len * 8);
	req.u.mlme_setkeys_req.key_id = cpu_to_le16(key_id);
	req.u.mlme_setkeys_req.key_type = cpu_to_le16(key_type);
	req.u.mlme_setkeys_req.cipher_suite_selector = cpu_to_le32(cipher);
	if (addr)
		memcpy(req.u.mlme_setkeys_req.address, addr, ETH_ALEN);

	for (i = 0; i < seq_len / 2 && i < 8; i++)
		req.u.mlme_setkeys_req.sequence_number[i] =
			cpu_to_le16(seq[i * 2] | (seq[i * 2 + 1] << 8));

	scsc_mx_vinfo(mx,
		      "wlan: key %u type %u cipher 0x%08x, %u bytes, for %pM on vif %u\n",
		      key_id, key_type, cipher, key_len, addr, vif);

	ret = scsc_mx_hip_request(mx, MLME_SETKEYS_CFM, &req,
				  fapi_sig_size(mlme_setkeys_req), key, key_len);
	if (ret)
		return ret;

	if (scsc_mx_cfm_result(mx) != FAPI_RESULTCODE_SUCCESS) {
		dev_err(mx->dev, "wlan: set key refused, result 0x%04x (%s)\n",
			scsc_mx_cfm_result(mx),
			scsc_mx_result_name(scsc_mx_cfm_result(mx)));
		return -EIO;
	}

	return 0;
}

/* Accessors, so wlan.c never needs the layout of struct scsc_mx. */

struct device *scsc_mx_dev(struct scsc_mx *mx)
{
	return mx->dev;
}

struct scsc_wlan *scsc_mx_wlan(struct scsc_mx *mx)
{
	return mx->wlan;
}

void scsc_mx_set_wlan(struct scsc_mx *mx, struct scsc_wlan *wlan)
{
	mx->wlan = wlan;
}

struct scsc_bt *scsc_mx_bt(struct scsc_mx *mx)
{
	return mx->bt_drv;
}

void scsc_mx_set_bt(struct scsc_mx *mx, struct scsc_bt *bt)
{
	mx->bt_drv = bt;
}

const u8 *scsc_mx_chanranges(struct scsc_mx *mx, unsigned int *pairs)
{
	*pairs = mx->hip.nchanranges;

	return mx->hip.chanranges;
}

/*
 * The firmware's own HT/VHT capability MIBs, read once during
 * scsc_mx_wlan_probe_mib(). The return value is whether the firmware
 * has HT/VHT activated at all; *cap is the raw 802.11 Capabilities
 * element body either way, but only meaningful when it is.
 */
bool scsc_mx_ht_cap(struct scsc_mx *mx, const u8 **cap)
{
	*cap = mx->hip.ht_cap;
	return mx->hip.ht_activated;
}

bool scsc_mx_vht_cap(struct scsc_mx *mx, const u8 **cap)
{
	*cap = mx->hip.vht_cap;
	return mx->hip.vht_activated;
}

u64 scsc_mx_mac_address(void)
{
	return scsc_mx_mac_addr;
}

const char *scsc_mx_country_code(void)
{
	return scsc_mx_country;
}

static int scsc_mx_wlan_start(struct scsc_mx *mx)
{
	struct hip4_hip_control *ctl;
	const struct firmware *mib;
	u32 region, mib_len = 0;
	int fh, th, ret, i;

	/*
	 * hip4_hip_control is __aligned(4096) and sits at the region base.
	 *
	 * The vendor asks for a flat 2 MB (cm_if.c), not the 0x180000 the
	 * IMG_MGR_SEC_WLAN_* map adds up to, so there is half a megabyte
	 * past the end of the documented layout. Match that rather than
	 * the map: whether the firmware uses the slack is not stated
	 * anywhere, and this is not the place to find out.
	 */
	region = scsc_mx_ram_alloc(mx, &mx->ram, SCSC_WLAN_REGION_SIZE, SZ_4K);
	if (!region) {
		dev_err(mx->dev, "wlan: no room for a %u byte region\n",
			SCSC_WLAN_REGION_SIZE);
		return -ENOMEM;
	}

	fh = scsc_mx_intr_alloc(mx, &mx->intr.fromhost_r4);
	th = scsc_mx_intr_alloc(mx, &mx->intr.tohost);
	if (fh < 0 || th < 0)
		return -ENOSPC;

	/*
	 * The .hcf carries an 8-byte file header which must NOT reach the
	 * firmware: slsi_mib_open_file() validates it, then does
	 *
	 *	mib_info->mib_data += 8;
	 *	mib_info->mib_len  -= 8;
	 *
	 * before the MIB is copied into shared memory. Passing the header
	 * through leaves the firmware parsing it as MIB content, which it
	 * survives just long enough to die in an inlined mbulk accessor
	 * inside its own MIB module.
	 *
	 * Byte 7 is the file-format marker and must read 1; the vendor
	 * treats anything else as a bad header and refuses to start.
	 */
	if (!request_firmware(&mib, SCSC_WLAN_MIB_NAME, mx->dev)) {
		if (mib->size < SCSC_WLAN_MIB_HDR_LEN + 1) {
			dev_warn(mx->dev, "wlan: %s is too short (%zu bytes)\n",
				 SCSC_WLAN_MIB_NAME, mib->size);
		} else if (mib->data[7] != 1) {
			dev_warn(mx->dev,
				 "wlan: %s has a bad header (byte 7 = %u, want 1)\n",
				 SCSC_WLAN_MIB_NAME, mib->data[7]);
		} else if (mib->size - SCSC_WLAN_MIB_HDR_LEN >
			   IMG_MGR_SEC_WLAN_MIB_SIZE) {
			dev_warn(mx->dev,
				 "wlan: %s is %zu bytes, MIB area is %u\n",
				 SCSC_WLAN_MIB_NAME, mib->size,
				 IMG_MGR_SEC_WLAN_MIB_SIZE);
		} else {
			mib_len = mib->size - SCSC_WLAN_MIB_HDR_LEN;
			memcpy_toio(mx->dram + region +
				    IMG_MGR_SEC_WLAN_MIB_OFFSET,
				    mib->data + SCSC_WLAN_MIB_HDR_LEN, mib_len);
			scsc_mx_vinfo(mx,
				      "wlan: %s: %zu bytes, %u after the header\n",
				      SCSC_WLAN_MIB_NAME, mib->size, mib_len);
		}
		release_firmware(mib);
	} else {
		dev_warn(mx->dev, "wlan: no %s, starting without a MIB\n",
			 SCSC_WLAN_MIB_NAME);
	}

	scsc_mx_mbulk_pool_format(mx, MBULK_CLASS_FROM_HOST_DAT,
				  region + IMG_MGR_SEC_WLAN_TX_DAT_OFFSET,
				  IMG_MGR_SEC_WLAN_TX_DAT_SIZE, HIP4_DAT_SLOTS);
	scsc_mx_mbulk_pool_format(mx, MBULK_CLASS_FROM_HOST_CTL,
				  region + IMG_MGR_SEC_WLAN_TX_CTL_OFFSET,
				  IMG_MGR_SEC_WLAN_TX_CTL_SIZE, HIP4_CTL_SLOTS);

	mx->hip.ctl = region;
	mx->hip.scbrd = region + offsetof(struct hip4_hip_control, scoreboard);
	mx->hip.fh_bit = fh;
	mx->hip.th_bit = th;
	mx->hip.region = region;
	mx->hip.region_size = SCSC_WLAN_REGION_SIZE;

	/*
	 * Built in kernel memory and copied over in one go: 8 KB of
	 * structure is far easier to get right as plain C than as a few
	 * hundred writel()s into __iomem.
	 */
	ctl = kzalloc(sizeof(*ctl), GFP_KERNEL);
	if (!ctl)
		return -ENOMEM;

	ctl->init.magic_number = HIP4_INIT_MAGIC;
	/*
	 * a = v4, b = v3, which is backwards from what the header says.
	 * hip4.h comments these "Config structure A (old)" and "B (new)",
	 * but hip4_init() assigns version_a_ref from config_v4 and
	 * version_b_ref from config_v3. The firmware follows the code, not
	 * the comment: getting this the documented way round makes it
	 * parse each config as the other version and then panic in its own
	 * mbulk allocator, having taken the buffer locations from the
	 * wrong structure.
	 */
	ctl->init.version_a_ref = region +
		offsetof(struct hip4_hip_control, config_v4);
	ctl->init.version_b_ref = region +
		offsetof(struct hip4_hip_control, config_v3);

	scsc_mx_fill_hip_config(&ctl->config_v3, 3, region, mib_len, fh, th);
	scsc_mx_fill_hip_config(&ctl->config_v4, 4, region, mib_len, fh, th);

	memcpy_toio(mx->dram + region, ctl, sizeof(*ctl));
	kfree(ctl);

	/* Config visible before the firmware is told where it is. */
	wmb();

	scsc_mx_unmask_tohost(mx, mx->intr.tohost);

	scsc_mx_vinfo(mx,
		      "wlan: HIP4 control @0x%x, region 0x%x+0x%x, MIB %u bytes, irqs fh %d th %d\n",
		      region, region, IMG_MGR_SEC_WLAN_SIZE, mib_len, fh, th);

	ret = scsc_mx_service_start(mx, SCSC_SERVICE_ID_WLAN, region);

	/*
	 * Everything below is firmware-written: what it made of the
	 * structure we handed it. Read back even when the start failed --
	 * whether these fields were filled in at all separates "the
	 * firmware never parsed our config" from "it parsed it and then
	 * disliked something later", and nothing else distinguishes those.
	 */
	{
		u32 ver = readl(mx->dram + region +
				offsetof(struct hip4_hip_control,
					 init.conf_hip4_ver));
		/*
		 * Read the config the firmware actually chose, not the one
		 * we happen to prefer: it fills in its own fields only in
		 * that structure, and reading the other shows all zeroes
		 * whatever happened.
		 */
		u32 cfg = region + (ver == 4 ?
			offsetof(struct hip4_hip_control, config_v4) :
			offsetof(struct hip4_hip_control, config_v3));

		scsc_mx_vinfo(mx,
			      "wlan: firmware chose HIP config version %u, compat_flag 0x%x\n",
			      ver,
			      readl(mx->dram + cfg +
				    offsetof(struct hip4_hip_config_version_3, compat_flag)));
		u16 mlme_ver = readw(mx->dram + cfg + offsetof(struct hip4_hip_config_version_3, sap_mlme_ver));
		u16 ma_ver = readw(mx->dram + cfg + offsetof(struct hip4_hip_config_version_3, sap_ma_ver));
		u16 debug_ver = readw(mx->dram + cfg + offsetof(struct hip4_hip_config_version_3, sap_debug_ver));
		u16 test_ver = readw(mx->dram + cfg + offsetof(struct hip4_hip_config_version_3, sap_test_ver));

		scsc_mx_vinfo(mx,
			      "wlan: SAP versions mlme 0x%x ma 0x%x debug 0x%x test 0x%x, fw build 0x%x patch 0x%x\n",
			      mlme_ver, ma_ver, debug_ver, test_ver,
			      readl(mx->dram + cfg + offsetof(struct hip4_hip_config_version_3, fw_build_id)),
			      readl(mx->dram + cfg + offsetof(struct hip4_hip_config_version_3, fw_patch_id)));

		/*
		 * fapi.h is generated against one version of each SAP, and
		 * a mismatch means its structure layouts no longer describe
		 * what this firmware speaks. Say so rather than sending
		 * signals built to the wrong shape -- but do not refuse to
		 * run, because the header is the thing to replace and
		 * knowing by how much it is wrong is the useful part.
		 */
		if (mlme_ver != FAPI_CONTROL_SAP_VERSION ||
		    ma_ver != FAPI_DATA_SAP_VERSION ||
		    debug_ver != FAPI_DEBUG_SAP_VERSION ||
		    test_ver != FAPI_TEST_SAP_VERSION)
			dev_warn(mx->dev,
				 "wlan: firmware SAP versions differ from fapi.h (mlme 0x%x/0x%x ma 0x%x/0x%x debug 0x%x/0x%x test 0x%x/0x%x)\n",
				 mlme_ver, FAPI_CONTROL_SAP_VERSION,
				 ma_ver, FAPI_DATA_SAP_VERSION,
				 debug_ver, FAPI_DEBUG_SAP_VERSION,
				 test_ver, FAPI_TEST_SAP_VERSION);
		mx->hip.headroom = readb(mx->dram + cfg +
			offsetof(struct hip4_hip_config_version_3, unidat_req_headroom));
		mx->hip.tailroom = readb(mx->dram + cfg +
			offsetof(struct hip4_hip_config_version_3, unidat_req_tailroom));

		scsc_mx_vinfo(mx,
			      "wlan: mbulk hints: headroom %u tailroom %u bulk_align %u\n",
			      mx->hip.headroom, mx->hip.tailroom,
			      readb(mx->dram + cfg + offsetof(struct hip4_hip_config_version_3, bulk_buffer_align)));

		/*
		 * The vendor refuses to run a HIP it does not recognise, and
		 * so should we: version_a_ref and version_b_ref both point at
		 * structures we filled, so a value that is neither means the
		 * firmware answered from somewhere we did not write.
		 */
		if (!ret && ver != HIP4_SUPPORTED_V1 && ver != HIP4_SUPPORTED_V2) {
			dev_err(mx->dev,
				"wlan: firmware reports HIP config version %u, not 3 or 4\n",
				ver);
			ret = -EPROTO;
		}
	}

	if (ret)
		return ret;

	/*
	 * The transport is live from here: the scoreboard is zero (the
	 * whole control block was), the pools are formatted, and the
	 * firmware is already watching the queues. Publishing hip.ready
	 * last is what lets the interrupt handler start dispatching.
	 */
	mx->hip.pid = FAPI_PROCESS_ID_MIN - 1;
	mx->hip.ready = true;
	scsc_mx_vinfo(mx,
		      "wlan: HIP4 transport up, %u data and %u control mbulks free\n",
		      mx->hip.pool[MBULK_CLASS_FROM_HOST_DAT].free,
		      mx->hip.pool[MBULK_CLASS_FROM_HOST_CTL].free);

	/*
	 * The MIB read has to happen before the cfg80211 side comes up:
	 * the supported-channel list it returns is what the wiphy's band
	 * tables are built from, so the interface advertises what the
	 * firmware says it has rather than a guess.
	 */
	/*
	 * Before the MIB probe rather than after, so the probe's read of
	 * unifiDefaultCountry reports what the firmware ended up with --
	 * the write is verified for free instead of being assumed.
	 */
	if (scsc_mx_country)
		scsc_mx_mlme_set_country(mx, scsc_mx_country);

	if (scsc_mx_fw_debug)
		scsc_mx_enable_fw_debug(mx);

	/*
	 * 40 MHz channels in the 2.4 GHz band are off in the firmware and
	 * stay off unless asked for. This matches the vendor, which never
	 * writes this MIB either, so a stock device is 20 MHz there too --
	 * a 40 MHz channel occupies most of the band and the coexistence
	 * rules make it a poor trade. Off by default here for the same
	 * reason; the parameter exists because the alternative was
	 * wondering why the link negotiates HT20.
	 */
	if (scsc_mx_wide_2g4) {
		u8 mib[12];
		size_t len = scsc_mx_mib_encode_uint(mib,
						     SCSC_PSID_24G_40MHZ_CHANNELS,
						     1);

		if (scsc_mx_mlme_set_mib(mx, mib, len))
			dev_warn(mx->dev,
				 "wlan: could not enable 40 MHz on 2.4 GHz\n");
		else
			dev_info(mx->dev, "wlan: 40 MHz enabled on 2.4 GHz\n");
	}

	ret = scsc_mx_wlan_probe_mib(mx);
	if (ret)
		return ret;

	if (scsc_mx_ap_smoke)
		scsc_mx_ap_smoke_test(mx);

	return scsc_wlan_register(mx);
}

/*
 * Copy the Maxwell firmware into the DRAM window the chip masters into,
 * and hand the R4 boot ROM its entry point via the mailbox scratchpad.
 *
 * The reserved region is mapped write-combining (the DT node is marked
 * no-map for exactly this reason): the WLBT R4 is a separate CPU with
 * no coherency with AP caches, so the image has to actually reach DRAM
 * rather than sit in a dirty cache line.
 */
static int scsc_mx_load_firmware(struct scsc_mx *mx)
{
	const struct firmware *fw;
	u32 hdr_length, runtime_length, entry_point;
	char build_id[FWHDR_BUILD_ID_MAX + 1];
	void __iomem *dram;
	u32 mxconf_offset;
	int ret;

	ret = request_firmware(&fw, SCSC_FW_NAME, mx->dev);
	if (ret)
		return dev_err_probe(mx->dev, ret, "failed to load %s\n",
				      SCSC_FW_NAME);

	if (fw->size < FWHDR_MIN_LENGTH) {
		ret = dev_err_probe(mx->dev, -EINVAL,
				     "%s too short (%zu bytes)\n",
				     SCSC_FW_NAME, fw->size);
		goto out;
	}

	if (memcmp(fw->data + FWHDR_MAGIC_OFFSET, FWHDR_MAGIC,
		   strlen(FWHDR_MAGIC))) {
		ret = dev_err_probe(mx->dev, -EINVAL,
				     "%s has no '%s' header magic\n",
				     SCSC_FW_NAME, FWHDR_MAGIC);
		goto out;
	}

	hdr_length = get_unaligned_le32(fw->data + FWHDR_LENGTH_OFFSET);
	runtime_length = get_unaligned_le32(fw->data + FWHDR_RUNTIME_LENGTH_OFFSET);
	entry_point = get_unaligned_le32(fw->data + FWHDR_ENTRY_POINT_OFFSET);
	mx->r4_panic_record_offset =
		get_unaligned_le32(fw->data + FWHDR_R4_PANIC_RECORD_OFFSET);
	mx->fw_size = fw->size;

	memcpy(build_id, fw->data + FWHDR_BUILD_ID_OFFSET, FWHDR_BUILD_ID_MAX);
	build_id[FWHDR_BUILD_ID_MAX] = '\0';
	strreplace(build_id, '\n', ' ');

	scsc_mx_vinfo(mx,
		      "%s: %zu bytes, hdr_length=%u runtime_length=%u entry=0x%x\n",
		      SCSC_FW_NAME, fw->size, hdr_length, runtime_length, entry_point);
	dev_info(mx->dev, "%s build: %s\n", SCSC_FW_NAME, build_id);

	/*
	 * runtime_length covers BSS past the end of the image, so it is
	 * legitimately larger than the file -- it is what has to fit.
	 */
	if (runtime_length > mx->mem_size || fw->size > mx->mem_size) {
		ret = dev_err_probe(mx->dev, -ENOSPC,
				     "firmware (%u runtime / %zu file) exceeds %zu byte region\n",
				     runtime_length, fw->size, mx->mem_size);
		goto out;
	}

	dram = mx->dram;

	/*
	 * Recover the previous run's panic record before the image lands on
	 * top of it. On a module reload this is the only chance to see why
	 * the last boot died.
	 */
	scsc_mx_dump_panic_record(mx, dram, "pre-load (previous run)");

	memcpy_toio(dram, fw->data, fw->size);
	/* Zero the BSS tail so the R4 doesn't start on stale DRAM. */
	memset_io(dram + fw->size, 0, runtime_length - fw->size);

	/*
	 * Everything past the runtime image is heap, which is where the
	 * config structure and all the transport rings live.
	 */
	mxconf_offset = scsc_mx_build_mxconf(mx, dram, ALIGN(runtime_length, 64));
	if (!mxconf_offset) {
		ret = dev_err_probe(mx->dev, -ENOMEM, "failed to build mxconf\n");
		goto out;
	}

	/*
	 * The R4 boot ROM reads its entry point from ISSR(0) and expects the
	 * magic in ISSR(2) as it comes out of reset. ISSR(1) carries the
	 * R4-relative address of the mxconf structure and ISSR(3) the
	 * firmware startup flags (0 = normal boot). R4-relative means an
	 * offset from the base of the BAAW window, i.e. from mem_base --
	 * consistent with the entry point itself being a small offset.
	 */
	writel(entry_point, mx->r4_regs + ISSR(0));
	writel(mxconf_offset, mx->r4_regs + ISSR(1));
	writel(MBOX2_MAGIC_NUMBER, mx->r4_regs + ISSR(2));
	writel(scsc_mx_startup_flags, mx->r4_regs + ISSR(3));
	wmb(); /* image and entry point must be visible before reset release */

	scsc_mx_vinfo(mx,
		      "handoff: ISSR(0)=0x%x ISSR(1)=0x%x ISSR(2)=0x%x ISSR(3)=0x%x\n",
		      readl(mx->r4_regs + ISSR(0)), readl(mx->r4_regs + ISSR(1)),
		      readl(mx->r4_regs + ISSR(2)), readl(mx->r4_regs + ISSR(3)));

	ret = 0;
out:
	release_firmware(fw);
	return ret;
}

/*
 * How far did the firmware get?
 *
 * mx140's startup is one function of 21 steps, and every step is
 * followed by a call to a marker routine at 0x800d945a:
 *
 *	bl   <step>
 *	movs r0, #N
 *	bl   0x800d945a
 *
 * The marker builds SCSC_FW_STAGE_MAGIC | N and writes it through a
 * mailbox-register helper whose index 4 selects the M4 mailbox ISSR(0)
 * (0xa20e0080 in the R4's map, 0x120e0080 in ours). So the last stage
 * the firmware completed is readable from the AP at any time, even
 * with the core wedged and every ring untouched.
 *
 * Stage N means step N returned; the firmware is inside step N+1. The
 * table names each step by its entry address so a reported stage maps
 * straight back onto the disassembly.
 */
#define SCSC_FW_STAGE_MAGIC	0x22220000
/*
 * CRT0 writes its own series to the same register through a different
 * helper (0x800d5538), tagged 0x11110000, 18 steps of it. Seeing one of
 * those means the firmware never got as far as the main init function.
 */
#define SCSC_FW_STAGE_CRT0_MAGIC	0x11110000
#define SCSC_FW_STAGE_MASK	0xffff0000

static const char *const scsc_mx_fw_steps[] = {
	 [1] = "0x800d47b8?",	 [2] = "0x800d59e4?",
	 [3] = "0x800d47b8",	 [4] = "0x800d59e4",
	 [7] = "0x800d5d08",	 [8] = "0x800d3ca8",
	 [9] = "0x800d56c8",	[10] = "0x800d50ac",
	[11] = "0x800d4794",	[12] = "0x800d75f4",
	[13] = "0x800e0648",	[14] = "0x800d5214",
	[15] = "0x800d6004",	[16] = "0x800d58ec",
	[17] = "0x800d6970",	[18] = "0x800d5550",
	[19] = "0x800d5138",	[20] = "startup complete",
};

static void scsc_mx_report_fw_stage(struct scsc_mx *mx)
{
	u32 val = readl(mx->m4_regs + ISSR(0));
	u32 stage = val & ~SCSC_FW_STAGE_MASK;
	const char *stuck_in = NULL;

	if (!scsc_mx_verbose)
		return;

	if ((val & SCSC_FW_STAGE_MASK) == SCSC_FW_STAGE_CRT0_MAGIC) {
		dev_info(mx->dev,
			 "firmware stage: still in CRT0, step %u (M4 ISSR(0)=0x%x)\n",
			 stage, val);
		return;
	}

	if ((val & SCSC_FW_STAGE_MASK) != SCSC_FW_STAGE_MAGIC) {
		dev_info(mx->dev,
			 "firmware stage: none recorded (M4 ISSR(0)=0x%x)\n",
			 val);
		return;
	}

	if (stage + 1 < ARRAY_SIZE(scsc_mx_fw_steps))
		stuck_in = scsc_mx_fw_steps[stage + 1];

	dev_info(mx->dev,
		 "firmware stage: %u of %zu completed%s%s\n",
		 stage, ARRAY_SIZE(scsc_mx_fw_steps) - 1,
		 stuck_in ? ", stopped in " : "", stuck_in ?: "");
}

/*
 * Force the firmware to panic, and see whether it can.
 *
 * The R4 gets as far as initialising its GDB transports out of mxconf
 * and then goes quiet: no ring writes, no interrupt, no panic record.
 * That has two very different explanations -- a core that has stopped
 * dead, or a core that is alive and blocked waiting on something (the
 * APM handshake being the prime suspect) -- and nothing observed so far
 * separates them.
 *
 * Raising the reserved panic bit does. It is an ordinary from-host MIF
 * interrupt, so a core that is running and servicing interrupts must
 * respond by writing a panic record; a halted core cannot. Either
 * answer rules out half the remaining search space, and unlike a cold
 * boot it needs no physical access.
 */
static void scsc_mx_poke_panic(struct scsc_mx *mx)
{
	if (!scsc_mx_poke_panic_enabled)
		return;

	u32 val;

	dev_info(mx->dev, "poking R4 panic bit %d (liveness test)\n",
		 MIFINTRBIT_RESERVED_PANIC);

	/*
	 * Vendor unmasks the panic bit right before raising it -- "belt and
	 * braces, FW should already do this". On INTMR1 a 0 unmasks.
	 */
	val = readl(mx->r4_regs + INTMR1);
	writel(val & ~BIT(MIFINTRBIT_RESERVED_PANIC), mx->r4_regs + INTMR1);
	dev_info(mx->dev, "INTMR1 0x%x -> 0x%x\n", val,
		 readl(mx->r4_regs + INTMR1));

	writel(BIT(MIFINTRBIT_RESERVED_PANIC), mx->r4_regs + INTGR1);

	msleep(500);

	dev_info(mx->dev, "after panic poke: irqs=%d INTSR0=0x%x INTSR1=0x%x\n",
		 atomic_read(&mx->irq_count), readl(mx->r4_regs + INTSR0),
		 readl(mx->r4_regs + INTSR1));
}

/*
 * Did the R4 actually execute?
 *
 * Note CENTRAL_SEQ_WIFI_STATUS is deliberately not the test here: the
 * vendor only polls it in its *shutdown* path (platform_mif_pmu_reset),
 * never on power-up, and with low-power mode disabled (bit 16 high in
 * CENTRAL_SEQ_WIFI_CONFIGURATION) the central sequencer isn't driving
 * this domain at all, so it reads 0 whether or not the core is running.
 *
 * The real evidence is the core touching memory or signalling us:
 *   - the firmware image in DRAM diverging from what we wrote (BSS,
 *     stack, transport structures),
 *   - a populated R4 panic record (it ran, then died -- still progress),
 *   - a mailbox interrupt, counted by the handler.
 */
static void scsc_mx_check_running(struct scsc_mx *mx)
{
	void __iomem *dram;
	u32 hdr_word;
	unsigned int irqs = atomic_read(&mx->irq_count);
	u32 intsr0, intsr1;

	if (!scsc_mx_verbose)
		return;

	intsr0 = readl(mx->r4_regs + INTSR0);
	intsr1 = readl(mx->r4_regs + INTSR1);

	dev_info(mx->dev,
		 "post-reset: irqs=%u INTSR0=0x%x INTSR1=0x%x MCUCTRL=0x%x\n",
		 irqs, intsr0, intsr1, readl(mx->r4_regs + MCUCTRL));
	dev_info(mx->dev,
		 "post-reset: ISSR 0=0x%x 1=0x%x 2=0x%x 3=0x%x 4=0x%x 5=0x%x\n",
		 readl(mx->r4_regs + ISSR(0)), readl(mx->r4_regs + ISSR(1)),
		 readl(mx->r4_regs + ISSR(2)), readl(mx->r4_regs + ISSR(3)),
		 readl(mx->r4_regs + ISSR(4)), readl(mx->r4_regs + ISSR(5)));

	scsc_mx_report_irq_line(mx, "wdog", mx->irq_wdog);
	scsc_mx_report_irq_line(mx, "alive", mx->irq_alive);

	/*
	 * The M4 shares the DRAM window and is released by the same PMU
	 * sequence. If it is running while the R4 is not, that is a very
	 * different problem from both cores being dead.
	 */
	dev_info(mx->dev, "post-reset: M4 INTSR0=0x%x INTSR1=0x%x MCUCTRL=0x%x ISSR 0=0x%x 1=0x%x\n",
		 readl(mx->m4_regs + INTSR0), readl(mx->m4_regs + INTSR1),
		 readl(mx->m4_regs + MCUCTRL), readl(mx->m4_regs + ISSR(0)),
		 readl(mx->m4_regs + ISSR(1)));

	scsc_mx_report_fw_stage(mx);

	dram = mx->dram;

	/* Sanity: our own write should still be there. */
	hdr_word = readl(dram + FWHDR_MAGIC_OFFSET);
	dev_info(mx->dev, "DRAM header magic reads back 0x%08x (%s)\n",
		 hdr_word,
		 hdr_word == 0x66786d73 /* "smxf" LE */ ? "image intact"
						       : "IMAGE CORRUPT");

	scsc_mx_mxlog_drain(mx);

	/* Where the management ring stands, decoded messages aside. */
	if (mx->mgmt_ready)
		dev_info(mx->dev,
			 "mgmt ring: to-AP r %u w %u, from-AP r %u w %u\n",
			 readl(dram + mx->mgmt.in.ridx_loc),
			 readl(dram + mx->mgmt.in.widx_loc),
			 readl(dram + mx->mgmt.out.ridx_loc),
			 readl(dram + mx->mgmt.out.widx_loc));

	scsc_mx_dump_panic_record(mx, dram, "post-reset");
}

/*
 * Dump the WLBT-related PMU registers we don't otherwise touch. Called
 * either side of the power-on sequence: the interesting thing is not any
 * single value but which of them move when we drive WIFI_CTRL_NS/S.
 */
static void scsc_mx_dump_pmu(struct scsc_mx *mx, const char *when)
{
	static const struct {
		const char *name;
		u32 offset;
	} regs[] = {
		{ "WIFI_DEBUG", WIFI_DEBUG },
		{ "RESET_AHEAD_WIFI_PWR_REG", RESET_AHEAD_WIFI_PWR_REG },
		{ "CLEANY_BUS_WIFI_SYS_PWR_REG", CLEANY_BUS_WIFI_SYS_PWR_REG },
		{ "LOGIC_RESET_WIFI_SYS_PWR_REG", LOGIC_RESET_WIFI_SYS_PWR_REG },
		{ "TCXO_GATE_WIFI_SYS_PWR_REG", TCXO_GATE_WIFI_SYS_PWR_REG },
		{ "WIFI_DISABLE_ISO_SYS_PWR_REG", WIFI_DISABLE_ISO_SYS_PWR_REG },
		{ "WIFI_RESET_ISO_SYS_PWR_REG", WIFI_RESET_ISO_SYS_PWR_REG },
		{ "WIFI2AP_MEM_CONFIG0", WIFI2AP_MEM_CONFIG0 },
		{ "WIFI2AP_MEM_CONFIG1", WIFI2AP_MEM_CONFIG1 },
		{ "WIFI2AP_MEM_CONFIG2", WIFI2AP_MEM_CONFIG2 },
		{ "WIFI2AP_MEM_CONFIG3", WIFI2AP_MEM_CONFIG3 },
		{ "WLBT_BOOT_TEST_RST_CFG", WLBT_BOOT_TEST_RST_CFG },
	};
	unsigned int i;
	u32 val;

	if (!scsc_mx_verbose)
		return;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		if (regmap_read(mx->pmureg, regs[i].offset, &val)) {
			dev_warn(mx->dev, "%s: %s read failed\n", when,
				 regs[i].name);
			continue;
		}
		dev_info(mx->dev, "%s: %s = 0x%x\n", when, regs[i].name, val);
	}

	if (!regmap_read(mx->pmureg, WIFI_DEBUG, &val))
		dev_info(mx->dev,
			 "%s: WIFI_DEBUG decode: MASK_CLKREQ_WIFI=%d EN_MIF_REQ=%d EN_WIFI_ACTIVE=%d EN_MIF_RESET_REQ=%d\n",
			 when, !!(val & MASK_CLKREQ_WIFI), !!(val & EN_MIF_REQ),
			 !!(val & EN_WIFI_ACTIVE), !!(val & EN_MIF_RESET_REQ));
}

/*
 * Copy the whole carveout out to a shadow buffer, and CRC it per
 * SCSC_DRAM_CHUNK on the way. The bounce is needed anyway because the
 * region is mapped write-combining and crc32_le() can't take an
 * __iomem pointer; keeping the bytes as well as the checksum is what
 * lets scsc_mx_compare_dram() say *which words* the R4 wrote rather
 * than just which 64K it dirtied.
 */
static int scsc_mx_crc_dram(struct scsc_mx *mx, u32 *crcs, void *shadow)
{
	void *buf;
	unsigned int i;

	buf = kmalloc(SCSC_DRAM_CHUNK, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	for (i = 0; i < mx->dram_chunks; i++) {
		memcpy_fromio(buf, mx->dram + i * SCSC_DRAM_CHUNK,
			      SCSC_DRAM_CHUNK);
		crcs[i] = crc32_le(~0, buf, SCSC_DRAM_CHUNK);
		if (shadow)
			memcpy(shadow + i * SCSC_DRAM_CHUNK, buf,
			       SCSC_DRAM_CHUNK);
	}

	kfree(buf);

	return 0;
}

/* Snapshot taken while the R4 is still held in reset. */
static void scsc_mx_snapshot_dram(struct scsc_mx *mx)
{
	/* 4M of shadow and two full CRC passes; leaves compare a no-op. */
	if (!scsc_mx_verbose)
		return;

	mx->dram_chunks = mx->mem_size / SCSC_DRAM_CHUNK;

	mx->dram_crc = devm_kcalloc(mx->dev, mx->dram_chunks,
				    sizeof(*mx->dram_crc), GFP_KERNEL);
	if (!mx->dram_crc)
		return;

	/* Best effort: without it we still get the per-chunk CRC report. */
	mx->dram_shadow = vmalloc(mx->mem_size);

	if (scsc_mx_crc_dram(mx, mx->dram_crc, mx->dram_shadow)) {
		dev_warn(mx->dev, "failed to snapshot DRAM\n");
		mx->dram_crc = NULL;
		return;
	}

	dev_info(mx->dev, "DRAM snapshot: %u chunks of %uK\n",
		 mx->dram_chunks, SCSC_DRAM_CHUNK / SZ_1K);
}

/*
 * Re-CRC and report. The AP writes nothing to the carveout between the
 * snapshot and here, so every difference is the R4's doing.
 */
static void scsc_mx_compare_dram(struct scsc_mx *mx)
{
	unsigned int i, changed = 0, words = 0;
	void *now_bytes = NULL;
	u32 *now;

	if (!mx->dram_crc)
		return;

	now = kcalloc(mx->dram_chunks, sizeof(*now), GFP_KERNEL);
	if (!now)
		return;

	if (mx->dram_shadow)
		now_bytes = vmalloc(mx->mem_size);

	if (scsc_mx_crc_dram(mx, now, now_bytes)) {
		dev_warn(mx->dev, "failed to re-read DRAM\n");
		goto out;
	}

	for (i = 0; i < mx->dram_chunks; i++) {
		if (now[i] == mx->dram_crc[i])
			continue;

		changed++;
		/* Cap the log: a running core could dirty the whole region. */
		if (changed <= 8)
			dev_info(mx->dev,
				 "DRAM changed at +0x%06x (chunk %u): crc 0x%08x -> 0x%08x\n",
				 i * SCSC_DRAM_CHUNK, i, mx->dram_crc[i], now[i]);

		/*
		 * Word-level diff of the dirtied chunk. This is the whole
		 * point of the shadow: the one chunk the R4 touches happens
		 * to be the one holding the panic record, so what it wrote
		 * there decides whether "panic record empty" means the
		 * firmware never died or that we are misreading the record.
		 */
		if (!now_bytes || words > SCSC_DRAM_DIFF_WORDS)
			continue;

		for (unsigned int off = 0; off < SCSC_DRAM_CHUNK; off += 4) {
			u32 abs = i * SCSC_DRAM_CHUNK + off;
			u32 was = get_unaligned_le32(mx->dram_shadow + abs);
			u32 is = get_unaligned_le32(now_bytes + abs);

			if (was == is)
				continue;

			if (++words > SCSC_DRAM_DIFF_WORDS) {
				dev_info(mx->dev,
					 "DRAM diff: ... more than %u words differ, truncating\n",
					 SCSC_DRAM_DIFF_WORDS);
				break;
			}

			dev_info(mx->dev,
				 "DRAM diff +0x%06x: 0x%08x -> 0x%08x%s\n",
				 abs, was, is,
				 abs == mx->r4_panic_record_offset ?
					" <-- panic record word 0" : "");
		}
	}

	if (changed)
		dev_info(mx->dev,
			 "DRAM: %u of %u chunks changed -- THE R4 IS EXECUTING\n",
			 changed, mx->dram_chunks);
	else
		dev_info(mx->dev,
			 "DRAM: bit-identical across %u chunks since last sample\n",
			 mx->dram_chunks);

	/*
	 * Re-baseline so a subsequent call reports movement since *this*
	 * sample rather than since reset release. That distinguishes a core
	 * still doing work from one that ran briefly and halted.
	 */
	memcpy(mx->dram_crc, now, mx->dram_chunks * sizeof(*now));
	if (now_bytes)
		memcpy(mx->dram_shadow, now_bytes, mx->mem_size);
out:
	vfree(now_bytes);
	kfree(now);
}

/*
 * Back the BT-ABOX window (BAAW SECTION_1) with real memory.
 *
 * Downstream declares this as a reserved-memory node carrying only a
 * size, so the address is whatever the reserved-mem allocator picks at
 * boot -- on a 4.4 run it landed at 0x8fb400000. Nothing about the
 * placement is fixed, so a plain CMA allocation of the same size is an
 * equivalent carve-out and keeps this a module-only change.
 *
 * Never fatal: without it we simply close the window as before.
 */
#define SCSC_ABOX_SIZE	SZ_4M

static void scsc_mx_alloc_abox(struct scsc_mx *mx)
{
	if (dma_set_mask_and_coherent(mx->dev, DMA_BIT_MASK(36))) {
		dev_warn(mx->dev, "no 36-bit DMA mask; BT-ABOX window closed\n");
		return;
	}

	mx->abox_cpu = dma_alloc_coherent(mx->dev, SCSC_ABOX_SIZE,
					  &mx->abox_dma, GFP_KERNEL);
	if (!mx->abox_cpu) {
		dev_warn(mx->dev, "BT-ABOX alloc failed; window closed\n");
		return;
	}

	/*
	 * The node has no IOMMU and no dma-ranges, so the DMA address is the
	 * physical one -- which is what the BAAW takes. virt_to_phys() would
	 * not do: with DMA_DIRECT_REMAP the CPU pointer is a vmap alias.
	 */
	mx->abox_base = mx->abox_dma;
	mx->abox_size = SCSC_ABOX_SIZE;
	scsc_mx_vinfo(mx, "BT-ABOX window %pa+%zx\n",
		      &mx->abox_base, mx->abox_size);
}

/*
 * Ask EL3 to open the TZASC window covering our carveout to WLBT.
 * Never fatal: the vendor keeps going on failure too, and if the
 * monitor doesn't implement the call we want the rest of the bring-up
 * to run anyway so the failure mode stays comparable to before.
 */
static void scsc_mx_configure_tzasc(struct scsc_mx *mx)
{
	struct arm_smccc_res res;

	arm_smccc_smc(EXYNOS_SMC_WLBT_TZASC_CMD, WLBT_TZASC, mx->mem_base,
		      mx->mem_size, 0, 0, 0, 0, &res);

	/*
	 * Observed: 0 on the first call after a cold boot, 3 on every call
	 * after that. The monitor implements this and is refusing to
	 * reprogram a region it has already set up, so a non-zero return on
	 * a module reload says the window is open, not that it failed.
	 */
	if ((long)res.a0 == SMCCC_RET_NOT_SUPPORTED)
		dev_warn(mx->dev,
			 "TZASC config not implemented by EL3; WLBT may not be able to read DRAM\n");
	else
		scsc_mx_vinfo(mx, "TZASC window %pa+%zx: ret 0x%lx (%s)\n",
			      &mx->mem_base, mx->mem_size, res.a0,
			      res.a0 ? "already configured" : "configured");
}

static int scsc_mx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *mem_np;
	struct reserved_mem *rmem;
	struct scsc_mx *mx;
	u32 ctrl_ns;
	int ret;

	mx = devm_kzalloc(dev, sizeof(*mx), GFP_KERNEL);
	if (!mx)
		return -ENOMEM;

	mx->dev = dev;

	mx->r4_regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mx->r4_regs))
		return dev_err_probe(dev, PTR_ERR(mx->r4_regs),
				      "failed to map R4 registers\n");

	mx->m4_regs = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(mx->m4_regs))
		return dev_err_probe(dev, PTR_ERR(mx->m4_regs),
				      "failed to map M4 registers\n");

	mx->pmureg = syscon_regmap_lookup_by_phandle(dev->of_node,
						      "samsung,syscon-phandle");
	if (IS_ERR(mx->pmureg))
		return dev_err_probe(dev, PTR_ERR(mx->pmureg),
				      "failed to look up PMU syscon\n");

	mx->acpm = devm_acpm_get_by_phandle(dev);
	if (IS_ERR(mx->acpm))
		return dev_err_probe(dev, PTR_ERR(mx->acpm),
				      "failed to get ACPM handle\n");

	ret = of_property_read_u32(dev->of_node, "samsung,acpm-dvfs-channel",
				   &mx->acpm_chan);
	if (ret)
		return dev_err_probe(dev, ret,
				      "no samsung,acpm-dvfs-channel property\n");

	mem_np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!mem_np)
		return dev_err_probe(dev, -ENODEV,
				      "no memory-region property\n");

	rmem = of_reserved_mem_lookup(mem_np);
	of_node_put(mem_np);
	if (!rmem)
		return dev_err_probe(dev, -ENODEV,
				      "failed to look up reserved-memory region\n");

	mx->mem_base = rmem->base;
	mx->mem_size = rmem->size;

	/*
	 * Write-combining, because the WLBT cores are not coherent with
	 * AP caches: what we write has to actually reach DRAM rather
	 * than sit in a dirty line. Mapped once and kept, since the
	 * management transport is worked from interrupt context.
	 */
	mx->dram = devm_ioremap_wc(dev, mx->mem_base, mx->mem_size);
	if (!mx->dram)
		return dev_err_probe(dev, -ENOMEM,
				      "failed to map shared DRAM region\n");

	INIT_WORK(&mx->mgmt_work, scsc_mx_mgmt_work);
	init_completion(&mx->mm_start_ind);
	init_completion(&mx->mm_halt_rsp);
	init_completion(&mx->sm_start_cfm);
	init_completion(&mx->sm_stop_cfm);
	INIT_WORK(&mx->mxlog_work, scsc_mx_mxlog_work);
	INIT_WORK(&mx->hip.work, scsc_mx_hip_work);
	init_completion(&mx->hip.cfm);
	mutex_init(&mx->hip.lock);
	INIT_WORK(&mx->bt.work, scsc_mx_bt_work);
	mutex_init(&mx->bt.lock);

	/*
	 * Registered after the mapping so it unwinds before it: queued
	 * work reaches into the rings, and a probe that fails after the
	 * firmware is running would otherwise leave that work to run
	 * against an unmapped window.
	 */
	ret = devm_add_action_or_reset(dev, scsc_mx_stop_mgmt, mx);
	if (ret)
		return ret;

	/*
	 * Optional: without it phase-5 mxlog records still print, just as
	 * an offset instead of text.
	 */
	if (request_firmware(&mx->logstr, SCSC_LOG_STRINGS_NAME, dev))
		dev_info(dev, "no %s; phase-5 mxlog will not be rendered\n",
			 SCSC_LOG_STRINGS_NAME);
	else
		scsc_mx_vinfo(mx, "%s: %zu bytes\n", SCSC_LOG_STRINGS_NAME,
			      mx->logstr->size);

	/* Vendor does this at probe, as soon as the carveout is known. */
	scsc_mx_configure_tzasc(mx);

	scsc_mx_alloc_abox(mx);

	ret = scsc_mx_request_irq(pdev, mx, "mbox", scsc_mx_irq, 0,
				  &mx->irq_mbox);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request mbox irq\n");

	ret = scsc_mx_request_irq(pdev, mx, "alive", scsc_mx_irq,
				  IRQF_NO_AUTOEN, &mx->irq_alive);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request alive irq\n");

	ret = scsc_mx_request_irq(pdev, mx, "wdog", scsc_mx_wdog_irq,
				  IRQF_NO_AUTOEN, &mx->irq_wdog);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request wdog irq\n");

	platform_set_drvdata(pdev, mx);

	scsc_mx_vinfo(mx,
		      "probed: r4=%p m4=%p mem=%pa+%zx\n",
		      mx->r4_regs, mx->m4_regs, &mx->mem_base, mx->mem_size);

	/*
	 * Order follows the vendor mxman_start(): everything below happens
	 * while the R4 is still held in reset, so that the moment it is
	 * released it finds a valid image in DRAM and its entry point in
	 * the mailbox.
	 */
	/* PMU state the downstream cal-if sets up at boot; mainline doesn't. */
	scsc_mx_pmu_init(mx);

	scsc_mx_enable_clocks(mx);

	/*
	 * Sample the stage marker before we reset anything.
	 *
	 * This was meant to carry a reading across kernels -- boot 4.4,
	 * bring WLAN up (which works there), warm-reboot into mainline and
	 * read the vendor firmware's final stage, since the vendor driver
	 * cannot be instrumented on 4.4 (that build has CONFIG_DEVMEM unset
	 * and exposes no register interface). It does not survive: M4
	 * ISSR(0) reads 0 here on every boot, with clocks already enabled,
	 * so the mailbox is reset across a warm reboot.
	 *
	 * Kept because that negative result is worth having on every run:
	 * it proves the marker we read after reset release was written by
	 * *this* boot rather than left over from the last one.
	 *
	 * Must stay after enable_clocks() -- with MAILBOX_WLBT_CM4 gated the
	 * window reads back zero whatever it holds, the same trap that made
	 * ISSR(0) look like it would not hold a value early on.
	 */
	scsc_mx_vinfo(mx, "stage marker left by the previous run:\n");
	scsc_mx_report_fw_stage(mx);

	/*
	 * If the domain is still powered from a previous probe (a reload,
	 * or a probe that failed after power-on), reset it first -- a
	 * power-on applied to an already-running domain does not take. On a
	 * genuine cold boot WIFI_PWRON reads 0 and this is skipped, leaving
	 * the known-good first-boot path untouched.
	 */
	ret = regmap_read(mx->pmureg, WIFI_CTRL_NS, &ctrl_ns);
	if (!ret && (ctrl_ns & WIFI_PWRON)) {
		dev_info(dev, "domain already powered (WIFI_CTRL_NS=0x%x), resetting first\n",
			 ctrl_ns);
		scsc_mx_power_off(mx);
	}

	if (!scsc_mx_mailbox_is_live(mx))
		dev_warn(dev,
			 "mailbox SFR window not responding; firmware handoff will not reach the R4\n");

	scsc_mx_mailbox_init(mx);

	/* 0 = boot from the external DRAM window, not internal IRAM. */
	ret = regmap_write(mx->pmureg, WLBT_BOOT_TEST_RST_CFG, 0);
	if (ret)
		return dev_err_probe(dev, ret,
				      "failed to clear WLBT_BOOT_TEST_RST_CFG\n");

	ret = scsc_mx_load_firmware(mx);
	if (ret)
		return ret;

	/* Last thing before the core can touch DRAM. */
	scsc_mx_snapshot_dram(mx);

	scsc_mx_dump_pmu(mx, "pre-power-on");

	ret = scsc_mx_power_on(mx);
	if (ret)
		return ret;

	/* The firmware is now free to run; wait for it to say so. */
	ret = scsc_mx_wait_start_ind(mx);

	scsc_mx_dump_pmu(mx, "post-power-on");
	scsc_mx_check_running(mx);
	scsc_mx_compare_dram(mx);

	/*
	 * A firmware that never announced itself is a failed boot, but
	 * probe still succeeds: the diagnostics above are the reason
	 * this driver exists at this stage, and tearing the device down
	 * on the way out would take the evidence with it.
	 */
	if (ret) {
		dev_warn(dev, "firmware did not complete startup\n");
	} else if (mx->irq_wdog > 0) {
		/*
		 * Only from here on can the wdog line be trusted to mean
		 * something: before MM_START_IND it reads asserted simply
		 * because the block is down. scsc_mx_poke_panic() below,
		 * when enabled, is a convenient way to see this actually
		 * fire against a real (deliberately induced) firmware
		 * fault rather than trusting it untested.
		 */
		enable_irq(mx->irq_wdog);
		mx->wdog_armed = true;
	}

	/*
	 * Only worth attempting on a firmware that announced itself: a
	 * start request into a chip that never finished booting would
	 * just time out and say nothing new.
	 */
	if (!ret && scsc_mx_service == SCSC_SERVICE_ID_BT) {
		scsc_mx_bt_start(mx);
	} else if (!ret && scsc_mx_service == SCSC_SERVICE_ID_WLAN) {
		scsc_mx_wlan_start(mx);
	} else if (!ret && scsc_mx_service == SCSC_SERVICE_ID_INVALID) {
		/*
		 * Default: both. Proven to coexist on this firmware (see
		 * the WLAN+BT concurrency finding in bt.md), so a plain
		 * `modprobe scsc_mx` should just bring both up rather than
		 * making every boot ask by hand. Each start is independent
		 * and synchronous, so trying BT regardless of whether WLAN
		 * succeeded is deliberate -- one failing should not take
		 * the other down with it.
		 */
		int wlan_ret = scsc_mx_wlan_start(mx);
		int bt_ret = scsc_mx_bt_start(mx);

		dev_info(dev, "wlan %s (%d), bt %s (%d)\n",
			 wlan_ret ? "failed" : "started", wlan_ret,
			 bt_ret ? "failed" : "started", bt_ret);
	} else if (!ret) {
		scsc_mx_service_start(mx, scsc_mx_service, 0);
	}

	/* Is it still doing anything, or did it run once and stop? */
	if (scsc_mx_verbose) {
		msleep(2000);
		dev_info(dev, "--- second look, 2s later ---");
		scsc_mx_compare_dram(mx);
		scsc_mx_check_running(mx);
	}

	/* Destructive by design, so it goes last. */
	if (scsc_mx_poke_panic_enabled) {
		scsc_mx_poke_panic(mx);
		scsc_mx_vinfo(mx, "--- after panic poke ---");
		scsc_mx_compare_dram(mx);
		scsc_mx_check_running(mx);
	}

	return 0;
}

static void scsc_mx_remove(struct platform_device *pdev)
{
	struct scsc_mx *mx = platform_get_drvdata(pdev);

	/*
	 * Userspace first: once the netdev is gone nothing can ask for a
	 * scan, so the service below it can be stopped without racing a
	 * request that is already in flight.
	 */
	scsc_wlan_unregister(mx);
	scsc_bt_unregister(mx);

	/*
	 * Unwind in the order it was built: services go first, then the
	 * firmware is asked to halt. Let it stop itself before the power
	 * sequence runs -- it has an 8051 sequencer and an RF front end to
	 * quiesce, none of which we know how to do from here.
	 *
	 * Iterated rather than a single call: with the WLAN+BT concurrency
	 * experiment (see bt.md), more than one service can be active, and
	 * each needs its own SM_MSG_STOP_REQ/CFM round trip -- scsc_mx_service_stop()
	 * is a no-op for any service that was never started.
	 */
	for (u8 id = SCSC_SERVICE_ID_WLAN; id <= SCSC_SERVICE_ID_CLK20MHZ; id++)
		scsc_mx_service_stop(mx, id);
	scsc_mx_mgmt_halt(mx);

	/* Nothing should be walking the rings while the power drops. */
	scsc_mx_stop_mgmt(mx);

	/*
	 * Stop listening before the line goes back to reading asserted
	 * simply because the block is down -- the same reason it was not
	 * enabled before boot in the first place.
	 */
	if (mx->wdog_armed) {
		disable_irq(mx->irq_wdog);
		mx->wdog_armed = false;
	}

	/*
	 * Leave the domain in a state the next probe can start from,
	 * rather than relying on a physical power cycle.
	 */
	scsc_mx_power_off(mx);

	release_firmware(mx->logstr);
	vfree(mx->dram_shadow);

	/* power_off() has already closed the window this backs. */
	if (mx->abox_cpu)
		dma_free_coherent(mx->dev, mx->abox_size, mx->abox_cpu,
				  mx->abox_dma);
}

/*
 * System suspend/resume. MM_HOST_SUSPEND and MM_HOST_RESUME exist on
 * channel 0 purely as a courtesy to the firmware -- fire-and-forget, no
 * confirm, same shape as MM_HALT_REQ -- so there is nothing here to
 * wait for and nothing that can fail in a way worth blocking a suspend
 * over.
 *
 * Only sent if the firmware is confirmed up (see wdog_armed); a boot
 * that never finished, or a firmware the watchdog has already declared
 * dead, has nobody to tell.
 *
 * This device supports s2idle only (mem_sleep shows [s2idle], not
 * deep) on the hardware this was verified against, so the suspend these
 * exercise is real but shallow: cpuidle rather than a full domain
 * power-off, and PMU_ALIVE is documented elsewhere in this driver as
 * surviving far more disruptive events (a warm reboot) than that. No
 * register save/restore is done here on that basis -- see wifi.md for
 * what was actually checked before relying on it.
 */
static int scsc_mx_pm_suspend(struct device *dev)
{
	struct scsc_mx *mx = dev_get_drvdata(dev);

	if (mx->wdog_armed)
		scsc_mx_mm_send(mx, MM_HOST_SUSPEND, 0);

	return 0;
}

static int scsc_mx_pm_resume(struct device *dev)
{
	struct scsc_mx *mx = dev_get_drvdata(dev);
	int ret;

	if (mx->wdog_armed)
		scsc_mx_mm_send(mx, MM_HOST_RESUME, 0);

	/*
	 * The vendor clears this unconditionally on resume, ahead of its
	 * own resume_handler. Nothing here ever asks WLBT to raise
	 * WIFI_ACTIVE_REQ, so this is expected to be a no-op in practice --
	 * cheap enough to do anyway rather than assume.
	 */
	ret = regmap_update_bits(mx->pmureg, WIFI_CTRL_NS, WIFI_ACTIVE_CLR,
				 WIFI_ACTIVE_CLR);
	if (ret)
		dev_warn(mx->dev, "failed to clear WIFI_ACTIVE_REQ: %d\n", ret);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(scsc_mx_pm_ops, scsc_mx_pm_suspend,
				scsc_mx_pm_resume);

static const struct of_device_id scsc_mx_of_match[] = {
	{ .compatible = "samsung,exynos7885-scsc-wifibt" },
	{ }
};
MODULE_DEVICE_TABLE(of, scsc_mx_of_match);

static struct platform_driver scsc_mx_driver = {
	.probe = scsc_mx_probe,
	.remove = scsc_mx_remove,
	.driver = {
		.name = "scsc_mx",
		.of_match_table = scsc_mx_of_match,
		.pm = pm_sleep_ptr(&scsc_mx_pm_ops),
	},
};
module_platform_driver(scsc_mx_driver);

MODULE_DESCRIPTION("Samsung SCSC Maxwell (WLBT) core driver");
MODULE_LICENSE("GPL");
