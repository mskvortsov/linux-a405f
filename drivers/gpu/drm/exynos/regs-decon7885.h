/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Exynos7885 DECON (Display Enhancement CONtroller) register definitions.
 *
 * The 7885 DPU block is a clean break from the Exynos7/7870 DECON register
 * layout used in regs-decon7.h.  Almost nothing is at the same offset.
 *
 * Address map (all relative to DECON0 base 0x14860000, size 0x10000):
 *
 *   0x0000–0x02FF  DECON0 global / timing / path / blender
 *   0x1000–0x110F  DECON0 per-window control
 *
 * Pixel data DMA lives in the separate DPP/IDMA block, not here:
 *   IDMA0 base: 0x148B1000  (referenced as idma_regs in the driver)
 *
 * Timing registers (DISPIF_TIMING_CONTROL_*) are DECON2/DP only; DECON0
 * in DSI-video mode gets its timing from the DSIM controller, so we do
 * not define or write those here.
 *
 * Sources: downstream Samsung 4.4 kernel
 *   drivers/video/fbdev/exynos/dpu/regs-decon.h
 *   drivers/video/fbdev/exynos/dpu/regs-dpp.h
 */

#ifndef EXYNOS_REGS_DECON7885_H
#define EXYNOS_REGS_DECON7885_H

/* ------------------------------------------------------------------ */
/* GLOBAL_CONTROL  0x0000                                              */
/* ------------------------------------------------------------------ */
#define DECON7885_GLOBAL_CONTROL			0x0000

#define DECON7885_GLOBAL_SRESET				BIT(28)
/* idle/run status (read-only) */
#define DECON7885_GLOBAL_IDLE_STATUS			BIT(5)
#define DECON7885_GLOBAL_RUN_STATUS			BIT(4)
/* operation mode: 0 = RGB/video, 1 = i80/command */
#define DECON7885_GLOBAL_OP_MODE_I80			BIT(8)
/* shadow enable: written in pairs; _EN latches at vsync, _EN_F is immediate */
#define DECON7885_GLOBAL_DECON_EN			BIT(1)
#define DECON7885_GLOBAL_DECON_EN_F			BIT(0)

/* ------------------------------------------------------------------ */
/* Resource occupancy info (read-only, useful for debug)               */
/* ------------------------------------------------------------------ */
#define DECON7885_RESOURCE_OCCUPANCY_0			0x0010
#define DECON7885_RESOURCE_OCCUPANCY_1			0x0014
#define DECON7885_RESOURCE_OCCUPANCY_2			0x0018

/* ------------------------------------------------------------------ */
/* SRAM_SHARE_ENABLE  0x0030                                           */
/* Single-display path for DECON0: enable floating + FF0 (8K total)   */
/* ------------------------------------------------------------------ */
#define DECON7885_SRAM_SHARE_ENABLE			0x0030
#define DECON7885_SRAM_FLOATING_EN_F			BIT(24)
#define DECON7885_SRAM_FF0_EN_F				BIT(12)
/* value to write for single DECON0 display (8K FIFO) */
#define DECON7885_SRAM_8K_DECON0 \
	(DECON7885_SRAM_FLOATING_EN_F | DECON7885_SRAM_FF0_EN_F)

/* ------------------------------------------------------------------ */
/* INTERRUPT_ENABLE  0x0040                                            */
/* ------------------------------------------------------------------ */
#define DECON7885_INTERRUPT_ENABLE			0x0040
#define DECON7885_INT_EN_MDNIE_DIMMING_END		BIT(25)
#define DECON7885_INT_EN_MDNIE_DIMMING_START		BIT(24)
#define DECON7885_INT_EN_DQE_DIMMING_END		BIT(21)
#define DECON7885_INT_EN_DQE_DIMMING_START		BIT(20)
#define DECON7885_INT_EN_FRAME_DONE			BIT(13)
#define DECON7885_INT_EN_FRAME_START			BIT(12)
#define DECON7885_INT_EN_FIFO_SEL_FIFO0		(0 << 9)
#define DECON7885_INT_EN_UNDERFLOW			BIT(8)
#define DECON7885_INT_EN_EXTRA				BIT(4)
#define DECON7885_INT_EN_GLOBAL				BIT(0)
/* mask covering all writable int-enable bits for DECON0 */
#define DECON7885_INTERRUPT_ENABLE_MASK			0x03303711U

/* EXTRA_INTERRUPT_ENABLE  0x0044 */
#define DECON7885_EXTRA_INTERRUPT_ENABLE		0x0044
#define DECON7885_EXTRA_INT_EN_RESOURCE_CONFLICT	BIT(8)
#define DECON7885_EXTRA_INT_EN_TIMEOUT			BIT(4)
#define DECON7885_EXTRA_INT_EN_ERROR			BIT(0)

/* ------------------------------------------------------------------ */
/* INTERRUPT_PENDING  0x004C                                           */
/* ------------------------------------------------------------------ */
#define DECON7885_INTERRUPT_PENDING			0x004C
#define DECON7885_INT_PEND_MDNIE_DIMMING_END		BIT(25)
#define DECON7885_INT_PEND_MDNIE_DIMMING_START		BIT(24)
#define DECON7885_INT_PEND_DQE_DIMMING_END		BIT(21)
#define DECON7885_INT_PEND_DQE_DIMMING_START		BIT(20)
#define DECON7885_INT_PEND_FRAME_DONE			BIT(13)
#define DECON7885_INT_PEND_FRAME_START			BIT(12)
#define DECON7885_INT_PEND_UNDERFLOW			BIT(8)
#define DECON7885_INT_PEND_EXTRA			BIT(4)

/* EXTRA_INTERRUPT_PENDING  0x0050 */
#define DECON7885_EXTRA_INTERRUPT_PENDING		0x0050
#define DECON7885_EXTRA_INT_PEND_RESOURCE_CONFLICT	BIT(8)
#define DECON7885_EXTRA_INT_PEND_TIMEOUT		BIT(4)
#define DECON7885_EXTRA_INT_PEND_ERROR			BIT(0)

/* ------------------------------------------------------------------ */
/* SHADOW_REG_UPDATE_REQ  0x0060                                       */
/* Write 1 to the relevant bit(s) to latch shadow registers.           */
/* ------------------------------------------------------------------ */
#define DECON7885_SHADOW_REG_UPDATE_REQ			0x0060
#define DECON7885_SHADOW_UPDATE_GLOBAL			BIT(31)
#define DECON7885_SHADOW_UPDATE_WIN(_win)		BIT(_win)
/* shorthand: update all 6 windows + global for DECON0 */
#define DECON7885_SHADOW_UPDATE_ALL_DECON0		0x8000003fU

/* ------------------------------------------------------------------ */
/* CLOCK_CONTROL_0  0x00F0                                             */
/* Auto clock-gate enable bits for DECON0.                             */
/* ------------------------------------------------------------------ */
#define DECON7885_CLOCK_CONTROL_0			0x00F0
#define DECON7885_CLOCK_CONTROL_0_DECON0_MASK		0x00511101U

/* ------------------------------------------------------------------ */
/* SPLITTER  0x0100–0x010C  (feeds FF0 for single-DSI path)           */
/* ------------------------------------------------------------------ */
#define DECON7885_SPLITTER_SIZE_CONTROL_0		0x0100
#define DECON7885_SPLITTER_HEIGHT_F(_v)			((_v) << 16)
#define DECON7885_SPLITTER_HEIGHT_MASK			(0x3fffU << 16)
#define DECON7885_SPLITTER_WIDTH_F(_v)			((_v) << 0)
#define DECON7885_SPLITTER_WIDTH_MASK			(0x3fffU << 0)

#define DECON7885_SPLITTER_SIZE_CONTROL_1		0x0104	/* pixel count */
#define DECON7885_SPLITTER_SPLIT_IDX_CONTROL		0x0108
#define DECON7885_SPLITTER_SPLIT_IDX_F(_v)		((_v) << 0)
#define DECON7885_SPLITTER_OVERLAP_CONTROL		0x010C
#define DECON7885_SPLITTER_OVERLAP_F(_v)		((_v) << 0)

/* ------------------------------------------------------------------ */
/* FRAME_FIFO  0x0120–0x0130                                           */
/* ------------------------------------------------------------------ */
#define DECON7885_FRAME_FIFO_0_SIZE_CONTROL_0		0x0120
#define DECON7885_FRAME_FIFO_HEIGHT_F(_v)		((_v) << 16)
#define DECON7885_FRAME_FIFO_HEIGHT_MASK		(0x3fffU << 16)
#define DECON7885_FRAME_FIFO_WIDTH_F(_v)		((_v) << 0)
#define DECON7885_FRAME_FIFO_WIDTH_MASK			(0x3fffU << 0)

#define DECON7885_FRAME_FIFO_0_SIZE_CONTROL_1		0x0124	/* pixel count */
#define DECON7885_FRAME_FIFO_1_SIZE_CONTROL_0		0x0128
#define DECON7885_FRAME_FIFO_1_SIZE_CONTROL_1		0x012C

#define DECON7885_FRAME_FIFO_TH_CONTROL_0		0x0130
#define DECON7885_FRAME_FIFO_1_TH_F(_v)		((_v) << 16)
#define DECON7885_FRAME_FIFO_1_TH_MASK			(0xffffU << 16)
#define DECON7885_FRAME_FIFO_0_TH_F(_v)		((_v) << 0)
#define DECON7885_FRAME_FIFO_0_TH_MASK			(0xffffU << 0)

/* ------------------------------------------------------------------ */
/* FORMATTER  0x0190, 0x01A0–0x01B4                                   */
/* ------------------------------------------------------------------ */
#define DECON7885_FORMATTER_CONTROL			0x0190
#define DECON7885_FORMATTER_RGB_ORDER_F(_v)		((_v) << 4)
#define DECON7885_FORMATTER_RGB_ORDER_MASK		(0x7U << 4)
/* RGB order values (match downstream enum decon_rgb_order) */
#define DECON7885_RGB_ORDER_RGB				0x0
#define DECON7885_RGB_ORDER_BGR				0x4

#define DECON7885_FORMATTER0_SIZE_CONTROL_0		0x01A0
#define DECON7885_FORMATTER_HEIGHT_F(_v)		((_v) << 16)
#define DECON7885_FORMATTER_HEIGHT_MASK			(0x3fffU << 16)
#define DECON7885_FORMATTER_WIDTH_F(_v)			((_v) << 0)
#define DECON7885_FORMATTER_WIDTH_MASK			(0x3fffU << 0)

#define DECON7885_FORMATTER0_SIZE_CONTROL_1		0x01A4	/* pixel count */
#define DECON7885_FORMATTER1_SIZE_CONTROL_0		0x01B0
#define DECON7885_FORMATTER1_SIZE_CONTROL_1		0x01B4

/* ------------------------------------------------------------------ */
/* BLENDER background canvas  0x0200–0x0204                            */
/* ------------------------------------------------------------------ */
#define DECON7885_BLENDER_BG_IMAGE_SIZE_0		0x0200
#define DECON7885_BG_HEIGHT_F(_v)			((_v) << 16)
#define DECON7885_BG_HEIGHT_MASK			(0x3fffU << 16)
#define DECON7885_BG_WIDTH_F(_v)			((_v) << 0)
#define DECON7885_BG_WIDTH_MASK				(0x3fffU << 0)

#define DECON7885_BLENDER_BG_IMAGE_SIZE_1		0x0204	/* pixel count */

/* ------------------------------------------------------------------ */
/* DATA_PATH_CONTROL  0x0214–0x0230                                    */
/* ------------------------------------------------------------------ */

/*
 * DATA_PATH_CONTROL_0  0x0214
 * Per-window enable and colormap-enable.
 *   WIN_EN_F(win)        = bit  4*win
 *   WIN_MAPCOLOR_EN_F(win) = bit (4*win + 1)
 */
#define DECON7885_DATA_PATH_CONTROL_0			0x0214
#define DECON7885_WIN_EN_F(_win)			BIT(4 * (_win))
#define DECON7885_WIN_MAPCOLOR_EN_F(_win)		BIT(4 * (_win) + 1)

/*
 * DATA_PATH_CONTROL_1  0x0218
 * Window → DPP channel mapping: 3-bit channel id at bits 4*win+2:4*win.
 */
#define DECON7885_DATA_PATH_CONTROL_1			0x0218
#define DECON7885_WIN_CHMAP_F(_win, _ch)		(((_ch) & 0x7) << (4 * (_win)))
#define DECON7885_WIN_CHMAP_MASK(_win)			(0x7U << (4 * (_win)))

/*
 * DATA_PATH_CONTROL_2  0x0230
 * Composite data-path selector; also holds DQE/APS/HSC path bits.
 *
 * COMP_LINKIF_WB_PATH[8:0] encodes the full pipeline selection:
 *   0x001 = no-compression, FF0, formatter0, DSIM_IF0  (A40 path)
 *
 * ENHANCE_LOGIC_PATH[14:12]: 0 = all enhancement off
 * APS_PATH[18:16]:  0 = DQE_USE
 * HSC_PATH[22:20]:  0 = DQE_USE
 */
#define DECON7885_DATA_PATH_CONTROL_2			0x0230
#define DECON7885_ENHANCE_LOGIC_PATH_F(_v)		((_v) << 12)
#define DECON7885_ENHANCE_LOGIC_PATH_MASK		(0x7U << 12)
#define DECON7885_APS_PATH_F(_v)			((_v) << 16)
#define DECON7885_APS_PATH_MASK				(0x7U << 16)
#define DECON7885_HSC_PATH_F(_v)			((_v) << 20)
#define DECON7885_HSC_PATH_MASK				(0x7U << 20)
#define DECON7885_COMP_LINKIF_WB_PATH_F(_v)		((_v) << 0)
#define DECON7885_COMP_LINKIF_WB_PATH_MASK		(0x1ffU << 0)
/* no-compression, single-FF0, formatter0, DSIM_IF0 */
#define DECON7885_DPATH_NOCOMP_FF0_FMT0_DSIMIF0	0x001U

/* ------------------------------------------------------------------ */
/* Per-window registers  base 0x1000, stride 0x30                      */
/* ------------------------------------------------------------------ */

#define DECON7885_WIN_STRIDE				0x30

/*
 * WIN_CONTROL_0  0x1000 + win*0x30
 * [31:24] alpha1  [23:16] alpha0  [11:8] blend-func  [3:0] misc
 */
#define DECON7885_WIN_CONTROL_0(_win)	(0x1000 + (_win) * DECON7885_WIN_STRIDE)
#define DECON7885_WIN_ALPHA1_F(_v)			(((_v) & 0xff) << 24)
#define DECON7885_WIN_ALPHA1_MASK			(0xffU << 24)
#define DECON7885_WIN_ALPHA0_F(_v)			(((_v) & 0xff) << 16)
#define DECON7885_WIN_ALPHA0_MASK			(0xffU << 16)
#define DECON7885_WIN_FUNC_F(_v)			(((_v) & 0xf) << 8)
#define DECON7885_WIN_FUNC_MASK				(0xfU << 8)
#define DECON7885_WIN_ALPHA_MULT_SRC_SEL_F(_v)		(((_v) & 0x3) << 0)

/* WIN_CONTROL_1  0x1004 + win*0x30 (blend equation selectors) */
#define DECON7885_WIN_CONTROL_1(_win)	(0x1004 + (_win) * DECON7885_WIN_STRIDE)

/*
 * WIN_START_POSITION  0x1008 + win*0x30
 * [29:16] Y  [13:0] X   (note: opposite word-order from Exynos7 VIDOSD_A)
 */
#define DECON7885_WIN_START_POSITION(_win) \
	(0x1008 + (_win) * DECON7885_WIN_STRIDE)
#define DECON7885_WIN_STRPTR_Y_F(_v)			(((_v) & 0x3fff) << 16)
#define DECON7885_WIN_STRPTR_X_F(_v)			(((_v) & 0x3fff) << 0)

/*
 * WIN_END_POSITION  0x100C + win*0x30
 * [29:16] Y  [13:0] X
 */
#define DECON7885_WIN_END_POSITION(_win) \
	(0x100C + (_win) * DECON7885_WIN_STRIDE)
#define DECON7885_WIN_ENDPTR_Y_F(_v)			(((_v) & 0x3fff) << 16)
#define DECON7885_WIN_ENDPTR_X_F(_v)			(((_v) & 0x3fff) << 0)

/*
 * WIN_COLORMAP_0/1  0x1010/0x1014 + win*0x30
 * COLORMAP_0: [23:16] A  [7:0] R
 * COLORMAP_1: [23:16] G  [7:0] B
 */
#define DECON7885_WIN_COLORMAP_0(_win)	(0x1010 + (_win) * DECON7885_WIN_STRIDE)
#define DECON7885_WIN_MAPCOLOR_A_F(_v)			((_v) << 16)
#define DECON7885_WIN_MAPCOLOR_A_MASK			(0xffU << 16)
#define DECON7885_WIN_MAPCOLOR_R_F(_v)			((_v) << 0)
#define DECON7885_WIN_MAPCOLOR_R_MASK			(0xffU << 0)

#define DECON7885_WIN_COLORMAP_1(_win)	(0x1014 + (_win) * DECON7885_WIN_STRIDE)
#define DECON7885_WIN_MAPCOLOR_G_F(_v)			((_v) << 16)
#define DECON7885_WIN_MAPCOLOR_G_MASK			(0xffU << 16)
#define DECON7885_WIN_MAPCOLOR_B_F(_v)			((_v) << 0)
#define DECON7885_WIN_MAPCOLOR_B_MASK			(0xffU << 0)

/* WIN_START_TIME_CONTROL  0x1018 + win*0x30 */
#define DECON7885_WIN_START_TIME_CONTROL(_win) \
	(0x1018 + (_win) * DECON7885_WIN_STRIDE)

/* WIN_PIXEL_COUNT  0x101C + win*0x30  (width * height, write-only) */
#define DECON7885_WIN_PIXEL_COUNT(_win)	(0x101C + (_win) * DECON7885_WIN_STRIDE)

/* ================================================================== */
/* IDMA0 registers — separate MMIO at 0x148B1000 (idma_regs)          */
/*                                                                      */
/* These control the DMA engine that feeds pixel data into DECON.       */
/* On Exynos7885 this IP is entirely separate from the DECON block;     */
/* the Exynos7/7870 VIDW_BUF_START registers do not exist in DECON.    */
/* ================================================================== */

/* IDMA_ENABLE  0x0000 */
#define IDMA7885_ENABLE					0x0000
#define IDMA7885_SRESET					BIT(24)
#define IDMA7885_SFR_UPDATE_FORCE			BIT(4)
#define IDMA7885_CLOCK_GATE_EN_ALL			(0x3U << 9)
#define IDMA7885_OP_STATUS				BIT(2)

/* IDMA_IN_CON  0x0008 — format, flip, compression select */
#define IDMA7885_IN_CON					0x0008
#define IDMA7885_IMG_FORMAT_F(_v)			((_v) << 11)
#define IDMA7885_IMG_FORMAT_MASK			(0x1fU << 11)
/* format codes (subset used by the A40 path) */
#define IDMA7885_FMT_ARGB8888				0
#define IDMA7885_FMT_ABGR8888				1
#define IDMA7885_FMT_RGBA8888				2
#define IDMA7885_FMT_BGRA8888				3	/* DRM BGRA8888 → framebuffer default */
#define IDMA7885_FMT_XRGB8888				4
#define IDMA7885_FMT_RGB565				8

/* IDMA_SRC_SIZE  0x0010 — allocated buffer dimensions */
#define IDMA7885_SRC_SIZE				0x0010
#define IDMA7885_SRC_HEIGHT_F(_v)			((_v) << 16)
#define IDMA7885_SRC_HEIGHT_MASK			(0x3fffU << 16)
#define IDMA7885_SRC_WIDTH_F(_v)			((_v) << 0)
#define IDMA7885_SRC_WIDTH_MASK				(0xffffU << 0)

/* IDMA_SRC_OFFSET  0x0014 — crop start within buffer */
#define IDMA7885_SRC_OFFSET				0x0014
#define IDMA7885_SRC_OFFSET_Y_F(_v)			((_v) << 16)
#define IDMA7885_SRC_OFFSET_X_F(_v)			((_v) << 0)

/* IDMA_IMG_SIZE  0x0018 — visible region size */
#define IDMA7885_IMG_SIZE				0x0018
#define IDMA7885_IMG_HEIGHT_F(_v)			((_v) << 16)
#define IDMA7885_IMG_HEIGHT_MASK			(0x1fffU << 16)
#define IDMA7885_IMG_WIDTH_F(_v)			((_v) << 0)
#define IDMA7885_IMG_WIDTH_MASK				(0x1fffU << 0)

/* IDMA_IN_BASE_ADDR_Y  0x0040 — framebuffer DMA address (luma / packed) */
#define IDMA7885_IN_BASE_ADDR_Y				0x0040
/* IDMA_IN_BASE_ADDR_C  0x0044 — chroma plane (not used for RGB) */
#define IDMA7885_IN_BASE_ADDR_C				0x0044

#endif /* EXYNOS_REGS_DECON7885_H */
