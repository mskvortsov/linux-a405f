/****************************************************************************
 *
 * Copyright (c) 2014 - 2017 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/
/*
 * mbulk ABI, taken from the Samsung 4.14 BSP (mbulk_def.h and mbulk.h in
 * drivers/net/wireless/scsc/), reduced to the parts that describe shared
 * memory. The host writes these descriptors into the WLAN region and the
 * firmware walks them, so the layout is ABI. Do not tidy it.
 *
 * Note enum mbulk_class is __packed, which is what makes struct mbulk 20
 * bytes rather than 23 -- and is exactly the kind of detail that makes
 * retyping these structures a bad idea.
 */

#ifndef __SCSC_MBULK_H__
#define __SCSC_MBULK_H__

/* Segment descriptors are aligned to this, and so are pool bases. */
#define MBULK_ALIGN	64

typedef u16 mbulk_len_t;
typedef s32 scsc_mifram_ref;

enum mbulk_class {
	MBULK_CLASS_FROM_HOST_DAT    = 0,
	MBULK_CLASS_FROM_HOST_CTL    = 1,
	MBULK_CLASS_FROM_RADIO       = 2,
	MBULK_CLASS_CONTRL           = 3,
	MBULK_CLASS_DEBUG            = 4,
	MBULK_CLASS_OTHERS           = 5,
	MBULK_CLASS_MAX
} __packed;

struct mbulk {
	scsc_mifram_ref  next_offset;       /** next mbulk offset */
	u8               flag;              /** mbulk flags */
	enum mbulk_class clas;              /** bulk buffer classification */
	u8               pid;               /** mbulk pool id */
	u8               refcnt;            /** reference counter of bulk buffer */
	mbulk_len_t      dat_bufsz;         /** data buffer size in byte */
	mbulk_len_t      sig_bufsz;         /** signal buffer size in byte */
	mbulk_len_t      len;               /** valid data length */
	mbulk_len_t      head;              /** start offset of data after mbulk structure */
	scsc_mifram_ref  chain_next_offset; /** chain next mbulk offset */
} __packed;

/* Longest scatter/gather chain the firmware will build. */
#define MBULK_MAX_CHAIN		16

/* mbulk flags */
#define MBULK_F_SIG                 (1 << 0) /** has a signal after mbulk descriptor*/
#define MBULK_F_READONLY            (1 << 1) /** is read-only */
#define MBULK_F_OBOUND              (1 << 2) /** other CPU(host) owns the buffer */
#define MBULK_F_WAKEUP		    (1 << 3) /** frame waking up the host */
#define MBULK_F_FREE                (1 << 5) /** mbulk alreay freed */
#define MBULK_F_CHAIN_HEAD          (1 << 6) /** is an head in scatter/gather chain */
#define MBULK_F_CHAIN               (1 << 7) /** is scatter/gather chain */

#endif /* __SCSC_MBULK_H__ */
