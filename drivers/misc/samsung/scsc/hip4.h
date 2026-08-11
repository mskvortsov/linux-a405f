/****************************************************************************
 *
 * Copyright (c) 2014 - 2017 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/
/*
 * HIP4 ABI definitions, taken verbatim from the Samsung 4.14 BSP
 * (drivers/net/wireless/scsc/hip4.h), with the host-side driver state
 * removed -- only the structures the firmware also reads are kept.
 *
 * These describe shared memory the firmware parses, so field order,
 * padding, __packed and __aligned all matter. Do not tidy them.
 *
 * The IMG_MGR_SEC_WLAN_* map below is from the same BSP's hip4.c, where
 * it is defined in the .c file rather than a header; it is the layout of
 * the WLAN shared-memory region, of which hip4_hip_control occupies the
 * first 8 kB.
 */

#ifndef __SCSC_HIP4_H__
#define __SCSC_HIP4_H__

#define MIF_HIP_COMPAT_FLAG_NEED_MLME_RESET     BIT(0)
#define MIF_HIP_COMPAT_FLAG_MIB_DAT_BY_FAPI     BIT(1)

#define HIP4_DAT_SLOTS                        218
#define HIP4_CTL_SLOTS                         32

#define MIF_HIP_CFG_Q_NUM       6

/* Current versions supported by this HIP */
#define HIP4_SUPPORTED_V1	3
#define HIP4_SUPPORTED_V2	4

enum hip4_hip_q_conf {
	HIP4_MIF_Q_FH_CTRL,
	HIP4_MIF_Q_FH_DAT,
	HIP4_MIF_Q_FH_RFB,
	HIP4_MIF_Q_TH_CTRL,
	HIP4_MIF_Q_TH_DAT,
	HIP4_MIF_Q_TH_RFB
};

struct hip4_hip_config_version_4 {
	/* Host owned */
	u32 magic_number;       /* 0xcaba0401 */
	u16 hip_config_ver;     /* Version of this configuration structure = 2*/
	u16 config_len;         /* Size of this configuration structure */

	/* FW owned */
	u32 compat_flag;         /* flag of the expected driver's behaviours */

	u16 sap_mlme_ver;        /* Fapi SAP_MLME version*/
	u16 sap_ma_ver;          /* Fapi SAP_MA version */
	u16 sap_debug_ver;       /* Fapi SAP_DEBUG version */
	u16 sap_test_ver;        /* Fapi SAP_TEST version */

	u32 fw_build_id;         /* Firmware Build Id */
	u32 fw_patch_id;         /* Firmware Patch Id */

	u8  unidat_req_headroom; /* Headroom the host shall reserve in mbulk for MA-UNITDATA.REQ signal */
	u8  unidat_req_tailroom; /* Tailroom the host shall reserve in mbulk for MA-UNITDATA.REQ signal */
	u8  bulk_buffer_align;   /* 4 */

	/* Host owned */
	u8  host_cache_line;    /* 64 */

	u32 host_buf_loc;       /* location of the host buffer in MIF_ADDR */
	u32 host_buf_sz;        /* in byte, size of the host buffer */
	u32 fw_buf_loc;         /* location of the firmware buffer in MIF_ADDR */
	u32 fw_buf_sz;          /* in byte, size of the firmware buffer */
	u32 mib_loc;            /* MIB location in MIF_ADDR */
	u32 mib_sz;             /* MIB size */
	u32 log_config_loc;     /* Logging Configuration Location in MIF_ADDR */
	u32 log_config_sz;      /* Logging Configuration Size in MIF_ADDR */

	u8  mif_fh_int_n;       /* MIF from-host interrupt bit position */
	u8  mif_th_int_n;       /* MIF to-host interrpt bit position */
	u8  reserved[2];

	u32 scbrd_loc;          /* Scoreboard locatin in MIF_ADDR */

	u16 q_num;              /* 6 */
	u16 q_len;              /* 256 */
	u16 q_idx_sz;           /* 1 */
	u8  reserved2[2];

	u32 q_loc[MIF_HIP_CFG_Q_NUM];

	u8  reserved3[16];
} __packed;

struct hip4_hip_config_version_3 {
	/* Host owned */
	u32 magic_number;       /* 0xcaba0401 */
	u16 hip_config_ver;     /* Version of this configuration structure = 2*/
	u16 config_len;         /* Size of this configuration structure */

	/* FW owned */
	u32 compat_flag;         /* flag of the expected driver's behaviours */

	u16 sap_mlme_ver;        /* Fapi SAP_MLME version*/
	u16 sap_ma_ver;          /* Fapi SAP_MA version */
	u16 sap_debug_ver;       /* Fapi SAP_DEBUG version */
	u16 sap_test_ver;        /* Fapi SAP_TEST version */

	u32 fw_build_id;         /* Firmware Build Id */
	u32 fw_patch_id;         /* Firmware Patch Id */

	u8  unidat_req_headroom; /* Headroom the host shall reserve in mbulk for MA-UNITDATA.REQ signal */
	u8  unidat_req_tailroom; /* Tailroom the host shall reserve in mbulk for MA-UNITDATA.REQ signal */
	u8  bulk_buffer_align;   /* 4 */

	/* Host owned */
	u8  host_cache_line;    /* 64 */

	u32 host_buf_loc;       /* location of the host buffer in MIF_ADDR */
	u32 host_buf_sz;        /* in byte, size of the host buffer */
	u32 fw_buf_loc;         /* location of the firmware buffer in MIF_ADDR */
	u32 fw_buf_sz;          /* in byte, size of the firmware buffer */
	u32 mib_loc;            /* MIB location in MIF_ADDR */
	u32 mib_sz;             /* MIB size */
	u32 log_config_loc;     /* Logging Configuration Location in MIF_ADDR */
	u32 log_config_sz;      /* Logging Configuration Size in MIF_ADDR */

	u8  mif_fh_int_n;       /* MIF from-host interrupt bit position */
	u8  mif_th_int_n;       /* MIF to-host interrpt bit position */
	u8  reserved[2];

	u32 scbrd_loc;          /* Scoreboard locatin in MIF_ADDR */

	u16 q_num;              /* 6 */
	u16 q_len;              /* 256 */
	u16 q_idx_sz;           /* 1 */
	u8  reserved2[2];

	u32 q_loc[MIF_HIP_CFG_Q_NUM];

	u8  reserved3[16];
} __packed;

struct hip4_hip_init {
	/* Host owned */
	u32 magic_number;       /* 0xcaaa0400 */
	/* FW owned */
	u32 conf_hip4_ver;
	/* Host owned */
	u32 version_a_ref;      /* Location of Config structure A (old) */
	u32 version_b_ref;      /* Location of Config structure B (new) */
} __packed;

#define MAX_NUM 256
struct hip4_hip_q {
	u32 array[MAX_NUM];
	u8  idx_read;      /* To keep track */
	u8  idx_write;     /* To keep track */
	u8  total;
} __aligned(64);

struct hip4_hip_control {
	struct hip4_hip_init             init;
	struct hip4_hip_config_version_3 config_v3 __aligned(32);
	struct hip4_hip_config_version_4 config_v4 __aligned(32);
	u32                              scoreboard[256] __aligned(64);
	struct hip4_hip_q                q[MIF_HIP_CFG_Q_NUM] __aligned(64);
} __aligned(4096);

/* WLAN shared-memory region layout (BSP hip4.c). Total 0x180000. */
#define IMG_MGR_SEC_WLAN_CONFIG_OFFSET  0x00000
#define IMG_MGR_SEC_WLAN_CONFIG_SIZE    0x02000 /* 8 kB*/
#define IMG_MGR_SEC_WLAN_MIB_OFFSET     0x02000
#define IMG_MGR_SEC_WLAN_MIB_SIZE       0x01000 /* 4 kB*/
#define IMG_MGR_SEC_WLAN_TX_OFFSET      0x03000
#define IMG_MGR_SEC_WLAN_TX_SIZE        0x7d000 /* 500 kB*/
#define IMG_MGR_SEC_WLAN_TX_DAT_OFFSET  0x03000 /*   =   */
#define IMG_MGR_SEC_WLAN_TX_DAT_SIZE    0x6d000 /* 436 kB*/
#define IMG_MGR_SEC_WLAN_TX_CTL_OFFSET  0x70000 /*   +   */
#define IMG_MGR_SEC_WLAN_TX_CTL_SIZE    0x10000 /*  64 kB*/
#define IMG_MGR_SEC_WLAN_RX_OFFSET      0x80000
#define IMG_MGR_SEC_WLAN_RX_SIZE        0x100000 /* 1MB */
#define IMG_MGR_SEC_WLAN_SIZE           (IMG_MGR_SEC_WLAN_RX_OFFSET + \
					 IMG_MGR_SEC_WLAN_RX_SIZE)

#endif /* __SCSC_HIP4_H__ */
