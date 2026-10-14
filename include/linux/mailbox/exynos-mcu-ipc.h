/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung Exynos MCU_IPC mailbox: shared registers, as seen by clients
 * through the syscon regmap the mailbox driver registers for its node.
 */

#ifndef _LINUX_MAILBOX_EXYNOS_MCU_IPC_H_
#define _LINUX_MAILBOX_EXYNOS_MCU_IPC_H_

/*
 * 64 shared 32-bit registers (ISSR0..63). Writing one does not raise an
 * interrupt; ring a doorbell through the mailbox channel for that.
 */
#define EXYNOS_MCU_IPC_ISSR(n)		(0x80 + 4 * (n))
#define EXYNOS_MCU_IPC_NUM_ISSR		64

#endif /* _LINUX_MAILBOX_EXYNOS_MCU_IPC_H_ */
