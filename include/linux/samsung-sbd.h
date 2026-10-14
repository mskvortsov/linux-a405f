/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung SBD link: the shared-memory IPC link between the AP and a
 * Shannon modem (CP), driven by the modem's remoteproc driver.
 */

#ifndef _LINUX_SAMSUNG_SBD_H_
#define _LINUX_SAMSUNG_SBD_H_

#include <linux/types.h>

struct device;
struct sbd_link;

/* AP <-> CP messages, in the mailbox message registers */
#define SBD_MSG_INT_VALID	BIT(7)
#define SBD_MSG_CMD_VALID	BIT(6)
#define SBD_MSG_CMD		GENMASK(5, 0)
#define SBD_MSG(cmd)		(SBD_MSG_INT_VALID | SBD_MSG_CMD_VALID | (cmd))

#define SBD_CMD_INIT_START	0x01
#define SBD_CMD_INIT_END	0x02
#define SBD_CMD_CRASH_RESET	0x07
#define SBD_CMD_PHONE_START	0x08
#define SBD_CMD_CRASH_EXIT	0x09
#define SBD_CMD_PIF_INIT_DONE	0x0d

/**
 * struct sbd_link_ops - what the link needs from the modem driver
 * @send_msg: write @msg to the AP->CP message register and ring the
 *	      message doorbell. Called in process context, may sleep.
 */
struct sbd_link_ops {
	void (*send_msg)(void *ctx, u32 msg);
};

struct sbd_link *sbd_link_create(struct device *dev, void *ipc,
				 size_t ipc_size,
				 const struct sbd_link_ops *ops, void *ctx);
void sbd_link_destroy(struct sbd_link *link);

int sbd_link_start(struct sbd_link *link);
void sbd_link_stop(struct sbd_link *link);

void sbd_link_notify(struct sbd_link *link, u32 msg);

#endif /* _LINUX_SAMSUNG_SBD_H_ */
