/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The seam between the WLBT core (core.c) and the BT driver (bt.c),
 * mirroring wlan.h's split for the same reason: struct scsc_mx stays
 * private to core.c and struct scsc_bt private to bt.c, and everything
 * that crosses does so through the calls below.
 *
 * Unlike WLAN's HIP4/FAPI split, core.c does not decode BSMHCP traffic
 * into a higher-level signal here -- an HCI command/event is already
 * the unit both a hci_dev and the ring deal in, so core.c only moves
 * bytes in and out of the ring and bt.c only wraps them in an skb.
 */

#ifndef __SCSC_BT_H__
#define __SCSC_BT_H__

#include <linux/types.h>

struct device;
struct scsc_mx;
struct scsc_bt;

/* bt.c, called by the core */

/*
 * Register/unregister the hci_dev once the BT service is up. Called
 * from scsc_mx_bt_start() on success and from scsc_mx_remove() (a
 * no-op if never registered), the same timing scsc_wlan_register()/
 * scsc_wlan_unregister() have relative to the WLAN service.
 */
int scsc_bt_register(struct scsc_mx *mx);
void scsc_bt_unregister(struct scsc_mx *mx);

/*
 * HCI's own packet-type octet (see hci.h's HCI_EVENT_PKT/HCI_ACLDATA_PKT)
 * duplicated here rather than pulled in from net/bluetooth/hci.h, the
 * same reasoning wlan.h's SCSC_MSGTYPE_* have for mirroring FAPI's:
 * these are a stable wire constant, not core.c's business to parse BT
 * headers to obtain.
 */
#define SCSC_BT_PKT_EVENT	0x04
#define SCSC_BT_PKT_ACL		0x02

/*
 * One HCI event or ACL data packet, copied out of a ring entry by the
 * core's RX work and delivered as bytes rather than a parsed struct,
 * because that is exactly what both the ring entry and an HCI skb are
 * -- see BSMHCP_TD_HCI_EVT/BSMHCP_TD_ACL_RX in bsmhcp.h. Also how the
 * core delivers its own synthesised Number-Of-Completed-Packets event
 * (SCSC_BT_PKT_EVENT) when acl_tx_free_transfer_ring frees a buffer --
 * see the comment above scsc_mx_bt_work()'s free-list drain.
 */
void scsc_bt_recv(struct scsc_mx *mx, u8 pkt_type, const u8 *data, u16 len);

/* core.c, called by bt.c */

struct scsc_bt *scsc_mx_bt(struct scsc_mx *mx);
void scsc_mx_set_bt(struct scsc_mx *mx, struct scsc_bt *bt);
struct device *scsc_mx_dev(struct scsc_mx *mx);

/*
 * Post one HCI command into hci_cmd_transfer_ring and kick the R4.
 * data/len is the whole command as hci_dev handed it over (opcode,
 * length byte, parameters) -- BSMHCP_TD_CONTROL is a length-prefixed
 * blob, not a parsed command, so nothing here needs to know HCI's own
 * framing. -ENOSPC means the ring is full (BSMHCP_HAS_ROOM says no),
 * -EMSGSIZE means len will never fit (> BSMHCP_CMD_EVT_BUFFER_SIZE).
 */
int scsc_mx_bt_cmd_transfer(struct scsc_mx *mx, const u8 *data, u16 len);

/*
 * Post one HCI ACL data packet into acl_tx_data_transfer_ring and kick
 * the R4's ap_to_fg doorbell. data/len is the whole packet hci_dev
 * handed to ->send(), starting with the 4-byte hci_acl_hdr (handle +
 * PB/BC flags, then length) -- BSMHCP_TD_ACL_TX_DATA wants the handle
 * and flags unpacked into its own fields and a buffer allocated out of
 * the shared acl_tx_buffer pool, but the framing is the same one
 * hci_core already built, not something to reconstruct.
 *
 * -ENOSPC means either the descriptor ring or the 30-slot buffer pool
 * is full; -EMSGSIZE means the payload will never fit a pool buffer;
 * -EINVAL means a malformed/reserved PB flag combination.
 */
int scsc_mx_bt_acl_transfer(struct scsc_mx *mx, const u8 *data, u16 len);

#endif /* __SCSC_BT_H__ */
