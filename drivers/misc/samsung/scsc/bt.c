// SPDX-License-Identifier: GPL-2.0-only
/*
 * BT (BSMHCP) driver for the Samsung SCSC "Maxwell" WLBT chip.
 *
 * core.c owns the transport: scsc_mx_bt_start() brings the firmware
 * service up and scsc_mx_bt_cmd_transfer()/scsc_mx_bt_acl_transfer()/
 * scsc_bt_recv() move bytes across the BSMHCP rings. This file's whole
 * job is to be the Linux-facing half of that, the same relationship
 * wlan.c has to its wiphy -- register a hci_dev, hand outgoing packets
 * to the core, and wrap incoming events/ACL data back into skbs for
 * hci_recv_frame().
 *
 * Deliberately not the vendor's scsc_bt_module.c/scsc_bluez.c split: a
 * BSMHCP ring entry is already a structured descriptor, not a byte
 * stream, so there is nothing here for H4 framing to reassemble --
 * see bt.md for the reasoning.
 *
 * HCI command TX/event RX and ACL TX/RX are wired up; SCO is not --
 * this firmware's SCO ring is zero-sized (BSMHCP_TRANSFER_RING_SCO_SIZE
 * in bsmhcp.h), so hdev->send() has nothing to hand a SCO packet to.
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "bt.h"

struct scsc_bt {
	struct scsc_mx *mx;
	struct hci_dev *hdev;
};

/*
 * The service is already started (scsc_mx_bt_start() ran before this is
 * ever called) and stays up for the module's whole life, so there is
 * nothing left for open/close to do -- hci_dev_open() sending HCI_Reset
 * as its first command is the real proof the ring works, not anything
 * needed here first.
 */
static int scsc_bt_hci_open(struct hci_dev *hdev)
{
	return 0;
}

static int scsc_bt_hci_close(struct hci_dev *hdev)
{
	return 0;
}

static int scsc_bt_hci_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct scsc_bt *bt = hci_get_drvdata(hdev);
	int ret;
	bool is_acl = hci_skb_pkt_type(skb) == HCI_ACLDATA_PKT;

	if (!is_acl && hci_skb_pkt_type(skb) != HCI_COMMAND_PKT) {
		/* SCO: this firmware's SCO ring is zero-sized. */
		kfree_skb(skb);
		return -EILSEQ;
	}

	ret = is_acl ? scsc_mx_bt_acl_transfer(bt->mx, skb->data, skb->len) :
		       scsc_mx_bt_cmd_transfer(bt->mx, skb->data, skb->len);
	if (ret) {
		hdev->stat.err_tx++;
		kfree_skb(skb);
		return ret;
	}

	if (is_acl)
		hdev->stat.acl_tx++;
	else
		hdev->stat.cmd_tx++;
	hdev->stat.byte_tx += skb->len;
	kfree_skb(skb);

	return 0;
}

void scsc_bt_recv(struct scsc_mx *mx, u8 pkt_type, const u8 *data, u16 len)
{
	struct scsc_bt *bt = scsc_mx_bt(mx);
	struct sk_buff *skb;

	if (!bt)
		return;

	skb = bt_skb_alloc(len, GFP_KERNEL);
	if (!skb) {
		bt->hdev->stat.err_rx++;
		return;
	}

	memcpy(skb_put(skb, len), data, len);
	hci_skb_pkt_type(skb) = pkt_type;

	if (pkt_type == HCI_ACLDATA_PKT)
		bt->hdev->stat.acl_rx++;
	else
		bt->hdev->stat.evt_rx++;
	bt->hdev->stat.byte_rx += len;

	hci_recv_frame(bt->hdev, skb);
}

int scsc_bt_register(struct scsc_mx *mx)
{
	struct scsc_bt *bt;
	struct hci_dev *hdev;
	int ret;

	bt = kzalloc(sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;

	hdev = hci_alloc_dev();
	if (!hdev) {
		kfree(bt);
		return -ENOMEM;
	}

	bt->mx = mx;
	bt->hdev = hdev;

	/*
	 * No standard bus fits a shared-DRAM/doorbell link to a companion
	 * core exactly; HCI_SMD is the closest existing match, used by the
	 * other in-tree driver with the same shape (Qualcomm's shared
	 * memory transport, btqcomsmd.c).
	 */
	hdev->bus = HCI_SMD;
	hdev->open = scsc_bt_hci_open;
	hdev->close = scsc_bt_hci_close;
	hdev->send = scsc_bt_hci_send;

	hci_set_drvdata(hdev, bt);
	SET_HCIDEV_DEV(hdev, scsc_mx_dev(mx));

	ret = hci_register_dev(hdev);
	if (ret) {
		hci_free_dev(hdev);
		kfree(bt);
		return ret;
	}

	scsc_mx_set_bt(mx, bt);

	dev_info(scsc_mx_dev(mx), "bt: registered %s\n", hdev->name);

	return 0;
}

void scsc_bt_unregister(struct scsc_mx *mx)
{
	struct scsc_bt *bt = scsc_mx_bt(mx);

	if (!bt)
		return;

	hci_unregister_dev(bt->hdev);
	hci_free_dev(bt->hdev);
	scsc_mx_set_bt(mx, NULL);
	kfree(bt);
}
