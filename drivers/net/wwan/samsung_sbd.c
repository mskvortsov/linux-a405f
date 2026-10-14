// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung SBD link: the shared-memory IPC link to a Shannon modem (CP).
 *
 * Port of the AP side of downstream modem_v1's SBD link
 * (link_device_memory_sbd.c, include/sbd.h, include/sipc5.h), reduced to
 * what the modem needs from a plain AP:
 *
 *  - ring layout: the AP describes every link channel in the descriptor
 *    region of the IPC memory -- one uplink and one downlink ring of
 *    fixed-size slots per channel, plus the read/write pointers -- and
 *    then tells the CP with PIF_INIT_DONE. The CP answers PHONE_START;
 *    the AP resets the IPC magic and acknowledges with INIT_END.
 *  - data: each slot holds one SIPC5 frame, {cfg, ch, u16 len} + payload.
 *    Messages longer than a slot travel as a run of multi-frame frames
 *    with a fifth header byte, {more:1, id:7}. The CP signals new
 *    downlink frames with a data message on the message doorbell; the AP
 *    signals new uplink frames the same way.
 *
 * Each channel with a userspace protocol is a WWAN port, one message per
 * read and per write: the SIPC5 framing stays in the kernel. A channel's
 * downlink ring is only drained while its port is open, so the shared
 * ring itself buffers what the CP sends before a daemon opens the port.
 *
 * Packet data (downstream's rmnet0-7 over multipdp / multipdp_hiprio) is
 * different: a slot holds one raw IP packet with no SIPC5 header
 * (ATTR_NO_LINK_HEADER in the stock DT), and the slot's size word carries
 * the PDP channel in bits 16-23, 0x0a + n for rmnetN. Those become WWAN
 * network links, link id n for channel 0x0a + n; libsamsung-ipc pairs
 * rmnetN with PDP context n + 1. Both data rings are always drained;
 * uplink goes to the normal-priority ring.
 *
 * The AP owns downlink rp and uplink wp, the CP downlink wp and uplink
 * rp. The IPC memory is mapped write-combine by the modem driver.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/if_arp.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/rcupdate.h>
#include <linux/rtnetlink.h>
#include <linux/samsung-sbd.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/wwan.h>

/* IPC region layout */
#define IPC_MAGIC		0x00
#define IPC_ACCESS		0x04
#define IPC_LEGACY_START	0x08	/* legacy queue pointers, up to 0x28 */
#define IPC_LEGACY_END		0x28
#define IPC_MAGIC_LINK		0xaa

#define DESC_RGN_OFFSET		0x800
#define DESC_RGN_END		0x10000
#define BUFF_RGN_OFFSET		0x10000

#define SBD_VERSION		1
#define SBD_MAX_CHANNELS	32
#define SBD_UL			0
#define SBD_DL			1

/* data messages: MASK_INT_VALID | MASK_SEND_DATA */
#define SBD_MSG_SEND_DATA	(SBD_MSG_INT_VALID | BIT(0))
#define SBD_MSG_REQ_ACK		(BIT(5) | BIT(4))

/* SIPC5 frame header */
#define SIPC5_START_MASK	0xf8
#define SIPC5_PADDING		BIT(2)
#define SIPC5_EXT_MASK		0x03
#define SIPC5_MULTI_FRAME	0x03
#define SIPC5_EXT_LEN		0x02
#define SIPC5_HDR_LEN		4
#define SIPC5_HDR_LEN_CTRL	5
#define SIPC5_CFG_SINGLE	0xf8
#define SIPC5_CFG_MULTI		(0xf8 | SIPC5_MULTI_FRAME)
#define SIPC5_CTRL_MORE		BIT(7)
#define SIPC5_CTRL_ID		GENMASK(6, 0)

/* packet data: rmnet0-7 are PDP channels 0x0a-0x11 */
#define SBD_PS_CH_FIRST		0x0a
#define SBD_PS_LINKS		8
#define SBD_PS_SIZE_LEN		GENMASK(15, 0)
#define SBD_PS_SIZE_CH		GENMASK(23, 16)
#define SBD_PS_TX_RETRY_MS	10

/* shared descriptors, include/sbd.h downstream */
struct sbd_rb_desc {
	__le16 ch;
	__le16 reserved;
	__le16 direction;
	__le16 signaling;	/* 1: interrupt */
	__le32 sig_mask;
	__le16 length;
	__le16 id;
	__le16 buff_size;
	__le16 payload_offset;
} __packed;

struct sbd_rb_channel {
	__le32 ul_rbd_offset;
	__le32 ul_sbdv_offset;
	__le32 dl_rbd_offset;
	__le32 dl_sbdv_offset;
} __packed;

struct sbd_global_desc {
	__le32 version;
	__le32 num_channels;
	__le32 rbps_offset;
	struct sbd_rb_channel rb_ch[SBD_MAX_CHANNELS];
	struct sbd_rb_desc rb_desc[SBD_MAX_CHANNELS][2];
} __packed;

/*
 * The link channels, in link id order. From the stock Galaxy A40 device
 * tree (dtbo, "Samsung A40 EUR OPEN"): its iodevs in DT order, keeping the
 * IPC channels that are not packet-switched plus the multi-raw ones.
 */
struct sbd_chan_info {
	const char *name;
	u8 ch;
	u16 ul_len, ul_size;
	u16 dl_len, dl_size;
	int port_type;		/* -1: no port */
	bool ps;		/* packet data, see the top of the file */
};

static const struct sbd_chan_info sbd_channels[] = {
	{ "umts_cass",       0x23,  16, SZ_2K,   16, SZ_2K, -1 },
	{ "umts_ipc0",       0xeb,  16, SZ_4K,   32, SZ_4K, WWAN_PORT_SIPC },
	{ "umts_ipc1",       0xec,  16, SZ_4K,   32, SZ_4K, WWAN_PORT_SIPC },
	{ "umts_rfs0",       0xf5,  16, SZ_2K,  512, SZ_2K, WWAN_PORT_SIPC_RFS },
	{ "umts_csd",        0x01,  32, SZ_2K,   64, SZ_2K, -1 },
	{ "umts_router",     0x19,  16, SZ_2K,   16, SZ_2K, -1 },
	{ "umts_dm0",        0x1c,  16, SZ_2K,  128, SZ_2K, -1 },
	{ "multipdp_hiprio", 0x0a, 256, SZ_2K,  256, SZ_2K, -1, true },
	{ "multipdp",        0x0b, 512, SZ_2K, 1024, SZ_2K, -1, true },
};

/* the uplink ring for packet data, downstream's QOS_NORMAL */
#define SBD_PS_TX_CHAN		8

#define SBD_NUM_CHANNELS	ARRAY_SIZE(sbd_channels)

struct sbd_ring {
	u32 addr_v;		/* slot offsets: u32[len] */
	u32 size_v;		/* slot frame lengths: u32[len] */
	u32 rp, wp;		/* offsets of the u16 pointers */
	u16 len;
	u16 size;
};

struct sbd_chan {
	struct sbd_link *link;
	const struct sbd_chan_info *info;
	struct sbd_ring ring[2];
	struct wwan_port *port;
	bool open;

	/* multi-frame reassembly */
	struct sk_buff_head frags;
	u8 frag_id;

	u8 tx_id;
};

struct sbd_link {
	struct device *dev;
	void *ipc;
	size_t ipc_size;
	const struct sbd_link_ops *ops;
	void *ctx;

	struct sbd_chan chans[SBD_NUM_CHANNELS];

	/*
	 * active: layout written, the rings are valid. online: PHONE_START
	 * acknowledged. Plain flags, read and written with READ_ONCE() /
	 * WRITE_ONCE(): sbd_link_notify() runs in hard IRQ context.
	 */
	bool active;
	bool online;

	struct work_struct rx_work;
	struct work_struct ctrl_work;
	unsigned long ctrl_pending;
#define SBD_CTRL_PHONE_START	0

	wait_queue_head_t tx_wq;

	/* packet data links, by link id; RCU for the RX path */
	struct net_device __rcu *netdevs[SBD_PS_LINKS];
	spinlock_t ps_tx_lock;		/* the uplink ring, across links */
	struct work_struct ps_kick_work;	/* uplink doorbell, may sleep */
	struct delayed_work ps_wake_work;	/* uplink ring full: retry */
	bool wwan_ops;
};

struct sbd_netdev {
	struct sbd_link *link;
	u32 if_id;
};

static u16 sbd_r16(struct sbd_link *link, u32 off)
{
	return le16_to_cpu(READ_ONCE(*(__le16 *)(link->ipc + off)));
}

static void sbd_w16(struct sbd_link *link, u32 off, u16 val)
{
	WRITE_ONCE(*(__le16 *)(link->ipc + off), cpu_to_le16(val));
}

static u32 sbd_r32(struct sbd_link *link, u32 off)
{
	return le32_to_cpu(READ_ONCE(*(__le32 *)(link->ipc + off)));
}

static void sbd_w32(struct sbd_link *link, u32 off, u32 val)
{
	WRITE_ONCE(*(__le32 *)(link->ipc + off), cpu_to_le32(val));
}

static bool sbd_link_is_active(struct sbd_link *link)
{
	return READ_ONCE(link->active);
}

/*
 * init_sbd_link() downstream: every ring's descriptor, its slot vector
 * and the read/write pointer set, in the descriptor region; slot buffers
 * from the buffer region on. The offsets only depend on the channel
 * table, so they are computed once, at create time.
 */
static int sbd_link_plan(struct sbd_link *link)
{
	u32 desc = DESC_RGN_OFFSET + sizeof(struct sbd_global_desc);
	u32 rbps = desc, buff = BUFF_RGN_OFFSET;
	int i, d;

	desc += 2 * 2 * sizeof(u16) * SBD_NUM_CHANNELS;

	for (i = 0; i < SBD_NUM_CHANNELS; i++) {
		struct sbd_chan *chan = &link->chans[i];
		const struct sbd_chan_info *info = &sbd_channels[i];

		for (d = SBD_UL; d <= SBD_DL; d++) {
			struct sbd_ring *ring = &chan->ring[d];

			ring->len = d == SBD_UL ? info->ul_len : info->dl_len;
			ring->size = d == SBD_UL ? info->ul_size : info->dl_size;
			ring->addr_v = desc;
			desc += ring->len * sizeof(u32);
			ring->size_v = desc;
			desc += ring->len * sizeof(u32);
			/* rp[UL][n], wp[UL][n], rp[DL][n], wp[DL][n] */
			ring->rp = rbps + sizeof(u16) *
				   (SBD_NUM_CHANNELS * 2 * d + i);
			ring->wp = rbps + sizeof(u16) *
				   (SBD_NUM_CHANNELS * (2 * d + 1) + i);
			buff += ring->len * ring->size;
		}
	}

	if (desc > DESC_RGN_END || buff > link->ipc_size) {
		dev_err(link->dev, "SBD layout does not fit: desc 0x%x buff 0x%x\n",
			desc, buff);
		return -ENOSPC;
	}

	return 0;
}

static void sbd_link_write_layout(struct sbd_link *link)
{
	u32 rbps = DESC_RGN_OFFSET + sizeof(struct sbd_global_desc);
	struct sbd_global_desc *g = link->ipc + DESC_RGN_OFFSET;
	u32 buff = BUFF_RGN_OFFSET;
	int i, d, j;

	memset(link->ipc + DESC_RGN_OFFSET, 0, DESC_RGN_END - DESC_RGN_OFFSET);

	g->version = cpu_to_le32(SBD_VERSION);
	g->num_channels = cpu_to_le32(SBD_NUM_CHANNELS);
	g->rbps_offset = cpu_to_le32(rbps);

	for (i = 0; i < SBD_NUM_CHANNELS; i++) {
		struct sbd_chan *chan = &link->chans[i];
		u32 rbd[2];

		for (d = SBD_UL; d <= SBD_DL; d++) {
			struct sbd_ring *ring = &chan->ring[d];
			struct sbd_rb_desc *rb = &g->rb_desc[i][d];

			for (j = 0; j < ring->len; j++)
				sbd_w32(link, ring->addr_v + 4 * j,
					buff + j * ring->size);
			buff += ring->len * ring->size;

			rb->ch = cpu_to_le16(chan->info->ch);
			rb->direction = cpu_to_le16(d);
			rb->signaling = cpu_to_le16(1);
			rb->sig_mask = cpu_to_le32(SBD_MSG_SEND_DATA);
			rb->length = cpu_to_le16(ring->len);
			rb->id = cpu_to_le16(i);
			rb->buff_size = cpu_to_le16(ring->size);

			rbd[d] = (void *)rb - link->ipc;
		}

		g->rb_ch[i].ul_rbd_offset = cpu_to_le32(rbd[SBD_UL]);
		g->rb_ch[i].ul_sbdv_offset = cpu_to_le32(chan->ring[SBD_UL].addr_v);
		g->rb_ch[i].dl_rbd_offset = cpu_to_le32(rbd[SBD_DL]);
		g->rb_ch[i].dl_sbdv_offset = cpu_to_le32(chan->ring[SBD_DL].addr_v);
	}

	wmb();	/* the whole layout before PIF_INIT_DONE */
}

/* -> payload of one downlink frame, or an error for a malformed slot */
static int sbd_rx_frame(struct sbd_chan *chan, u32 addr, u32 size)
{
	struct sbd_link *link = chan->link;
	const u8 *frame = link->ipc + addr;
	unsigned int hdr_len = SIPC5_HDR_LEN, len;
	struct sk_buff *skb, *frag;
	u8 cfg = frame[0], ctrl = 0;
	bool multi = false;

	if (size < SIPC5_HDR_LEN || (cfg & SIPC5_START_MASK) != SIPC5_START_MASK)
		return -EPROTO;

	switch (cfg & SIPC5_EXT_MASK) {
	case SIPC5_MULTI_FRAME:
		hdr_len = SIPC5_HDR_LEN_CTRL;
		ctrl = frame[4];
		multi = true;
		break;
	case SIPC5_EXT_LEN:
		/* a frame never outgrows a 2-4 KB slot */
		return -EPROTO;
	}

	len = get_unaligned_le16(frame + 2);
	if (len < hdr_len || len > size)
		return -EPROTO;
	len -= hdr_len;

	skb = alloc_skb(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;
	skb_put_data(skb, frame + hdr_len, len);

	if (!multi && skb_queue_empty(&chan->frags)) {
		wwan_port_rx(chan->port, skb);
		return 0;
	}

	if (!skb_queue_empty(&chan->frags) &&
	    (!multi || (ctrl & SIPC5_CTRL_ID) != chan->frag_id)) {
		dev_warn(link->dev, "%s: multi-frame %u cut short\n",
			 chan->info->name, chan->frag_id);
		skb_queue_purge(&chan->frags);
	}

	if (!multi) {
		wwan_port_rx(chan->port, skb);
		return 0;
	}

	chan->frag_id = ctrl & SIPC5_CTRL_ID;
	skb_queue_tail(&chan->frags, skb);
	if (ctrl & SIPC5_CTRL_MORE)
		return 0;

	/* last piece: hand the whole message over as one skb */
	len = 0;
	skb_queue_walk(&chan->frags, frag)
		len += frag->len;

	skb = alloc_skb(len, GFP_KERNEL);
	if (!skb) {
		skb_queue_purge(&chan->frags);
		return -ENOMEM;
	}
	while ((frag = __skb_dequeue(&chan->frags))) {
		skb_put_data(skb, frag->data, frag->len);
		consume_skb(frag);
	}

	wwan_port_rx(chan->port, skb);

	return 0;
}

static void sbd_rx_chan(struct sbd_chan *chan)
{
	struct sbd_link *link = chan->link;
	struct sbd_ring *ring = &chan->ring[SBD_DL];
	u16 rp, wp;

	for (;;) {
		u32 addr, size;
		int ret;

		rp = sbd_r16(link, ring->rp);
		wp = sbd_r16(link, ring->wp);
		if (rp == wp)
			return;

		if (rp >= ring->len || wp >= ring->len) {
			dev_err_ratelimited(link->dev, "%s: DL rp/wp %u/%u out of range\n",
					    chan->info->name, rp, wp);
			return;
		}

		rmb();	/* wp before the slot it covers */
		addr = sbd_r32(link, ring->addr_v + 4 * rp);
		size = sbd_r32(link, ring->size_v + 4 * rp) & 0xffff;
		if (size > ring->size || addr + size > link->ipc_size) {
			dev_err_ratelimited(link->dev, "%s: DL slot %u: 0x%x+0x%x\n",
					    chan->info->name, rp, addr, size);
			ret = -EPROTO;
		} else {
			ret = sbd_rx_frame(chan, addr, size);
		}
		if (ret == -ENOMEM)
			return;		/* leave it in the ring, retry later */
		if (ret)
			dev_err_ratelimited(link->dev, "%s: dropped bad frame in slot %u\n",
					    chan->info->name, rp);

		mb();	/* done with the slot before handing it back */
		sbd_w16(link, ring->rp, (rp + 1) % ring->len);
	}
}

/* one downlink packet: a raw IP packet for the link its channel names */
static void sbd_rx_packet(struct sbd_link *link, u32 addr, u32 word)
{
	const u8 *data = link->ipc + addr;
	unsigned int len = FIELD_GET(SBD_PS_SIZE_LEN, word);
	unsigned int id = FIELD_GET(SBD_PS_SIZE_CH, word) - SBD_PS_CH_FIRST;
	struct net_device *ndev;
	struct sk_buff *skb;
	__be16 proto;

	if (!len)
		return;

	switch (data[0] >> 4) {
	case 4:
		proto = htons(ETH_P_IP);
		break;
	case 6:
		proto = htons(ETH_P_IPV6);
		break;
	default:
		proto = 0;
	}

	rcu_read_lock();
	ndev = id < SBD_PS_LINKS ? rcu_dereference(link->netdevs[id]) : NULL;
	if (!ndev) {
		dev_dbg_ratelimited(link->dev, "packet for PDP channel 0x%02lx, no link\n",
				    FIELD_GET(SBD_PS_SIZE_CH, word));
		goto out;
	}
	if (!proto) {
		DEV_STATS_INC(ndev, rx_errors);
		goto out;
	}

	skb = netdev_alloc_skb(ndev, len);
	if (!skb) {
		DEV_STATS_INC(ndev, rx_dropped);
		goto out;
	}
	skb_put_data(skb, data, len);
	skb->protocol = proto;
	skb_reset_network_header(skb);

	dev_sw_netstats_rx_add(ndev, len);
	netif_rx(skb);
out:
	rcu_read_unlock();
}

static void sbd_rx_ps(struct sbd_chan *chan)
{
	struct sbd_link *link = chan->link;
	struct sbd_ring *ring = &chan->ring[SBD_DL];
	u16 rp, wp;

	for (;;) {
		u32 addr, word;

		rp = sbd_r16(link, ring->rp);
		wp = sbd_r16(link, ring->wp);
		if (rp == wp)
			return;

		if (rp >= ring->len || wp >= ring->len) {
			dev_err_ratelimited(link->dev, "%s: DL rp/wp %u/%u out of range\n",
					    chan->info->name, rp, wp);
			return;
		}

		rmb();	/* wp before the slot it covers */
		addr = sbd_r32(link, ring->addr_v + 4 * rp);
		word = sbd_r32(link, ring->size_v + 4 * rp);
		if (FIELD_GET(SBD_PS_SIZE_LEN, word) > ring->size ||
		    addr + ring->size > link->ipc_size)
			dev_err_ratelimited(link->dev, "%s: DL slot %u: 0x%x size 0x%x\n",
					    chan->info->name, rp, addr, word);
		else
			sbd_rx_packet(link, addr, word);

		mb();	/* done with the slot before handing it back */
		sbd_w16(link, ring->rp, (rp + 1) % ring->len);
	}
}

static void sbd_rx_work(struct work_struct *work)
{
	struct sbd_link *link = container_of(work, struct sbd_link, rx_work);
	int i;

	for (i = 0; i < SBD_NUM_CHANNELS; i++) {
		struct sbd_chan *chan = &link->chans[i];

		if (!READ_ONCE(link->active))
			break;
		if (chan->info->ps)
			sbd_rx_ps(chan);
		else if (READ_ONCE(chan->open))
			sbd_rx_chan(chan);
	}

	wmb();	/* drain the rp updates */
}

/*
 * PHONE_START: shmem_reset_ipc_link() -- legacy queues reset, magic and
 * access set for the running link -- then INIT_END.
 */
static void sbd_ctrl_work(struct work_struct *work)
{
	struct sbd_link *link = container_of(work, struct sbd_link, ctrl_work);
	u32 off;

	if (!test_and_clear_bit(SBD_CTRL_PHONE_START, &link->ctrl_pending))
		return;
	if (!sbd_link_is_active(link))
		return;

	sbd_w32(link, IPC_ACCESS, 0);
	sbd_w32(link, IPC_MAGIC, 0);
	for (off = IPC_LEGACY_START; off < IPC_LEGACY_END; off += 4)
		sbd_w32(link, off, 0);
	sbd_w32(link, IPC_MAGIC, IPC_MAGIC_LINK);
	sbd_w32(link, IPC_ACCESS, 1);
	wmb();

	link->ops->send_msg(link->ctx, SBD_MSG(SBD_CMD_INIT_END));

	WRITE_ONCE(link->online, true);

	dev_info(link->dev, "SBD link online\n");

	/* anything that arrived before the ack */
	queue_work(system_highpri_wq, &link->rx_work);
}

/**
 * sbd_link_notify() - feed the link a CP->AP message
 * @link: the link
 * @msg: the value of the CP->AP message register
 *
 * Call for every message doorbell. Atomic context is fine.
 */
void sbd_link_notify(struct sbd_link *link, u32 msg)
{
	if (!(msg & SBD_MSG_INT_VALID))
		return;

	if (msg & SBD_MSG_CMD_VALID) {
		switch (msg & SBD_MSG_CMD) {
		case SBD_CMD_PHONE_START:
			set_bit(SBD_CTRL_PHONE_START, &link->ctrl_pending);
			queue_work(system_highpri_wq, &link->ctrl_work);
			break;
		case SBD_CMD_CRASH_RESET:
		case SBD_CMD_CRASH_EXIT:
			WRITE_ONCE(link->active, false);
			WRITE_ONCE(link->online, false);
			wake_up(&link->tx_wq);
			break;
		}
		return;
	}

	if (msg & SBD_MSG_REQ_ACK)
		dev_warn_once(link->dev, "CP asks for an ack (0x%02x), not implemented\n",
			      msg);

	queue_work(system_highpri_wq, &link->rx_work);
	wake_up(&link->tx_wq);
}
EXPORT_SYMBOL_GPL(sbd_link_notify);

static u16 sbd_ul_free(struct sbd_chan *chan)
{
	struct sbd_ring *ring = &chan->ring[SBD_UL];
	u16 rp = sbd_r16(chan->link, ring->rp);
	u16 wp = sbd_r16(chan->link, ring->wp);

	if (rp >= ring->len || wp >= ring->len)
		return 0;

	return (rp + ring->len - wp - 1) % ring->len;
}

static unsigned int sbd_frames_needed(struct sbd_ring *ring, unsigned int len)
{
	if (len + SIPC5_HDR_LEN <= ring->size)
		return 1;

	return DIV_ROUND_UP(len, ring->size - SIPC5_HDR_LEN_CTRL);
}

/*
 * Queue up to @nframes frames of @skb from @off on, as many as the ring
 * has room for; -> bytes of payload queued.
 */
static unsigned int sbd_tx_frames(struct sbd_chan *chan, struct sk_buff *skb,
				  unsigned int off, bool multi)
{
	struct sbd_link *link = chan->link;
	struct sbd_ring *ring = &chan->ring[SBD_UL];
	unsigned int hdr_len = multi ? SIPC5_HDR_LEN_CTRL : SIPC5_HDR_LEN;
	unsigned int start = off;
	u16 wp = sbd_r16(link, ring->wp);
	u16 free = sbd_ul_free(chan);

	while (free-- && off < skb->len) {
		unsigned int chunk = min(skb->len - off, ring->size - hdr_len);
		u32 addr = sbd_r32(link, ring->addr_v + 4 * wp);
		u8 *frame = link->ipc + addr;

		if (addr + ring->size > link->ipc_size)
			break;

		frame[0] = multi ? SIPC5_CFG_MULTI : SIPC5_CFG_SINGLE;
		frame[1] = chan->info->ch;
		put_unaligned_le16(hdr_len + chunk, frame + 2);
		if (multi)
			frame[4] = (off + chunk < skb->len ? SIPC5_CTRL_MORE : 0) |
				   chan->tx_id;
		skb_copy_bits(skb, off, frame + hdr_len, chunk);
		sbd_w32(link, ring->size_v + 4 * wp, hdr_len + chunk);

		off += chunk;
		wp = (wp + 1) % ring->len;
	}

	if (off != start) {
		wmb();	/* frames before wp */
		sbd_w16(link, ring->wp, wp);
		wmb();
		link->ops->send_msg(link->ctx, SBD_MSG_SEND_DATA);
	}

	return off - start;
}

static int sbd_port_tx_common(struct wwan_port *port, struct sk_buff *skb,
			      bool block)
{
	struct sbd_chan *chan = wwan_port_get_drvdata(port);
	struct sbd_link *link = chan->link;
	struct sbd_ring *ring = &chan->ring[SBD_UL];
	unsigned int nframes = sbd_frames_needed(ring, skb->len);
	bool multi = nframes > 1;
	unsigned int off = 0;
	int ret;

	if (!sbd_link_is_active(link))
		return -ENETDOWN;

	/* without blocking, a message goes out whole or not at all */
	if (!block && nframes > sbd_ul_free(chan))
		return nframes >= ring->len ? -EMSGSIZE : -EAGAIN;

	if (multi)
		chan->tx_id = (chan->tx_id + 1) & SIPC5_CTRL_ID;

	while (off < skb->len) {
		off += sbd_tx_frames(chan, skb, off, multi);
		if (off == skb->len)
			break;

		/*
		 * The CP does not signal uplink consumption by itself, so
		 * poll for room as well as waiting for its next message.
		 */
		ret = wait_event_interruptible_timeout(link->tx_wq,
						       sbd_ul_free(chan) ||
						       !sbd_link_is_active(link),
						       msecs_to_jiffies(10));
		if (ret < 0)
			return ret;	/* the CP got a partial message */
		if (!sbd_link_is_active(link))
			return -ENETDOWN;
	}

	consume_skb(skb);

	return 0;
}

static int sbd_port_tx(struct wwan_port *port, struct sk_buff *skb)
{
	return sbd_port_tx_common(port, skb, false);
}

static int sbd_port_tx_blocking(struct wwan_port *port, struct sk_buff *skb)
{
	return sbd_port_tx_common(port, skb, true);
}

static int sbd_port_start(struct wwan_port *port)
{
	struct sbd_chan *chan = wwan_port_get_drvdata(port);
	struct sbd_link *link = chan->link;

	WRITE_ONCE(chan->open, true);

	/* deliver what queued up in the ring while nobody listened */
	queue_work(system_highpri_wq, &link->rx_work);

	return 0;
}

static void sbd_port_stop(struct wwan_port *port)
{
	struct sbd_chan *chan = wwan_port_get_drvdata(port);
	struct sbd_link *link = chan->link;

	WRITE_ONCE(chan->open, false);

	flush_work(&link->rx_work);
	skb_queue_purge(&chan->frags);
}

static const struct wwan_port_ops sbd_port_ops = {
	.start = sbd_port_start,
	.stop = sbd_port_stop,
	.tx = sbd_port_tx,
	.tx_blocking = sbd_port_tx_blocking,
};

/* ---- packet data: WWAN network links ---- */

static void sbd_ps_kick_work(struct work_struct *work)
{
	struct sbd_link *link = container_of(work, struct sbd_link,
					     ps_kick_work);

	if (sbd_link_is_active(link))
		link->ops->send_msg(link->ctx, SBD_MSG_SEND_DATA);
}

static void sbd_ps_wake_queues(struct sbd_link *link)
{
	struct net_device *ndev;
	int i;

	rcu_read_lock();
	for (i = 0; i < SBD_PS_LINKS; i++) {
		ndev = rcu_dereference(link->netdevs[i]);
		if (ndev && netif_running(ndev))
			netif_wake_queue(ndev);
	}
	rcu_read_unlock();
}

/*
 * The CP does not signal uplink consumption, so a full ring is retried
 * on a timer; the queues are woken once there is room again.
 */
static void sbd_ps_wake_work(struct work_struct *work)
{
	struct sbd_link *link = container_of(to_delayed_work(work),
					     struct sbd_link, ps_wake_work);
	struct sbd_chan *chan = &link->chans[SBD_PS_TX_CHAN];

	if (!sbd_link_is_active(link))
		return;

	if (sbd_ul_free(chan))
		sbd_ps_wake_queues(link);
	else
		schedule_delayed_work(&link->ps_wake_work,
				      msecs_to_jiffies(SBD_PS_TX_RETRY_MS));
}

static netdev_tx_t sbd_ndo_start_xmit(struct sk_buff *skb,
				      struct net_device *ndev)
{
	struct sbd_netdev *priv = wwan_netdev_drvpriv(ndev);
	struct sbd_link *link = priv->link;
	struct sbd_chan *chan = &link->chans[SBD_PS_TX_CHAN];
	struct sbd_ring *ring = &chan->ring[SBD_UL];
	unsigned int len = skb->len;
	u32 addr;
	u16 wp;

	if (!sbd_link_is_active(link) || len > ring->size) {
		DEV_STATS_INC(ndev, tx_dropped);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	spin_lock(&link->ps_tx_lock);

	if (!sbd_ul_free(chan)) {
		spin_unlock(&link->ps_tx_lock);
		netif_stop_queue(ndev);
		schedule_delayed_work(&link->ps_wake_work,
				      msecs_to_jiffies(SBD_PS_TX_RETRY_MS));
		return NETDEV_TX_BUSY;
	}

	wp = sbd_r16(link, ring->wp);
	addr = sbd_r32(link, ring->addr_v + 4 * wp);
	if (addr + ring->size > link->ipc_size) {
		spin_unlock(&link->ps_tx_lock);
		DEV_STATS_INC(ndev, tx_errors);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	skb_copy_bits(skb, 0, link->ipc + addr, len);
	sbd_w32(link, ring->size_v + 4 * wp,
		FIELD_PREP(SBD_PS_SIZE_LEN, len) |
		FIELD_PREP(SBD_PS_SIZE_CH, SBD_PS_CH_FIRST + priv->if_id));
	wmb();	/* the packet before wp */
	sbd_w16(link, ring->wp, (wp + 1) % ring->len);

	spin_unlock(&link->ps_tx_lock);

	dev_sw_netstats_tx_add(ndev, 1, len);
	dev_consume_skb_any(skb);

	queue_work(system_highpri_wq, &link->ps_kick_work);

	return NETDEV_TX_OK;
}

static int sbd_ndo_open(struct net_device *ndev)
{
	netif_start_queue(ndev);

	return 0;
}

static int sbd_ndo_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);

	return 0;
}

static const struct net_device_ops sbd_netdev_ops = {
	.ndo_open = sbd_ndo_open,
	.ndo_stop = sbd_ndo_stop,
	.ndo_start_xmit = sbd_ndo_start_xmit,
};

static void sbd_netdev_setup(struct net_device *ndev)
{
	ndev->netdev_ops = &sbd_netdev_ops;
	ndev->type = ARPHRD_RAWIP;
	ndev->flags = IFF_POINTOPOINT | IFF_NOARP;
	ndev->hard_header_len = 0;
	ndev->addr_len = 0;
	ndev->mtu = ETH_DATA_LEN;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = SZ_2K;		/* one slot, no header */
	ndev->tx_queue_len = 1000;
	ndev->needs_free_netdev = true;
	ndev->pcpu_stat_type = NETDEV_PCPU_STAT_TSTATS;
}

static int sbd_wwan_newlink(void *ctxt, struct net_device *ndev, u32 if_id,
			    struct netlink_ext_ack *extack)
{
	struct sbd_netdev *priv = wwan_netdev_drvpriv(ndev);
	struct sbd_link *link = ctxt;
	int ret;

	if (if_id >= SBD_PS_LINKS) {
		NL_SET_ERR_MSG_MOD(extack, "link id 0-7 (rmnet0-7)");
		return -EINVAL;
	}
	if (rtnl_dereference(link->netdevs[if_id]))
		return -EBUSY;

	priv->link = link;
	priv->if_id = if_id;

	ret = register_netdevice(ndev);
	if (ret)
		return ret;

	rcu_assign_pointer(link->netdevs[if_id], ndev);

	return 0;
}

static void sbd_wwan_dellink(void *ctxt, struct net_device *ndev,
			     struct list_head *head)
{
	struct sbd_netdev *priv = wwan_netdev_drvpriv(ndev);
	struct sbd_link *link = ctxt;

	RCU_INIT_POINTER(link->netdevs[priv->if_id], NULL);
	unregister_netdevice_queue(ndev, head);
}

static const struct wwan_ops sbd_wwan_ops = {
	.priv_size = sizeof(struct sbd_netdev),
	.setup = sbd_netdev_setup,
	.newlink = sbd_wwan_newlink,
	.dellink = sbd_wwan_dellink,
};

/**
 * sbd_link_start() - bring the link up
 * @link: the link
 *
 * Write the ring layout and send PIF_INIT_DONE. The CP answers with
 * PHONE_START, which sbd_link_notify() acknowledges.
 */
int sbd_link_start(struct sbd_link *link)
{
	sbd_link_write_layout(link);

	WRITE_ONCE(link->online, false);
	WRITE_ONCE(link->active, true);

	link->ops->send_msg(link->ctx, SBD_MSG(SBD_CMD_PIF_INIT_DONE));

	return 0;
}
EXPORT_SYMBOL_GPL(sbd_link_start);

void sbd_link_stop(struct sbd_link *link)
{
	WRITE_ONCE(link->active, false);
	WRITE_ONCE(link->online, false);

	wake_up(&link->tx_wq);
	cancel_work_sync(&link->ctrl_work);
	cancel_work_sync(&link->rx_work);
	cancel_work_sync(&link->ps_kick_work);
	cancel_delayed_work_sync(&link->ps_wake_work);
}
EXPORT_SYMBOL_GPL(sbd_link_stop);

/**
 * sbd_link_create() - set up the link and its WWAN ports
 * @dev: the modem device, parent of the WWAN device
 * @ipc: kernel mapping of the IPC region (write-combine)
 * @ipc_size: its size
 * @ops: callbacks into the modem driver
 * @ctx: their context
 *
 * The ports exist from here on, so daemons can open them before the CP
 * runs; the link carries data once sbd_link_start() has been called.
 */
struct sbd_link *sbd_link_create(struct device *dev, void *ipc,
				 size_t ipc_size,
				 const struct sbd_link_ops *ops, void *ctx)
{
	struct sbd_link *link;
	int i, ret;

	link = kzalloc(sizeof(*link), GFP_KERNEL);
	if (!link)
		return ERR_PTR(-ENOMEM);

	link->dev = dev;
	link->ipc = ipc;
	link->ipc_size = ipc_size;
	link->ops = ops;
	link->ctx = ctx;
	INIT_WORK(&link->rx_work, sbd_rx_work);
	INIT_WORK(&link->ctrl_work, sbd_ctrl_work);
	init_waitqueue_head(&link->tx_wq);
	spin_lock_init(&link->ps_tx_lock);
	INIT_WORK(&link->ps_kick_work, sbd_ps_kick_work);
	INIT_DELAYED_WORK(&link->ps_wake_work, sbd_ps_wake_work);

	for (i = 0; i < SBD_NUM_CHANNELS; i++) {
		link->chans[i].link = link;
		link->chans[i].info = &sbd_channels[i];
		skb_queue_head_init(&link->chans[i].frags);
	}

	ret = sbd_link_plan(link);
	if (ret)
		goto err_free;

	for (i = 0; i < SBD_NUM_CHANNELS; i++) {
		struct sbd_chan *chan = &link->chans[i];
		struct wwan_port *port;

		if (chan->info->port_type < 0)
			continue;

		port = wwan_create_port(dev, chan->info->port_type,
					&sbd_port_ops, NULL, chan);
		if (IS_ERR(port)) {
			ret = PTR_ERR(port);
			dev_err(dev, "%s: failed to create port: %d\n",
				chan->info->name, ret);
			goto err_ports;
		}
		chan->port = port;
	}

	/* link 0 (rmnet0, PDP context 1) is created as wwan0 right away */
	ret = wwan_register_ops(dev, &sbd_wwan_ops, link, 0);
	if (ret) {
		dev_err(dev, "failed to register the network links: %d\n", ret);
		goto err_ports;
	}
	link->wwan_ops = true;

	return link;

err_ports:
	while (--i >= 0)
		if (link->chans[i].port)
			wwan_remove_port(link->chans[i].port);
err_free:
	kfree(link);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(sbd_link_create);

void sbd_link_destroy(struct sbd_link *link)
{
	int i;

	sbd_link_stop(link);

	if (link->wwan_ops)
		wwan_unregister_ops(link->dev);

	for (i = 0; i < SBD_NUM_CHANNELS; i++) {
		if (link->chans[i].port)
			wwan_remove_port(link->chans[i].port);
		skb_queue_purge(&link->chans[i].frags);
	}

	kfree(link);
}
EXPORT_SYMBOL_GPL(sbd_link_destroy);

MODULE_DESCRIPTION("Samsung SBD shared-memory link to a Shannon modem");
MODULE_LICENSE("GPL");
