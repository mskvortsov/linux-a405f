// SPDX-License-Identifier: GPL-2.0-only
/*
 * cfg80211 front end for the SCSC WLBT chip.
 *
 * Everything below the FAPI signal layer lives in core.c; this is the
 * part userspace can see -- a wiphy, station netdevs, and the cfg80211
 * operations to scan and associate from `iw` rather than from the
 * driver's probe function.
 *
 * Open and WPA/WPA2 networks both associate. EAPOL is carried on the
 * control path, which is how the 4-way handshake works without a data
 * path -- and there is no data path: ndo_start_xmit passes EAPOL to
 * MLME-SEND-FRAME and drops everything else, because MA_UNITDATA is not
 * implemented and a netdev that silently queued frames into nothing
 * would be worse than one that says so in its counters.
 */

#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/ieee80211.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/unaligned.h>
#include <net/cfg80211.h>

#include "wlan.h"

/*
 * Station interfaces start at 1; vif 0 is the device itself, which the
 * firmware always has and which scans run on.
 *
 * Two is what this driver can honestly claim. The firmware models more
 * -- the vendor builds with CONFIG_SCSC_WLAN_MAX_INTERFACES and runs
 * wlan0 alongside p2p0 -- but nothing here has exercised a third, and
 * an interface combination is a promise to userspace.
 *
 * SCSC_WLAN_MAX_VIFS itself lives in wlan.h, because core.c indexes by
 * vif too.
 */

/*
 * A joining station on an AP vif, keyed by both its address and the
 * firmware's peer_index -- MLME-PROCEDURE-STARTED gives the index and the
 * frame's source address, and everything after that (MLME-CONNECTED,
 * MLME-DISCONNECTED, .add_key, the transmit path) gives one or the other
 * and needs the rest.
 */
/* Same bound as core.c's per-vif mx->hip.assoc_req_ie, for the same reason. */
#define SCSC_WLAN_PEER_IE_MAX	512

/*
 * The most this driver will hand the firmware in one MLME-SEND-FRAME --
 * matches core.c's own SCSC_HIP_DATA_MAX, since a frame that would not
 * fit in one HIP send could never have gone out in the first place.
 * Kept as a separate constant rather than shared with core.c: that side
 * stays private to it by design, see wlan.h's header comment.
 */
#define SCSC_WLAN_MGMT_FRAME_MAX	2048

struct scsc_wlan_vif;

/*
 * One block-ack RX reorder session, one per (peer, TID) -- there is a
 * separate session per QoS traffic class because each has its own
 * sequence-number space and can be negotiated independently. Modelled on
 * the vendor driver's ba.c, with the same algorithm (see
 * scsc_wlan_ba_consume() below for the frame-arrival state machine and
 * scsc_wlan_ba_age() for what happens when a gap never fills in) but a
 * plain kcalloc'd buffer sized to what was actually negotiated rather
 * than a pool of MAX_BA_BUFFER_SIZE-sized sessions, since this driver has
 * no equivalent of the vendor's shared rx_ba_buffer_pool to borrow from.
 *
 * Sequence numbers are the 12-bit numbers 802.11 actually uses, so the
 * arithmetic on them wraps at SCSC_WLAN_BA_SN_MASK; SCSC_WLAN_BA_WINDOW is
 * half that space, the usual trick to tell a sequence number that has
 * wrapped around from one that is genuinely behind.
 */
#define SCSC_WLAN_BA_MAX_TID		8
#define SCSC_WLAN_BA_BUFFER_MAX		64
#define SCSC_WLAN_BA_AGE_MS		150
#define SCSC_WLAN_BA_SN_MASK		0x0FFF
#define SCSC_WLAN_BA_WINDOW		2048

struct scsc_wlan_ba_session {
	spinlock_t lock;
	bool active;
	u16 buffer_size;
	u16 occupied;
	u16 start_sn;
	u16 expected_sn;
	u16 highest_sn;
	bool timer_on;
	struct timer_list timer;
	/* buffer_size slots, indexed by scsc_wlan_ba_index(); NULL is empty. */
	struct sk_buff **buffer;
	/* Set once at scsc_wlan_ba_setup() and never changed after. */
	struct scsc_wlan_vif *v;
};

struct scsc_wlan_peer {
	u8 addr[ETH_ALEN];
	/* 0 means the slot is free; the firmware numbers real peers from 1. */
	u16 index;
	/*
	 * Whether MLME-CONNECTED.response has gone out, i.e. whether the
	 * firmware has opened this peer's data path. False while waiting
	 * for the pairwise key on a secured network -- see
	 * scsc_mx_mlme_connected_resp().
	 */
	bool connected;
	/*
	 * The information elements of this station's association request,
	 * from MLME-PROCEDURE-STARTED. cfg80211_new_sta() needs to hand
	 * these to hostapd -- without its RSN element hostapd has no way to
	 * know the station is doing WPA2 and never starts the 4-way
	 * handshake, which is silent on the air (nothing to capture) and
	 * looks identical to an open station until it times out.
	 */
	u8 assoc_ie[SCSC_WLAN_PEER_IE_MAX];
	u16 assoc_ie_len;
	/* One block-ack RX reorder session per TID -- see scsc_wlan_blockack(). */
	struct scsc_wlan_ba_session ba_rx[SCSC_WLAN_BA_MAX_TID];
};

/* SLSI_AP_PEER_CONNECTIONS_MAX in the vendor driver. */
#define SCSC_WLAN_MAX_PEERS	10

/*
 * A station's one peer -- the AP it is joined to -- is always this index:
 * the vendor derives peer_index as queue set + 1, and a station has a
 * single queue set, 0. See scsc_mx_tx_data().
 */
#define SCSC_WLAN_STA_PEER_INDEX	1

/* Per-interface state, living in the netdev's private area. */
struct scsc_wlan_vif {
	struct wireless_dev wdev;
	struct scsc_wlan *wlan;
	u16 vif;
	/* The BSS an association is in progress with, or joined. */
	u8 bssid[ETH_ALEN];
	bool connecting;
	/* jiffies at the moment a station connect succeeded, for .get_station. */
	unsigned long connected_jiffies;
	/*
	 * Whether this connection carries security, which decides when the
	 * association gets acknowledged: immediately on an open network,
	 * and only once the pairwise key is in on a secured one. See
	 * scsc_mx_mlme_connect_resp().
	 */
	bool secured;
	/*
	 * Whether the association negotiated QoS -- HT, VHT or a WMM
	 * element in the association response (the same three-way check
	 * the vendor's slsi_search_ies_for_qos_indicators() does). A
	 * non-QoS AP wants FAPI_PRIORITY_CONTENTION on every frame and
	 * nothing else; only a QoS one wants the classified TID. See
	 * scsc_wlan_ndo_xmit().
	 */
	bool qos;
	/* Whether this interface is beaconing, so .stop_ap knows. */
	bool ap_started;
	/*
	 * Whether the firmware currently has a vif for this interface.
	 *
	 * The vendor creates one per connection attempt and destroys it
	 * again when the attempt ends, rather than keeping one for the
	 * life of the netdev, so the firmware always sees a fresh vif
	 * when a connect arrives. Cleanup is lazy -- a failed attempt
	 * leaves the vif behind and the next connect removes it -- because
	 * failure is reported from the transport's bottom half, which
	 * already holds the lock that sending MLME-DEL-VIF would need.
	 */
	bool vif_added;
	/*
	 * Whether a firmware-driven roam is waiting on a fresh pairwise
	 * key before it can be acknowledged. Set by scsc_wlan_roamed() when
	 * MLME_ROAMED_IND says temporal_keys_required, consumed by
	 * scsc_wlan_add_key() -- see scsc_mx_mlme_roamed_resp(). Mirrors
	 * what `secured` already does for a fresh connect, split out
	 * because the two acknowledge different signals.
	 */
	bool roaming;
	/* The SSID being connected to, for scsc_wlan_synchronised(). */
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	u8 ssid_len;
	/*
	 * A management frame sent through .mgmt_tx, waiting to hear back
	 * whether it was acknowledged. One slot, not a table: cfg80211
	 * itself only ever has this driver send the next frame of an
	 * exchange (an SAE Commit/Confirm, an action frame reply) once the
	 * previous one's outcome is known, so there is never more than one
	 * genuinely in flight per vif.
	 */
	struct {
		bool pending;
		u16 host_tag;
		u64 cookie;
		u8 frame[SCSC_WLAN_MGMT_FRAME_MAX];
		u16 len;
	} mgmt_tx;
	/*
	 * EAPOL waiting to go out. ndo_start_xmit cannot sleep and the
	 * transport lock is a mutex, so frames are handed to a work item
	 * rather than sent inline. The handshake tolerates it: its
	 * timeouts are in hundreds of milliseconds.
	 */
	struct sk_buff_head tx_queue;
	struct work_struct tx_work;
	/* Stations on an AP vif. Unused -- all zero -- on a station vif. */
	struct scsc_wlan_peer peer[SCSC_WLAN_MAX_PEERS];
	/*
	 * The block-ack RX reorder sessions for a station vif's one peer,
	 * the AP -- SCSC_WLAN_STA_PEER_INDEX again, the same reason the
	 * peer[] table above has no row for it. Unused on an AP vif, where
	 * each row of peer[] carries its own ba_rx[] instead.
	 */
	struct scsc_wlan_ba_session ba_rx[SCSC_WLAN_BA_MAX_TID];
};

struct scsc_wlan {
	struct scsc_mx *mx;
	struct wiphy *wiphy;
	/* Indexed by vif; slot 0 is never used. */
	struct net_device *ndev[SCSC_WLAN_MAX_VIFS + 1];

	/* Channels, expanded from what the firmware said it supports. */
	struct ieee80211_channel *chan2;
	struct ieee80211_channel *chan5;
	struct ieee80211_supported_band band2;
	struct ieee80211_supported_band band5;

	/*
	 * The scan cfg80211 is waiting on. Written under this lock and
	 * read from the transport's bottom half, which is a different
	 * thread; the lock is only ever held across pointer handling, so
	 * it stays a spinlock.
	 */
	spinlock_t lock;
	struct cfg80211_scan_request *scan_req;
};

/* Find a peer by the id the firmware gave it, or NULL. */
static struct scsc_wlan_peer *scsc_wlan_peer_by_index(struct scsc_wlan_vif *v,
						       u16 peer_index)
{
	unsigned int i;

	for (i = 0; i < SCSC_WLAN_MAX_PEERS; i++)
		if (v->peer[i].index == peer_index)
			return &v->peer[i];

	return NULL;
}

/* Find a peer by its station address, or NULL. */
static struct scsc_wlan_peer *scsc_wlan_peer_by_addr(struct scsc_wlan_vif *v,
						      const u8 *addr)
{
	unsigned int i;

	for (i = 0; i < SCSC_WLAN_MAX_PEERS; i++)
		if (v->peer[i].index && ether_addr_equal(v->peer[i].addr, addr))
			return &v->peer[i];

	return NULL;
}

/*
 * Block-ack session lifecycle, needed by interface add/remove below but
 * implemented next to the RX data path further down, where the rest of the
 * reorder algorithm lives.
 */
static void scsc_wlan_ba_setup(struct scsc_wlan_vif *v, struct scsc_wlan_ba_session *rx);
static void scsc_wlan_ba_stop_all(struct scsc_wlan_ba_session *rx);

/*
 * Interface combinations.
 *
 * cfg80211 rejects a combination of one outright -- "combinations with
 * just one interface aren't real" -- so this table only exists because
 * the driver genuinely supports two station interfaces at once, which
 * the firmware accepts as two vifs.
 *
 * num_different_channels is 2: a station and an AP on two different
 * channels concurrently is real, confirmed with an independent receiver
 * rather than assumed -- see wifi.md. A *second* AP is a different,
 * unsafe story: it wedges the firmware's command channel outright and
 * needs a reboot to recover. cfg80211 does not stop a station vif being
 * turned into a second AP by type change alone (that path never
 * consults this table -- only claiming an actual channel does), so nothing
 * below currently prevents reaching that state; see wifi.md before
 * changing AP handling near two vifs.
 */
static const struct ieee80211_iface_limit scsc_wlan_iface_limits[] = {
	{
		.max = SCSC_WLAN_MAX_VIFS,
		.types = BIT(NL80211_IFTYPE_STATION),
	},
	{
		/*
		 * One access point. There is no way to say "an AP on its
		 * own" -- cfg80211 rejects a combination whose
		 * max_interfaces is below two -- so this table also permits
		 * an AP beside a station, which is what is actually tested
		 * and working (station + AP, two channels). It does not
		 * stop a second AP being reached by other means -- see above.
		 */
		.max = 1,
		.types = BIT(NL80211_IFTYPE_AP),
	},
};

static const struct ieee80211_iface_combination scsc_wlan_iface_combinations[] = {
	{
		.limits = scsc_wlan_iface_limits,
		.n_limits = ARRAY_SIZE(scsc_wlan_iface_limits),
		.max_interfaces = SCSC_WLAN_MAX_VIFS,
		.num_different_channels = 2,
	},
};

/*
 * What the firmware will install as a key. CCMP and TKIP cover WPA2 and
 * WPA; the two group-management ciphers are what an AP with protected
 * management frames will hand over. WEP is deliberately absent -- the
 * key would have to be prepended to the material and counted in the
 * length, and nothing here has tried it.
 */
static const u32 scsc_wlan_cipher_suites[] = {
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_AES_CMAC,
};

/*
 * 802.11b/g/a rates. The first four are the DSSS rates, which exist
 * only on 2.4 GHz; the 5 GHz band starts at the fifth entry. Values are
 * in units of 100 kbit/s, which is what cfg80211 wants.
 */
static struct ieee80211_rate scsc_wlan_rates[] = {
	{ .bitrate = 10, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 20, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 55, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 110, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 60 },
	{ .bitrate = 90 },
	{ .bitrate = 120 },
	{ .bitrate = 180 },
	{ .bitrate = 240 },
	{ .bitrate = 360 },
	{ .bitrate = 480 },
	{ .bitrate = 540 },
};

#define SCSC_WLAN_OFDM_FIRST	4

static u32 scsc_wlan_chan_to_freq(u8 chan)
{
	if (chan == 14)
		return 2484;
	if (chan < 14)
		return 2407 + 5 * chan;

	return 5000 + 5 * chan;
}

/* 5 GHz radar-detection band: usable, but only passively. */
static bool scsc_wlan_chan_is_dfs(u8 chan)
{
	return chan >= 52 && chan <= 144;
}

/*
 * HT capabilities, common to both bands: the same radio, same MCS/AMPDU
 * limits regardless of which channels it currently has open. The
 * leading capability word and the AMPDU parameters byte come from the
 * firmware's own UnifiHtCapabilities MIB (scsc_mx_ht_cap()); the MCS-set
 * bytes that follow it in that MIB are left at a single-spatial-stream
 * default -- MCS 0-7 -- rather than parsed from bytes whose bitmap
 * layout has not been independently confirmed the way the leading
 * capability word has. This part is 1x1 (RF_CHIP_VERSION reports S612),
 * so that default is not a guess so much as the only value it could
 * report.
 */
static struct ieee80211_sta_ht_cap scsc_wlan_ht_cap(struct scsc_wlan *wlan)
{
	struct ieee80211_sta_ht_cap cap = {};
	const u8 *raw;

	cap.ht_supported = scsc_mx_ht_cap(wlan->mx, &raw);
	if (!cap.ht_supported)
		return cap;

	cap.cap = get_unaligned_le16(raw);
	cap.ampdu_factor = raw[2] & IEEE80211_HT_AMPDU_PARM_FACTOR;
	cap.ampdu_density = (raw[2] & IEEE80211_HT_AMPDU_PARM_DENSITY) >> 2;
	cap.mcs.rx_mask[0] = 0xff;

	return cap;
}

/*
 * Build the band tables from the channel list the firmware reported,
 * rather than from a fixed idea of what the part supports. The list
 * arrives as (first channel, count) pairs and spans both bands, so the
 * split is by channel number.
 */
static int scsc_wlan_build_bands(struct scsc_wlan *wlan)
{
	const u8 *ranges;
	unsigned int pairs, i, n2 = 0, n5 = 0;

	ranges = scsc_mx_chanranges(wlan->mx, &pairs);
	if (!ranges || !pairs)
		return -ENODEV;

	for (i = 0; i < pairs; i++) {
		u8 first = ranges[i * 2];
		u8 count = ranges[i * 2 + 1];

		if (first + count <= 15)
			n2 += count;
		else
			n5 += count;
	}

	if (n2) {
		wlan->chan2 = kcalloc(n2, sizeof(*wlan->chan2), GFP_KERNEL);
		if (!wlan->chan2)
			return -ENOMEM;
	}
	if (n5) {
		wlan->chan5 = kcalloc(n5, sizeof(*wlan->chan5), GFP_KERNEL);
		if (!wlan->chan5) {
			kfree(wlan->chan2);
			wlan->chan2 = NULL;
			return -ENOMEM;
		}
	}

	n2 = 0;
	n5 = 0;
	for (i = 0; i < pairs; i++) {
		u8 first = ranges[i * 2];
		u8 count = ranges[i * 2 + 1];
		/*
		 * count is how many channels, not how far the channel number
		 * spans: 2.4 GHz channels are numbered one apart (1, 2, 3...),
		 * but 5 GHz ones only exist every 20 MHz, four channel
		 * numbers apart (36, 40, 44...). Same MIB the vendor's
		 * slsi_get_supported_channels() decodes with increment 1 vs
		 * 4 for exactly this reason -- get this wrong and the range
		 * both invents channel numbers nothing uses and runs out
		 * before reaching the real ones past it.
		 */
		u8 step = (first + count <= 15) ? 1 : 4;
		u8 c = first;
		unsigned int n;

		for (n = 0; n < count; n++, c += step) {
			struct ieee80211_channel *ch;

			if (c <= 14)
				ch = &wlan->chan2[n2++];
			else
				ch = &wlan->chan5[n5++];

			ch->band = c <= 14 ? NL80211_BAND_2GHZ : NL80211_BAND_5GHZ;
			ch->center_freq = scsc_wlan_chan_to_freq(c);
			ch->hw_value = c;
			ch->max_power = 20;
			if (scsc_wlan_chan_is_dfs(c))
				ch->flags = IEEE80211_CHAN_RADAR |
					    IEEE80211_CHAN_NO_IR;
		}
	}

	if (n2) {
		wlan->band2.band = NL80211_BAND_2GHZ;
		wlan->band2.channels = wlan->chan2;
		wlan->band2.n_channels = n2;
		wlan->band2.bitrates = scsc_wlan_rates;
		wlan->band2.n_bitrates = ARRAY_SIZE(scsc_wlan_rates);
		wlan->band2.ht_cap = scsc_wlan_ht_cap(wlan);
		wlan->wiphy->bands[NL80211_BAND_2GHZ] = &wlan->band2;
	}
	if (n5) {
		const u8 *raw;

		wlan->band5.band = NL80211_BAND_5GHZ;
		wlan->band5.channels = wlan->chan5;
		wlan->band5.n_channels = n5;
		wlan->band5.bitrates = scsc_wlan_rates + SCSC_WLAN_OFDM_FIRST;
		wlan->band5.n_bitrates = ARRAY_SIZE(scsc_wlan_rates) -
					 SCSC_WLAN_OFDM_FIRST;
		wlan->band5.ht_cap = scsc_wlan_ht_cap(wlan);

		/*
		 * VHT is 5 GHz only -- 802.11ac has no business on 2.4 GHz,
		 * and the firmware does not advertise it there either. Same
		 * trust boundary as HT above: the capability word comes
		 * from the firmware's own UnifiVhtCapabilities MIB, the
		 * MCS map is the single-spatial-stream default for this
		 * chip -- see scsc_wlan_ht_cap().
		 */
		wlan->band5.vht_cap.vht_supported =
			scsc_mx_vht_cap(wlan->mx, &raw);
		if (wlan->band5.vht_cap.vht_supported) {
			wlan->band5.vht_cap.cap = get_unaligned_le32(raw);
			wlan->band5.vht_cap.vht_mcs.rx_mcs_map =
				cpu_to_le16(0xfffe);
			wlan->band5.vht_cap.vht_mcs.tx_mcs_map =
				cpu_to_le16(0xfffe);
		}

		wlan->wiphy->bands[NL80211_BAND_5GHZ] = &wlan->band5;
	}

	dev_info(scsc_mx_dev(wlan->mx), "wlan: %u channels on 2.4 GHz, %u on 5 GHz\n",
		 n2, n5);

	return 0;
}

/* cfg80211 operations */

static int scsc_wlan_scan(struct wiphy *wiphy,
			  struct cfg80211_scan_request *request)
{
	struct scsc_wlan *wlan = wiphy_priv(wiphy);
	const u8 *ssid = NULL;
	u8 ssid_len = 0;
	u32 *freqs;
	unsigned int i;
	int ret;

	spin_lock_bh(&wlan->lock);
	if (wlan->scan_req) {
		spin_unlock_bh(&wlan->lock);
		return -EBUSY;
	}
	wlan->scan_req = request;
	spin_unlock_bh(&wlan->lock);

	freqs = kcalloc(request->n_channels, sizeof(*freqs), GFP_KERNEL);
	if (!freqs) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < request->n_channels; i++)
		freqs[i] = request->channels[i]->center_freq;

	/*
	 * One SSID only. The firmware takes a list, but nothing here
	 * needs more and reporting a limit we do not exercise would be
	 * worse than the restriction.
	 */
	if (request->n_ssids && request->ssids[0].ssid_len) {
		ssid = request->ssids[0].ssid;
		ssid_len = request->ssids[0].ssid_len;
	}

	ret = scsc_mx_mlme_scan_start(wlan->mx, ssid, ssid_len, freqs,
				      request->n_channels);
	kfree(freqs);

out:
	if (ret) {
		spin_lock_bh(&wlan->lock);
		wlan->scan_req = NULL;
		spin_unlock_bh(&wlan->lock);
	}

	return ret;
}

static void scsc_wlan_abort_scan(struct wiphy *wiphy, struct wireless_dev *wdev)
{
	struct scsc_wlan *wlan = wiphy_priv(wiphy);

	scsc_mx_mlme_scan_stop(wlan->mx);
	scsc_wlan_scan_done(wlan->mx, true);
}

/* netdev operations */

static int scsc_wlan_ndo_open(struct net_device *ndev)
{
	/*
	 * No vif here. The firmware's virtual interface belongs to a
	 * connection, not to the netdev: it is created in .connect and
	 * destroyed when the connection ends, which is what the vendor
	 * does and what the firmware appears to expect.
	 *
	 * No carrier either -- there is no association, so nothing should
	 * believe this interface can carry traffic.
	 */
	netif_carrier_off(ndev);
	netif_start_queue(ndev);

	return 0;
}

static int scsc_wlan_ndo_stop(struct net_device *ndev)
{
	struct scsc_wlan_vif *v = netdev_priv(ndev);

	netif_stop_queue(ndev);

	/*
	 * Stop the transmit path before the vif goes: the work item sends
	 * on it, so cancelling afterwards would leave a window in which a
	 * frame is queued against a vif the firmware has already dropped.
	 */
	cancel_work_sync(&v->tx_work);
	skb_queue_purge(&v->tx_queue);

	if (v->vif_added) {
		scsc_mx_mlme_vif_del(v->wlan->mx, v->vif);
		v->vif_added = false;
	}

	return 0;
}

/*
 * How much may sit between ndo_start_xmit and the work item.
 *
 * The transport has HIP4_DAT_SLOTS (218) segments, so a backlog deeper
 * than this cannot be a burst the firmware is about to absorb -- it
 * means the work item is not keeping up, and the honest answer is
 * backpressure rather than an unbounded list.
 */
#define SCSC_WLAN_TX_BACKLOG	256

/*
 * Frames go out through a work item, not from here: this cannot sleep
 * and the transport lock is a mutex.
 *
 * EAPOL takes the control path even now that the data path exists --
 * the firmware installs the pairwise key straight after the last
 * handshake frame, and it only knows which frame that was because
 * MLME-SEND-FRAME says so.
 */
static netdev_tx_t scsc_wlan_ndo_xmit(struct sk_buff *skb,
				      struct net_device *ndev)
{
	struct scsc_wlan_vif *v = netdev_priv(ndev);

	skb_queue_tail(&v->tx_queue, skb);

	if (skb_queue_len(&v->tx_queue) >= SCSC_WLAN_TX_BACKLOG)
		netif_stop_queue(ndev);

	schedule_work(&v->tx_work);

	return NETDEV_TX_OK;
}

static void scsc_wlan_tx_work(struct work_struct *work)
{
	struct scsc_wlan_vif *v = container_of(work, struct scsc_wlan_vif,
					       tx_work);
	struct net_device *ndev = v->wdev.netdev;
	struct sk_buff *skb;

	while ((skb = skb_dequeue(&v->tx_queue))) {
		int ret;

		if (skb->protocol == htons(ETH_P_PAE)) {
			u16 type = scsc_mx_eapol_message_type(skb->data,
							      skb->len);

			ret = scsc_mx_mlme_send_frame(v->wlan->mx, v->vif,
						      type, skb->data,
						      skb->len);
		} else {
			u16 peer_index = SCSC_WLAN_STA_PEER_INDEX;
			u16 priority = SCSC_PRIORITY_CONTENTION;

			/*
			 * On an AP vif the destination address picks the
			 * peer -- unless it is a group address, which the
			 * vendor's queue-to-AID mapping always resolves to
			 * peer_index 0 (a negative queue set plus one), so
			 * that is what the firmware expects for its own
			 * broadcast/multicast queue. A frame to a unicast
			 * station that has not finished its handshake yet
			 * has nowhere to go.
			 */
			if (v->wdev.iftype == NL80211_IFTYPE_AP) {
				if (is_multicast_ether_addr(skb->data)) {
					peer_index = 0;
				} else {
					struct scsc_wlan_peer *p =
						scsc_wlan_peer_by_addr(v, skb->data);

					if (!p || !p->connected) {
						ndev->stats.tx_dropped++;
						dev_kfree_skb(skb);
						continue;
					}
					peer_index = p->index;
				}
			}

			/*
			 * A non-QoS association wants FAPI_PRIORITY_CONTENTION
			 * on every frame -- the AP has no per-TID queues to
			 * address. A QoS one wants the classified TID instead,
			 * from the IP DSCP/802.1p field cfg80211's own helper
			 * already knows how to read; no custom QoS map (no
			 * .set_qos_map yet), so NULL falls back to the
			 * standard DSCP-to-UP mapping.
			 */
			if (v->qos)
				priority = cfg80211_classify8021d(skb, NULL);

			ret = scsc_mx_tx_data(v->wlan->mx, v->vif, peer_index,
					      skb->data, skb->len, priority);
		}

		/*
		 * A full transport is not an error and the frame is not
		 * lost: put it back and stop until the buffers it is
		 * waiting for come home, which is what wakes us again.
		 */
		if (ret == -ENOBUFS || ret == -ENOSPC) {
			skb_queue_head(&v->tx_queue, skb);
			netif_stop_queue(ndev);
			return;
		}

		if (ret) {
			ndev->stats.tx_errors++;
		} else {
			ndev->stats.tx_packets++;
			ndev->stats.tx_bytes += skb->len;
		}
		dev_kfree_skb(skb);
	}

	if (netif_queue_stopped(ndev))
		netif_wake_queue(ndev);
}

void scsc_wlan_tx_resume(struct scsc_mx *mx)
{
	struct scsc_wlan *wlan = scsc_mx_wlan(mx);
	unsigned int i;

	if (!wlan)
		return;

	/*
	 * Which interface stalled is not recorded -- the transport's
	 * buffer pool is shared, so any of them could have been the one
	 * that filled it. Poking them all is cheap and cannot be wrong.
	 */
	for (i = 1; i <= SCSC_WLAN_MAX_VIFS; i++) {
		struct net_device *ndev = wlan->ndev[i];

		if (ndev)
			schedule_work(&((struct scsc_wlan_vif *)
					netdev_priv(ndev))->tx_work);
	}
}

static const struct net_device_ops scsc_wlan_netdev_ops = {
	.ndo_open = scsc_wlan_ndo_open,
	.ndo_stop = scsc_wlan_ndo_stop,
	.ndo_start_xmit = scsc_wlan_ndo_xmit,
};

/* Interfaces */

/*
 * Create one station interface on the first free vif.
 *
 * Called both from registration, where the RTNL is ours to take, and
 * from cfg80211's add_virtual_intf, where it is already held -- hence
 * the flag: cfg80211_register_netdevice() is the one to use from inside
 * a callback, and register_netdev() the one to use outside.
 */
static struct wireless_dev *scsc_wlan_add_iface(struct scsc_wlan *wlan,
						const char *name,
						unsigned char name_assign_type,
						enum nl80211_iftype iftype,
						bool in_callback)
{
	struct device *dev = scsc_mx_dev(wlan->mx);
	struct scsc_wlan_vif *v;
	struct net_device *ndev;
	u8 addr[ETH_ALEN];
	unsigned int vif;
	u64 mac;
	int ret, i;

	for (vif = 1; vif <= SCSC_WLAN_MAX_VIFS; vif++)
		if (!wlan->ndev[vif])
			break;
	if (vif > SCSC_WLAN_MAX_VIFS)
		return ERR_PTR(-ENOSPC);

	ndev = alloc_netdev(sizeof(*v), name, name_assign_type, ether_setup);
	if (!ndev)
		return ERR_PTR(-ENOMEM);

	v = netdev_priv(ndev);
	v->wlan = wlan;
	v->vif = vif;
	skb_queue_head_init(&v->tx_queue);
	INIT_WORK(&v->tx_work, scsc_wlan_tx_work);
	scsc_wlan_ba_setup(v, v->ba_rx);
	v->wdev.wiphy = wlan->wiphy;
	v->wdev.iftype = iftype;
	v->wdev.netdev = ndev;

	ndev->netdev_ops = &scsc_wlan_netdev_ops;
	ndev->ieee80211_ptr = &v->wdev;
	ndev->needs_free_netdev = true;
	SET_NETDEV_DEV(ndev, dev);

	/*
	 * One address per interface, derived from the configured base by
	 * its vif index. The base is already locally administered, so
	 * varying the last octet keeps every interface's address unique
	 * without claiming an OUI that is not ours.
	 */
	mac = scsc_mx_mac_address();
	for (i = 0; i < ETH_ALEN; i++)
		addr[i] = mac >> (8 * (ETH_ALEN - 1 - i));
	addr[ETH_ALEN - 1] += vif - 1;
	eth_hw_addr_set(ndev, addr);

	if (in_callback)
		ret = cfg80211_register_netdevice(ndev);
	else
		ret = register_netdev(ndev);
	if (ret) {
		free_netdev(ndev);
		return ERR_PTR(ret);
	}

	wlan->ndev[vif] = ndev;
	dev_info(dev, "wlan: %s on vif %u, %pM\n", ndev->name, vif,
		 ndev->dev_addr);

	return &v->wdev;
}

static struct wireless_dev *scsc_wlan_add_virtual_intf(struct wiphy *wiphy,
						       const char *name,
						       unsigned char name_assign_type,
						       enum nl80211_iftype type,
						       struct vif_params *params)
{
	struct scsc_wlan *wlan = wiphy_priv(wiphy);

	if (type != NL80211_IFTYPE_STATION && type != NL80211_IFTYPE_AP)
		return ERR_PTR(-EOPNOTSUPP);

	return scsc_wlan_add_iface(wlan, name, name_assign_type, type, true);
}

static int scsc_wlan_del_virtual_intf(struct wiphy *wiphy,
				      struct wireless_dev *wdev)
{
	struct scsc_wlan *wlan = wiphy_priv(wiphy);
	struct scsc_wlan_vif *v;
	unsigned int i;

	if (!wdev->netdev)
		return -EINVAL;

	v = netdev_priv(wdev->netdev);

	/*
	 * Block-ack sessions hold a kcalloc'd buffer and, while a hole is
	 * open, a running timer -- both have to go before the netdev they
	 * point back into does.
	 */
	scsc_wlan_ba_stop_all(v->ba_rx);
	for (i = 0; i < SCSC_WLAN_MAX_PEERS; i++)
		scsc_wlan_ba_stop_all(v->peer[i].ba_rx);

	/*
	 * Any interface may go, including the one created with the wiphy:
	 * the phy outlives them and a new one can be added at any time.
	 * Refusing the first was an arbitrary restriction, and a bad one
	 * -- cfg80211 closes the netdev before asking, so the refusal
	 * still left the interface down.
	 */
	wlan->ndev[v->vif] = NULL;
	cfg80211_unregister_netdevice(wdev->netdev);

	return 0;
}

/*
 * Associate.
 *
 * WPA and WPA2 authenticate as Open System at the 802.11 level and do
 * the real work afterward in the 4-way handshake, over EAPOL carried on
 * the control path (see scsc_wlan_add_key()). WPA3-SAE authenticates
 * *at* the 802.11 level instead -- the SAE exchange itself takes the
 * place of Open System authentication -- which this driver honours by
 * asking the firmware for FAPI_AUTHENTICATIONTYPE_SAE and then running
 * that exchange from here rather than in the firmware: see
 * scsc_wlan_synchronised() and scsc_wlan_external_auth(). Any other
 * auth type is refused outright, which is better than associating and
 * then timing out somewhere in wpa_supplicant.
 */
static int scsc_wlan_connect(struct wiphy *wiphy, struct net_device *ndev,
			     struct cfg80211_connect_params *sme)
{
	struct scsc_wlan_vif *v = netdev_priv(ndev);
	struct cfg80211_bss *bss;
	u32 freq;
	int ret;
	bool sae;

	const u8 *sec_ie = NULL;
	u16 sec_ie_len = 0;

	sae = sme->auth_type == NL80211_AUTHTYPE_SAE &&
	      (sme->flags & CONNECT_REQ_EXTERNAL_AUTH_SUPPORT);

	if (sme->auth_type != NL80211_AUTHTYPE_OPEN_SYSTEM &&
	    sme->auth_type != NL80211_AUTHTYPE_AUTOMATIC && !sae) {
		netdev_warn(ndev, "unsupported auth type %d\n", sme->auth_type);
		return -EOPNOTSUPP;
	}

	/*
	 * The RSN element from the supplicant goes into the connect
	 * request; the firmware builds the association request from it.
	 * Anything else in sme->ie belongs in MLME-ADD-INFO-ELEMENTS,
	 * which is not implemented, so it is dropped rather than
	 * smuggled somewhere it does not belong.
	 */
	if (sme->ie_len) {
		sec_ie = cfg80211_find_ie(WLAN_EID_RSN, sme->ie, sme->ie_len);
		if (sec_ie)
			sec_ie_len = sec_ie[1] + 2;
	}

	if (sme->privacy && !sec_ie) {
		netdev_warn(ndev, "encrypted network without an RSN element\n");
		return -EOPNOTSUPP;
	}

	/*
	 * cfg80211 may give an SSID with no BSSID or channel, so resolve
	 * them from the scan results it already holds rather than making
	 * the caller supply what it has just been told.
	 */
	bss = cfg80211_get_bss(wiphy, sme->channel, sme->bssid, sme->ssid,
			       sme->ssid_len, IEEE80211_BSS_TYPE_ESS,
			       IEEE80211_PRIVACY_ANY);
	if (!bss) {
		netdev_warn(ndev, "no BSS for that SSID; scan first\n");
		return -ENOENT;
	}

	memcpy(v->bssid, bss->bssid, ETH_ALEN);
	freq = bss->channel->center_freq;
	cfg80211_put_bss(wiphy, bss);

	/*
	 * Clear out a vif left behind by an attempt that failed, then
	 * create the one this connection will use.
	 */
	if (v->vif_added) {
		scsc_mx_mlme_vif_del(v->wlan->mx, v->vif);
		v->vif_added = false;
	}

	ret = scsc_mx_mlme_vif_add(v->wlan->mx, v->vif, ndev->dev_addr, false);
	if (ret)
		return ret;
	v->vif_added = true;

	/* The vendor does this on every fresh station vif, before connecting. */
	ret = scsc_mx_mlme_register_action_frames(v->wlan->mx, v->vif);
	if (ret)
		netdev_warn(ndev, "action frame registration failed: %d\n", ret);

	v->connecting = true;
	v->secured = sme->privacy;
	v->ssid_len = min_t(u8, sme->ssid_len, IEEE80211_MAX_SSID_LEN);
	memcpy(v->ssid, sme->ssid, v->ssid_len);
	ret = scsc_mx_mlme_connect(v->wlan->mx, v->vif, v->bssid, freq,
				   sme->ssid, sme->ssid_len, sec_ie, sec_ie_len,
				   sae);
	if (ret) {
		v->connecting = false;
		scsc_mx_mlme_vif_del(v->wlan->mx, v->vif);
		v->vif_added = false;
	}

	return ret;
}

static int scsc_wlan_disconnect(struct wiphy *wiphy, struct net_device *ndev,
				u16 reason_code)
{
	struct scsc_wlan_vif *v = netdev_priv(ndev);
	int ret;

	ret = scsc_mx_mlme_disconnect(v->wlan->mx, v->vif, v->bssid,
				      reason_code);

	if (v->vif_added) {
		scsc_mx_mlme_vif_del(v->wlan->mx, v->vif);
		v->vif_added = false;
	}

	/*
	 * Tell cfg80211 either way. A firmware that will not disconnect
	 * cleanly still leaves us with an interface userspace believes is
	 * associated, and that is worse than reporting the loss.
	 */
	v->connecting = false;
	netif_carrier_off(ndev);
	cfg80211_disconnected(ndev, reason_code, NULL, 0, true, GFP_KERNEL);

	return ret;
}

/*
 * The result of a WPA3-SAE exchange run from here -- see
 * scsc_wlan_synchronised(), which is what asked for this in the first
 * place. Either way the firmware is told and the connection attempt
 * proceeds or fails accordingly; there is nothing else to decide here.
 */
static int scsc_wlan_external_auth(struct wiphy *wiphy,
				   struct net_device *ndev,
				   struct cfg80211_external_auth_params *params)
{
	struct scsc_wlan_vif *v = netdev_priv(ndev);

	scsc_mx_mlme_synchronised_resp(v->wlan->mx, v->vif, params->bssid,
				       params->status == WLAN_STATUS_SUCCESS);
	return 0;
}

/*
 * Send an arbitrary management frame -- an SAE Commit/Confirm from
 * scsc_wlan_synchronised()'s exchange, or any other action frame
 * wpa_supplicant wants out (SA Query, WNM, ...).
 *
 * There is no off-channel vif here, unlike a full P2P-capable driver:
 * a frame only ever goes out on whatever channel this vif is already
 * on, which covers every case this driver actually has a use for --
 * SAE and post-connect action frames are both addressed to the BSS
 * already being joined or joined, never somewhere else. params->chan
 * is accepted rather than acted on for that reason.
 */
static int scsc_wlan_mgmt_tx(struct wiphy *wiphy, struct wireless_dev *wdev,
			     struct cfg80211_mgmt_tx_params *params,
			     u64 cookie)
{
	struct net_device *ndev = wdev->netdev;
	struct scsc_wlan_vif *v = netdev_priv(ndev);
	const struct ieee80211_mgmt *mgmt = (const void *)params->buf;
	u16 msg_type;
	u16 host_tag;
	int ret;

	if (params->len < offsetof(struct ieee80211_mgmt, u) ||
	    !ieee80211_is_mgmt(mgmt->frame_control))
		return -EINVAL;

	if (params->len > SCSC_WLAN_MGMT_FRAME_MAX)
		return -EMSGSIZE;

	if (v->mgmt_tx.pending)
		return -EBUSY;

	msg_type = ieee80211_is_auth(mgmt->frame_control) ?
			   SCSC_MSGTYPE_IEEE80211_MGMT :
			   SCSC_MSGTYPE_IEEE80211_ACTION;

	ret = scsc_mx_mlme_send_mgmt_frame(v->wlan->mx, v->vif, msg_type,
					   params->buf, params->len,
					   &host_tag);
	if (ret)
		return ret;

	v->mgmt_tx.pending = true;
	v->mgmt_tx.host_tag = host_tag;
	v->mgmt_tx.cookie = cookie;
	v->mgmt_tx.len = params->len;
	memcpy(v->mgmt_tx.frame, params->buf, params->len);

	return 0;
}

/*
 * 802.11 power-save, station vifs only -- an AP vif has nobody to poll
 * and no PM bit of its own to set. A vif the firmware does not yet have
 * (not yet associated, or already torn down) is a silent no-op rather
 * than an error: whatever mode is requested will apply once there is a
 * vif to apply it to, the same way userspace does not treat "not
 * connected yet" as a power-save failure.
 */
static int scsc_wlan_set_power_mgmt(struct wiphy *wiphy,
				    struct net_device *ndev, bool enabled,
				    int timeout)
{
	struct scsc_wlan_vif *v = netdev_priv(ndev);

	if (ndev->ieee80211_ptr->iftype != NL80211_IFTYPE_STATION ||
	    !v->vif_added)
		return 0;

	return scsc_mx_mlme_powermgt(v->wlan->mx, v->vif,
				     enabled ? SCSC_POWERMGT_POWER_SAVE :
					       SCSC_POWERMGT_ACTIVE);
}

/*
 * The firmware's own encoding for a data rate, confirmed against the
 * vendor's slsi_decode_fw_rate(): bits 14-15 pick legacy/HT/VHT, the
 * rest is class-specific. Unlike the vendor, this only fills the raw
 * struct rate_info fields (mcs/nss/bw/flags) rather than also computing
 * a plain Mbit/s figure -- the kernel already does that math from
 * exactly these fields when it builds NL80211_STA_INFO_TX_BITRATE, so
 * there is nothing here that would not be redone less correctly.
 */
#define SCSC_FW_RATE_CLASS		0xc000
#define SCSC_FW_RATE_CLASS_LEGACY	0x4000
#define SCSC_FW_RATE_CLASS_HT		0x8000
#define SCSC_FW_RATE_CLASS_VHT		0xc000
#define SCSC_FW_RATE_LEGACY_IDX		0x1fff
#define SCSC_FW_RATE_HT_MCS		0x003f
#define SCSC_FW_RATE_VHT_MCS		0x000f
#define SCSC_FW_RATE_VHT_NSS		0x0070
#define SCSC_FW_RATE_BW			0x0600
#define SCSC_FW_RATE_BW_40		0x0200
#define SCSC_FW_RATE_BW_80		0x0400
#define SCSC_FW_RATE_SGI		0x0100

/* In 100 kbit/s, indexed by the low 13 bits of a legacy-class rate. */
static const u16 scsc_fw_rate_legacy[] = {
	0, 10, 20, 55, 60, 90, 110, 120, 180, 240, 360, 480, 540,
};

static void scsc_wlan_decode_rate(u16 fw_rate, struct rate_info *rate)
{
	memset(rate, 0, sizeof(*rate));

	switch (fw_rate & SCSC_FW_RATE_CLASS) {
	case SCSC_FW_RATE_CLASS_LEGACY: {
		u16 idx = fw_rate & SCSC_FW_RATE_LEGACY_IDX;

		if (idx < ARRAY_SIZE(scsc_fw_rate_legacy))
			rate->legacy = scsc_fw_rate_legacy[idx];
		break;
	}
	case SCSC_FW_RATE_CLASS_HT:
		rate->flags |= RATE_INFO_FLAGS_MCS;
		/* HT MCS 0-31 already implies spatial streams; no nss field. */
		rate->mcs = fw_rate & SCSC_FW_RATE_HT_MCS;
		rate->bw = (fw_rate & SCSC_FW_RATE_BW) == SCSC_FW_RATE_BW_40 ?
				   RATE_INFO_BW_40 : RATE_INFO_BW_20;
		if (fw_rate & SCSC_FW_RATE_SGI)
			rate->flags |= RATE_INFO_FLAGS_SHORT_GI;
		break;
	case SCSC_FW_RATE_CLASS_VHT:
		rate->flags |= RATE_INFO_FLAGS_VHT_MCS;
		rate->mcs = fw_rate & SCSC_FW_RATE_VHT_MCS;
		rate->nss = ((fw_rate & SCSC_FW_RATE_VHT_NSS) >> 4) + 1;
		switch (fw_rate & SCSC_FW_RATE_BW) {
		case SCSC_FW_RATE_BW_40:
			rate->bw = RATE_INFO_BW_40;
			break;
		case SCSC_FW_RATE_BW_80:
			rate->bw = RATE_INFO_BW_80;
			break;
		default:
			rate->bw = RATE_INFO_BW_20;
			break;
		}
		if (fw_rate & SCSC_FW_RATE_SGI)
			rate->flags |= RATE_INFO_FLAGS_SHORT_GI;
		break;
	}
}

/*
 * RSSI, TX rate and how long the link has been up -- the one peer a
 * station vif has. AP-vif peers are not covered: they have far more
 * interesting counters (per-station tx/rx/retry, not just RSSI/rate),
 * gated on indexed MIB-GETs (SLSI_PSID_UNIFI_FRAME_TX_COUNTERS and
 * friends) this driver's MIB-GET encoder does not build yet -- left for
 * later rather than shipped half-right.
 */
static int scsc_wlan_get_station(struct wiphy *wiphy, struct wireless_dev *wdev,
				 const u8 *mac, struct station_info *sinfo)
{
	struct net_device *ndev = wdev->netdev;
	struct scsc_wlan_vif *v = netdev_priv(ndev);
	s16 rssi;
	u16 fw_rate;

	if (ndev->ieee80211_ptr->iftype != NL80211_IFTYPE_STATION ||
	    !v->vif_added || !ether_addr_equal(mac, v->bssid))
		return -ENOENT;

	sinfo->connected_time = (jiffies - v->connected_jiffies) / HZ;
	sinfo->filled |= BIT_ULL(NL80211_STA_INFO_CONNECTED_TIME);

	if (!scsc_mx_mlme_get_station_info(v->wlan->mx, v->vif, &rssi, &fw_rate)) {
		if (rssi) {
			sinfo->signal = rssi;
			sinfo->filled |= BIT_ULL(NL80211_STA_INFO_SIGNAL);
		}
		if (fw_rate) {
			scsc_wlan_decode_rate(fw_rate, &sinfo->txrate);
			sinfo->filled |= BIT_ULL(NL80211_STA_INFO_TX_BITRATE);
		}
	}

	return 0;
}

/*
 * A station vif has exactly one peer worth reporting -- the AP it is
 * joined to -- so this is scsc_wlan_get_station() with idx as the only
 * new thing: cfg80211 calls .dump_station repeatedly with idx = 0, 1,
 * 2, ... until something other than 0 comes back, which is the
 * iteration protocol `iw ... station dump` relies on.
 */
static int scsc_wlan_dump_station(struct wiphy *wiphy, struct wireless_dev *wdev,
				  int idx, u8 *mac, struct station_info *sinfo)
{
	struct net_device *ndev = wdev->netdev;
	struct scsc_wlan_vif *v = netdev_priv(ndev);

	if (idx || ndev->ieee80211_ptr->iftype != NL80211_IFTYPE_STATION ||
	    !v->vif_added)
		return -ENOENT;

	memcpy(mac, v->bssid, ETH_ALEN);

	return scsc_wlan_get_station(wiphy, wdev, mac, sinfo);
}

/*
 * Keys.
 *
 * cfg80211 hands these over during and after the 4-way handshake: the
 * pairwise key once it completes, then the group key. A pairwise key is
 * addressed to the peer, a group key to nobody in particular.
 */
static int scsc_wlan_add_key(struct wiphy *wiphy, struct wireless_dev *wdev,
			     int link_id, u8 key_index, bool pairwise,
			     const u8 *mac_addr, struct key_params *params)
{
	struct net_device *ndev = wdev->netdev;
	bool is_ap;
	struct scsc_wlan_vif *v;
	struct scsc_wlan_peer *p = NULL;
	int ret;

	if (!ndev)
		return -EINVAL;

	v = netdev_priv(ndev);
	is_ap = ndev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP;

	if (is_ap && mac_addr) {
		p = scsc_wlan_peer_by_addr(v, mac_addr);
		if (!p)
			return -ENOENT;
	}

	/*
	 * A group key has no address of its own, and the firmware will not
	 * take the broadcast address for one: on a station it wants the
	 * peer the key belongs to, which is the BSS we are joined to, and
	 * an AP wants its own interface address instead. Getting this
	 * wrong costs a MLME-SETKEYS result of 0x0026, which is all the
	 * firmware will say about it.
	 */
	ret = scsc_mx_mlme_set_key(v->wlan->mx, v->vif, key_index, pairwise,
				   mac_addr ? mac_addr :
				   is_ap ? ndev->dev_addr : v->bssid,
				   params->cipher, params->key,
				   params->key_len, params->seq,
				   params->seq_len);
	if (ret)
		return ret;

	/*
	 * The pairwise key is the end of the handshake, and on a secured
	 * network that is when the association gets acknowledged -- the
	 * point of withholding it being that nothing leaves in clear
	 * before there is a key to protect it. On an AP vif that is a
	 * per-station decision instead of a per-vif one.
	 */
	if (pairwise && v->secured) {
		if (p && !p->connected) {
			p->connected = true;
			scsc_mx_mlme_connected_resp(v->wlan->mx, v->vif,
						    p->index);
		} else if (!is_ap && v->roaming) {
			v->roaming = false;
			scsc_mx_mlme_roamed_resp(v->wlan->mx, v->vif);
		} else if (!is_ap) {
			scsc_mx_mlme_connect_resp(v->wlan->mx, v->vif);
		}
	}

	return 0;
}

static int scsc_wlan_del_key(struct wiphy *wiphy, struct wireless_dev *wdev,
			     int link_id, u8 key_index, bool pairwise,
			     const u8 *mac_addr)
{
	/*
	 * The firmware drops a vif's keys when the vif goes away, which
	 * is the only time this driver removes them. Reporting success
	 * for the individual delete keeps cfg80211's teardown quiet
	 * without pretending a signal was sent.
	 */
	return 0;
}

static int scsc_wlan_set_default_key(struct wiphy *wiphy,
				     struct net_device *ndev, int link_id,
				     u8 key_index, bool unicast, bool multicast)
{
	/* The key index is carried in each MLME-SETKEYS.request already. */
	return 0;
}

/* AP mode */

/*
 * The beacon body cfg80211 hands over, minus the fixed fields.
 *
 * settings->beacon.head is a whole management frame: the 802.11 header,
 * then timestamp, beacon interval and capability, then elements. Only
 * the elements go to the firmware, which builds the rest itself.
 *
 * A hidden SSID complicates it. cfg80211 still puts an SSID element in
 * the beacon head -- emptied one of two ways -- and the firmware wants
 * it gone entirely, receiving the real SSID through MLME-START instead.
 * Both forms are a leading element, so both are skipped by length.
 */
static const u8 *scsc_wlan_beacon_head_ies(struct cfg80211_ap_settings *s,
					   u16 *len)
{
	const size_t hdrlen = offsetof(struct ieee80211_mgmt, u.beacon.variable);
	const u8 *ies;
	size_t skip = 0;

	if (s->beacon.head_len < hdrlen)
		return NULL;

	ies = s->beacon.head + hdrlen;
	*len = s->beacon.head_len - hdrlen;

	if (s->hidden_ssid == NL80211_HIDDEN_SSID_ZERO_LEN)
		skip = 2;
	else if (s->hidden_ssid == NL80211_HIDDEN_SSID_ZERO_CONTENTS)
		skip = 2 + s->ssid_len;

	if (skip) {
		if (*len < skip)
			return NULL;
		ies += skip;
		*len -= skip;
	}

	return ies;
}

/*
 * Whether some other vif on this wiphy is already an active AP.
 *
 * The AP <= 1 limit in scsc_wlan_iface_limits[] does not enforce
 * itself: cfg80211_check_combinations() exists for a driver to call,
 * and nothing here does, so it is purely advisory against this driver
 * -- unlike a mac80211-based one, which gets it called automatically
 * from mac80211's own interface-management code. A second AP is not
 * merely unsupported: MLME-START on it never confirms and wedges the
 * whole command channel, proven twice over in wifi.md, so this is
 * enforced directly rather than left to a table nothing consults.
 */
static bool scsc_wlan_ap_active(struct scsc_wlan *wlan,
				const struct scsc_wlan_vif *exclude)
{
	unsigned int i;

	for (i = 1; i <= SCSC_WLAN_MAX_VIFS; i++) {
		struct net_device *ndev = wlan->ndev[i];
		struct scsc_wlan_vif *v;

		if (!ndev)
			continue;

		v = netdev_priv(ndev);
		if (v != exclude && v->ap_started)
			return true;
	}

	return false;
}

/*
 * hostapd's beacon_ies/proberesp_ies/assocresp_ies can carry a leading
 * Extended Capabilities element that duplicates one already in
 * beacon.tail -- the actual "extra" content (WPS, P2P) only starts at
 * the first vendor-specific element after it. Confirmed on air: without
 * stripping it, a real client's mac80211 flags the beacon "corrupt"
 * (elems->parse_error, IEEE80211_BSS_CORRUPT_BEACON) for carrying
 * Extended Capabilities twice. The vendor driver strips it the same way
 * (slsi_ap_prepare_add_info_ies(): "the Extended capability IE needs to
 * be excluded") -- and if there is no vendor-specific element at all,
 * there is nothing here worth sending, which is what a minimal hostapd
 * config (no WPS, no P2P) actually produces.
 */
static void scsc_wlan_add_vendor_ies(struct scsc_mx *mx, u16 vif, u16 purpose,
				     const u8 *ies, size_t len)
{
	const u8 *vendor;

	if (!len)
		return;

	vendor = cfg80211_find_ie(WLAN_EID_VENDOR_SPECIFIC, ies, len);
	if (!vendor)
		return;

	scsc_mx_mlme_add_info_elements(mx, vif, purpose, vendor,
				       len - (vendor - ies));
}

static int scsc_wlan_start_ap(struct wiphy *wiphy, struct net_device *ndev,
			      struct cfg80211_ap_settings *settings)
{
	struct scsc_wlan_vif *v = netdev_priv(ndev);
	const struct ieee80211_mgmt *mgmt;
	const u8 *head_ies;
	u16 head_ies_len, width;
	u8 *ies;
	u16 ies_len;
	int ret;

	if (v->ap_started)
		return -EALREADY;

	if (scsc_wlan_ap_active(v->wlan, v)) {
		netdev_warn(ndev, "refusing a second AP -- known to wedge the firmware\n");
		return -EBUSY;
	}

	mgmt = (const struct ieee80211_mgmt *)settings->beacon.head;
	head_ies = scsc_wlan_beacon_head_ies(settings, &head_ies_len);
	if (!head_ies) {
		netdev_warn(ndev, "malformed beacon head\n");
		return -EINVAL;
	}

	switch (settings->chandef.width) {
	case NL80211_CHAN_WIDTH_20:
	case NL80211_CHAN_WIDTH_20_NOHT:
		width = 20;
		break;
	case NL80211_CHAN_WIDTH_40:
		width = 40;
		break;
	case NL80211_CHAN_WIDTH_80:
		width = 80;
		break;
	default:
		netdev_warn(ndev, "unsupported channel width %d\n",
			    settings->chandef.width);
		return -EOPNOTSUPP;
	}

	/* The beacon elements are the head's and the tail's, in that order. */
	ies_len = head_ies_len + settings->beacon.tail_len;
	ies = kmalloc(ies_len ? ies_len : 1, GFP_KERNEL);
	if (!ies)
		return -ENOMEM;
	memcpy(ies, head_ies, head_ies_len);
	if (settings->beacon.tail_len)
		memcpy(ies + head_ies_len, settings->beacon.tail,
		       settings->beacon.tail_len);

	if (!v->vif_added) {
		ret = scsc_mx_mlme_vif_add(v->wlan->mx, v->vif, ndev->dev_addr,
					   true);
		if (ret)
			goto out;
		v->vif_added = true;
	}

	/*
	 * Extra elements for the frames the firmware generates. These are
	 * additions to what MLME-START carries, so a failure is worth a
	 * complaint but not an abort -- the AP is still usable without
	 * whatever hostapd wanted appended.
	 */
	scsc_wlan_add_vendor_ies(v->wlan->mx, v->vif, SCSC_IE_PURPOSE_BEACON,
				 settings->beacon.beacon_ies,
				 settings->beacon.beacon_ies_len);
	scsc_wlan_add_vendor_ies(v->wlan->mx, v->vif,
				 SCSC_IE_PURPOSE_PROBE_RESPONSE,
				 settings->beacon.proberesp_ies,
				 settings->beacon.proberesp_ies_len);
	scsc_wlan_add_vendor_ies(v->wlan->mx, v->vif,
				 SCSC_IE_PURPOSE_ASSOCIATION_RESPONSE,
				 settings->beacon.assocresp_ies,
				 settings->beacon.assocresp_ies_len);

	scsc_mx_mlme_register_action_frames(v->wlan->mx, v->vif);

	ret = scsc_mx_mlme_start_ap(v->wlan->mx, v->vif, ndev->dev_addr,
				    settings->chandef.chan->center_freq,
				    settings->chandef.center_freq1, width,
				    settings->beacon_interval,
				    settings->dtim_period,
				    le16_to_cpu(mgmt->u.beacon.capab_info),
				    settings->hidden_ssid !=
					    NL80211_HIDDEN_SSID_NOT_IN_USE,
				    settings->ssid, settings->ssid_len,
				    ies, ies_len);
	if (ret) {
		scsc_mx_mlme_vif_del(v->wlan->mx, v->vif);
		v->vif_added = false;
		goto out;
	}

	v->ap_started = true;
	v->secured = settings->privacy;
	netif_carrier_on(ndev);

	netdev_info(ndev, "ap started, ssid %.*s on %d MHz\n",
		    (int)settings->ssid_len, settings->ssid,
		    settings->chandef.chan->center_freq);
out:
	kfree(ies);

	return ret;
}

/*
 * A beacon update. The firmware owns the beacon, so the only part this
 * can change without restarting is the extra elements.
 */
static int scsc_wlan_change_beacon(struct wiphy *wiphy, struct net_device *ndev,
				   struct cfg80211_ap_update *info)
{
	struct scsc_wlan_vif *v = netdev_priv(ndev);
	struct cfg80211_beacon_data *beacon = &info->beacon;

	if (!v->ap_started)
		return -ENETDOWN;

	scsc_wlan_add_vendor_ies(v->wlan->mx, v->vif, SCSC_IE_PURPOSE_BEACON,
				 beacon->beacon_ies, beacon->beacon_ies_len);
	scsc_wlan_add_vendor_ies(v->wlan->mx, v->vif,
				 SCSC_IE_PURPOSE_PROBE_RESPONSE,
				 beacon->proberesp_ies,
				 beacon->proberesp_ies_len);
	scsc_wlan_add_vendor_ies(v->wlan->mx, v->vif,
				 SCSC_IE_PURPOSE_ASSOCIATION_RESPONSE,
				 beacon->assocresp_ies,
				 beacon->assocresp_ies_len);

	return 0;
}

static int scsc_wlan_stop_ap(struct wiphy *wiphy, struct net_device *ndev,
			     unsigned int link_id)
{
	struct scsc_wlan_vif *v = netdev_priv(ndev);

	if (!v->ap_started)
		return 0;

	netif_carrier_off(ndev);
	v->ap_started = false;

	/* Removing the vif is what stops the beacons. */
	if (v->vif_added) {
		scsc_mx_mlme_vif_del(v->wlan->mx, v->vif);
		v->vif_added = false;
	}

	netdev_info(ndev, "ap stopped\n");

	return 0;
}

/*
 * Switch an interface between station and access point.
 *
 * Only while it is idle: the firmware's vif carries the type, so
 * changing it under a live connection would leave the two disagreeing.
 */
static int scsc_wlan_change_iface(struct wiphy *wiphy, struct net_device *ndev,
				  enum nl80211_iftype type,
				  struct vif_params *params)
{
	struct scsc_wlan_vif *v = netdev_priv(ndev);

	if (type != NL80211_IFTYPE_STATION && type != NL80211_IFTYPE_AP)
		return -EOPNOTSUPP;

	if (v->vif_added || v->connecting || v->ap_started)
		return -EBUSY;

	ndev->ieee80211_ptr->iftype = type;

	return 0;
}

static const struct cfg80211_ops scsc_wlan_cfg80211_ops = {
	.start_ap = scsc_wlan_start_ap,
	.change_beacon = scsc_wlan_change_beacon,
	.stop_ap = scsc_wlan_stop_ap,
	.change_virtual_intf = scsc_wlan_change_iface,
	.add_virtual_intf = scsc_wlan_add_virtual_intf,
	.del_virtual_intf = scsc_wlan_del_virtual_intf,
	.scan = scsc_wlan_scan,
	.abort_scan = scsc_wlan_abort_scan,
	.connect = scsc_wlan_connect,
	.disconnect = scsc_wlan_disconnect,
	.external_auth = scsc_wlan_external_auth,
	.mgmt_tx = scsc_wlan_mgmt_tx,
	.add_key = scsc_wlan_add_key,
	.del_key = scsc_wlan_del_key,
	.set_default_key = scsc_wlan_set_default_key,
	.set_power_mgmt = scsc_wlan_set_power_mgmt,
	.get_station = scsc_wlan_get_station,
	.dump_station = scsc_wlan_dump_station,
};

/*
 * Which management frame subtypes cfg80211 will let userspace send
 * through .mgmt_tx or register to receive via cfg80211_rx_mgmt() --
 * both are refused at the cfg80211 layer for anything not listed here,
 * whatever the firmware itself would actually deliver.
 *
 * Authentication frames matter for a reason specific to this driver:
 * WPA3-SAE authenticates at the 802.11 level rather than afterward in
 * a handshake, and this driver runs that exchange from the host (see
 * scsc_wlan_synchronised()), which means wpa_supplicant needs to send
 * and receive raw Authentication frames itself. Without AUTH in both
 * bitmaps here, cfg80211 refuses wpa_supplicant's registration for them
 * outright (net/wireless/mlme.c, cfg80211_mlme_register_mgmt()) and SAE
 * cannot work at all, regardless of anything core.c does correctly.
 */
static const struct ieee80211_txrx_stypes scsc_wlan_mgmt_stypes[NUM_NL80211_IFTYPES] = {
	[NL80211_IFTYPE_STATION] = {
		.tx = BIT(IEEE80211_STYPE_AUTH >> 4) |
		      BIT(IEEE80211_STYPE_ACTION >> 4),
		.rx = BIT(IEEE80211_STYPE_AUTH >> 4) |
		      BIT(IEEE80211_STYPE_ACTION >> 4),
	},
	[NL80211_IFTYPE_AP] = {
		.tx = BIT(IEEE80211_STYPE_ACTION >> 4),
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
		      BIT(IEEE80211_STYPE_PROBE_REQ >> 4),
	},
};


/* Association progress, from the transport's bottom half */

static struct net_device *scsc_wlan_ndev(struct scsc_mx *mx, u16 vif)
{
	struct scsc_wlan *wlan = scsc_mx_wlan(mx);

	if (!wlan || vif < 1 || vif > SCSC_WLAN_MAX_VIFS)
		return NULL;

	return wlan->ndev[vif];
}


/* Find the block-ack session for a peer/TID, or NULL if none applies. */
static struct scsc_wlan_ba_session *scsc_wlan_ba_session(struct scsc_wlan_vif *v,
							  u16 peer_index, u16 tid)
{
	struct scsc_wlan_peer *p;

	if (tid >= SCSC_WLAN_BA_MAX_TID)
		return NULL;

	/*
	 * A station vif always has exactly one peer, the AP, and its own
	 * ba_rx[] for it -- v->peer[] is unused there (see the struct
	 * comment) and cannot be looked up by index safely: 0 is both the
	 * sentinel for "slot free" and, on some firmware, the actual
	 * peer_index a station's own RX traffic carries, so a lookup that
	 * tried v->peer[] first would silently match an empty slot instead
	 * of falling through. Same branch scsc_wlan_tx_work() already uses.
	 */
	if (v->wdev.iftype != NL80211_IFTYPE_AP)
		return &v->ba_rx[tid];

	p = scsc_wlan_peer_by_index(v, peer_index);

	return p ? &p->ba_rx[tid] : NULL;
}

static void scsc_wlan_rx_deliver(struct net_device *ndev, struct sk_buff *skb)
{
	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += skb->len;
	netif_rx(skb);
}

/*
 * Index into a session's buffer for a given sequence number -- fixed
 * relative to start_sn for the life of the session, so it does not shift
 * as expected_sn advances. Mirrors the vendor's SN_TO_INDEX().
 */
static u16 scsc_wlan_ba_index(const struct scsc_wlan_ba_session *s, u16 sn)
{
	return ((sn - s->start_sn) & SCSC_WLAN_BA_SN_MASK) % s->buffer_size;
}

/* Whether sn1 is behind sn2 in the wrapping 12-bit sequence space. */
static bool scsc_wlan_ba_before(u16 sn1, u16 sn2)
{
	return ((sn1 - sn2) & SCSC_WLAN_BA_SN_MASK) > SCSC_WLAN_BA_WINDOW;
}

/*
 * Advance expected_sn to sn, moving any buffered frames between the old and
 * new expected_sn onto ready as they are passed. Used both when a frame
 * arrives beyond the window (the window has to catch up) and by
 * scsc_wlan_blockack() on a firmware-reported error recovery.
 */
static void scsc_wlan_ba_advance(struct scsc_wlan_ba_session *s, u16 sn,
				 struct sk_buff_head *ready)
{
	u16 gap = (sn - s->expected_sn) & SCSC_WLAN_BA_SN_MASK;
	u16 i, j;

	for (j = 0; j < gap && j < s->buffer_size; j++) {
		i = scsc_wlan_ba_index(s, s->expected_sn);
		if (s->buffer[i]) {
			__skb_queue_tail(ready, s->buffer[i]);
			s->buffer[i] = NULL;
			s->occupied--;
		}
		s->expected_sn = (s->expected_sn + 1) & SCSC_WLAN_BA_SN_MASK;
	}
	s->expected_sn = sn;
}

/*
 * Move every buffered frame starting at expected_sn onto ready, for as
 * long as there is no gap.
 */
static void scsc_wlan_ba_drain(struct scsc_wlan_ba_session *s,
			       struct sk_buff_head *ready)
{
	u16 i = scsc_wlan_ba_index(s, s->expected_sn);

	while (s->buffer[i]) {
		__skb_queue_tail(ready, s->buffer[i]);
		s->buffer[i] = NULL;
		s->occupied--;
		s->expected_sn = (s->expected_sn + 1) & SCSC_WLAN_BA_SN_MASK;
		i = scsc_wlan_ba_index(s, s->expected_sn);
	}
}

/*
 * A frame has arrived with sequence number sn. Either it goes straight
 * onto ready (in order, or clearly older than anything the session is
 * tracking -- once a session has moved past a lost retransmission there
 * is nothing useful left to do with a late arrival except pass it up, the
 * same thing the vendor driver does) or it has to wait for what is
 * missing ahead of it, in which case this returns the buffer slot to
 * store it at. Returns -1 when skb has already been consumed (delivered
 * or dropped as a duplicate) and needs no storage.
 */
static int scsc_wlan_ba_consume(struct scsc_wlan_ba_session *s, struct sk_buff *skb,
				 u16 sn, struct sk_buff_head *ready, bool *stop_timer)
{
	int i;

	*stop_timer = false;

	if (((sn - s->expected_sn) & SCSC_WLAN_BA_SN_MASK) <= SCSC_WLAN_BA_WINDOW) {
		u16 edge = (s->expected_sn + s->buffer_size) & SCSC_WLAN_BA_SN_MASK;

		/* Beyond the window: the sender has moved on, so the window has to. */
		if (((sn - edge) & SCSC_WLAN_BA_SN_MASK) <= SCSC_WLAN_BA_WINDOW) {
			u16 new_expected = ((sn - s->buffer_size) & SCSC_WLAN_BA_SN_MASK) + 1;

			new_expected &= SCSC_WLAN_BA_SN_MASK;
			if (s->timer_on)
				*stop_timer = true;
			scsc_wlan_ba_advance(s, new_expected, ready);
		}

		i = -1;
		if (sn == s->expected_sn) {
			if (s->timer_on)
				*stop_timer = true;
			s->expected_sn = (s->expected_sn + 1) & SCSC_WLAN_BA_SN_MASK;
			__skb_queue_tail(ready, skb);
		} else {
			i = scsc_wlan_ba_index(s, sn);
			if (s->buffer[i]) {
				/* Duplicate of a frame already buffered. */
				i = -1;
				kfree_skb(skb);
			}
		}

		if (!scsc_wlan_ba_before(sn, s->highest_sn))
			s->highest_sn = sn;
	} else {
		/*
		 * Clearly behind the window: old, or a retransmit of
		 * something already delivered. Nothing to reorder, pass it
		 * straight up.
		 */
		i = -1;
		__skb_queue_tail(ready, skb);
	}

	return i;
}

static void scsc_wlan_ba_process(struct scsc_wlan_ba_session *s, struct sk_buff *skb, u16 sn)
{
	struct net_device *ndev = s->v->wdev.netdev;
	struct sk_buff_head ready;
	struct sk_buff *r;
	bool stop_timer;
	int i;

	__skb_queue_head_init(&ready);

	spin_lock_bh(&s->lock);

	if (!s->active) {
		spin_unlock_bh(&s->lock);
		scsc_wlan_rx_deliver(ndev, skb);
		return;
	}

	i = scsc_wlan_ba_consume(s, skb, sn, &ready, &stop_timer);
	if (i >= 0) {
		s->buffer[i] = skb;
		s->occupied++;
	}

	scsc_wlan_ba_drain(s, &ready);

	/*
	 * Timer decision, same as the vendor's: start it if there is now a
	 * hole and it was not already running; stop it if the last hole was
	 * just filled; restart it if scsc_wlan_ba_consume() had to slide
	 * the window (whatever it was waiting for is moot now) and a hole
	 * remains.
	 */
	if (!s->timer_on) {
		if (s->occupied) {
			mod_timer(&s->timer, jiffies + msecs_to_jiffies(SCSC_WLAN_BA_AGE_MS));
			s->timer_on = true;
		}
	} else if (!s->occupied) {
		stop_timer = true;
	} else if (stop_timer) {
		mod_timer(&s->timer, jiffies + msecs_to_jiffies(SCSC_WLAN_BA_AGE_MS));
		stop_timer = false;
	}

	if (stop_timer)
		s->timer_on = false;

	spin_unlock_bh(&s->lock);

	/*
	 * timer_delete_sync() has to happen with the lock dropped: the timer
	 * callback takes it too, and running on another CPU it could be
	 * waiting on this same lock right now.
	 */
	if (stop_timer)
		timer_delete_sync(&s->timer);

	while ((r = __skb_dequeue(&ready)))
		scsc_wlan_rx_deliver(ndev, r);
}

/*
 * A hole that never filled in -- give up waiting on it and release
 * whatever comes after, the same policy as the vendor's aging timer.
 */
static void scsc_wlan_ba_age(struct timer_list *t)
{
	struct scsc_wlan_ba_session *s = timer_container_of(s, t, timer);
	struct sk_buff_head ready;
	struct sk_buff *r;
	struct net_device *ndev;
	u16 gap = 1, temp_sn, i, j;

	__skb_queue_head_init(&ready);

	spin_lock(&s->lock);

	s->timer_on = false;

	if (s->active && s->occupied) {
		temp_sn = (s->expected_sn + 1) & SCSC_WLAN_BA_SN_MASK;

		for (j = 0; j < s->buffer_size; j++) {
			i = scsc_wlan_ba_index(s, temp_sn);

			if (s->buffer[i]) {
				while (gap--)
					s->expected_sn = (s->expected_sn + 1) & SCSC_WLAN_BA_SN_MASK;

				__skb_queue_tail(&ready, s->buffer[i]);
				s->buffer[i] = NULL;
				s->occupied--;
				s->expected_sn = (s->expected_sn + 1) & SCSC_WLAN_BA_SN_MASK;
				scsc_wlan_ba_drain(s, &ready);
				break;
			}
			temp_sn = (temp_sn + 1) & SCSC_WLAN_BA_SN_MASK;
			gap++;
		}

		if (s->occupied) {
			mod_timer(&s->timer, jiffies + msecs_to_jiffies(SCSC_WLAN_BA_AGE_MS));
			s->timer_on = true;
		}
	}

	ndev = s->v->wdev.netdev;
	spin_unlock(&s->lock);

	while ((r = __skb_dequeue(&ready)))
		scsc_wlan_rx_deliver(ndev, r);
}

/* One-time setup for a peer's (or a station vif's own) block-ack sessions. */
static void scsc_wlan_ba_setup(struct scsc_wlan_vif *v, struct scsc_wlan_ba_session *rx)
{
	unsigned int tid;

	for (tid = 0; tid < SCSC_WLAN_BA_MAX_TID; tid++) {
		spin_lock_init(&rx[tid].lock);
		timer_setup(&rx[tid].timer, scsc_wlan_ba_age, 0);
		rx[tid].v = v;
	}
}

/* Tear a single session down, delivering whatever it was still holding. */
static void scsc_wlan_ba_stop(struct scsc_wlan_ba_session *s)
{
	struct net_device *ndev;
	struct sk_buff_head flushed;
	struct sk_buff *r;
	bool active;
	unsigned int n;

	/*
	 * A peer slot that has never had scsc_wlan_ba_setup() run on it --
	 * an AP vif's row nothing has ever connected on -- has no lock or
	 * timer to touch, only the zero bytes alloc_netdev() gave it.
	 */
	if (!s->v)
		return;

	ndev = s->v->wdev.netdev;
	__skb_queue_head_init(&flushed);

	spin_lock_bh(&s->lock);
	active = s->active;
	if (active) {
		for (n = 0; n < s->buffer_size && s->occupied; n++) {
			u16 i = scsc_wlan_ba_index(s, s->expected_sn);

			if (s->buffer[i]) {
				__skb_queue_tail(&flushed, s->buffer[i]);
				s->buffer[i] = NULL;
				s->occupied--;
			}
			s->expected_sn = (s->expected_sn + 1) & SCSC_WLAN_BA_SN_MASK;
		}
		s->active = false;
	}
	spin_unlock_bh(&s->lock);

	if (!active)
		return;

	if (s->timer_on) {
		s->timer_on = false;
		timer_delete_sync(&s->timer);
	}

	kfree(s->buffer);
	s->buffer = NULL;

	while ((r = __skb_dequeue(&flushed)))
		scsc_wlan_rx_deliver(ndev, r);
}

static void scsc_wlan_ba_stop_all(struct scsc_wlan_ba_session *rx)
{
	unsigned int tid;

	for (tid = 0; tid < SCSC_WLAN_BA_MAX_TID; tid++)
		scsc_wlan_ba_stop(&rx[tid]);
}

void scsc_wlan_rx_data(struct scsc_mx *mx, u16 vif, const u8 *frame,
		       size_t len)
{
	struct net_device *ndev = scsc_wlan_ndev(mx, vif);
	struct sk_buff *skb;

	if (!ndev || len < ETH_HLEN)
		return;

	skb = netdev_alloc_skb(ndev, len);
	if (!skb) {
		ndev->stats.rx_dropped++;
		return;
	}

	skb_put_data(skb, frame, len);
	skb->protocol = eth_type_trans(skb, ndev);

	scsc_wlan_rx_deliver(ndev, skb);
}

void scsc_wlan_rx_unitdata(struct scsc_mx *mx, u16 vif, u16 peer_index,
			   u16 tid, u16 sequence_number,
			   const u8 *frame, size_t len)
{
	struct net_device *ndev = scsc_wlan_ndev(mx, vif);
	struct scsc_wlan_vif *v;
	struct scsc_wlan_ba_session *s;
	struct sk_buff *skb;

	if (!ndev || len < ETH_HLEN)
		return;

	v = netdev_priv(ndev);

	skb = netdev_alloc_skb(ndev, len);
	if (!skb) {
		ndev->stats.rx_dropped++;
		return;
	}

	skb_put_data(skb, frame, len);
	skb->protocol = eth_type_trans(skb, ndev);

	/* Broadcast/multicast has no per-station sequence space to reorder. */
	s = is_multicast_ether_addr(eth_hdr(skb)->h_dest) ?
		NULL : scsc_wlan_ba_session(v, peer_index, tid);

	if (!s) {
		scsc_wlan_rx_deliver(ndev, skb);
		return;
	}

	scsc_wlan_ba_process(s, skb, sequence_number & SCSC_WLAN_BA_SN_MASK);
}

/* MA-BLOCKACK.indication, receive direction -- start, end or reset a session. */
void scsc_wlan_blockack(struct scsc_mx *mx, u16 vif, const u8 *peer_addr,
			u16 tid, u16 buffer_size, u16 sequence_number, u16 reason)
{
	struct net_device *ndev = scsc_wlan_ndev(mx, vif);
	struct scsc_wlan_vif *v;
	struct scsc_wlan_peer *p;
	struct scsc_wlan_ba_session *s;
	struct sk_buff_head ready;
	struct sk_buff *r;

	if (!ndev || tid >= SCSC_WLAN_BA_MAX_TID)
		return;

	v = netdev_priv(ndev);

	if (ndev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP) {
		p = scsc_wlan_peer_by_addr(v, peer_addr);
		if (!p)
			return;
		s = &p->ba_rx[tid];
	} else {
		s = &v->ba_rx[tid];
	}

	switch (reason) {
	case SCSC_BLOCKACK_START:
		if (!buffer_size || buffer_size > SCSC_WLAN_BA_BUFFER_MAX)
			return;

		spin_lock_bh(&s->lock);
		if (s->active && s->buffer_size == buffer_size &&
		    s->expected_sn == sequence_number) {
			/* Same parameters as the running session -- keep it. */
			spin_unlock_bh(&s->lock);
			return;
		}
		spin_unlock_bh(&s->lock);

		scsc_wlan_ba_stop(s);

		s->buffer = kcalloc(buffer_size, sizeof(*s->buffer), GFP_KERNEL);
		if (!s->buffer)
			return;

		spin_lock_bh(&s->lock);
		s->buffer_size = buffer_size;
		s->start_sn = sequence_number;
		s->expected_sn = sequence_number;
		s->highest_sn = 0;
		s->occupied = 0;
		s->active = true;
		spin_unlock_bh(&s->lock);
		break;

	case SCSC_BLOCKACK_END:
		scsc_wlan_ba_stop(s);
		break;

	case SCSC_BLOCKACK_RESET:
		__skb_queue_head_init(&ready);

		spin_lock_bh(&s->lock);
		if (s->active &&
		    ((sequence_number - s->expected_sn) & SCSC_WLAN_BA_SN_MASK) <=
		    SCSC_WLAN_BA_WINDOW) {
			scsc_wlan_ba_advance(s, sequence_number, &ready);
			scsc_wlan_ba_drain(s, &ready);
		}
		spin_unlock_bh(&s->lock);

		while ((r = __skb_dequeue(&ready)))
			scsc_wlan_rx_deliver(ndev, r);
		break;

	default:
		break;
	}
}

void scsc_wlan_connect_result(struct scsc_mx *mx, u16 vif, bool success,
			      const u8 *req_ie, u16 req_ie_len,
			      const u8 *resp_ie, u16 resp_ie_len)
{
	struct net_device *ndev = scsc_wlan_ndev(mx, vif);
	struct scsc_wlan_vif *v;

	if (!ndev)
		return;

	v = netdev_priv(ndev);
	if (!v->connecting)
		return;
	v->connecting = false;

	/*
	 * For a station the association is the connect confirm: there is no
	 * later "you are up now" signal. MLME_CONNECTED_IND belongs to AP
	 * mode, where it announces a peer rather than the link.
	 */
	if (success) {
		netif_carrier_on(ndev);
		v->connected_jiffies = jiffies;
		v->qos = cfg80211_find_ie(WLAN_EID_HT_CAPABILITY, resp_ie,
					  resp_ie_len) ||
			 cfg80211_find_ie(WLAN_EID_VHT_CAPABILITY, resp_ie,
					  resp_ie_len) ||
			 cfg80211_find_vendor_ie(WLAN_OUI_MICROSOFT,
						 WLAN_OUI_TYPE_MICROSOFT_WMM,
						 resp_ie, resp_ie_len);

		/*
		 * Open the data path. On a secured network this waits for
		 * the pairwise key instead, so that nothing goes out in
		 * clear between associating and finishing the handshake.
		 */
		if (!v->secured)
			scsc_mx_mlme_connect_resp(v->wlan->mx, v->vif);
	}

	/*
	 * The elements matter on a secured network: the supplicant builds
	 * message 2 of the handshake from the request elements reported
	 * here, and the AP checks them against what it received.
	 */
	cfg80211_connect_result(ndev, v->bssid, req_ie, req_ie_len,
				resp_ie, resp_ie_len,
				success ? WLAN_STATUS_SUCCESS :
					  WLAN_STATUS_UNSPECIFIED_FAILURE,
				GFP_ATOMIC);
}

void scsc_wlan_peer_started(struct scsc_mx *mx, u16 vif, u16 peer_index,
			    const u8 *addr, const u8 *ie, u16 ie_len)
{
	struct net_device *ndev = scsc_wlan_ndev(mx, vif);
	struct scsc_wlan_vif *v;
	struct scsc_wlan_peer *p;

	if (!ndev)
		return;

	v = netdev_priv(ndev);

	/* A re-association reuses the peer_index; anything else needs a slot. */
	p = scsc_wlan_peer_by_index(v, peer_index);
	if (p) {
		/*
		 * Reused without an intervening disconnect -- any block-ack
		 * sessions from the old association are stale (wrong
		 * sequence-number space) and their lock/timer are already
		 * live, so they have to be stopped, not just overwritten,
		 * before scsc_wlan_ba_setup() runs timer_setup() again.
		 */
		scsc_wlan_ba_stop_all(p->ba_rx);
	} else {
		p = scsc_wlan_peer_by_index(v, 0);
	}
	if (!p) {
		netdev_warn(ndev, "wlan: no room for peer index %u\n",
			    peer_index);
		return;
	}

	memcpy(p->addr, addr, ETH_ALEN);
	p->index = peer_index;
	p->connected = false;
	scsc_wlan_ba_setup(v, p->ba_rx);

	if (ie_len > SCSC_WLAN_PEER_IE_MAX)
		ie_len = SCSC_WLAN_PEER_IE_MAX;
	p->assoc_ie_len = ie_len;
	memcpy(p->assoc_ie, ie, ie_len);
}

void scsc_wlan_connected(struct scsc_mx *mx, u16 vif, u16 peer_index)
{
	struct net_device *ndev = scsc_wlan_ndev(mx, vif);
	struct scsc_wlan_vif *v;
	struct scsc_wlan_peer *p;
	struct station_info sinfo = {};

	if (!ndev)
		return;

	v = netdev_priv(ndev);

	/*
	 * peer_index 0 is a station vif, where the link coming up is the
	 * connect result instead and this indication does no more than
	 * mark the interface live.
	 */
	if (!peer_index) {
		netif_carrier_on(ndev);
		return;
	}

	p = scsc_wlan_peer_by_index(v, peer_index);
	if (!p) {
		netdev_warn(ndev, "wlan: connected peer index %u unknown\n",
			    peer_index);
		return;
	}

	/*
	 * hostapd reads its RSN element out of this to decide the station
	 * wants WPA2 and start the handshake -- an empty sinfo here leaves
	 * it thinking the station is open, and it never sends anything.
	 */
	sinfo.assoc_req_ies = p->assoc_ie;
	sinfo.assoc_req_ies_len = p->assoc_ie_len;

	cfg80211_new_sta(&v->wdev, p->addr, &sinfo, GFP_ATOMIC);

	/*
	 * Open the data path. On a secured network this waits for the
	 * pairwise key instead, so that nothing goes out in clear before
	 * the handshake finishes -- see scsc_wlan_add_key().
	 */
	if (!v->secured) {
		p->connected = true;
		scsc_mx_mlme_connected_resp(v->wlan->mx, v->vif, peer_index);
	}
}

void scsc_wlan_disconnected(struct scsc_mx *mx, u16 vif, const u8 *addr,
			    u16 reason)
{
	struct net_device *ndev = scsc_wlan_ndev(mx, vif);
	struct scsc_wlan_vif *v;

	if (!ndev)
		return;

	v = netdev_priv(ndev);

	/*
	 * An AP vif keeps beaconing after one station leaves -- there is no
	 * single link to tear down, just a row in the peer table and a
	 * notification to userspace.
	 */
	if (ndev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP) {
		struct scsc_wlan_peer *p = scsc_wlan_peer_by_addr(v, addr);

		if (!p)
			return;

		cfg80211_del_sta(&v->wdev, p->addr, GFP_ATOMIC);
		scsc_wlan_ba_stop_all(p->ba_rx);
		memset(p, 0, sizeof(*p));
		return;
	}

	scsc_wlan_ba_stop_all(v->ba_rx);
	netif_carrier_off(ndev);

	/*
	 * A failure during association is reported as a connect result,
	 * not a disconnection: cfg80211 is still waiting for the outcome
	 * of the connect it asked for.
	 */
	if (v->connecting) {
		v->connecting = false;
		cfg80211_connect_result(ndev, v->bssid, NULL, 0, NULL, 0,
					WLAN_STATUS_UNSPECIFIED_FAILURE,
					GFP_ATOMIC);
		return;
	}

	cfg80211_disconnected(ndev, reason, NULL, 0, false, GFP_ATOMIC);
}

void scsc_wlan_roamed(struct scsc_mx *mx, u16 vif, bool temporal_keys_required,
		      const u8 *bssid, const u8 *req_ie, u16 req_ie_len,
		      const u8 *resp_ie, u16 resp_ie_len)
{
	struct net_device *ndev = scsc_wlan_ndev(mx, vif);
	struct scsc_wlan_vif *v;
	struct cfg80211_roam_info info = {};

	if (!ndev)
		return;

	v = netdev_priv(ndev);

	/*
	 * A roam only makes sense once already connected; a roam signal
	 * arriving for a vif that is not is not something to act on.
	 */
	if (!netif_carrier_ok(ndev))
		return;

	/*
	 * The new BSS has its own sequence-number space -- any block-ack
	 * session from the old one would just misinterpret it.
	 */
	scsc_wlan_ba_stop_all(v->ba_rx);

	memcpy(v->bssid, bssid, ETH_ALEN);

	info.req_ie = req_ie;
	info.req_ie_len = req_ie_len;
	info.resp_ie = resp_ie;
	info.resp_ie_len = resp_ie_len;
	info.links[0].bssid = bssid;

	/*
	 * cfg80211_roamed() resolves this to a cfg80211_bss itself if the
	 * bss pointer here is NULL, but only by looking one up that is
	 * already known -- and every BSS the firmware has ever reported is
	 * already in cfg80211's cache unconditionally, scan or not (see
	 * scsc_wlan_scan_result()), including the one just roamed to: the
	 * firmware always sends an MLME-SCAN.indication for a BSS right
	 * before joining it, roam or fresh connect alike.
	 */
	cfg80211_roamed(ndev, &info, GFP_ATOMIC);

	/*
	 * Same withholding as a fresh connect -- see
	 * scsc_mx_mlme_connect_resp() -- except the decision on whether a
	 * new pairwise key is needed first comes from the firmware this
	 * time, in MLME_ROAMED_IND, rather than always being true.
	 */
	if (temporal_keys_required)
		v->roaming = true;
	else
		scsc_mx_mlme_roamed_resp(v->wlan->mx, v->vif);
}

void scsc_wlan_channel_switched(struct scsc_mx *mx, u16 vif, u32 freq_mhz,
				u32 center_freq1_mhz, u8 width_mhz)
{
	struct net_device *ndev = scsc_wlan_ndev(mx, vif);
	struct scsc_wlan_vif *v;
	struct cfg80211_chan_def chandef = {};

	if (!ndev)
		return;

	v = netdev_priv(ndev);

	chandef.chan = ieee80211_get_channel(v->wlan->wiphy, freq_mhz);
	if (!chandef.chan) {
		netdev_warn(ndev, "wlan: channel switch to unknown %u MHz\n",
			   freq_mhz);
		return;
	}

	switch (width_mhz) {
	case 20:
		chandef.width = NL80211_CHAN_WIDTH_20;
		break;
	case 40:
		chandef.width = NL80211_CHAN_WIDTH_40;
		break;
	case 80:
		chandef.width = NL80211_CHAN_WIDTH_80;
		break;
	case 160:
		chandef.width = NL80211_CHAN_WIDTH_160;
		break;
	default:
		netdev_warn(ndev, "wlan: channel switch, unknown width %u MHz\n",
			   width_mhz);
		return;
	}
	chandef.center_freq1 = center_freq1_mhz;

	wiphy_lock(v->wlan->wiphy);
	cfg80211_ch_switch_notify(ndev, &chandef, 0);
	wiphy_unlock(v->wlan->wiphy);
}

void scsc_wlan_synchronised(struct scsc_mx *mx, u16 vif, const u8 *bssid)
{
	struct net_device *ndev = scsc_wlan_ndev(mx, vif);
	struct scsc_wlan_vif *v;
	struct cfg80211_external_auth_params auth = {};

	if (!ndev)
		return;

	v = netdev_priv(ndev);

	auth.action = NL80211_EXTERNAL_AUTH_START;
	memcpy(auth.bssid, bssid, ETH_ALEN);
	memcpy(auth.ssid.ssid, v->ssid, v->ssid_len);
	auth.ssid.ssid_len = v->ssid_len;
	auth.key_mgmt_suite = WLAN_AKM_SUITE_SAE;

	if (cfg80211_external_auth_request(ndev, &auth, GFP_ATOMIC))
		netdev_warn(ndev, "wlan: external auth request refused\n");
}

void scsc_wlan_frame_transmission(struct scsc_mx *mx, u16 vif, u16 host_tag,
				  bool ack)
{
	struct net_device *ndev = scsc_wlan_ndev(mx, vif);
	struct scsc_wlan_vif *v;

	if (!ndev)
		return;

	v = netdev_priv(ndev);

	if (!v->mgmt_tx.pending || v->mgmt_tx.host_tag != host_tag)
		return;

	v->mgmt_tx.pending = false;
	cfg80211_mgmt_tx_status(&v->wdev, v->mgmt_tx.cookie, v->mgmt_tx.frame,
				v->mgmt_tx.len, ack, GFP_ATOMIC);
}

void scsc_wlan_rx_mgmt(struct scsc_mx *mx, u16 vif, u32 freq_mhz,
		       const u8 *frame, size_t len)
{
	struct net_device *ndev = scsc_wlan_ndev(mx, vif);
	struct scsc_wlan_vif *v;

	if (!ndev)
		return;

	v = netdev_priv(ndev);

	cfg80211_rx_mgmt(&v->wdev, freq_mhz, 0, frame, len, 0);
}

/* Results, arriving from the transport's bottom half */

void scsc_wlan_scan_result(struct scsc_mx *mx, u16 freq_mhz, s16 rssi,
			   const u8 *frame, size_t len)
{
	struct scsc_wlan *wlan = scsc_mx_wlan(mx);
	struct ieee80211_channel *chan;
	struct cfg80211_bss *bss;

	if (!wlan)
		return;

	chan = ieee80211_get_channel(wlan->wiphy, freq_mhz);
	if (!chan)
		return;

	/* cfg80211 wants mBm, the firmware reports whole dBm. */
	bss = cfg80211_inform_bss_frame(wlan->wiphy, chan,
					(struct ieee80211_mgmt *)frame, len,
					rssi * 100, GFP_ATOMIC);
	if (bss)
		cfg80211_put_bss(wlan->wiphy, bss);
}

void scsc_wlan_scan_done(struct scsc_mx *mx, bool aborted)
{
	struct scsc_wlan *wlan = scsc_mx_wlan(mx);
	struct cfg80211_scan_info info = { .aborted = aborted };
	struct cfg80211_scan_request *req;

	if (!wlan)
		return;

	spin_lock_bh(&wlan->lock);
	req = wlan->scan_req;
	wlan->scan_req = NULL;
	spin_unlock_bh(&wlan->lock);

	if (!req)
		return;

	/* cfg80211_scan_done() asserts the wiphy mutex. */
	wiphy_lock(wlan->wiphy);
	cfg80211_scan_done(req, &info);
	wiphy_unlock(wlan->wiphy);
}

/* Setup and teardown */

int scsc_wlan_register(struct scsc_mx *mx)
{
	struct device *dev = scsc_mx_dev(mx);
	struct scsc_wlan *wlan;
	struct wireless_dev *wdev;
	struct wiphy *wiphy;
	int ret;

	wiphy = wiphy_new(&scsc_wlan_cfg80211_ops, sizeof(*wlan));
	if (!wiphy)
		return -ENOMEM;

	wlan = wiphy_priv(wiphy);
	wlan->mx = mx;
	wlan->wiphy = wiphy;
	spin_lock_init(&wlan->lock);

	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) |
				 BIT(NL80211_IFTYPE_AP);
	wiphy->iface_combinations = scsc_wlan_iface_combinations;
	wiphy->n_iface_combinations = ARRAY_SIZE(scsc_wlan_iface_combinations);
	wiphy->cipher_suites = scsc_wlan_cipher_suites;
	wiphy->n_cipher_suites = ARRAY_SIZE(scsc_wlan_cipher_suites);
	wiphy->max_scan_ssids = 1;
	wiphy->max_scan_ie_len = 0;
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	wiphy->mgmt_stypes = scsc_wlan_mgmt_stypes;
	/*
	 * WPA3-SAE with the SME (the actual Commit/Confirm exchange) in
	 * user space -- NL80211_FEATURE_SAE, not the newer
	 * NL80211_EXT_FEATURE_SAE_OFFLOAD, which is the other model where
	 * the password itself is handed to the driver and the firmware
	 * does the exchange. This firmware does not: see
	 * scsc_wlan_synchronised().
	 */
	wiphy->features |= NL80211_FEATURE_SAE;
	/*
	 * One antenna each way -- confirmed, not assumed: the VHT/HT MCS
	 * maps this driver advertises (scsc_wlan_build_bands()) both cap
	 * out at a single spatial stream, matching the vendor's own
	 * hardcoded default for this chip family. No .get_antenna/
	 * .set_antenna: there is nothing to switch between on a 1x1 part.
	 */
	wiphy->available_antennas_tx = 1;
	wiphy->available_antennas_rx = 1;
	set_wiphy_dev(wiphy, dev);

	ret = scsc_wlan_build_bands(wlan);
	if (ret)
		goto err_wiphy;

	ret = wiphy_register(wiphy);
	if (ret)
		goto err_bands;

	/*
	 * Tell the kernel's regulatory core what the firmware was told.
	 * Without this the two disagree -- the chip is in one domain and
	 * cfg80211 still has the world one, which leaves every channel
	 * flagged no-IR in `iw reg get` however the firmware is set up.
	 * It has to follow wiphy_register().
	 */
	if (scsc_mx_country_code())
		regulatory_hint(wiphy, scsc_mx_country_code());

	/*
	 * Published before the first interface exists, because
	 * scsc_wlan_add_iface() registers a netdev and cfg80211 can call
	 * back into us the moment it does.
	 */
	scsc_mx_set_wlan(mx, wlan);

	wdev = scsc_wlan_add_iface(wlan, "wlan%d", NET_NAME_ENUM,
				   NL80211_IFTYPE_STATION, false);
	if (IS_ERR(wdev)) {
		ret = PTR_ERR(wdev);
		goto err_published;
	}

	return 0;

err_published:
	scsc_mx_set_wlan(mx, NULL);
	wiphy_unregister(wiphy);
err_bands:
	kfree(wlan->chan2);
	kfree(wlan->chan5);
err_wiphy:
	wiphy_free(wiphy);

	return ret;
}

void scsc_wlan_unregister(struct scsc_mx *mx)
{
	struct scsc_wlan *wlan = scsc_mx_wlan(mx);
	unsigned int vif;

	if (!wlan)
		return;

	/* Nothing may report a result once the wiphy is gone. */
	scsc_mx_set_wlan(mx, NULL);

	/*
	 * The netdevs free themselves (needs_free_netdev), so this only
	 * has to unregister them -- and it has to happen before the wiphy
	 * goes, since each one holds a wdev pointing at it.
	 */
	for (vif = 1; vif <= SCSC_WLAN_MAX_VIFS; vif++) {
		if (!wlan->ndev[vif])
			continue;
		unregister_netdev(wlan->ndev[vif]);
		wlan->ndev[vif] = NULL;
	}

	wiphy_unregister(wlan->wiphy);
	kfree(wlan->chan2);
	kfree(wlan->chan5);
	wiphy_free(wlan->wiphy);
}
