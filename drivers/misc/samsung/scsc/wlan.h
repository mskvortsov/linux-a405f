/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The seam between the WLBT core (core.c) and the cfg80211 driver
 * (wlan.c).
 *
 * struct scsc_mx stays private to core.c and struct scsc_wlan private
 * to wlan.c: each side sees the other only as a pointer, and everything
 * that crosses does so through the calls below. That is a deliberate
 * constraint rather than tidiness -- the two halves belong in different
 * subsystems (drivers/misc and drivers/net/wireless) and will have to
 * be split into separate modules before this goes anywhere, so the less
 * they know about each other's layout the smaller that job is.
 */

#ifndef __SCSC_WLAN_H__
#define __SCSC_WLAN_H__

#include <linux/types.h>

struct device;
struct scsc_mx;
struct scsc_wlan;

/*
 * Firmware virtual interfaces this driver will use, numbered from 1.
 * Both halves index arrays by vif, so they have to agree on the bound.
 */
#define SCSC_WLAN_MAX_VIFS	2

/* wlan.c, called by the core */

int scsc_wlan_register(struct scsc_mx *mx);
void scsc_wlan_unregister(struct scsc_mx *mx);

/*
 * One scan result, as a whole 802.11 management frame. Called from the
 * transport's bottom half with the transport lock held, so it must not
 * block -- cfg80211_inform_bss_frame() only takes the BSS spinlock, so
 * that is fine, but cfg80211_scan_done() wants the wiphy mutex and is
 * therefore deferred to scsc_wlan_scan_done() once the lock is dropped.
 */
void scsc_wlan_scan_result(struct scsc_mx *mx, u16 freq_mhz, s16 rssi,
			   const u8 *frame, size_t len);
void scsc_wlan_scan_done(struct scsc_mx *mx, bool aborted);

/*
 * Association progress, in the order the firmware reports it. Unlike
 * the scan these are safe to call with the transport lock held:
 * cfg80211's notifiers queue an event under wdev->event_lock rather
 * than taking the wiphy mutex.
 */
/*
 * The association elements are the ones the firmware actually put on the
 * air, not the ones the supplicant asked for -- it rewrites parts of the
 * RSN element -- and reporting them is what lets a WPA2 handshake agree
 * with the AP. See scsc_mx_assoc_ies().
 */
void scsc_wlan_connect_result(struct scsc_mx *mx, u16 vif, bool success,
			      const u8 *req_ie, u16 req_ie_len,
			      const u8 *resp_ie, u16 resp_ie_len);
void scsc_wlan_rx_data(struct scsc_mx *mx, u16 vif, const u8 *frame,
		       size_t len);
/*
 * A received 802.3 frame off the data path -- MA-UNITDATA.indication,
 * as opposed to the individual control-plane frames scsc_wlan_rx_data()
 * carries, which were never part of a block-ack sequence and so never
 * need reordering. tid and sequence_number are the frame's QoS TID and
 * 802.11 sequence number; if a block-ack session is running for this
 * peer/tid (see scsc_wlan_blockack()) the frame may be held back for
 * reordering rather than delivered immediately.
 */
void scsc_wlan_rx_unitdata(struct scsc_mx *mx, u16 vif, u16 peer_index,
			   u16 tid, u16 sequence_number,
			   const u8 *frame, size_t len);
/*
 * MA-BLOCKACK.indication, receive direction only -- the transmit
 * direction only reports capabilities (A-MSDU-in-A-MPDU) this driver's TX
 * path does not use. tid and buffer_size are already unpacked from the
 * firmware's packed blockack_parameter_set; reason is one of the
 * SCSC_BLOCKACK_* below, mirroring FAPI_REASONCODE_START/_END/
 * _UNSPECIFIED_REASON -- see the static_asserts next to SCSC_MSGTYPE_* in
 * core.c.
 */
#define SCSC_BLOCKACK_START	0x0029
#define SCSC_BLOCKACK_END	0x0025
#define SCSC_BLOCKACK_RESET	0x0001

void scsc_wlan_blockack(struct scsc_mx *mx, u16 vif, const u8 *peer_addr,
			u16 tid, u16 buffer_size, u16 sequence_number,
			u16 reason);
/*
 * The transport has buffers again after refusing a frame. Called with
 * the transport lock dropped, because waking a netdev queue re-enters
 * ndo_start_xmit.
 */
void scsc_wlan_tx_resume(struct scsc_mx *mx);
/*
 * A station has started associating to an AP vif. addr is its address and
 * ie/ie_len its association request elements, both read out of the
 * request frame -- MLME-PROCEDURE-STARTED carries neither directly, only
 * the peer_index the firmware has allocated it. The elements matter: they
 * are what cfg80211_new_sta() hands hostapd, and without an RSN element
 * in them hostapd never learns the station wants WPA2 and does not start
 * the handshake. Not called for a station vif, where peer_index reads 0.
 */
void scsc_wlan_peer_started(struct scsc_mx *mx, u16 vif, u16 peer_index,
			    const u8 *addr, const u8 *ie, u16 ie_len);
/*
 * peer_index is 0 on a station vif -- there the link coming up is the
 * connect result, not this -- and the firmware's per-peer id on an AP
 * vif, identifying a row scsc_wlan_peer_started() already added.
 */
void scsc_wlan_connected(struct scsc_mx *mx, u16 vif, u16 peer_index);
/*
 * addr is only meaningful on an AP vif, where it says which peer left;
 * NULL on a station vif.
 */
void scsc_wlan_disconnected(struct scsc_mx *mx, u16 vif, const u8 *addr,
			    u16 reason);
/*
 * The firmware roamed a station to a new BSS on its own. bssid is the
 * new AP's address; req_ie/resp_ie are the association elements, same
 * meaning as scsc_wlan_connect_result()'s. temporal_keys_required says
 * whether the new BSS needs a fresh pairwise key before the roam is
 * acknowledged -- see scsc_mx_mlme_roamed_resp().
 */
void scsc_wlan_roamed(struct scsc_mx *mx, u16 vif, bool temporal_keys_required,
		      const u8 *bssid, const u8 *req_ie, u16 req_ie_len,
		      const u8 *resp_ie, u16 resp_ie_len);
/*
 * A channel move, on a station or an AP vif alike. width_mhz is the new
 * channel width, decoded from the firmware's packed encoding; freq_mhz
 * and center_freq1_mhz are both already in MHz.
 */
void scsc_wlan_channel_switched(struct scsc_mx *mx, u16 vif, u32 freq_mhz,
				u32 center_freq1_mhz, u8 width_mhz);
/*
 * The firmware needs the host to run a WPA3-SAE authentication itself
 * -- see scsc_mx_mlme_connect()'s sae parameter and
 * scsc_wlan_external_auth(). bssid is who to authenticate against.
 */
void scsc_wlan_synchronised(struct scsc_mx *mx, u16 vif, const u8 *bssid);
/*
 * The outcome of a frame sent through scsc_mx_mlme_send_mgmt_frame(),
 * matched by the host_tag it returned. ack is whether the peer
 * acknowledged it -- cfg80211's own meaning for a management frame,
 * not an 802.11 term the firmware uses elsewhere in this driver.
 */
void scsc_wlan_frame_transmission(struct scsc_mx *mx, u16 vif, u16 host_tag,
				  bool ack);
/*
 * A management frame the firmware could not place elsewhere -- not a
 * scan result, not an association step it already reports through a
 * dedicated signal. freq_mhz is where it was received.
 */
void scsc_wlan_rx_mgmt(struct scsc_mx *mx, u16 vif, u32 freq_mhz,
		       const u8 *frame, size_t len);

/* core.c, called by the cfg80211 driver */

struct device *scsc_mx_dev(struct scsc_mx *mx);
struct scsc_wlan *scsc_mx_wlan(struct scsc_mx *mx);
void scsc_mx_set_wlan(struct scsc_mx *mx, struct scsc_wlan *wlan);

/*
 * The channel list the firmware reported, as (first channel, count)
 * pairs -- the raw MIB encoding, because expanding it is the caller's
 * business and both callers want it differently.
 */
const u8 *scsc_mx_chanranges(struct scsc_mx *mx, unsigned int *pairs);
/*
 * The firmware's own HT/VHT capability MIBs. The return value is
 * whether HT/VHT is activated at all; *cap is the raw 802.11
 * Capabilities element body (21 bytes for HT, 12 for VHT) either way,
 * meaningful only when it is.
 */
bool scsc_mx_ht_cap(struct scsc_mx *mx, const u8 **cap);
bool scsc_mx_vht_cap(struct scsc_mx *mx, const u8 **cap);
/*
 * RSSI (dBm) and the current TX rate, in the firmware's own encoding --
 * see the comment above the implementation for the bit layout. Queried
 * fresh each call, for .get_station.
 */
int scsc_mx_mlme_get_station_info(struct scsc_mx *mx, u16 vif, s16 *rssi,
				  u16 *rate);
u64 scsc_mx_mac_address(void);
const char *scsc_mx_country_code(void);

/*
 * Start a scan and return once the firmware has accepted it; results
 * arrive later through scsc_wlan_scan_result(). freqs are in MHz.
 * A NULL ssid means the wildcard, which is what makes a scan active.
 */
int scsc_mx_mlme_scan_start(struct scsc_mx *mx, const u8 *ssid, u8 ssid_len,
			    const u32 *freqs, unsigned int nfreqs);
int scsc_mx_mlme_scan_stop(struct scsc_mx *mx);

/* ap selects the firmware's interface type; everything else follows it. */
int scsc_mx_mlme_vif_add(struct scsc_mx *mx, u16 vif, const u8 *addr, bool ap);
int scsc_mx_mlme_vif_del(struct scsc_mx *mx, u16 vif);

/* AP mode. purpose is one of the SCSC_IE_PURPOSE_* below. */
#define SCSC_IE_PURPOSE_BEACON			0x0001
#define SCSC_IE_PURPOSE_PROBE_RESPONSE		0x0002
#define SCSC_IE_PURPOSE_ASSOCIATION_RESPONSE	0x0004

int scsc_mx_mlme_add_info_elements(struct scsc_mx *mx, u16 vif, u16 purpose,
				   const u8 *ies, u16 ies_len);
/*
 * Begin beaconing. Widths are in MHz and frequencies are the primary and
 * the centre of the whole channel, as a cfg80211 chandef gives them; the
 * firmware's own encoding of that is core.c's business. ies is the
 * beacon body without the fixed fields the signal already carries, and
 * ssid is needed only when it is hidden, since then it is not in ies.
 */
int scsc_mx_mlme_start_ap(struct scsc_mx *mx, u16 vif, const u8 *addr,
			  u32 freq_mhz, u32 center_freq_mhz, u16 width_mhz,
			  u16 beacon_period, u16 dtim_period, u16 capability,
			  bool hidden_ssid, const u8 *ssid, u8 ssid_len,
			  const u8 *ies, u16 ies_len);
int scsc_mx_mlme_register_action_frames(struct scsc_mx *mx, u16 vif);

/*
 * message_type for scsc_mx_mlme_send_mgmt_frame(), mirroring
 * FAPI_MESSAGETYPE_IEEE80211_MGMT/_ACTION -- see the static_asserts
 * next to those in core.c.
 */
#define SCSC_MSGTYPE_IEEE80211_ACTION	0x0010
#define SCSC_MSGTYPE_IEEE80211_MGMT	0x0011

/*
 * sae asks for FAPI_AUTHENTICATIONTYPE_SAE instead of the usual Open
 * System -- WPA3-SAE authenticates at the 802.11 level itself, unlike
 * WPA2, which authenticates Open and does its real work afterward in
 * the 4-way handshake. See scsc_wlan_external_auth().
 */
int scsc_mx_mlme_connect(struct scsc_mx *mx, u16 vif, const u8 *bssid,
			 u32 freq_mhz, const u8 *ssid, u8 ssid_len,
			 const u8 *sec_ie, u16 sec_ie_len, bool sae);
/*
 * Acknowledge a successful association. The firmware will not carry data
 * until this arrives, so it is not optional -- see the implementation for
 * why its timing depends on whether the network is secured. Queued, not
 * sent, so it is safe to call from the transport's bottom half.
 */
void scsc_mx_mlme_connect_resp(struct scsc_mx *mx, u16 vif);
/*
 * Accept a station joining an AP vif, by the peer_index the firmware gave
 * it. The AP-mode counterpart of scsc_mx_mlme_connect_resp(): the
 * firmware holds that peer's data path shut until this arrives, timed the
 * same way -- immediately on an open network, or once its pairwise key is
 * in on a secured one. Queued, not sent, for the same reason.
 */
void scsc_mx_mlme_connected_resp(struct scsc_mx *mx, u16 vif, u16 peer_index);
/*
 * Acknowledge a roam, the same way and for the same reason as
 * scsc_mx_mlme_connect_resp() -- see scsc_wlan_roamed().
 */
void scsc_mx_mlme_roamed_resp(struct scsc_mx *mx, u16 vif);
/*
 * Tell the firmware the host-driven authentication scsc_wlan_synchronised()
 * announced is done, success or not -- the firmware holds the connection
 * attempt at authentication until this arrives, the SAE counterpart of
 * scsc_mx_mlme_connect_resp() holding data until the association is
 * acknowledged.
 */
void scsc_mx_mlme_synchronised_resp(struct scsc_mx *mx, u16 vif,
				    const u8 *bssid, bool success);
u16 scsc_mx_eapol_message_type(const u8 *frame, u16 len);
int scsc_mx_mlme_send_frame(struct scsc_mx *mx, u16 vif, u16 msg_type,
			    const u8 *frame, u16 len);
/*
 * Send a whole 802.11 management frame as given -- cfg80211's .mgmt_tx,
 * one layer down. host_tag comes back so the caller can match the
 * eventual scsc_wlan_frame_transmission() to this specific frame.
 */
int scsc_mx_mlme_send_mgmt_frame(struct scsc_mx *mx, u16 vif, u16 msg_type,
				 const u8 *frame, u16 len, u16 *host_tag);
/*
 * priority is FAPI_PRIORITY_CONTENTION for a non-QoS association, or a
 * TID (0-7) for a QoS one -- mirrors FAPI_PRIORITY_QOS_UP0..7's own
 * values, so cfg80211_classify8021d()'s return value needs no
 * translation. See the static_assert next to SCSC_MSGTYPE_* in core.c.
 */
#define SCSC_PRIORITY_CONTENTION	0x8000

/*
 * Send an 802.3 frame on the data path, to the given peer. -ENOBUFS or
 * -ENOSPC means the transport is full rather than broken: stop the queue
 * and wait for scsc_wlan_tx_resume().
 */
int scsc_mx_tx_data(struct scsc_mx *mx, u16 vif, u16 peer_index,
		    const u8 *frame, u16 len, u16 priority);
int scsc_mx_mlme_set_key(struct scsc_mx *mx, u16 vif, u16 key_id,
			 bool pairwise, const u8 *addr, u32 cipher,
			 const u8 *key, u16 key_len, const u8 *seq, u8 seq_len);
int scsc_mx_mlme_disconnect(struct scsc_mx *mx, u16 vif, const u8 *bssid,
			    u16 reason);

/* mode is one of the SCSC_POWERMGT_* below. Station vifs only. */
#define SCSC_POWERMGT_ACTIVE		0x0000
#define SCSC_POWERMGT_POWER_SAVE	0x0001

int scsc_mx_mlme_powermgt(struct scsc_mx *mx, u16 vif, u16 mode);

#endif /* __SCSC_WLAN_H__ */
