/*

  Broadcom B43 wireless driver

  Transmission (TX/RX) related functions.

  Ported to iPXE
  Copyright (c) 2018-2020 Michael Bazzinotti <bazz@bazz1.com>
  Original from Linux kernel 3.18.11

  Copyright (C) 2005 Martin Langer <martin-langer@gmx.de>
  Copyright (C) 2005 Stefano Brivio <stefano.brivio@polimi.it>
  Copyright (C) 2005, 2006 Michael Buesch <m@bues.ch>
  Copyright (C) 2005 Danny van Dyk <kugelfang@gentoo.org>
  Copyright (C) 2005 Andreas Jaggi <andreas.jaggi@waterwave.ch>

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; see the file COPYING.  If not, write to
  the Free Software Foundation, Inc., 51 Franklin Steet, Fifth Floor,
  Boston, MA 02110-1301, USA.

*/

#include "b43_xmit.h"
#include "b43_phy_common.h"
#include "b43_dma.h"
#include "b43_pio.h"

static const struct b43_tx_legacy_rate_phy_ctl_entry b43_tx_legacy_rate_phy_ctl[] = {
	{ B43_CCK_RATE_1MB,	0x0,			0x0 },
	{ B43_CCK_RATE_2MB,	0x0,			0x1 },
	{ B43_CCK_RATE_5MB,	0x0,			0x2 },
	{ B43_CCK_RATE_11MB,	0x0,			0x3 },
	{ B43_OFDM_RATE_6MB,	B43_TXH_PHY1_CRATE_1_2,	B43_TXH_PHY1_MODUL_BPSK },
	{ B43_OFDM_RATE_9MB,	B43_TXH_PHY1_CRATE_3_4,	B43_TXH_PHY1_MODUL_BPSK },
	{ B43_OFDM_RATE_12MB,	B43_TXH_PHY1_CRATE_1_2,	B43_TXH_PHY1_MODUL_QPSK },
	{ B43_OFDM_RATE_18MB,	B43_TXH_PHY1_CRATE_3_4,	B43_TXH_PHY1_MODUL_QPSK },
	{ B43_OFDM_RATE_24MB,	B43_TXH_PHY1_CRATE_1_2,	B43_TXH_PHY1_MODUL_QAM16 },
	{ B43_OFDM_RATE_36MB,	B43_TXH_PHY1_CRATE_3_4,	B43_TXH_PHY1_MODUL_QAM16 },
	{ B43_OFDM_RATE_48MB,	B43_TXH_PHY1_CRATE_2_3,	B43_TXH_PHY1_MODUL_QAM64 },
	{ B43_OFDM_RATE_54MB,	B43_TXH_PHY1_CRATE_3_4,	B43_TXH_PHY1_MODUL_QAM64 },
};

static const struct b43_tx_legacy_rate_phy_ctl_entry *
b43_tx_legacy_rate_phy_ctl_ent(u8 bitrate)
{
	const struct b43_tx_legacy_rate_phy_ctl_entry *e;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(b43_tx_legacy_rate_phy_ctl); i++) {
		e = &(b43_tx_legacy_rate_phy_ctl[i]);
		if (e->bitrate == bitrate)
			return e;
	}

	B43_WARN_ON(1);
	return NULL;
}

/* Extract the bitrate index out of a CCK PLCP header. */
static int b43_plcp_get_bitrate_idx_cck(struct b43_plcp_hdr6 *plcp)
{
	switch (plcp->raw[0]) {
	case 0x0A:
		return 0;
	case 0x14:
		return 1;
	case 0x37:
		return 2;
	case 0x6E:
		return 3;
	}
	return -1;
}

/* Extract the bitrate index out of an OFDM PLCP header. */
static int b43_plcp_get_bitrate_idx_ofdm(struct b43_plcp_hdr6 *plcp, bool ghz5)
{
	/* For 2 GHz band first OFDM rate is at index 4, see main.c */
	int base = ghz5 ? 0 : 4;

	switch (plcp->raw[0] & 0xF) {
	case 0xB:
		return base + 0;
	case 0xF:
		return base + 1;
	case 0xA:
		return base + 2;
	case 0xE:
		return base + 3;
	case 0x9:
		return base + 4;
	case 0xD:
		return base + 5;
	case 0x8:
		return base + 6;
	case 0xC:
		return base + 7;
	}
	return -1;
}

u8 b43_plcp_get_ratecode_cck(const u8 bitrate)
{
	switch (bitrate) {
	case B43_CCK_RATE_1MB:
		return 0x0A;
	case B43_CCK_RATE_2MB:
		return 0x14;
	case B43_CCK_RATE_5MB:
		return 0x37;
	case B43_CCK_RATE_11MB:
		return 0x6E;
	}
	B43_WARN_ON(1);
	return 0;
}

u8 b43_plcp_get_ratecode_ofdm(const u8 bitrate)
{
	switch (bitrate) {
	case B43_OFDM_RATE_6MB:
		return 0xB;
	case B43_OFDM_RATE_9MB:
		return 0xF;
	case B43_OFDM_RATE_12MB:
		return 0xA;
	case B43_OFDM_RATE_18MB:
		return 0xE;
	case B43_OFDM_RATE_24MB:
		return 0x9;
	case B43_OFDM_RATE_36MB:
		return 0xD;
	case B43_OFDM_RATE_48MB:
		return 0x8;
	case B43_OFDM_RATE_54MB:
		return 0xC;
	}
	B43_WARN_ON(1);
	return 0;
}

void b43_generate_plcp_hdr(struct b43_plcp_hdr4 *plcp,
			   const u16 octets, const u8 bitrate)
{
	__u8 *raw = plcp->raw;
	//DBGC(plcp, "generate_plcp_hdr; ");

	if (b43_is_ofdm_rate(bitrate)) {
		u32 d;

		d = b43_plcp_get_ratecode_ofdm(bitrate);
		B43_WARN_ON(octets & 0xF000);
		d |= (octets << 5);
		plcp->data = cpu_to_le32(d);
	} else {
		u32 plen;

		plen = octets * 16 / bitrate;
		if ((octets * 16 % bitrate) > 0) {
			plen++;
			if ((bitrate == B43_CCK_RATE_11MB)
			    && ((octets * 8 % 11) < 4)) {
				raw[1] = 0x84;
			} else
				raw[1] = 0x04;
		} else
			raw[1] = 0x04;
		plcp->data |= cpu_to_le32(plen << 16);
		raw[0] = b43_plcp_get_ratecode_cck(bitrate);
	}
}

/* TODO: verify if needed for SSLPN or LCN  */
static u16 b43_generate_tx_phy_ctl1(struct b43_wldev *dev, u8 bitrate)
{
	const struct b43_phy *phy = &dev->phy;
	const struct b43_tx_legacy_rate_phy_ctl_entry *e;
	u16 control = 0;
	u16 bw;

	//DBGC(dev, "generate_tx_pgt_ctl1; ");

	if (phy->type == B43_PHYTYPE_LP)
		bw = B43_TXH_PHY1_BW_20;
	else /* FIXME */
		bw = B43_TXH_PHY1_BW_20;

	if (0) { /* FIXME: MIMO */
	} else if (b43_is_cck_rate(bitrate) && phy->type != B43_PHYTYPE_LP) {
		control = bw;
	} else {
		control = bw;
		e = b43_tx_legacy_rate_phy_ctl_ent(bitrate);
		if (e) {
			control |= e->coding_rate;
			control |= e->modulation;
		}
		control |= B43_TXH_PHY1_MODE_SISO;
	}

	return control;
}

static u8 b43_calc_fallback_rate(u8 bitrate)
{
	switch (bitrate) {
	case B43_CCK_RATE_1MB:
		return B43_CCK_RATE_1MB;
	case B43_CCK_RATE_2MB:
		return B43_CCK_RATE_1MB;
	case B43_CCK_RATE_5MB:
		return B43_CCK_RATE_2MB;
	case B43_CCK_RATE_11MB:
		return B43_CCK_RATE_5MB;
	case B43_OFDM_RATE_6MB:
		return B43_CCK_RATE_5MB;
	case B43_OFDM_RATE_9MB:
		return B43_OFDM_RATE_6MB;
	case B43_OFDM_RATE_12MB:
		return B43_OFDM_RATE_9MB;
	case B43_OFDM_RATE_18MB:
		return B43_OFDM_RATE_12MB;
	case B43_OFDM_RATE_24MB:
		return B43_OFDM_RATE_18MB;
	case B43_OFDM_RATE_36MB:
		return B43_OFDM_RATE_24MB;
	case B43_OFDM_RATE_48MB:
		return B43_OFDM_RATE_36MB;
	case B43_OFDM_RATE_54MB:
		return B43_OFDM_RATE_48MB;
	}
	B43_WARN_ON(1);
	return 0;
}

// bazzy addition
extern const struct b43_rate __b43_ratetable[];
extern const size_t __b43_ratetable_size;

const struct b43_rate *bitrate_to_b43_rate(u16 bitrate)
{
	unsigned int i;
	const struct b43_rate *b43r;

	//DBGC(bitrate, "bitrate_to_b43_rate: bitrate = %d,  ", bitrate);
	for (i = 0; i < __b43_ratetable_size; i++)
	{
		b43r = &__b43_ratetable[i];
		if (b43r->bitrate == bitrate)
		{
			//DBGC(bitrate, "DIRECT MATCH; ");
			return b43r;
		}
	}
	// no match :'(
	//DBGC(bitrate, "match = %d\n", closest_match->bitrate);
	return NULL; //closest_match;
}

static const struct b43_rate *ieee80211_get_tx_rate(struct b43_wl *wl)
{
	struct net80211_device *ndev = b43_wl_to_ndev(wl);
	u16 cur_bitrate = ndev->rates[ndev->rate];
	//DBGC(wl, "ieee80211_get_tx_rate (after array access)\n");
	// publicize the b43_rate ratetable
	// navigate through the rate table, comparing bitrates until a match is
	// found
	return bitrate_to_b43_rate(cur_bitrate);
}

static const struct b43_rate *ieee80211_get_alt_retry_rate(struct b43_wl *wl, size_t idx)
{
	struct net80211_device *ndev = b43_wl_to_ndev(wl);
	u16 bitrate;

	if ((idx + 1) >= ndev->nr_rates)
		return NULL;

	bitrate = ndev->rates[idx + 1];

	return bitrate_to_b43_rate(bitrate);
}

static void ieee80211_ctstoself_get(struct net80211_device *ndev,
    const void *frame, size_t frame_len, struct ieee80211_cts *cts)
{
	const struct ieee80211_frame *hdr =
        (const struct ieee80211_frame *)frame;
	//DBGC(ndev, "ieee80211_ctstoself_get\n");
	cts->fc = cpu_to_le16(IEEE80211_FTYPE_CTL | IEEE80211_STYPE_CTS);
	// ieee80211_ctstoself_duration
	cts->duration = net80211_cts_duration(ndev, frame_len);
	memcpy(cts->addr1, hdr->addr1, sizeof(cts->addr1));
}

static void ieee80211_rts_get(struct net80211_device *ndev,
		const void *frame, size_t frame_len, struct ieee80211_rts *rts)
{
	const struct ieee80211_hdr *hdr = (const struct ieee80211_hdr *)frame;

	rts->fc = cpu_to_le16(IEEE80211_FTYPE_CTL | IEEE80211_STYPE_RTS);
	// ieee80211_rts_duration()
	rts->duration = (
			net80211_duration ( ndev, 10, ndev->rates[ndev->rtscts_rate] ) +
			net80211_duration ( ndev, frame_len, ndev->rates[ndev->rate] ) +
			net80211_duration ( ndev, 10, ndev->rates[ndev->rtscts_rate] ));

	memcpy(rts->addr1, hdr->addr1, sizeof(rts->addr1));
	memcpy(rts->addr2, hdr->addr2, sizeof(rts->addr2));
}
/* Generate a TX data header. */
int b43_generate_txhdr(struct b43_wldev *dev,
		       u8 *_txhdr,
		       struct io_buffer *iob_frag,
		       u16 cookie)
{
	struct net80211_device *ndev = b43_wldev_to_ndev(dev);
	unsigned char *fragment_data = iob_frag->data;
	unsigned int fragment_len = iob_len(iob_frag);
	struct b43_txhdr *txhdr = (struct b43_txhdr *)_txhdr;
	const struct b43_phy *phy = &dev->phy;
	const struct ieee80211_frame *wlhdr =
	    (const struct ieee80211_frame *)fragment_data;
	const struct b43_rate *fbrate;
	/* It is safe for these to be u8, see hw_values @ b43.h:595+ */
	u8 rate, rate_fb; //fb = fallback
	int rate_ofdm, rate_fb_ofdm;
	unsigned int plcp_fragment_len;
	u32 mac_ctl = 0;
	u16 phy_ctl = 0;
	bool fill_phy_ctl1 = (phy->type == B43_PHYTYPE_LP ||
			      phy->type == B43_PHYTYPE_N ||
			      phy->type == B43_PHYTYPE_HT);
	u8 extra_ft = 0;
	const struct b43_rate *txrate;

	memset(txhdr, 0, sizeof(*txhdr));

	txrate = ieee80211_get_tx_rate(dev->wl);
	rate = txrate ? txrate->hw_value : B43_CCK_RATE_1MB;

	rate_ofdm = b43_is_ofdm_rate(rate);
	fbrate = ieee80211_get_alt_retry_rate(dev->wl, 0) ? : txrate;

	rate_fb = fbrate->hw_value;
	rate_fb_ofdm = b43_is_ofdm_rate(rate_fb);

	if (rate_ofdm)
		txhdr->phy_rate = b43_plcp_get_ratecode_ofdm(rate);
	else
		txhdr->phy_rate = b43_plcp_get_ratecode_cck(rate);
	txhdr->mac_frame_ctl = wlhdr->fc;
	memcpy(txhdr->tx_receiver, wlhdr->addr1, ETH_ALEN);

	/* Calculate duration for fallback rate */
	if ((rate_fb == rate) ||
	    (wlhdr->duration & cpu_to_le16(0x8000)) ||
	    (wlhdr->duration == cpu_to_le16(0))) {
		/* If the fallback rate equals the normal rate or the
		 * dur_id field contains an AID, CFP magic or 0,
		 * use the original dur_id field. */
		txhdr->dur_fb = wlhdr->duration;
	} else {
		txhdr->dur_fb = net80211_duration(ndev, fragment_len, (fbrate->bitrate));
	}
	plcp_fragment_len = fragment_len + FCS_LEN;

	switch (dev->fw.hdr_format) {
	case B43_FW_HDR_598:
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)(&txhdr->format_598.plcp),
				      plcp_fragment_len, rate);
		break;
	case B43_FW_HDR_351:
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)(&txhdr->format_351.plcp),
				      plcp_fragment_len, rate);
		break;
	case B43_FW_HDR_410:
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)(&txhdr->format_410.plcp),
				      plcp_fragment_len, rate);
		break;
	}
	b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)(&txhdr->plcp_fb),
			      plcp_fragment_len, rate_fb);

	/* Extra Frame Types */
	if (rate_fb_ofdm)
		extra_ft |= B43_TXH_EFT_FB_OFDM;
	else
		extra_ft |= B43_TXH_EFT_FB_CCK;

	/* Set channel radio code. Note that the micrcode ORs 0x100 to
	 * this value before comparing it to the value in SHM, if this
	 * is a 5Ghz packet.
	 */
	txhdr->chan_radio_code = phy->channel;

	/* PHY TX Control word */
	if (rate_ofdm)
		phy_ctl |= B43_TXH_PHY_ENC_OFDM;
	else
		phy_ctl |= B43_TXH_PHY_ENC_CCK;
	if ( (txrate->flags & IEEE80211_RATE_SHORT_PREAMBLE) &&
       (ndev->phy_flags & NET80211_PHY_USE_SHORT_PREAMBLE) )
	{
		//printf ("Using short preamble; ");
		phy_ctl |= B43_TXH_PHY_SHORTPRMBL;
	}
	switch (b43_ieee80211_antenna_sanitize(dev, 0)) {
	case 0: /* Default */
		phy_ctl |= B43_TXH_PHY_ANT01AUTO;
		break;
	case 1: /* Antenna 0 */
		phy_ctl |= B43_TXH_PHY_ANT0;
		break;
	case 2: /* Antenna 1 */
		phy_ctl |= B43_TXH_PHY_ANT1;
		break;
	case 3: /* Antenna 2 */
		phy_ctl |= B43_TXH_PHY_ANT2;
		break;
	case 4: /* Antenna 3 */
		phy_ctl |= B43_TXH_PHY_ANT3;
		break;
	default:
		B43_WARN_ON(1);
	}

	/* MAC control */
	// todo ipxe: don't expect ACKS on certain frames, PROBE REQ etc.
	mac_ctl |= B43_TXH_MAC_ACK;
	if (!phy->gmode)
		mac_ctl |= B43_TXH_MAC_5GHZ;

	/* Overwrite rates[0].count to make the retry calculation
	 * in the tx status easier. need the actual retry limit to
	 * detect whether the fallback rate was used.
	 */
	if (ndev->phy_flags & NET80211_PHY_USE_PROTECTION) {
		//printf("PROTECT\n");
		unsigned int len;
		// defined in ipxe ieee80211.h
		struct ieee80211_frame *uninitialized_var(hdr);
		int rts_rate, rts_rate_fb;
		int rts_rate_ofdm, rts_rate_fb_ofdm;
		struct b43_plcp_hdr6 *uninitialized_var(plcp);
		// defined in osdep as b43_rate
		const struct ieee80211_rate *rts_cts_rate;

		rts_cts_rate = bitrate_to_b43_rate(ndev->rates[ndev->rtscts_rate]);

		rts_rate = rts_cts_rate ? rts_cts_rate->hw_value : B43_CCK_RATE_1MB;
		rts_rate_ofdm = b43_is_ofdm_rate(rts_rate);
		rts_rate_fb = b43_calc_fallback_rate(rts_rate);
		rts_rate_fb_ofdm = b43_is_ofdm_rate(rts_rate_fb);

    // ipxe note: adapted from ath9k_bss_info_changed
    if (b43_current_band(dev->wl) != NET80211_BAND_5GHZ) {
			struct ieee80211_cts *uninitialized_var(cts);

			switch (dev->fw.hdr_format) {
			case B43_FW_HDR_598:
				cts = (struct ieee80211_cts *)
					(txhdr->format_598.rts_frame);
				break;
			case B43_FW_HDR_351:
				cts = (struct ieee80211_cts *)
					(txhdr->format_351.rts_frame);
				break;
			case B43_FW_HDR_410:
				cts = (struct ieee80211_cts *)
					(txhdr->format_410.rts_frame);
				break;
			}
			ieee80211_ctstoself_get(ndev, fragment_data, fragment_len, cts);
			mac_ctl |= B43_TXH_MAC_SENDCTS;
			printf("SENDCTS\n");
			len = sizeof(struct ieee80211_cts);
		} else {
			struct ieee80211_rts *uninitialized_var(rts);
			mac_ctl |= B43_TXH_MAC_LONGFRAME;

			switch (dev->fw.hdr_format) {
			case B43_FW_HDR_598:
				rts = (struct ieee80211_rts *)
					(txhdr->format_598.rts_frame);
				break;
			case B43_FW_HDR_351:
				rts = (struct ieee80211_rts *)
					(txhdr->format_351.rts_frame);
				break;
			case B43_FW_HDR_410:
				rts = (struct ieee80211_rts *)
					(txhdr->format_410.rts_frame);
				break;
			}
			ieee80211_rts_get(ndev, fragment_data, fragment_len, rts);
			mac_ctl |= B43_TXH_MAC_SENDRTS;
			len = sizeof(struct ieee80211_rts);
		}
		len += FCS_LEN;

		/* Generate the PLCP headers for the RTS/CTS frame */
		switch (dev->fw.hdr_format) {
		case B43_FW_HDR_598:
			plcp = &txhdr->format_598.rts_plcp;
			break;
		case B43_FW_HDR_351:
			plcp = &txhdr->format_351.rts_plcp;
			break;
		case B43_FW_HDR_410:
			plcp = &txhdr->format_410.rts_plcp;
			break;
		}
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)plcp,
				      len, rts_rate);
		plcp = &txhdr->rts_plcp_fb;
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)plcp,
				      len, rts_rate_fb);

		switch (dev->fw.hdr_format) {
		case B43_FW_HDR_598:
			hdr = (struct ieee80211_hdr *)
				(&txhdr->format_598.rts_frame);
			break;
		case B43_FW_HDR_351:
			hdr = (struct ieee80211_hdr *)
				(&txhdr->format_351.rts_frame);
			break;
		case B43_FW_HDR_410:
			hdr = (struct ieee80211_hdr *)
				(&txhdr->format_410.rts_frame);
			break;
		}
		txhdr->rts_dur_fb = hdr->duration;

		if (rts_rate_ofdm) {
			extra_ft |= B43_TXH_EFT_RTS_OFDM;
			txhdr->phy_rate_rts =
			    b43_plcp_get_ratecode_ofdm(rts_rate);
		} else {
			extra_ft |= B43_TXH_EFT_RTS_CCK;
			txhdr->phy_rate_rts =
			    b43_plcp_get_ratecode_cck(rts_rate);
		}
		if (rts_rate_fb_ofdm)
			extra_ft |= B43_TXH_EFT_RTSFB_OFDM;
		else
			extra_ft |= B43_TXH_EFT_RTSFB_CCK;

		if (fill_phy_ctl1 && b43_current_band(dev->wl) == NET80211_BAND_5GHZ) {
			txhdr->phy_ctl1_rts = cpu_to_le16(
				b43_generate_tx_phy_ctl1(dev, rts_rate));
			txhdr->phy_ctl1_rts_fb = cpu_to_le16(
				b43_generate_tx_phy_ctl1(dev, rts_rate_fb));
		}
  }

	/* Magic cookie */
	switch (dev->fw.hdr_format) {
	case B43_FW_HDR_598:
		txhdr->format_598.cookie = cpu_to_le16(cookie);
		break;
	case B43_FW_HDR_351:
		txhdr->format_351.cookie = cpu_to_le16(cookie);
		break;
	case B43_FW_HDR_410:
		txhdr->format_410.cookie = cpu_to_le16(cookie);
		break;
	}

	if (fill_phy_ctl1) {
		txhdr->phy_ctl1 =
			cpu_to_le16(b43_generate_tx_phy_ctl1(dev, rate));
		txhdr->phy_ctl1_fb =
			cpu_to_le16(b43_generate_tx_phy_ctl1(dev, rate_fb));
	}

	/* Apply the bitfields */
	txhdr->mac_ctl = cpu_to_le32(mac_ctl);
	txhdr->phy_ctl = cpu_to_le16(phy_ctl);
	txhdr->extra_ft = extra_ft;

	return 0;
}

static s8 b43_rssi_postprocess(struct b43_wldev *dev,
			       u8 in_rssi, int ofdm,
			       int adjust_2053, int adjust_2050)
{
	struct b43_phy *phy = &dev->phy;
	struct b43_phy_g *gphy = phy->g;
	s32 tmp;

	switch (phy->radio_ver) {
	case 0x2050:
		if (ofdm) {
			tmp = in_rssi;
			if (tmp > 127)
				tmp -= 256;
			tmp *= 73;
			tmp /= 64;
			if (adjust_2050)
				tmp += 25;
			else
				tmp -= 3;
		} else {
			if (dev->dev->bus_sprom->
			    boardflags_lo & B43_BFL_RSSI) {
				if (in_rssi > 63)
					in_rssi = 63;
				B43_WARN_ON(phy->type != B43_PHYTYPE_G);
				tmp = gphy->nrssi_lt[in_rssi];
				tmp = 31 - tmp;
				tmp *= -131;
				tmp /= 128;
				tmp -= 57;
			} else {
				tmp = in_rssi;
				tmp = 31 - tmp;
				tmp *= -149;
				tmp /= 128;
				tmp -= 68;
			}
			if (phy->type == B43_PHYTYPE_G && adjust_2050)
				tmp += 25;
		}
		break;
	case 0x2060:
		if (in_rssi > 127)
			tmp = in_rssi - 256;
		else
			tmp = in_rssi;
		break;
	default:
		tmp = in_rssi;
		tmp -= 11;
		tmp *= 103;
		tmp /= 64;
		if (adjust_2053)
			tmp -= 109;
		else
			tmp -= 83;
	}

	return (s8) tmp;
}

//TODO
#if 0
static s8 b43_rssinoise_postprocess(struct b43_wldev *dev, u8 in_rssi)
{
	struct b43_phy *phy = &dev->phy;
	s8 ret;

	if (phy->type == B43_PHYTYPE_A) {
		//TODO: Incomplete specs.
		ret = 0;
	} else
		ret = b43_rssi_postprocess(dev, in_rssi, 0, 1, 1);

	return ret;
}
#endif

void b43_rx(struct b43_wldev *dev, struct io_buffer *iob, const void *_rxhdr)
{
	struct b43_plcp_hdr6 *plcp;
	const struct b43_rxhdr_fw4 *rxhdr = _rxhdr;
	u16 phystat0, phystat3;
	u16 uninitialized_var(chanstat);
	u32 uninitialized_var(macstat);
	u16 phytype __unused;
	int padding, rate_idx;

	// Added for ipxe driver
	s8 signal = 0;

	/* Get metadata about the frame from the header. */
	phystat0 = le16_to_cpu(rxhdr->phy_status0);
	phystat3 = le16_to_cpu(rxhdr->phy_status3);
	switch (dev->fw.hdr_format) {
	case B43_FW_HDR_598:
		macstat = le32_to_cpu(rxhdr->format_598.mac_status);
		chanstat = le16_to_cpu(rxhdr->format_598.channel);
		break;
	case B43_FW_HDR_410:
	case B43_FW_HDR_351:
		macstat = le32_to_cpu(rxhdr->format_351.mac_status);
		chanstat = le16_to_cpu(rxhdr->format_351.channel);
		break;
	}
	phytype = chanstat & B43_RX_CHAN_PHYTYPE;

	if (unlikely(macstat & B43_RX_MAC_FCSERR)) {
		dev->wl->ieee_stats.dot11FCSErrorCount++;
	}
	if (macstat & B43_RX_MAC_DECERR) {
		/* Decryption with the given key failed.
		 * Drop the packet. We also won't be able to decrypt it with
		 * the key in software. */
		goto drop;
	}

	/* Skip PLCP and padding */
	padding = (macstat & B43_RX_MAC_PADDING) ? 2 : 0;
	if (unlikely(iob_len(iob) < (sizeof(struct b43_plcp_hdr6) + padding))) {
		b43dbg(dev->wl, "RX: Packet size underrun (1)\n");
		goto drop;
	}
	plcp = (struct b43_plcp_hdr6 *)(iob->data + padding);
	iob_pull(iob, sizeof(struct b43_plcp_hdr6) + padding);
	/* The skb contains the Wireless Header + payload data now */
	if (unlikely(iob_len(iob) < (2 + 2 + 6 /*minimum hdr */  + FCS_LEN))) {
		b43dbg(dev->wl, "RX: Packet size underrun (2)\n");
		goto drop;
	}

	if (macstat & B43_RX_MAC_DEC) {
		/* Since ipxe handles decryption in software, the encrypted packet
		 * is handled just the same as regular ones. No special treatment.
		 * The status flag above is misleading: for ipxe, the b43 hardware
		 * encryption keys are kept cleared (set to no encryption method),
		 * but this decryption flag gets set anyways. Actually, it indicates
		 * "decryption attempted" but ultimately the packet was not harmed.
		 * In a different scenario (with hardware keys set?), when a
		 * bad decryption attempt mangles the packet, it would probably be
		 * marked by the B43_RX_MAC_DECERR check earlier in this function.*/
	}

	/* Link quality statistics */
	switch (chanstat & B43_RX_CHAN_PHYTYPE) {
	case B43_PHYTYPE_HT:
		/* TODO: is max the right choice? */
		signal = max_t(__s8,
			max(rxhdr->phy_ht_power0, rxhdr->phy_ht_power1),
			rxhdr->phy_ht_power2);
		break;
	case B43_PHYTYPE_N:
		/* Broadcom has code for min and avg, but always uses max */
		if (rxhdr->power0 == 16 || rxhdr->power0 == 32)
			signal = max(rxhdr->power1, rxhdr->power2);
		else
			signal = max(rxhdr->power0, rxhdr->power1);
		break;
	case B43_PHYTYPE_A:
	case B43_PHYTYPE_B:
	case B43_PHYTYPE_G:
	case B43_PHYTYPE_LP:
		signal = b43_rssi_postprocess(dev, rxhdr->jssi,
						  (phystat0 & B43_RX_PHYST0_OFDM),
						  (phystat0 & B43_RX_PHYST0_GAINCTL),
						  (phystat3 & B43_RX_PHYST3_TRSTATE));
		break;
	}

	if (phystat0 & B43_RX_PHYST0_OFDM)
		rate_idx = b43_plcp_get_bitrate_idx_ofdm(plcp,
					!!(chanstat & B43_RX_CHAN_5GHZ));
	else
		rate_idx = b43_plcp_get_bitrate_idx_cck(plcp);
	if (unlikely(rate_idx == -1)) {
		/* PLCP seems to be corrupted.
		 * Drop the frame, if we are not interested in corrupted frames. */
		if (!(dev->wl->filter_flags & FIF_PLCPFAIL))
			goto drop;
	}

	switch (chanstat & B43_RX_CHAN_PHYTYPE) {
	case B43_PHYTYPE_A:
	case B43_PHYTYPE_G:
	case B43_PHYTYPE_N:
	case B43_PHYTYPE_LP:
	case B43_PHYTYPE_HT:
		break;
	default:
		B43_WARN_ON(1);
		goto drop;
	}

	/* Uncomment when you need to inspect RX frame hex dumps */
	/* DBGC2_HD(dev, iob->data, iob_len(iob)); */

	net80211_rx(b43_wldev_to_ndev(dev), iob, signal,
              __b43_ratetable[rate_idx].bitrate);

#if B43_DEBUG
	dev->rx_count++;
#endif
	return;
drop:
	free_iob(iob);
}

void b43_handle_txstatus(struct b43_wldev *dev,
			 const struct b43_txstatus *status)
{
	if (status->intermediate)
		return;
	if (status->for_ampdu)
		return;
	if (!status->acked)
		dev->wl->ieee_stats.dot11ACKFailureCount++;
	if (status->rts_count) {
		if (status->rts_count == 0xF)	//FIXME
			dev->wl->ieee_stats.dot11RTSFailureCount++;
		else
			dev->wl->ieee_stats.dot11RTSSuccessCount++;
	}

	if (b43_using_pio_transfers(dev))
		b43_pio_handle_txstatus(dev, status);
	else
		b43_dma_handle_txstatus(dev, status);

	b43_phy_txpower_check(dev, 0);
}

/* Fill out the mac80211 TXstatus report based on the b43-specific
 * txstatus report data. This returns a boolean whether the frame was
 * successfully transmitted. */
bool b43_fill_txstatus_report(struct b43_wldev *dev __unused,
			      const struct b43_txstatus *status)
{
	bool frame_success = true;


	if (status->acked) {
		/* The frame was ACKed. */
	} else {
		/* The frame was not ACKed... */
			frame_success = false;
	}

	return frame_success;
}

/* Stop any TX operation on the device (suspend the hardware queues) */
void b43_tx_suspend(struct b43_wldev *dev)
{
	if (b43_using_pio_transfers(dev))
		b43_pio_tx_suspend(dev);
	else
		b43_dma_tx_suspend(dev);
}

/* Resume any TX operation on the device (resume the hardware queues) */
void b43_tx_resume(struct b43_wldev *dev)
{
	if (b43_using_pio_transfers(dev))
		b43_pio_tx_resume(dev);
	else
		b43_dma_tx_resume(dev);
}
