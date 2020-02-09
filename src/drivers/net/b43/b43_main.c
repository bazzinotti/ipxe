/*

  Broadcom B43 wireless driver

  Ported to iPXE
  Copyright (c) 2018-2020 Michael Bazzinotti <bazz@bazz1.com>
  Original from Linux kernel 3.18.11

  Copyright (c) 2005 Martin Langer <martin-langer@gmx.de>
  Copyright (c) 2005 Stefano Brivio <stefano.brivio@polimi.it>
  Copyright (c) 2005-2009 Michael Buesch <m@bues.ch>
  Copyright (c) 2005 Danny van Dyk <kugelfang@gentoo.org>
  Copyright (c) 2005 Andreas Jaggi <andreas.jaggi@waterwave.ch>
  Copyright (c) 2010-2011 Rafał Miłecki <zajec5@gmail.com>

  SDIO support
  Copyright (c) 2009 Albert Herranz <albert_herranz@yahoo.es>

  Some parts of the code in this file are derived from the ipw2200
  driver  Copyright(c) 2003 - 2004 Intel Corporation.

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

#include <ipxe/timer.h>
#include <ipxe/init.h>
#include <ipxe/if_arp.h>
#include <ipxe/if_ether.h>
#include <ipxe/io.h>

#include "b43.h"
#include "b43_main.h"
#include "b43_phy_common.h"
#include "b43_phy_g.h"
#include "b43_phy_n.h"
#include "b43_dma.h"
#include "b43_pio.h"
#include "b43_xmit.h"
#include "b43_lo.h"
#include "b43_pcmcia.h"
#include "b43_sdio.h"

// the firmware files
#include "b43_fw.h" /* is generated from util/b43-import-fw.sh */

#include <ipxe/ethernet.h> // is_valid_ether_addr()

/* iPXE DEBUG setting to toggle between PIO / DMA
 *
 * Given the nature that iPXE settings are loaded always, only load this
 * setting if the module is being compiled with DEBUG support. Otherwise,
 * do not clog up the settings page with this, as it will be an unusable
 * setting on systems with no b43 devices loaded */
#ifndef NDEBUG
#define B43_SETTINGS
#endif

#ifdef B43_SETTINGS
#include <ipxe/settings.h>
const struct setting b43_dma_setting __setting ( SETTING_NETDEV_EXTRA,
		b43-dma ) = {
	.name = "b43-dma",
	.description = "For b43 devices only. Use DMA instead of default PIO. "
	               "Requires re-opening the interface if already open.",
	.type = &setting_type_int8,
};
#endif /* B43_SETTINGS */

static void b43_periodic_work_handler(struct b43_wldev *dev);

//from include/linux/unaligned/be_byteshift.h
static inline u32 get_unaligned_be32(const u8 *p)
{
  return p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3];
}

FILE_LICENCE(GPL_ANY);

/* "enable(1) / disable(0) Bad Frames Preemption" */
static int modparam_bad_frames_preempt = 1;

/* "Enable hardware-side power control (default off)" */
static int modparam_hwpctl = 0;

/* "Enable hardware tkip." */
static int modparam_hwtkip = 0;

/* "Enable Bluetooth coexistence (default on)" */
static int modparam_btcoex = 1;

/* "Log message verbosity: 0=error, 1=warn, 2=info(default), 3=debug" */
int b43_modparam_verbose = B43_VERBOSITY_DEFAULT;

/* "Use PIO accesses by default: 0=DMA, 1=PIO" */
static int b43_modparam_pio = 1;
/* On iPXE, PIO accesses have proven to be the speediest. Perhaps this is
 * because on DMA, the number of TX/RX slots has to be so much lower than
 * specified in the Linux driver, due to mem constraints.
 * (relevant definition: B43_TXRING_SLOTS) */

/* "Enable support for all hardware (even it if overlaps with the brcmsmac driver)" */
static int modparam_allhwsupport = 1;

#ifdef CONFIG_B43_BCMA
static const struct bcma_device_id b43_bcma_tbl[] = {
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x11, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x17, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x18, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x1C, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x1D, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x1E, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x28, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x2A, BCMA_ANY_CLASS),
	BCMA_CORETABLE_END
};
#endif

#ifdef CONFIG_B43_SSB
static const struct ssb_device_id b43_ssb_tbl[] = {
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 5),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 6),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 7),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 9),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 10),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 11),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 12),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 13),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 15),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 16),
	SSB_DEVTABLE_END
};
#endif

/* Channel and ratetables are shared for all devices.
 * They can't be const, because ieee80211 puts some precalculated
 * data in there. This data is the same for all devices, so we don't
 * get concurrency issues */
#define RATETAB_ENT(_rateid, _flags) \
	{ \
		.bitrate  = B43_RATE_TO_BASE100KBPS(_rateid), \
		.hw_value = (_rateid), \
		.flags    = (_flags), \
	}

const struct b43_rate __b43_ratetable[] = {
	RATETAB_ENT(B43_CCK_RATE_1MB, 0), /* 802.11b */
	RATETAB_ENT(B43_CCK_RATE_2MB, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(B43_CCK_RATE_5MB, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(B43_CCK_RATE_11MB, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(B43_OFDM_RATE_6MB, 0), /* 802.11g */
	RATETAB_ENT(B43_OFDM_RATE_9MB, 0),
	RATETAB_ENT(B43_OFDM_RATE_12MB, 0),
	RATETAB_ENT(B43_OFDM_RATE_18MB, 0),
	RATETAB_ENT(B43_OFDM_RATE_24MB, 0),
	RATETAB_ENT(B43_OFDM_RATE_36MB, 0),
	RATETAB_ENT(B43_OFDM_RATE_48MB, 0),
	RATETAB_ENT(B43_OFDM_RATE_54MB, 0),
	{0}, {0}, {0}, {0},   /* index safely using a value masked with 0xF */
};
const size_t __b43_ratetable_size = 12; // avoid the '0' entries

#define b43_a_ratetable		(__b43_ratetable + 4)
#define b43_a_ratetable_size	8
#define b43_b_ratetable		(__b43_ratetable + 0)
#define b43_b_ratetable_size	4
#define b43_g_ratetable		(__b43_ratetable + 0)
#define b43_g_ratetable_size	12


#define CHAN2G(_channel, _freq, _flags) {			\
	.band			= NET80211_BAND_2GHZ,		\
	.center_freq		= (_freq),			\
	.hw_value		= (_channel),			\
	.channel_nr = (_channel),			\
	.maxpower		= 30,				\
}
static struct net80211_channel b43_2ghz_chantable[] = {
	CHAN2G(1, 2412, 0),
	CHAN2G(2, 2417, 0),
	CHAN2G(3, 2422, 0),
	CHAN2G(4, 2427, 0),
	CHAN2G(5, 2432, 0),
	CHAN2G(6, 2437, 0),
	CHAN2G(7, 2442, 0),
	CHAN2G(8, 2447, 0),
	CHAN2G(9, 2452, 0),
	CHAN2G(10, 2457, 0),
	CHAN2G(11, 2462, 0),
	CHAN2G(12, 2467, 0),
	CHAN2G(13, 2472, 0),
	CHAN2G(14, 2484, 0),
};

/* No support for the last 3 channels (12, 13, 14) */
#define b43_2ghz_chantable_limited_size		11
#undef CHAN2G

#define CHAN4G(_channel, _flags) {				\
	.band			= NET80211_BAND_5GHZ,		\
	.center_freq		= 4000 + (5 * (_channel)),	\
	.hw_value		= (_channel),			\
	.maxpower		= 30,				\
}
#define CHAN5G(_channel, _flags) {				\
	.band			= NET80211_BAND_5GHZ,		\
	.center_freq		= 5000 + (5 * (_channel)),	\
	.hw_value		= (_channel),			\
	.maxpower		= 30,				\
}
static struct net80211_channel b43_5ghz_nphy_chantable[] = {
	CHAN4G(184, 0),		CHAN4G(186, 0),
	CHAN4G(188, 0),		CHAN4G(190, 0),
	CHAN4G(192, 0),		CHAN4G(194, 0),
	CHAN4G(196, 0),		CHAN4G(198, 0),
	CHAN4G(200, 0),		CHAN4G(202, 0),
	CHAN4G(204, 0),		CHAN4G(206, 0),
	CHAN4G(208, 0),		CHAN4G(210, 0),
	CHAN4G(212, 0),		CHAN4G(214, 0),
	CHAN4G(216, 0),		CHAN4G(218, 0),
	CHAN4G(220, 0),		CHAN4G(222, 0),
	CHAN4G(224, 0),		CHAN4G(226, 0),
	CHAN4G(228, 0),
	CHAN5G(32, 0),		CHAN5G(34, 0),
	CHAN5G(36, 0),		CHAN5G(38, 0),
	CHAN5G(40, 0),		CHAN5G(42, 0),
	CHAN5G(44, 0),		CHAN5G(46, 0),
	CHAN5G(48, 0),		CHAN5G(50, 0),
	CHAN5G(52, 0),		CHAN5G(54, 0),
	CHAN5G(56, 0),		CHAN5G(58, 0),
	CHAN5G(60, 0),		CHAN5G(62, 0),
	CHAN5G(64, 0),		CHAN5G(66, 0),
	CHAN5G(68, 0),		CHAN5G(70, 0),
	CHAN5G(72, 0),		CHAN5G(74, 0),
	CHAN5G(76, 0),		CHAN5G(78, 0),
	CHAN5G(80, 0),		CHAN5G(82, 0),
	CHAN5G(84, 0),		CHAN5G(86, 0),
	CHAN5G(88, 0),		CHAN5G(90, 0),
	CHAN5G(92, 0),		CHAN5G(94, 0),
	CHAN5G(96, 0),		CHAN5G(98, 0),
	CHAN5G(100, 0),		CHAN5G(102, 0),
	CHAN5G(104, 0),		CHAN5G(106, 0),
	CHAN5G(108, 0),		CHAN5G(110, 0),
	CHAN5G(112, 0),		CHAN5G(114, 0),
	CHAN5G(116, 0),		CHAN5G(118, 0),
	CHAN5G(120, 0),		CHAN5G(122, 0),
	CHAN5G(124, 0),		CHAN5G(126, 0),
	CHAN5G(128, 0),		CHAN5G(130, 0),
	CHAN5G(132, 0),		CHAN5G(134, 0),
	CHAN5G(136, 0),		CHAN5G(138, 0),
	CHAN5G(140, 0),		CHAN5G(142, 0),
	CHAN5G(144, 0),		CHAN5G(145, 0),
	CHAN5G(146, 0),		CHAN5G(147, 0),
	CHAN5G(148, 0),		CHAN5G(149, 0),
	CHAN5G(150, 0),		CHAN5G(151, 0),
	CHAN5G(152, 0),		CHAN5G(153, 0),
	CHAN5G(154, 0),		CHAN5G(155, 0),
	CHAN5G(156, 0),		CHAN5G(157, 0),
	CHAN5G(158, 0),		CHAN5G(159, 0),
	CHAN5G(160, 0),		CHAN5G(161, 0),
	CHAN5G(162, 0),		CHAN5G(163, 0),
	CHAN5G(164, 0),		CHAN5G(165, 0),
	CHAN5G(166, 0),		CHAN5G(168, 0),
	CHAN5G(170, 0),		CHAN5G(172, 0),
	CHAN5G(174, 0),		CHAN5G(176, 0),
	CHAN5G(178, 0),		CHAN5G(180, 0),
	CHAN5G(182, 0),
};

static struct net80211_channel b43_5ghz_nphy_chantable_limited[] = {
	CHAN5G(36, 0),		CHAN5G(40, 0),
	CHAN5G(44, 0),		CHAN5G(48, 0),
	CHAN5G(149, 0),		CHAN5G(153, 0),
	CHAN5G(157, 0),		CHAN5G(161, 0),
	CHAN5G(165, 0),
};

static struct net80211_channel b43_5ghz_aphy_chantable[] = {
	CHAN5G(34, 0),		CHAN5G(36, 0),
	CHAN5G(38, 0),		CHAN5G(40, 0),
	CHAN5G(42, 0),		CHAN5G(44, 0),
	CHAN5G(46, 0),		CHAN5G(48, 0),
	CHAN5G(52, 0),		CHAN5G(56, 0),
	CHAN5G(60, 0),		CHAN5G(64, 0),
	CHAN5G(100, 0),		CHAN5G(104, 0),
	CHAN5G(108, 0),		CHAN5G(112, 0),
	CHAN5G(116, 0),		CHAN5G(120, 0),
	CHAN5G(124, 0),		CHAN5G(128, 0),
	CHAN5G(132, 0),		CHAN5G(136, 0),
	CHAN5G(140, 0),		CHAN5G(149, 0),
	CHAN5G(153, 0),		CHAN5G(157, 0),
	CHAN5G(161, 0),		CHAN5G(165, 0),
	CHAN5G(184, 0),		CHAN5G(188, 0),
	CHAN5G(192, 0),		CHAN5G(196, 0),
	CHAN5G(200, 0),		CHAN5G(204, 0),
	CHAN5G(208, 0),		CHAN5G(212, 0),
	CHAN5G(216, 0),
};
#undef CHAN4G
#undef CHAN5G

static void b43_wireless_core_exit(struct b43_wldev *dev);
static int b43_wireless_core_init(struct b43_wldev *dev);
static struct b43_wldev * b43_wireless_core_stop(struct b43_wldev *dev);
static int b43_wireless_core_start(struct b43_wldev *dev);
inline static void b43_op_bss_info_changed(struct net80211_device *ndev,
                                           u32 changed);

/* added for the ipxe net80211_device_operations .irq support
 * NOTE: the .irq callback is merely stubbed in ipxe. Make sure
 * the driver takes full responsibility on enabling hw interrupts during
 * its init routines, because the irq callback will never be called.
 */
static inline u32 b43_irq_mask(struct b43_wldev *wldev)
{
  return wldev->irq_enable ? wldev->irq_mask : 0;
}


static int b43_ratelimit(struct b43_wl *wl)
{
	if (!wl || !wl->current_dev)
		return 1;
	if (b43_status(wl->current_dev) < B43_STAT_STARTED)
		return 1;
	/* We are up and running.
	 * Ratelimit the messages to avoid DoS over the net. */

	/* stub */
	return 1;
}

/* There are 2 levels of debugging this driver. when the objects are
 * compiled without debug support through ipxe (by make DEBUG=), no
 * printouts from the driver whatsoever will reach the console. (If they
 * do, they are remnants of old test code and should be removed or
 * commented). When compiled with DEBUG enabled, depending on whether
 * B43_DEBUG is defined, the resultant B43_VERBOSITY_DEFAULT will specify
 * the depth of the native driver printout messages. I have also added my
 * ipxe port debug messages as needed (using DBG, DBGC) and for extremely
 * rare usage, ipxe debug level 2 (DBGC2) has been rarely put into use as
 * well.
 *
 * To get B43_DEBUG defined, I recommend adding EXTRA_CFLAGS=-DB43_DEBUG
 * to your `make` invocation, or add the definition to b43.h and
 * b43_phy_common.h */
#ifdef NDEBUG
#define b43info (wl, fmt, ...)
#define b43err (wl, fmt, ...)
#define b43warn (wl, fmt, ...)
#define b43dbg (wl, fmt, ...)
#else
void b43info(struct b43_wl *wl, const char *fmt, ...)
{
	va_list args;

	if (b43_modparam_verbose < B43_VERBOSITY_INFO)
		return;
	if (!b43_ratelimit(wl))
		return;

	va_start(args, fmt);

	printk(KERN_INFO "b43-%s: ",
	       (wl && wl->hwinfo) ? wl->ndev->netdev->name : "wlan");
	vprintf(fmt, args);

	va_end(args);
}

void b43err(struct b43_wl *wl, const char *fmt, ...)
{
	va_list args;

	if (b43_modparam_verbose < B43_VERBOSITY_ERROR)
		return;
	if (!b43_ratelimit(wl))
		return;

	va_start(args, fmt);

	printk(KERN_ERR "b43-%s ERROR: ",
	       (wl && wl->hwinfo) ? wl->ndev->netdev->name : "wlan");
	vprintf(fmt, args);

	va_end(args);
}

void b43warn(struct b43_wl *wl, const char *fmt, ...)
{
	va_list args;

	if (b43_modparam_verbose < B43_VERBOSITY_WARN)
		return;
	if (!b43_ratelimit(wl))
		return;

	va_start(args, fmt);

	printk(KERN_WARNING "b43-%s warning: ",
	       (wl && wl->hwinfo) ? wl->ndev->netdev->name : "wlan");
	vprintf(fmt, args);

	va_end(args);
}

void b43dbg(struct b43_wl *wl, const char *fmt, ...)
{
	va_list args;

	if (b43_modparam_verbose < B43_VERBOSITY_DEBUG)
		return;

	va_start(args, fmt);

	printk(KERN_DEBUG "b43-%s debug: ",
	       (wl && wl->hwinfo) ? wl->ndev->netdev->name : "wlan");
	vprintf(fmt, args);

	va_end(args);
}

#endif /* NDEBUG */


static void b43_ram_write(struct b43_wldev *dev, u16 offset, u32 val)
{
	u32 macctl;

	B43_WARN_ON(offset % 4 != 0);

	macctl = b43_read32(dev, B43_MMIO_MACCTL);
	if (macctl & B43_MACCTL_BE)
		val = swab32(val);

	b43_write32(dev, B43_MMIO_RAM_CONTROL, offset);
	mmiowb();
	b43_write32(dev, B43_MMIO_RAM_DATA, val);
}

static inline void b43_shm_control_word(struct b43_wldev *dev,
					u16 routing, u16 offset)
{
	u32 control;

	/* "offset" is the WORD offset. */
	control = routing;
	control <<= 16;
	control |= offset;
	b43_write32(dev, B43_MMIO_SHM_CONTROL, control);
}

u32 b43_shm_read32(struct b43_wldev *dev, u16 routing, u16 offset)
{
	u32 ret;

	if (routing == B43_SHM_SHARED) {
		B43_WARN_ON(offset & 0x0001);
		if (offset & 0x0003) {
			/* Unaligned access */
			b43_shm_control_word(dev, routing, offset >> 2);
			ret = b43_read16(dev, B43_MMIO_SHM_DATA_UNALIGNED);
			b43_shm_control_word(dev, routing, (offset >> 2) + 1);
			ret |= ((u32)b43_read16(dev, B43_MMIO_SHM_DATA)) << 16;

			goto out;
		}
		offset >>= 2;
	}
	b43_shm_control_word(dev, routing, offset);
	ret = b43_read32(dev, B43_MMIO_SHM_DATA);
out:
	return ret;
}

u16 b43_shm_read16(struct b43_wldev *dev, u16 routing, u16 offset)
{
	u16 ret;

	if (routing == B43_SHM_SHARED) {
		B43_WARN_ON(offset & 0x0001);
		if (offset & 0x0003) {
			/* Unaligned access */
			b43_shm_control_word(dev, routing, offset >> 2);
			ret = b43_read16(dev, B43_MMIO_SHM_DATA_UNALIGNED);

			goto out;
		}
		offset >>= 2;
	}
	b43_shm_control_word(dev, routing, offset);
	ret = b43_read16(dev, B43_MMIO_SHM_DATA);
out:
	return ret;
}

void b43_shm_write32(struct b43_wldev *dev, u16 routing, u16 offset, u32 value)
{
	if (routing == B43_SHM_SHARED) {
		B43_WARN_ON(offset & 0x0001);
		if (offset & 0x0003) {
			/* Unaligned access */
			b43_shm_control_word(dev, routing, offset >> 2);
			b43_write16(dev, B43_MMIO_SHM_DATA_UNALIGNED,
				    value & 0xFFFF);
			b43_shm_control_word(dev, routing, (offset >> 2) + 1);
			b43_write16(dev, B43_MMIO_SHM_DATA,
				    (value >> 16) & 0xFFFF);
			return;
		}
		offset >>= 2;
	}
	b43_shm_control_word(dev, routing, offset);
	b43_write32(dev, B43_MMIO_SHM_DATA, value);
}

void b43_shm_write16(struct b43_wldev *dev, u16 routing, u16 offset, u16 value)
{
	if (routing == B43_SHM_SHARED) {
		B43_WARN_ON(offset & 0x0001);
		if (offset & 0x0003) {
			/* Unaligned access */
			b43_shm_control_word(dev, routing, offset >> 2);
			b43_write16(dev, B43_MMIO_SHM_DATA_UNALIGNED, value);
			return;
		}
		offset >>= 2;
	}
	b43_shm_control_word(dev, routing, offset);
	b43_write16(dev, B43_MMIO_SHM_DATA, value);
}

/* Read HostFlags */
u64 b43_hf_read(struct b43_wldev *dev)
{
	u64 ret;

	ret = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_HOSTF3);
	ret <<= 16;
	ret |= b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_HOSTF2);
	ret <<= 16;
	ret |= b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_HOSTF1);

	return ret;
}

/* Write HostFlags */
void b43_hf_write(struct b43_wldev *dev, u64 value)
{
	u16 lo, mi, hi;

	lo = (value & 0x00000000FFFFULL);
	mi = (value & 0x0000FFFF0000ULL) >> 16;
	hi = (value & 0xFFFF00000000ULL) >> 32;
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_HOSTF1, lo);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_HOSTF2, mi);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_HOSTF3, hi);
}

/* Read the firmware capabilities bitmask (Opensource firmware only) */
static u16 b43_fwcapa_read(struct b43_wldev *dev)
{
	B43_WARN_ON(!dev->fw.opensource);
	return b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_FWCAPA);
}

void b43_tsf_read(struct b43_wldev *dev, u64 *tsf)
{
	u32 low, high;

	B43_WARN_ON(dev->dev->core_rev < 3);

	/* The hardware guarantees us an atomic read, if we
	 * read the low register first. */
	low = b43_read32(dev, B43_MMIO_REV3PLUS_TSF_LOW);
	high = b43_read32(dev, B43_MMIO_REV3PLUS_TSF_HIGH);

	*tsf = high;
	*tsf <<= 32;
	*tsf |= low;
}

static void b43_time_lock(struct b43_wldev *dev)
{
	b43_maskset32(dev, B43_MMIO_MACCTL, ~0, B43_MACCTL_TBTTHOLD);
	/* Commit the write */
	b43_read32(dev, B43_MMIO_MACCTL);
}

static void b43_time_unlock(struct b43_wldev *dev)
{
	b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_TBTTHOLD, 0);
	/* Commit the write */
	b43_read32(dev, B43_MMIO_MACCTL);
}

static void b43_tsf_write_locked(struct b43_wldev *dev, u64 tsf)
{
	u32 low, high;

	B43_WARN_ON(dev->dev->core_rev < 3);

	low = tsf;
	high = (tsf >> 32);
	/* The hardware guarantees us an atomic write, if we
	 * write the low register first. */
	b43_write32(dev, B43_MMIO_REV3PLUS_TSF_LOW, low);
	mmiowb();
	b43_write32(dev, B43_MMIO_REV3PLUS_TSF_HIGH, high);
	mmiowb();
}

void b43_tsf_write(struct b43_wldev *dev, u64 tsf)
{
	b43_time_lock(dev);
	b43_tsf_write_locked(dev, tsf);
	b43_time_unlock(dev);
}

static
void b43_macfilter_set(struct b43_wldev *dev, u16 offset, const u8 *mac)
{
	static const u8 zero_addr[ETH_ALEN] = { 0 };
	u16 data;

	if (!mac)
		mac = zero_addr;

	offset |= 0x0020;
	b43_write16(dev, B43_MMIO_MACFILTER_CONTROL, offset);

	data = mac[0];
	data |= mac[1] << 8;
	b43_write16(dev, B43_MMIO_MACFILTER_DATA, data);
	data = mac[2];
	data |= mac[3] << 8;
	b43_write16(dev, B43_MMIO_MACFILTER_DATA, data);
	data = mac[4];
	data |= mac[5] << 8;
	b43_write16(dev, B43_MMIO_MACFILTER_DATA, data);
}

static void b43_write_mac_bssid_templates(struct b43_wldev *dev)
{
	const u8 *mac;
	const u8 *bssid;
	u8 mac_bssid[ETH_ALEN * 2];
	unsigned int i;
	u32 tmp;

	bssid = dev->wl->bssid;
	mac = dev->wl->mac_addr;

	b43_macfilter_set(dev, B43_MACFILTER_BSSID, bssid);

	memcpy(mac_bssid, mac, ETH_ALEN);
	memcpy(mac_bssid + ETH_ALEN, bssid, ETH_ALEN);

	/* Write our MAC address and BSSID to template ram */
	for (i = 0; i < ARRAY_SIZE(mac_bssid); i += sizeof(u32)) {
		tmp = (u32) (mac_bssid[i + 0]);
		tmp |= (u32) (mac_bssid[i + 1]) << 8;
		tmp |= (u32) (mac_bssid[i + 2]) << 16;
		tmp |= (u32) (mac_bssid[i + 3]) << 24;
		b43_ram_write(dev, 0x20 + i, tmp);
	}
}

static void b43_upload_card_macaddress(struct b43_wldev *dev)
{
	b43_write_mac_bssid_templates(dev);
	b43_macfilter_set(dev, B43_MACFILTER_SELF, dev->wl->mac_addr);
}

static void b43_set_slot_time(struct b43_wldev *dev, u16 slot_time)
{
	/* slot_time is in usec. */
	/* This test used to exit for all but a G PHY. */
	if (b43_current_band(dev->wl) == IEEE80211_BAND_5GHZ)
		return;
	b43_write16(dev, B43_MMIO_IFSSLOT, 510 + slot_time);
	/* Shared memory location 0x0010 is the slot time and should be
	 * set to slot_time; however, this register is initially 0 and changing
	 * the value adversely affects the transmit rate for BCM4311
	 * devices. Until this behavior is unterstood, delete this step
	 *
	 * b43_shm_write16(dev, B43_SHM_SHARED, 0x0010, slot_time);
	 */
}

static void b43_short_slot_timing_enable(struct b43_wldev *dev)
{
	b43_set_slot_time(dev, 9);
}

static void b43_short_slot_timing_disable(struct b43_wldev *dev)
{
	b43_set_slot_time(dev, 20);
}

/* DummyTransmission function, as documented on
 * http://bcm-v4.sipsolutions.net/802.11/DummyTransmission
 */
void b43_dummy_transmission(struct b43_wldev *dev, bool ofdm, bool pa_on __unused)
{
	struct b43_phy *phy = &dev->phy;
	unsigned int i, max_loop;
	u16 value;
	u32 buffer[5] = {
		0x00000000,
		0x00D40000,
		0x00000000,
		0x01000000,
		0x00000000,
	};

	if (ofdm) {
		max_loop = 0x1E;
		buffer[0] = 0x000201CC;
	} else {
		max_loop = 0xFA;
		buffer[0] = 0x000B846E;
	}

	for (i = 0; i < 5; i++)
		b43_ram_write(dev, i * 4, buffer[i]);

	b43_write16(dev, B43_MMIO_XMTSEL, 0x0000);

	if (dev->dev->core_rev < 11)
		b43_write16(dev, B43_MMIO_WEPCTL, 0x0000);
	else
		b43_write16(dev, B43_MMIO_WEPCTL, 0x0100);

	value = (ofdm ? 0x41 : 0x40);
	b43_write16(dev, B43_MMIO_TXE0_PHYCTL, value);
	if (phy->type == B43_PHYTYPE_N || phy->type == B43_PHYTYPE_LP ||
	    phy->type == B43_PHYTYPE_LCN)
		b43_write16(dev, B43_MMIO_TXE0_PHYCTL1, 0x1A02);

	b43_write16(dev, B43_MMIO_TXE0_WM_0, 0x0000);
	b43_write16(dev, B43_MMIO_TXE0_WM_1, 0x0000);

	b43_write16(dev, B43_MMIO_XMTTPLATETXPTR, 0x0000);
	b43_write16(dev, B43_MMIO_XMTTXCNT, 0x0014);
	b43_write16(dev, B43_MMIO_XMTSEL, 0x0826);
	b43_write16(dev, B43_MMIO_TXE0_CTL, 0x0000);

	if (!pa_on && phy->type == B43_PHYTYPE_N)
  {
		/*b43_nphy_pa_override(dev, false) */
  }

	switch (phy->type) {
	case B43_PHYTYPE_N:
	case B43_PHYTYPE_LCN:
		b43_write16(dev, B43_MMIO_TXE0_AUX, 0x00D0);
		break;
	case B43_PHYTYPE_LP:
		b43_write16(dev, B43_MMIO_TXE0_AUX, 0x0050);
		break;
	default:
		b43_write16(dev, B43_MMIO_TXE0_AUX, 0x0030);
	}
	b43_read16(dev, B43_MMIO_TXE0_AUX);

	if (phy->radio_ver == 0x2050 && phy->radio_rev <= 0x5)
		b43_radio_write16(dev, 0x0051, 0x0017);
	for (i = 0x00; i < max_loop; i++) {
		value = b43_read16(dev, B43_MMIO_TXE0_STATUS);
		if (value & 0x0080)
			break;
		udelay(10);
	}
	for (i = 0x00; i < 0x0A; i++) {
		value = b43_read16(dev, B43_MMIO_TXE0_STATUS);
		if (value & 0x0400)
			break;
		udelay(10);
	}
	for (i = 0x00; i < 0x19; i++) {
		value = b43_read16(dev, B43_MMIO_IFSSTAT);
		if (!(value & 0x0100))
			break;
		udelay(10);
	}
	if (phy->radio_ver == 0x2050 && phy->radio_rev <= 0x5)
		b43_radio_write16(dev, 0x0051, 0x0037);
}

static void key_write(struct b43_wldev *dev,
		      u8 index, u8 algorithm, const u8 *key)
{
	unsigned int i;
	u32 offset;
	u16 value;
	u16 kidx;

	/* Key index/algo block */
	kidx = b43_kidx_to_fw(dev, index);
	value = ((kidx << 4) | algorithm);
	b43_shm_write16(dev, B43_SHM_SHARED,
			B43_SHM_SH_KEYIDXBLOCK + (kidx * 2), value);

	/* Write the key to the Key Table Pointer offset */
	offset = dev->ktp + (index * B43_SEC_KEYSIZE);
	for (i = 0; i < B43_SEC_KEYSIZE; i += 2) {
		value = key[i];
		value |= (u16) (key[i + 1]) << 8;
		b43_shm_write16(dev, B43_SHM_SHARED, offset + i, value);
	}
}

static void keymac_write(struct b43_wldev *dev, u8 index, const u8 *addr)
{
	u32 addrtmp[2] = { 0, 0, };
	u8 pairwise_keys_start = B43_NR_GROUP_KEYS * 2;

	if (b43_new_kidx_api(dev))
		pairwise_keys_start = B43_NR_GROUP_KEYS;

	B43_WARN_ON(index < pairwise_keys_start);
	/* We have four default TX keys and possibly four default RX keys.
	 * Physical mac 0 is mapped to physical key 4 or 8, depending
	 * on the firmware version.
	 * So we must adjust the index here.
	 */
	index -= pairwise_keys_start;
	B43_WARN_ON(index >= B43_NR_PAIRWISE_KEYS);

	if (addr) {
		addrtmp[0] = addr[0];
		addrtmp[0] |= ((u32) (addr[1]) << 8);
		addrtmp[0] |= ((u32) (addr[2]) << 16);
		addrtmp[0] |= ((u32) (addr[3]) << 24);
		addrtmp[1] = addr[4];
		addrtmp[1] |= ((u32) (addr[5]) << 8);
	}

	/* Receive match transmitter address (RCMTA) mechanism */
	b43_shm_write32(dev, B43_SHM_RCMTA,
			(index * 2) + 0, addrtmp[0]);
	b43_shm_write16(dev, B43_SHM_RCMTA,
			(index * 2) + 1, addrtmp[1]);
}

/* The ucode will use phase1 key with TEK key to decrypt rx packets.
 * When a packet is received, the iv32 is checked.
 * - if it doesn't the packet is returned without modification (and software
 *   decryption can be done). That's what happen when iv16 wrap.
 * - if it does, the rc4 key is computed, and decryption is tried.
 *   Either it will success and B43_RX_MAC_DEC is returned,
 *   either it fails and B43_RX_MAC_DEC|B43_RX_MAC_DECERR is returned
 *   and the packet is not usable (it got modified by the ucode).
 * So in order to never have B43_RX_MAC_DECERR, we should provide
 * a iv32 and phase1key that match. Because we drop packets in case of
 * B43_RX_MAC_DECERR, if we have a correct iv32 but a wrong phase1key, all
 * packets will be lost without higher layer knowing (ie no resync possible
 * until next wrap).
 *
 * NOTE : this should support 50 key like RCMTA because
 * (B43_SHM_SH_KEYIDXBLOCK - B43_SHM_SH_TKIPTSCTTAK)/14 = 50
 */
static void rx_tkip_phase1_write(struct b43_wldev *dev, u8 index, u32 iv32,
		u16 *phase1key)
{
	unsigned int i;
	u32 offset;
	u8 pairwise_keys_start = B43_NR_GROUP_KEYS * 2;

	if (!modparam_hwtkip)
		return;

	if (b43_new_kidx_api(dev))
		pairwise_keys_start = B43_NR_GROUP_KEYS;

	B43_WARN_ON(index < pairwise_keys_start);
	/* We have four default TX keys and possibly four default RX keys.
	 * Physical mac 0 is mapped to physical key 4 or 8, depending
	 * on the firmware version.
	 * So we must adjust the index here.
	 */
	index -= pairwise_keys_start;
	B43_WARN_ON(index >= B43_NR_PAIRWISE_KEYS);

	if (b43_debug(dev, B43_DBG_KEYS)) {
		b43dbg(dev->wl, "rx_tkip_phase1_write : idx 0x%x, iv32 0x%x\n",
				index, iv32);
	}
	/* Write the key to the  RX tkip shared mem */
	offset = B43_SHM_SH_TKIPTSCTTAK + index * (10 + 4);
	for (i = 0; i < 10; i += 2) {
		b43_shm_write16(dev, B43_SHM_SHARED, offset + i,
				phase1key ? phase1key[i / 2] : 0);
	}
	b43_shm_write16(dev, B43_SHM_SHARED, offset + i, iv32);
	b43_shm_write16(dev, B43_SHM_SHARED, offset + i + 2, iv32 >> 16);
}

static void do_key_write(struct b43_wldev *dev,
			 u8 index, u8 algorithm,
			 const u8 *key, size_t key_len, const u8 *mac_addr)
{
	u8 buf[B43_SEC_KEYSIZE] = { 0, };
	u8 pairwise_keys_start = B43_NR_GROUP_KEYS * 2;

	if (b43_new_kidx_api(dev))
		pairwise_keys_start = B43_NR_GROUP_KEYS;

	B43_WARN_ON(index >= ARRAY_SIZE(dev->key));
	B43_WARN_ON(key_len > B43_SEC_KEYSIZE);

	if (index >= pairwise_keys_start)
		keymac_write(dev, index, NULL);	/* First zero out mac. */
	if (algorithm == B43_SEC_ALGO_TKIP) {
		/*
		 * We should provide an initial iv32, phase1key pair.
		 * We could start with iv32=0 and compute the corresponding
		 * phase1key, but this means calling ieee80211_get_tkip_key
		 * with a fake skb (or export other tkip function).
		 * Because we are lazy we hope iv32 won't start with
		 * 0xffffffff and let's b43_op_update_tkip_key provide a
		 * correct pair.
		 */
		rx_tkip_phase1_write(dev, index, 0xffffffff, (u16*)buf);
	} else if (index >= pairwise_keys_start) /* clear it */
		rx_tkip_phase1_write(dev, index, 0, NULL);
	if (key)
		memcpy(buf, key, key_len);
	key_write(dev, index, algorithm, buf);
	if (index >= pairwise_keys_start)
		keymac_write(dev, index, mac_addr);

	dev->key[index].algorithm = algorithm;
}

static int b43_key_clear(struct b43_wldev *dev, int index)
{
	if (B43_WARN_ON((index < 0) || ((unsigned int)index >= ARRAY_SIZE(dev->key))))
		return -EINVAL;
	do_key_write(dev, index, B43_SEC_ALGO_NONE,
		     NULL, B43_SEC_KEYSIZE, NULL);
	if ((index <= 3) && !b43_new_kidx_api(dev)) {
		do_key_write(dev, index + 4, B43_SEC_ALGO_NONE,
			     NULL, B43_SEC_KEYSIZE, NULL);
	}

	return 0;
}

static void b43_clear_keys(struct b43_wldev *dev)
{
	unsigned int i, count;

	if (b43_new_kidx_api(dev))
		count = B43_NR_GROUP_KEYS + B43_NR_PAIRWISE_KEYS;
	else
		count = B43_NR_GROUP_KEYS * 2 + B43_NR_PAIRWISE_KEYS;
	for (i = 0; i < count; i++)
		b43_key_clear(dev, i);
}

void b43_power_saving_ctl_bits(struct b43_wldev *dev, unsigned int ps_flags)
{
	u32 macctl;
	u16 ucstat;
	bool hwps;
	bool awake;
	int i;

	B43_WARN_ON((ps_flags & B43_PS_ENABLED) &&
		    (ps_flags & B43_PS_DISABLED));
	B43_WARN_ON((ps_flags & B43_PS_AWAKE) && (ps_flags & B43_PS_ASLEEP));

	if (ps_flags & B43_PS_ENABLED) {
		hwps = true;
	} else if (ps_flags & B43_PS_DISABLED) {
		hwps = false;
	} else {
		//TODO: If powersave is not off and FIXME is not set and we are not in adhoc
		//      and thus is not an AP and we are associated, set bit 25
	}
	if (ps_flags & B43_PS_AWAKE) {
		awake = true;
	} else if (ps_flags & B43_PS_ASLEEP) {
		awake = false;
	} else {
		//TODO: If the device is awake or this is an AP, or we are scanning, or FIXME,
		//      or we are associated, or FIXME, or the latest PS-Poll packet sent was
		//      successful, set bit26
	}

/* FIXME: For now we force awake-on and hwps-off */
	hwps = false;
	awake = true;

	macctl = b43_read32(dev, B43_MMIO_MACCTL);
	if (hwps)
		macctl |= B43_MACCTL_HWPS;
	else
		macctl &= ~B43_MACCTL_HWPS;
	if (awake)
		macctl |= B43_MACCTL_AWAKE;
	else
		macctl &= ~B43_MACCTL_AWAKE;
	b43_write32(dev, B43_MMIO_MACCTL, macctl);
	/* Commit write */
	b43_read32(dev, B43_MMIO_MACCTL);
	if (awake && dev->dev->core_rev >= 5) {
		/* Wait for the microcode to wake up. */
		for (i = 0; i < 100; i++) {
			ucstat = b43_shm_read16(dev, B43_SHM_SHARED,
						B43_SHM_SH_UCODESTAT);
			if (ucstat != B43_SHM_SH_UCODESTAT_SLEEP)
				break;
			udelay(10);
		}
	}
}

/* http://bcm-v4.sipsolutions.net/802.11/PHY/BmacCorePllReset */
void b43_wireless_core_phy_pll_reset(struct b43_wldev *dev)
{
	struct bcma_drv_cc *bcma_cc __maybe_unused;
	struct ssb_chipcommon *ssb_cc __maybe_unused;

	switch (dev->dev->bus_type) {
#ifdef CONFIG_B43_BCMA
	case B43_BUS_BCMA:
		bcma_cc = &dev->dev->bdev->bus->drv_cc;

		bcma_cc_write32(bcma_cc, BCMA_CC_CHIPCTL_ADDR, 0);
		bcma_cc_mask32(bcma_cc, BCMA_CC_CHIPCTL_DATA, ~0x4);
		bcma_cc_set32(bcma_cc, BCMA_CC_CHIPCTL_DATA, 0x4);
		bcma_cc_mask32(bcma_cc, BCMA_CC_CHIPCTL_DATA, ~0x4);
		break;
#endif
#ifdef CONFIG_B43_SSB
	case B43_BUS_SSB:
		ssb_cc = &dev->dev->sdev->bus->chipco;

		chipco_write32(ssb_cc, SSB_CHIPCO_CHIPCTL_ADDR, 0);
		chipco_mask32(ssb_cc, SSB_CHIPCO_CHIPCTL_DATA, ~0x4);
		chipco_set32(ssb_cc, SSB_CHIPCO_CHIPCTL_DATA, 0x4);
		chipco_mask32(ssb_cc, SSB_CHIPCO_CHIPCTL_DATA, ~0x4);
		break;
#endif
	}
}

#ifdef CONFIG_B43_BCMA
static void b43_bcma_phy_reset(struct b43_wldev *dev)
{
	u32 flags;

	/* Put PHY into reset */
	flags = bcma_aread32(dev->dev->bdev, BCMA_IOCTL);
	flags |= B43_BCMA_IOCTL_PHY_RESET;
	flags |= B43_BCMA_IOCTL_PHY_BW_20MHZ; /* Make 20 MHz def */
	bcma_awrite32(dev->dev->bdev, BCMA_IOCTL, flags);
	udelay(2);

	b43_phy_take_out_of_reset(dev);
}

static void b43_bcma_wireless_core_reset(struct b43_wldev *dev, bool gmode)
{
	u32 req = B43_BCMA_CLKCTLST_80211_PLL_REQ |
		  B43_BCMA_CLKCTLST_PHY_PLL_REQ;
	u32 status = B43_BCMA_CLKCTLST_80211_PLL_ST |
		     B43_BCMA_CLKCTLST_PHY_PLL_ST;
	u32 flags;

	flags = B43_BCMA_IOCTL_PHY_CLKEN;
	if (gmode)
		flags |= B43_BCMA_IOCTL_GMODE;
	b43_device_enable(dev, flags);

	bcma_core_set_clockmode(dev->dev->bdev, BCMA_CLKMODE_FAST);
	b43_bcma_phy_reset(dev);
	bcma_core_pll_ctl(dev->dev->bdev, req, status, true);
}
#endif

#ifdef CONFIG_B43_SSB
static void b43_ssb_wireless_core_reset(struct b43_wldev *dev, bool gmode)
{
	u32 flags = 0;

	if (gmode)
		flags |= B43_TMSLOW_GMODE;
	flags |= B43_TMSLOW_PHYCLKEN;
	flags |= B43_TMSLOW_PHYRESET;
	if (dev->phy.type == B43_PHYTYPE_N)
		flags |= B43_TMSLOW_PHY_BANDWIDTH_20MHZ; /* Make 20 MHz def */
	b43_device_enable(dev, flags);
	msleep(2);		/* Wait for the PLL to turn on. */

	b43_phy_take_out_of_reset(dev);
}
#endif

void b43_wireless_core_reset(struct b43_wldev *dev, bool gmode)
{
	u32 macctl;

	switch (dev->dev->bus_type) {
#ifdef CONFIG_B43_BCMA
	case B43_BUS_BCMA:
		b43_bcma_wireless_core_reset(dev, gmode);
		break;
#endif
#ifdef CONFIG_B43_SSB
	case B43_BUS_SSB:
		b43_ssb_wireless_core_reset(dev, gmode);
		break;
#endif
	}

	/* Turn Analog ON, but only if we already know the PHY-type.
	 * This protects against very early setup where we don't know the
	 * PHY-type, yet. wireless_core_reset will be called once again later,
	 * when we know the PHY-type. */
	if (dev->phy.ops)
		dev->phy.ops->switch_analog(dev, 1);

	macctl = b43_read32(dev, B43_MMIO_MACCTL);
	macctl &= ~B43_MACCTL_GMODE;
	if (gmode)
		macctl |= B43_MACCTL_GMODE;
	macctl |= B43_MACCTL_IHR_ENABLED;
	b43_write32(dev, B43_MMIO_MACCTL, macctl);
}

static void handle_irq_transmit_status(struct b43_wldev *dev)
{
	u32 v0, v1;
	u16 tmp;
	struct b43_txstatus stat;

	while (1) {
		v0 = b43_read32(dev, B43_MMIO_XMITSTAT_0);
		if (!(v0 & 0x00000001))
			break;
		v1 = b43_read32(dev, B43_MMIO_XMITSTAT_1);

		stat.cookie = (v0 >> 16);
		stat.seq = (v1 & 0x0000FFFF);
		stat.phy_stat = ((v1 & 0x00FF0000) >> 16);
		tmp = (v0 & 0x0000FFFF);
		stat.frame_count = ((tmp & 0xF000) >> 12);
		stat.rts_count = ((tmp & 0x0F00) >> 8);
		stat.supp_reason = ((tmp & 0x001C) >> 2);
		stat.pm_indicated = !!(tmp & 0x0080);
		stat.intermediate = !!(tmp & 0x0040);
		stat.for_ampdu = !!(tmp & 0x0020);
		stat.acked = !!(tmp & 0x0002);

		b43_handle_txstatus(dev, &stat);
	}
}

static void drain_txstatus_queue(struct b43_wldev *dev)
{
	u32 dummy;

	if (dev->dev->core_rev < 5)
		return;
	/* Read all entries from the microcode TXstatus FIFO
	 * and throw them away.
	 */
	while (1) {
		dummy = b43_read32(dev, B43_MMIO_XMITSTAT_0);
		if (!(dummy & 0x00000001))
			break;
		dummy = b43_read32(dev, B43_MMIO_XMITSTAT_1);
	}
}

static u32 b43_jssi_read(struct b43_wldev *dev)
{
	u32 val = 0;

	val = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_JSSI1);
	val <<= 16;
	val |= b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_JSSI0);

	return val;
}

static void b43_jssi_write(struct b43_wldev *dev, u32 jssi)
{
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_JSSI0,
			(jssi & 0x0000FFFF));
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_JSSI1,
			(jssi & 0xFFFF0000) >> 16);
}

static void b43_generate_noise_sample(struct b43_wldev *dev)
{
	b43_jssi_write(dev, 0x7F7F7F7F);
	b43_write32(dev, B43_MMIO_MACCMD,
		    b43_read32(dev, B43_MMIO_MACCMD) | B43_MACCMD_BGNOISE);
}

static void b43_calculate_link_quality(struct b43_wldev *dev)
{
	/* Top half of Link Quality calculation. */

	if (dev->phy.type != B43_PHYTYPE_G)
		return;
	if (dev->noisecalc.calculation_running)
		return;
	dev->noisecalc.calculation_running = true;
	dev->noisecalc.nr_samples = 0;

	b43_generate_noise_sample(dev);
}

static void handle_irq_noise(struct b43_wldev *dev)
{
	struct b43_phy_g *phy = dev->phy.g;
	u16 tmp;
	u8 noise[4];
	u8 i, j;
	s32 average;

	/* Bottom half of Link Quality calculation. */

	if (dev->phy.type != B43_PHYTYPE_G)
		return;

	/* Possible race condition: It might be possible that the user
	 * changed to a different channel in the meantime since we
	 * started the calculation. We ignore that fact, since it's
	 * not really that much of a problem. The background noise is
	 * an estimation only anyway. Slightly wrong results will get damped
	 * by the averaging of the 8 sample rounds. Additionally the
	 * value is shortlived. So it will be replaced by the next noise
	 * calculation round soon. */

	B43_WARN_ON(!dev->noisecalc.calculation_running);
	*((__le32 *)noise) = cpu_to_le32(b43_jssi_read(dev));
	if (noise[0] == 0x7F || noise[1] == 0x7F ||
	    noise[2] == 0x7F || noise[3] == 0x7F)
		goto generate_new;

	/* Get the noise samples. */
	B43_WARN_ON(dev->noisecalc.nr_samples >= 8);
	i = dev->noisecalc.nr_samples;
	noise[0] = clamp_val(noise[0], 0, ARRAY_SIZE(phy->nrssi_lt) - 1);
	noise[1] = clamp_val(noise[1], 0, ARRAY_SIZE(phy->nrssi_lt) - 1);
	noise[2] = clamp_val(noise[2], 0, ARRAY_SIZE(phy->nrssi_lt) - 1);
	noise[3] = clamp_val(noise[3], 0, ARRAY_SIZE(phy->nrssi_lt) - 1);
	dev->noisecalc.samples[i][0] = phy->nrssi_lt[noise[0]];
	dev->noisecalc.samples[i][1] = phy->nrssi_lt[noise[1]];
	dev->noisecalc.samples[i][2] = phy->nrssi_lt[noise[2]];
	dev->noisecalc.samples[i][3] = phy->nrssi_lt[noise[3]];
	dev->noisecalc.nr_samples++;
	if (dev->noisecalc.nr_samples == 8) {
		/* Calculate the Link Quality by the noise samples. */
		average = 0;
		for (i = 0; i < 8; i++) {
			for (j = 0; j < 4; j++)
				average += dev->noisecalc.samples[i][j];
		}
		average /= (8 * 4);
		average *= 125;
		average += 64;
		average /= 128;
		tmp = b43_shm_read16(dev, B43_SHM_SHARED, 0x40C);
		tmp = (tmp / 128) & 0x1F;
		if (tmp >= 8)
			average += 2;
		else
			average -= 25;
		if (tmp == 8)
			average -= 72;
		else
			average -= 48;

		dev->stats.link_noise = average;
		dev->noisecalc.calculation_running = false;
		return;
	}
generate_new:
	b43_generate_noise_sample(dev);
}

static void handle_irq_tbtt_indication(struct b43_wldev *dev)
{
		if (1 /*FIXME: the last PSpoll frame was sent successfully */ )
			b43_power_saving_ctl_bits(dev, 0);
}

static void handle_irq_atim_end(struct b43_wldev *dev)
{
	if (dev->dfq_valid) {
		b43_write32(dev, B43_MMIO_MACCMD,
			    b43_read32(dev, B43_MMIO_MACCMD)
			    | B43_MACCMD_DFQ_VALID);
		dev->dfq_valid = false;
	}
}

static void handle_irq_pmq(struct b43_wldev *dev)
{
	u32 tmp;

	//TODO: AP mode.

	while (1) {
		tmp = b43_read32(dev, B43_MMIO_PS_STATUS);
		if (!(tmp & 0x00000008))
			break;
	}
	/* 16bit write is odd, but correct. */
	b43_write16(dev, B43_MMIO_PS_STATUS, 0x0002);
}

/* Check if the use of the antenna that ieee80211 told us to
 * use is possible. This will fall back to DEFAULT.
 * "antenna_nr" is the antenna identifier we got from ieee80211. */
u8 b43_ieee80211_antenna_sanitize(struct b43_wldev *dev,
				  u8 antenna_nr)
{
	u8 antenna_mask;

	if (antenna_nr == 0) {
		/* Zero means "use default antenna". That's always OK. */
		return 0;
	}

	/* Get the mask of available antennas. */
	if (dev->phy.gmode)
		antenna_mask = dev->dev->bus_sprom->ant_available_bg;
	else
		antenna_mask = dev->dev->bus_sprom->ant_available_a;

	if (!(antenna_mask & (1 << (antenna_nr - 1)))) {
		/* This antenna is not available. Fall back to default. */
		return 0;
	}

	return antenna_nr;
}

/* Convert a b43 antenna number value to the PHY TX control value. */
static u16 b43_antenna_to_phyctl(int antenna)
{
	switch (antenna) {
	case B43_ANTENNA0:
		return B43_TXH_PHY_ANT0;
	case B43_ANTENNA1:
		return B43_TXH_PHY_ANT1;
	case B43_ANTENNA2:
		return B43_TXH_PHY_ANT2;
	case B43_ANTENNA3:
		return B43_TXH_PHY_ANT3;
	case B43_ANTENNA_AUTO0:
	case B43_ANTENNA_AUTO1:
		return B43_TXH_PHY_ANT01AUTO;
	}
	B43_WARN_ON(1);
	return 0;
}

static void b43_handle_firmware_panic(struct b43_wldev *dev)
{
	u16 reason;

	/* Read the register that contains the reason code for the panic. */
	reason = b43_shm_read16(dev, B43_SHM_SCRATCH, B43_FWPANIC_REASON_REG);
	b43err(dev->wl, "Whoopsy, firmware panic! Reason: %d\n", reason);

	switch (reason) {
	default:
		b43dbg(dev->wl, "The panic reason is unknown.\n");
		/* fallthrough */
	case B43_FWPANIC_DIE:
		/* Do not restart the controller or firmware.
		 * The device is nonfunctional from now on.
		 * Restarting would result in this panic to trigger again,
		 * so we avoid that recursion. */
		break;
	case B43_FWPANIC_RESTART:
		b43_controller_restart(dev, "Microcode panic");
		break;
	}
}

static void handle_irq_ucode_debug(struct b43_wldev *dev)
{
	unsigned int i, cnt;
	u16 reason, marker_id, marker_line;
	__le16 *buf;

	/* The proprietary firmware doesn't have this IRQ. */
	if (!dev->fw.opensource)
		return;

	/* Read the register that contains the reason code for this IRQ. */
	reason = b43_shm_read16(dev, B43_SHM_SCRATCH, B43_DEBUGIRQ_REASON_REG);

	switch (reason) {
	case B43_DEBUGIRQ_PANIC:
		b43_handle_firmware_panic(dev);
		break;
	case B43_DEBUGIRQ_DUMP_SHM:
		if (!B43_DEBUG)
			break; /* Only with driver debugging enabled. */
		buf = kmalloc(4096, GFP_ATOMIC);
		if (!buf) {
			b43dbg(dev->wl, "SHM-dump: Failed to allocate memory\n");
			goto out;
		}
		for (i = 0; i < 4096; i += 2) {
			u16 tmp = b43_shm_read16(dev, B43_SHM_SHARED, i);
			buf[i / 2] = cpu_to_le16(tmp);
		}
		b43info(dev->wl, "Shared memory dump:\n");
		// Give access to ipxe debug hexdump functions even if we're not in
		// debug mode so we can capture this important data
#if !(DBGLOG)
#undef DBGLOG
#define DBGLOG 1
		DBG_HD(buf, 4096);
#undef DBGLOG
#define DBGLOG 0
#else
		DBG_HD(buf, 4096);
#endif
		kfree(buf);
		break;
	case B43_DEBUGIRQ_DUMP_REGS:
		if (!B43_DEBUG)
			break; /* Only with driver debugging enabled. */
		b43info(dev->wl, "Microcode register dump:\n");
		for (i = 0, cnt = 0; i < 64; i++) {
			u16 tmp = b43_shm_read16(dev, B43_SHM_SCRATCH, i);
			if (cnt == 0)
				printk(KERN_INFO);
			printk("r%02d: 0x%04X  ", i, tmp);
			cnt++;
			if (cnt == 6) {
				printk("\n");
				cnt = 0;
			}
		}
		printk("\n");
		break;
	case B43_DEBUGIRQ_MARKER:
		if (!B43_DEBUG)
			break; /* Only with driver debugging enabled. */
		marker_id = b43_shm_read16(dev, B43_SHM_SCRATCH,
					   B43_MARKER_ID_REG);
		marker_line = b43_shm_read16(dev, B43_SHM_SCRATCH,
					     B43_MARKER_LINE_REG);
		b43info(dev->wl, "The firmware just executed the MARKER(%d) "
			"at line number %d\n",
			marker_id, marker_line);
		break;
	default:
		b43dbg(dev->wl, "Debug-IRQ triggered for unknown reason: %d\n",
		       reason);
	}
out:
	/* Acknowledge the debug-IRQ, so the firmware can continue. */
	b43_shm_write16(dev, B43_SHM_SCRATCH,
			B43_DEBUGIRQ_REASON_REG, B43_DEBUGIRQ_ACK);
}

static void b43_do_interrupt_thread(struct b43_wldev *dev)
{
	u32 reason;
	u32 dma_reason[ARRAY_SIZE(dev->dma_reason)];
	u32 merged_dma_reason = 0;
	unsigned int i;

	if (unlikely(b43_status(dev) != B43_STAT_STARTED))
		return;

	reason = dev->irq_reason;
	for (i = 0; i < ARRAY_SIZE(dma_reason); i++) {
		dma_reason[i] = dev->dma_reason[i];
		merged_dma_reason |= dma_reason[i];
	}

	if (unlikely(reason & B43_IRQ_MAC_TXERR))
		b43err(dev->wl, "MAC transmission error\n");

	if (unlikely(reason & B43_IRQ_PHY_TXERR)) {
		// see too many of these even though sha1sums are perfect, lets ignore
		b43err(dev->wl, "PHY transmission error\n");
		rmb();
		if (unlikely(atomic_dec_and_test(&dev->phy.txerr_cnt))) {
			atomic_set(&dev->phy.txerr_cnt,
				   B43_PHY_TX_BADNESS_LIMIT);
			b43err(dev->wl, "Too many PHY TX errors, "
					"restarting the controller\n");
			b43_controller_restart(dev, "PHY TX errors");
		}
	}

	if (unlikely(merged_dma_reason & (B43_DMAIRQ_FATALMASK))) {
		b43err(dev->wl,
			"Fatal DMA error: 0x%08X, 0x%08X, 0x%08X, 0x%08X, 0x%08X, 0x%08X\n",
			dma_reason[0], dma_reason[1],
			dma_reason[2], dma_reason[3],
			dma_reason[4], dma_reason[5]);
		b43err(dev->wl, "This device does not support DMA "
			       "on your system. It will now be switched to PIO.\n");
		/* Fall back to PIO transfers if we get fatal DMA errors! */
		dev->use_pio = true;
		b43_controller_restart(dev, "DMA error");
		return;
	}

	if (unlikely(reason & B43_IRQ_UCODE_DEBUG)) {
		DBGC(dev, "IRQ: Ucode Debug\n");
		handle_irq_ucode_debug(dev);
	}
	if (reason & B43_IRQ_TBTT_INDI) {
		DBGC(dev, "IRQ: TBTT INDI\n");
		handle_irq_tbtt_indication(dev);
	}
	if (reason & B43_IRQ_ATIM_END) {
		DBGC(dev, "IRQ: ATIM END\n");
		handle_irq_atim_end(dev);
	}
	if (reason & B43_IRQ_PMQ) {
		DBGC(dev, "IRQ: PMQ\n");
		handle_irq_pmq(dev);
	}
	if (reason & B43_IRQ_TXFIFO_FLUSH_OK) {
		DBGC(dev, "IRQ: TXFIFO FLUSH\n");
		;/* TODO */
	}
	if (reason & B43_IRQ_NOISESAMPLE_OK) {
		DBGC(dev, "IRQ: NOISESAMPLE OK\n");
		handle_irq_noise(dev);
	}

	/* Check the DMA reason registers for received data. */
	if (dma_reason[0] & B43_DMAIRQ_RDESC_UFLOW) {
		if (B43_DEBUG)
			b43warn(dev->wl, "RX descriptor underrun\n");
		b43_dma_handle_rx_overflow(dev->dma.rx_ring);
	}
	if (dma_reason[0] & B43_DMAIRQ_RX_DONE) {
		if (b43_using_pio_transfers(dev))
			b43_pio_rx(dev->pio.rx_queue);
		else
			b43_dma_rx(dev->dma.rx_ring);
	}
	B43_WARN_ON(dma_reason[1] & B43_DMAIRQ_RX_DONE);
	B43_WARN_ON(dma_reason[2] & B43_DMAIRQ_RX_DONE);
	B43_WARN_ON(dma_reason[3] & B43_DMAIRQ_RX_DONE);
	B43_WARN_ON(dma_reason[4] & B43_DMAIRQ_RX_DONE);
	B43_WARN_ON(dma_reason[5] & B43_DMAIRQ_RX_DONE);

	if (reason & B43_IRQ_TX_OK) {
		handle_irq_transmit_status(dev);
	}

	/* Re-enable interrupts on the device by restoring the current interrupt mask. */
	b43_write32(dev, B43_MMIO_GEN_IRQ_MASK, dev->irq_mask); //b43_irq_mask(dev));

#if B43_DEBUG
	if (b43_debug(dev, B43_DBG_VERBOSESTATS)) {
		dev->irq_count++;
		for (i = 0; i < ARRAY_SIZE(dev->irq_bit_count); i++) {
			if (reason & (1 << i))
				dev->irq_bit_count[i]++;
		}
	}
#endif
}

static irqreturn_t b43_do_interrupt(struct b43_wldev *dev)
{
	u32 reason;
	unsigned long timestamp = currticks();

	if(timestamp >= dev->periodic_work && dev->periodic_work)
		b43_periodic_work_handler(dev);

	reason = b43_read32(dev, B43_MMIO_GEN_IRQ_REASON);
	if (reason == 0xffffffff)	{ /* shared IRQ */
		DBGC(dev, "IRQ: NONE\n");
		return IRQ_NONE;
	}
	reason &= dev->irq_mask;
	if (!reason)
		return IRQ_NONE;

	dev->dma_reason[0] = b43_read32(dev, B43_MMIO_DMA0_REASON)
	    & 0x0001FC00;
	dev->dma_reason[1] = b43_read32(dev, B43_MMIO_DMA1_REASON)
	    & 0x0000DC00;
	dev->dma_reason[2] = b43_read32(dev, B43_MMIO_DMA2_REASON)
	    & 0x0000DC00;
	dev->dma_reason[3] = b43_read32(dev, B43_MMIO_DMA3_REASON)
	    & 0x0001DC00;
	dev->dma_reason[4] = b43_read32(dev, B43_MMIO_DMA4_REASON)
	    & 0x0000DC00;
/* Unused ring
	dev->dma_reason[5] = b43_read32(dev, B43_MMIO_DMA5_REASON)
	    & 0x0000DC00;
*/

	/* ACK the interrupt. */
	b43_write32(dev, B43_MMIO_GEN_IRQ_REASON, reason);
	b43_write32(dev, B43_MMIO_DMA0_REASON, dev->dma_reason[0]);
	b43_write32(dev, B43_MMIO_DMA1_REASON, dev->dma_reason[1]);
	b43_write32(dev, B43_MMIO_DMA2_REASON, dev->dma_reason[2]);
	b43_write32(dev, B43_MMIO_DMA3_REASON, dev->dma_reason[3]);
	b43_write32(dev, B43_MMIO_DMA4_REASON, dev->dma_reason[4]);
/* Unused ring
	b43_write32(dev, B43_MMIO_DMA5_REASON, dev->dma_reason[5]);
*/

	/* Disable IRQs on the device. The IRQ thread handler will re-enable them. */
	b43_write32(dev, B43_MMIO_GEN_IRQ_MASK, 0);
	/* Save the reason bitmasks for the IRQ thread handler. */
	dev->irq_reason = reason;

	return IRQ_WAKE_THREAD;
}

/* interrupt handler. This runs in process context. */
static void b43_interrupt_handler(struct net80211_device *ndev)
{
	irqreturn_t ret;
	struct b43_wl *wl = ndev_to_b43_wl(ndev);
	struct b43_wldev *wldev = wl->current_dev;

	assert(wldev != NULL);

	ret = b43_do_interrupt(wldev);
	if (ret == IRQ_WAKE_THREAD)
		b43_do_interrupt_thread(wldev);
}

static void b43_irq(struct net80211_device *ndev, int enable)
{
	struct b43_wl *wl = ndev_to_b43_wl(ndev);
	struct b43_wldev *wldev = wl->current_dev;
	//DBGC(ndev, "b43_irq; ");

	assert(wldev != NULL);

	wldev->irq_enable = enable;

	b43_write32(wldev, B43_MMIO_GEN_IRQ_MASK, b43_irq_mask(wldev));
}

void b43_do_release_fw(struct b43_firmware_file *fw)
{
	fw->data = NULL;
	fw->filename = NULL;
}

static void b43_release_firmware(struct b43_wldev *dev)
{
	b43_do_release_fw(&dev->fw.ucode);
	b43_do_release_fw(&dev->fw.pcm);
	b43_do_release_fw(&dev->fw.initvals);
	b43_do_release_fw(&dev->fw.initvals_band);
}

static void b43_print_fw_helptext(struct b43_wl *wl, bool error)
{
	const char text[] =
		"You must go to " \
		"http://wireless.kernel.org/en/users/Drivers/b43#devicefirmware " \
		"and download the correct firmware for this driver version. " \
		"Please carefully read all instructions on this website.\n";

	if (error)
		b43err(wl, text);
	else
		b43warn(wl, text);
}

static int b43_do_request_fw(struct b43_request_fw_context *ctx,
					const struct b43_linux_firmware *blob,
		      const char *name, struct b43_firmware_file *fw)
{
	struct b43_fw_header *hdr;
	u32 size;

	if (!name) {
		/* Don't fetch anything. Free possibly cached firmware. */
		/* FIXME: We should probably keep it anyway, to save some headache
		 * on suspend/resume with multiband devices. */
		b43_do_release_fw(fw);
		return 0;
	}
	b43dbg(ctx->dev->wl, "Requesting Firmware file \"%s\"\n", name);
	if (fw->filename) {
		if ((fw->type == ctx->req_type) &&
		    (strcmp(fw->filename, name) == 0))
			return 0; /* Already have this fw. */
		/* Free the cached firmware first. */
		/* FIXME: We should probably do this later after we successfully
		 * got the new fw. This could reduce headache with multiband devices.
		 * We could also redesign this to cache the firmware for all possible
		 * bands all the time. */
		b43_do_release_fw(fw);
	}

	/* FIXME ipxe; here is the conditional section whether to load open
	 * source firmware or proprietary.*/
	switch (ctx->req_type) {
	case B43_FWTYPE_PROPRIETARY:
		break;
	case B43_FWTYPE_OPENSOURCE:
		break;
	default:
		B43_WARN_ON(1);
		return -ENOSYS;
	}

	ctx->blob = blob;

	if (ctx->blob->size < sizeof(struct b43_fw_header))
		goto err_format;
	hdr = (struct b43_fw_header *)(ctx->blob->data);
	switch (hdr->type) {
	case B43_FW_TYPE_UCODE:
	case B43_FW_TYPE_PCM:
		size = be32_to_cpu(hdr->size);
		if (size != ctx->blob->size - sizeof(struct b43_fw_header))
			goto err_format;
		/* fallthrough */
	case B43_FW_TYPE_IV:
		if (hdr->ver != 1)
			goto err_format;
		break;
	default:
		goto err_format;
	}

	fw->data = ctx->blob;
	fw->filename = name;
	fw->type = ctx->req_type;

	return 0;

err_format:
	snprintf(ctx->errors[ctx->req_type],
		 sizeof(ctx->errors[ctx->req_type]),
		 "Firmware file \"%s\" format error.\n", ctx->fwname);
	return -EPROTO;
}

/* http://bcm-v4.sipsolutions.net/802.11/Init/Firmware */
static int b43_try_request_fw(struct b43_request_fw_context *ctx)
{
	struct b43_wldev *dev = ctx->dev;
	struct b43_firmware *fw = &ctx->dev->fw;
	struct b43_phy *phy = &dev->phy;
	const u8 rev = ctx->dev->dev->core_rev;
	const char *filename;
	const struct b43_linux_firmware *blob;
	int err;

	/* Get microcode */
	filename = NULL;
	switch (rev) {
	case 42:
		if (phy->type == B43_PHYTYPE_AC)
		{
			filename = "ucode42";
			blob = &b43_fw_ucode42;
		}
		break;
	case 40:
		if (phy->type == B43_PHYTYPE_AC)
		{
			filename = "ucode40";
			blob = &b43_fw_ucode40;
		}
		break;
	case 33:
		if (phy->type == B43_PHYTYPE_LCN40)
		{
			filename = "ucode33_lcn40";
			blob = &b43_fw_ucode33_lcn40;
		}
		break;
	case 30:
		if (phy->type == B43_PHYTYPE_N)
		{
			filename = "ucode30_mimo";
			blob = &b43_fw_ucode30_mimo;
		}
		break;
	case 29:
		if (phy->type == B43_PHYTYPE_HT)
		{
			filename = "ucode29_mimo";
			blob = &b43_fw_ucode29_mimo;
		}
		break;
	case 26:
		if (phy->type == B43_PHYTYPE_HT)
		{
			filename = "ucode26_mimo";
			blob = &b43_fw_ucode26_mimo;
		}
		break;
	case 28:
	case 25:
		if (phy->type == B43_PHYTYPE_N)
		{
			filename = "ucode25_mimo";
			blob = &b43_fw_ucode25_mimo;
		}
		else if (phy->type == B43_PHYTYPE_LCN)
		{
			filename = "ucode25_lcn";
			blob = &b43_fw_ucode25_lcn;
		}
		break;
	case 24:
		if (phy->type == B43_PHYTYPE_LCN)
		{
			filename = "ucode24_lcn";
			blob = &b43_fw_ucode24_lcn;
		}
		break;
	case 23:
		if (phy->type == B43_PHYTYPE_N)
		{
			filename = "ucode16_mimo";
			blob = &b43_fw_ucode16_mimo;
		}
		break;
	case 16 ... 19:
		if (phy->type == B43_PHYTYPE_N)
		{
			filename = "ucode16_mimo";
			blob = &b43_fw_ucode16_mimo;
		}
		else if (phy->type == B43_PHYTYPE_LP)
		{
			filename = "ucode16_lp";
			blob = &b43_fw_ucode16_lp;
		}
		break;
	case 15:
		filename = "ucode15";
		blob = &b43_fw_ucode15;
		break;
	case 14:
		filename = "ucode14";
		blob = &b43_fw_ucode14;
		break;
	case 13:
		filename = "ucode13";
		blob = &b43_fw_ucode13;
		break;
	case 11 ... 12:
		filename = "ucode11";
		blob = &b43_fw_ucode11;
		break;
	case 5 ... 10:
		filename = "ucode5";
		blob = &b43_fw_ucode5;
		break;
	}
	if (!filename)
		goto err_no_ucode;
	err = b43_do_request_fw(ctx, blob, filename, &fw->ucode);
	if (err)
		goto err_load;

	/* Get PCM code */
	if ((rev >= 5) && (rev <= 10))
	{
		filename = "pcm5";
		blob = &b43_fw_pcm5;
	}
	else if (rev >= 11)
		filename = NULL;
	else
		goto err_no_pcm;
	fw->pcm_request_failed = false;
	err = b43_do_request_fw(ctx, blob, filename, &fw->pcm);
	if (err == -ENOENT) {
		/* We did not find a PCM file? Not fatal, but
		 * core rev <= 10 must do without hwcrypto then. */
		fw->pcm_request_failed = true;
	} else if (err)
		goto err_load;

	/* Get initvals */
	filename = NULL;
	switch (dev->phy.type) {
	case B43_PHYTYPE_G:
		if (rev == 13)
		{
			filename = "b0g0initvals13";
			blob = &b43_fw_b0g0initvals13;
		}
		else if (rev >= 5 && rev <= 10)
		{
			filename = "b0g0initvals5";
			blob = &b43_fw_b0g0initvals5;
		}
		break;
	case B43_PHYTYPE_N:
		if (rev == 30)
		{
			filename = "n16initvals30";
			blob = &b43_fw_n16initvals30;
		}
		else if (rev == 28 || rev == 25)
		{
			filename = "n0initvals25";
			blob = &b43_fw_n0initvals25;
		}
		else if (rev == 24)
		{
			filename = "n0initvals24";
			blob = &b43_fw_n0initvals24;
		}
		else if (rev == 23)
		{
			filename = "n0initvals16"; /* What about n0initvals22? */
			blob = &b43_fw_n0initvals16;
		}
		else if (rev >= 16 && rev <= 18)
		{
			filename = "n0initvals16";
			blob = &b43_fw_n0initvals16;
		}
		else if (rev >= 11 && rev <= 12)
		{
			filename = "n0initvals11";
			blob = &b43_fw_n0initvals11;
		}
		break;
	case B43_PHYTYPE_LP:
		if (rev >= 16 && rev <= 18)
		{
			filename = "lp0initvals16";
			blob = &b43_fw_lp0initvals16;
		}
		else if (rev == 15)
		{
			filename = "lp0initvals15";
			blob = &b43_fw_lp0initvals15;
		}
		else if (rev == 14)
		{
			filename = "lp0initvals14";
			blob = &b43_fw_lp0initvals14;
		}
		else if (rev == 13)
		{
			filename = "lp0initvals13";
			blob = &b43_fw_lp0initvals13;
		}
		break;
	case B43_PHYTYPE_HT:
		if (rev == 29)
		{
			filename = "ht0initvals29";
			blob = &b43_fw_ht0initvals29;
		}
		else if (rev == 26)
		{
			filename = "ht0initvals26";
			blob = &b43_fw_ht0initvals26;
		}
		break;
	case B43_PHYTYPE_LCN:
		if (rev == 24)
		{
			filename = "lcn0initvals24";
			blob = &b43_fw_lcn0initvals24;
		}
		break;
	case B43_PHYTYPE_LCN40:
		if (rev == 33)
		{
			filename = "lcn400initvals33";
			blob = &b43_fw_lcn400initvals33;
		}
		break;
	case B43_PHYTYPE_AC:
		if (rev == 42)
		{
			filename = "ac1initvals42";
			blob = &b43_fw_ac1initvals42;
		}
		else if (rev == 40)
		{
			filename = "ac0initvals40";
			blob = &b43_fw_ac0initvals40;
		}
		break;
	}
	if (!filename)
		goto err_no_initvals;
	err = b43_do_request_fw(ctx, blob, filename, &fw->initvals);
	if (err)
		goto err_load;

	/* Get bandswitch initvals */
	filename = NULL;
	switch (dev->phy.type) {
	case B43_PHYTYPE_G:
		if (rev == 13)
		{
			filename = "b0g0bsinitvals13";
			blob = &b43_fw_b0g0bsinitvals13;
		}
		else if (rev >= 5 && rev <= 10)
		{
			filename = "b0g0bsinitvals5";
			blob = &b43_fw_b0g0bsinitvals5;
		}
		break;
	case B43_PHYTYPE_N:
		if (rev == 30)
		{
			filename = "n16bsinitvals30";
			blob = &b43_fw_n16bsinitvals30;
		}
		else if (rev == 28 || rev == 25)
		{
			filename = "n0bsinitvals25";
			blob = &b43_fw_n0bsinitvals25;
		}
		else if (rev == 24)
		{
			filename = "n0bsinitvals24";
			blob = &b43_fw_n0bsinitvals24;
		}
		else if (rev == 23)
		{
			filename = "n0bsinitvals16"; /* What about n0bsinitvals22? */
			blob = &b43_fw_n0bsinitvals16;
		}
		else if (rev >= 16 && rev <= 18)
		{
			filename = "n0bsinitvals16";
			blob = &b43_fw_n0bsinitvals16;
		}
		else if (rev >= 11 && rev <= 12)
		{
			filename = "n0bsinitvals11";
			blob = &b43_fw_n0bsinitvals11;
		}
		break;
	case B43_PHYTYPE_LP:
		if (rev >= 16 && rev <= 18)
		{
			filename = "lp0bsinitvals16";
			blob = &b43_fw_lp0bsinitvals16;
		}
		else if (rev == 15)
		{
			filename = "lp0bsinitvals15";
			blob = &b43_fw_lp0bsinitvals15;
		}
		else if (rev == 14)
		{
			filename = "lp0bsinitvals14";
			blob = &b43_fw_lp0bsinitvals14;
		}
		else if (rev == 13)
		{
			filename = "lp0bsinitvals13";
			blob = &b43_fw_lp0bsinitvals13;
		}
		break;
	case B43_PHYTYPE_HT:
		if (rev == 29)
		{
			filename = "ht0bsinitvals29";
			blob = &b43_fw_ht0bsinitvals29;
		}
		else if (rev == 26)
		{
			filename = "ht0bsinitvals26";
			blob = &b43_fw_ht0bsinitvals26;
		}
		break;
	case B43_PHYTYPE_LCN:
		if (rev == 24)
		{
			filename = "lcn0bsinitvals24";
			blob = &b43_fw_lcn0bsinitvals24;
		}
		break;
	case B43_PHYTYPE_LCN40:
		if (rev == 33)
		{
			filename = "lcn400bsinitvals33";
			blob = &b43_fw_lcn400bsinitvals33;
		}
		break;
	case B43_PHYTYPE_AC:
		if (rev == 42)
		{
			filename = "ac1bsinitvals42";
			blob = &b43_fw_ac1bsinitvals42;
		}
		else if (rev == 40)
		{
			filename = "ac0bsinitvals40";
			blob = &b43_fw_ac0bsinitvals40;
		}
		break;
	}
	if (!filename)
		goto err_no_initvals;
	err = b43_do_request_fw(ctx, blob, filename, &fw->initvals_band);
	if (err)
		goto err_load;

	fw->opensource = (ctx->req_type == B43_FWTYPE_OPENSOURCE);

	return 0;

err_no_ucode:
	err = ctx->fatal_failure = -EOPNOTSUPP;
	b43err(dev->wl, "The driver does not know which firmware (ucode) "
	       "is required for your device (wl-core rev %d)\n", rev);
	goto error;

err_no_pcm:
	err = ctx->fatal_failure = -EOPNOTSUPP;
	b43err(dev->wl, "The driver does not know which firmware (PCM) "
	       "is required for your device (wl-core rev %d)\n", rev);
	goto error;

err_no_initvals:
	err = ctx->fatal_failure = -EOPNOTSUPP;
	b43err(dev->wl, "The driver does not know which firmware (initvals) "
	       "is required for your device (wl-core rev %d)\n", rev);
	goto error;

err_load:
	/* We failed to load this firmware image. The error message
	 * already is in ctx->errors. Return and let our caller decide
	 * what to do. */
	goto error;

error:
	DBGC(dev->wl, "Firmware load error :(\n");
	b43_release_firmware(dev);
	return err;
}

static int b43_one_core_attach(struct b43_bus_dev *dev, struct b43_wl *wl);
static void b43_one_core_detach(struct b43_bus_dev *dev);

static struct net80211_device_operations b43_hw_ops;

static void b43_request_firmware(struct b43_wl *wl)
{
	struct b43_wldev *dev = wl->current_dev;
	struct b43_request_fw_context *ctx;
	unsigned int i;
	int err;
	const char *errmsg;

  //DBGC(wl, "b43_request_firmware: ");
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
  {
    DBGC(wl, "\t Could not allocate ctx!\n");
		return;
  }
	ctx->dev = dev;

	ctx->req_type = B43_FWTYPE_PROPRIETARY;
	err = b43_try_request_fw(ctx);
	if (!err)
		goto start_ieee80211; /* Successfully loaded it. */
	/* Was fw version known? */
	if (ctx->fatal_failure)
		goto out;

	/* FIXME IPXE: currently no opensource firmware option support */
	/* proprietary fw not found, try open source */
	ctx->req_type = B43_FWTYPE_OPENSOURCE;
	err = b43_try_request_fw(ctx);
	if (!err)
		goto start_ieee80211; /* Successfully loaded it. */
	if(ctx->fatal_failure)
		goto out;

	/* Could not find a usable firmware. Print the errors. */
	for (i = 0; i < B43_NR_FWTYPES; i++) {
		errmsg = ctx->errors[i];
		if (strlen(errmsg))
			b43err(dev->wl, "%s", errmsg);
	}
	b43_print_fw_helptext(dev->wl, 1);
	goto out;

start_ieee80211:

	err = net80211_register(b43_wl_to_ndev(wl), &b43_hw_ops, wl->hwinfo);
	if (err)
		goto err_one_core_detach;
	DBGC(b43_wl_to_ndev(wl), "Successfully Registered with iPXE!\n");
	wl->hw_registred = true;
	// FIXME ipxe LED support is postponed, be sure to not enable
	// CONFIG_B43_LEDS
	b43_leds_register(wl->current_dev);

	goto out;

err_one_core_detach:
	DBGC(dev, "something was wrong, going to one_core_detach\n");
	b43_one_core_detach(dev->dev);

out:
	kfree(ctx);
}

static int b43_upload_microcode(struct b43_wldev *dev)
{
	const size_t hdr_len = sizeof(struct b43_fw_header);
	const __be32 *data;
	unsigned int i, len;
	u16 fwrev, fwpatch, fwdate, fwtime;
	u32 tmp, macctl;
	int err = 0;

	/* Jump the microcode PSM to offset 0 */
	macctl = b43_read32(dev, B43_MMIO_MACCTL);
	B43_WARN_ON(macctl & B43_MACCTL_PSM_RUN);
	macctl |= B43_MACCTL_PSM_JMP0;
	b43_write32(dev, B43_MMIO_MACCTL, macctl);
	/* Zero out all microcode PSM registers and shared memory. */
	for (i = 0; i < 64; i++)
		b43_shm_write16(dev, B43_SHM_SCRATCH, i, 0);
	for (i = 0; i < 4096; i += 2)
		b43_shm_write16(dev, B43_SHM_SHARED, i, 0);

	/* Upload Microcode. */
	data = (__be32 *) (dev->fw.ucode.data->data + hdr_len);
	len = (dev->fw.ucode.data->size - hdr_len) / sizeof(__be32);
	b43_shm_control_word(dev, B43_SHM_UCODE | B43_SHM_AUTOINC_W, 0x0000);
	for (i = 0; i < len; i++) {
		b43_write32(dev, B43_MMIO_SHM_DATA, be32_to_cpu(data[i]));
		udelay(10);
	}

	if (dev->fw.pcm.data) {
		/* Upload PCM data. */
		data = (__be32 *) (dev->fw.pcm.data->data + hdr_len);
		len = (dev->fw.pcm.data->size - hdr_len) / sizeof(__be32);
		b43_shm_control_word(dev, B43_SHM_HW, 0x01EA);
		b43_write32(dev, B43_MMIO_SHM_DATA, 0x00004000);
		/* No need for autoinc bit in SHM_HW */
		b43_shm_control_word(dev, B43_SHM_HW, 0x01EB);
		for (i = 0; i < len; i++) {
			b43_write32(dev, B43_MMIO_SHM_DATA, be32_to_cpu(data[i]));
			udelay(10);
		}
	}

	b43_write32(dev, B43_MMIO_GEN_IRQ_REASON, B43_IRQ_ALL);

	/* Start the microcode PSM */
	b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_PSM_JMP0,
		      B43_MACCTL_PSM_RUN);

	/* Wait for the microcode to load and respond */
	i = 0;
	while (1) {
		tmp = b43_read32(dev, B43_MMIO_GEN_IRQ_REASON);
		if (tmp == B43_IRQ_MAC_SUSPENDED)
			break;
		i++;
		if (i >= 20) {
			b43err(dev->wl, "Microcode not responding\n");
			b43_print_fw_helptext(dev->wl, 1);
			err = -ENODEV;
			goto error;
		}
		msleep(50);
	}
	b43_read32(dev, B43_MMIO_GEN_IRQ_REASON);	/* dummy read */

	/* Get and check the revisions. */
	fwrev = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_UCODEREV);
	fwpatch = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_UCODEPATCH);
	fwdate = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_UCODEDATE);
	fwtime = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_UCODETIME);

	if (fwrev <= 0x128) {
		b43err(dev->wl, "YOUR FIRMWARE IS TOO OLD. Firmware from "
		       "binary drivers older than version 4.x is unsupported. "
		       "You must upgrade your firmware files.\n");
		b43_print_fw_helptext(dev->wl, 1);
		err = -EOPNOTSUPP;
		goto error;
	}
	dev->fw.rev = fwrev;
	dev->fw.patch = fwpatch;
	if (dev->fw.rev >= 598)
		dev->fw.hdr_format = B43_FW_HDR_598;
	else if (dev->fw.rev >= 410)
		dev->fw.hdr_format = B43_FW_HDR_410;
	else
		dev->fw.hdr_format = B43_FW_HDR_351;
	WARN_ON(dev->fw.opensource != (fwdate == 0xFFFF));

	/* Default to firmware/hardware crypto acceleration. */
	dev->hwcrypto_enabled = true;

	if (dev->fw.opensource) {
		u16 fwcapa;

		/* Patchlevel info is encoded in the "time" field. */
		dev->fw.patch = fwtime;
		b43info(dev->wl, "Loading OpenSource firmware version %d.%d\n",
			dev->fw.rev, dev->fw.patch);

		fwcapa = b43_fwcapa_read(dev);
		if (!(fwcapa & B43_FWCAPA_HWCRYPTO) || dev->fw.pcm_request_failed) {
			b43info(dev->wl, "Hardware crypto acceleration not supported by firmware\n");
			/* Disable hardware crypto and fall back to software crypto. */
			dev->hwcrypto_enabled = false;
		}
	} else {
		b43info(dev->wl, "Loading firmware version %d.%d "
			"(20%02d-%02d-%02d %02d:%02d:%02d)\n",
			fwrev, fwpatch,
			(fwdate >> 12) & 0xF, (fwdate >> 8) & 0xF, fwdate & 0xFF,
			(fwtime >> 11) & 0x1F, (fwtime >> 5) & 0x3F, fwtime & 0x1F);
		if (dev->fw.pcm_request_failed) {
			b43warn(dev->wl, "No \"pcm5.fw\" firmware file found. "
				"Hardware accelerated cryptography is disabled.\n");
			b43_print_fw_helptext(dev->wl, 0);
		}
	}

	if (dev->fw.hdr_format == B43_FW_HDR_351) {
		/* We're over the deadline, but we keep support for old fw
		 * until it turns out to be in major conflict with something new. */
		b43warn(dev->wl, "You are using an old firmware image. "
			"Support for old firmware will be removed soon "
			"(official deadline was July 2008).\n");
		b43_print_fw_helptext(dev->wl, 0);
	}

	return 0;

error:
	/* Stop the microcode PSM. */
	b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_PSM_RUN,
		      B43_MACCTL_PSM_JMP0);

	return err;
}

static int b43_write_initvals(struct b43_wldev *dev,
			      const struct b43_iv *ivals,
			      size_t count,
			      size_t array_size)
{
	const struct b43_iv *iv;
	u16 offset;
	size_t i;
	bool bit32;

	BUILD_BUG_ON(sizeof(struct b43_iv) != 6);
	iv = ivals;
	for (i = 0; i < count; i++) {
		if (array_size < sizeof(iv->offset_size))
			goto err_format;
		array_size -= sizeof(iv->offset_size);
		offset = be16_to_cpu(iv->offset_size);
		bit32 = !!(offset & B43_IV_32BIT);
		offset &= B43_IV_OFFSET_MASK;
		if (offset >= 0x1000)
			goto err_format;
		if (bit32) {
			u32 value;

			if (array_size < sizeof(iv->data.d32))
				goto err_format;
			array_size -= sizeof(iv->data.d32);

			value = get_unaligned_be32((const u8 *)&iv->data.d32);
			b43_write32(dev, offset, value);

			iv = (const struct b43_iv *)((const uint8_t *)iv +
							sizeof(__be16) +
							sizeof(__be32));
		} else {
			u16 value;

			if (array_size < sizeof(iv->data.d16))
				goto err_format;
			array_size -= sizeof(iv->data.d16);

			value = be16_to_cpu(iv->data.d16);
			b43_write16(dev, offset, value);

			iv = (const struct b43_iv *)((const uint8_t *)iv +
							sizeof(__be16) +
							sizeof(__be16));
		}
	}
	if (array_size)
		goto err_format;

	return 0;

err_format:
	b43err(dev->wl, "Initial Values Firmware file-format error.\n");
	b43_print_fw_helptext(dev->wl, 1);

	return -EPROTO;
}

static int b43_upload_initvals(struct b43_wldev *dev)
{
	const size_t hdr_len = sizeof(struct b43_fw_header);
	const struct b43_fw_header *hdr;
	struct b43_firmware *fw = &dev->fw;
	const struct b43_iv *ivals;
	size_t count;

	hdr = (const struct b43_fw_header *)(fw->initvals.data->data);
	ivals = (const struct b43_iv *)(fw->initvals.data->data + hdr_len);
	count = be32_to_cpu(hdr->size);
	return b43_write_initvals(dev, ivals, count,
				 fw->initvals.data->size - hdr_len);
}

static int b43_upload_initvals_band(struct b43_wldev *dev)
{
	const size_t hdr_len = sizeof(struct b43_fw_header);
	const struct b43_fw_header *hdr;
	struct b43_firmware *fw = &dev->fw;
	const struct b43_iv *ivals;
	size_t count;

	if (!fw->initvals_band.data)
		return 0;

	hdr = (const struct b43_fw_header *)(fw->initvals_band.data->data);
	ivals = (const struct b43_iv *)(fw->initvals_band.data->data + hdr_len);
	count = be32_to_cpu(hdr->size);
	return b43_write_initvals(dev, ivals, count,
				  fw->initvals_band.data->size - hdr_len);
}

/* Initialize the GPIOs
 * http://bcm-specs.sipsolutions.net/GPIO
 */

#ifdef CONFIG_B43_SSB
static struct ssb_device *b43_ssb_gpio_dev(struct b43_wldev *dev)
{
	struct ssb_bus *bus = dev->dev->sdev->bus;

#ifdef CONFIG_SSB_DRIVER_PCICORE
	return (bus->chipco.dev ? bus->chipco.dev : bus->pcicore.dev);
#else
	return bus->chipco.dev;
#endif
}
#endif

static int b43_gpio_init(struct b43_wldev *dev)
{
#ifdef CONFIG_B43_SSB
	struct ssb_device *gpiodev;
#endif
	u32 mask, set;

	b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_GPOUTSMSK, 0);
	b43_maskset16(dev, B43_MMIO_GPIO_MASK, ~0, 0xF);

	mask = 0x0000001F;
	set = 0x0000000F;
	if (dev->dev->chip_id == 0x4301) {
		mask |= 0x0060;
		set |= 0x0060;
	} else if (dev->dev->chip_id == 0x5354) {
		/* Don't allow overtaking buttons GPIOs */
		set &= 0x2; /* 0x2 is LED GPIO on BCM5354 */
	}

	if (0 /* FIXME: conditional unknown */ ) {
		b43_write16(dev, B43_MMIO_GPIO_MASK,
			    b43_read16(dev, B43_MMIO_GPIO_MASK)
			    | 0x0100);
		/* BT Coexistance Input */
		mask |= 0x0080;
		set |= 0x0080;
		/* BT Coexistance Out */
		mask |= 0x0100;
		set |= 0x0100;
	}
	if (dev->dev->bus_sprom->boardflags_lo & B43_BFL_PACTRL) {
		/* PA is controlled by gpio 9, let ucode handle it */
		b43_write16(dev, B43_MMIO_GPIO_MASK,
			    b43_read16(dev, B43_MMIO_GPIO_MASK)
			    | 0x0200);
		mask |= 0x0200;
		set |= 0x0200;
	}

	switch (dev->dev->bus_type) {
#ifdef CONFIG_B43_BCMA
	case B43_BUS_BCMA:
		bcma_chipco_gpio_control(&dev->dev->bdev->bus->drv_cc, mask, set);
		break;
#endif
#ifdef CONFIG_B43_SSB
	case B43_BUS_SSB:
		gpiodev = b43_ssb_gpio_dev(dev);
		if (gpiodev)
			ssb_write32(gpiodev, B43_GPIO_CONTROL,
				    (ssb_read32(gpiodev, B43_GPIO_CONTROL)
				    & ~mask) | set);
		break;
#endif
	}

	return 0;
}

/* Turn off all GPIO stuff. Call this on module unload, for example. */
static void b43_gpio_cleanup(struct b43_wldev *dev)
{
#ifdef CONFIG_B43_SSB
	struct ssb_device *gpiodev;
#endif

	switch (dev->dev->bus_type) {
#ifdef CONFIG_B43_BCMA
	case B43_BUS_BCMA:
		bcma_chipco_gpio_control(&dev->dev->bdev->bus->drv_cc, ~0, 0);
		break;
#endif
#ifdef CONFIG_B43_SSB
	case B43_BUS_SSB:
		gpiodev = b43_ssb_gpio_dev(dev);
		if (gpiodev)
			ssb_write32(gpiodev, B43_GPIO_CONTROL, 0);
		break;
#endif
	}
}

/* http://bcm-specs.sipsolutions.net/EnableMac */
void b43_mac_enable(struct b43_wldev *dev)
{
	if (b43_debug(dev, B43_DBG_FIRMWARE)) {
		u16 fwstate;

		fwstate = b43_shm_read16(dev, B43_SHM_SHARED,
					 B43_SHM_SH_UCODESTAT);
		if ((fwstate != B43_SHM_SH_UCODESTAT_SUSP) &&
		    (fwstate != B43_SHM_SH_UCODESTAT_SLEEP)) {
			b43err(dev->wl, "b43_mac_enable(): The firmware "
			       "should be suspended, but current state is %d\n",
			       fwstate);
		}
	}

	dev->mac_suspended--;
	B43_WARN_ON(dev->mac_suspended < 0);
	if (dev->mac_suspended == 0) {
		b43_maskset32(dev, B43_MMIO_MACCTL, ~0, B43_MACCTL_ENABLED);
		b43_write32(dev, B43_MMIO_GEN_IRQ_REASON,
			    B43_IRQ_MAC_SUSPENDED);
		/* Commit writes */
		b43_read32(dev, B43_MMIO_MACCTL);
		b43_read32(dev, B43_MMIO_GEN_IRQ_REASON);
		b43_power_saving_ctl_bits(dev, 0);
	}
}

/* http://bcm-specs.sipsolutions.net/SuspendMAC */
void b43_mac_suspend(struct b43_wldev *dev)
{
	int i;
	u32 tmp;

	B43_WARN_ON(dev->mac_suspended < 0);

	if (dev->mac_suspended == 0) {
		b43_power_saving_ctl_bits(dev, B43_PS_AWAKE);
		b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_ENABLED, 0);
		/* force pci to flush the write */
		b43_read32(dev, B43_MMIO_MACCTL);
		for (i = 35; i; i--) {
			tmp = b43_read32(dev, B43_MMIO_GEN_IRQ_REASON);
			if (tmp & B43_IRQ_MAC_SUSPENDED)
				goto out;
			udelay(10);
		}
		/* Hm, it seems this will take some time. Use msleep(). */
		for (i = 40; i; i--) {
			tmp = b43_read32(dev, B43_MMIO_GEN_IRQ_REASON);
			if (tmp & B43_IRQ_MAC_SUSPENDED)
				goto out;
			msleep(1);
		}
		b43err(dev->wl, "MAC suspend failed\n");
	}
out:
	dev->mac_suspended++;
}

/* http://bcm-v4.sipsolutions.net/802.11/PHY/N/MacPhyClkSet */
void b43_mac_phy_clock_set(struct b43_wldev *dev, bool on)
{
	u32 tmp;

	switch (dev->dev->bus_type) {
#ifdef CONFIG_B43_BCMA
	case B43_BUS_BCMA:
		tmp = bcma_aread32(dev->dev->bdev, BCMA_IOCTL);
		if (on)
			tmp |= B43_BCMA_IOCTL_MACPHYCLKEN;
		else
			tmp &= ~B43_BCMA_IOCTL_MACPHYCLKEN;
		bcma_awrite32(dev->dev->bdev, BCMA_IOCTL, tmp);
		break;
#endif
#ifdef CONFIG_B43_SSB
	case B43_BUS_SSB:
		tmp = ssb_read32(dev->dev->sdev, SSB_TMSLOW);
		if (on)
			tmp |= B43_TMSLOW_MACPHYCLKEN;
		else
			tmp &= ~B43_TMSLOW_MACPHYCLKEN;
		ssb_write32(dev->dev->sdev, SSB_TMSLOW, tmp);
		break;
#endif
	}
}

/* brcms_b_switch_macfreq */
void b43_mac_switch_freq(struct b43_wldev *dev, u8 spurmode)
{
	u16 chip_id = dev->dev->chip_id;

	if (chip_id == BCMA_CHIP_ID_BCM4331) {
		switch (spurmode) {
		case 2: /* 168 Mhz: 2^26/168 = 0x61862 */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0x1862);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0x6);
			break;
		case 1: /* 164 Mhz: 2^26/164 = 0x63e70 */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0x3e70);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0x6);
			break;
		default: /* 160 Mhz: 2^26/160 = 0x66666 */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0x6666);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0x6);
			break;
		}
	} else if (chip_id == BCMA_CHIP_ID_BCM43131 ||
	    chip_id == BCMA_CHIP_ID_BCM43217 ||
	    chip_id == BCMA_CHIP_ID_BCM43222 ||
	    chip_id == BCMA_CHIP_ID_BCM43224 ||
	    chip_id == BCMA_CHIP_ID_BCM43225 ||
	    chip_id == BCMA_CHIP_ID_BCM43227 ||
	    chip_id == BCMA_CHIP_ID_BCM43228) {
		switch (spurmode) {
		case 2: /* 126 Mhz */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0x2082);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0x8);
			break;
		case 1: /* 123 Mhz */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0x5341);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0x8);
			break;
		default: /* 120 Mhz */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0x8889);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0x8);
			break;
		}
	} else if (dev->phy.type == B43_PHYTYPE_LCN) {
		switch (spurmode) {
		case 1: /* 82 Mhz */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0x7CE0);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0xC);
			break;
		default: /* 80 Mhz */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0xCCCD);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0xC);
			break;
		}
	}
}

static void b43_adjust_opmode(struct b43_wldev *dev)
{
	struct b43_wl *wl = dev->wl;
	u32 ctl;
	u16 cfp_pretbtt;

	ctl = b43_read32(dev, B43_MMIO_MACCTL);
	/* Reset status to STA infrastructure mode. */
	ctl &= ~B43_MACCTL_AP;
	ctl &= ~B43_MACCTL_KEEP_CTL;
	ctl &= ~B43_MACCTL_KEEP_BADPLCP;
	ctl &= ~B43_MACCTL_KEEP_BAD;
	ctl &= ~B43_MACCTL_PROMISC;
	ctl &= ~B43_MACCTL_BEACPROMISC;
	ctl |= B43_MACCTL_INFRA;

	if (wl->filter_flags & FIF_CONTROL)
		ctl |= B43_MACCTL_KEEP_CTL;
	if (wl->filter_flags & FIF_FCSFAIL)
		ctl |= B43_MACCTL_KEEP_BAD;
	if (wl->filter_flags & FIF_PLCPFAIL)
		ctl |= B43_MACCTL_KEEP_BADPLCP;
	if (wl->filter_flags & FIF_PROMISC_IN_BSS)
		ctl |= B43_MACCTL_PROMISC;
	if (wl->filter_flags & FIF_BCN_PRBRESP_PROMISC)
		ctl |= B43_MACCTL_BEACPROMISC;

	/* Workaround: On old hardware the HW-MAC-address-filter
	 * doesn't work properly, so always run promisc in filter
	 * it in software. */
	if (dev->dev->core_rev <= 4)
		ctl |= B43_MACCTL_PROMISC;

	b43_write32(dev, B43_MMIO_MACCTL, ctl);

	cfp_pretbtt = 2;
	if ((ctl & B43_MACCTL_INFRA) && !(ctl & B43_MACCTL_AP)) {
		if (dev->dev->chip_id == 0x4306 &&
		    dev->dev->chip_rev == 3)
			cfp_pretbtt = 100;
		else
			cfp_pretbtt = 50;
	}
	b43_write16(dev, 0x612, cfp_pretbtt);

	/* FIXME: We don't currently implement the PMQ mechanism,
	 *        so always disable it. If we want to implement PMQ,
	 *        we need to enable it here (clear DISCPMQ) in AP mode.
	 */
	if (0  /* ctl & B43_MACCTL_AP */)
		b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_DISCPMQ, 0);
	else
		b43_maskset32(dev, B43_MMIO_MACCTL, ~0, B43_MACCTL_DISCPMQ);
}

static void b43_rate_memory_write(struct b43_wldev *dev, u16 rate, int is_ofdm)
{
	u16 offset;

	if (is_ofdm) {
		offset = 0x480;
		offset += (b43_plcp_get_ratecode_ofdm(rate) & 0x000F) * 2;
	} else {
		offset = 0x4C0;
		offset += (b43_plcp_get_ratecode_cck(rate) & 0x000F) * 2;
	}
	b43_shm_write16(dev, B43_SHM_SHARED, offset + 0x20,
			b43_shm_read16(dev, B43_SHM_SHARED, offset));
}

static void b43_rate_memory_init(struct b43_wldev *dev)
{
	switch (dev->phy.type) {
	case B43_PHYTYPE_A:
	case B43_PHYTYPE_G:
	case B43_PHYTYPE_N:
	case B43_PHYTYPE_LP:
	case B43_PHYTYPE_HT:
	case B43_PHYTYPE_LCN:
		b43_rate_memory_write(dev, B43_OFDM_RATE_6MB, 1);
		b43_rate_memory_write(dev, B43_OFDM_RATE_9MB, 1);
		b43_rate_memory_write(dev, B43_OFDM_RATE_12MB, 1);
		b43_rate_memory_write(dev, B43_OFDM_RATE_18MB, 1);
		b43_rate_memory_write(dev, B43_OFDM_RATE_24MB, 1);
		b43_rate_memory_write(dev, B43_OFDM_RATE_36MB, 1);
		b43_rate_memory_write(dev, B43_OFDM_RATE_48MB, 1);
		b43_rate_memory_write(dev, B43_OFDM_RATE_54MB, 1);
		if (dev->phy.type == B43_PHYTYPE_A)
			break;
		/* fallthrough */
	case B43_PHYTYPE_B:
		b43_rate_memory_write(dev, B43_CCK_RATE_1MB, 0);
		b43_rate_memory_write(dev, B43_CCK_RATE_2MB, 0);
		b43_rate_memory_write(dev, B43_CCK_RATE_5MB, 0);
		b43_rate_memory_write(dev, B43_CCK_RATE_11MB, 0);
		break;
	default:
		B43_WARN_ON(1);
	}
}

/* Set the default values for the PHY TX Control Words. */
static void b43_set_phytxctl_defaults(struct b43_wldev *dev)
{
	u16 ctl = 0;

	ctl |= B43_TXH_PHY_ENC_CCK;
	ctl |= B43_TXH_PHY_ANT01AUTO;
	ctl |= B43_TXH_PHY_TXPWR;

	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_BEACPHYCTL, ctl);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_ACKCTSPHYCTL, ctl);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_PRPHYCTL, ctl);
}

/* Set the TX-Antenna for management frames sent by firmware. */
static void b43_mgmtframe_txantenna(struct b43_wldev *dev, int antenna)
{
	u16 ant;
	u16 tmp;

	ant = b43_antenna_to_phyctl(antenna);

	/* For ACK/CTS */
	tmp = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_ACKCTSPHYCTL);
	tmp = (tmp & ~B43_TXH_PHY_ANT) | ant;
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_ACKCTSPHYCTL, tmp);
	/* For Probe Resposes */
	tmp = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_PRPHYCTL);
	tmp = (tmp & ~B43_TXH_PHY_ANT) | ant;
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_PRPHYCTL, tmp);
}

/* This is the opposite of b43_chip_init() */
static void b43_chip_exit(struct b43_wldev *dev)
{
	b43_phy_exit(dev);
	b43_gpio_cleanup(dev);
	/* firmware is released later */
}

/* Initialize the chip
 * http://bcm-specs.sipsolutions.net/ChipInit
 */
static int b43_chip_init(struct b43_wldev *dev)
{
	struct b43_phy *phy = &dev->phy;
	int err;
	u32 macctl;
	u16 value16;

	/* Initialize the MAC control */
	macctl = B43_MACCTL_IHR_ENABLED | B43_MACCTL_SHM_ENABLED;
	if (dev->phy.gmode)
		macctl |= B43_MACCTL_GMODE;
	macctl |= B43_MACCTL_INFRA;
	b43_write32(dev, B43_MMIO_MACCTL, macctl);

	err = b43_upload_microcode(dev);
	if (err)
		goto out;	/* firmware is released later */

	err = b43_gpio_init(dev);
	if (err)
		goto out;	/* firmware is released later */

	err = b43_upload_initvals(dev);
	if (err)
		goto err_gpio_clean;

	err = b43_upload_initvals_band(dev);
	if (err)
		goto err_gpio_clean;

	/* Turn the Analog on and initialize the PHY. */
	phy->ops->switch_analog(dev, 1);
	err = b43_phy_init(dev);
	if (err)
		goto err_gpio_clean;

	/* Disable Interference Mitigation. */
	if (phy->ops->interf_mitigation)
		phy->ops->interf_mitigation(dev, B43_INTERFMODE_NONE);

	/* Select the antennae */
	if (phy->ops->set_rx_antenna)
		phy->ops->set_rx_antenna(dev, B43_ANTENNA_DEFAULT);
	b43_mgmtframe_txantenna(dev, B43_ANTENNA_DEFAULT);

	if (phy->type == B43_PHYTYPE_B) {
		value16 = b43_read16(dev, 0x005E);
		value16 |= 0x0004;
		b43_write16(dev, 0x005E, value16);
	}
	b43_write32(dev, 0x0100, 0x01000000);
	if (dev->dev->core_rev < 5)
		b43_write32(dev, 0x010C, 0x01000000);

	b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_INFRA, 0);
	b43_maskset32(dev, B43_MMIO_MACCTL, ~0, B43_MACCTL_INFRA);

	/* Probe Response Timeout value */
	/* FIXME: Default to 0, has to be set by ioctl probably... :-/ */
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_PRMAXTIME, 0);

	/* Initially set the wireless operation mode. */
	b43_adjust_opmode(dev);

	if (dev->dev->core_rev < 3) {
		b43_write16(dev, 0x060E, 0x0000);
		b43_write16(dev, 0x0610, 0x8000);
		b43_write16(dev, 0x0604, 0x0000);
		b43_write16(dev, 0x0606, 0x0200);
	} else {
		b43_write32(dev, 0x0188, 0x80000000);
		b43_write32(dev, 0x018C, 0x02000000);
	}
	b43_write32(dev, B43_MMIO_GEN_IRQ_REASON, 0x00004000);
	b43_write32(dev, B43_MMIO_DMA0_IRQ_MASK, 0x0001FC00);
	b43_write32(dev, B43_MMIO_DMA1_IRQ_MASK, 0x0000DC00);
	b43_write32(dev, B43_MMIO_DMA2_IRQ_MASK, 0x0000DC00);
	b43_write32(dev, B43_MMIO_DMA3_IRQ_MASK, 0x0001DC00);
	b43_write32(dev, B43_MMIO_DMA4_IRQ_MASK, 0x0000DC00);
	b43_write32(dev, B43_MMIO_DMA5_IRQ_MASK, 0x0000DC00);

	b43_mac_phy_clock_set(dev, true);

	switch (dev->dev->bus_type) {
#ifdef CONFIG_B43_BCMA
	case B43_BUS_BCMA:
		/* FIXME: 0xE74 is quite common, but should be read from CC */
		b43_write16(dev, B43_MMIO_POWERUP_DELAY, 0xE74);
		break;
#endif
#ifdef CONFIG_B43_SSB
	case B43_BUS_SSB:
		b43_write16(dev, B43_MMIO_POWERUP_DELAY,
			    dev->dev->sdev->bus->chipco.fast_pwrup_delay);
		break;
#endif
	}

	err = 0;
	b43dbg(dev->wl, "Chip initialized\n");
out:
	return err;

err_gpio_clean:
	b43_gpio_cleanup(dev);
	return err;
}

static void b43_periodic_every60sec(struct b43_wldev *dev)
{
	const struct b43_phy_operations *ops = dev->phy.ops;
	//DBGC(dev, "b43_periodic_every60sec\n");

	if (ops->pwork_60sec)
		ops->pwork_60sec(dev);

	/* Force check the TX power emission now. */
	b43_phy_txpower_check(dev, B43_TXPWR_IGNORE_TIME);
}

static void b43_periodic_every30sec(struct b43_wldev *dev)
{
	//DBGC(dev, "b43_periodic_every30sec\n");
	/* Update device statistics. */
	b43_calculate_link_quality(dev);
}

static void b43_periodic_every15sec(struct b43_wldev *dev)
{
	struct b43_phy *phy = &dev->phy;
	u16 wdr;
	//DBGC(dev, "b43_periodic_every15sec\n");

	if (dev->fw.opensource) {
		/* Check if the firmware is still alive.
		 * It will reset the watchdog counter to 0 in its idle loop. */
		wdr = b43_shm_read16(dev, B43_SHM_SCRATCH, B43_WATCHDOG_REG);
		if (unlikely(wdr)) {
			b43err(dev->wl, "Firmware watchdog: The firmware died!\n");
			b43_controller_restart(dev, "Firmware watchdog");
			return;
		} else {
			b43_shm_write16(dev, B43_SHM_SCRATCH,
					B43_WATCHDOG_REG, 1);
		}
	}

	if (phy->ops->pwork_15sec)
		phy->ops->pwork_15sec(dev);

	atomic_set(&phy->txerr_cnt, B43_PHY_TX_BADNESS_LIMIT);
	wmb();

#if B43_DEBUG
	if (b43_debug(dev, B43_DBG_VERBOSESTATS)) {
		unsigned int i;

		b43dbg(dev->wl, "Stats: %7d IRQs/sec, %7d TX/sec, %7d RX/sec\n",
		       dev->irq_count / 15,
		       dev->tx_count / 15,
		       dev->rx_count / 15);
		dev->irq_count = 0;
		dev->tx_count = 0;
		dev->rx_count = 0;
		for (i = 0; i < ARRAY_SIZE(dev->irq_bit_count); i++) {
			if (dev->irq_bit_count[i]) {
				b43dbg(dev->wl, "Stats: %7d IRQ-%02d/sec (0x%08X)\n",
				       dev->irq_bit_count[i] / 15, i, (1 << i));
				dev->irq_bit_count[i] = 0;
			}
		}
	}
#endif
}

static void do_periodic_work(struct b43_wldev *dev)
{
	unsigned int state;

	state = dev->periodic_state;
	if (state % 4 == 0)
		b43_periodic_every60sec(dev);
	if (state % 2 == 0)
		b43_periodic_every30sec(dev);
	b43_periodic_every15sec(dev);
}

static void b43_periodic_work_handler(struct b43_wldev *dev)
{
	unsigned long delay;

	if (unlikely(b43_status(dev) != B43_STAT_STARTED))
		goto out;
	if (b43_debug(dev, B43_DBG_PWORK_STOP))
		goto out_requeue;

	do_periodic_work(dev);

	dev->periodic_state++;
out_requeue:
	if (b43_debug(dev, B43_DBG_PWORK_FAST))
		delay = msecs_to_jiffies(50);
	else
		delay = round_jiffies_relative(HZ * 15);
	dev->periodic_work = currticks() + delay;
out:
	return;
}

static void b43_periodic_tasks_setup(struct b43_wldev *dev)
{
	dev->periodic_state = 0;
	b43_periodic_work_handler(dev);
}

/* Check if communication with the device works correctly. */
static int b43_validate_chipaccess(struct b43_wldev *dev)
{
	u32 v, backup0, backup4;

	backup0 = b43_shm_read32(dev, B43_SHM_SHARED, 0);
	backup4 = b43_shm_read32(dev, B43_SHM_SHARED, 4);

	/* Check for read/write and endianness problems. */
	b43_shm_write32(dev, B43_SHM_SHARED, 0, 0x55AAAA55);
	if (b43_shm_read32(dev, B43_SHM_SHARED, 0) != 0x55AAAA55)
		goto error;
	b43_shm_write32(dev, B43_SHM_SHARED, 0, 0xAA5555AA);
	if (b43_shm_read32(dev, B43_SHM_SHARED, 0) != 0xAA5555AA)
		goto error;

	/* Check if unaligned 32bit SHM_SHARED access works properly.
	 * However, don't bail out on failure, because it's noncritical. */
	b43_shm_write16(dev, B43_SHM_SHARED, 0, 0x1122);
	b43_shm_write16(dev, B43_SHM_SHARED, 2, 0x3344);
	b43_shm_write16(dev, B43_SHM_SHARED, 4, 0x5566);
	b43_shm_write16(dev, B43_SHM_SHARED, 6, 0x7788);
	if (b43_shm_read32(dev, B43_SHM_SHARED, 2) != 0x55663344)
		b43warn(dev->wl, "Unaligned 32bit SHM read access is broken\n");
	b43_shm_write32(dev, B43_SHM_SHARED, 2, 0xAABBCCDD);
	if (b43_shm_read16(dev, B43_SHM_SHARED, 0) != 0x1122 ||
	    b43_shm_read16(dev, B43_SHM_SHARED, 2) != 0xCCDD ||
	    b43_shm_read16(dev, B43_SHM_SHARED, 4) != 0xAABB ||
	    b43_shm_read16(dev, B43_SHM_SHARED, 6) != 0x7788)
		b43warn(dev->wl, "Unaligned 32bit SHM write access is broken\n");

	b43_shm_write32(dev, B43_SHM_SHARED, 0, backup0);
	b43_shm_write32(dev, B43_SHM_SHARED, 4, backup4);

	if ((dev->dev->core_rev >= 3) && (dev->dev->core_rev <= 10)) {
		/* The 32bit register shadows the two 16bit registers
		 * with update sideeffects. Validate this. */
		b43_write16(dev, B43_MMIO_TSF_CFP_START, 0xAAAA);
		b43_write32(dev, B43_MMIO_TSF_CFP_START, 0xCCCCBBBB);
		if (b43_read16(dev, B43_MMIO_TSF_CFP_START_LOW) != 0xBBBB)
			goto error;
		if (b43_read16(dev, B43_MMIO_TSF_CFP_START_HIGH) != 0xCCCC)
			goto error;
	}
	b43_write32(dev, B43_MMIO_TSF_CFP_START, 0);

	v = b43_read32(dev, B43_MMIO_MACCTL);
	v |= B43_MACCTL_GMODE;
	if (v != (B43_MACCTL_GMODE | B43_MACCTL_IHR_ENABLED))
		goto error;

	return 0;
error:
	b43err(dev->wl, "Failed to validate the chipaccess\n");
	return -ENODEV;
}

static void b43_security_init(struct b43_wldev *dev)
{
	dev->ktp = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_KTP);
	/* KTP is a word address, but we address SHM bytewise.
	 * So multiply by two.
	 */
	dev->ktp *= 2;
	/* Number of RCMTA address slots */
	b43_write16(dev, B43_MMIO_RCMTA_COUNT, B43_NR_PAIRWISE_KEYS);
	/* Clear the key memory. */
	b43_clear_keys(dev);
}

/* I made this function return int to cause freeing of the buffer to the
 * ipxe transmit caller, but then realized that I may need to free several
 * buffers from this function, it became not necessary to use the return
 * value anymore. So I just return 0 instead of reverting the code */
static int b43_tx_work(struct b43_wl *wl, struct io_buffer *iob)
{
	struct b43_wldev *dev;
	int queue_num;
	int err = 0;

	//DBGC(wl, "b43_tx_work: ");

	dev = wl->current_dev;
	if (unlikely(!dev || b43_status(dev) < B43_STAT_STARTED)) {
		return -EAGAIN;
	}

/* translation: for every queue (only 1 in ipxe 'cause no QoS),
 * for every buffer in the queue,
 * transfer it by pio or dma
 * if the transfer ucked up, stopped the queue, and put the buffer back in
 * the queue head.
 * For some other error, free the buffer (i suppose we could try this in
 * ipxe)*/
	for (queue_num = 0; queue_num < B43_QOS_QUEUE_NUM; queue_num++) {
		if (b43_using_pio_transfers(dev))
			err = b43_pio_tx(dev, iob);
		else
			err = b43_dma_tx(dev, iob);
		if (err == -ENOSPC) {
			//printf("TX QUEUE STOPPED; ");
			wl->tx_queue_stopped[queue_num] = 1;
			err = -ENOBUFS;
			break;
		}

		if (!err)
			wl->tx_queue_stopped[queue_num] = 0;
	}

#if B43_DEBUG
	dev->tx_count++;
#endif

	return err;
}

static int b43_op_tx(struct net80211_device *ndev,
		      struct io_buffer *iob)
{
  int ret = 0;
	struct b43_wl *wl = ndev_to_b43_wl(ndev);

	if (unlikely(iob_len(iob) < 2 + 2 + 6)) {
		/* Too short, this can't be a valid frame. */
		// iPXE will free the buffer
		ret = -EINVAL;
    goto tx_exit;
	}

	if (!wl->tx_queue_stopped[0]) {
		ret = b43_tx_work(wl, iob);
    if (ret) {
      //b43dbg(wl, "TX failed\n");
    }
	} else {
		ret = -ENOBUFS;
	}

tx_exit:
	//DBGC("tx ret = %d\n", ret);
  return ret;
}

/* Initialize the core's QOS capabilities */
static void b43_qos_init(struct b43_wldev *dev)
{
		/* Disable QOS support. */
		b43_hf_write(dev, b43_hf_read(dev) & ~B43_HF_EDCF);
		b43_write16(dev, B43_MMIO_IFSCTL,
			    b43_read16(dev, B43_MMIO_IFSCTL)
			    & ~B43_MMIO_IFSCTL_USE_EDCF);
		b43dbg(dev->wl, "QoS disabled\n");
}

static const char *band_to_string(enum ieee80211_band band)
{
	switch (band) {
	case IEEE80211_BAND_5GHZ:
		return "5";
	case IEEE80211_BAND_2GHZ:
		return "2.4";
	default:
		break;
	}
	B43_WARN_ON(1);
	return "";
}

static int b43_switch_band(struct b43_wldev *dev,
			   struct net80211_channel *chan)
{
	struct b43_phy *phy = &dev->phy;
	bool gmode;
	u32 tmp;

	switch (chan->band) {
	case IEEE80211_BAND_5GHZ:
		gmode = false;
		break;
	case IEEE80211_BAND_2GHZ:
		gmode = true;
		break;
	default:
		B43_WARN_ON(1);
		return -EINVAL;
	}

	if (!((gmode && phy->supports_2ghz) ||
	      (!gmode && phy->supports_5ghz))) {
		b43err(dev->wl, "This device doesn't support %s-GHz band\n",
		       band_to_string(chan->band));
		return -ENODEV;
	}

	if (!!phy->gmode == !!gmode) {
		/* This device is already running. */
		return 0;
	}

	b43dbg(dev->wl, "Switching to %s GHz band\n",
	       band_to_string(chan->band));

	/* Some new devices don't need disabling radio for band switching */
	if (!(phy->type == B43_PHYTYPE_N && phy->rev >= 3))
		b43_software_rfkill(dev, true);

	phy->gmode = gmode;
	b43_phy_put_into_reset(dev);
	switch (dev->dev->bus_type) {
#ifdef CONFIG_B43_BCMA
	case B43_BUS_BCMA:
		tmp = bcma_aread32(dev->dev->bdev, BCMA_IOCTL);
		if (gmode)
			tmp |= B43_BCMA_IOCTL_GMODE;
		else
			tmp &= ~B43_BCMA_IOCTL_GMODE;
		bcma_awrite32(dev->dev->bdev, BCMA_IOCTL, tmp);
		break;
#endif
#ifdef CONFIG_B43_SSB
	case B43_BUS_SSB:
		tmp = ssb_read32(dev->dev->sdev, SSB_TMSLOW);
		if (gmode)
			tmp |= B43_TMSLOW_GMODE;
		else
			tmp &= ~B43_TMSLOW_GMODE;
		ssb_write32(dev->dev->sdev, SSB_TMSLOW, tmp);
		break;
#endif
	}
	b43_phy_take_out_of_reset(dev);

	b43_upload_initvals_band(dev);

	b43_phy_init(dev);

	return 0;
}

static void b43_set_beacon_listen_interval(struct b43_wldev *dev, u16 interval)
{
	interval = min_t(u16, interval, (u16)0xFF);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_BCN_LI, interval);
}

/* Write the short and long frame retry limit values. */
static void b43_set_retry_limits(struct b43_wldev *dev,
				 unsigned int short_retry,
				 unsigned int long_retry)
{
	/* The retry limit is a 4-bit counter. Enforce this to avoid overflowing
	 * the chip-internal counter. */
	short_retry = min(short_retry, (unsigned int)0xF);
	long_retry = min(long_retry, (unsigned int)0xF);

	b43_shm_write16(dev, B43_SHM_SCRATCH, B43_SHM_SC_SRLIMIT,
			short_retry);
	b43_shm_write16(dev, B43_SHM_SCRATCH, B43_SHM_SC_LRLIMIT,
			long_retry);
}

static int b43_op_config(struct net80211_device *ndev, int changed)
{
	struct b43_wl *wl = ndev_to_b43_wl(ndev);
	struct b43_wldev *dev = wl->current_dev;
	struct b43_phy *phy = &dev->phy;
	struct net80211_channel *curchan = ndev_to_curchan(ndev);
	int antenna;
	int err = 0;

	//DBGC(ndev, "b43_op_config, changed=%d; ", changed);

	b43_op_bss_info_changed(ndev, changed);

	if (changed & NET80211_CFG_CHANNEL) {
		//DBGC(ndev, "Set channel %d = %d MHz\n", ndev->channel + 1, curchan->center_freq);

		phy->chandef = curchan;
		phy->channel = curchan->hw_value;

		/* Switch the band (if necessary). */
		err = b43_switch_band(dev, curchan);
		if (err)
			goto out_mac_enable;

		/* Switch to the requested channel.
		 * The firmware takes care of races with the TX handler.
		 */
		b43_switch_channel(dev, phy->channel);
	}

	if (!changed)
		goto out_mac_enable;

		/* Adjust the desired TX power level. */
	if (curchan->maxpower != phy->desired_txpower) {
		phy->desired_txpower = curchan->maxpower;
		b43_phy_txpower_check(dev, B43_TXPWR_IGNORE_TIME |
		    B43_TXPWR_IGNORE_TSSI);
	}

	/* Antennas for RX and management frame TX. */
	antenna = B43_ANTENNA_DEFAULT;
	b43_mgmtframe_txantenna(dev, antenna);
	antenna = B43_ANTENNA_DEFAULT;
	if (phy->ops->set_rx_antenna)
		phy->ops->set_rx_antenna(dev, antenna);

	if (wl->radio_enabled != phy->radio_on) {
		if (wl->radio_enabled) {
			b43_software_rfkill(dev, false);
			b43info(dev->wl, "Radio turned on by software\n");
			if (!dev->radio_hw_enable) {
				b43info(dev->wl, "The hardware RF-kill button "
					"still turns the radio physically off. "
					"Press the button to turn it on.\n");
			}
		} else {
			b43_software_rfkill(dev, true);
			b43info(dev->wl, "Radio turned off by software\n");
		}
	}

out_mac_enable:
	b43_mac_enable(dev);

	//DBGC(ndev, "err = %d; ",err);
	return err;
}

static void b43_update_basic_rates(struct b43_wldev *dev, u32 brates)
{
	int curband = b43_current_band(dev->wl);
	//struct net80211_device *ndev = b43_wldev_to_ndev(dev);
	struct net80211_hw_info *hwinfo = dev->wl->hwinfo;
	int i, j;
	u16 basic, direct, offset, basic_offset, rateptr;
	const struct b43_rate *ratetable = NULL;

	//printf("update_basic_rates: curate = %d, slowrate = %d, brate = 0x%04X\n", ndev->rates[ndev->rate], ndev->rates[ndev->rtscts_rate], brates);
	if (brates == 0 || brates == dev->brates) return;
	dev->brates = brates;

	if (curband == NET80211_BAND_2GHZ)
		ratetable = b43_g_ratetable;
	else if (curband == NET80211_BAND_5GHZ)
		ratetable = b43_a_ratetable;
	else
		B43_WARN_ON(1);

	for (i = 0; i < hwinfo->nr_rates[curband]; i++) {
		u16 bitrate = hwinfo->rates[curband][i];
		u16 hw_value = ratetable[i].hw_value;

		if (b43_is_cck_rate(hw_value)) {
			direct = B43_SHM_SH_CCKDIRECT;
			basic = B43_SHM_SH_CCKBASIC;
			offset = b43_plcp_get_ratecode_cck(hw_value);
			offset &= 0xF;
		} else {
			direct = B43_SHM_SH_OFDMDIRECT;
			basic = B43_SHM_SH_OFDMBASIC;
			offset = b43_plcp_get_ratecode_ofdm(hw_value);
			offset &= 0xF;
		}

		/* this for loop is a Port of ieee80211_get_response_rate, with the
		 * bitrates and hw_values kept separate cause that's how it went down
		 * naturally (no reason). This is the only place this function is
		 * called, so I didn't make it a function. Otherwise, port this to a
		 * function.
		 */
		for (j = 0; j < hwinfo->nr_rates[curband]; j++) {
			if (!(brates & BIT(j)))
				continue;
			if (hwinfo->rates[curband][j] > bitrate)
				continue;
			hw_value = ratetable[j].hw_value;
		}

		if (b43_is_cck_rate(hw_value)) {
			basic_offset = b43_plcp_get_ratecode_cck(hw_value);
			basic_offset &= 0xF;
		} else {
			basic_offset = b43_plcp_get_ratecode_ofdm(hw_value);
			basic_offset &= 0xF;
		}

		/*
		 * Get the pointer that we need to point to
		 * from the direct map
		 */
		rateptr = b43_shm_read16(dev, B43_SHM_SHARED,
					 direct + 2 * basic_offset);
		/* and write it to the basic map */
		b43_shm_write16(dev, B43_SHM_SHARED, basic + 2 * offset,
				rateptr);
	}
}

/* ipxe noteS: This is becoming an extension to b43_op_config and nothing
 * more. It is kept for posterity and easy diff for backporting
 */
inline static void b43_op_bss_info_changed(struct net80211_device *ndev,
                                           u32 changed)
{
	struct b43_wl *wl = ndev->priv;
	struct b43_wldev *dev;

	dev = wl->current_dev;
	if (!dev || b43_status(dev) < B43_STAT_STARTED)
		goto out_unlock_mutex;

	if (changed & NET80211_CFG_ASSOC) {
		//DBGC(ndev, "Assoc \"%s\"\n", ndev->essid);
		if (ndev->bssid)
			memcpy(wl->bssid, ndev->bssid, ETH_ALEN);
		else
			memset(wl->bssid, 0, ETH_ALEN);
	}

	if (b43_status(dev) >= B43_STAT_INITIALIZED) {
		if (changed & NET80211_CFG_ASSOC)
			b43_write_mac_bssid_templates(dev);
	}

	b43_mac_suspend(dev);

	if (changed & NET80211_CFG_RATE) {
		b43_update_basic_rates(dev, ndev->basic_rates);
	}

	/* CTS protection, short preambles,
	 * and short-slot operation.
	 */
	if (changed & NET80211_CFG_PHY_PARAMS) {
		//DBGC(ndev, "PHY PARAMS CHANGED: %d; ", ndev->phy_flags);
		if (ndev->phy_flags & NET80211_PHY_USE_SHORT_SLOT)
			b43_short_slot_timing_enable(dev);
		else
			b43_short_slot_timing_disable(dev);

		/* other phy params handled in xmit.c b43_generate_txhdr */
	}

out_unlock_mutex:
	return;
}

/* Returns the current dev. */
static struct b43_wldev * b43_wireless_core_stop(struct b43_wldev *dev)
{
	struct b43_wl *wl;
	struct b43_wldev *orig_dev;
	u32 mask;
	int queue_num;

	if (!dev)
		return NULL;
	wl = dev->wl;
redo:
	if (!dev || b43_status(dev) < B43_STAT_STARTED)
		return dev;

	/* Cancel work. Unlock to avoid deadlocks. */
	cancel_delayed_work_sync(&dev->periodic_work);
	dev = wl->current_dev;
	if (!dev || b43_status(dev) < B43_STAT_STARTED) {
		/* Whoops, aliens ate up the device while we were unlocked. */
		return dev;
	}

	/* Disable interrupts on the device. */
	b43_set_status(dev, B43_STAT_INITIALIZED);
	if (b43_bus_host_is_sdio(dev->dev)) {
		b43_write32(dev, B43_MMIO_GEN_IRQ_MASK, 0);
		b43_read32(dev, B43_MMIO_GEN_IRQ_MASK);	/* Flush */
	} else {
		b43_write32(dev, B43_MMIO_GEN_IRQ_MASK, 0);
		b43_read32(dev, B43_MMIO_GEN_IRQ_MASK);	/* Flush */
	}
	/* Synchronize and free the interrupt handlers. Unlock to avoid deadlocks. */
	orig_dev = dev;

	dev = wl->current_dev;
	if (!dev)
		return dev;
	if (dev != orig_dev) {
		if (b43_status(dev) >= B43_STAT_STARTED)
			goto redo;
		return dev;
	}
	mask = b43_read32(dev, B43_MMIO_GEN_IRQ_MASK);
	B43_WARN_ON(mask != 0xFFFFFFFF && mask);

	/* Drain all TX queues. */
	for (queue_num = 0; queue_num < B43_QOS_QUEUE_NUM; queue_num++) {
		struct io_buffer *iob, *tmp;
		list_for_each_entry_safe(iob, tmp, &wl->tx_queue[queue_num], list) {
			net80211_tx_complete(b43_wldev_to_ndev(dev), iob, 0, 0);
		}
	}

	b43_mac_suspend(dev);
	b43_leds_exit(dev);
	b43dbg(wl, "Wireless interface stopped\n");

	return dev;
}

static int b43_wireless_core_start(struct b43_wldev *dev)
{
	DBGC (dev, "b43_wireless_core_start; ");
	B43_WARN_ON(b43_status(dev) != B43_STAT_INITIALIZED);

	drain_txstatus_queue(dev);

	/* We are ready to run. */
	b43_set_status(dev, B43_STAT_STARTED);

	/* Start data flow (TX/RX). */
	b43_mac_enable(dev);
	b43_write32(dev, B43_MMIO_GEN_IRQ_MASK, b43_irq_mask(dev));

	/* Start maintenance work */
	b43_periodic_tasks_setup(dev);

	b43_leds_init(dev);

	b43dbg(dev->wl, "Wireless interface started\n");

	return 0;
}

static char *b43_phy_name(struct b43_wldev *dev __unused, u8 phy_type)
{
	switch (phy_type) {
	case B43_PHYTYPE_A:
		return "A";
	case B43_PHYTYPE_B:
		return "B";
	case B43_PHYTYPE_G:
		return "G";
	case B43_PHYTYPE_N:
		return "N";
	case B43_PHYTYPE_LP:
		return "LP";
	case B43_PHYTYPE_SSLPN:
		return "SSLPN";
	case B43_PHYTYPE_HT:
		return "HT";
	case B43_PHYTYPE_LCN:
		return "LCN";
	case B43_PHYTYPE_LCNXN:
		return "LCNXN";
	case B43_PHYTYPE_LCN40:
		return "LCN40";
	case B43_PHYTYPE_AC:
		return "AC";
	}
	return "UNKNOWN";
}

/* Get PHY and RADIO versioning numbers */
static int b43_phy_versioning(struct b43_wldev *dev)
{
	struct b43_phy *phy = &dev->phy;
	const u8 core_rev = dev->dev->core_rev;
	u32 tmp;
	u8 analog_type;
	u8 phy_type;
	u8 phy_rev;
	u16 radio_manuf;
	u16 radio_id;
	u16 radio_rev;
	u8 radio_ver;
	int unsupported = 0;

	/* Get PHY versioning */
	tmp = b43_read16(dev, B43_MMIO_PHY_VER);
	analog_type = (tmp & B43_PHYVER_ANALOG) >> B43_PHYVER_ANALOG_SHIFT;
	phy_type = (tmp & B43_PHYVER_TYPE) >> B43_PHYVER_TYPE_SHIFT;
	phy_rev = (tmp & B43_PHYVER_VERSION);

	/* LCNXN is continuation of N which run out of revisions */
	if (phy_type == B43_PHYTYPE_LCNXN) {
		phy_type = B43_PHYTYPE_N;
		phy_rev += 16;
	}

	switch (phy_type) {
#ifdef CONFIG_B43_PHY_G
	case B43_PHYTYPE_G:
		if (phy_rev > 9)
			unsupported = 1;
		break;
#endif
#ifdef CONFIG_B43_PHY_N
	case B43_PHYTYPE_N:
		if (phy_rev >= 19)
			unsupported = 1;
		break;
#endif
#ifdef CONFIG_B43_PHY_LP
	case B43_PHYTYPE_LP:
		if (phy_rev > 2)
			unsupported = 1;
		break;
#endif
#ifdef CONFIG_B43_PHY_HT
	case B43_PHYTYPE_HT:
		if (phy_rev > 1)
			unsupported = 1;
		break;
#endif
#ifdef CONFIG_B43_PHY_LCN
	case B43_PHYTYPE_LCN:
		if (phy_rev > 1)
			unsupported = 1;
		break;
#endif
	default:
		unsupported = 1;
	}
	if (unsupported) {
		b43err(dev->wl, "FOUND UNSUPPORTED PHY (Analog %d, Type %d (%s), Revision %d)\n",
		       analog_type, phy_type, b43_phy_name(dev, phy_type),
		       phy_rev);
		return -EOPNOTSUPP;
	}
	b43info(dev->wl, "Found PHY: Analog %d, Type %d (%s), Revision %d\n",
		analog_type, phy_type, b43_phy_name(dev, phy_type), phy_rev);

	/* Get RADIO versioning */
	if (core_rev == 40 || core_rev == 42) {
		radio_manuf = 0x17F;

		b43_write16f(dev, B43_MMIO_RADIO24_CONTROL, 0);
		radio_rev = b43_read16(dev, B43_MMIO_RADIO24_DATA);

		b43_write16f(dev, B43_MMIO_RADIO24_CONTROL, 1);
		radio_id = b43_read16(dev, B43_MMIO_RADIO24_DATA);

		radio_ver = 0; /* Is there version somewhere? */
	} else if (core_rev >= 24) {
		u16 radio24[3];

		for (tmp = 0; tmp < 3; tmp++) {
			b43_write16f(dev, B43_MMIO_RADIO24_CONTROL, tmp);
			radio24[tmp] = b43_read16(dev, B43_MMIO_RADIO24_DATA);
		}

		radio_manuf = 0x17F;
		radio_id = (radio24[2] << 8) | radio24[1];
		radio_rev = (radio24[0] & 0xF);
		radio_ver = (radio24[0] & 0xF0) >> 4;
	} else {
		if (dev->dev->chip_id == 0x4317) {
			if (dev->dev->chip_rev == 0)
				tmp = 0x3205017F;
			else if (dev->dev->chip_rev == 1)
				tmp = 0x4205017F;
			else
				tmp = 0x5205017F;
		} else {
			b43_write16f(dev, B43_MMIO_RADIO_CONTROL,
				     B43_RADIOCTL_ID);
			tmp = b43_read16(dev, B43_MMIO_RADIO_DATA_LOW);
			b43_write16f(dev, B43_MMIO_RADIO_CONTROL,
				     B43_RADIOCTL_ID);
			tmp |= b43_read16(dev, B43_MMIO_RADIO_DATA_HIGH) << 16;
		}
		radio_manuf = (tmp & 0x00000FFF);
		radio_id = (tmp & 0x0FFFF000) >> 12;
		radio_rev = (tmp & 0xF0000000) >> 28;
		radio_ver = 0; /* Probably not available on old hw */
	}

	if (radio_manuf != 0x17F /* Broadcom */)
		unsupported = 1;
	switch (phy_type) {
	case B43_PHYTYPE_A:
		if (radio_id != 0x2060)
			unsupported = 1;
		if (radio_rev != 1)
			unsupported = 1;
		if (radio_manuf != 0x17F)
			unsupported = 1;
		break;
	case B43_PHYTYPE_B:
		if ((radio_id & 0xFFF0) != 0x2050)
			unsupported = 1;
		break;
	case B43_PHYTYPE_G:
		if (radio_id != 0x2050)
			unsupported = 1;
		break;
	case B43_PHYTYPE_N:
		if (radio_id != 0x2055 && radio_id != 0x2056 &&
		    radio_id != 0x2057)
			unsupported = 1;
		if (radio_id == 0x2057 &&
		    !(radio_rev == 9 || radio_rev == 14))
			unsupported = 1;
		break;
	case B43_PHYTYPE_LP:
		if (radio_id != 0x2062 && radio_id != 0x2063)
			unsupported = 1;
		break;
	case B43_PHYTYPE_HT:
		if (radio_id != 0x2059)
			unsupported = 1;
		break;
	case B43_PHYTYPE_LCN:
		if (radio_id != 0x2064)
			unsupported = 1;
		break;
	default:
		B43_WARN_ON(1);
	}
	if (unsupported) {
		b43err(dev->wl,
		       "FOUND UNSUPPORTED RADIO (Manuf 0x%X, ID 0x%X, Revision %d, Version %d)\n",
		       radio_manuf, radio_id, radio_rev, radio_ver);
		return -EOPNOTSUPP;
	}
	b43info(dev->wl,
		"Found Radio: Manuf 0x%X, ID 0x%X, Revision %d, Version %d\n",
		radio_manuf, radio_id, radio_rev, radio_ver);

	/* FIXME: b43 treats "id" as "ver" and ignores the real "ver" */
	phy->radio_manuf = radio_manuf;
	phy->radio_ver = radio_id;
	phy->radio_rev = radio_rev;

	phy->analog = analog_type;
	phy->type = phy_type;
	phy->rev = phy_rev;

	return 0;
}

static void setup_struct_phy_for_init(struct b43_wldev *dev __unused,
				      struct b43_phy *phy)
{
	phy->hardware_power_control = !!modparam_hwpctl;
	phy->next_txpwr_check_time = jiffies;
	/* PHY TX errors counter. */
	atomic_set(&phy->txerr_cnt, B43_PHY_TX_BADNESS_LIMIT);

#if B43_DEBUG
	phy->phy_locked = false;
	phy->radio_locked = false;
#endif
}

static void setup_struct_wldev_for_init(struct b43_wldev *dev)
{
	dev->dfq_valid = false;

	/* Assume the radio is enabled. If it's not enabled, the state will
	 * immediately get fixed on the first periodic work run. */
	dev->radio_hw_enable = true;

	/* Stats */
	memset(&dev->stats, 0, sizeof(dev->stats));

	setup_struct_phy_for_init(dev, &dev->phy);

	/* IRQ related flags */
	dev->irq_reason = 0;
	memset(dev->dma_reason, 0, sizeof(dev->dma_reason));
	dev->irq_mask = B43_IRQ_MASKTEMPLATE;
	if (b43_modparam_verbose < B43_VERBOSITY_DEBUG)
		dev->irq_mask &= ~B43_IRQ_PHY_TXERR;
	dev->irq_enable = true;	// disable interrupts until ipxe irq() is called

	dev->mac_suspended = 1;

	/* Noise calculation context */
	memset(&dev->noisecalc, 0, sizeof(dev->noisecalc));
}

static void b43_bluetooth_coext_enable(struct b43_wldev *dev)
{
	struct ssb_sprom *sprom = dev->dev->bus_sprom;
	u64 hf;

	if (!modparam_btcoex)
		return;
	if (!(sprom->boardflags_lo & B43_BFL_BTCOEXIST))
		return;
	if (dev->phy.type != B43_PHYTYPE_B && !dev->phy.gmode)
		return;

	hf = b43_hf_read(dev);
	if (sprom->boardflags_lo & B43_BFL_BTCMOD)
		hf |= B43_HF_BTCOEXALT;
	else
		hf |= B43_HF_BTCOEX;
	b43_hf_write(dev, hf);
}

static void b43_bluetooth_coext_disable(struct b43_wldev *dev __unused)
{
	if (!modparam_btcoex)
		return;
	//TODO
}

static void b43_imcfglo_timeouts_workaround(struct b43_wldev *dev)
{
	struct ssb_bus *bus;
	u32 tmp;

#ifdef CONFIG_B43_SSB
	if (dev->dev->bus_type != B43_BUS_SSB)
		return;
#else
	return;
#endif

	bus = dev->dev->sdev->bus;

	if ((bus->chip_id == 0x4311 && bus->chip_rev == 2) ||
	    (bus->chip_id == 0x4312)) {
		tmp = ssb_read32(dev->dev->sdev, SSB_IMCFGLO);
		tmp &= ~SSB_IMCFGLO_REQTO;
		tmp &= ~SSB_IMCFGLO_SERTO;
		tmp |= 0x3;
		ssb_write32(dev->dev->sdev, SSB_IMCFGLO, tmp);
		ssb_commit_settings(bus);
	}
}

static void b43_set_synth_pu_delay(struct b43_wldev *dev, bool idle)
{
	u16 pu_delay;

	/* The time value is in microseconds. */
	if (dev->phy.type == B43_PHYTYPE_A)
		pu_delay = 3700;
	else
		pu_delay = 1050;
	if (idle)
		pu_delay = 500;
	if ((dev->phy.radio_ver == 0x2050) && (dev->phy.radio_rev == 8))
		pu_delay = max(pu_delay, (u16)2400);

	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_SPUWKUP, pu_delay);
}

/* Set the TSF CFP pre-TargetBeaconTransmissionTime. */
static void b43_set_pretbtt(struct b43_wldev *dev)
{
  u16 pretbtt;

  /* The time value is in microseconds. */

  if (dev->phy.type == B43_PHYTYPE_A)
    pretbtt = 120;
  else
    pretbtt = 250;
  b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_PRETBTT, pretbtt);
  b43_write16(dev, B43_MMIO_TSF_CFP_PRETBTT, pretbtt);
}

/* Shutdown a wireless core */
static void b43_wireless_core_exit(struct b43_wldev *dev)
{
	B43_WARN_ON(dev && b43_status(dev) > B43_STAT_INITIALIZED);
	if (!dev || b43_status(dev) != B43_STAT_INITIALIZED)
		return;

	b43_set_status(dev, B43_STAT_UNINIT);

	/* Stop the microcode PSM. */
	b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_PSM_RUN,
		      B43_MACCTL_PSM_JMP0);

	switch (dev->dev->bus_type) {
#ifdef CONFIG_B43_BCMA
	case B43_BUS_BCMA:
		bcma_core_pci_down(dev->dev->bdev->bus);
		break;
#endif
#ifdef CONFIG_B43_SSB
	case B43_BUS_SSB:
		/* TODO */
		break;
#endif
	}

	b43_dma_free(dev);
	b43_pio_free(dev);
	b43_chip_exit(dev);
	dev->phy.ops->switch_analog(dev, 0);

	b43_device_disable(dev, 0);
	b43_bus_may_powerdown(dev);
}

/* Initialize a wireless core */
static int b43_wireless_core_init(struct b43_wldev *dev)
{
	struct ssb_sprom *sprom = dev->dev->bus_sprom;
	struct b43_phy *phy = &dev->phy;
	int err;
	u64 hf;

	DBGC(dev, "b43_wireless_core_init; ");
	B43_WARN_ON(b43_status(dev) != B43_STAT_UNINIT);

	err = b43_bus_powerup(dev, 0);
	if (err)
		goto out;
	if (!b43_device_is_enabled(dev))
		b43_wireless_core_reset(dev, phy->gmode);

	/* Reset all data structures. */
	setup_struct_wldev_for_init(dev);
	phy->ops->prepare_structs(dev);

	/* Enable IRQ routing to this device. */
	switch (dev->dev->bus_type) {
#ifdef CONFIG_B43_BCMA
	case B43_BUS_BCMA:
		bcma_core_pci_irq_ctl(&dev->dev->bdev->bus->drv_pci[0],
				      dev->dev->bdev, true);
		bcma_core_pci_up(dev->dev->bdev->bus);
		break;
#endif
#ifdef CONFIG_B43_SSB
	case B43_BUS_SSB:
		ssb_pcicore_dev_irqvecs_enable(&dev->dev->sdev->bus->pcicore,
					       dev->dev->sdev);
		break;
#endif
	}

	b43_imcfglo_timeouts_workaround(dev);
	b43_bluetooth_coext_disable(dev);
	if (phy->ops->prepare_hardware) {
		err = phy->ops->prepare_hardware(dev);
		if (err)
			goto err_busdown;
	}
	err = b43_chip_init(dev);
	if (err)
		goto err_busdown;
	b43_shm_write16(dev, B43_SHM_SHARED,
			B43_SHM_SH_WLCOREREV, dev->dev->core_rev);
	hf = b43_hf_read(dev);
	if (phy->type == B43_PHYTYPE_G) {
		hf |= B43_HF_SYMW;
		if (phy->rev == 1)
			hf |= B43_HF_GDCW;
		if (sprom->boardflags_lo & B43_BFL_PACTRL)
			hf |= B43_HF_OFDMPABOOST;
	}
	if (phy->radio_ver == 0x2050) {
		if (phy->radio_rev == 6)
			hf |= B43_HF_4318TSSI;
		if (phy->radio_rev < 6)
			hf |= B43_HF_VCORECALC;
	}
	if (sprom->boardflags_lo & B43_BFL_XTAL_NOSLOW)
		hf |= B43_HF_DSCRQ; /* Disable slowclock requests from ucode. */
#if defined(CONFIG_B43_SSB) && defined(CONFIG_SSB_DRIVER_PCICORE)
	if (dev->dev->bus_type == B43_BUS_SSB &&
	    dev->dev->sdev->bus->bustype == SSB_BUSTYPE_PCI &&
	    dev->dev->sdev->bus->pcicore.dev->id.revision <= 10)
		hf |= B43_HF_PCISCW; /* PCI slow clock workaround. */
#endif
	hf &= ~B43_HF_SKCFPUP;
	b43_hf_write(dev, hf);

	/* tell the ucode MAC capabilities */
	if (dev->dev->core_rev >= 13) {
		u32 mac_hw_cap = b43_read32(dev, B43_MMIO_MAC_HW_CAP);

		b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_MACHW_L,
				mac_hw_cap & 0xffff);
		b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_MACHW_H,
				(mac_hw_cap >> 16) & 0xffff);
	}

	b43_set_retry_limits(dev, B43_DEFAULT_SHORT_RETRY_LIMIT,
			     B43_DEFAULT_LONG_RETRY_LIMIT);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_SFFBLIM, 3);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_LFFBLIM, 2);

	/* Disable sending probe responses from firmware.
	 * Setting the MaxTime to one usec will always trigger
	 * a timeout, so we never send any probe resp.
	 * A timeout of zero is infinite. */
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_PRMAXTIME, 1);

	b43_rate_memory_init(dev);
	b43_set_phytxctl_defaults(dev);

	/* Minimum Contention Window */
	if (phy->type == B43_PHYTYPE_B)
		b43_shm_write16(dev, B43_SHM_SCRATCH, B43_SHM_SC_MINCONT, 0x1F);
	else
		b43_shm_write16(dev, B43_SHM_SCRATCH, B43_SHM_SC_MINCONT, 0xF);
	/* Maximum Contention Window */
	b43_shm_write16(dev, B43_SHM_SCRATCH, B43_SHM_SC_MAXCONT, 0x3FF);

	/* write phytype and phyvers */
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_PHYTYPE, phy->type);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_PHYVER, phy->rev);

	if (b43_bus_host_is_pcmcia(dev->dev) ||
	    b43_bus_host_is_sdio(dev->dev)) {
		dev->__using_pio_transfers = true;
		err = b43_pio_init(dev);
	} else if (dev->use_pio) {
		dev->__using_pio_transfers = true;
		err = b43_pio_init(dev);
	} else {
		dev->__using_pio_transfers = false;
		err = b43_dma_init(dev);
	}
	if (err)
		goto err_chip_exit;
	b43_qos_init(dev);
  b43_set_pretbtt(dev);
	b43_set_synth_pu_delay(dev, 1);
	b43_bluetooth_coext_enable(dev);

	b43_bus_powerup(dev, !(sprom->boardflags_lo & B43_BFL_XTAL_NOSLOW));

	/* Let's allow the user to specify an alternate mac address before
	 * opening the interface. However, do not write this address to hw
	 * persistent memory. Although it may be possible to rewrite the SSB
	 * sprom hw ethaddr, this is considered unsupported and a 'do at your
	 * own risk' manuever. */
	{
		struct net80211_device *ndev = b43_wldev_to_ndev(dev);
		/* Uncomment next line to force the hardware address */
		/* memcpy( ndev->netdev->ll_addr, ndev->hwinfo->hwaddr, ETH_ALEN); */
		memcpy(dev->wl->mac_addr, ndev->netdev->ll_addr, ETH_ALEN);
		/* wl->mac_addr will be used to hw-filter RX packets, and ll_addr
		 * will be used by ipxe software MAC for TX packet generation, and
		 * possibly other stuff on RX */
	}

	b43_upload_card_macaddress(dev);
	b43_security_init(dev);

	b43_set_status(dev, B43_STAT_INITIALIZED);

out:
	return err;

err_chip_exit:
	b43_chip_exit(dev);
err_busdown:
	b43_bus_may_powerdown(dev);
	B43_WARN_ON(b43_status(dev) != B43_STAT_UNINIT);
	return err;
}

static int b43_op_start(struct net80211_device *ndev)
{
	struct b43_wl *wl = ndev_to_b43_wl(ndev);
	struct b43_wldev *dev = wl->current_dev;
	int did_init = 0;
	int err = 0;

#ifdef B43_SETTINGS
	dev->use_pio = !fetch_intz_setting ( NULL, &b43_dma_setting );
#endif

	/* Kill all old instance specific information to make sure
	 * the card won't use it in the short timeframe between start
	 * and mac80211 reconfiguring it. */
	memset(wl->bssid, 0, ETH_ALEN);
	memset(wl->mac_addr, 0, ETH_ALEN);
	wl->filter_flags = 0;
	wl->radio_enabled = true;


	if (b43_status(dev) < B43_STAT_INITIALIZED) {
		err = b43_wireless_core_init(dev);
		if (err)
			goto out_mutex_unlock;
		did_init = 1;
	}

	if (b43_status(dev) < B43_STAT_STARTED) {
		err = b43_wireless_core_start(dev);
		if (err) {
			if (did_init)
				b43_wireless_core_exit(dev);
			goto out_mutex_unlock;
		}
	}

	/* XXX: only do if device doesn't support rfkill irq */
	// Fixme: ipxe rfkill poll?
	//wiphy_rfkill_start_polling(hw->wiphy);

 out_mutex_unlock:

	/*
	 * Configuration may have been overwritten during initialization.
	 * Reload the configuration, but only if initialization was
	 * successful. Reloading the configuration after a failed init
	 * may hang the system.
	 */
	if (!err) {
		b43_set_beacon_listen_interval(dev, 5);
		b43_op_config(ndev, ~0);
	}

	return err;
}

static void b43_op_stop(struct net80211_device *ndev)
{
	struct b43_wl *wl = ndev_to_b43_wl(ndev);
	struct b43_wldev *dev = wl->current_dev;

	if (!dev)
		goto out;

	if (b43_status(dev) >= B43_STAT_STARTED) {
		dev = b43_wireless_core_stop(dev);
		if (!dev)
			goto out_unlock;
	}
	b43_wireless_core_exit(dev);
	wl->radio_enabled = false;

out_unlock:
out:
	return;
}

static struct net80211_device_operations b43_hw_ops = {
	.transmit = b43_op_tx,
	.open     = b43_op_start,
	.close    = b43_op_stop,
	.config   = b43_op_config,
	.poll     = b43_interrupt_handler,
	.irq      = b43_irq
};

/* Hard-reset the chip. Do not call this directly.
 * Use b43_controller_restart()
 */
static void b43_chip_reset(struct b43_wldev *dev)
{
	struct b43_wl *wl = dev->wl;
	int err = 0;
	int prev_status;


	prev_status = b43_status(dev);
	/* Bring the device down... */
	if (prev_status >= B43_STAT_STARTED) {
		dev = b43_wireless_core_stop(dev);
		if (!dev) {
			err = -ENODEV;
			goto out;
		}
	}
	if (prev_status >= B43_STAT_INITIALIZED)
		b43_wireless_core_exit(dev);

	/* ...and up again. */
	if (prev_status >= B43_STAT_INITIALIZED) {
		err = b43_wireless_core_init(dev);
		if (err)
			goto out;
	}
	if (prev_status >= B43_STAT_STARTED) {
		err = b43_wireless_core_start(dev);
		if (err) {
			b43_wireless_core_exit(dev);
			goto out;
		}
	}
out:
	if (err)
		wl->current_dev = NULL; /* Failed to init the dev. */

	if (err) {
		b43err(wl, "Controller restart FAILED\n");
		return;
	}

	/* reload configuration */
	b43_op_config(wl->ndev, ~0);

	b43info(wl, "Controller restarted\n");
}

static int b43_setup_bands(struct b43_wldev *dev,
			   bool have_2ghz_phy, bool have_5ghz_phy)
{
	struct net80211_hw_info *hwinfo = dev->wl->hwinfo;
	struct b43_phy *phy = &dev->phy;
	bool limited_2g;
	bool limited_5g;
	unsigned int i;
	size_t chanslots_available;

	/* We don't support all 2 GHz channels on some devices */
	limited_2g = phy->radio_ver == 0x2057 &&
		     (phy->radio_rev == 9 || phy->radio_rev == 14);
	limited_5g = phy->radio_ver == 0x2057 &&
		     phy->radio_rev == 9;

	assert(hwinfo->nr_channels == 0);

	if (have_2ghz_phy)
	{
		size_t n_channels = limited_2g ?
			b43_2ghz_chantable_limited_size : ARRAY_SIZE(b43_2ghz_chantable);

		memcpy(&hwinfo->channels[hwinfo->nr_channels], b43_2ghz_chantable,
			n_channels * sizeof(struct net80211_channel));

		hwinfo->nr_channels += n_channels;

		for (i = 0; i < b43_g_ratetable_size; i++)
			hwinfo->rates[NET80211_BAND_2GHZ][i] = b43_g_ratetable[i].bitrate;

		hwinfo->nr_rates[NET80211_BAND_2GHZ] = b43_g_ratetable_size;

		hwinfo->bands |= NET80211_BAND_BIT_2GHZ;
		hwinfo->modes |= NET80211_MODE_B | NET80211_MODE_G;
	}
	if (have_5ghz_phy)
	{
		size_t n_channels = 0;
		const struct net80211_channel *channels = NULL;

		if (dev->phy.type == B43_PHYTYPE_N) {
			n_channels = limited_5g ?
				ARRAY_SIZE(b43_5ghz_nphy_chantable_limited) :
				ARRAY_SIZE(b43_5ghz_nphy_chantable);

			channels = limited_5g ?
				b43_5ghz_nphy_chantable_limited : b43_5ghz_nphy_chantable;

			hwinfo->modes |= NET80211_MODE_N | NET80211_MODE_A;
		}
		else {
			n_channels = ARRAY_SIZE(b43_5ghz_aphy_chantable);
			channels = b43_5ghz_aphy_chantable;
			hwinfo->modes |= NET80211_MODE_A;
		}

		// common 5ghz section
		chanslots_available = min(n_channels,
		    (size_t)(NET80211_MAX_CHANNELS - hwinfo->nr_channels));

		memcpy(&hwinfo->channels[hwinfo->nr_channels], channels,
           chanslots_available * sizeof(struct net80211_channel)); //n_channels);
		hwinfo->nr_channels += chanslots_available; // n_channels;

		for (i = 0; i < b43_a_ratetable_size; i++)
			hwinfo->rates[NET80211_BAND_5GHZ][i] = b43_a_ratetable[i].bitrate;

		hwinfo->nr_rates[NET80211_BAND_5GHZ] = b43_a_ratetable_size;
		hwinfo->bands |= NET80211_BAND_BIT_5GHZ;

		//memcpy(hwinfo->rates[NET80211_BAND_5GHZ], __b43_ratetable,
		//		sizeof(*__b43_ratetable) * b43_g_ratetable_size);
	}

	dev->phy.supports_2ghz = have_2ghz_phy;
	dev->phy.supports_5ghz = have_5ghz_phy;

	return 0;
}

static void b43_wireless_core_detach(struct b43_wldev *dev)
{
	/* We release firmware that late to not be required to re-request
	 * is all the time when we reinit the core. */
	b43_release_firmware(dev);
	b43_phy_free(dev);
}

static void b43_supported_bands(struct b43_wldev *dev, bool *have_2ghz_phy,
				bool *have_5ghz_phy)
{
	u16 dev_id = 0;

#ifdef CONFIG_B43_BCMA
	if (dev->dev->bus_type == B43_BUS_BCMA &&
	    dev->dev->bdev->bus->hosttype == BCMA_HOSTTYPE_PCI)
		dev_id = dev->dev->bdev->bus->host_pci->device;
#endif
#ifdef CONFIG_B43_SSB
	if (dev->dev->bus_type == B43_BUS_SSB &&
	    dev->dev->sdev->bus->bustype == SSB_BUSTYPE_PCI)
		dev_id = dev->dev->sdev->bus->host_pci->device;
#endif
	/* Override with SPROM value if available */
	if (dev->dev->bus_sprom->dev_id)
		dev_id = dev->dev->bus_sprom->dev_id;

	/* Note: below IDs can be "virtual" (not maching e.g. real PCI ID) */
	switch (dev_id) {
	case 0x4324: /* BCM4306 */
	case 0x4312: /* BCM4311 */
	case 0x4319: /* BCM4318 */
	case 0x4328: /* BCM4321 */
	case 0x432b: /* BCM4322 */
	case 0x4350: /* BCM43222 */
	case 0x4353: /* BCM43224 */
	case 0x0576: /* BCM43224 */
	case 0x435f: /* BCM6362 */
	case 0x4331: /* BCM4331 */
	case 0x4359: /* BCM43228 */
	case 0x43a0: /* BCM4360 */
	case 0x43b1: /* BCM4352 */
		/* Dual band devices */
		*have_2ghz_phy = true;
		*have_5ghz_phy = true;
		return;
	case 0x4321: /* BCM4306 */
	case 0x4313: /* BCM4311 */
	case 0x431a: /* BCM4318 */
	case 0x432a: /* BCM4321 */
	case 0x432d: /* BCM4322 */
	case 0x4352: /* BCM43222 */
	case 0x4333: /* BCM4331 */
	case 0x43a2: /* BCM4360 */
	case 0x43b3: /* BCM4352 */
		/* 5 GHz only devices */
		*have_2ghz_phy = false;
		*have_5ghz_phy = true;
		return;
	}

	/* As a fallback, try to guess using PHY type */
	switch (dev->phy.type) {
	case B43_PHYTYPE_A:
		*have_2ghz_phy = false;
		*have_5ghz_phy = true;
		return;
	case B43_PHYTYPE_G:
	case B43_PHYTYPE_N:
	case B43_PHYTYPE_LP:
	case B43_PHYTYPE_HT:
	case B43_PHYTYPE_LCN:
		*have_2ghz_phy = true;
		*have_5ghz_phy = false;
		return;
	}

	B43_WARN_ON(1);
}

static int b43_wireless_core_attach(struct b43_wldev *dev)
{
	struct b43_wl *wl = dev->wl;
	struct b43_phy *phy = &dev->phy;
	int err;
	u32 tmp;
	bool have_2ghz_phy = false, have_5ghz_phy = false;

	DBGC(dev, "wireless_core_attach; ");

	/* Do NOT do any device initialization here.
	 * Do it in wireless_core_init() instead.
	 * This function is for gathering basic information about the HW, only.
	 * Also some structs may be set up here. But most likely you want to have
	 * that in core_init(), too.
	 */

	err = b43_bus_powerup(dev, 0);
	if (err) {
		b43err(wl, "Bus powerup failed\n");
		goto out;
	}

	phy->do_full_init = true;

	/* Try to guess supported bands for the first init needs */
	switch (dev->dev->bus_type) {
#ifdef CONFIG_B43_BCMA
	case B43_BUS_BCMA:
		tmp = bcma_aread32(dev->dev->bdev, BCMA_IOST);
		have_2ghz_phy = !!(tmp & B43_BCMA_IOST_2G_PHY);
		have_5ghz_phy = !!(tmp & B43_BCMA_IOST_5G_PHY);
		break;
#endif
#ifdef CONFIG_B43_SSB
	case B43_BUS_SSB:
		if (dev->dev->core_rev >= 5) {
			tmp = ssb_read32(dev->dev->sdev, SSB_TMSHIGH);
			have_2ghz_phy = !!(tmp & B43_TMSHIGH_HAVE_2GHZ_PHY);
			have_5ghz_phy = !!(tmp & B43_TMSHIGH_HAVE_5GHZ_PHY);
		} else
			B43_WARN_ON(1);
		break;
#endif
	}

	dev->phy.gmode = have_2ghz_phy;
	b43_wireless_core_reset(dev, dev->phy.gmode);

	/* Get the PHY type. */
	err = b43_phy_versioning(dev);
	if (err)
		goto err_powerdown;

	/* Get real info about supported bands */
	b43_supported_bands(dev, &have_2ghz_phy, &have_5ghz_phy);

	/* We don't support 5 GHz on some PHYs yet */
	if (have_5ghz_phy) {
		switch (dev->phy.type) {
		case B43_PHYTYPE_A:
		case B43_PHYTYPE_G:
		case B43_PHYTYPE_LP:
		case B43_PHYTYPE_HT:
			b43warn(wl, "5 GHz band is unsupported on this PHY\n");
			have_5ghz_phy = false;
		}
	}

	if (!have_2ghz_phy && !have_5ghz_phy) {
		b43err(wl, "b43 can't support any band on this device\n");
		err = -EOPNOTSUPP;
		goto err_powerdown;
	}

	err = b43_phy_allocate(dev);
	if (err)
		goto err_powerdown;

	dev->phy.gmode = have_2ghz_phy;
	b43_wireless_core_reset(dev, dev->phy.gmode);

	err = b43_validate_chipaccess(dev);
	if (err)
		goto err_phy_free;
	err = b43_setup_bands(dev, have_2ghz_phy, have_5ghz_phy);
	if (err)
		goto err_phy_free;

	/* Now set some default "current_dev" */
	if (!wl->current_dev)
		wl->current_dev = dev;

	dev->phy.ops->switch_analog(dev, 0);
	b43_device_disable(dev, 0);
	b43_bus_may_powerdown(dev);

out:
	return err;

err_phy_free:
	b43_phy_free(dev);
err_powerdown:
	b43_bus_may_powerdown(dev);
	return err;
}

static void b43_one_core_detach(struct b43_bus_dev *dev)
{
	struct b43_wldev *wldev;

	/* Do not cancel ieee80211-workqueue based work here.
	 * See comment in b43_remove(). */

	wldev = b43_bus_get_wldev(dev);
	b43_wireless_core_detach(wldev);
	list_del(&wldev->list);
	b43_bus_set_wldev(dev, NULL);
	kfree(wldev);
}

static int b43_one_core_attach(struct b43_bus_dev *dev, struct b43_wl *wl)
{
	struct b43_wldev *wldev;
	int err = -ENOMEM;

	DBGC(dev, "b43_one_core_attach; ");

	wldev = kzalloc(sizeof(*wldev), GFP_KERNEL);
	if (!wldev)
		goto out;

	wldev->use_pio = b43_modparam_pio;
	wldev->dev = dev;
	wldev->wl = wl;
	b43_set_status(wldev, B43_STAT_UNINIT);
	wldev->bad_frames_preempt = modparam_bad_frames_preempt;
	INIT_LIST_HEAD(&wldev->list);

	err = b43_wireless_core_attach(wldev);
	if (err) {
    DBGC(dev, "wireless_core_attach failed!\n");
		goto err_kfree_wldev;
  }

	b43_bus_set_wldev(dev, wldev);

      out:
	return err;

      err_kfree_wldev:
	kfree(wldev);
	return err;
}

#define IS_PDEV(pdev, _vendor, _device, _subvendor, _subdevice)		( \
	(pdev->vendor == PCI_VENDOR_ID_##_vendor) &&			\
	(pdev->device == _device) &&					\
	(subsystem_vendor == PCI_VENDOR_ID_##_subvendor) &&	\
	(subsystem_device == _subdevice)				)

#ifdef CONFIG_B43_SSB
static void b43_sprom_fixup(struct ssb_bus *bus)
{
	struct pci_device *pdev;

	/* boardflags workarounds */
	if (bus->boardinfo.vendor == SSB_BOARDVENDOR_DELL &&
	    bus->chip_id == 0x4301 && bus->sprom.board_rev == 0x74)
		bus->sprom.boardflags_lo |= B43_BFL_BTCOEXIST;
	if (bus->boardinfo.vendor == PCI_VENDOR_ID_APPLE &&
	    bus->boardinfo.type == 0x4E && bus->sprom.board_rev > 0x40)
		bus->sprom.boardflags_lo |= B43_BFL_PACTRL;
	if (bus->bustype == SSB_BUSTYPE_PCI) {
		pdev = bus->host_pci;
		u16 subsystem_vendor;
		u16 subsystem_device;

		pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &subsystem_vendor);
		pci_read_config_word(pdev, PCI_SUBSYSTEM_ID, &subsystem_device);

		if (IS_PDEV(pdev, BROADCOM, 0x4318, ASUSTEK, 0x100F) ||
		    IS_PDEV(pdev, BROADCOM, 0x4320,    DELL, 0x0003) ||
		    IS_PDEV(pdev, BROADCOM, 0x4320,      HP, 0x12f8) ||
		    IS_PDEV(pdev, BROADCOM, 0x4320, LINKSYS, 0x0015) ||
		    IS_PDEV(pdev, BROADCOM, 0x4320, LINKSYS, 0x0014) ||
		    IS_PDEV(pdev, BROADCOM, 0x4320, LINKSYS, 0x0013) ||
		    IS_PDEV(pdev, BROADCOM, 0x4320, MOTOROLA, 0x7010))
			bus->sprom.boardflags_lo &= ~B43_BFL_BTCOEXIST;
	}
}

static void b43_wireless_exit(struct b43_bus_dev *dev,
                              struct b43_wl *wl)
{
	ssb_set_devtypedata(dev->sdev, NULL);
	net80211_free(b43_wl_to_ndev(wl));
}
#endif

static struct b43_wl *b43_wireless_init(struct b43_bus_dev *dev)
{
	struct ssb_sprom *sprom = dev->bus_sprom;
	struct net80211_hw_info *hwinfo;
	struct net80211_device *ndev;
	struct b43_wl *wl;
	char chip_name[6];
	int queue_num;

	DBGC(dev, "b43_wireless_init; ");

	hwinfo = zalloc(sizeof(*hwinfo));
	if (!hwinfo) {
		b43err(NULL, "hwinfo alloc failed\n");
		return ERR_PTR(-ENOMEM);
	}

	ndev = net80211_alloc(sizeof(*wl));
	if (!ndev) {
		b43err(NULL, "net80211 alloc failed\n");
		free(hwinfo);
		return ERR_PTR(-ENOMEM);
	}

	/* do this madness so we can get the display name and description from
	 * the parent bridge driver (as shown in ifstat) */
	ndev->netdev->dev = dev->dev->parent;

	wl = ndev_to_b43_wl(ndev);
	wl->ndev = ndev;
	/* fill hw info */
	// bands below
	hwinfo->flags = NET80211_HW_RX_HAS_FCS;
	hwinfo->signal_type = NET80211_SIGNAL_DBM;
	// hwinfo->signal_max unnecessary on NET80211_SIGNAL_DBM, and unused.
	hwinfo->channel_change_time = 4000; // fixme iPXE: taking a guess

	if (is_valid_ether_addr(sprom->et1mac))
		SET_IEEE80211_PERM_ADDR(hwinfo, sprom->et1mac);
	else
		SET_IEEE80211_PERM_ADDR(hwinfo, sprom->il0mac);

	/* Initialize struct b43_wl */
	wl->hwinfo = hwinfo;

	/* Initialize queues and flags. */
	for (queue_num = 0; queue_num < B43_QOS_QUEUE_NUM; queue_num++) {
		INIT_LIST_HEAD(&wl->tx_queue[queue_num]);
		wl->tx_queue_stopped[queue_num] = 0;
	}

	snprintf(chip_name, ARRAY_SIZE(chip_name),
		 (dev->chip_id > 0x9999) ? "%d" : "%04X", dev->chip_id);
	b43info(wl, "Broadcom %s WLAN found (core revision %d)\n", chip_name,
		dev->core_rev);
	return wl;
}

static void b43_print_driverinfo(void)
{
  const char *feat_pci = "", *feat_pcmcia = "", *feat_nphy = "",
        *feat_leds = "", *feat_sdio = "";

#ifdef CONFIG_B43_PCI_AUTOSELECT
  feat_pci = "P";
#endif
#ifdef CONFIG_B43_PCMCIA
  feat_pcmcia = "M";
#endif
#ifdef CONFIG_B43_PHY_N
  feat_nphy = "N";
#endif
#ifdef CONFIG_B43_LEDS
  feat_leds = "L";
#endif
#ifdef CONFIG_B43_SDIO
  feat_sdio = "S";
#endif
  printk(KERN_INFO "Broadcom 43xx driver loaded "
      "[ Features: %s%s%s%s%s ]\n",
      feat_pci, feat_pcmcia, feat_nphy,
      feat_leds, feat_sdio);
}

#ifdef CONFIG_B43_BCMA
static int b43_bcma_probe(struct bcma_device *core)
{
	struct b43_bus_dev *dev;
	struct b43_wl *wl;
	int err;

	if (!modparam_allhwsupport &&
	    (core->id.rev == 0x17 || core->id.rev == 0x18)) {
		pr_err("Support for cores revisions 0x17 and 0x18 disabled by module param allhwsupport=0. Try b43.allhwsupport=1\n");
		return -ENOTSUP;
	}

	dev = b43_bus_dev_bcma_init(core);
	if (!dev)
		return -ENODEV;

	wl = b43_wireless_init(dev);
	if (IS_ERR(wl)) {
		err = PTR_ERR(wl);
		goto bcma_out;
	}

	err = b43_one_core_attach(dev, wl);
	if (err)
		goto bcma_err_wireless_exit;

	/* setup and start work to load firmware */
	b43_request_firmware(wl);

bcma_out:
	return err;

bcma_err_wireless_exit:
	net80211_free(wl->ndev);
	return err;
}

static void b43_bcma_remove(struct bcma_device *core)
{
	struct b43_wldev *wldev = bcma_get_drvdata(core);
	struct b43_wl *wl = wldev->wl;

	/* We must cancel any work here before unregistering from ieee80211,
	 * as the ieee80211 unreg will destroy the workqueue. */

	B43_WARN_ON(!wl);
	if (!wldev->fw.ucode.data)
		return;			/* NULL if firmware never loaded */
	if (wl->current_dev == wldev && wl->hw_registred) {
		b43_leds_stop(wldev);
		net80211_unregister(wl->ndev);
	}

	b43_one_core_detach(wldev->dev);

	b43_leds_unregister(wl);
	net80211_free(b43_wl_to_ndev(wl));
}

const struct bcma_driver b43_bcma_driver __bcma_driver = {
	.name = "b43",
	.ids	= b43_bcma_tbl,
	.probe		= b43_bcma_probe,
	.remove		= b43_bcma_remove,
};
#endif

#ifdef CONFIG_B43_SSB
static int b43_ssb_probe(struct ssb_device *sdev,
                  const struct ssb_device_id *id __unused)
{
	struct b43_bus_dev *dev;
	struct b43_wl *wl;
	int err;

	b43_print_driverinfo();

  dev = b43_bus_dev_ssb_init(sdev);
	if (!dev)
		return -ENOMEM;

  // make sure this has never been filled
	wl = ssb_get_devtypedata(sdev);
	if (wl) {
		b43err(NULL, "Dual-core devices are not supported\n");
		err = -ENOTSUP;
		goto err_ssb_kfree_dev;
	}

	b43_sprom_fixup(sdev->bus);

	wl = b43_wireless_init(dev);
	if (IS_ERR(wl)) {
		DBGC(sdev, "b43_wireless_init() failed! ");
		err = PTR_ERR(wl);
		goto err_ssb_kfree_dev;
	}
	ssb_set_devtypedata(sdev, wl);
	B43_WARN_ON(ssb_get_devtypedata(sdev) != wl);

	err = b43_one_core_attach(dev, wl);
	if (err)
		goto err_ssb_wireless_exit;

	/* setup and start work to load firmware */
	b43_request_firmware(wl);

	return err;

err_ssb_wireless_exit:
	b43_wireless_exit(dev, wl);
err_ssb_kfree_dev:
	kfree(dev);
	return err;
}

static void b43_ssb_remove(struct ssb_device *sdev)
{
	struct b43_wl *wl = ssb_get_devtypedata(sdev);
	struct b43_wldev *wldev = ssb_get_drvdata(sdev);
	struct b43_bus_dev *dev = wldev->dev;

	/* We must cancel any work here before unregistering from ieee80211,
	 * as the ieee80211 unreg will destroy the workqueue. */

	B43_WARN_ON(!wl);
	if (!wldev->fw.ucode.data)
		return;			/* NULL if firmware never loaded */
	if (wl->current_dev == wldev && wl->hw_registred) {
		b43_leds_stop(wldev);
		net80211_unregister(wl->ndev);
	}

	b43_one_core_detach(dev);

	b43_leds_unregister(wl);
	b43_wireless_exit(dev, wl);
}

const struct ssb_driver b43_ssb_driver __ssb_driver = {
	.name		= "b43",
	.ids	= b43_ssb_tbl,
	.probe		= b43_ssb_probe,
	.remove		= b43_ssb_remove,
};
#endif /* CONFIG_B43_SSB */

/* Perform a hardware reset. This can be called from any context. */
void b43_controller_restart(struct b43_wldev *dev, const char *reason)
{
	/* Must avoid requeueing, if we are in shutdown. */
	if (b43_status(dev) < B43_STAT_INITIALIZED)
		return;
	b43info(dev->wl, "Controller RESET (%s) ...\n", reason);
	b43_chip_reset(dev);
}
