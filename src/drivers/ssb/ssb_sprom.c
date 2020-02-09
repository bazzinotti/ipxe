/*
 * Sonics Silicon Backplane
 * Common SPROM support routines
 *
 * Ported to iPXE
 * Copyright (c) 2020 Michael Bazzinotti <bazz@bazz1.com>
 * Original from Linux kernel 3.18.11
 *
 * Copyright (C) 2005-2008 Michael Buesch <m@bues.ch>
 * Copyright (C) 2005 Martin Langer <martin-langer@gmx.de>
 * Copyright (C) 2005 Stefano Brivio <st3@riseup.net>
 * Copyright (C) 2005 Danny van Dyk <kugelfang@gentoo.org>
 * Copyright (C) 2005 Andreas Jaggi <andreas.jaggi@waterwave.ch>
 *
 * Licensed under the GNU/GPL. See COPYING for details.
 */

#include "ssb_private.h"


static int(*get_fallback_sprom)(struct ssb_bus *dev, struct ssb_sprom *out);

/**
 * ssb_arch_register_fallback_sprom - Registers a method providing a
 * fallback SPROM if no SPROM is found.
 *
 * @sprom_callback: The callback function.
 *
 * With this function the architecture implementation may register a
 * callback handler which fills the SPROM data structure. The fallback is
 * only used for PCI based SSB devices, where no valid SPROM can be found
 * in the shadow registers.
 *
 * This function is useful for weird architectures that have a half-assed
 * SSB device hardwired to their PCI bus.
 *
 * Note that it does only work with PCI attached SSB devices. PCMCIA
 * devices currently don't use this fallback.
 * Architectures must provide the SPROM for native SSB devices anyway, so
 * the fallback also isn't used for native devices.
 *
 * This function is available for architecture code, only. So it is not
 * exported.
 */
int ssb_arch_register_fallback_sprom(int (*sprom_callback)(struct ssb_bus *bus,
				     struct ssb_sprom *out))
{
	if (get_fallback_sprom)
		return -EEXIST;
	get_fallback_sprom = sprom_callback;

	return 0;
}

int ssb_fill_sprom_with_fallback(struct ssb_bus *bus, struct ssb_sprom *out)
{
	if (!get_fallback_sprom)
		return -ENOENT;

	return get_fallback_sprom(bus, out);
}

/* http://bcm-v4.sipsolutions.net/802.11/IsSpromAvailable */
bool ssb_is_sprom_available(struct ssb_bus *bus)
{
	/* status register only exists on chipcomon rev >= 11 and we need check
	   for >= 31 only */
	/* this routine differs from specs as we do not access SPROM directly
	   on PCMCIA */
	if (bus->bustype == SSB_BUSTYPE_PCI &&
	    bus->chipco.dev &&	/* can be unavailable! */
	    bus->chipco.dev->id.revision >= 31)
		return bus->chipco.capabilities & SSB_CHIPCO_CAP_SPROM;

	return true;
}
