/*
 * Broadcom 43xx PCI-SSB bridge module
 *
 * Ported to iPXE
 * Copyright (c) 2017-2020 Michael Bazzinotti <bazz@bazz1.com>
 * Original from Linux kernel 3.18.11
 *
 * This technically is a separate PCI driver module, but
 * because of its small size we include it in the SSB core
 * instead of creating a standalone module.
 *
 * Copyright 2007  Michael Buesch <m@bues.ch>
 *
 * Licensed under the GNU/GPL. See COPYING for details.
 */

#include <ipxe/pci.h>
#include <ipxe/ssb/ssb.h>
#include <ipxe/ssb/ssb_pcihost_wrapper.h>
#include "../../ssb/ssb_private.h"

/*#define PCI_VENDOR_ID_BROADCOM    0x14e4
 * we can't use definitions or decimal in the PCI table. it messes up
 * util/parse_roms. Use hex codes
*/

/* names and descriptions derived from The PCI ID Repository
 * https://pci-ids.ucw.cz */
static struct pci_device_id b43_pci_bridge_tbl[] = {
	PCI_ROM (0x14e4, 0x4301, "BCM4301", "BCM4301 802.11b Wireless LAN Controller", 0),
	PCI_ROM (0x14e4, 0x4306, "BCM4306", "BCM4306 802.11bg Wireless LAN controller", 0),
	PCI_ROM (0x14e4, 0x4307, "BCM4306", "BCM4306 802.11bg Wireless LAN Controller", 0),
	PCI_ROM (0x14e4, 0x4311, "BCM4311", "BCM4311 802.11b/g WLAN", 0),
	PCI_ROM (0x14e4, 0x4312, "BCM4311", "BCM4311 802.11a/b/g", 0),
	PCI_ROM (0x14e4, 0x4315, "BCM4312", "BCM4312 802.11b/g LP-PHY", 0),
	PCI_ROM (0x14e4, 0x4318, "BCM4318", "BCM4318 [AirForce One 54g] 802.11g Wireless LAN Controller", 0),
	// vendor ID is intentionally different
	PCI_ROM (0x14a4, 0x4318, "BCM4318", "Broadcom BCM4318 [AirForce One 54g] 802.11g WLAN Controller", 0),
	PCI_ROM (0x14e4, 0x4319, "BCM4318", "BCM4318 [AirForce 54g] 802.11a/b/g PCI Express Transceiver", 0),
	PCI_ROM (0x14e4, 0x4320, "BCM4306", "BCM4306 802.11b/g Wireless LAN Controller", 0),
	PCI_ROM (0x14e4, 0x4321, "BCM4321", "BCM4321 802.11a Wireless Network Controller", 0),
	PCI_ROM (0x14e4, 0x4322, "BCM4322", "BCM4322 802.11bgn Wireless Network Controller", 0),
	// this device id was intentionally base 10 (43222 == 0xa8d6)
	PCI_ROM (0x14e4, 0xa8d6, "BCM43222", "BCM43222 Wireless Network Adapter", 0),
	PCI_ROM (0x14e4, 0x4324, "BCM4309", "BCM4309 802.11abg Wireless Network Controller", 0),
	PCI_ROM (0x14e4, 0x4325, "BCM4306", "BCM4306 802.11bg Wireless Network Controller", 0),
	PCI_ROM (0x14e4, 0x4328, "BCM4321", "BCM4321 802.11a/b/g/n", 0),
	PCI_ROM (0x14e4, 0x4329, "BCM4321", "BCM4321 802.11b/g/n", 0),
	PCI_ROM (0x14e4, 0x432b, "BCM4322", "BCM4322 802.11a/b/g/n Wireless LAN Controller", 0),
	PCI_ROM (0x14e4, 0x432c, "BCM4322", "BCM4322 802.11b/g/n", 0),
	PCI_ROM (0x14e4, 0x4350, "BCM4322", "BCM43222 Wireless Network Adapter", 0),
	PCI_ROM (0x14e4, 0x4351, "BCM4322", "BCM43222 802.11abgn Wireless Network Adapter", 0),
};

static int  b43_probe(struct pci_device *pci)
{
  DBGC(pci, "b43_probe()\n");
  return ssb_pcihost_probe(pci);
}

static void b43_remove(struct pci_device *pci)
{
  DBGC(pci, "b43_remove()\n");
  ssb_pcihost_remove(pci);
}

struct pci_driver b43_pci_bridge_driver __pci_driver = {
  .ids = b43_pci_bridge_tbl,
  .id_count = (sizeof (b43_pci_bridge_tbl) / sizeof (b43_pci_bridge_tbl[0])),
  .probe = b43_probe,
  .remove = b43_remove,
};

REQUIRING_SYMBOL(b43_pci_bridge_driver);
/* note: the requiring symbol is not an arbitrary title, it refers to a
 * defined symbol, and in this case the variable above */
REQUIRE_OBJECT( b43_main );
/* need linkage on b43_main to link its __ssb_driver linker table
 * entries. To learn more, see include/ipxe/tables.h */
