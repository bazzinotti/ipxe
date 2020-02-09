/*
 * Sonics Silicon Backplane
 * PCI Hostdevice wrapper
 *
 * Ported to iPXE
 * Copyright (c) 2020 Michael Bazzinotti <bazz@bazz1.com>
 * Original from Linux kernel 3.18.11
 *
 * Copyright (c) 2005 Martin Langer <martin-langer@gmx.de>
 * Copyright (c) 2005 Stefano Brivio <st3@riseup.net>
 * Copyright (c) 2005 Danny van Dyk <kugelfang@gentoo.org>
 * Copyright (c) 2005 Andreas Jaggi <andreas.jaggi@waterwave.ch>
 * Copyright (c) 2005-2007 Michael Buesch <m@bues.ch>
 *
 * Licensed under the GNU/GPL. See COPYING for details.
 */

#include <ipxe/pci.h>
#include <ipxe/ssb/ssb.h>


int ssb_pcihost_probe(struct pci_device *dev)
{
	struct ssb_bus *ssb;
	int err = -ENOMEM;
	u32 val;

  DBGC(dev, "ssb_pcihost_probe: ");

	ssb = kzalloc(sizeof(*ssb), GFP_KERNEL);
	if (!ssb)
		goto out;
	adjust_pci_device(dev);

	/* Disable the RETRY_TIMEOUT register (0x41) to keep
	 * PCI Tx retries from interfering with C3 CPU state */
	pci_read_config_dword(dev, 0x40, &val);
	if ((val & 0x0000ff00) != 0)
		pci_write_config_dword(dev, 0x40, val & 0xffff00ff);

	err = ssb_bus_pcibus_register(ssb, dev);
	if (err)
		goto err_pci_release_regions;

	pci_set_drvdata(dev, ssb);

out:
	return err;

err_pci_release_regions:
	kfree(ssb);
	return err;
}

/* Called from an SSB bridge driver's remove callback.
 * such as b43_pci_bridge.c */
void ssb_pcihost_remove(struct pci_device *dev)
{
	struct ssb_bus *ssb = pci_get_drvdata(dev);

	ssb_bus_unregister(ssb);
	kfree(ssb);
	pci_set_drvdata(dev, NULL);
}
