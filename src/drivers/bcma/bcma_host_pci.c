/*
 * Broadcom specific AMBA
 * PCI Host
 *
 * Ported to iPXE
 * Copyright (c) 2020 Michael Bazzinotti <bazz@bazz1.com>
 * Original from Linux kernel 3.18.11
 *
 * Licensed under the GNU/GPL. See COPYING for details.
 */

#include "bcma_private.h"
#include <ipxe/bcma/bcma.h>
#include <ipxe/pci.h>

static void bcma_host_pci_switch_core(struct bcma_device *core)
{
	pci_write_config_dword(core->bus->host_pci, BCMA_PCI_BAR0_WIN,
			       core->addr);
	pci_write_config_dword(core->bus->host_pci, BCMA_PCI_BAR0_WIN2,
			       core->wrap);
	core->bus->mapped_core = core;
	bcma_debug(core->bus, "Switched to core: 0x%X\n", core->id.id);
}

/* Provides access to the requested core. Returns base offset that has to be
 * used. It makes use of fixed windows when possible. */
static u16 bcma_host_pci_provide_access_to_core(struct bcma_device *core)
{
	switch (core->id.id) {
	case BCMA_CORE_CHIPCOMMON:
		return 3 * BCMA_CORE_SIZE;
	case BCMA_CORE_PCIE:
		return 2 * BCMA_CORE_SIZE;
	}

	if (core->bus->mapped_core != core)
		bcma_host_pci_switch_core(core);
	return 0;
}

static u8 bcma_host_pci_read8(struct bcma_device *core, u16 offset)
{
	offset += bcma_host_pci_provide_access_to_core(core);
	return ioread8(core->bus->mmio + offset);
}

static u16 bcma_host_pci_read16(struct bcma_device *core, u16 offset)
{
	offset += bcma_host_pci_provide_access_to_core(core);
	return ioread16(core->bus->mmio + offset);
}

static u32 bcma_host_pci_read32(struct bcma_device *core, u16 offset)
{
	offset += bcma_host_pci_provide_access_to_core(core);
	return ioread32(core->bus->mmio + offset);
}

static void bcma_host_pci_write8(struct bcma_device *core, u16 offset,
				 u8 value)
{
	offset += bcma_host_pci_provide_access_to_core(core);
	iowrite8(value, core->bus->mmio + offset);
}

static void bcma_host_pci_write16(struct bcma_device *core, u16 offset,
				 u16 value)
{
	offset += bcma_host_pci_provide_access_to_core(core);
	iowrite16(value, core->bus->mmio + offset);
}

static void bcma_host_pci_write32(struct bcma_device *core, u16 offset,
				 u32 value)
{
	offset += bcma_host_pci_provide_access_to_core(core);
	iowrite32(value, core->bus->mmio + offset);
}

#ifdef CONFIG_BCMA_BLOCKIO
static void bcma_host_pci_block_read(struct bcma_device *core, void *buffer,
				     size_t count, u16 offset, u8 reg_width)
{
	void __iomem *addr = core->bus->mmio + offset;
	if (core->bus->mapped_core != core)
		bcma_host_pci_switch_core(core);
	switch (reg_width) {
	case sizeof(u8):
		ioread8_rep(addr, buffer, count);
		break;
	case sizeof(u16):
		WARN_ON(count & 1);
		ioread16_rep(addr, buffer, count >> 1);
		break;
	case sizeof(u32):
		WARN_ON(count & 3);
		ioread32_rep(addr, buffer, count >> 2);
		break;
	default:
		WARN_ON(1);
	}
}

static void bcma_host_pci_block_write(struct bcma_device *core,
				      const void *buffer, size_t count,
				      u16 offset, u8 reg_width)
{
	void __iomem *addr = core->bus->mmio + offset;
	if (core->bus->mapped_core != core)
		bcma_host_pci_switch_core(core);
	switch (reg_width) {
	case sizeof(u8):
		iowrite8_rep(addr, buffer, count);
		break;
	case sizeof(u16):
		WARN_ON(count & 1);
		iowrite16_rep(addr, buffer, count >> 1);
		break;
	case sizeof(u32):
		WARN_ON(count & 3);
		iowrite32_rep(addr, buffer, count >> 2);
		break;
	default:
		WARN_ON(1);
	}
}
#endif

static u32 bcma_host_pci_aread32(struct bcma_device *core, u16 offset)
{
	if (core->bus->mapped_core != core)
		bcma_host_pci_switch_core(core);
	return ioread32(core->bus->mmio + (1 * BCMA_CORE_SIZE) + offset);
}

static void bcma_host_pci_awrite32(struct bcma_device *core, u16 offset,
				  u32 value)
{
	if (core->bus->mapped_core != core)
		bcma_host_pci_switch_core(core);
	iowrite32(value, core->bus->mmio + (1 * BCMA_CORE_SIZE) + offset);
}

static const struct bcma_host_ops bcma_host_pci_ops = {
	.read8		= bcma_host_pci_read8,
	.read16		= bcma_host_pci_read16,
	.read32		= bcma_host_pci_read32,
	.write8		= bcma_host_pci_write8,
	.write16	= bcma_host_pci_write16,
	.write32	= bcma_host_pci_write32,
#ifdef CONFIG_BCMA_BLOCKIO
	.block_read	= bcma_host_pci_block_read,
	.block_write	= bcma_host_pci_block_write,
#endif
	.aread32	= bcma_host_pci_aread32,
	.awrite32	= bcma_host_pci_awrite32,
};

static int bcma_host_pci_probe(struct pci_device *dev)
{
	struct bcma_bus *bus;
	int err = -ENOMEM;
	u32 val;
	unsigned long reg_base, reg_size;
	/* Alloc */
	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus)
		goto out;

	/* Basic PCI configuration */
	adjust_pci_device(dev);

	/* Disable the RETRY_TIMEOUT register (0x41) to keep
	 * PCI Tx retries from interfering with C3 CPU state */
	pci_read_config_dword(dev, 0x40, &val);
	if ((val & 0x0000ff00) != 0)
		pci_write_config_dword(dev, 0x40, val & 0xffff00ff);

	/* SSB needed additional powering up, do we have any AMBA PCI cards? */
	if (!pci_is_pcie(dev)) {
		bcma_err(bus, "PCI card detected, they are not supported.\n");
		err = -ENXIO;
		goto err_pci_release_regions;
	}

	/* Map MMIO */
	err = -ENOMEM;
	reg_base = pci_bar_start(dev, PCI_BASE_ADDRESS_0);
	reg_size = pci_bar_size (dev, PCI_BASE_ADDRESS_0);
	bus->mmio = ioremap(reg_base, reg_size);
	if (!bus->mmio)
		goto err_pci_release_regions;

	/* Host specific */
	bus->host_pci = dev;
	bus->hosttype = BCMA_HOSTTYPE_PCI;
	bus->ops = &bcma_host_pci_ops;

	pci_read_config_word(dev, PCI_SUBSYSTEM_VENDOR_ID, &bus->boardinfo.vendor);
	pci_read_config_word(dev, PCI_SUBSYSTEM_ID, &bus->boardinfo.type);

	/* Initialize struct, detect chip */
	bcma_init_bus(bus);

	/* Register */
	err = bcma_bus_register(bus);
	if (err)
		goto err_pci_unmap_mmio;

	pci_set_drvdata(dev, bus);

out:
	return err;

err_pci_unmap_mmio:
	pci_iounmap(dev, bus->mmio);
err_pci_release_regions:
	kfree(bus);
	return err;
}

static void bcma_host_pci_remove(struct pci_device *dev)
{
	struct bcma_bus *bus = pci_get_drvdata(dev);

	bcma_bus_unregister(bus);
	iounmap(bus->mmio);
	kfree(bus);
}

/* we can't use definitions or decimal in the PCI table. it messes up
 * util/parse_roms. Use hex codes
 *
 * names and descriptions derived from The PCI ID Repository
 * https://pci-ids.ucw.cz */
static struct pci_device_id bcma_pci_bridge_tbl[] = {
	PCI_ROM (0x14e4, 0x0576, "BCM43224", "BCM43224 802.11a/b/g/n", 0),
	PCI_ROM (0x14e4, 0x4313, "BCM4311",  "BCM4311 802.11a",        0),
	PCI_ROM (0x14e4, 0xa8d8, "BCM43224/5", "BCM43224/5 Wireless Network Adapter", 0),
	PCI_ROM (0x14e4, 0x4331, "BCM4331",  "BCM4331 802.11a/b/g/n",  0),
	PCI_ROM (0x14e4, 0x4353, "BCM43224", "BCM43224 802.11a/b/g/n", 0),
	PCI_ROM (0x14e4, 0x4357, "BCM43225", "BCM43225 802.11b/g/n",   0),
	PCI_ROM (0x14e4, 0x4358, "BCM43227", "BCM43227 802.11b/g/n",   0),
	PCI_ROM (0x14e4, 0x4359, "BCM43228", "BCM43228 802.11a/b/g/n", 0),
	PCI_ROM (0x14e4, 0x4365, "BCM43142", "BCM43142 802.11b/g/n",   0),
	PCI_ROM (0x14e4, 0x43a9, "BCM43217", "BCM43217 802.11b/g/n",   0),
	PCI_ROM (0x14e4, 0x43aa, "BCM43131", "BCM43131 802.11b/g/n",   0),
	PCI_ROM (0x14e4, 0x4727, "BCM4313",  "BCM4313 802.11bgn Wireless Network Adapter", 0),

	/* The following two entries were not in the PCI ID repo. I used the
	 * following links to determine what names to use:
	 *
	 * https://wireless.wiki.kernel.org/en/users/Drivers/b43?highlight=%28b43%29#Supported_devices
	 * a8db listed as BCM43217
	 */

	PCI_ROM (0x14e4, 0xa8db, "BCM43217", "BCM43217 802.11bgn Wireless Network Adapter", 0),

	/* https://openwrt.org/toh/arcadyan/ar7516 (OEM boot log uses pci id
	 * a8dc, chip listed as BCM43227 */

	PCI_ROM (0x14e4, 0xa8dc, "BCM43227", "BCM43227 802.11bgn Wireless Network Adapter", 0),
};

struct pci_driver bcma_pci_bridge_driver __pci_driver = {
	.ids = bcma_pci_bridge_tbl,
	.id_count = (sizeof (bcma_pci_bridge_tbl) / sizeof (bcma_pci_bridge_tbl[0])),
	.probe = bcma_host_pci_probe,
	.remove = bcma_host_pci_remove,
};
