/*
 * Broadcom specific AMBA
 * Bus subsystem
 *
 * Ported to iPXE
 * Copyright (c) 2020 Michael Bazzinotti <bazz@bazz1.com>
 * Original from Linux kernel 3.18.11
 *
 * Licensed under the GNU/GPL. See COPYING for details.
 */

#include "bcma_private.h"
//#include <linux/module.h>
#include <ipxe/bcma/bcma.h>
// fixme ipxe
//#include <linux/of_address.h>

MODULE_DESCRIPTION("Broadcom's specific AMBA driver");
MODULE_LICENSE("GPL");

/* contains the number the next bus should get. */
static unsigned int bcma_bus_next_num = 0;


static u16 bcma_cc_core_id(struct bcma_bus *bus)
{
	if (bus->chipinfo.id == BCMA_CHIP_ID_BCM4706)
		return BCMA_CORE_4706_CHIPCOMMON;
	return BCMA_CORE_CHIPCOMMON;
}

struct bcma_device *bcma_find_core_unit(struct bcma_bus *bus, u16 coreid,
					u8 unit)
{
	struct bcma_device *core;

	list_for_each_entry(core, &bus->cores, list) {
		if (core->id.id == coreid && core->core_unit == unit)
			return core;
	}
	return NULL;
}

bool bcma_wait_value(struct bcma_device *core, u16 reg, u32 mask, u32 value,
		     int timeout)
{
	unsigned long deadline = jiffies + timeout;
	u32 val;

	do {
		val = bcma_read32(core, reg);
		if ((val & mask) == value)
			return true;
		mb();
		udelay(10);
	} while (!time_after_eq(jiffies, deadline));

	bcma_warn(core->bus, "Timeout waiting for register 0x%04X!\n", reg);

	return false;
}

static void bcma_release_core_dev(struct device *dev)
{
	struct bcma_device *core = container_of(dev, struct bcma_device, dev);
	if (core->io_addr)
		iounmap(core->io_addr);
	if (core->io_wrap)
		iounmap(core->io_wrap);
	kfree(core);
}

static void bcma_register_core(struct bcma_bus *bus, struct bcma_device *core)
{
	sprintf(core->dev.name, "bcma%d:%d", bus->num, core->core_index);

	switch (bus->hosttype) {
	case BCMA_HOSTTYPE_PCI:
		core->dev.parent = &bus->host_pci->dev;
		core->irq = bus->host_pci->irq;
		break;
	case BCMA_HOSTTYPE_SOC:
	case BCMA_HOSTTYPE_SDIO:
		break;
	}

	core->dev_registered = true;
}

static int bcma_register_devices(struct bcma_bus *bus)
{
	struct bcma_device *core;
	int err;

	list_for_each_entry(core, &bus->cores, list) {
		/* We support that cores ourself */
		switch (core->id.id) {
		case BCMA_CORE_4706_CHIPCOMMON:
		case BCMA_CORE_CHIPCOMMON:
		case BCMA_CORE_NS_CHIPCOMMON_B:
		case BCMA_CORE_PCI:
		case BCMA_CORE_PCIE:
		case BCMA_CORE_PCIE2:
		case BCMA_CORE_MIPS_74K:
		case BCMA_CORE_4706_MAC_GBIT_COMMON:
			continue;
		}

		/* Early cores were already registered */
		/* iPXE note: Since we don't support MIPS and its flash-enabled devices,
		 * no need for the early core init */

		/* Only first GMAC core on BCM4706 is connected and working */
		if (core->id.id == BCMA_CORE_4706_MAC_GBIT &&
		    core->core_unit > 0)
			continue;

		bcma_register_core(bus, core);

		/* Look for a driver */
		if ( ( err = bcma_find_driver ( core ) ) != 0 ) {
			DBGC ( core, BCMA_FMT " (%s) has no driver\n",
					BCMA_ARGS (core), core->dev.name);
			continue;
		}

		/* Probe the found driver */
		if ( ( err = bcma_device_probe ( core ) ) == 0 ) {
			/* bcma core successfully probed */
		} else {
			/* Not registered */
			bcma_err(bus, "Could not probe %s\n", core->dev.name);

			goto error;
		}
	}

	return 0;
error:
	bcma_err(bus, "Unregistering the bus\n");
	bcma_bus_unregister(bus);
	return err;
}

static void bcma_unregister_cores(struct bcma_bus *bus)
{
	struct bcma_device *core, *tmp;

	list_for_each_entry_safe(core, tmp, &bus->cores, list) {
		list_del(&core->list);
		if (core->dev_registered) {
			bcma_release_core_dev(&core->dev);
		}
	}
}

int bcma_bus_register(struct bcma_bus *bus)
{
	int err;
	struct bcma_device *core;

	bus->num = bcma_bus_next_num++;

	/* Scan for devices (cores) */
	err = bcma_bus_scan(bus);
	if (err) {
		bcma_err(bus, "Failed to scan: %d\n", err);
		return err;
	}

	/* Early init CC core */
	core = bcma_find_core(bus, bcma_cc_core_id(bus));
	if (core) {
		bus->drv_cc.core = core;
		bcma_core_chipcommon_early_init(&bus->drv_cc);
	}

	/* Cores providing flash access go before SPROM init */
	/* iPXE note: Since we don't support MIPS and its flash-enabled devices,
	 * no need for the early core init */

	/* Try to get SPROM */
	err = bcma_sprom_get(bus);
	if (err == -ENOENT) {
		bcma_err(bus, "No SPROM available\n");
	} else if (err)
		bcma_err(bus, "Failed to get SPROM: %d\n", err);

	/* Init CC core */
	core = bcma_find_core(bus, bcma_cc_core_id(bus));
	if (core) {
		bus->drv_cc.core = core;
		bcma_core_chipcommon_init(&bus->drv_cc);
	}

	/* Init CC core */
	core = bcma_find_core(bus, BCMA_CORE_NS_CHIPCOMMON_B);
	if (core) {
		bus->drv_cc_b.core = core;
		bcma_core_chipcommon_b_init(&bus->drv_cc_b);
	}

	/* Init MIPS core */
	core = bcma_find_core(bus, BCMA_CORE_MIPS_74K);
	if (core) {
		bcma_err(bus, "MIPS Core found but not supported!\n");
	}

	/* Init PCIE core */
	core = bcma_find_core_unit(bus, BCMA_CORE_PCIE, 0);
	if (core) {
		bus->drv_pci[0].core = core;
		bcma_core_pci_init(&bus->drv_pci[0]);
	}

	/* Init PCIE core */
	core = bcma_find_core_unit(bus, BCMA_CORE_PCIE, 1);
	if (core) {
		bus->drv_pci[1].core = core;
		bcma_core_pci_init(&bus->drv_pci[1]);
	}

	/* Init PCIe Gen 2 core */
	core = bcma_find_core_unit(bus, BCMA_CORE_PCIE2, 0);
	if (core) {
		bus->drv_pcie2.core = core;
		bcma_core_pcie2_init(&bus->drv_pcie2);
	}

	/* Init GBIT MAC COMMON core */
	core = bcma_find_core(bus, BCMA_CORE_4706_MAC_GBIT_COMMON);
	if (core) {
		bus->drv_gmac_cmn.core = core;
		bcma_core_gmac_cmn_init(&bus->drv_gmac_cmn);
	}

	/* Register found cores */
	if ( ( err =	bcma_register_devices(bus) ) == 0)
		bcma_info(bus, "Bus registered\n");

	return err;
}

void bcma_bus_unregister(struct bcma_bus *bus)
{
	struct bcma_device *cores[3];

	bcma_core_chipcommon_b_free(&bus->drv_cc_b);

	cores[0] = bcma_find_core(bus, BCMA_CORE_MIPS_74K);
	cores[1] = bcma_find_core(bus, BCMA_CORE_PCIE);
	cores[2] = bcma_find_core(bus, BCMA_CORE_4706_MAC_GBIT_COMMON);

	bcma_unregister_cores(bus);

	kfree(cores[2]);
	kfree(cores[1]);
	kfree(cores[0]);
}
