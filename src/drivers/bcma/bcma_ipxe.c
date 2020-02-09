/* Broadcom specific AMBA
 * Probe layer for iPXE
 *
 * Copyright (c) 2020 Michael Bazzinotti <bazz@bazz1.com>
 * Original from Linux kernel 3.18.11
 */

#include <ipxe/bcma/bcma.h>
#include <ipxe/bcma/bcma_ipxe.h>

static int bcma_bus_match(struct bcma_device *core, struct bcma_driver *adrv)
{
	const struct bcma_device_id *cid = &core->id;
	const struct bcma_device_id *did;

	for (did = adrv->ids; did->manuf || did->id || did->rev; did++) {
		if ((did->manuf == cid->manuf || did->manuf == BCMA_ANY_MANUF) &&
				(did->id == cid->id || did->id == BCMA_ANY_ID) &&
				(did->rev == cid->rev || did->rev == BCMA_ANY_REV) &&
				(did->class == cid->class || did->class == BCMA_ANY_CLASS))
			return 1;
	}
	return 0;
}

int bcma_device_probe(struct bcma_device *core)
{
	int err = 0;

	if (core->drv && core->drv->probe)
		err = core->drv->probe(core);

	return err;
}

int bcma_device_remove(struct bcma_device *core)
{
	if (core->drv && core->drv->remove)
		core->drv->remove(core);

	return 0;
}

static int bcma_read_config ( struct bcma_device *bcma ) {
	// maybe fixme
	/* Initialise generic device component */
	bcma->dev.desc.bus_type = BUS_TYPE_BCMA;
	bcma->dev.desc.vendor = bcma->id.manuf;
	bcma->dev.desc.device = bcma->id.id;
	bcma->dev.desc.location = bcma->id.rev;
	bcma->dev.desc.class = bcma->id.class;
	bcma->dev.desc.ioaddr = 0;
	bcma->dev.desc.irq = bcma->irq;

	return 0;
}

int bcma_find_driver ( struct bcma_device *bcma ) {
	struct bcma_driver *driver;

	for_each_table_entry ( driver, BCMA_DRIVERS ) {
		if (bcma_bus_match( bcma, driver) == 1) {
			bcma_read_config ( bcma );
			bcma_set_driver ( bcma, driver );
			return 0;
		}
	}
	return -ENOENT;
}
