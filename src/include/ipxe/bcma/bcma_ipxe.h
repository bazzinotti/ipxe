#ifndef IPXE_BCMA_IPXE_H
#define IPXE_BCMA_IPXE_H

#include <ipxe/tables.h>

int bcma_find_driver  (struct bcma_device *bcma);
int bcma_device_probe (struct bcma_device *bcma);
int bcma_device_remove(struct bcma_device *bcma);



/** BCMA device debug message format */
#define BCMA_FMT "%04x:%04x:%02x:%02x"
/** BCMA device debug message arguments */
/* needs to be updated for BCMA (bcma_SEG / SLOT don't exist)*/
#define BCMA_ARGS( bcma )	(bcma)->id.manuf, (bcma)->id.id, \
                          (bcma)->id.rev, (bcma)->id.class

// include/ipxe/BCMA.h
/** BCMA driver table */
#define BCMA_DRIVERS __table ( struct bcma_driver, "bcma_drivers" )

/** Declare a BCMA driver */
#define __bcma_driver __table_entry ( BCMA_DRIVERS, 01 )

/** Declare a fallback BCMA driver */
#define __bcma_driver_fallback __table_entry ( BCMA_DRIVERS, 02 )

static inline void bcma_set_driver (
            struct bcma_device *bcma, struct bcma_driver *driver)
{
	bcma->drv = driver;
	bcma->dev.driver_name = driver->name;
}


#endif /* IPXE_BCMA_IPXE_H */
