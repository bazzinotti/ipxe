#ifndef IPXE_SSB_IPXE_H
#define IPXE_SSB_IPXE_H

#include <ipxe/tables.h>
/* This file is to recreate the ipxe device model enough to serve as the
 * SSB architecture.
 *
 * SSB_ROM is used to build up entries in a struct ssb_device_id array.
 * However, I don't think it generates ROM entries quite like its PCI
 * brother. In fact, I have no experience ever needing to muck with PCI
 * "ROMs." On my setup, I get on just fine without any sort of support via
 * parserom.pl. As long as the object file where the __ssb_driver variable
 * is defined is linked in to the ipxe binary, we are golden. This is done
 * using REQUIRE_OBJECT. See the b43_pci_bridge.c driver file for a
 * running example. That way its table entries are included into the SSB
 * driver table for when the SSB bus does its device to driver matching.
 *
 * If you happen to know anything about PCI ROMs as it pertains to iPXE,
 * and whether or not SSB needs to make a change to support that, please
 * let me know. Thank you
 *
 */

/* Note: ssb_device_id does not have a name and description field, unlike
 * its iPXE PCI counterpart.  Count in that SSB without MIPS support is
 * exclusively a bridged bus. We can conveniently reference the
 * bridged device's name or description from the SSB driver, if a name or
 * description is needed for the device. (not the driver. There is a name
 * field for ssb_driver. See ssb_set_driver())
 **/

int ssb_probe ( struct ssb_device *ssb );
int ssb_find_driver ( struct ssb_device *ssb );
void ssb_device_remove ( struct ssb_device *ssb );

/* SSB_ID could be used to generate entries without creating a
 * corresponding ROM in the build process. */
#define SSB_ID(_vendor, _coreid, _revision)  \
{ .vendor = _vendor, .coreid = _coreid, .revision = _revision, }
/* Note: Currently SSB_ROM does not generate ROM entries. See above
 * comments */
#define SSB_ROM(_vendor, _coreid, _revision) \
  SSB_ID(_vendor, _coreid, _revision)

/* Native Linux wrap */
#define SSB_DEVICE(_vendor, _coreid, _revision) \
  SSB_ID(_vendor, _coreid, _revision)
#define SSB_DEVTABLE_END  \
{ 0, },

#define SSB_ANY_VENDOR 0xFFFF
#define SSB_ANY_ID     0xFFFF
#define SSB_ANY_REV    0xFF

/** SSB device debug message format */
#define SSB_FMT "%04x:%04x:%02x"
/** SSB device debug message arguments */
/* needs to be updated for SSB (SSB_SEG / SLOT don't exist)*/
#define SSB_ARGS( ssb )	(ssb)->id.vendor, (ssb)->id.coreid,	(ssb)->id.revision

// include/ipxe/ssb.h
/** SSB driver table */
#define SSB_DRIVERS __table ( struct ssb_driver, "ssb_drivers" )

/** Declare a SSB driver */
#define __ssb_driver __table_entry ( SSB_DRIVERS, 01 )

/** Declare a fallback SSB driver */
#define __ssb_driver_fallback __table_entry ( SSB_DRIVERS, 02 )

/**
 * Set SSB driver
 *
 * @v ssb		SSB device
 * @v driver		SSB driver
 * @v id		SSB device ID
 */
static inline void ssb_set_driver ( struct ssb_device *ssb,
    struct ssb_driver *driver) {
  ssb->drv = driver;
  ssb->dev->driver_name = driver->name;
}


#endif /* IPXE_SSB_IPXE_H */
