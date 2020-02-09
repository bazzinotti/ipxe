/* Sonics Silicon Backplane
 * Probe layer for iPXE
 *
 * Copyright (c) 2020 Michael Bazzinotti <bazz@bazz1.com>
 * Original from Linux kernel 3.18.11
 */

#include <ipxe/ssb/ssb.h>
#include <ipxe/ssb/ssb_ipxe.h>

/**
 * Probe a SSB device
 *
 * @v ssb		SSB device
 * @ret rc		Return status code
 *
 * Searches for a driver for the SSB device.  If a driver is found,
 * its probe() routine is called.
 */
int ssb_probe ( struct ssb_device *ssb ) {
  int rc;

  DBGC(ssb, "ssb_probe()\n");

  DBGC ( ssb, SSB_FMT " has driver: %s\n",
        SSB_ARGS ( ssb ), ssb->dev->parent->name );
  DBGC ( ssb, SSB_FMT " has irq %d\n",
    SSB_ARGS ( ssb ), ssb->irq );

  if ( ( rc = ssb->drv->probe ( ssb, &ssb->id ) ) != 0 ) {
    DBGC ( ssb, SSB_FMT " probe failed: %s\n",
    SSB_ARGS ( ssb ), strerror ( rc ) );
    return rc;
  }

  return 0;
}

/**
 * Remove a SSB device
 *
 * @v ssb		SSB device
 */
void ssb_device_remove ( struct ssb_device *ssb ) {
  struct ssb_driver *ssb_drv = ssb->drv;

  DBGC ( ssb, "ssb_device_remove()\n");
  if (ssb_drv && ssb_drv->remove)
  {
    ssb_drv->remove(ssb);
    DBGC ( ssb, SSB_FMT " removed\n", SSB_ARGS ( ssb ) );
  }
  else {
    DBGC ( ssb, SSB_FMT " not removed!\n", SSB_ARGS ( ssb ) );
  }
}


/**
 * Read SSB device configuration
 *
 * @v ssb		SSB device
 * @ret rc		Return status code
 */
static int ssb_read_config ( struct ssb_device *ssb ) {
  // maybe fixme
  /* Initialise generic device component */
  ssb->dev->desc.bus_type = BUS_TYPE_SSB;
  ssb->dev->desc.location = 0;
  ssb->dev->desc.vendor = ssb->id.vendor;
  ssb->dev->desc.device = ssb->id.coreid;
  ssb->dev->desc.class = ssb->id.revision;
  ssb->dev->desc.ioaddr = 0;
  ssb->dev->desc.irq = ssb->irq;

  return 0;
}

static int ssb_match_devid(const struct ssb_device_id *tabid,
    const struct ssb_device_id *devid)
{
  if ((tabid->vendor != devid->vendor) &&
      tabid->vendor != SSB_ANY_VENDOR)
    return 0;
  if ((tabid->coreid != devid->coreid) &&
      tabid->coreid != SSB_ANY_ID)
    return 0;
  if ((tabid->revision != devid->revision) &&
      tabid->revision != SSB_ANY_REV)
    return 0;
  return 1;
}

static int ssb_bus_match(struct ssb_device *ssb_dev, struct ssb_driver *ssb_drv)
{
  const struct ssb_device_id *id;

  for (id = ssb_drv->ids;
      id->vendor || id->coreid || id->revision;
      id++) {
    if (ssb_match_devid(id, &ssb_dev->id))
      return 1; /* found */
  }

  return 0;
}

/**
 * Find driver for SSB device
 *
 * @v ssb		SSB device
 * @ret rc		Return status code
 */
int ssb_find_driver ( struct ssb_device *ssb ) {
  struct ssb_driver *driver;

  for_each_table_entry ( driver, SSB_DRIVERS ) {
    if (ssb_bus_match( ssb, driver) == 1) {
      ssb_read_config ( ssb );
      ssb_set_driver ( ssb, driver );
      return 0;
    }
  }
  return -ENOENT;
}
