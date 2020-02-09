/*
 * Broadcom specific AMBA
 * ChipCommon core driver
 *
 * Ported to iPXE
 * Copyright (c) 2020 Michael Bazzinotti <bazz@bazz1.com>
 * Original from Linux kernel 3.18.11
 *
 * Copyright 2005, Broadcom Corporation
 * Copyright 2006, 2007, Michael Buesch <m@bues.ch>
 * Copyright 2012, Hauke Mehrtens <hauke@hauke-m.de>
 *
 * Licensed under the GNU/GPL. See COPYING for details.
 */

#include "bcma_private.h"
#include <ipxe/bcma/bcma.h>

static inline u32 bcma_cc_write32_masked(struct bcma_drv_cc *cc, u16 offset,
					 u32 mask, u32 value)
{
	value &= mask;
	value |= bcma_cc_read32(cc, offset) & ~mask;
	bcma_cc_write32(cc, offset, value);

	return value;
}

u32 bcma_chipco_get_alp_clock(struct bcma_drv_cc *cc)
{
	if (cc->capabilities & BCMA_CC_CAP_PMU)
		return bcma_pmu_get_alp_clock(cc);

	return 20000000;
}

static int bcma_chipco_watchdog_ticks_per_ms(struct bcma_drv_cc *cc)
{
	struct bcma_bus *bus = cc->core->bus;

	if (cc->capabilities & BCMA_CC_CAP_PMU) {
		if (bus->chipinfo.id == BCMA_CHIP_ID_BCM4706)
			/* 4706 CC and PMU watchdogs are clocked at 1/4 of ALP clock */
			return bcma_chipco_get_alp_clock(cc) / 4000;
		else
			/* based on 32KHz ILP clock */
			return 32;
	} else {
		return bcma_chipco_get_alp_clock(cc) / 1000;
	}
}

void bcma_core_chipcommon_early_init(struct bcma_drv_cc *cc)
{
	if (cc->early_setup_done)
		return;

	if (cc->core->id.rev >= 11)
		cc->status = bcma_cc_read32(cc, BCMA_CC_CHIPSTAT);
	cc->capabilities = bcma_cc_read32(cc, BCMA_CC_CAP);
	if (cc->core->id.rev >= 35)
		cc->capabilities_ext = bcma_cc_read32(cc, BCMA_CC_CAP_EXT);

	if (cc->capabilities & BCMA_CC_CAP_PMU)
		bcma_pmu_early_init(cc);

	cc->early_setup_done = true;
}

void bcma_core_chipcommon_init(struct bcma_drv_cc *cc)
{
	u32 leddc_on = 10;
	u32 leddc_off = 90;

	if (cc->setup_done)
		return;

	bcma_core_chipcommon_early_init(cc);

	if (cc->core->id.rev >= 20) {
		u32 pullup = 0, pulldown = 0;

		if (cc->core->bus->chipinfo.id == BCMA_CHIP_ID_BCM43142) {
			pullup = 0x402e0;
			pulldown = 0x20500;
		}

		bcma_cc_write32(cc, BCMA_CC_GPIOPULLUP, pullup);
		bcma_cc_write32(cc, BCMA_CC_GPIOPULLDOWN, pulldown);
	}

	if (cc->capabilities & BCMA_CC_CAP_PMU)
		bcma_pmu_init(cc);
	if (cc->capabilities & BCMA_CC_CAP_PCTL)
		bcma_err(cc->core->bus, "Power control not implemented!\n");

	if (cc->core->id.rev >= 16) {
		if (cc->core->bus->sprom.leddc_on_time &&
		    cc->core->bus->sprom.leddc_off_time) {
			leddc_on = cc->core->bus->sprom.leddc_on_time;
			leddc_off = cc->core->bus->sprom.leddc_off_time;
		}
		bcma_cc_write32(cc, BCMA_CC_GPIOTIMER,
			((leddc_on << BCMA_CC_GPIOTIMER_ONTIME_SHIFT) |
			 (leddc_off << BCMA_CC_GPIOTIMER_OFFTIME_SHIFT)));
	}
	cc->ticks_per_ms = bcma_chipco_watchdog_ticks_per_ms(cc);

	cc->setup_done = true;
}


void bcma_chipco_irq_mask(struct bcma_drv_cc *cc, u32 mask, u32 value)
{
	bcma_cc_write32_masked(cc, BCMA_CC_IRQMASK, mask, value);
}

u32 bcma_chipco_irq_status(struct bcma_drv_cc *cc, u32 mask)
{
	return bcma_cc_read32(cc, BCMA_CC_IRQSTAT) & mask;
}

u32 bcma_chipco_gpio_in(struct bcma_drv_cc *cc, u32 mask)
{
	return bcma_cc_read32(cc, BCMA_CC_GPIOIN) & mask;
}

u32 bcma_chipco_gpio_out(struct bcma_drv_cc *cc, u32 mask, u32 value)
{
	u32 res;

	res = bcma_cc_write32_masked(cc, BCMA_CC_GPIOOUT, mask, value);

	return res;
}

u32 bcma_chipco_gpio_outen(struct bcma_drv_cc *cc, u32 mask, u32 value)
{
	u32 res;

	res = bcma_cc_write32_masked(cc, BCMA_CC_GPIOOUTEN, mask, value);

	return res;
}

/*
 * If the bit is set to 0, chipcommon controlls this GPIO,
 * if the bit is set to 1, it is used by some part of the chip and not our code.
 */
u32 bcma_chipco_gpio_control(struct bcma_drv_cc *cc, u32 mask, u32 value)
{
	u32 res;

	res = bcma_cc_write32_masked(cc, BCMA_CC_GPIOCTL, mask, value);

	return res;
}

u32 bcma_chipco_gpio_intmask(struct bcma_drv_cc *cc, u32 mask, u32 value)
{
	u32 res;

	res = bcma_cc_write32_masked(cc, BCMA_CC_GPIOIRQ, mask, value);

	return res;
}

u32 bcma_chipco_gpio_polarity(struct bcma_drv_cc *cc, u32 mask, u32 value)
{
	u32 res;

	res = bcma_cc_write32_masked(cc, BCMA_CC_GPIOPOL, mask, value);

	return res;
}

u32 bcma_chipco_gpio_pullup(struct bcma_drv_cc *cc, u32 mask, u32 value)
{
	u32 res;

	if (cc->core->id.rev < 20)
		return 0;

	res = bcma_cc_write32_masked(cc, BCMA_CC_GPIOPULLUP, mask, value);

	return res;
}

u32 bcma_chipco_gpio_pulldown(struct bcma_drv_cc *cc, u32 mask, u32 value)
{
	u32 res;

	if (cc->core->id.rev < 20)
		return 0;

	res = bcma_cc_write32_masked(cc, BCMA_CC_GPIOPULLDOWN, mask, value);

	return res;
}
