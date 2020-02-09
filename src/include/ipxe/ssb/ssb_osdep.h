/* SSB OS Compatability Layer for iPXE
 *
 * Copyright 2017-2020 Michael Bazzinotti <bazz@bazz1.com>
 *
 * These are great to get the project started. Definitely come back here
 * and cleanup the code by patching the definitions directly and removing
 * them altogether or don't because there's not enough time in life.
 */
#ifndef IPXE_SSB_OSDEP_H
#define IPXE_SSB_OSDEP_H

#include <stdbool.h>
#include <errno.h>

// b43 and bcma want BIT defined
#define BIT(nr)			(1UL << (nr))

#define MODULE_DESCRIPTION(x)
#define MODULE_LICENSE(x) //FILE_LICENCE
#define __init
#define __exit

typedef uint16_t    __le16;
typedef uint32_t    __le32;
typedef uint16_t    __be16;
typedef uint32_t    __be32;

//sprom.c
#define kstrtoul(a,b,c) strtoul(a,b,c)
//borrowed from ath driver
#define ___constant_swab16(x) ((uint16_t)(			\
      (((uint16_t)(x) & (uint16_t)0x00ffU) << 8) |		\
      (((uint16_t)(x) & (uint16_t)0xff00U) >> 8)))
#define ___constant_swab32(x) ((uint32_t)(			\
      (((uint32_t)(x) & (uint32_t)0x000000ffUL) << 24) |	\
      (((uint32_t)(x) & (uint32_t)0x0000ff00UL) <<  8) |	\
      (((uint32_t)(x) & (uint32_t)0x00ff0000UL) >>  8) |	\
      (((uint32_t)(x) & (uint32_t)0xff000000UL) >> 24)))
#define __swab16(x) ___constant_swab16(x)
#define __swab32(x) ___constant_swab32(x)
#define swab16 __swab16
#define swab32 __swab32

#define kzalloc(a,b) zalloc(a)
#define kcalloc(a,b,c) calloc(a,b)
#define kfree(a) free(a)

typedef struct mutex mutex;
#define mutex u8
#define mutex_init(x) do { } while (0)
#define mutex_lock(x) do { } while (0)
#define mutex_lock_interruptible(a) 0
#define mutex_unlock(a) do { } while (0)

// Linux compatibility
//fixme this actually doesn't work
typedef struct pci_device pci_dev;
typedef struct ssb_device_id ssb_id;
#define  PCI_STATUS_SIG_TARGET_ABORT	0x800 /* Set on target abort */

/**
 * pci_is_pcie - check if the PCI device is PCI Express capable
 * @dev: PCI device
 *
 * Returns: true if the PCI device is PCI Express capable, false otherwise.
 */
static inline bool pci_is_pcie(struct pci_device *dev)
{
  return pci_find_capability(dev, PCI_CAP_ID_EXP) == 0 ? false : true;
}

#define pci_enable_device(dev) adjust_pci_device(dev)

#define pci_iounmap(dev, mem) iounmap(mem)

#define mmiowb() mb()

#define __iomem
#define ioread8(x)     readb(x)
#define ioread16(x)    readw(x)
#define ioread32(x)    readl(x)
#define iowrite8(v,x)  writeb(v,x)
#define iowrite16(v,x) writew(v,x)
#define iowrite32(v,x) writel(v,x)

static inline void iowrite32_rep(void *addr, const void *src, int count)
{
	uint32_t *srcp = (uint32_t *)src;
	while (--count >= 0) {
		iowrite32(*srcp, (uint32_t *)addr);
		srcp++;
	}
}
static inline void iowrite16_rep(void *addr, const void *src, int count)
{
	uint16_t *srcp = (uint16_t *)src;
	while (--count >= 0) {
		iowrite16(*srcp, (uint16_t *)addr);
		srcp++;
	}
}
static inline void iowrite8_rep( void *addr, const void *src, int count)
{
	uint8_t *srcp = (uint8_t *)src;
	while (--count >= 0) {
		iowrite8(*srcp, (uint8_t *)addr);
		srcp++;
	}
}

static inline void ioread32_rep(void  *addr, void *dst, int count)
{
	uint32_t *dstp = (uint32_t *)dst;
	while (--count >= 0) {
		uint32_t data = ioread32((uint32_t *)addr);
		*dstp = data;
		dstp++;
	}
}
static inline void ioread16_rep(void  *addr, void *dst, int count)
{
	uint16_t *dstp = (uint16_t *)dst;
	while (--count >= 0) {
		uint16_t data = ioread16((uint16_t *)addr);
		*dstp = data;
		dstp++;
	}
}
static inline void ioread8_rep(void  *addr, void *dst, int count)
{
	uint8_t *dstp = (uint8_t *)dst;
	while (--count >= 0) {
		uint8_t data = ioread8((uint8_t *)addr);
		*dstp = data;
		dstp++;
	}
}

/* Linux->ipxe */
/* may need to go in main SSB header honey */
#define likely(x) (x)
#define unlikely(x) (x)
#define printk DBG
#define dump_stack() do {} while (0)

#define msleep mdelay

// this works only because all references use `err = device..`
#define device_create_file(dev, sprom) 0

// may want to put this in main ssb.h
// fixme impl these, at least WARN_ON

// include/linux/kern_levels.h
#define KERN_SOH /* ASCII Start Of Header */
#define KERN_EMERG	KERN_SOH "0"	/* system is unusable */
#define KERN_ALERT	KERN_SOH "1"	/* action must be taken immediately */
#define KERN_CRIT	KERN_SOH "2"	/* critical conditions */
#define KERN_ERR	KERN_SOH "3"	/* error conditions */
#define KERN_WARNING	KERN_SOH "4"	/* warning conditions */
#define KERN_NOTICE	KERN_SOH "5"	/* normal but significant condition */
#define KERN_INFO	KERN_SOH "6"	/* informational */
#define KERN_DEBUG	KERN_SOH "7"	/* debug-level messages */
#define KERN_DEFAULT	KERN_SOH "d"	/* the default kernel loglevel */
#define KERN_CONT	""


#define spin_lock(l)            do { } while (0)
#define spin_unlock(l)          do { } while (0)
#define spin_lock_irqsave(l,f) do { } while (0)
#define spin_unlock_irqrestore(l,f) do { } while (0)
#define spin_lock_init(s)       do { } while (0)
#define spin_trylock(l)         (1)
typedef int spinlock_t;
#define DEFINE_SPINLOCK(x) spinlock_t x

#define BUG() do { \
  printf("BUG: failure at %s:%d/%s()!\n", \
      __FILE__, __LINE__, __FUNCTION__); \
  while(1); \
} while (0)
#define BUG_ON(condition) do { if (condition) BUG(); } while (0)

#define WARNING() do { \
  printf("WARN: failure at %s:%d/%s()!\n", \
      __FILE__, __LINE__, __FUNCTION__); \
} while (0)

#define WARN_ON(condition) ({						\
  int __ret_warn_on = !!(condition);				\
  if (unlikely(__ret_warn_on))					\
     WARNING();						\
    unlikely(__ret_warn_on);					\
})

#define WARN_ON_ONCE(condition) WARN_ON(condition)

#define WARN(condition, format...) ({					\
    int __ret_warn_on = !!(condition);				\
    printk(format);						\
    unlikely(__ret_warn_on);					\
})

#define __force
// i have no idea the implications of doing this
#define __raw_readb readb
#define __raw_readw readw
#define __raw_readl readl
#define __raw_writeb writeb
#define __raw_writew writew
#define __raw_writel writel

#define __baligned(x) __attribute__ (( aligned (x) ))

#ifndef uninitialized_var
#define uninitialized_var(x) x = x
#endif

#define list_move_tail(x,y) do { list_del(x); list_add_tail(x,y); } while (0)

#define BUILD_BUG_ON_ZERO(e) (sizeof(struct{int: -!!(e); }))
#define BUILD_BUG_ON(e) ((void)BUILD_BUG_ON_ZERO(e))

#define DIV_ROUND_UP(n,d)	(((n) + (d) - 1) / (d))

#define pr_err(format...) DBG(format)
#define pr_warn(format...) DBG(format)
#define pr_info(format...) DBG(format)
#define pr_debug(format...) DBG(format)

#endif /* IPXE_SSB_OSDEP_H */
