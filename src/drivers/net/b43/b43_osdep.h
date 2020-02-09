#ifndef B43_OSDEP_H
#define B43_OSDEP_H

#include <ipxe/timer.h>
#include <ipxe/net80211.h>
#include <byteswap.h>
#include <ctype.h>

// ipxe: migrated from linux-3.18.11 include/linux/pci_ids.h
#define PCI_VENDOR_ID_APPLE		0x106b
#define PCI_VENDOR_ID_BROADCOM		0x14e4
#define PCI_VENDOR_ID_ASUSTEK		0x1043
#define PCI_VENDOR_ID_DELL		0x1028
#define PCI_VENDOR_ID_HP		0x103c
#define PCI_VENDOR_ID_LINKSYS		0x1737
#define PCI_VENDOR_ID_MOTOROLA		0x1057

////////////////////////////////////////////////
#define typecheck(type,x) \
	({	type __dummy; \
	 typeof(x) __dummy2; \
	 (void)(&__dummy == &__dummy2); \
	 1; \
	 })

#define time_after(a,b)		\
	(typecheck(unsigned long, a) && \
	 typecheck(unsigned long, b) && \
	 ((long)((b) - (a)) < 0))
#define time_before(a,b)	time_after(b,a)
///////////////////////////////////////////////

#define kmalloc(num, flags) malloc(num)
#define __maybe_unused  __attribute__((unused))

// math funcs from ath.h
#define min(x, y) ({					\
		typeof(x) _min1 = (x);				\
		typeof(y) _min2 = (y);				\
		(void) (&_min1 == &_min2);			\
		_min1 < _min2 ? _min1 : _min2; })
#define max(x, y) ({					\
		typeof(x) _max1 = (x);				\
		typeof(y) _max2 = (y);				\
		(void) (&_max1 == &_max2);			\
		_max1 > _max2 ? _max1 : _max2; })
/*#define abs(x) ({					\
		long ret;				\
		if (sizeof(x) == sizeof(long)) {	\
		long __x = (x);			\
		ret = (__x < 0) ? -__x : __x;	\
		} else {				\
		int __x = (x);			\
		ret = (__x < 0) ? -__x : __x;	\
		}					\
		ret;					\
		})
*/
// the rest from include/linux/kernel.h linux 3.18.11
#define clamp(val, lo, hi) min((typeof(val))max(val, lo), hi)

#define min_t(type, x, y) ({			\
		type __min1 = (x);			\
		type __min2 = (y);			\
		__min1 < __min2 ? __min1: __min2; })

#define max_t(type, x, y) ({			\
		type __max1 = (x);			\
		type __max2 = (y);			\
		__max1 > __max2 ? __max1: __max2; })

#define clamp_t(type, val, lo, hi) min_t(type, max_t(type, val, lo), hi)

#define clamp_val(val, lo, hi) clamp_t(typeof(val), val, lo, hi)



#define __packed    __attribute__((__packed__))

// from linux: include/linux/dma-mapping.h:65
#define DMA_BIT_MASK(n)	(((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))

// from linux: include/linux/kernel.h
#define lower_32_bits(n) ((u32)(n))

//from Linux where?
// defined in ssb-osdeps.h
//#define likely(x) x
//#define unlikely(x) x
// BUILD_BUG_ON
//
// bazzy homebrew
typedef int atomic_t;
// all calls to atomic_set are not in logic expressions so this is OK
#define atomic_set(x,v) *(x) = v
#define atomic_read(x) *(x)
#define atomic_dec_and_test(x) (*(x)-=1,(*(x))==0)



//
///
//// linux/irqreturn.h
/**
 * enum irqreturn
 * @IRQ_NONE		interrupt was not from this device
 * @IRQ_HANDLED		interrupt was handled by this device
 * @IRQ_WAKE_THREAD	handler requests to wake the handler thread
 */
enum irqreturn {
  IRQ_NONE		= (0 << 0),
  IRQ_HANDLED		= (1 << 0),
  IRQ_WAKE_THREAD		= (1 << 1),
};

typedef enum irqreturn irqreturn_t;

// fixme this line can probably be removed!
#define IRQ_RETVAL(x)	((x) != IRQ_NONE)
////
///
//

#define SET_IEEE80211_PERM_ADDR(hwinfo, mac) memcpy ( hwinfo->hwaddr, mac, ETH_ALEN )

#define jiffies currticks()
#define HZ TICKS_PER_SEC
#define inc_timer_ms(timer, ms) (currticks() + (ms * TICKS_PER_MS))
#define msecs_to_jiffies(ms) (ms * TICKS_PER_MS)

#define cancel_delayed_work_sync(ptr) *(ptr) = 0

enum ieee80211_band {
	IEEE80211_BAND_2GHZ = NET80211_BAND_2GHZ,
	IEEE80211_BAND_5GHZ = NET80211_BAND_5GHZ,
};
//#define IEEE80211_BAND_2GHZ NET80211_BAND_2GHZ
//#define IEEE80211_BAND_5GHZ NET80211_BAND_5GHZ

// unsure why the following was included. So removed
//asm-generic/cache.h
//#define L1_CACHE_SHIFT		5
//#define L1_CACHE_BYTES		(1 << L1_CACHE_SHIFT)

/*
 * DMA SECTION *
 * */

#define dma_map_single(a,buf,c,d) virt_to_bus(buf)
#define dma_unmap_single(a,b,c,d)
//typedef unsigned long dma_addr_t;
// note: virt_to_bus returns unsigned long, same type as phydaddr_t
typedef physaddr_t dma_addr_t;

// I gathered this from studying Ath port
#define mmiowb() mb()

/*
 * END DMA SECTION
 * */

/*
 * IEEE80211
 * */

enum ieee80211_filter_flags {
	FIF_PROMISC_IN_BSS	= 1<<0,
	FIF_ALLMULTI		= 1<<1,
	FIF_FCSFAIL		= 1<<2,
	FIF_PLCPFAIL		= 1<<3,
	FIF_BCN_PRBRESP_PROMISC	= 1<<4,
	FIF_CONTROL		= 1<<5,
	FIF_OTHER_BSS		= 1<<6,
	FIF_PSPOLL		= 1<<7,
	FIF_PROBE_REQ		= 1<<8,
};

struct ieee80211_low_level_stats {
	unsigned int dot11ACKFailureCount;
	unsigned int dot11RTSFailureCount;
	unsigned int dot11FCSErrorCount;
	unsigned int dot11RTSSuccessCount;
};

// manually migrated definitions:
//#define ieee80211_free_txskb(hw, buf) free_iob(buf)

#define __le16 u16

#define ieee80211_hdr ieee80211_frame
#define ieee80211_rate b43_rate
// include/linux/ieee80211.h
// fixme ipxe : not sure if we need to account for FCS_LEN
#define FCS_LEN 4
// match the following definitions to ipxe's include/ipxe/ieee80211.h
#define IEEE80211_FTYPE_CTL IEEE80211_TYPE_CTRL
#define IEEE80211_FTYPE_DATA IEEE80211_TYPE_DATA
#define IEEE80211_FCTL_TODS IEEE80211_FC_TODS
#define IEEE80211_FCTL_FROMDS IEEE80211_FC_FROMDS
#define IEEE80211_FCTL_FTYPE IEEE80211_FC_TYPE
// IEEE80211_STYPE_CTS already matching


/* FROM linux: include/linux/ieee80211.h */


/**
 * ieee80211_has_a4 - check if IEEE80211_FCTL_TODS and IEEE80211_FCTL_FROMDS are set
 * @fc: frame control bytes in little-endian byteorder
 */
static inline int ieee80211_has_a4(__le16 fc)
{
  __le16 tmp = cpu_to_le16(IEEE80211_FCTL_TODS | IEEE80211_FCTL_FROMDS);
  return (fc & tmp) == tmp;
}

/**
 * ieee80211_is_ctl - check if type is IEEE80211_FTYPE_CTL
 * @fc: frame control bytes in little-endian byteorder
 */
static inline int ieee80211_is_ctl(__le16 fc)
{
  return (fc & cpu_to_le16(IEEE80211_FCTL_FTYPE)) ==
    cpu_to_le16(IEEE80211_FTYPE_CTL);
}

/**
 * ieee80211_is_data - check if type is IEEE80211_FTYPE_DATA
 * @fc: frame control bytes in little-endian byteorder
 */
static inline int ieee80211_is_data(__le16 fc)
{
  return (fc & cpu_to_le16(IEEE80211_FCTL_FTYPE)) ==
    cpu_to_le16(IEEE80211_FTYPE_DATA);
}


/* from linux: net/wireless/util.c */
static inline unsigned int ieee80211_hdrlen(u16 fc)
{
  unsigned int hdrlen = 24;

  if (ieee80211_is_data(fc)) {
    if (ieee80211_has_a4(fc))
      hdrlen = 30;
    goto out;
  }

  if (ieee80211_is_ctl(fc)) {
    /*
     * ACK and CTS are 10 bytes, all others 16. To see how
     * to get this condition consider
     *   subtype mask:   0b0000000011110000 (0x00F0)
     *   ACK subtype:    0b0000000011010000 (0x00D0)
     *   CTS subtype:    0b0000000011000000 (0x00C0)
     *   bits that matter:         ^^^      (0x00E0)
     *   value of those: 0b0000000011000000 (0x00C0)
     */
    if ((fc & cpu_to_le16(0x00E0)) == cpu_to_le16(0x00C0))
      hdrlen = 10;
    else
      hdrlen = 16;
  }
out:
  return hdrlen;
}

/*
 * END IEEE80211
 * */
// Imported from Linux kernel/timer.c only for b43_periodic_work_handler
static inline unsigned long round_jiffies_common(unsigned long j, bool force_up)
{
	int rem;
	unsigned long original = j;

	rem = j % HZ;

	/*
	 * If the target jiffie is just after a whole second (which can happen
	 * due to delays of the timer irq, long irq off times etc etc) then
	 * we should round down to the whole second, not up. Use 1/4th second
	 * as cutoff for this rounding as an extreme upper bound for this.
	 * But never round down if @force_up is set.
	 */
	if (rem < (HZ/4) && !force_up) /* round down */
		j = j - rem;
	else /* round up */
		j = j - rem + HZ;

	if (j <= jiffies) /* rounding ate our timeout entirely; */
		return original;
	return j;
}

static inline unsigned long round_jiffies_relative(unsigned long j)
{
	unsigned long j0 = currticks();
	return round_jiffies_common(j + j0, false) - j0;
}
static inline unsigned long round_jiffies(unsigned long j)
{
	return round_jiffies_common(j, false);
}

#include <ipxe/isqrt.h>
#define int_sqrt(u) isqrt(u)

#define ieee80211_channel net80211_channel




// from asm-generic/bitops/const_hweight.h
/*
 * Compile time versions of __arch_hweightN()
 */
#define __const_hweight8(w)		\
	((unsigned int)			\
	 ((!!((w) & (1ULL << 0))) +	\
		(!!((w) & (1ULL << 1))) +	\
		(!!((w) & (1ULL << 2))) +	\
		(!!((w) & (1ULL << 3))) +	\
		(!!((w) & (1ULL << 4))) +	\
		(!!((w) & (1ULL << 5))) +	\
		(!!((w) & (1ULL << 6))) +	\
		(!!((w) & (1ULL << 7)))))

#define __const_hweight16(w) (__const_hweight8(w)  + __const_hweight8((w)  >> 8 ))
#define hweight32(w) (__const_hweight16(w) + __const_hweight16((w) >> 16))

//linux/kernel.h
#define upper_32_bits(n) ((u32)(((n) >> 16) >> 16))

#define __stringify(x) _S1(x)

// linux/kernel.h
#define roundup(x, y) (					\
		{							\
		const typeof(y) __y = y;			\
		(((x) + (__y - 1)) / __y) * __y;		\
		}							\
)




typedef u8 __u8;
typedef s8 __s8;

#endif
