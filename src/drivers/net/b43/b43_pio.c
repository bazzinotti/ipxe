/*

  Broadcom B43 wireless driver

  PIO data transfer

  Ported to iPXE
  Copyright (c) 2018-2020 Michael Bazzinotti <bazz@bazz1.com>
  Original from Linux kernel 3.18.11

  Copyright (c) 2005-2008 Michael Buesch <m@bues.ch>

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; see the file COPYING.  If not, write to
  the Free Software Foundation, Inc., 51 Franklin Steet, Fifth Floor,
  Boston, MA 02110-1301, USA.

*/

#include "b43.h"
#include "b43_pio.h"
#include "b43_dma.h"
#include "b43_main.h"
#include "b43_xmit.h"

#include <ipxe/timer.h>


static u16 generate_cookie(struct b43_pio_txqueue *q,
			   struct b43_pio_txpacket *pack)
{
	u16 cookie;

	/* Use the upper 4 bits of the cookie as
	 * PIO controller ID and store the packet index number
	 * in the lower 12 bits.
	 * Note that the cookie must never be 0, as this
	 * is a special value used in RX path.
	 * It can also not be 0xFFFF because that is special
	 * for multicast frames.
	 */
	cookie = (((u16)q->index + 1) << 12);
	cookie |= pack->index;

	return cookie;
}

static
struct b43_pio_txqueue *parse_cookie(struct b43_wldev *dev,
				     u16 cookie,
				      struct b43_pio_txpacket **pack)
{
	struct b43_pio *pio = &dev->pio;
	struct b43_pio_txqueue *q = NULL;
	unsigned int pack_index;

	q = pio->tx_queue_AC_BE;

	if (B43_WARN_ON(!q))
		return NULL;
	pack_index = (cookie & 0x0FFF);
	if (B43_WARN_ON(pack_index >= ARRAY_SIZE(q->packets)))
		return NULL;
	*pack = &q->packets[pack_index];

	return q;
}

static u16 index_to_pioqueue_base(struct b43_wldev *dev,
				  unsigned int index)
{
	static const u16 bases[] = {
		B43_MMIO_PIO_BASE0,
		B43_MMIO_PIO_BASE1,
		B43_MMIO_PIO_BASE2,
		B43_MMIO_PIO_BASE3,
		B43_MMIO_PIO_BASE4,
		B43_MMIO_PIO_BASE5,
		B43_MMIO_PIO_BASE6,
		B43_MMIO_PIO_BASE7,
	};
	static const u16 bases_rev11[] = {
		B43_MMIO_PIO11_BASE0,
		B43_MMIO_PIO11_BASE1,
		B43_MMIO_PIO11_BASE2,
		B43_MMIO_PIO11_BASE3,
		B43_MMIO_PIO11_BASE4,
		B43_MMIO_PIO11_BASE5,
	};

	if (dev->dev->core_rev >= 11) {
		B43_WARN_ON(index >= ARRAY_SIZE(bases_rev11));
		return bases_rev11[index];
	}
	B43_WARN_ON(index >= ARRAY_SIZE(bases));
	return bases[index];
}

static u16 pio_txqueue_offset(struct b43_wldev *dev)
{
	if (dev->dev->core_rev >= 11)
		return 0x18;
	return 0;
}

static u16 pio_rxqueue_offset(struct b43_wldev *dev)
{
	if (dev->dev->core_rev >= 11)
		return 0x38;
	return 8;
}

static struct b43_pio_txqueue *b43_setup_pioqueue_tx(struct b43_wldev *dev,
						     unsigned int index)
{
	struct b43_pio_txqueue *q;
	struct b43_pio_txpacket *p;
	unsigned int i;

	q = kzalloc(sizeof(*q), GFP_KERNEL);
	if (!q)
		return NULL;
	q->dev = dev;
	q->rev = dev->dev->core_rev;
	q->mmio_base = index_to_pioqueue_base(dev, index) +
		       pio_txqueue_offset(dev);
	q->index = index;

	q->free_packet_slots = B43_PIO_MAX_NR_TXPACKETS;
	if (q->rev >= 8) {
		q->buffer_size = 1920; //FIXME this constant is wrong.
	} else {
		q->buffer_size = b43_piotx_read16(q, B43_PIO_TXQBUFSIZE);
		q->buffer_size -= 80;
	}

	INIT_LIST_HEAD(&q->packets_list);
	for (i = 0; i < ARRAY_SIZE(q->packets); i++) {
		p = &(q->packets[i]);
		INIT_LIST_HEAD(&p->list);
		p->index = i;
		p->queue = q;
		list_add(&p->list, &q->packets_list);
	}

	return q;
}

static struct b43_pio_rxqueue *b43_setup_pioqueue_rx(struct b43_wldev *dev,
						     unsigned int index)
{
	struct b43_pio_rxqueue *q;

	q = kzalloc(sizeof(*q), GFP_KERNEL);
	if (!q)
		return NULL;
	q->dev = dev;
	q->rev = dev->dev->core_rev;
	q->mmio_base = index_to_pioqueue_base(dev, index) +
		       pio_rxqueue_offset(dev);

	/* Enable Direct FIFO RX (PIO) on the engine. */
	b43_dma_direct_fifo_rx(dev, index, 1);

	return q;
}

static void b43_pio_cancel_tx_packets(struct net80211_device *ndev,
            struct b43_pio_txqueue *q)
{
	struct b43_pio_txpacket *pack;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(q->packets); i++) {
		pack = &(q->packets[i]);
		if (pack->iob) {
      net80211_tx_complete(ndev, pack->iob, 0, 0);
			pack->iob = NULL;
		}
	}
}

static void b43_destroy_pioqueue_tx(struct net80211_device *ndev,
            struct b43_pio_txqueue *q, const char *name __unused)
{
	if (!q)
		return;
	b43_pio_cancel_tx_packets(ndev, q);
	kfree(q);
}

static void b43_destroy_pioqueue_rx(struct b43_pio_rxqueue *q,
				    const char *name __unused)
{
	if (!q)
		return;
	kfree(q);
}

#define destroy_queue_tx(ndev, pio, queue) do {				\
	b43_destroy_pioqueue_tx(ndev, (pio)->queue, __stringify(queue));	\
	(pio)->queue = NULL;						\
  } while (0)

#define destroy_queue_rx(pio, queue) do {				\
	b43_destroy_pioqueue_rx((pio)->queue, __stringify(queue));	\
	(pio)->queue = NULL;						\
  } while (0)

void b43_pio_free(struct b43_wldev *dev)
{
	struct b43_pio *pio;
  struct net80211_device *ndev = b43_wldev_to_ndev(dev);

	if (!b43_using_pio_transfers(dev))
		return;
	pio = &dev->pio;

	destroy_queue_rx(pio, rx_queue);
	destroy_queue_tx(ndev, pio, tx_queue_AC_BE);
}

int b43_pio_init(struct b43_wldev *dev)
{
	struct b43_pio *pio = &dev->pio;
	struct net80211_device *ndev = b43_wldev_to_ndev(dev);
	int err = -ENOMEM;

	b43_write32(dev, B43_MMIO_MACCTL, b43_read32(dev, B43_MMIO_MACCTL)
		    & ~B43_MACCTL_BE);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_RXPADOFF, 0);

	pio->tx_queue_AC_BE = b43_setup_pioqueue_tx(dev, 1);
	if (!pio->tx_queue_AC_BE)
		goto out;

	pio->rx_queue = b43_setup_pioqueue_rx(dev, 0);
	if (!pio->rx_queue)
		goto err_destroy_be;

	b43dbg(dev->wl, "PIO initialized\n");
	err = 0;
out:
	return err;

err_destroy_be:
	destroy_queue_tx(ndev, pio, tx_queue_AC_BE);
	return err;
}

static u16 tx_write_2byte_queue(struct b43_pio_txqueue *q,
				u16 ctl,
				const void *_data,
				unsigned int data_len)
{
	struct b43_wldev *dev = q->dev;
	struct b43_wl *wl = dev->wl;
	const u8 *data = _data;

	ctl |= B43_PIO_TXCTL_WRITELO | B43_PIO_TXCTL_WRITEHI;
	b43_piotx_write16(q, B43_PIO_TXCTL, ctl);

	b43_block_write(dev, data, (data_len & ~1),
			q->mmio_base + B43_PIO_TXDATA,
			sizeof(u16));
	if (data_len & 1) {
		u8 *tail = wl->pio_tailspace;
		BUILD_BUG_ON(sizeof(wl->pio_tailspace) < 2);

		/* Write the last byte. */
		ctl &= ~B43_PIO_TXCTL_WRITEHI;
		b43_piotx_write16(q, B43_PIO_TXCTL, ctl);
		tail[0] = data[data_len - 1];
		tail[1] = 0;
		b43_block_write(dev, tail, 2,
				q->mmio_base + B43_PIO_TXDATA,
				sizeof(u16));
	}

	return ctl;
}

static void pio_tx_frame_2byte_queue(struct b43_pio_txpacket *pack,
				     const u8 *hdr, unsigned int hdrlen)
{
	struct b43_pio_txqueue *q = pack->queue;
	const char *frame = pack->iob->data;
	unsigned int frame_len = iob_len(pack->iob);
	u16 ctl;

	ctl = b43_piotx_read16(q, B43_PIO_TXCTL);
	ctl |= B43_PIO_TXCTL_FREADY;
	ctl &= ~B43_PIO_TXCTL_EOF;

	/* Transfer the header data. */
	ctl = tx_write_2byte_queue(q, ctl, hdr, hdrlen);
	/* Transfer the frame data. */
	ctl = tx_write_2byte_queue(q, ctl, frame, frame_len);

	ctl |= B43_PIO_TXCTL_EOF;
	b43_piotx_write16(q, B43_PIO_TXCTL, ctl);
}

static u32 tx_write_4byte_queue(struct b43_pio_txqueue *q,
				u32 ctl,
				const void *_data,
				unsigned int data_len)
{
	struct b43_wldev *dev = q->dev;
	struct b43_wl *wl = dev->wl;
	const u8 *data = _data;

	ctl |= B43_PIO8_TXCTL_0_7 | B43_PIO8_TXCTL_8_15 |
	       B43_PIO8_TXCTL_16_23 | B43_PIO8_TXCTL_24_31;
	b43_piotx_write32(q, B43_PIO8_TXCTL, ctl);

	b43_block_write(dev, data, (data_len & ~3),
			q->mmio_base + B43_PIO8_TXDATA,
			sizeof(u32));
	if (data_len & 3) {
		u8 *tail = wl->pio_tailspace;
		BUILD_BUG_ON(sizeof(wl->pio_tailspace) < 4);

		memset(tail, 0, 4);
		/* Write the last few bytes. */
		ctl &= ~(B43_PIO8_TXCTL_8_15 | B43_PIO8_TXCTL_16_23 |
			 B43_PIO8_TXCTL_24_31);
		switch (data_len & 3) {
		case 3:
			ctl |= B43_PIO8_TXCTL_16_23 | B43_PIO8_TXCTL_8_15;
			tail[0] = data[data_len - 3];
			tail[1] = data[data_len - 2];
			tail[2] = data[data_len - 1];
			break;
		case 2:
			ctl |= B43_PIO8_TXCTL_8_15;
			tail[0] = data[data_len - 2];
			tail[1] = data[data_len - 1];
			break;
		case 1:
			tail[0] = data[data_len - 1];
			break;
		}
		b43_piotx_write32(q, B43_PIO8_TXCTL, ctl);
		b43_block_write(dev, tail, 4,
				q->mmio_base + B43_PIO8_TXDATA,
				sizeof(u32));
	}

	return ctl;
}

static void pio_tx_frame_4byte_queue(struct b43_pio_txpacket *pack,
				     const u8 *hdr, unsigned int hdrlen)
{
	struct b43_pio_txqueue *q = pack->queue;
	const char *frame = pack->iob->data;
	unsigned int frame_len = iob_len(pack->iob);
	u32 ctl;

	ctl = b43_piotx_read32(q, B43_PIO8_TXCTL);
	ctl |= B43_PIO8_TXCTL_FREADY;
	ctl &= ~B43_PIO8_TXCTL_EOF;

	/* Transfer the header data. */
	ctl = tx_write_4byte_queue(q, ctl, hdr, hdrlen);
	/* Transfer the frame data. */
	ctl = tx_write_4byte_queue(q, ctl, frame, frame_len);

	ctl |= B43_PIO8_TXCTL_EOF;
	b43_piotx_write32(q, B43_PIO_TXCTL, ctl);
}

static int pio_tx_frame(struct b43_pio_txqueue *q,
			struct io_buffer *iob)
{
	struct b43_wldev *dev = q->dev;
	struct b43_wl *wl = dev->wl;
	struct b43_pio_txpacket *pack;
	u16 cookie;
	int err;
	unsigned int hdrlen;
	struct b43_txhdr *txhdr = (struct b43_txhdr *)wl->pio_scratchspace;

	//DBGC(q, "pio_tx_frame:\n");
	B43_WARN_ON(list_empty(&q->packets_list));
	pack = list_entry(q->packets_list.next,
			  struct b43_pio_txpacket, list);

	cookie = generate_cookie(q, pack);
	//DBGC(dev, "generated cookie = 0x%04X\n", cookie);
	hdrlen = b43_txhdr_size(dev);
	BUILD_BUG_ON(sizeof(wl->pio_scratchspace) < sizeof(struct b43_txhdr));
	B43_WARN_ON(sizeof(wl->pio_scratchspace) < hdrlen);
	err = b43_generate_txhdr(dev, (u8 *)txhdr, iob, cookie);
	if (err)
		return err;

	pack->iob = iob;
	if (q->rev >= 8)
		pio_tx_frame_4byte_queue(pack, (const u8 *)txhdr, hdrlen);
	else
		pio_tx_frame_2byte_queue(pack, (const u8 *)txhdr, hdrlen);

	// transmit frame thru ipxe stack!


	/* Remove it from the list of available packet slots.
	 * It will be put back when we receive the status report. */
	list_del(&pack->list);

	/* Update the queue statistics. */
	q->buffer_used += roundup(iob_len(iob) + hdrlen, 4);
	q->free_packet_slots -= 1;

	return 0;
}

int b43_pio_tx(struct b43_wldev *dev, struct io_buffer *iob)
{
	struct b43_pio_txqueue *q;
	unsigned int hdrlen, total_len;
	int err = 0;

	//DBGC(dev, "b43_pio_tx:\n");

	q = dev->pio.tx_queue_AC_BE;

	hdrlen = b43_txhdr_size(dev);
	total_len = roundup(iob_len(iob) + hdrlen, 4);

	if (unlikely(total_len > q->buffer_size)) {
		err = -ENOBUFS;
		b43dbg(dev->wl, "PIO: TX packet longer than queue.\n");
		goto out;
	}
	if (unlikely(q->free_packet_slots == 0)) {
		err = -ENOBUFS;
		b43warn(dev->wl, "PIO: TX packet overflow.\n");
		goto out;
	}
	B43_WARN_ON(q->buffer_used > q->buffer_size);

	if (total_len > (unsigned int)(q->buffer_size - q->buffer_used)) {
		/* Not enough memory on the queue. */
		err = -ENOBUFS;
		q->stopped = true;
		goto out;
	}

	err = pio_tx_frame(q, iob);
	if (unlikely(err)) {
		b43err(dev->wl, "PIO transmission failure\n");
		goto out;
	}

	B43_WARN_ON(q->buffer_used > q->buffer_size);
	if (((q->buffer_size - q->buffer_used) < roundup(2 + 2 + 6, 4)) ||
	    (q->free_packet_slots == 0)) {
		/* The queue is full. */
		q->stopped = true;
	}

out:
	return err;
}

void b43_pio_handle_txstatus(struct b43_wldev *dev,
           const struct b43_txstatus *status)
{
	struct b43_pio_txqueue *q;
	struct b43_pio_txpacket *pack = NULL;
	unsigned int total_len;
	bool frame_succeed;

	q = parse_cookie(dev, status->cookie, &pack);
	if (unlikely(!q))
		return;
	B43_WARN_ON(!pack);

	frame_succeed = b43_fill_txstatus_report(dev, status);

	total_len = iob_len(pack->iob) + b43_txhdr_size(dev);
	total_len = roundup(total_len, 4);
	q->buffer_used -= total_len;
	q->free_packet_slots += 1;

	net80211_tx_complete(b43_wldev_to_ndev(dev), pack->iob,
                       status->frame_count ? status->frame_count - 1 : 0,
	                     frame_succeed ? 0 : -EIO);
	pack->iob = NULL;

	list_add(&pack->list, &q->packets_list);

	if (q->stopped) {
		q->stopped = false;
	}
}

/* Returns whether we should fetch another frame. */
static bool pio_rx_frame(struct b43_pio_rxqueue *q)
{
	struct b43_wldev *dev = q->dev;
	struct b43_wl *wl = dev->wl;
	u16 len;
	u32 macstat = 0;
	unsigned int i, padding;
	struct io_buffer *iob;
	u8 *iobp;
	const char *err_msg = NULL;
	struct b43_rxhdr_fw4 *rxhdr =
		(struct b43_rxhdr_fw4 *)wl->pio_scratchspace;
	size_t rxhdr_size = sizeof(*rxhdr);

	BUILD_BUG_ON(sizeof(wl->pio_scratchspace) < sizeof(*rxhdr));
	switch (dev->fw.hdr_format) {
	case B43_FW_HDR_410:
	case B43_FW_HDR_351:
		rxhdr_size -= sizeof(rxhdr->format_598) -
			sizeof(rxhdr->format_351);
		break;
	case B43_FW_HDR_598:
		break;
	}
	memset(rxhdr, 0, rxhdr_size);

	/* Check if we have data and wait for it to get ready. */
	if (q->rev >= 8) {
		u32 ctl;

		ctl = b43_piorx_read32(q, B43_PIO8_RXCTL);
		if (!(ctl & B43_PIO8_RXCTL_FRAMERDY))
			return false;
		b43_piorx_write32(q, B43_PIO8_RXCTL,
				  B43_PIO8_RXCTL_FRAMERDY);
		for (i = 0; i < 10; i++) {
			ctl = b43_piorx_read32(q, B43_PIO8_RXCTL);
			if (ctl & B43_PIO8_RXCTL_DATARDY)
				goto data_ready;
			udelay(10);
		}
	} else {
		u16 ctl;

		ctl = b43_piorx_read16(q, B43_PIO_RXCTL);
		if (!(ctl & B43_PIO_RXCTL_FRAMERDY))
			return false;
		b43_piorx_write16(q, B43_PIO_RXCTL,
				  B43_PIO_RXCTL_FRAMERDY);
		for (i = 0; i < 10; i++) {
			ctl = b43_piorx_read16(q, B43_PIO_RXCTL);
			if (ctl & B43_PIO_RXCTL_DATARDY)
				goto data_ready;
			udelay(10);
		}
	}
	b43dbg(q->dev->wl, "PIO RX timed out\n");
	return true;
data_ready:

	/* Get the preamble (RX header) */
	if (q->rev >= 8) {
		b43_block_read(dev, rxhdr, rxhdr_size,
			       q->mmio_base + B43_PIO8_RXDATA,
			       sizeof(u32));
	} else {
		b43_block_read(dev, rxhdr, rxhdr_size,
			       q->mmio_base + B43_PIO_RXDATA,
			       sizeof(u16));
	}
	/* Sanity checks. */
	len = le16_to_cpu(rxhdr->frame_len);
	if (unlikely(len > 0x700)) {
		err_msg = "len > 0x700";
		goto rx_error;
	}
	if (unlikely(len == 0)) {
		err_msg = "len == 0";
		goto rx_error;
	}

	switch (dev->fw.hdr_format) {
	case B43_FW_HDR_598:
		macstat = le32_to_cpu(rxhdr->format_598.mac_status);
		break;
	case B43_FW_HDR_410:
	case B43_FW_HDR_351:
		macstat = le32_to_cpu(rxhdr->format_351.mac_status);
		break;
	}
	if (macstat & B43_RX_MAC_FCSERR) {
		if (!(q->dev->wl->filter_flags & FIF_FCSFAIL)) {
			/* Drop frames with failed FCS. */
			err_msg = "Frame FCS error";
			goto rx_error;
		}
	}

	/* We always pad 2 bytes, as that's what upstream code expects
	 * due to the RX-header being 30 bytes. In case the frame is
	 * unaligned, we pad another 2 bytes. */
	padding = (macstat & B43_RX_MAC_PADDING) ? 2 : 0;
	iob = alloc_iob(len + padding + 2);
	if (unlikely(!iob)) {
		err_msg = "Out of memory";
		goto rx_error;
	}
	iob_reserve(iob, 2);
	iob_put(iob, len + padding);
	iobp = (u8 *)iob->data; // used to work with the raw data

	if (q->rev >= 8) {
		b43_block_read(dev, iob->data + padding, (len & ~3),
			       q->mmio_base + B43_PIO8_RXDATA,
			       sizeof(u32));
		if (len & 3) {
			u8 *tail = wl->pio_tailspace;
			BUILD_BUG_ON(sizeof(wl->pio_tailspace) < 4);

			/* Read the last few bytes. */
			b43_block_read(dev, tail, 4,
				       q->mmio_base + B43_PIO8_RXDATA,
				       sizeof(u32));
			switch (len & 3) {
			case 3:
				iobp[len + padding - 3] = tail[0];
				iobp[len + padding - 2] = tail[1];
				iobp[len + padding - 1] = tail[2];
				break;
			case 2:
				iobp[len + padding - 2] = tail[0];
				iobp[len + padding - 1] = tail[1];
				break;
			case 1:
				iobp[len + padding - 1] = tail[0];
				break;
			}
		}
	} else {
		b43_block_read(dev, iob->data + padding, (len & ~1),
			       q->mmio_base + B43_PIO_RXDATA,
			       sizeof(u16));
		if (len & 1) {
			u8 *tail = wl->pio_tailspace;
			BUILD_BUG_ON(sizeof(wl->pio_tailspace) < 2);

			/* Read the last byte. */
			b43_block_read(dev, tail, 2,
				       q->mmio_base + B43_PIO_RXDATA,
				       sizeof(u16));
			iobp[len + padding - 1] = tail[0];
		}
	}

	//DBGC(q->dev->wl, "RX NO PROBLEM! ");
	b43_rx(q->dev, iob, rxhdr);

	return true;

rx_error:
	if (err_msg)
		b43dbg(q->dev->wl, "PIO RX error: %s, len=%d\n", err_msg, len);
	if (q->rev >= 8)
		b43_piorx_write32(q, B43_PIO8_RXCTL, B43_PIO8_RXCTL_DATARDY);
	else
		b43_piorx_write16(q, B43_PIO_RXCTL, B43_PIO_RXCTL_DATARDY);

	return true;
}

void b43_pio_rx(struct b43_pio_rxqueue *q)
{
	unsigned int count = 0;
	bool stop;

	while (1) {
		stop = (pio_rx_frame(q) == 0);
		if (stop)
			break;
		if (WARN_ON_ONCE(++count > 10000))
			break;
	}
}

static void b43_pio_tx_suspend_queue(struct b43_pio_txqueue *q)
{
	if (q->rev >= 8) {
		b43_piotx_write32(q, B43_PIO8_TXCTL,
				  b43_piotx_read32(q, B43_PIO8_TXCTL)
				  | B43_PIO8_TXCTL_SUSPREQ);
	} else {
		b43_piotx_write16(q, B43_PIO_TXCTL,
				  b43_piotx_read16(q, B43_PIO_TXCTL)
				  | B43_PIO_TXCTL_SUSPREQ);
	}
}

static void b43_pio_tx_resume_queue(struct b43_pio_txqueue *q)
{
	if (q->rev >= 8) {
		b43_piotx_write32(q, B43_PIO8_TXCTL,
				  b43_piotx_read32(q, B43_PIO8_TXCTL)
				  & ~B43_PIO8_TXCTL_SUSPREQ);
	} else {
		b43_piotx_write16(q, B43_PIO_TXCTL,
				  b43_piotx_read16(q, B43_PIO_TXCTL)
				  & ~B43_PIO_TXCTL_SUSPREQ);
	}
}

void b43_pio_tx_suspend(struct b43_wldev *dev)
{
	b43_power_saving_ctl_bits(dev, B43_PS_AWAKE);
	b43_pio_tx_suspend_queue(dev->pio.tx_queue_AC_BE);
}

void b43_pio_tx_resume(struct b43_wldev *dev)
{
	b43_pio_tx_resume_queue(dev->pio.tx_queue_AC_BE);
	b43_power_saving_ctl_bits(dev, 0);
}
