// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Faraday FTMAC100 10/100 Ethernet
 *
 * (C) Copyright 2009-2011 Faraday Technology
 * Po-Yu Chuang <ratbert@faraday-tech.com>
 */

#define LOG_MODULE_NAME eth_atfmac100
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <net/ethernet.h>
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <device.h>
#include <soc.h>
#include <ethernet/eth_stats.h>
#include "eth_andes_atfmac100_priv.h"

#define DT_DRV_COMPAT	andestech_atfmac100

#if defined(CONFIG_CACHE_ENABLE) && !defined(CONFIG_SOC_ANDES_V5_CM)
#ifdef CONFIG_NOCACHE_MEMORY
__nocache unsigned char txd_orig_buffer[sizeof(struct tx_desc) * MACD_DESCRIPTOR_SLOTS + 16] = {0x01};
__nocache unsigned char rxd_orig_buffer[sizeof(struct rx_desc) * MACD_DESCRIPTOR_SLOTS + 16] = {0x01};
__nocache unsigned char rx_pool[2048 * MACD_DESCRIPTOR_SLOTS + 4] = {0x01};
__nocache unsigned char tx_pool[2048 * MACD_DESCRIPTOR_SLOTS + 4] = {0x01};
#else
#error "You have to disable cache or enable nocache memory!!"
#endif
#else /* defined(CONFIG_CACHE_ENABLE) && !defined(CONFIG_SOC_ANDES_V5_CM) */
unsigned char txd_orig_buffer[sizeof(struct tx_desc) * MACD_DESCRIPTOR_SLOTS + 16] = {0x01};
unsigned char rxd_orig_buffer[sizeof(struct rx_desc) * MACD_DESCRIPTOR_SLOTS + 16] = {0x01};
unsigned char rx_pool[2048 * MACD_DESCRIPTOR_SLOTS + 4] = {0x01};
unsigned char tx_pool[2048 * MACD_DESCRIPTOR_SLOTS + 4] = {0x01};
#endif /* defined(CONFIG_CACHE_ENABLE) && !defined(CONFIG_SOC_ANDES_V5_CM) */

#ifndef CONFIG_ANDES_ATFMAC100_TX_POLLING_MODE
static void eth_enable_tx(const struct device *dev, int enable);
#endif

static void eth_assign_mac(const struct device *dev)
{
	uint8_t mac_addr[6] = DT_INST_PROP(0, local_mac_address);
	uint32_t value = 0x0;

	value |= mac_addr[0] << 8;
	value |= mac_addr[1];
	OUTWORD(MAC_MACMADR(dev), value);

	value = 0x0;
	value |= mac_addr[2] << 24;
	value |= mac_addr[3] << 16;
	value |= mac_addr[4] << 8;
	value |= mac_addr[5];
	OUTWORD(MAC_MACLADR(dev), value);
}

static int eth_atfmac100_send(const struct device *dev, struct net_pkt *pkt)
{
	struct eth_atfmac100_data *const dev_data = DEV_ETH_DATA(dev);
	uint32_t core_intl, total_len, last_len;
	struct tx_desc *first_txd, *cur_txd, *last_txd;
	struct net_buf *frag;

	int buf_len = 0;
	last_len = 0;

	core_intl = irq_lock();

	first_txd = dev_data->txd_iter;
	cur_txd = first_txd;
	last_txd = first_txd;

	buf_len = dev_data->mtu;
	total_len = net_pkt_get_len(pkt);
	

	frag = pkt->frags;
	cur_txd = first_txd;
	while(frag)
	{
		frag = frag->frags;
		cur_txd = (struct tx_desc *)(unsigned long)cur_txd->next;
	}
	dev_data->txd_iter = cur_txd;

	irq_unlock(core_intl);

	for (frag = pkt->frags, cur_txd = first_txd; frag; frag = frag->frags, cur_txd = (struct tx_desc *)(unsigned long)cur_txd->next)
	{
		MEMCPY((char *)(unsigned long)cur_txd->txb, frag->data , frag->len);

		MACD_TXD_SET_TXB_SIZE(cur_txd, frag->len);
		MACD_TXD_CLR_FTS(cur_txd);
		MACD_TXD_CLR_LTS(cur_txd);
		last_txd = cur_txd;
		last_len = frag->len;
	}

	if (total_len < 60)
	{
		memset((char *)(unsigned long)last_txd->txb + last_len, '\0', 60 - total_len);
		MACD_TXD_SET_TXB_SIZE(last_txd, last_len + 60 - total_len);
	}

	MACD_TXD_SET_FTS(first_txd, 0, 1);
	MACD_TXD_SET_LTS(last_txd);

	/*
	 * When the frame contains more than one txd, accroding to the spec,
	 * the first txd needs to be set last. So we begin with the second txd,
	 * and after all other txd are prepared, then back to the first one.
	 */
	if (first_txd != last_txd)
	{
		/* TXDES0: TXPKT_LATECOL, TXPKT_EXCOL, TXDMA_OWN */
		for (cur_txd = (struct tx_desc *)(unsigned long)first_txd->next; cur_txd != (struct tx_desc *)(unsigned long)last_txd->next; cur_txd = (struct tx_desc *)(unsigned long)cur_txd->next)
			MACD_TXD_SET_DMA_OWN(cur_txd);
	}
	MACD_TXD_SET_DMA_OWN(first_txd);

#ifndef CONFIG_ANDES_ATFMAC100_TX_POLLING_MODE
	/* TXDES0: TXPKT_LATECOL, TXPKT_EXCOL, TXDMA_OWN */

	if (k_sem_take(&dev_data->tx_sem, K_MSEC(1000)) != 0)
	{
		return DRIVER_ERROR_TIMEOUT;
	}

	eth_enable_tx(dev, 1);
#endif

	return DRIVER_OK;
}

static struct net_pkt *eth_atfmac100_rx_pkt(const struct device *dev, struct net_if *iface)
{
	struct eth_atfmac100_data *const dev_data = DEV_ETH_DATA(dev);
	struct rx_desc *done_iter = dev_data->rxd_done_iter;
	struct net_pkt *pkt;
	int len;
	int count = 0;

	if (MACD_RXD_PROBE_DMA_OWN(done_iter))
	{
		return NULL;
	}
	else if(!MACD_RXD_PROBE_FRS(done_iter))
	{
		while(1);
	}

	len = MACD_RXD_GET_RFL(done_iter);
	pkt = net_pkt_rx_alloc_with_buffer(iface, len, AF_UNSPEC, 0, K_NO_WAIT);

	if (!pkt)
	{
		for (count = 0; (count < 8); count++) {
			MACD_RXD_SET_DMA_OWN(done_iter);
			dev_data->rxd_done_iter = (struct rx_desc *)(unsigned long)done_iter->next;
		}
		return (struct net_pkt *)(-ENOMEM);
	}

	unsigned char *src = (unsigned char *)(unsigned long)done_iter->rxb;
	net_pkt_write(pkt, src, len);

	MACD_RXD_SET_DMA_OWN(done_iter);
	dev_data->rxd_done_iter = (struct rx_desc *)(unsigned long)done_iter->next;

	return pkt;
}

static int eth_atfmac100_receive(const struct device *dev)
{
	struct eth_atfmac100_data const *dev_data = DEV_ETH_DATA(dev);
	struct net_if *iface = dev_data->iface;
	struct net_pkt *pkt;
	static int err_num = 0;
	int pkt_count = 0;

	while(pkt_count < MACD_DESCRIPTOR_SLOTS)
	{
		pkt = eth_atfmac100_rx_pkt(dev, iface);

		if (!pkt)
		{
			pkt_count = MACD_DESCRIPTOR_SLOTS;
			continue;
		}
		else if(pkt == (struct net_pkt *)(-ENOMEM))
		{
			pkt_count = MACD_DESCRIPTOR_SLOTS;
			err_num++;
			NET_INFO("There is no tcp rx buffer to store rx packets from mac buffer\n");
			continue;
		}
		else
		{
			err_num = 0;
		}

		if (net_recv_data(iface, pkt) < 0)
		{
			net_pkt_unref(pkt);
			goto err_mem;
		}
	}

	return DRIVER_OK;

err_mem:
	return DRIVER_ERROR;
}

void eth_hw_init(const struct device *dev)
{
	struct eth_atfmac100_data *const dev_data = DEV_ETH_DATA(dev);
	uint32_t reg = 0;
	unsigned long tmp = 0;
	volatile int count = 0;

	/* disable tx & rx */
	reg = INWORD(MAC_MACCR(dev));
	reg &= ~(MAC_MACCR_XMT_EN | MAC_MACCR_RCV_EN);
	OUTWORD(MAC_MACCR(dev), reg);

	reg = INWORD(MAC_IME(dev));
	reg &= ~(MAC_IME_NOTXBUF_EN | MAC_IME_XPKT_OK_EN | MAC_IME_XPKT_LOST_EN | MAC_IME_RPKT_FINISH_EN | MAC_IME_NORXBUF_EN | MAC_IME_RPKT_LOST_EN);
	OUTWORD(MAC_IME(dev), reg);

	/* SW-Reset MAC Controller (require 64 AHB cycles) */	

	reg = INWORD(MAC_MACCR(dev));
	reg |= (MAC_MACCR_SW_RST);
	OUTWORD(MAC_MACCR(dev), reg);

	while (count++ < 100);

	/* Assign MAC address to Hardware */
	eth_assign_mac(dev);

	OUTWORD(MAC_MAHT0(dev), 0);
	OUTWORD(MAC_MAHT1(dev), 0);

	/* MAC - IME (Interrupt Enable Register) */
#ifndef CONFIG_ANDES_ATFMAC100_TX_POLLING_MODE
	reg = (MAC_IME_RPKT_FINISH_EN | MAC_IME_XPKT_OK_EN | MAC_IME_PHYSTS_CHG_EN);
#else
	reg = (MAC_IME_RPKT_FINISH_EN | MAC_IME_PHYSTS_CHG_EN);
#endif
	reg &= ~( MAC_IME_XPKT_FINISH_EN
		| MAC_IME_NOTXBUF_EN
		| MAC_IME_XPKT_LOST_EN
		| MAC_IME_RPKT_SAV_EN
		| MAC_IME_RPKT_LOST_EN
		| MAC_IME_AHB_ERR_EN);
	OUTWORD(MAC_IME(dev), reg);

	/*
	 * MAC - ITC (Interrupt Timer Control)
	 * Current Setting: 0x00001010 (spec suggested: 0x00001010)
	 *   rx - disabled wait time, 3 queued int, rx cycle 5.12/51.2us, 0 rx cycle timeout before int
	 *   tx - disabled wait time, 0 queued int, tx cycle 5.12/51.2us, 0 tx cycle timeout before int
	 */
	reg = ((0 << MAC_ITC_RXINT_CNT_SHIFT)
		| (3 << MAC_ITC_RXINT_THR_SHIFT)
		| (8 << MAC_ITC_RXINT_TIME_SEL_BIT)
		| (0 << MAC_ITC_TXINT_CNT_SHIFT)
		| (0 << MAC_ITC_TXINT_THR_SHIFT)
		| (0 << MAC_ITC_TXINT_TIME_SEL_BIT));
	OUTWORD(MAC_ITC(dev), reg);

	/*
	 * MAC - DBLAC (DMA Burst Length and Arbitration Control, 0x30)
	 * Current Setting: 0x00000390 (spec suggested: 0x00000390)
	 */
	reg = ((MAC_FIFO_512 << MAC_DBLAC_RXFIFO_LTHR_SHIFT) | (MAC_FIFO_1024 << MAC_DBLAC_RXFIFO_HTHR_SHIFT) | (0 << MAC_DBLAC_INCR_SEL_SHIFT) | MAC_DBLAC_RX_THR_EN);
	reg &= ~(MAC_DBLAC_INCR4_EN | MAC_DBLAC_INCR8_EN | MAC_DBLAC_INCR16_EN);
	OUTWORD(MAC_DBLAC(dev), reg);

	/* MAC - FCR (Flow Control Register) */
	reg = ((4 << MAC_FCR_FC_HIGH_SHIFT) | (4 << MAC_FCR_FC_LOW_SHIFT) | (0 << MAC_FCR_PAUSE_TIME_SHIFT));
	reg &= ~(MAC_FCR_FCTHR_EN | MAC_FCR_FC_EN);
	OUTWORD(MAC_FCR(dev), reg);

	/* MAC - BPR (Back Pressure) */
	reg = ((0 << MAC_BPR_BKJAM_LEN_SHIFT) | (4 << MAC_BPR_BK_LOW_SHIFT) | (0 << MAC_BPR_BK_MODE_BIT));
	reg &= ~(MAC_BPR_BK_EN);
	OUTWORD(MAC_BPR(dev), reg);

	/* MAC - WOL (Wake On Lan) */
	OUTWORD(MAC_WOLCR(dev), 0);

	/* MAC - Transmit Ring Base Address */
	tmp = (unsigned long)(dev_data->txd_base);
	reg = 0xffffffff & (uint32_t)tmp;
	OUTWORD(MAC_TXRBADR(dev), reg);

	/* MAC - Receive Ring Base Address */
	tmp = (unsigned long)(dev_data->rxd_base);
	reg = 0xffffffff & (uint32_t)tmp;
	OUTWORD(MAC_RXRBADR(dev), reg);

	/*
	 * MAC - APTC (Automatic-Polling Timer Control)
	 * Current Setting: 0x00000000 (spec suggested: 0x00000001)
	 *   rx - disable auto-polling, rx cycle 5.12/51.2us
	 *   tx - disable auto-polling, tx cycle 5.12/51.2us
	 */
	reg = ((1 << MAC_APTC_RXPOLL_CNT_SHIFT)
		| (0 << MAC_APTC_RXPOLL_TIME_SEL_BIT)
		| (1 << MAC_APTC_TXPOLL_CNT_SHIFT)
		| (0 << MAC_APTC_TXPOLL_TIME_SEL_BIT));
	OUTWORD(MAC_APTC(dev), reg);

	/* MAC - MACCR (MAC Control Register) */
	reg = (MAC_MACCR_XDMA_EN
		| MAC_MACCR_RDMA_EN
		| MAC_MACCR_RCV_EN
		| MAC_MACCR_XMT_EN
		| MAC_MACCR_RX_RUNT
		| MAC_MACCR_RX_FLT
		| MAC_MACCR_CRC_APD
		| MAC_MACCR_FULLDUP
		| MAC_MACCR_HT_MULTI_EN
		| MAC_MACCR_RX_MULTIPKT
		| MAC_MACCR_RX_BROADPKT);
	reg &= ~(MAC_MACCR_SW_RST
		| MAC_MACCR_CRC_DIS
#ifndef CONFIG_ANDES_ATFMAC100_TX_POLLING_MODE
		| MAC_MACCR_XMT_EN
#endif
		| MAC_MACCR_LOOP_EN
		| MAC_MACCR_ENRX_IN_HALFTX
		| MAC_MACCR_RCV_ALL);
	OUTWORD(MAC_MACCR(dev), reg);

	/* MAC - Write RXPD to transfer ownership of rxds to MAC controller */
	if (dev_data->init_rx_en)
		OUTWORD(MAC_RXPD(dev), 0);
}

#ifndef CONFIG_ANDES_ATFMAC100_TX_POLLING_MODE
static void eth_enable_tx(const struct device *dev, int enable)
{
	//const struct eth_atfmac100_config *const dev_cfg = DEV_ETH_CFG(dev);
	uint32_t reg = 0;
	uint32_t tx_irq_lock = 0;

	if (enable)
	{
		//TODO Just for walk around the problem occuring IRQ without MIP
		tx_irq_lock = irq_lock();
		//irq_disable(dev_cfg->eth_irq_num);

		reg = INWORD(MAC_MACCR(dev));
		reg |= MAC_MACCR_XMT_EN;
		OUTWORD(MAC_MACCR(dev), reg);

		reg = INWORD(MAC_IME(dev));
		reg |= (MAC_IME_NOTXBUF_EN | MAC_IME_XPKT_OK_EN | MAC_IME_XPKT_LOST_EN | MAC_IME_XPKT_FINISH_EN);
		OUTWORD(MAC_IME(dev), reg);

		//irq_enable(dev_cfg->eth_irq_num);
		irq_unlock(tx_irq_lock);
		OUTWORD(MAC_TXPD(dev), 0);
	}
	else
	{
		reg = INWORD(MAC_MACCR(dev));
		reg &= ~(MAC_MACCR_XMT_EN);
		OUTWORD(MAC_MACCR(dev), reg);

		reg = INWORD(MAC_IME(dev));
		reg &= ~(MAC_IME_NOTXBUF_EN | MAC_IME_XPKT_OK_EN | MAC_IME_XPKT_LOST_EN | MAC_IME_XPKT_FINISH_EN);
		OUTWORD(MAC_IME(dev), reg);
	}

}
#endif

static void eth_init_txd_ring(const struct device *dev)
{
	struct eth_atfmac100_data *const dev_data = DEV_ETH_DATA(dev);

	unsigned char *buf = (unsigned char*)((unsigned long)(dev_data->tx_buf_pool + 3) &~ 3);
	uint32_t i;


	for (i = 0; i < MACD_DESCRIPTOR_SLOTS; ++i)
	{
		dev_data->txd_base[i].desc0  = 0;
		dev_data->txd_base[i].desc1  = 0;
		dev_data->txd_base[i].txb    = (uint32_t)(unsigned long)buf;
		dev_data->txd_base[i].next   = (uint32_t)(unsigned long)&dev_data->txd_base[i + 1];

		buf += 2048;
	}

	dev_data->txd_base[MACD_DESCRIPTOR_SLOTS - 1].desc1 |= MAC_TXDES1_EDOTR_MASK;
	dev_data->txd_base[MACD_DESCRIPTOR_SLOTS - 1].next = (uint32_t)(unsigned long)&dev_data->txd_base[0];

	dev_data->txd_iter = dev_data->txd_base;

}

static void eth_init_rxd_ring(const struct device *dev)
{
	struct eth_atfmac100_data *const dev_data = DEV_ETH_DATA(dev);

	unsigned char *buf = (unsigned char*)((unsigned long)(dev_data->rx_buf_pool + 3) &~ 3);
	uint32_t i;

	for (i = 0; i < MACD_DESCRIPTOR_SLOTS; ++i)
	{
		dev_data->rxd_base[i].desc0  = MAC_RXDES0_RXDMA_OWN_MASK;
		dev_data->rxd_base[i].desc1  = (2028 << 0);
		dev_data->rxd_base[i].rxb    = (uint32_t)(unsigned long)buf;
		dev_data->rxd_base[i].next   = (uint32_t)(unsigned long)&dev_data->rxd_base[i + 1];

		buf += 2048;
	}

	dev_data->rxd_base[MACD_DESCRIPTOR_SLOTS - 1].desc1 |= MAC_RXDES1_EDORR_MASK;
	dev_data->rxd_base[MACD_DESCRIPTOR_SLOTS - 1].next = (uint32_t)(unsigned long)&dev_data->rxd_base[0];

	dev_data->rxd_iter = dev_data->rxd_base;
}

static void eth_atfmac100_init(struct net_if *iface)
{
	const struct device * dev = net_if_get_device(iface);
	struct eth_atfmac100_data * dev_data = DEV_ETH_DATA(dev);
	const struct eth_atfmac100_config *const dev_cfg = DEV_ETH_CFG(dev);

	static bool init_done;

	if (dev_data->iface == NULL)
	{
		dev_data->iface = iface;
	}

	ethernet_init(iface);

	/* The rest of initialization should only be done once */
	if (init_done)
	{
		return;
	}

	/* Initialize semaphore. */
	k_sem_init(&dev_data->tx_sem, 0, 1);
	k_sem_give(&dev_data->tx_sem);

	/* Register Ethernet MAC Address with the upper layer */
	net_if_set_link_addr(iface, dev_data->mac_addr,
			     sizeof(dev_data->mac_addr),
			     NET_LINK_ETHERNET);

	dev_cfg->config_func();

	init_done = true;
}

//static struct device DEVICE_GET(eth_atfmac100);


static int eth_atfmac100_dev_init(const struct device *dev)
{
	struct eth_atfmac100_data *const dev_data = DEV_ETH_DATA(dev);

	dev_data->txd_orig = txd_orig_buffer;
	dev_data->txd_base = (struct tx_desc*)((unsigned long)(dev_data->txd_orig + 15) &~ 15);

	dev_data->rxd_orig = rxd_orig_buffer;
	dev_data->rxd_base = (struct rx_desc*)((unsigned long)(dev_data->rxd_orig + 15) &~ 15);

	dev_data->rxd_done_iter = dev_data->rxd_base;
	dev_data->tx_buf_pool = tx_pool;
	dev_data->rx_buf_pool = rx_pool;

	dev_data->init_rx_en = 1;
	dev_data->mtu = MACD_TXB_SIZE < 1500 ? MACD_TXB_SIZE : 1500;

	eth_init_txd_ring(dev);
	eth_init_rxd_ring(dev);

	eth_hw_init(dev);

	return 0;
}

static void eth_atfmac100_irq(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	uint32_t mac_isr, reg;
	int mac_status = 0;

	/* Mask MAC interrupt */
	/* Clean pending */
	mac_isr = INWORD(MAC_ISR(dev)); /* read & clear the mac int status */
	/*
	 * RX status
	 *  [6] RPKT_SAV		rx -> fifo -> ok
	 *  [0] RPKT_FINISH		rx -> rxb -> ok
	 *  [1] NORXBUF			rxb unavailable
	 *  [7] RPKT_LOST		rx lost - rx fifo full
	 * TX status
	 *  [2] XPKT_FINISH		tx -> fifo -> ok
	 *  [3] NOTXBUF			txb unavailable
	 *  [4] XPKT_OK			tx -> phy -> ok
	 *  [5] XPKT_LOST		tx -> phy -> collision
	 * misc
	 *  [8] AHB_ERR			ahb error
	 *  [9] PHYSTS_CHG		link status change
	 */
	if (mac_isr & MAC_ISR_RPKT_FINISH)
	{
		/* Disable RX-int before upper layers completed the processing of incoming frame */
		reg = INWORD(MAC_IME(dev));
		reg &= ~(MAC_IME_RPKT_FINISH_EN | MAC_IME_NORXBUF_EN | MAC_IME_RPKT_LOST_EN);
		OUTWORD(MAC_IME(dev), reg);

		mac_status = eth_atfmac100_receive(dev);
		if( (mac_status == DRIVER_ERROR) && (mac_isr & MAC_ISR_RPKT_LOST) && (mac_isr & MAC_ISR_NORXBUF))
		{
			NET_INFO("It may be a problem, because both rx fifo and descriptor are full, but system can't handle rx packets from descriptor efficiently\n");
		}

		/* Re-enable RX-int */
		reg = INWORD(MAC_IME(dev));
		reg |= (MAC_IME_RPKT_FINISH_EN | MAC_IME_NORXBUF_EN | MAC_IME_RPKT_LOST_EN);
		OUTWORD(MAC_IME(dev), reg);
	}

#ifndef CONFIG_ANDES_ATFMAC100_TX_POLLING_MODE
	struct eth_atfmac100_data *const dev_data = DEV_ETH_DATA(dev);

	if (mac_isr & MAC_ISR_XPKT_OK)
	{
		reg = INWORD(MAC_IME(dev));
		reg &= ~(MAC_IME_NOTXBUF_EN | MAC_IME_XPKT_OK_EN | MAC_IME_XPKT_LOST_EN | MAC_IME_XPKT_FINISH_EN);
		OUTWORD(MAC_IME(dev), reg);

		eth_enable_tx(dev, 0);
		k_sem_give(&dev_data->tx_sem);
	}
#endif

	if ((mac_isr & MAC_ISR_PHYSTS_CHG) || (mac_isr & MAC_ISR_AHB_ERR))
	{
		if(mac_isr & MAC_ISR_PHYSTS_CHG)
		{
			NET_INFO("MAC_ISR_PHYSTS_CHG\n");
		}
		else
		{
			NET_INFO("MAC_ISR_AHB_ERR\n");
		}
		while(1);
	}

	if ((mac_isr & MAC_ISR_NORXBUF) || (mac_isr & MAC_ISR_XPKT_LOST) || (mac_isr & MAC_ISR_RPKT_LOST))
	{
		if (mac_isr & MAC_ISR_XPKT_LOST)
		{
			NET_INFO("Tx packets lost.\n");
		}
		if (mac_isr & MAC_ISR_RPKT_LOST)
		{
			NET_INFO("Rx packets lost due to rx FIFO full.\n");
		}
		if (mac_isr & MAC_ISR_NORXBUF)
		{
			NET_INFO("Rx descriptor is full, may lost the rx packets\n");
			eth_atfmac100_receive(dev);
		}

	}
}

DEVICE_DECLARE(eth_atfmac100);

static void eth_atfmac100_irq_config()
{
	/* Enable Interrupt. */
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    eth_atfmac100_irq, DEVICE_GET(eth_atfmac100), 0);
	irq_enable(DT_INST_IRQN(0));
}

static const struct eth_atfmac100_config eth_cfg = {
	.eth_base_addr = DT_INST_REG_ADDR(0),
	.eth_irq_num = DT_INST_IRQN(0),
	.config_func = eth_atfmac100_irq_config,
};

struct eth_atfmac100_data eth_data = {
	.mac_addr = DT_INST_PROP(0, local_mac_address),
};

static const struct ethernet_api eth_atfmac100_apis = {
	.iface_api.init = eth_atfmac100_init,
	.send =  eth_atfmac100_send,
};

NET_DEVICE_INIT(eth_atfmac100, DT_INST_LABEL(0),
		eth_atfmac100_dev_init, device_pm_control_nop, &eth_data, &eth_cfg,
		CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		&eth_atfmac100_apis, ETHERNET_L2,
		NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);
