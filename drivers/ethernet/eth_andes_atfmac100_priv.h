
#include <errno.h>
#include <kernel.h>
#include <sys/util.h>
#include <sys/sys_io.h>
#include <zephyr/types.h>
//#include <nds_intrinsic.h>

#ifndef BIT
#define BIT(n)		  ((unsigned int) 1 << (n))
#define BITS2(m,n)	      (BIT(m) | BIT(n) )

/* bits range: for example BITS(16,23) = 0xFF0000
    *   ==>  (BIT(m)-1)   = 0x0000FFFF     ~(BIT(m)-1)   => 0xFFFF0000
     *   ==>  (BIT(n+1)-1) = 0x00FFFFFF
      */
#endif

#ifndef BITS
#define BITS(m,n)		(~(BIT(m)-1) & ((BIT(n) - 1) | BIT(n)))
#endif /* BIT */

#define	MACD_DESCRIPTOR_SLOTS	30
#define MACD_TXB_SIZE		2028

#define DRIVER_OK			0 ///< Operation succeeded
#define DRIVER_ERROR			-1 ///< Unspecified error
#define DRIVER_ERROR_BUSY		-2 ///< Driver is busy
#define DRIVER_ERROR_TIMEOUT		-3 ///< Timeout occurred
#define DRIVER_ERROR_UNSUPPORTED	-4 ///< Operation not supported
#define DRIVER_ERROR_PARAMETER		-5 ///< Parameter error
#define DRIVER_ERROR_SPECIFIC		-6 ///< Start of driver specific errors

#define REG_ISR			0x00 /* Interrupt Status Register */
#define REG_IME			0x04 /* Interrupt Enable Register */
#define REG_MACMADR		0x08 /* MAC Most Significant Address Register */
#define REG_MACLADR		0x0c /* MAC Least Significant Address Register */
#define REG_MAHT0		0x10 /* Multicast Address Hash Table 0 Register */
#define REG_MAHT1		0x14 /* Multicast Address Hash Table 1 Register */
#define REG_TXPD		0x18 /* Transmit Poll Demand Register */
#define REG_RXPD		0x1c /* Receive Poll Demand Register */
#define REG_TXRBADR		0x20 /* Transmit Ring Base Address Register */
#define REG_RXRBADR		0x24 /* Receive Ring Base Address Register */
#define REG_ITC			0x28 /* Interrupt Timer Control Register */
#define REG_APTC		0x2c /* Automatic Polling Timer Control Register */
#define REG_DBLAC		0x30 /* DMA Burst Length and Arbitration Control Register */
#define REG_REVR		0x34 /* Revision Register */
#define REG_FEAR		0x38 /* Feature Register */
#define REG_MACCR		0x88 /* MAC Control Register */
#define REG_MACSR		0x8c /* MAC Status Register */
#define REG_PHYCR		0x90 /* PHY Control Register */
#define REG_PHYWDATA		0x94 /* PHY Write Data Register */
#define REG_FCR			0x98 /* Flow Control Register */
#define REG_BPR			0x9c /* Back Pressure Register */
#define REG_WOLCR		0xa0 /* Wake-On-LAN Control Register */
#define REG_WOLSR		0xa4 /* Wake-On-LAN Status Register */
#define REG_WFCRC		0xa8 /* Wake-up Frame CRC Register */
#define REG_WFBM1		0xb0 /* Wake-up Frame Byte Mask 1st Double Word Register */
#define REG_WFBM2		0xb4 /* Wake-up Frame Byte Mask 2nd Double Word Register */
#define REG_WFBM3		0xb8 /* Wake-up Frame Byte Mask 3rd Double Word Register */
#define REG_WFBM4		0xbc /* Wake-up Frame Byte Mask 4th Double Word Register */
#define REG_TS			0xc4 /* Test Seed Register */
#define REG_DMAFIFOS		0xc8 /* DMA/FIFO State Register */
#define REG_TM			0xcc /* Test Mode Register */
#define REG_TXMSCOL		0xd4 /* TX_MCOL and TX_SCOL Counter Register */
#define REG_RPFAEPCNT		0xd8 /* RPF and AEP Counter Register */
#define REG_XMPGCNT		0xdc /* XM and PG Counter Register */
#define REG_RUNTTLCCNT		0xe0 /* RUNT_CNT and TLCC Counter Register */
#define REG_CRFTLCNT		0xe4 /* CRCER_CNT and FTL_CNT Counter Register */
#define REG_RLCRCCNT		0xe8 /* RLC and RCC Counter Register */
#define REG_BRPCNT		0xec /* BROC Counter Register */
#define REG_MULCACNT		0xf0 /* MULCA Counter Register */
#define REG_RPCNT		0xf4 /* RP Counter Register */
#define REG_XPCNT		0xf8 /* XP Counter Register */


#define DEV_ETH_CFG(dev)				\
		((const struct eth_atfmac100_config* const)(dev)->config)
#define DEV_ETH_DATA(dev)				\
		((struct eth_atfmac100_data* const)(dev)->data)


#define MAC_ISR(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_ISR)
#define MAC_IME(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_IME)
#define MAC_MACMADR(dev)	(DEV_ETH_CFG(dev)->eth_base_addr + REG_MACMADR)
#define MAC_MACLADR(dev)	(DEV_ETH_CFG(dev)->eth_base_addr + REG_MACLADR)
#define MAC_MAHT0(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_MAHT0)
#define MAC_MAHT1(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_MAHT1)
#define MAC_TXPD(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_TXPD)
#define MAC_RXPD(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_RXPD)
#define MAC_TXRBADR(dev)	(DEV_ETH_CFG(dev)->eth_base_addr + REG_TXRBADR)
#define MAC_RXRBADR(dev)	(DEV_ETH_CFG(dev)->eth_base_addr + REG_RXRBADR)
#define MAC_ITC(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_ITC)
#define MAC_APTC(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_APTC)
#define MAC_DBLAC(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_DBLAC)
#define MAC_REVR(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_REVR)
#define MAC_FEAR(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_FEAR)
#define MAC_MACCR(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_MACCR)
#define MAC_MACSR(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_MACSR)
#define MAC_PHYCR(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_PHYCR)
#define MAC_PHYWDATA(dev)	(DEV_ETH_CFG(dev)->eth_base_addr + REG_PHYWDATA)
#define MAC_FCR(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_FCR)
#define MAC_BPR(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_BPR)
#define MAC_WOLCR(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_WOLCR)
#define MAC_WOLSR(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_WOLSR)
#define MAC_WFCRC(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_WFCRC)
#define MAC_WFBM1(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_WFBM1)
#define MAC_WFBM2(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_WFBM2)
#define MAC_WFBM3(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_WFBM3)
#define MAC_WFBM4(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_WFBM4)
#define MAC_TS(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_TS)
#define MAC_DMAFIFOS(dev)	(DEV_ETH_CFG(dev)->eth_base_addr + REG_DMAFIFOS)
#define MAC_TM(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_TM)
#define MAC_TXMSCOL(dev)	(DEV_ETH_CFG(dev)->eth_base_addr + REG_TXMSCOL)
#define MAC_RPFAEPCNT(dev)	(DEV_ETH_CFG(dev)->eth_base_addr + REG_RPFAEPCNT)
#define MAC_XMPGCNT(dev)	(DEV_ETH_CFG(dev)->eth_base_addr + REG_XMPGCNT)
#define MAC_RUNTTLCCNT(dev)	(DEV_ETH_CFG(dev)->eth_base_addr + REG_RUNTTLCCNT)
#define MAC_CRFTLCNT(dev)	(DEV_ETH_CFG(dev)->eth_base_addr + REG_CRFTLCNT)
#define MAC_RLCRCCNT(dev)	(DEV_ETH_CFG(dev)->eth_base_addr + REG_RLCRCCNT)
#define MAC_BRPCNT(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_BRPCNT)
#define MAC_MULCACNT(dev)	(DEV_ETH_CFG(dev)->eth_base_addr + REG_MULCACNT)
#define MAC_RPCNT(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_RPCNT)
#define MAC_XPCNT(dev)		(DEV_ETH_CFG(dev)->eth_base_addr + REG_XPCNT)

#define MEMSET(s, c, n)		__builtin_memset ((s), (c), (n))
#define MEMCPY(des, src, n)	__builtin_memcpy((des), (src), (n))
#define INWORD(x)		sys_read32(x)
#define OUTWORD(x, d)		sys_write32(d, x)

#define set_csr(reg, bit)	__nds__csrrs(bit, reg)
#define clear_csr(reg, bit)	__nds__csrrc(bit, reg)

/* TXDES0: contain the transmit frame status and descriptor ownership information */
#define	MAC_TXDES0_TXDMA_OWN_MASK	(uint32_t)BIT(31)
#define	MAC_TXDES0_TXPKT_EXSCOL_MASK	(uint32_t)BIT(1)
#define	MAC_TXDES0_TXPKT_LATECOL_MASK	(uint32_t)BIT(0)

/* TXDES1: contain the control bits and transmit buffer size */
#define MAC_TXDES1_EDOTR_MASK		(uint32_t)BIT(31)
#define	MAC_TXDES1_TXIC_BIT		30
#define	MAC_TXDES1_TXIC_MASK		(uint32_t)BIT(30)
#define	MAC_TXDES1_TX2FIC_BIT		29
#define	MAC_TXDES1_TX2FIC_MASK		(uint32_t)BIT(29)
#define	MAC_TXDES1_FTS_MASK		(uint32_t)BIT(28)
#define	MAC_TXDES1_LTS_MASK		(uint32_t)BIT(27)
#define	MAC_TXDES1_TXBUF_SIZE_MASK	(uint32_t)BITS(0,10)

/* tx descriptor supplemental macros */
#define MACD_TXD_PROBE_LATE_COL(txd)	((txd)->desc0 & MAC_TXDES0_LATECOL_MASK)
#define MACD_TXD_PROBE_EXS_COL(txd)	((txd)->desc0 & MAC_TXDES0_EXSCOL_MASK)
#define MACD_TXD_PROBE_DMA_OWN(txd)	((txd)->desc0 & MAC_TXDES0_TXDMA_OWN_MASK)
#define MACD_TXD_CLR_DMA_OWN(txd)	((txd)->desc0 &= ~MAC_TXDES0_TXDMA_OWN_MASK)
#define MACD_TXD_SET_DMA_OWN(txd)	((txd)->desc0 |= MAC_TXDES0_TXDMA_OWN_MASK)

#define MACD_TXD_CLR_TXB_SIZE(txd, sz)  ((txd)->desc1 = ((txd)->desc1 & ~MAC_TXDES1_TXBUF_SIZE_MASK))
#define MACD_TXD_SET_TXB_SIZE(txd, sz)  ((txd)->desc1 = ((txd)->desc1 & ~MAC_TXDES1_TXBUF_SIZE_MASK) | \
					((sz) & MAC_TXDES1_TXBUF_SIZE_MASK))

#define MACD_TXD_CLR_LTS(txd)		((txd)->desc1 &= ~MAC_TXDES1_LTS_MASK)
#define MACD_TXD_SET_LTS(txd)		((txd)->desc1 |= MAC_TXDES1_LTS_MASK)

#define MACD_TXD_CLR_FTS(txd)		((txd)->desc1 = (((txd)->desc1) & ~(MAC_TXDES1_FTS_MASK | \
					MAC_TXDES1_TX2FIC_MASK | MAC_TXDES1_TXIC_MASK)))

#define MACD_TXD_SET_FTS(txd, fi, ti)	((txd)->desc1 = (((txd)->desc1) | (MAC_TXDES1_FTS_MASK | \
					(((fi) << MAC_TXDES1_TX2FIC_BIT) & MAC_TXDES1_TX2FIC_MASK) | \
					(((ti) << MAC_TXDES1_TXIC_BIT) & MAC_TXDES1_TXIC_MASK))))

#define MACD_TXD_CLR_EDOTR(txd)		((txd)->desc1 &= ~MAC_TXDES1_EDOTR_MASK)
#define MACD_TXD_SET_EDOTR(txd)		((txd)->desc1 |= MAC_TXDES1_EDOTR_MASK)

#define MACD_TXD_CLR_TXB(txd)		((txd)->txb = 0)
#define MACD_TXD_SET_TXB(txd, p)	((txd)->txb = (p))

/* rx descriptor supplemental macros */
#define MACD_RXD_GET_RFL(rxd)		((rxd)->desc0 & MAC_RXDES0_RFL_MASK)
#define MACD_RXD_PROBE_MULTICAST(rxd)	((rxd)->desc0 & MAC_RXDES0_MULTICAST_MASK)
#define MACD_RXD_PROBE_BROADCAST(rxd)	((rxd)->desc0 & MAC_RXDES0_BROADCAST_MASK)
#define MACD_RXD_PROBE_RX_ERR(rxd)	((rxd)->desc0 & MAC_RXDES0_RX_ERR_MASK)
#define MACD_RXD_PROBE_CRC_ERR(rxd)	((rxd)->desc0 & MAC_RXDES0_CRC_ERR_MASK)
#define MACD_RXD_PROBE_FTL(rxd)		((rxd)->desc0 & MAC_RXDES0_FTL_MASK)
#define MACD_RXD_PROBE_RUNT(rxd)	((rxd)->desc0 & MAC_RXDES0_RUNT_MASK)
#define MACD_RXD_PROBE_ODD_NB(rxd)	((rxd)->desc0 & MAC_RXDES0_RX_ODD_NB_MASK)
#define MACD_RXD_PROBE_LRS(rxd)		((rxd)->desc0 & MAC_RXDES0_LRS_MASK)
#define MACD_RXD_PROBE_FRS(rxd)		((rxd)->desc0 & MAC_RXDES0_FRS_MASK)
#define MACD_RXD_PROBE_DMA_OWN(rxd)	((rxd)->desc0 & MAC_RXDES0_RXDMA_OWN_MASK)
#define MACD_RXD_CLR_DMA_OWN(rxd)	((rxd)->desc0 &= ~MAC_RXDES0_RXDMA_OWN_MASK)
#define MACD_RXD_SET_DMA_OWN(rxd)	((rxd)->desc0 |= MAC_RXDES0_RXDMA_OWN_MASK)

#define MACD_RXD_SET_RXB_SIZE(rxd, sz)	((rxd)->desc1 = ((rxd)->desc1 & ~MAC_RXDES1_RXBUF_SIZE_MASK) |\
					((uint32_t)(sz) & MAC_RXDES1_RXBUF_SIZE_MASK))
#define MACD_RXD_CLR_EDORR(rxd)		((rxd)->desc1 &= ~MAC_RXDES1_EDORR_MASK)
#define MACD_RXD_SET_EDORR(rxd)		((rxd)->desc1 |= MAC_RXDES1_EDORR_MASK)
#define MACD_RXD_SET_CTRL(rxd, sz, eor)	((rxd)->desc1 = (((sz) & MAC_RXDES1_RXBUF_SIZE_MASK) |\
					(((uint32_t)(eor) << MAC_RXDES1_EDORR_BIT) & MAC_RXDES1_EDORR_MASK)))

#define MACD_RXD_CLR_RXB(rxd)		((rxd)->rxb = 0)
#define MACD_RXD_SET_RXB(rxd, p)	((rxd)->rxb = (uint32_t)(p))


/*  RXDES0: contains the receive frame status and descriptor ownership information */
#define	MAC_RXDES0_RXDMA_OWN_MASK	(uint32_t)BIT(31)
#define	MAC_RXDES0_FRS_MASK		(uint32_t)BIT(29)
#define	MAC_RXDES0_LRS_MASK		(uint32_t)BIT(28)
#define	MAC_RXDES0_RX_ODD_NB_MASK	(uint32_t)BIT(22)
#define	MAC_RXDES0_RUNT_MASK		(uint32_t)BIT(21)
#define	MAC_RXDES0_FTL_MASK		(uint32_t)BIT(20)
#define	MAC_RXDES0_CRC_ERR_MASK		(uint32_t)BIT(19)
#define	MAC_RXDES0_RX_ERR_MASK		(uint32_t)BIT(18)
#define	MAC_RXDES0_BROADCAST_MASK	(uint32_t)BIT(17)
#define	MAC_RXDES0_MULTICAST_MASK	(uint32_t)BIT(16)
#define MAC_RXDES0_RFL_MASK		(uint32_t)BITS(0,10)

/* RXDES1: contains the control bits and receive buffer size */
#define	MAC_RXDES1_EDORR_MASK		(uint32_t)BIT(31)
#define	MAC_RXDES1_RXBUF_SIZE_MASK	(uint32_t)BITS(0,10)

/* Interrupt Status Register */
#define	MAC_ISR_RPKT_FINISH		(uint32_t)BIT(0)
#define	MAC_ISR_NORXBUF			(uint32_t)BIT(1)
#define	MAC_ISR_XPKT_FINISH		(uint32_t)BIT(2)
#define	MAC_ISR_NOTXBUF			(uint32_t)BIT(3)
#define	MAC_ISR_XPKT_OK			(uint32_t)BIT(4)
#define	MAC_ISR_XPKT_LOST		(uint32_t)BIT(5)
#define	MAC_ISR_RPKT_SAV		(uint32_t)BIT(6)
#define	MAC_ISR_RPKT_LOST		(uint32_t)BIT(7)
#define	MAC_ISR_AHB_ERR			(uint32_t)BIT(8)
#define	MAC_ISR_PHYSTS_CHG		(uint32_t)BIT(9)

/* MAC Control Register */
#define MAC_MACCR_XDMA_EN		(uint32_t)BIT(0)
#define MAC_MACCR_RDMA_EN		(uint32_t)BIT(1)
#define MAC_MACCR_SW_RST		(uint32_t)BIT(2)
#define MAC_MACCR_LOOP_EN		(uint32_t)BIT(3)
#define MAC_MACCR_CRC_DIS		(uint32_t)BIT(4)
#define MAC_MACCR_XMT_EN		(uint32_t)BIT(5)
#define MAC_MACCR_ENRX_IN_HALFTX	(uint32_t)BIT(6)
#define MAC_MACCR_RCV_EN		(uint32_t)BIT(8)
#define MAC_MACCR_HT_MULTI_EN		(uint32_t)BIT(9)
#define MAC_MACCR_RX_RUNT		(uint32_t)BIT(10)
#define MAC_MACCR_RX_FLT		(uint32_t)BIT(11)
#define MAC_MACCR_RCV_ALL		(uint32_t)BIT(12)
#define MAC_MACCR_CRC_APD		(uint32_t)BIT(14)
#define MAC_MACCR_FULLDUP		(uint32_t)BIT(15)
#define MAC_MACCR_RX_MULTIPKT		(uint32_t)BIT(16)
#define MAC_MACCR_RX_BROADPKT		(uint32_t)BIT(17)

/* Interrupt Enable Register */
#define	MAC_IME_RPKT_FINISH_EN		(uint32_t)BIT(0)
#define	MAC_IME_NORXBUF_EN		(uint32_t)BIT(1)
#define	MAC_IME_XPKT_FINISH_EN		(uint32_t)BIT(2)
#define	MAC_IME_NOTXBUF_EN		(uint32_t)BIT(3)
#define	MAC_IME_XPKT_OK_EN		(uint32_t)BIT(4)
#define	MAC_IME_XPKT_LOST_EN		(uint32_t)BIT(5)
#define	MAC_IME_RPKT_SAV_EN		(uint32_t)BIT(6)
#define	MAC_IME_RPKT_LOST_EN		(uint32_t)BIT(7)
#define MAC_IME_AHB_ERR_EN		(uint32_t)BIT(8)
#define MAC_IME_PHYSTS_CHG_EN		(uint32_t)BIT(9)

/* Interrupt Timer Control Register */
#define	MAC_ITC_RXINT_CNT_SHIFT		0
#define	MAC_ITC_RXINT_THR_SHIFT		4
#define	MAC_ITC_RXINT_TIME_SEL_BIT	7
#define	MAC_ITC_TXINT_CNT_SHIFT		8
#define	MAC_ITC_TXINT_THR_SHIFT		12
#define	MAC_ITC_TXINT_TIME_SEL_BIT	15

/* DMA Burst Length and Arbitration Control Register */
#define	MAC_DBLAC_INCR4_EN		(uint32_t)BIT(0)
#define	MAC_DBLAC_INCR8_EN		(uint32_t)BIT(1)
#define	MAC_DBLAC_INCR16_EN		(uint32_t)BIT(2)
#define	MAC_DBLAC_RXFIFO_LTHR_SHIFT	3
#define	MAC_DBLAC_RXFIFO_HTHR_SHIFT	6
#define	MAC_DBLAC_RX_THR_EN		(uint32_t)BIT(9)
#define	MAC_DBLAC_INCR_SEL_SHIFT	14

/* Flow Control Register */
#define	MAC_FCR_FC_EN			(uint32_t)BIT(0)
#define	MAC_FCR_TX_PAUSE		(uint32_t)BIT(1)
#define	MAC_FCR_FCTHR_EN		(uint32_t)BIT(2)
#define	MAC_FCR_TXPAUSED		(uint32_t)BIT(3)
#define	MAC_FCR_RX_PAUSE		(uint32_t)BIT(4)
#define	MAC_FCR_FC_LOW_SHIFT		8
#define	MAC_FCR_FC_HIGH_SHIFT		12
#define	MAC_FCR_PAUSE_TIME_SHIFT	16

/* Back Pressure Register */
#define	MAC_BPR_BK_EN			(uint32_t)BIT(0)
#define	MAC_BPR_BK_MODE_BIT		1
#define	MAC_BPR_BKJAM_LEN_SHIFT		4
#define	MAC_BPR_BK_LOW_SHIFT		8

/* Threshold value */
#define MAC_FIFO_0			0
#define	MAC_FIFO_256			1
#define	MAC_FIFO_512			2
#define	MAC_FIFO_768			3
#define	MAC_FIFO_1024			4
#define	MAC_FIFO_1280			5
#define	MAC_FIFO_1536			6
#define	MAC_FIFO_1792			7

/* Automatic Polling Timer Control Register */
#define	MAC_APTC_RXPOLL_CNT_SHIFT	0
#define	MAC_APTC_RXPOLL_TIME_SEL_BIT	4
#define	MAC_APTC_TXPOLL_CNT_SHIFT	8
#define	MAC_APTC_TXPOLL_TIME_SEL_BIT	12

struct eth_atfmac100_config {
	uint32_t eth_base_addr;
	uint32_t eth_irq_num;
	void (*config_func)(void);
};

struct tx_desc
{
	uint32_t	desc0;
	uint32_t	desc1;
	uint32_t	txb;
	uint32_t	next;
} __attribute__((__packed__));

struct rx_desc
{
	uint32_t	desc0;
	uint32_t	desc1;
	uint32_t	rxb;
	uint32_t	next;
} __attribute__((__packed__));

struct eth_atfmac100_data
{
	struct net_if *iface;
	/* Packing of various short parameters */
	uint8_t		hisr_as;	/* HISR activation state */
	//hal_bh_t	hcb;

	uint8_t		mac_addr[6];    /* MAC address, valid: mac_addr[0] ~ mac_addr[5] */
	uint8_t		init_rx_en;     /* enable tx/rx during initialization */
	int 		mtu;

	/* Synchronization */
	struct k_sem	tx_sem;		/* control tx access */

	/* TX buff */
	unsigned char	*tx_buf_pool;
	unsigned char	*txd_orig;      /* The first entry in the tx descriptor ring */
	struct tx_desc	*txd_base;      /* The base address of the entire tx buffer */
	struct tx_desc	*txd_iter;      /* The next descriptor entry to start tx operations */

	/* RX buff */
	unsigned char	*rx_buf_pool;
	unsigned char	*rxd_orig;      /* The first entry in the rx descriptor ring */
	struct rx_desc	*rxd_base;      /* The base address of the entire rx buffer */
	struct rx_desc	*rxd_iter;      /* The next descriptor entry to start reading when rx interrupt got triggered */
	struct rx_desc	*rxd_done_iter; /* The next descriptor entry to start reading when rx interrupt got triggered */
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth stats;
#endif
};
