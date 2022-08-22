/*
 * Copyright (c) 2019 Andes Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_atcspi200);

#include "spi_context.h"
#include <device.h>
#include <drivers/spi.h>

#define DT_DRV_COMPAT andestech_atcspi200

#define REG_IDR         0x00
#define REG_TFMAT       0x10
#define REG_DIRIO       0x14
#define REG_TCTRL       0x20
#define REG_CMD         0x24
#define REG_ADDR        0x28
#define REG_DATA        0x2c
#define REG_CTRL        0x30
#define REG_STAT        0x34
#define REG_INTEN       0x38
#define REG_INTST       0x3c
#define REG_TIMIN       0x40
#define REG_MCTRL       0x50
#define REG_SLVST       0x60
#define REG_SDCNT       0x64
#define REG_CONFIG      0x7c

#define SPI_DEV_CFG(dev) ((const struct spi_atcspi200_device_config * const) (dev)->config)
#define SPI_DEV_DATA(dev) ((struct spi_atcspi200_dev_data_t *) (dev)->data)

#define SPI_IDR(dev)    (SPI_DEV_CFG(dev)->spi_base_addr + REG_IDR)
#define SPI_TFMAT(dev)  (SPI_DEV_CFG(dev)->spi_base_addr + REG_TFMAT)
#define SPI_DIRIO(dev)  (SPI_DEV_CFG(dev)->spi_base_addr + REG_DIRIO)
#define SPI_TCTRL(dev)  (SPI_DEV_CFG(dev)->spi_base_addr + REG_TCTRL)
#define SPI_CMD(dev)    (SPI_DEV_CFG(dev)->spi_base_addr + REG_CMD)
#define SPI_ADDR(dev)   (SPI_DEV_CFG(dev)->spi_base_addr + REG_ADDR)
#define SPI_DATA(dev)   (SPI_DEV_CFG(dev)->spi_base_addr + REG_DATA)
#define SPI_CTRL(dev)   (SPI_DEV_CFG(dev)->spi_base_addr + REG_CTRL)
#define SPI_STAT(dev)   (SPI_DEV_CFG(dev)->spi_base_addr + REG_STAT)
#define SPI_INTEN(dev)  (SPI_DEV_CFG(dev)->spi_base_addr + REG_INTEN)
#define SPI_INTST(dev)  (SPI_DEV_CFG(dev)->spi_base_addr + REG_INTST)
#define SPI_TIMIN(dev)  (SPI_DEV_CFG(dev)->spi_base_addr + REG_TIMIN)
#define SPI_MCTRL(dev)  (SPI_DEV_CFG(dev)->spi_base_addr + REG_MCTRL)
#define SPI_SLVST(dev)  (SPI_DEV_CFG(dev)->spi_base_addr + REG_SLVST)
#define SPI_SDCNT(dev)  (SPI_DEV_CFG(dev)->spi_base_addr + REG_SDCNT)
#define SPI_CONFIG(dev) (SPI_DEV_CFG(dev)->spi_base_addr + REG_CONFIG)

/* Field mask of SPI transfer format register */
#define REG_TFMAT_CPHA_OFFSET           (0)
#define REG_TFMAT_CPOL_OFFSET           (1)
#define REG_TFMAT_SLVMODE_OFFSET        (2)
#define REG_TFMAT_LSB_OFFSET            (3)
#define REG_TFMAT_MOSI_BIDIR_OFFSET     (4)
#define REG_TFMAT_DATA_MERGE_OFFSET     (7)
#define REG_TFMAT_DATA_LEN_OFFSET       (8)
#define REG_TFMAT_ADDR_LEN_OFFSET       (16)

#define REG_TFMAT_CPHA_MSK              (0x1 << REG_TFMAT_CPHA_OFFSET)
#define REG_TFMAT_CPOL_MSK              (0x1 << REG_TFMAT_CPOL_OFFSET)
#define REG_TFMAT_SLVMODE_MSK           (0x1 << REG_TFMAT_SLVMODE_OFFSET)
#define REG_TFMAT_LSB_MSK               (0x1 << REG_TFMAT_LSB_OFFSET)
#define REG_TFMAT_MOSI_BIDIR_MSK        (0x1 << REG_TFMAT_MOSI_BIDIR_OFFSET)
#define REG_TFMAT_DATA_MERGE_MSK        (0x1 << REG_TFMAT_DATA_MERGE_OFFSET)
#define REG_TFMAT_DATA_LEN_MSK          (0x1f << REG_TFMAT_DATA_LEN_OFFSET)
#define REG_TFMAT_ADDR_LEN_MSK          (0x3 << REG_TFMAT_ADDR_LEN_OFFSET)

/* Field mask of SPI transfer control register */
#define REG_TCTRL_RD_TCNT_OFFSET        (0)
#define REG_TCTRL_DUMMY_CNT_OFFSET      (9)
#define REG_TCTRL_TOKEN_VAL_OFFSET      (11)
#define REG_TCTRL_WR_TCNT_OFFSET        (12)
#define REG_TCTRL_TOKEN_EN_OFFSET       (21)
#define REG_TCTRL_TRNS_MODE_OFFSET      (24)
#define REG_TCTRL_ADDR_FMT_OFFSET       (28)
#define REG_TCTRL_ADDR_EN_OFFSET        (29)
#define REG_TCTRL_CMD_EN_OFFSET         (30)

#define REG_TCTRL_RD_TCNT_MSK           (0x1ff << REG_TCTRL_RD_TCNT_OFFSET)
#define REG_TCTRL_DUMMY_CNT_MSK         (0x3 << REG_TCTRL_DUMMY_CNT_OFFSET)
#define REG_TCTRL_TOKEN_VAL_MSK         (0x1 << REG_TCTRL_TOKEN_VAL_OFFSET)
#define REG_TCTRL_WR_TCNT_MSK           (0x1ff << REG_TCTRL_WR_TCNT_OFFSET)
#define REG_TCTRL_TOKEN_EN_MSK          (0x1 << REG_TCTRL_TOKEN_EN_OFFSET)
#define REG_TCTRL_DQ_MSK                (0x3 << REG_TCTRL_TOKEN_EN_OFFSET)
#define REG_TCTRL_TRNS_MODE_MSK         (0xf << REG_TCTRL_TRNS_MODE_OFFSET)
#define REG_TCTRL_ADDR_FMT_MSK          (0x1 << REG_TCTRL_ADDR_FMT_OFFSET)
#define REG_TCTRL_ADDR_EN_MSK           (0x1 << REG_TCTRL_ADDR_EN_OFFSET)
#define REG_TCTRL_CMD_EN_MSK            (0x1 << REG_TCTRL_CMD_EN_OFFSET)

/* Transfer mode */
#define TRNS_MODE_WRITE_READ             (0)
#define TRNS_MODE_WRITE_ONLY             (1)
#define TRNS_MODE_READ_ONLY              (2)

/* Field mask of SPI interrupt enable register */
#define REG_INTEN_RX_FIFO_OR_IEN_OFFSET  (0)
#define REG_INTEN_TX_FIFO_UR_IEN_OFFSET  (1)
#define REG_INTEN_RX_FIFO_IEN_OFFSET     (2)
#define REG_INTEN_TX_FIFO_IEN_OFFSET     (3)
#define REG_INTEN_END_IEN_OFFSET         (4)
#define REG_INTEN_SLV_CMD_EN_OFFSET      (5)

#define REG_INTEN_RX_FIFO_OR_IEN_MSK     (0x1 << REG_INTEN_RX_FIFO_OR_IEN_OFFSET)
#define REG_INTEN_TX_FIFO_UR_IEN_MSK     (0x1 << REG_INTEN_TX_FIFO_UR_IEN_OFFSET)
#define REG_INTEN_RX_FIFO_IEN_MSK        (0x1 << REG_INTEN_RX_FIFO_IEN_OFFSET)
#define REG_INTEN_TX_FIFO_IEN_MSK        (0x1 << REG_INTEN_TX_FIFO_IEN_OFFSET)
#define REG_INTEN_END_IEN_MSK            (0x1 << REG_INTEN_END_IEN_OFFSET)
#define REG_INTEN_SLV_CMD_EN_MSK         (0x1 << REG_INTEN_SLV_CMD_EN_OFFSET)

/* Field mask of SPI interrupt status register */
#define REG_INTST_RX_FIFO_OR_INT_OFFSET  (0)
#define REG_INTST_TX_FIFO_UR_INT_OFFSET  (1)
#define REG_INTST_RX_FIFO_INT_OFFSET     (2)
#define REG_INTST_TX_FIFO_INT_OFFSET     (3)
#define REG_INTST_END_INT_OFFSET         (4)
#define REG_INTST_SLV_CMD_INT_OFFSET     (5)

#define REG_INTST_RX_FIFO_OR_INT_MSK     (0x1 << REG_INTST_RX_FIFO_OR_INT_OFFSET)
#define REG_INTST_TX_FIFO_UR_INT_MSK     (0x1 << REG_INTST_TX_FIFO_UR_INT_OFFSET)
#define REG_INTST_RX_FIFO_INT_MSK        (0x1 << REG_INTST_RX_FIFO_INT_OFFSET)
#define REG_INTST_TX_FIFO_INT_MSK        (0x1 << REG_INTST_TX_FIFO_INT_OFFSET)
#define REG_INTST_END_INT_MSK            (0x1 << REG_INTST_END_INT_OFFSET)
#define REG_INTST_SLV_CMD_INT_MSK        (0x1 << REG_INTST_SLV_CMD_INT_OFFSET)

/* Field mask of SPI config register */
#define REG_CONFIG_RX_FIFO_SIZE_OFFSET   (0)
#define REG_CONFIG_TX_FIFO_SIZE_OFFSET   (4)
#define REG_CONFIG_DUAL_SPI_OFFSET       (8)
#define REG_CONFIG_QUAD_SPI_OFFSET       (9)
#define REG_CONFIG_DIRECT_IO_OFFSET      (11)
#define REG_CONFIG_AHB_MEM_OFFSET        (12)
#define REG_CONFIG_EILM_MEM_OFFSET       (13)
#define REG_CONFIG_SLV_OFFSET            (14)

#define REG_CONFIG_RX_FIFO_SIZE_MSK      (0x3 << REG_CONFIG_RX_FIFO_SIZE_OFFSET)
#define REG_CONFIG_TX_FIFO_SIZE_MSK      (0x3 << REG_CONFIG_TX_FIFO_SIZE_OFFSET)
#define REG_CONFIG_DUAL_SPI_MSK          (0x1 << REG_CONFIG_DUAL_SPI_OFFSET)
#define REG_CONFIG_QUAD_SPI_MSK          (0x1 << REG_CONFIG_QUAD_SPI_OFFSET)
#define REG_CONFIG_DIRECT_IO_MSK         (0x1 << REG_CONFIG_DIRECT_IO_OFFSET)
#define REG_CONFIG_AHB_MEM_MSK           (0x1 << REG_CONFIG_AHB_MEM_OFFSET)
#define REG_CONFIG_EILM_MEM_MSK          (0x1 << REG_CONFIG_EILM_MEM_OFFSET)
#define REG_CONFIG_SLV_MSK               (0x1 << REG_CONFIG_SLV_OFFSET)

/* Field mask of SPI control register */
#define REG_CTRL_SPI_RST_OFFSET          (0)
#define REG_CTRL_RX_FIFO_RST_OFFSET      (1)
#define REG_CTRL_TX_FIFO_RST_OFFSET      (2)
#define REG_CTRL_RX_DMA_EN_OFFSET        (3)
#define REG_CTRL_TX_DMA_EN_OFFSET        (4)
#define REG_CTRL_RX_THRES_OFFSET         (8)
#define REG_CTRL_TX_THRES_OFFSET         (16)

#define REG_CTRL_SPI_RST_MSK             (0x1 << REG_CTRL_SPI_RST_OFFSET)
#define REG_CTRL_RX_FIFO_RST_MSK         (0x1 << REG_CTRL_RX_FIFO_RST_OFFSET)
#define REG_CTRL_TX_FIFO_RST_MSK         (0x1 << REG_CTRL_TX_FIFO_RST_OFFSET)
#define REG_CTRL_RX_DMA_EN_MSK           (0x1 << REG_CTRL_RX_DMA_EN_OFFSET)
#define REG_CTRL_TX_DMA_EN_MSK           (0x1 << REG_CTRL_TX_DMA_EN_OFFSET)
#define REG_CTRL_RX_THRES_MSK            (0x1f << REG_CTRL_RX_THRES_OFFSET)
#define REG_CTRL_TX_THRES_MSK            (0x1f << REG_CTRL_TX_THRES_OFFSET)

#define INWORD(x)     sys_read32(x)
#define OUTWORD(x, d) sys_write32(d, x)


#define TX_FIFO_THRESHOLD                (1)
#define RX_FIFO_THRESHOLD                (1)

typedef void (*atcspi200_cfg_func_t)(void);

struct spi_atcspi200_dev_data_t {
	struct spi_context ctx;
	uint32_t tx_fifo_size;
	uint32_t rx_fifo_size;
	int send_rcv_diff;
};

struct spi_atcspi200_device_config {
	atcspi200_cfg_func_t spi_cfg_func;
	uint32_t spi_base_addr;
	uint32_t spi_irq_num;
	uint32_t f_sys;
};

/* API Functions */
static int spi_config(const struct device *dev,
                          const struct spi_config *config)
{
	const struct spi_atcspi200_device_config * const dev_cfg = SPI_DEV_CFG(dev);
	uint32_t sclk_div, data_len;

	// Set the divisor for SPI interface sclk
	sclk_div = (dev_cfg->f_sys / (config->frequency << 1)) - 1;
	OUTWORD(SPI_TIMIN(dev), (INWORD(SPI_TIMIN(dev)) & ~0xff));
	OUTWORD(SPI_TIMIN(dev), (INWORD(SPI_TIMIN(dev)) | sclk_div) );

	// Set Master mode
	OUTWORD(SPI_TFMAT(dev), INWORD(SPI_TFMAT(dev)) & ~(REG_TFMAT_SLVMODE_MSK|REG_TFMAT_DATA_MERGE_MSK));

	// Set data length
	data_len = SPI_WORD_SIZE_GET(config->operation) - 1;
	OUTWORD(SPI_TFMAT(dev), INWORD(SPI_TFMAT(dev)) & ~REG_TFMAT_DATA_LEN_MSK);
	OUTWORD(SPI_TFMAT(dev), INWORD(SPI_TFMAT(dev)) | (data_len << REG_TFMAT_DATA_LEN_OFFSET));

	// Set SPI frame format
	if (config->operation & SPI_MODE_CPHA)
		OUTWORD(SPI_TFMAT(dev), INWORD(SPI_TFMAT(dev)) | REG_TFMAT_CPHA_MSK);
	else
		OUTWORD(SPI_TFMAT(dev), INWORD(SPI_TFMAT(dev)) & ~REG_TFMAT_CPHA_MSK);

	if (config->operation & SPI_MODE_CPOL)
		OUTWORD(SPI_TFMAT(dev), INWORD(SPI_TFMAT(dev)) | REG_TFMAT_CPOL_MSK);
	else
		OUTWORD(SPI_TFMAT(dev), INWORD(SPI_TFMAT(dev)) & ~REG_TFMAT_CPOL_MSK);

	// Set SPI bit order
	if (config->operation & SPI_TRANSFER_LSB)
		OUTWORD(SPI_TFMAT(dev), INWORD(SPI_TFMAT(dev)) | REG_TFMAT_LSB_MSK);
	else
		OUTWORD(SPI_TFMAT(dev), INWORD(SPI_TFMAT(dev)) & ~REG_TFMAT_LSB_MSK);

	// Set TX/RX FIFO threshold
	OUTWORD(SPI_CTRL(dev), INWORD(SPI_CTRL(dev)) &  ~(REG_CTRL_RX_THRES_MSK | REG_CTRL_TX_THRES_MSK));
	OUTWORD(SPI_CTRL(dev), INWORD(SPI_CTRL(dev)) |  \
		((RX_FIFO_THRESHOLD << REG_CTRL_RX_THRES_OFFSET) | (TX_FIFO_THRESHOLD << REG_CTRL_TX_THRES_OFFSET)));

	return 0;
}

static uint32_t spi_compute_bufs_len(const struct spi_buf *bufs,
                                   size_t count, uint8_t dfs)
{
	uint32_t len = 0U;

	if(!bufs || !count)
		return 0;

	for (; count; bufs++, count--)
		len += bufs->len;

	return (len / dfs);
}

static void spi_send(const struct device *dev, uint32_t len) {

	OUTWORD(SPI_TCTRL(dev), (TRNS_MODE_WRITE_ONLY << REG_TCTRL_TRNS_MODE_OFFSET) |  \
				((len-1) << REG_TCTRL_WR_TCNT_OFFSET)                   \
				);
	// Enable TX FIFO interrupts
	OUTWORD(SPI_INTEN(dev), REG_INTEN_TX_FIFO_IEN_MSK | REG_INTEN_END_IEN_MSK);

	// Start transfering
	OUTWORD(SPI_CMD(dev), 0);

	return;
}

static void spi_receive(const struct device *dev, uint32_t len) {

	OUTWORD(SPI_TCTRL(dev), (TRNS_MODE_READ_ONLY << REG_TCTRL_TRNS_MODE_OFFSET) |  \
				((len-1) << REG_TCTRL_RD_TCNT_OFFSET)                   \
				);
	// Enable RX FIFO interrupts
	OUTWORD(SPI_INTEN(dev), REG_INTEN_RX_FIFO_IEN_MSK | REG_INTEN_END_IEN_MSK);

	// Start transfering
	OUTWORD(SPI_CMD(dev), 0);

	return;
}

static void spi_transfer(const struct device *dev, uint32_t len) {

	struct spi_atcspi200_dev_data_t* const dev_data = SPI_DEV_DATA(dev);

	dev_data->send_rcv_diff = 0;

	OUTWORD(SPI_TCTRL(dev), (TRNS_MODE_WRITE_READ << REG_TCTRL_TRNS_MODE_OFFSET) |  \
				((len-1) << REG_TCTRL_WR_TCNT_OFFSET) |                 \
				((len-1) << REG_TCTRL_RD_TCNT_OFFSET)                   \
					);
	// Enable TX/RX FIFO interrupts
	OUTWORD(SPI_INTEN(dev), REG_INTEN_TX_FIFO_IEN_MSK | REG_INTEN_RX_FIFO_IEN_MSK | REG_INTEN_END_IEN_MSK);


	// Start transfering
	OUTWORD(SPI_CMD(dev), 0);

	return;
}


static int transceive(const struct device *dev,
			  const struct spi_config *config,
			  const struct spi_buf_set *tx_bufs,
			  const struct spi_buf_set *rx_bufs)
{
	struct spi_atcspi200_dev_data_t* const dev_data = SPI_DEV_DATA(dev);
	struct spi_context *ctx = &dev_data->ctx;
	int rc = 0;
	uint32_t tx_len = 0, rx_len = 0;

	if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER)
		return -EINVAL;

	if (config->operation & SPI_MODE_LOOP)
		return -EINVAL;

	if ((config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE)
		return -EINVAL;

	if(tx_bufs)
		tx_len = spi_compute_bufs_len(tx_bufs->buffers, tx_bufs->count, SPI_WORD_SIZE_GET(ctx->config->operation)>>3);

	if(rx_bufs)
		rx_len = spi_compute_bufs_len(rx_bufs->buffers, rx_bufs->count, SPI_WORD_SIZE_GET(ctx->config->operation)>>3);

	if (tx_len > 512 || rx_len > 512 || !MAX(tx_len, rx_len)) {
		rc = -EINVAL;
		goto out;
	}
	// Set configuration to SPI context
	ctx->config = config;

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, SPI_WORD_SIZE_GET(ctx->config->operation)>>3);

	// SPI configuration
	spi_config(dev, config);

	if (!rx_bufs || !rx_bufs->buffers || !rx_len) {
		spi_send(dev, tx_len);
	} else if (!tx_bufs || !tx_bufs->buffers || !tx_len) {
		spi_receive(dev, rx_len);
	} else {
		spi_transfer(dev, MAX(tx_len, rx_len));
	}

	rc = spi_context_wait_for_completion(ctx);

out:
	spi_context_release(ctx, rc);

	return rc;
}

int spi_atcspi200_transceive(const struct device *dev,
			const struct spi_config *config,
			const struct spi_buf_set *tx_bufs,
			const struct spi_buf_set *rx_bufs)
{
	struct spi_atcspi200_dev_data_t* const dev_data = SPI_DEV_DATA(dev);
	struct spi_context *ctx = &dev_data->ctx;

	spi_context_lock(ctx, false, NULL);

	return transceive(dev, config, tx_bufs, rx_bufs);
}

#ifdef CONFIG_SPI_ASYNC
int spi_atcspi200_transceive_async(const struct device *dev,
				const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs,
				struct k_poll_signal *async)
{
	struct spi_atcspi200_dev_data_t* const dev_data = SPI_DEV_DATA(dev);
	struct spi_context *ctx = &dev_data->ctx;

	spi_context_lock(ctx, true, async);

	return transceive(dev, config, tx_bufs, rx_bufs);
}
#endif

int spi_atcspi200_release(const struct device *dev, const struct spi_config *config)
{

	struct spi_atcspi200_dev_data_t* const dev_data = SPI_DEV_DATA(dev);

	spi_context_unlock_unconditionally(&dev_data->ctx);

	return 0;
}

int spi_atcspi200_init(const struct device *dev)
{
	const struct spi_atcspi200_device_config * const dev_cfg = SPI_DEV_CFG(dev);
	struct spi_atcspi200_dev_data_t* const dev_data = SPI_DEV_DATA(dev);

	spi_context_unlock_unconditionally(&dev_data->ctx);

	// Get the TX/RX FIFO size of this device
	dev_data->tx_fifo_size = (2 << ((INWORD(SPI_CONFIG(dev)) & REG_CONFIG_TX_FIFO_SIZE_MSK) >> REG_CONFIG_TX_FIFO_SIZE_OFFSET));
	dev_data->rx_fifo_size = (2 << ((INWORD(SPI_CONFIG(dev)) & REG_CONFIG_RX_FIFO_SIZE_MSK) >> REG_CONFIG_RX_FIFO_SIZE_OFFSET));

	dev_cfg->spi_cfg_func();

	irq_enable(dev_cfg->spi_irq_num);

	return 0;
}
/* Device Instantiation */

static struct spi_driver_api spi_atcspi200_api = {
	.transceive = spi_atcspi200_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_atcspi200_transceive_async,
#endif
	.release = spi_atcspi200_release
};

static void spi_atcspi200_irq_handler(void *arg)
{
	const struct device * const dev = (const struct device *) arg;
	struct spi_atcspi200_dev_data_t* const dev_data = SPI_DEV_DATA(dev);
	struct spi_context *ctx = &dev_data->ctx;
	int status = 0;
	uint32_t tx_data = 0, tx_num = 0;
	uint32_t i, intr_status, rx_data, cur_tx_fifo_num, cur_rx_fifo_num, spi_status;

	intr_status = INWORD(SPI_INTST(dev));

	if ((intr_status & REG_INTST_TX_FIFO_INT_MSK)&&!(intr_status & REG_INTST_END_INT_MSK)) {

		spi_status = INWORD(SPI_STAT(dev));
		cur_rx_fifo_num = (spi_status >> 8) & 0x1f;
		cur_tx_fifo_num = (spi_status >> 16) & 0x1f;

		tx_num = dev_data->tx_fifo_size - cur_tx_fifo_num;

		for (i=tx_num; i > 0; i--) {

			if (spi_context_tx_buf_on(ctx)) {

				switch (SPI_WORD_SIZE_GET(ctx->config->operation)) {
				case 8:
					tx_data = (uint8_t)*ctx->tx_buf;
					break;
				case 16:
					tx_data = (uint16_t)*ctx->tx_buf;
					break;
				}

			} else if (spi_context_rx_on(ctx) || spi_context_tx_on(ctx)) {
				tx_data = 0;
			} else {
				OUTWORD(SPI_INTEN(dev), INWORD(SPI_INTEN(dev)) & ~REG_INTEN_TX_FIFO_IEN_MSK);
				break;
			}

			if( INWORD(SPI_TCTRL(dev)) & (TRNS_MODE_WRITE_READ << REG_TCTRL_TRNS_MODE_OFFSET) )
				if(dev_data->send_rcv_diff >= dev_data->rx_fifo_size)
					break;

			OUTWORD(SPI_DATA(dev), tx_data);

			switch (SPI_WORD_SIZE_GET(ctx->config->operation)) {
			case 8:
				spi_context_update_tx(ctx, 1, 1);
				break;
			case 16:
				spi_context_update_tx(ctx, 2, 1);
				break;
			}

			dev_data->send_rcv_diff++;
		}
		OUTWORD(SPI_INTST(dev), REG_INTST_TX_FIFO_INT_MSK);

	}

	if (intr_status & REG_INTST_RX_FIFO_INT_MSK) {
		cur_rx_fifo_num = ((INWORD(SPI_STAT(dev)) >> 8) & 0x1f);

		for (i=cur_rx_fifo_num; i > 0; i--) {

			rx_data = INWORD(SPI_DATA(dev));

			dev_data->send_rcv_diff--;

			if (spi_context_rx_buf_on(ctx)) {

				switch (SPI_WORD_SIZE_GET(ctx->config->operation)) {
				case 8:
					*ctx->rx_buf = (uint8_t)rx_data;
					break;
				case 16:
					*ctx->rx_buf = (uint16_t)rx_data;
					break;
				}

			} else if (spi_context_tx_on(ctx) || spi_context_rx_on(ctx)) {
				// Still need to read data but skip them
			} else {
				OUTWORD(SPI_INTEN(dev), INWORD(SPI_INTEN(dev)) & ~REG_INTEN_RX_FIFO_IEN_MSK);
			}

			switch (SPI_WORD_SIZE_GET(ctx->config->operation)) {
			case 8:
				spi_context_update_rx(ctx, 1, 1);
				break;
			case 16:
				spi_context_update_rx(ctx, 2, 1);
				break;
			}
		}
		OUTWORD(SPI_INTST(dev), REG_INTST_RX_FIFO_INT_MSK);
	}

	if (intr_status & REG_INTST_END_INT_MSK) {

		// Reset TX/RX FIFO
		OUTWORD(SPI_CTRL(dev),  INWORD(SPI_CTRL(dev)) | (REG_CTRL_TX_FIFO_RST_MSK | REG_CTRL_RX_FIFO_RST_MSK));

		// Clear end interrupt
		OUTWORD(SPI_INTST(dev), REG_INTST_END_INT_MSK);

		// Disable all SPI interrupts
		OUTWORD(SPI_INTEN(dev), 0);

		spi_context_complete(ctx, status);
        }

	return;
}

#define SPI_INIT(n)                                                                     \
        static struct spi_atcspi200_dev_data_t spi_atcspi200_dev_data_##n = {           \
                SPI_CONTEXT_INIT_LOCK(spi_atcspi200_dev_data_##n, ctx),                 \
                SPI_CONTEXT_INIT_SYNC(spi_atcspi200_dev_data_##n, ctx),                 \
        };                                                                              \
        static void spi_atcspi200_cfg_##n(void);                                        \
        static struct spi_atcspi200_device_config spi_atcspi200_dev_cfg_##n = {         \
                .spi_cfg_func = spi_atcspi200_cfg_##n,                                  \
                .spi_base_addr = DT_INST_REG_ADDR(n),                                   \
                .spi_irq_num = DT_INST_IRQN(n),                                         \
                .f_sys = DT_INST_PROP(n, clock_frequency),                              \
        };                                                                              \
        DEVICE_AND_API_INIT(spi_atcspi200_##n,                                          \
                        DT_INST_LABEL(n),                                               \
                        spi_atcspi200_init,                                             \
                        &spi_atcspi200_dev_data_##n,                                    \
                        &spi_atcspi200_dev_cfg_##n,                                     \
                        POST_KERNEL,                                                    \
                        CONFIG_SPI_INIT_PRIORITY,                                       \
                        &spi_atcspi200_api);                                            \
        static void spi_atcspi200_cfg_##n(void)                                         \
        {                                                                               \
                                                                                        \
                IRQ_CONNECT(    DT_INST_IRQN(n),                                        \
                                DT_INST_IRQ(n, priority),                               \
                                spi_atcspi200_irq_handler,                              \
                                DEVICE_GET(spi_atcspi200_##n),                          \
                                0);                                                     \
                                                                                        \
        }                                                                               \




#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi0), okay)
#if !CONFIG_ANDES_ATCSPI200_SPI_0_ROM
SPI_INIT(0)
#endif
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi1), okay)
SPI_INIT(1)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi2), okay)
SPI_INIT(2)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi3), okay)
SPI_INIT(3)
#endif
