/*
 * Copyright (c) 2019 Andes Technology
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/counter.h>
#include <string.h>

#define DT_DRV_COMPAT andestech_atcpit100

/* register definitions */
#define REG_IDR   0x00 /* ID and Revision Reg.        */
#define REG_CFG   0x10 /* Configuration Reg.          */
#define REG_INTE  0x14 /* Interrupt Enable Reg.       */
#define REG_ISTA  0x18 /* Interrupt Status Reg.       */
#define REG_CHEN  0x1C /* Channel Enable Reg.         */
#define REG_CTRL0 0x20 /* Channel 0 Control Reg.      */
#define REG_RELD0 0x24 /* Channle 0 Reload Reg.       */
#define REG_CNTR0 0x28 /* Channel 0 Counter Reg.      */
#define REG_CTRL1 0x30 /* Channel 1 Control Reg.      */
#define REG_RELD1 0x34 /* Channle 1 Reload Reg.       */
#define REG_CNTR1 0x38 /* Channel 1 Counter Reg.      */
#define REG_CTRL2 0x40 /* Channel 2 Control Reg.      */
#define REG_RELD2 0x44 /* Channle 2 Reload Reg.       */
#define REG_CNTR2 0x48 /* Channel 2 Counter Reg.      */
#define REG_CTRL3 0x50 /* Channel 3 Control Reg.      */
#define REG_RELD3 0x54 /* Channle 3 Reload Reg.       */
#define REG_CNTR3 0x58 /* Channel 3 Counter Reg.      */

#define PIT_CHNCTRL_CLK_EXTERNAL   (0 << 3)
#define PIT_CHNCTRL_CLK_PCLK       (1 << 3)

#define PIT_CHNCTRL_TMR_32BIT      1
#define PIT_CHNCTRL_TMR_16BIT      2

#define PIT_CH0TMR0                (1 << 0)
#define PIT_CH1TMR0                (1 << 4)
#define PIT_CH2TMR0                (1 << 8)
#define PIT_CH3TMR0                (1 << 12)

#define PIT_DEV_CFG(dev)				\
	((const struct counter_atcpit100_device_config * const) \
	 (dev)->config)
#define PIT_DEV_DATA(dev)			\
	((struct counter_atcpit100_dev_data_t *)(dev)->data)

#define PIT_IDR(dev)   (PIT_DEV_CFG(dev)->pit_base_addr + REG_IDR)
#define PIT_CFG(dev)   (PIT_DEV_CFG(dev)->pit_base_addr + REG_CFG)
#define PIT_INTE(dev)  (PIT_DEV_CFG(dev)->pit_base_addr + REG_INTE)
#define PIT_ISTA(dev)  (PIT_DEV_CFG(dev)->pit_base_addr + REG_ISTA)
#define PIT_CHEN(dev)  (PIT_DEV_CFG(dev)->pit_base_addr + REG_CHEN)
#define PIT_CTRL0(dev) (PIT_DEV_CFG(dev)->pit_base_addr + REG_CTRL0)
#define PIT_RELD0(dev) (PIT_DEV_CFG(dev)->pit_base_addr + REG_RELD0)
#define PIT_CNTR0(dev) (PIT_DEV_CFG(dev)->pit_base_addr + REG_CNTR0)
#define PIT_CTRL1(dev) (PIT_DEV_CFG(dev)->pit_base_addr + REG_CTRL1)
#define PIT_RELD1(dev) (PIT_DEV_CFG(dev)->pit_base_addr + REG_RELD1)
#define PIT_CNTR1(dev) (PIT_DEV_CFG(dev)->pit_base_addr + REG_CNTR1)
#define PIT_CTRL2(dev) (PIT_DEV_CFG(dev)->pit_base_addr + REG_CTRL2)
#define PIT_RELD2(dev) (PIT_DEV_CFG(dev)->pit_base_addr + REG_RELD2)
#define PIT_CNTR2(dev) (PIT_DEV_CFG(dev)->pit_base_addr + REG_CNTR2)
#define PIT_CTRL3(dev) (PIT_DEV_CFG(dev)->pit_base_addr + REG_CTRL3)
#define PIT_RELD3(dev) (PIT_DEV_CFG(dev)->pit_base_addr + REG_RELD3)
#define PIT_CNTR3(dev) (PIT_DEV_CFG(dev)->pit_base_addr + REG_CNTR3)

#define INWORD(x)     sys_read32(x)
#define OUTWORD(x, d) sys_write32(d, x)

#define CH_NUM_PER_PIT      4
#define CH_NUM_PER_COUNTER  (CH_NUM_PER_PIT - 1)
#define TOP_CHANNEL         CH_NUM_PER_COUNTER
#define TOP_VALUE_MAX       UINT32_MAX

#error "Not using this code in QEMU RiSCv32"

typedef void (*atcpit100_cfg_func_t)(void);

struct counter_atcpit100_device_config {
	struct counter_config_info counter_info;
	uint32_t pit_base_addr;
	uint32_t pit_irq_num;
	atcpit100_cfg_func_t counter_cfg_func;
};

struct counter_atcpit100_dev_ch_data {
	counter_alarm_callback_t alarm_callback;
	void *alarm_user_data;
};

struct counter_atcpit100_dev_data_t {
	counter_top_callback_t   top_callback;
	void *top_user_data;
	uint32_t guard_period;
	struct counter_atcpit100_dev_ch_data ch_data[CH_NUM_PER_COUNTER];
};

static void counter_atcpit100_irq_handler(void *arg)
{
	struct device *dev = (struct device*)arg;
	struct counter_atcpit100_dev_data_t * const dev_data = PIT_DEV_DATA(dev);
	uint32_t int_status, current;

	int_status = INWORD(PIT_ISTA(dev));

	if (int_status & PIT_CH3TMR0) {
		if (dev_data->top_callback) {
			dev_data->top_callback(dev, dev_data->top_user_data);
		}
	}

	if (int_status & PIT_CH0TMR0) {

		counter_alarm_callback_t cb = dev_data->ch_data[0].alarm_callback;
		dev_data->ch_data[0].alarm_callback = NULL;

		OUTWORD(PIT_INTE(dev), INWORD(PIT_INTE(dev)) & ~PIT_CH0TMR0);
		OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) & ~PIT_CH0TMR0);

		current = INWORD(PIT_RELD3(dev)) - INWORD(PIT_CNTR3(dev));
		cb(dev, 0, current, dev_data->ch_data[0].alarm_user_data);

	}

	if (int_status & PIT_CH1TMR0) {

		counter_alarm_callback_t cb = dev_data->ch_data[1].alarm_callback;
		dev_data->ch_data[1].alarm_callback = NULL;

		OUTWORD(PIT_INTE(dev), INWORD(PIT_INTE(dev)) & ~PIT_CH1TMR0);
		OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) & ~PIT_CH1TMR0);

		current = INWORD(PIT_RELD3(dev)) - INWORD(PIT_CNTR3(dev));
		cb(dev, 1, current, dev_data->ch_data[1].alarm_user_data);

	}

	if (int_status & PIT_CH2TMR0) {

		counter_alarm_callback_t cb = dev_data->ch_data[2].alarm_callback;
		dev_data->ch_data[2].alarm_callback = NULL;

		OUTWORD(PIT_INTE(dev), INWORD(PIT_INTE(dev)) & ~PIT_CH2TMR0);
		OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) & ~PIT_CH2TMR0);

		current = INWORD(PIT_RELD3(dev)) - INWORD(PIT_CNTR3(dev));
		cb(dev, 2, current, dev_data->ch_data[2].alarm_user_data);

	}

	OUTWORD(PIT_ISTA(dev), int_status);

	return;
}

static int counter_atcpit100_init(const struct device *dev)
{
	const struct counter_atcpit100_device_config * const dev_cfg = PIT_DEV_CFG(dev);

	/* Disable all channels of PIT */
	OUTWORD(PIT_CHEN(dev), 0);

	/* Channel 0 ~ 3, 32 bits timer, PCLK source */
	OUTWORD(PIT_CTRL0(dev), PIT_CHNCTRL_TMR_32BIT | PIT_CHNCTRL_CLK_PCLK);
	OUTWORD(PIT_CTRL1(dev), PIT_CHNCTRL_TMR_32BIT | PIT_CHNCTRL_CLK_PCLK);
	OUTWORD(PIT_CTRL2(dev), PIT_CHNCTRL_TMR_32BIT | PIT_CHNCTRL_CLK_PCLK);
	OUTWORD(PIT_CTRL3(dev), PIT_CHNCTRL_TMR_32BIT | PIT_CHNCTRL_CLK_PCLK);

	/* Disable PIT interrupt, and clear all pending PIT interrupt */
	OUTWORD(PIT_INTE(dev), 0);
	OUTWORD(PIT_ISTA(dev), 0xFFFFFFFF);

	/* Set max top value to counter by default */
	/* Select channel 3 as the counter */
	OUTWORD(PIT_RELD3(dev), TOP_VALUE_MAX);

	dev_cfg->counter_cfg_func();

	irq_enable(dev_cfg->pit_irq_num);

	return 0;
}

static int counter_atcpit100_start(const struct device *dev)
{
	OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) | PIT_CH3TMR0);

	return 0;
}

static int counter_atcpit100_stop(const struct device *dev)
{
	OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) & ~PIT_CH3TMR0);

	return 0;
}

static int counter_atcpit100_get_value(const struct device *dev, uint32_t *ticks)
{
	*ticks = (INWORD(PIT_RELD3(dev)) - INWORD(PIT_CNTR3(dev)));
	return 0;
}

static int counter_atcpit100_set_alarm(const struct device *dev, uint8_t chan_id,
				const struct counter_alarm_cfg *alarm_cfg)
{

	struct counter_atcpit100_dev_data_t * const dev_data = PIT_DEV_DATA(dev);
	uint32_t top_value, cur_cnt, alarm_ticks;
	int err = 0;

	if (chan_id >= CH_NUM_PER_COUNTER)
		return -ENOTSUP;

	if (alarm_cfg->ticks > INWORD(PIT_RELD3(dev)))
		return -EINVAL;

	if (!alarm_cfg->callback)
		return -EINVAL;

	if(dev_data->ch_data[chan_id].alarm_callback)
		return -EBUSY;

	dev_data->ch_data[chan_id].alarm_callback = alarm_cfg->callback;
	dev_data->ch_data[chan_id].alarm_user_data = alarm_cfg->user_data;

	top_value = INWORD(PIT_RELD3(dev));
	cur_cnt = INWORD(PIT_CNTR3(dev));

	if (alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) {
		if (alarm_cfg->ticks > top_value - cur_cnt) {
			alarm_ticks = alarm_cfg->ticks - (top_value - cur_cnt) - 1;

		} else {
			if(alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE) {
				alarm_ticks = 1;
				err = -ETIME;
			} else {
				alarm_ticks =  cur_cnt + alarm_cfg->ticks - 1;

				if (alarm_ticks > top_value -  dev_data->guard_period) {
					dev_data->ch_data[chan_id].alarm_callback = NULL;
					dev_data->ch_data[chan_id].alarm_user_data = NULL;
					return -ETIME;
				}
			}
		}

	} else {
		alarm_ticks = alarm_cfg->ticks - 1;
	}

	switch(chan_id) {

	case 0:
		OUTWORD(PIT_RELD0(dev), alarm_ticks);
		OUTWORD(PIT_INTE(dev), INWORD(PIT_INTE(dev)) | PIT_CH0TMR0);
		OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) | PIT_CH0TMR0);
		break;

	case 1:
		OUTWORD(PIT_RELD1(dev), alarm_ticks);
		OUTWORD(PIT_INTE(dev), INWORD(PIT_INTE(dev)) | PIT_CH1TMR0);
		OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) | PIT_CH1TMR0);
		break;

	case 2:
		OUTWORD(PIT_RELD2(dev), alarm_ticks);
		OUTWORD(PIT_INTE(dev), INWORD(PIT_INTE(dev)) | PIT_CH2TMR0);
		OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) | PIT_CH2TMR0);
		break;

	default:
		return -ENOTSUP;

	}

	return err;
}

static int counter_atcpit100_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	switch(chan_id) {

	case 0:
		OUTWORD(PIT_INTE(dev), INWORD(PIT_INTE(dev)) & ~PIT_CH0TMR0);
		OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) & ~PIT_CH0TMR0);
		break;

        case 1:
		OUTWORD(PIT_INTE(dev), INWORD(PIT_INTE(dev)) & ~PIT_CH1TMR0);
		OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) & ~PIT_CH1TMR0);
		break;

        case 2:
		OUTWORD(PIT_INTE(dev), INWORD(PIT_INTE(dev)) & ~PIT_CH2TMR0);
		OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) & ~PIT_CH2TMR0);
		break;

	default:
		return -ENOTSUP;

	}

	return 0;
}

static int counter_atcpit100_set_top_value(const struct device *dev,
                                         const struct counter_top_cfg *cfg)
{
	struct counter_atcpit100_dev_data_t * const dev_data = PIT_DEV_DATA(dev);
	uint32_t count;
	uint8_t i;

	for (i = 0; i < counter_get_num_of_channels(dev); i++)
		if(dev_data->ch_data[i].alarm_callback)
			return -EBUSY;

	if (cfg->ticks > TOP_VALUE_MAX)
		return -ENOTSUP;

	if (cfg->callback) {

		OUTWORD(PIT_INTE(dev), INWORD(PIT_INTE(dev)) & ~PIT_CH3TMR0);

		dev_data->top_callback = cfg->callback;
		dev_data->top_user_data = cfg->user_data;
	}

	OUTWORD(PIT_RELD3(dev), cfg->ticks);

	if (cfg->flags & COUNTER_TOP_CFG_DONT_RESET) {
		counter_atcpit100_get_value(dev, &count);
		if ( count >= cfg->ticks ) {

			if ( cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE ) {

				// Reset counter
				OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) & ~PIT_CH3TMR0);
				OUTWORD(PIT_ISTA(dev), INWORD(PIT_ISTA(dev)) & ~PIT_CH3TMR0);

				OUTWORD(PIT_INTE(dev), INWORD(PIT_INTE(dev)) | PIT_CH3TMR0);
				OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) | PIT_CH3TMR0);

				return 0;
			}
			return -ETIME;
		}
	} else {

		// Reset counter
		OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) & ~PIT_CH3TMR0);
		OUTWORD(PIT_ISTA(dev), INWORD(PIT_ISTA(dev)) & ~PIT_CH3TMR0);

		OUTWORD(PIT_INTE(dev), INWORD(PIT_INTE(dev)) | PIT_CH3TMR0);
		OUTWORD(PIT_CHEN(dev), INWORD(PIT_CHEN(dev)) | PIT_CH3TMR0);
	}

	return 0;
}

static uint32_t counter_atcpit100_get_pending_int(const struct device *dev)
{
	return ( (INWORD(PIT_ISTA(dev)) & (PIT_CH0TMR0|PIT_CH1TMR0|	\
		PIT_CH2TMR0|PIT_CH3TMR0)) == 0 )			\
		 ? 0 : 1 ;
}

static uint32_t counter_atcpit100_get_top_value(const struct device *dev)
{
	return INWORD(PIT_RELD3(dev));
}

static uint32_t counter_atcpit100_get_max_relative_alarm(const struct device *dev)
{
	const struct counter_atcpit100_device_config * const dev_cfg = PIT_DEV_CFG(dev);
	return dev_cfg->counter_info.max_top_value;
}

static uint32_t counter_atcpit100_get_guard_period(const struct device *dev, uint32_t flags)
{
	struct counter_atcpit100_dev_data_t * const dev_data = PIT_DEV_DATA(dev);

	return dev_data->guard_period;
}

static int counter_atcpit100_set_guard_period(const struct device *dev, uint32_t ticks,
                                                uint32_t flags)
{

	struct counter_atcpit100_dev_data_t * const dev_data = PIT_DEV_DATA(dev);

	if (ticks > INWORD(PIT_RELD3(dev)))
		return -EINVAL;

	dev_data->guard_period = ticks;

	return 0;
}

static const struct counter_driver_api counter_atcpit100_driver_api = {
	.start = counter_atcpit100_start,
	.stop = counter_atcpit100_stop,
	.get_value = counter_atcpit100_get_value,
	.set_alarm = counter_atcpit100_set_alarm,
	.cancel_alarm = counter_atcpit100_cancel_alarm,
	.set_top_value = counter_atcpit100_set_top_value,
	.get_pending_int = counter_atcpit100_get_pending_int,
	.get_top_value = counter_atcpit100_get_top_value,
	.get_max_relative_alarm = counter_atcpit100_get_max_relative_alarm,
	.get_guard_period = counter_atcpit100_get_guard_period,
	.set_guard_period = counter_atcpit100_set_guard_period,
};

static void counter_atcpit100_cfg_0(void);

static const struct counter_atcpit100_device_config counter_atcpit100_dev_cfg_0 = {
	.counter_info = {
		.max_top_value = TOP_VALUE_MAX,
		.freq = DT_INST_PROP(0, clock_frequency),
		.flags = COUNTER_CONFIG_INFO_COUNT_UP,
		.channels = CH_NUM_PER_COUNTER,
	},
	.pit_base_addr = DT_INST_REG_ADDR(0),
	.pit_irq_num = DT_INST_IRQN(0),
	.counter_cfg_func = counter_atcpit100_cfg_0,
};

static struct counter_atcpit100_dev_data_t counter_atcpit100_dev_data_0;

DEVICE_AND_API_INIT(counter_atcpit100_0, DT_INST_LABEL(0),		\
		    counter_atcpit100_init,					\
		    &counter_atcpit100_dev_data_0, &counter_atcpit100_dev_cfg_0,	\
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,		\
		    &counter_atcpit100_driver_api);

static void counter_atcpit100_cfg_0(void)
{
	IRQ_CONNECT(	DT_INST_IRQN(0),			\
			DT_INST_IRQ(0, priority),		\
			counter_atcpit100_irq_handler,		\
			DEVICE_GET(counter_atcpit100_0),	\
			0);
}

