/*
 * Copyright (c) 2019 Andes Technology Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file GPIO driver for AndesTech atcgpio100 IP
 */

#include <errno.h>
#include <kernel.h>
#include <device.h>
#include <soc.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/sys_io.h>

#include "gpio_utils.h"

#define DT_DRV_COMPAT andestech_atcgpio100

/* Andes ATCGPIO100 register definition */
#define REG_IDR  0x00 /* ID and Revision reg.           */
#define REG_CFG  0x10 /* Hardware configure reg.        */
#define REG_DIN  0x20 /* Data In reg.                   */
#define REG_DOUT 0x24 /* Data Out reg.                  */
#define REG_DIR  0x28 /* Channel direction reg.         */
#define REG_DCLR 0x2C /* Data out clear reg.            */
#define REG_DSET 0x30 /* Data out set reg.              */
#define REG_PUEN 0x40 /* Pull enable reg.               */
#define REG_PTYP 0x44 /* Pull type reg.                 */
#define REG_INTE 0x50 /* Interrupt enable reg.          */
#define REG_IMD0 0x54 /* Interrupt mode 0 ~ 7 reg.      */
#define REG_IMD1 0x58 /* Interrupt mode 8 ~ 15 reg.     */
#define REG_IMD2 0x5C /* Interrupt mode 16 ~ 23 reg.    */
#define REG_IMD3 0x60 /* Interrupt mode 24 ~ 31 reg.    */
#define REG_ISTA 0x64 /* Interrupt status reg.          */
#define REG_DEBE 0x70 /* De-bounce enable reg.          */
#define REG_DEBC 0x74 /* De-Bounce control reg.         */

/* Can be declared in device tree*/
#define MAX_PIN_NUMBER		31

#define INT_NO_OPERATION        0x0
#define INT_HIGH_LEVEL          0x2
#define INT_LOW_LEVEL           0x3
#define INT_NEGATIVE_EDGE       0x5
#define INT_POSITIVE_EDGE       0x6
#define INT_DUAL_EDGE           0x7

#define PULL_CONFIGURED         (1 << 31)
#define DEBOUNCE_CONFIGURED     (1 << 29)
#define DF_DEBOUNCED_SETTING    (0x80000003)

/* Helper Macros for GPIO */
#define DEV_GPIO_CFG(dev)						\
	((const struct gpio_atcgpio100_device_config* const)(dev)->config)

#define DEV_GPIO_DATA(dev)				\
	((struct gpio_atcgpio100_dev_data_t*)(dev)->data)


#define GPIO_CFG(dev)  (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_CFG)
#define GPIO_DIR(dev)  (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_DIR)
#define GPIO_DIN(dev)  (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_DIN)
#define GPIO_DOUT(dev) (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_DOUT)
#define GPIO_DCLR(dev) (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_DCLR)
#define GPIO_DSET(dev) (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_DSET)
#define GPIO_PUEN(dev) (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_PUEN)
#define GPIO_PTYP(dev) (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_PTYP)

#define GPIO_INTE(dev) (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_INTE)
#define GPIO_IMD0(dev) (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_IMD0)
#define GPIO_IMD1(dev) (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_IMD1)
#define GPIO_IMD2(dev) (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_IMD2)
#define GPIO_IMD3(dev) (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_IMD3)
#define GPIO_ISTA(dev) (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_ISTA)
#define GPIO_DEBE(dev) (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_DEBE)
#define GPIO_DEBC(dev) (DEV_GPIO_CFG(dev)->gpio_base_addr + REG_DEBC)

#define INWORD(x)     sys_read32(x)
#define OUTWORD(x, d) sys_write32(d, x)

typedef void (*atcgpio100_cfg_func_t)(void);

struct gpio_atcgpio100_device_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	uint32_t                  gpio_base_addr;
	uint32_t                  gpio_irq_num;
	atcgpio100_cfg_func_t	  gpio_cfg_func;
};

struct gpio_atcgpio100_dev_data_t {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* list of callbacks */
	sys_slist_t cb;
};

static int gpio_atcgpio100_config(const struct device *port,
				  gpio_pin_t pin,
				  gpio_flags_t flags)
{
	uint32_t reg, pin_mask, io_flags;

	/* Does not supporting both input/output at same time.
	 */
	io_flags = (GPIO_OUTPUT | GPIO_INPUT);
	if ((flags & io_flags) == io_flags)
		return -ENOTSUP;

	pin_mask = (1 << pin);

	if (flags & GPIO_OUTPUT) {

		/* Set the initial output value before enabling output
		   to avoid glitches
		 */
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			OUTWORD(GPIO_DSET(port), pin_mask);
		}
		if (flags & GPIO_OUTPUT_INIT_LOW) {
			OUTWORD(GPIO_DCLR(port), pin_mask);
		}

		/* Set channel output */
		reg = INWORD(GPIO_DIR(port));
		reg |= pin_mask;
		OUTWORD(GPIO_DIR(port), reg);

	} else if (flags & GPIO_INPUT) {

		if (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN))
			return -ENOTSUP;

		/* Set de-bounce */
		if (flags & GPIO_INT_DEBOUNCE) {
			/* Default settings: Filter out less than 4 pclk
			   de-bounce period clock
			 */
			OUTWORD(GPIO_DEBC(port), DF_DEBOUNCED_SETTING);
			OUTWORD(GPIO_DEBE(port), 0x1);
		}

		/* Set channel input */
		reg = INWORD(GPIO_DIR(port));
		reg &= ~pin_mask;
		OUTWORD(GPIO_DIR(port), reg);

	} else
		/* GPIO disconnected condition */
		return -ENOTSUP;

	return 0;
}

static int gpio_atcgpio100_port_get_raw(const struct device *port,
					gpio_port_value_t *value)
{
	*value = INWORD(GPIO_DIN(port));
	return 0;
}

static int gpio_atcgpio100_set_masked_raw(const struct device *port,
					  gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	uint32_t reg;
	reg = INWORD(GPIO_DOUT(port)) & ~mask;
	OUTWORD(GPIO_DOUT(port), reg | (value & mask));

	return 0;
}

static int gpio_atcgpio100_set_bits_raw(const struct device *port,
					gpio_port_pins_t pins)
{
	OUTWORD(GPIO_DSET(port), pins);
	return 0;
}

static int gpio_atcgpio100_clear_bits_raw(const struct device *port,
					gpio_port_pins_t pins)
{
	OUTWORD(GPIO_DCLR(port), pins);
	return 0;
}

static int gpio_atcgpio100_toggle_bits(const struct device *port,
					gpio_port_pins_t pins)
{
	uint32_t reg;
	reg = INWORD(GPIO_DOUT(port));
	OUTWORD(GPIO_DOUT(port), reg ^ pins);

	return 0;
}

static int gpio_atcgpio100_pin_interrupt_configure(
					const struct device *port,
					gpio_pin_t pin,
					enum gpio_int_mode mode,
					enum gpio_int_trig trig)
{
	uint32_t reg, int_mode, int_mode_reg_num, int_mode_reg_ch;

	/* Set interrupt configuration */
	switch(mode | trig) {
	case GPIO_INT_EDGE_BOTH:
		int_mode = INT_DUAL_EDGE;
		break;
	case GPIO_INT_EDGE_RISING:
		int_mode = INT_POSITIVE_EDGE;
		break;
	case GPIO_INT_EDGE_FALLING:
		int_mode = INT_NEGATIVE_EDGE;
		break;
	case GPIO_INT_LEVEL_LOW:
		int_mode = INT_LOW_LEVEL;
		break;
	case GPIO_INT_LEVEL_HIGH:
		int_mode = INT_HIGH_LEVEL;
		break;
	default:
		int_mode = INT_NO_OPERATION;
	}

	int_mode_reg_num = (pin >> 3);
	int_mode_reg_ch = (pin & 0x7);

	switch(int_mode_reg_num) {
	case 0:
		reg = INWORD(GPIO_IMD0(port));
		reg &= ~(0x7 << (int_mode_reg_ch << 2));
		reg |= (int_mode << (int_mode_reg_ch << 2));
		OUTWORD(GPIO_IMD0(port), reg);
		break;
	case 1:
		reg = INWORD(GPIO_IMD1(port));
		reg &= ~(0x7 << (int_mode_reg_ch << 2));
		reg |= (int_mode << (int_mode_reg_ch << 2));
		OUTWORD(GPIO_IMD1(port), reg);
		break;
	case 2:
		reg = INWORD(GPIO_IMD2(port));
		reg &= ~(0x7 << (int_mode_reg_ch << 2));
		reg |= (int_mode << (int_mode_reg_ch << 2));
		OUTWORD(GPIO_IMD2(port), reg);
		break;
	case 3:
		reg = INWORD(GPIO_IMD3(port));
		reg &= ~(0x7 << (int_mode_reg_ch << 2));
		reg |= (int_mode << (int_mode_reg_ch << 2));
		OUTWORD(GPIO_IMD3(port), reg);
		break;
	default:
		return -EINVAL;
	}


	if (int_mode == INT_NO_OPERATION) {
		/* Disable interrupt*/
		reg = INWORD(GPIO_INTE(port));
		reg &= ~(1 << pin);
		OUTWORD(GPIO_INTE(port), reg);

		/* Clear the remain pending interrupt */
		reg = INWORD(GPIO_ISTA(port));
		OUTWORD(GPIO_ISTA(port), reg);
	} else {
		reg = INWORD(GPIO_INTE(port));
		reg |= (1 << pin);
		OUTWORD(GPIO_INTE(port), reg);
	}
	return 0;
}

static int gpio_atcgpio100_manage_callback(const struct device *port,
                                      struct gpio_callback *callback,
                                      bool set)
{

	struct gpio_atcgpio100_dev_data_t * const data = DEV_GPIO_DATA(port);

	return gpio_manage_callback(&data->cb, callback, set);
}

static void gpio_atcgpio100_irq_handler(const struct device *port)
{
	struct gpio_atcgpio100_dev_data_t * const data = DEV_GPIO_DATA(port);
	uint32_t reg;

	reg = INWORD(GPIO_ISTA(port));
	OUTWORD(GPIO_ISTA(port), reg);

	gpio_fire_callbacks(&data->cb, port, reg);

	return;
}

static const struct gpio_driver_api gpio_atcgpio100_driver = {
	.pin_configure           = gpio_atcgpio100_config,
	.port_get_raw            = gpio_atcgpio100_port_get_raw,
	.port_set_masked_raw     = gpio_atcgpio100_set_masked_raw,
	.port_set_bits_raw       = gpio_atcgpio100_set_bits_raw,
	.port_clear_bits_raw     = gpio_atcgpio100_clear_bits_raw,
	.port_toggle_bits        = gpio_atcgpio100_toggle_bits,
	.pin_interrupt_configure = gpio_atcgpio100_pin_interrupt_configure,
	.manage_callback         = gpio_atcgpio100_manage_callback
};

static int gpio_atcgpio100_init(const struct device *port)
{
	const struct gpio_atcgpio100_device_config * const dev_cfg = DEV_GPIO_CFG(port);

	// Disable all interrupts
	OUTWORD(GPIO_INTE(port), 0x00000000);

	// Write 1 to clear interrupt status
	OUTWORD(GPIO_ISTA(port), 0xFFFFFFFF);

	// Configure GPIO device
	dev_cfg->gpio_cfg_func();

	// Enable PLIC interrupt GPIO source
	irq_enable(dev_cfg->gpio_irq_num);

	return 0;
}

static void gpio_atcgpio100_cfg_func_0(void);
static struct gpio_atcgpio100_dev_data_t gpio_atcgpio100_dev_data_0;

static const struct gpio_atcgpio100_device_config gpio_atcgpio100_config_0 = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(0),
	},
	.gpio_base_addr = DT_INST_REG_ADDR(0),
	.gpio_irq_num = DT_INST_IRQN(0),
	.gpio_cfg_func = gpio_atcgpio100_cfg_func_0

};

DEVICE_AND_API_INIT(	gpio_atcgpio100_0, DT_INST_LABEL(0),			\
			gpio_atcgpio100_init,					\
			&gpio_atcgpio100_dev_data_0, &gpio_atcgpio100_config_0,	\
			POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			&gpio_atcgpio100_driver);

static void gpio_atcgpio100_cfg_func_0(void)
{
	IRQ_CONNECT(	DT_INST_IRQN(0), 		\
			DT_INST_IRQ(0, priority), 	\
			gpio_atcgpio100_irq_handler, 	\
			DEVICE_GET(gpio_atcgpio100_0),	\
			0);
	return;
}
