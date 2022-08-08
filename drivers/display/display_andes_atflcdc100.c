/*
 * Copyright (c) 2020 Andes Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <drivers/display.h>
#include <display/framebuf.h>
#include <logging/log.h>
#include "display_andes_atflcdc100.h"

#define DT_DRV_COMPAT andestech_atflcdc100

LOG_MODULE_REGISTER(display_andes_atflcdc100, CONFIG_DISPLAY_LOG_LEVEL);

#if defined(CONFIG_CACHE_ENABLE) && !defined(CONFIG_SOC_ANDES_V5_CM)

#ifdef CONFIG_NOCACHE_MEMORY
static pixel_t f_buffer[384000] __nocache __aligned(64);
#else
#error "You have to disable cache or enable nocache memory!!"
#endif

#else /* defined(CONFIG_CACHE_ENABLE) && !defined(CONFIG_SOC_ANDES_V5_CM) */
static pixel_t f_buffer[384000] __aligned(64);
#endif /* defined(CONFIG_CACHE_ENABLE) && !defined(CONFIG_SOC_ANDES_V5_CM) */

struct display_atflcdc100_device_config {
	void (*irq_config_func)(struct device *dev);
	uint32_t lcdc_base_addr;
};

static int andes_atflcdc100_write(const struct device *dev, const uint16_t x,
			     const uint16_t y,
			     const struct display_buffer_descriptor *desc,
			     const void *buf)
{

	struct framebuf_dev_data *data = FRAMEBUF_DATA(dev);
	pixel_t *dst = data->buffer;
	const pixel_t *src = buf;
	uint16_t row;

	dst += x;
	dst += (y * data->pitch);

	for (row = 0; row < desc->height; row++) {
		memcpy(dst, src, desc->pitch * sizeof(pixel_t));
		src += desc->pitch;
		dst += data->pitch;
	}

	return 0;
}

static int andes_atflcdc100_read(const struct device *dev, const uint16_t x,
			    const uint16_t y,
			    const struct display_buffer_descriptor *desc,
			    void *buf)
{

	struct framebuf_dev_data *data = FRAMEBUF_DATA(dev);
	const pixel_t *src = data->buffer;
	pixel_t *dst = buf;
	uint16_t row;

	src += x;
	src += (y * data->width);

	for (row = 0; row < desc->height; ++row) {
		memcpy(dst, src, desc->width * sizeof(pixel_t));
		src += data->pitch;
		dst += desc->pitch;
	}

	return 0;
}

static void *andes_atflcdc100_get_framebuffer(const struct device *dev)
{
	return FRAMEBUF_DATA(dev)->buffer;
}

static int andes_atflcdc100_display_blanking_off(const struct device *dev)
{
	OUTWORD(LCD_CTRL(dev),(INWORD(LCD_CTRL(dev)) | DISABLE_LCD_MASK));

	return 0;
}

static int andes_atflcdc100_display_blanking_on(const struct device *dev)
{
	OUTWORD(LCD_CTRL(dev),(INWORD(LCD_CTRL(dev)) & ~DISABLE_LCD_MASK));

	return 0;
}

static int andes_atflcdc100_set_brightness(const struct device *dev,
				      const uint8_t brightness)
{
	LOG_WRN("Set brightness not implemented");
	return -ENOTSUP;
}

static int andes_atflcdc100_set_contrast(const struct device *dev,
				    const uint8_t contrast)
{
	LOG_ERR("Set contrast not implemented");
	return -ENOTSUP;
}

static int andes_atflcdc100_set_pixel_format(const struct device *dev,
					const enum display_pixel_format
					pixel_format)
{
	if(PANEL_PIXEL_FORMAT != pixel_format)
		return -ENOTSUP;

	return 0;
}

static int andes_atflcdc100_set_orientation(const struct device *dev,
		const enum display_orientation orientation)
{
	if (orientation == DISPLAY_ORIENTATION_NORMAL) {
		return 0;
	}
	LOG_ERR("Changing display orientation not implemented");
	return -ENOTSUP;
}

static void andes_atflcdc100_get_capabilities(const struct device *dev,
		struct display_capabilities *capabilities)
{
	memset(capabilities, 0, sizeof(struct display_capabilities));

	capabilities->x_resolution = X_RES;
	capabilities->y_resolution = Y_RES;
	capabilities->supported_pixel_formats = PANEL_PIXEL_FORMAT;
	capabilities->current_pixel_format = PANEL_PIXEL_FORMAT;
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;

}

static void andes_atflcdc100_isr(void *arg)
{
	return;
}

static int andes_atflcdc100_init(const struct device *dev)
{
	struct framebuf_dev_data *data = FRAMEBUF_DATA(dev);

	data->buffer = (void*)f_buffer;
	data->pitch = X_RES;
	data->width = X_RES;
	data->height = Y_RES;

	memset(data->buffer, 0, FRAMEBUF_SIZE);

	// Set the base address of framebuffer
	OUTWORD(LCD_FRAME_BASE(dev), (mem_addr_t)data->buffer);

	// Set LCD Timing 0 register
	OUTWORD(LCD_TIMING_0(dev), LCD_TIMING_0_VALUE);

	// Set LCD Timing 1 register
	OUTWORD(LCD_TIMING_1(dev), LCD_TIMING_1_VALUE);

	// Set LCD Timing 2 register
	OUTWORD(LCD_TIMING_2(dev), LCD_TIMING_2_VALUE);

	// Set LCD Control register
	OUTWORD(LCD_CTRL(dev), LCD_CTRL_VALUE);

	return 0;
}

const struct display_driver_api framebuf_display_api = {
	.blanking_on = andes_atflcdc100_display_blanking_on,
	.blanking_off = andes_atflcdc100_display_blanking_off,
	.write = andes_atflcdc100_write,
	.read = andes_atflcdc100_read,
	.get_framebuffer = andes_atflcdc100_get_framebuffer,
	.set_brightness = andes_atflcdc100_set_brightness,
	.set_contrast = andes_atflcdc100_set_contrast,
	.get_capabilities = andes_atflcdc100_get_capabilities,
	.set_pixel_format = andes_atflcdc100_set_pixel_format,
	.set_orientation = andes_atflcdc100_set_orientation
};

static void andes_atflcdc100_config_func_1(struct device *dev);

static struct display_atflcdc100_device_config	\
display_atflcdc100_device_config_1 = {
	.irq_config_func = andes_atflcdc100_config_func_1,
	.lcdc_base_addr = DT_INST_REG_ADDR(0),
};

static struct framebuf_dev_data display_atflcdc100_dev_data_1;

DEVICE_AND_API_INIT(andes_atflcdc100_1, DT_INST_LABEL(0),
		    &andes_atflcdc100_init,
		    &display_atflcdc100_dev_data_1,
		    &display_atflcdc100_device_config_1,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &framebuf_display_api);

static void andes_atflcdc100_config_func_1(struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    andes_atflcdc100_isr, DEVICE_GET(andes_atflcdc100_1), 0);

	irq_enable(DT_INST_IRQN(0));
}
