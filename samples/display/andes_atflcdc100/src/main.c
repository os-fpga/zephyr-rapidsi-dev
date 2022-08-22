/*
 * Copyright (c) 2020 Andes Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/display.h>
#include <stdio.h>
#include <string.h>
#include <zephyr.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

static uint8_t buf[1600];
static uint8_t buf_rd[1600];


#ifdef CONFIG_ANDES_RGB_565
typedef uint16_t pixel_t;
#else
typedef uint32_t pixel_t;
#endif

/* This example will update each 500ms one of the LCD corners
 * wit a rectangular bitmap.
 * The color of the bit map is changed for every
 * iteration and is picked out of a set of red, green and blue.
 */
void main(void)
{
	const struct device *dev;

	struct display_capabilities capabilities;
	struct display_buffer_descriptor buf_desc;

	/* size of the rectangle */
	const size_t w = 20;
	const size_t h = 20;

	/* xy coordinates where to place rectangles*/
	size_t x0, y0, x1, y1, x2, y2, x3, y3, x4, y4;
	size_t idx = 0;
	size_t color = 0;
	size_t cnt = 0;

	pixel_t color_rgb;

	dev = device_get_binding("lcd_0");

	if (dev == NULL) {
		printk("Device not found. Aborting test.");
		return;
	}

	display_get_capabilities(dev, &capabilities);

	buf_desc.buf_size =  w * h * sizeof(pixel_t);
	buf_desc.pitch = w;
	buf_desc.width = w;
	buf_desc.height = h;

	x0 = 0;
	y0 = 0;
	x1 = capabilities.x_resolution - w;
	y1 = 0;
	x2 = capabilities.x_resolution - w;
	y2 = capabilities.y_resolution - h;
	x3 = 0;
	y3 = capabilities.y_resolution - h;
	x4 = (capabilities.x_resolution - w) / 2;
	y4 = (capabilities.y_resolution - h) / 2;

	display_blanking_off(dev);

	while (1) {

		switch (cnt %= 3) {

		case 0:

			/* Update the color of the rectangle buffer */

			#ifdef CONFIG_ANDES_RGB_565
			/* RGB565 format */
				switch(color){
				case 0:
					color_rgb = 0xF800U;
					break;
				case 1:
					color_rgb = 0x07E0U;
					break;
				case 2:
					color_rgb = 0x001FU;
					break;
				default:
					color_rgb = 0xFFFFU;
				}
			#else
			/* RGB888 format */
				switch(color){
				case 0:
					color_rgb = 0xFF0000U;
					break;
				case 1:
					color_rgb = 0x00FF00U;
					break;
				case 2:
					color_rgb = 0x0000FFU;
					break;
				default:
					color_rgb = 0xFFFFFFU;
				}
			#endif

			for (idx = 0; idx < buf_desc.buf_size; idx += sizeof(pixel_t))
				*(pixel_t*)(buf + idx) = color_rgb;

			if (++color > 2) {
				color = 0;
			}

			break;
		case 1:
			display_write(dev, x0, y0, &buf_desc, buf);
			display_write(dev, x0+w, y0, &buf_desc, buf);
			display_write(dev, x0, y0+h, &buf_desc, buf);
			display_write(dev, x1, y1, &buf_desc, buf);
			display_write(dev, x1-w, y1, &buf_desc, buf);
			display_write(dev, x1, y1+h, &buf_desc, buf);
			display_write(dev, x2, y2, &buf_desc, buf);
			display_write(dev, x2-w, y2, &buf_desc, buf);
			display_write(dev, x2, y2-h, &buf_desc, buf);
			display_write(dev, x3, y3, &buf_desc, buf);
			display_write(dev, x3+w, y3, &buf_desc, buf);
			display_write(dev, x3, y3-h, &buf_desc, buf);

			display_read(dev, x0, y0, &buf_desc, buf_rd);
			display_write(dev, x4, y4, &buf_desc, buf_rd);
			display_write(dev, x4-w, y4, &buf_desc, buf_rd);
			display_write(dev, x4+w, y4, &buf_desc, buf_rd);
			display_write(dev, x4, y4-h, &buf_desc, buf_rd);
			display_write(dev, x4, y4+h, &buf_desc, buf_rd);

			display_blanking_off(dev);

			break;

		case 2:
			display_blanking_on(dev);
			break;
		}
		++cnt;

		k_sleep(K_MSEC(500));
	}

}
