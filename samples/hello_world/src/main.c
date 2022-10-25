/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#undef CONFIG_ARCH
#define CONFIG_ARCH " Rapid Silicon QEMU Gemini"

void main(void)
{
	printk("Hello World! %s\n", CONFIG_ARCH);
}
