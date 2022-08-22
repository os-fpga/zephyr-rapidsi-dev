/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <ztest.h>

extern void test_i2c_simulate_eeprom_slave(void);

void test_main(void)
{
	ztest_test_suite(i2c_test, ztest_unit_test(test_i2c_simulate_eeprom_slave));
	ztest_run_test_suite(i2c_test);
}
