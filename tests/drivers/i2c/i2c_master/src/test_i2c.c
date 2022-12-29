/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * @addtogroup t_i2c_basic
 * @{
 * @defgroup t_i2c_read_write test_i2c_read_write
 * @brief TestPurpose: verify I2C master can read and write
 * @}
 */

#include <drivers/i2c.h>
#include <zephyr.h>
#include <ztest.h>

#ifndef TEST_I2C_EEPROM
#define TEST_I2C_EEPROM		1
#endif
#define TEST_I2C_TRANSFER	!(TEST_I2C_EEPROM)

#if DT_NODE_HAS_STATUS(DT_ALIAS(i2c_0), okay)
#define I2C_DEV_NAME	DT_LABEL(DT_ALIAS(i2c_0))
#elif DT_NODE_HAS_STATUS(DT_ALIAS(i2c_1), okay)
#define I2C_DEV_NAME	DT_LABEL(DT_ALIAS(i2c_1))
#elif DT_NODE_HAS_STATUS(DT_ALIAS(i2c_2), okay)
#define I2C_DEV_NAME	DT_LABEL(DT_ALIAS(i2c_2))
#else
#error "Please set the correct I2C device"
#endif

#if (TEST_I2C_EEPROM == 1)
#define EEPROM_I2C_ADDR				0x50
uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_MASTER;
#else
#define EEPROM_I2C_ADDR				0x3FA
uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_MASTER | I2C_ADDR_10_BITS;
#endif

#if TEST_I2C_EEPROM == 1
static int test_onboard_eeprom(void)
#else
static int test_simulated_eeprom(void)
#endif
{
	unsigned char datas[100];
	unsigned char receive[100];
	unsigned char send_data[100 + 2];
	uint8_t data_num = 64;
	unsigned char eeprom_data_addr[2] = {0x00, 0x00};
	int count = 0;
	const struct device *i2c_dev = device_get_binding(I2C_DEV_NAME);

	if (!i2c_dev) {
		TC_PRINT("Cannot get I2C device\n");
		return TC_FAIL;
	}

	/* 1. verify i2c_configure() */
	if (i2c_configure(i2c_dev, i2c_cfg)) {
		TC_PRINT("I2C config failed\n");
		return TC_FAIL;
	}

	for(count = 0; count < data_num; count+=4) {
		datas[count + 0] = 0x01;
		datas[count + 1] = 0x20;
		datas[count + 2] = 0x02;
		datas[count + 3] = 0x00;
	}

	send_data[0] = eeprom_data_addr[0];
	send_data[1] = eeprom_data_addr[1];
	for(count = 0; count < data_num; count++) {
		send_data[count + 2] = datas[count];
	}

	/* 2. verify i2c_burst_write() */
	if (i2c_write(i2c_dev, send_data, data_num + sizeof(eeprom_data_addr), EEPROM_I2C_ADDR)) {
		TC_PRINT("Fail to write to slave\n");
		return TC_FAIL;
	}

	k_sleep(K_SECONDS(1));

	(void)memset(receive, 0, sizeof(receive));

	/* 3. verify i2c_burst_read() */
	if (i2c_write_read(i2c_dev, EEPROM_I2C_ADDR, eeprom_data_addr, sizeof(eeprom_data_addr), receive, data_num)) {
		TC_PRINT("Fail to read from slave\n");
		return TC_FAIL;
	}

	k_sleep(K_SECONDS(1));

	for(count = 0; count < data_num; count++ ) {
		if(datas[count] != receive[count])
		{
			TC_PRINT("Receive data error");
			return TC_FAIL;
		}
	}

	TC_PRINT("Data correct\n");

	return TC_PASS;
}

void test_i2c_eeprom(void)
{
#if TEST_I2C_EEPROM == 1
	zassert_true(test_onboard_eeprom() == TC_PASS, NULL);
#else
	zassert_true(test_simulated_eeprom() == TC_PASS, NULL);
#endif
}
