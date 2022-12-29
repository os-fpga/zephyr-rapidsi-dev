/*
 * Copyright (c) 2017 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/util.h>

#include <drivers/i2c.h>

#include <ztest.h>
//#include "i2c_andes_atciic100.h"

// I2C slave tx/rx flag
typedef enum _SLAVE_TRX_FLAG_ID
{
	I2C_SLAVE_TX_NONE = 0,
	I2C_SLAVE_RX_NONE = 0,
	I2C_SLAVE_TX_START,
	I2C_SLAVE_TX_DONE,
	I2C_SLAVE_RX_START,
	I2C_SLAVE_RX_DONE,
	I2C_SLAVE_TX_END,
}SLAVE_TRX_FLAG_ID;

#define DUMMY				0

// Orca EVB EEPROM I2C slave address
#define EEPROM_I2C_ADDR			0x50
// simulate I2C slave rom flash device
// sim 10 bit I2C slave address
#define SIM_ROM_I2C_ADDR		(0x3FA)

#define I2C_SLAVE_ADDR			SIM_ROM_I2C_ADDR

// sim I2C flash memory address 0x0 ~ 0x400
// Max memory locations available
#define EEPROM_MAX_ADDR			1024
// sim I2C flash buffer size
#define MAX_XFER_SZ			256

#define CB_OK				0
#define CB_FAIL				1

#if DT_NODE_HAS_STATUS(DT_ALIAS(i2c_0), okay)
#define I2C_DEV_NAME	DT_LABEL(DT_ALIAS(i2c_0))
#elif DT_NODE_HAS_STATUS(DT_ALIAS(i2c_1), okay)
#define I2C_DEV_NAME	DT_LABEL(DT_ALIAS(i2c_1))
#elif DT_NODE_HAS_STATUS(DT_ALIAS(i2c_2), okay)
#define I2C_DEV_NAME	DT_LABEL(DT_ALIAS(i2c_2))
#else
#error "Please set the correct I2C device"
#endif

static volatile uint16_t flash_addr = 0;
static volatile uint8_t slave_rom_flash[EEPROM_MAX_ADDR] = {0};

static volatile int slave_tx = I2C_SLAVE_TX_NONE;
static volatile int slave_rx = I2C_SLAVE_RX_NONE;

static uint8_t rx_count, tx_count;

uint32_t i2c_cfg = I2C_ADDR_10_BITS;

int write_requested(struct i2c_slave_config *config)
{
	{
		slave_tx = I2C_SLAVE_TX_START;
		return CB_OK;
	}
	return CB_FAIL;
}

int read_requested(struct i2c_slave_config *config, uint8_t *val)
{
	{
		slave_rx = I2C_SLAVE_RX_START;
		return CB_OK;
	}
	return CB_FAIL;
}

int write_received(struct i2c_slave_config *config, uint8_t val)
{
	{
		slave_tx = I2C_SLAVE_TX_DONE;
		tx_count = val;
		return CB_OK;
	}
	return CB_FAIL;
}

int read_processed(struct i2c_slave_config *config, uint8_t *val)
{
	{
		slave_rx = I2C_SLAVE_RX_DONE;
		rx_count = *val;
		return CB_OK;
	}
	return CB_FAIL;
}

int32_t i2c_slave_tx(const struct device *i2c_dev)
{
	uint16_t r_addr = 0;
	
	r_addr = flash_addr;
	
	i2c_write(i2c_dev, (uint8_t*)&slave_rom_flash[r_addr], DUMMY, DUMMY);

	while(slave_tx != I2C_SLAVE_TX_DONE)
	{
	}
	r_addr += tx_count;

	if(r_addr > EEPROM_MAX_ADDR)
	{
		TC_PRINT("\n\rThe request of flash address is over the max size of EEPROM\n\r");
		while(1);
	}

	return 0;
}


int32_t i2c_slave_rx(const struct device *i2c_dev)
{
	uint32_t data_count = rx_count;
	uint8_t tmp[2] = {0};
	uint16_t w_addr = 0;
	
	// master device issue write-request w/ flash address,
	// then issue read-request to read flash data from the address
	flash_addr = 0;
	
	// error hit
	if(data_count > MAX_XFER_SZ)
	{
		TC_PRINT("\n\rThe number of receive data is over the MAX receive buffer size\n\r");
		while(1);
	}
	
	if(data_count > 0)
	{
		if(data_count >= 2)
		{
			// the first 2 bytes are falsh address
			i2c_read(i2c_dev, &tmp[0], 2, 0x0);
			w_addr = flash_addr = ((tmp[0] << 8) | tmp[1]);
			i2c_read(i2c_dev, (uint8_t*)&slave_rom_flash[w_addr], data_count - 2, DUMMY);
		}
		else
		{
			i2c_read(i2c_dev, (uint8_t*)&slave_rom_flash[0], data_count - 2, DUMMY);
		}
	}
	// I2C data transaction w/o payload data
	else
	{
		TC_PRINT("\n\rThere is no receive data.\n\r");
	}
	return 0;
}

int simulated_eeprom(void)
{
	struct i2c_slave_config *slave_config = (struct i2c_slave_config *)malloc(sizeof(struct i2c_slave_config));
	static const struct i2c_slave_callbacks i2c_callbacks = { 
		.write_requested = write_requested,
		.read_requested = read_requested,
		.write_received = write_received,
		.read_processed = read_processed,
	};
	char uart_buf[0x100];

	TC_PRINT("\n\r=====Start demo I2C slave: please hardwire 2 test EVB=====\n\r");

	memset(&uart_buf[0], 0, sizeof(uart_buf));

	const struct device *i2c_dev = device_get_binding(I2C_DEV_NAME);
	zassert_not_null(i2c_dev, "I2C device not found");

	slave_config->address = I2C_SLAVE_ADDR;
	slave_config->callbacks = &i2c_callbacks;

	/* 1. verify i2c_configure() */
	if (i2c_configure(i2c_dev, i2c_cfg)) {
		TC_PRINT("I2C config failed\n");
		return TC_FAIL;
	}

	i2c_slave_register(i2c_dev, slave_config);

	do
	{
		if(slave_rx == I2C_SLAVE_RX_DONE)
		{
			// A new I2C data transaction(start-addr-data-stop)
			i2c_slave_rx(i2c_dev);
			slave_rx = I2C_SLAVE_RX_NONE;
		}
		if(slave_tx == I2C_SLAVE_TX_START)
		{
			// A new I2C data transaction(start-addr-data-stop)
			i2c_slave_tx(i2c_dev);
			slave_tx = I2C_SLAVE_TX_END;
		}
	}
	while(slave_tx != I2C_SLAVE_TX_END);

	TC_PRINT("\n\r=====Please check the data on master to verify this test case=====\n\r");
	return TC_PASS;
	while(1);

}

void test_i2c_simulate_eeprom_slave(void)
{
	zassert_true(simulated_eeprom() == TC_PASS, NULL);
}
