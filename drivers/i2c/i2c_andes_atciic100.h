/*
 * Copyright (c) 2019 Andes Technology Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file I2C driver for AndesTech atciic100 IP
 */
#include <errno.h>
#include <kernel.h>
#include <device.h>
#include <soc.h>
#include <drivers/i2c.h>
#include <sys/util.h>
#include <sys/sys_io.h>

#ifndef BIT
#define BIT(n)			((unsigned int) 1 << (n))
#define BITS2(m,n)		(BIT(m) | BIT(n) )

/* bits range: for example BITS(16,23) = 0xFF0000
 *   ==>  (BIT(m)-1)   = 0x0000FFFF     ~(BIT(m)-1)   => 0xFFFF0000
 *   ==>  (BIT(n+1)-1) = 0x00FFFFFF
 */
#endif

#ifndef BITS
#define BITS(m,n)		(~(BIT(m)-1) & ((BIT(n) - 1) | BIT(n)))
#endif /* BIT */


#define RED_IDR			0x00 /* ID and Revision Register */
#define REG_CFG			0x10 /* Configuration Register */
#define REG_INTE		0x14 /* Interrupt Enable Register */
#define REG_STAT		0x18 /* Status Register */
#define REG_ADDR		0x1C /* Address Register */
#define REG_DATA		0x20 /* Data Register */
#define REG_CTRL		0x24 /* Control Register */
#define REG_CMD			0x28 /* Command Register */
#define REG_SET			0x2C /* Setup Register */

#define DEV_I2C_CFG(dev)					\
	((const struct i2c_atciic100_device_config* const)(dev)->config)

#define I2C_CFG(dev)  (DEV_I2C_CFG(dev)->i2c_base_addr + REG_CFG)
#define I2C_INTE(dev) (DEV_I2C_CFG(dev)->i2c_base_addr + REG_INTE)
#define I2C_STAT(dev) (DEV_I2C_CFG(dev)->i2c_base_addr + REG_STAT)
#define I2C_ADDR(dev) (DEV_I2C_CFG(dev)->i2c_base_addr + REG_ADDR)
#define I2C_CMD(dev)  (DEV_I2C_CFG(dev)->i2c_base_addr + REG_CMD)
#define I2C_SET(dev)  (DEV_I2C_CFG(dev)->i2c_base_addr + REG_SET)
#define I2C_DATA(dev) (DEV_I2C_CFG(dev)->i2c_base_addr + REG_DATA)
#define I2C_CTRL(dev)  (DEV_I2C_CFG(dev)->i2c_base_addr + REG_CTRL)

#define MEMSET(s, c, n)         __builtin_memset ((s), (c), (n))
#define MEMCPY(des, src, n)     __builtin_memcpy((des), (src), (n))
#define INWORD(x)     sys_read32(x)
#define OUTWORD(x, d) sys_write32(d, x)

#define SLAVE_ADDR_MSK			BITS(0,9)
#define DATA_MSK			BITS(0,7)

// Interrupt Enable Register(RW)
#define IEN_ALL				BITS(0,9)	// All Interrupts mask.
#define IEN_CMPL			BIT(9)		// Completion Interrupt.
#define IEN_BYTE_RECV			BIT(8)		// Byte Receive Interrupt.
#define IEN_BYTE_TRANS			BIT(7)		// Byte Transmit Interrupt.
#define IEN_START			BIT(6)		// START Condition Interrupt.
#define IEN_STOP			BIT(5)		// STOP Condition Interrupt.
#define IEN_ARB_LOSE			BIT(4)		// Arbitration Lose Interrupt.
#define IEN_ADDR_HIT			BIT(3)		// Address Hit Interrupt.
#define IEN_FIFO_HALF			BIT(2)		// FIFO Half Interrupt.
#define IEN_FIFO_FULL			BIT(1)		// FIFO Full Interrupt.
#define IEN_FIFO_EMPTY			BIT(0)		// FIFO Empty Interrupt.

// Status Register(RW)
#define STATUS_W1C_ALL			BITS(3,9)	// All Interrupts status write 1 clear mask.
#define STATUS_LINE_SDA			BIT(14)		// current status of the SDA line on the bus.
#define STATUS_LINE_SCL			BIT(13)		// current status of the SCL line on the bus.
#define STATUS_GEN_CALL			BIT(12)		// the address of the current transaction is a general call address.
#define STATUS_BUS_BUSY			BIT(11)		// the bus is busy.
#define STATUS_ACK			BIT(10)		// the type of the last received/transmitted acknowledgement bit.
#define STATUS_CMPL			BIT(9)		// Transaction Completion
#define STATUS_BYTE_RECV		BIT(8)		// a byte of data has been received
#define STATUS_BYTE_TRANS		BIT(7)		// a byte of data has been transmitted.
#define STATUS_START			BIT(6)		// a START Condition or a repeated TART condition has been transmitted/received.
#define STATUS_STOP			BIT(5)		// a STOP Condition has been transmitted/received.
#define STATUS_ARB_LOSE			BIT(4)		// the controller has lost the bus arbitration (master mode only).
#define STATUS_ADDR_HIT			BIT(3)		// Master: indicates that a slave has responded to the transaction, Slave: indicates that a transaction is targeting the controller (including the General Call).
#define STATUS_FIFO_HALF		BIT(2)		// Indicates that the FIFO is half-full or half-empty.
#define STATUS_FIFO_FULL		BIT(1)		// the FIFO is full.
#define STATUS_FIFO_EMPTY		BIT(0)		// the FIFO is empty.

// Control Register(RW)
#define CTRL_PHASE_START		BIT(12)		// Enable this bit to send a START condition at the beginning of transaction, master mode only.
#define CTRL_PHASE_ADDR			BIT(11)		// Enable this bit to send the address after START condition, master mode only.
#define CTRL_PHASE_DATA			BIT(10)		// Enable this bit to send the data after Address phase, master mode only.
#define CTRL_PHASE_STOP			BIT(9)		// Enable this bit to send a STOP condition at the end of a transaction, master mode only.
#define CTRL_DIR			BIT(8)		// Transaction direction
#define CTRL_DATA_COUNT			BITS(0,7)	// Data counts in bytes.

// Command Register(RW)
#define CMD_MSK				BITS(0,2)	// action command mask
#define CMD_NO_ACT			(0x0)		// no action
#define CMD_ISSUE_TRANSACTION		(0x1)		// issue a data transaction (Master only)
#define CMD_ACK				(0x2)		// respond with an ACK to the received byte
#define CMD_NACK			(0x3)		// respond with a NACK to the received byte
#define CMD_CLEAR_FIFO			(0x4)		// clear the FIFO
#define CMD_RESET_I2C			(0x5)		// reset the I2C controller

// Setup Register(RW)
#define SETUP_T_SUDAT			BITS(24,28)	// T_SUDAT defines the data setup time before releasing the SCL.
#define SETUP_T_SP			BITS(21,23)	// T_SP defines the pulse width of spikes that must be suppressed by the input filter.
#define SETUP_T_HDDAT			BITS(16,20)	// T_SUDAT defines the data hold time after SCL goes LOW.
#define SETUP_T_SCL_RATIO		BIT(13)		// The LOW period of the generated SCL clock is defined by the combination of T_SCLRatio and T_SCLHi values
#define SETUP_T_SCLHI			BITS(4,12)	// The HIGH period of generated SCL clock is defined by T_SCLHi.
#define SETUP_DMA_EN			BIT(3)		// Enable the direct memory access mode data transfer.
#define SETUP_MASTER			BIT(2)		// Configure this device as a master or a slave.
#define SETUP_ADDRESSING		BIT(1)		// I2C addressing mode: 10-bit or 7-bit addressing mode
#define SETUP_I2C_EN			BIT(0)		// Enable the ATCIIC100 I2C controller.

#if CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == 30000000

#define SETUP_T_SUDAT_STD		(0x3)		// setup time = (4 + 1 + 3) * 33ns = 264ns, Min 250ns
#define SETUP_T_SP_STD			(0x1)		// spikes time = 1 * 33ns = 33ns, Max 50ns
#define SETUP_T_HDDAT_STD		(5)		// hold time = (4 + 1 + 5) * 33ns = 330ns, Min 300ns
#define SETUP_T_SCL_RATIO_STD		(0x0)		// ratio=1, (4 + 1 + 138) * 33ns >= 4000ns, for I2C SCL clock, SCLHI Min 4000ns
#define SETUP_T_SCLHI_STD		(138)		// (4 + 1 + 138 * 1) * 33ns >= 4700ns, SCLLOW Min 4700ns

#define SETUP_T_SUDAT_FAST		(0x0)		// setup time = (4 + 1 + 0) * 33ns = 165ns, Min 100ns
#define SETUP_T_SP_FAST			(0x1)		// spikes time = 1 * 33ns =  33ns, Max 50ns
#define SETUP_T_HDDAT_FAST		(5)		// hold time = (4 + 1 + 5) * 33ns = 330ns, Min 300ns
#define SETUP_T_SCL_RATIO_FAST		(0x1)		// ratio=2, (4 + 1 + 18) * 33ns >= 600ns, for I2C SCL clock, SCLHI Min 600ns
#define SETUP_T_SCLHI_FAST		(18)		// (4 + 1 + 18 * 2) * 33ns >= 1300ns, SCLLOW Min 1300ns

#define SETUP_T_SUDAT_FAST_P		(0x0)		// setup time = (4 + 1 + 0) * 33ns = 165s, Min 50ns
#define SETUP_T_SP_FAST_P		(0x1)		// spikes time = 1 * 33ns = 33ns, Max 50ns
#define SETUP_T_HDDAT_FAST_P		(0x0)		// hold time = (4 + 1 + 0) * 33ns = 165ns, Min 0ns
#define SETUP_T_SCL_RATIO_FAST_P	(0x1)		// ratio=2, (4 + 1 + 6) * 33ns >= 260ns, for I2C SCL clock, SCLHI Min 260ns
#define SETUP_T_SCLHI_FAST_P		(6)		// (4 + 1 + 6 * 2) * 33ns >= 500ns, SCLLOW Min 500ns

#elif CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == 40000000

#define SETUP_T_SUDAT_STD		(0x4)		// setup time = (4 + 2 + 4) * 25ns = 250ns, Min 250ns
#define SETUP_T_SP_STD			(0x2)		// spikes time = 2 * 25ns = 50ns, Max 50ns
#define SETUP_T_HDDAT_STD		(0x6)		// hold time = (4 + 2 + 6) * 25ns = 300ns, Min 300ns
#define SETUP_T_SCL_RATIO_STD		(0x0)		// ratio=1, (4 + 2 + 182) * 25ns >= 4000ns, for I2C SCL clock, SCLHI Min 4000ns
#define SETUP_T_SCLHI_STD		(182)		// (4 + 2 + 182 * 1) * 25ns >= 4700ns, SCLLOW Min 4700ns

#define SETUP_T_SUDAT_FAST		(0x0)		// setup time = (4 + 2 + 0) * 25ns = 150ns, Min 100ns
#define SETUP_T_SP_FAST			(0x2)		// spikes time = 2 * 25ns = 50ns, Max 50ns
#define SETUP_T_HDDAT_FAST		(0x6)		// hold time = (4 + 2 + 6) * 25ns = 300ns, Min 300ns
#define SETUP_T_SCL_RATIO_FAST		(0x1)		// ratio=2, (4 + 2 + 23) * 25ns >= 600ns, for I2C SCL clock, SCLHI Min 600ns
#define SETUP_T_SCLHI_FAST		(23)		// (4 + 2 + 23 * 2) * 25ns >= 1300ns, SCLLOW Min 1300ns

#define SETUP_T_SUDAT_FAST_P		(0x0)		// setup time = (4 + 2 + 0) * 25ns = 150ns, Min 50ns
#define SETUP_T_SP_FAST_P		(0x2)		// spikes time = 2 * 25ns = 50ns, Max 50ns
#define SETUP_T_HDDAT_FAST_P		(0x0)		// hold time = (4 + 2 + 0) * 25ns = 150ns, Min 0ns
#define SETUP_T_SCL_RATIO_FAST_P	(0x1)		// ratio=2, (4 + 2 + 7) * 25ns >= 260ns, for I2C SCL clock, SCLHI Min 260ns
#define SETUP_T_SCLHI_FAST_P		(7)		// (4 + 2 + 7 * 2) * 25ns >= 500ns, SCLLOW Min 500ns

#else

#define SETUP_T_SUDAT_STD		(0x9)		// setup time = (4 + 3 + 9) * 16ns = 256ns, Min 250ns
#define SETUP_T_SP_STD			(0x3)		// spikes time = 3 * 16ns = 48ns, Max 50ns
#define SETUP_T_HDDAT_STD		(12)		// hold time = (4 + 3 + 12) * 16ns = 304ns, Min 300ns
#define SETUP_T_SCL_RATIO_STD		(0x0)		// ratio=1, (4 + 3 + 287) * 16ns >= 4000ns, for I2C SCL clock, SCLHI Min 4000ns
#define SETUP_T_SCLHI_STD		(287)		// (4 + 3 + 287 * 1) * 16ns >= 4700ns, SCLLOW Min 4700ns

#define SETUP_T_SUDAT_FAST		(0x0)		// setup time = (4 + 3 + 0) * 16ns = 112ns, Min 100ns
#define SETUP_T_SP_FAST			(0x3)		// spikes time = 3 * 16ns =  48ns, Max 50ns
#define SETUP_T_HDDAT_FAST		(12)		// hold time = (4 + 3 + 12) * 16ns = 304ns, Min 300ns
#define SETUP_T_SCL_RATIO_FAST		(0x1)		// ratio=2, (4 + 3 + 38) * 16ns >= 600ns, for I2C SCL clock, SCLHI Min 600ns
#define SETUP_T_SCLHI_FAST		(38)		// (4 + 3 + 38 * 2) * 16ns >= 1300ns, SCLLOW Min 1300ns

#define SETUP_T_SUDAT_FAST_P		(0x0)		// setup time = (4 + 3 + 0) * 16ns = 112ns, Min 50ns
#define SETUP_T_SP_FAST_P		(0x3)		// spikes time = 3 * 16ns = 48ns, Max 50ns
#define SETUP_T_HDDAT_FAST_P		(0x0)		// hold time = (4 + 3 + 0) * 16ns = 112ns, Min 0ns
#define SETUP_T_SCL_RATIO_FAST_P	(0x1)		// ratio=2, (4 + 3 + 13) * 16ns >= 260ns, for I2C SCL clock, SCLHI Min 260ns
#define SETUP_T_SCLHI_FAST_P		(13)		// (4 + 3 + 13 * 2) * 16ns >= 500ns, SCLLOW Min 500ns

#endif

#define DRIVER_OK			0 ///< Operation succeeded
#define DRIVER_ERROR			-1 ///< Unspecified error
#define DRIVER_ERROR_BUSY		-2 ///< Driver is busy
#define DRIVER_ERROR_TIMEOUT		-3 ///< Timeout occurred
#define DRIVER_ERROR_UNSUPPORTED	-4 ///< Operation not supported
#define DRIVER_ERROR_PARAMETER		-5 ///< Parameter error
#define DRIVER_ERROR_SPECIFIC		-6 ///< Start of driver specific errors

#define CB_OK                           0
#define CB_FAIL                         -7
#define DUMMY                           0xff

/****** I2C Event *****/
//#define I2C_EVENT_TRANSFER_DONE		(1UL << 0)  ///< Master/Slave Transmit/Receive finished
#define I2C_FLAG_SLAVE_TX_START		(1UL << 0)	//Slave Transmit start
#define I2C_FLAG_SLAVE_TX_DONE		(1UL << 1)	//Slave Transmit finished
#define I2C_FLAG_SLAVE_RX_START		(1UL << 2)	//Slave receive start
#define I2C_FLAG_SLAVE_RX_DONE		(1UL << 3)	//Slave receive finished
#define I2C_FLAG_GENERAL_CALL		(1UL << 4)	//Slave receive general call

#define MAX_XFER_SZ				(256)		// 256 bytes

typedef enum _i2c_ctrl_reg_item_dir
{
	I2C_MASTER_TX = 0x0,
	I2C_MASTER_RX = 0x1,
	I2C_SLAVE_TX = 0x1,
	I2C_SLAVE_RX = 0x0,
}i2c_ctrl_reg_item_dir;


// I2C driver running state
typedef enum _i2c_driver_state
{
	I2C_DRV_NONE = 0x0,
	I2C_DRV_INIT = BIT(0),
	I2C_DRV_POWER = BIT(1),
	I2C_DRV_CFG_PARAM = BIT(2),
	I2C_DRV_MASTER_TX = BIT(3),
	I2C_DRV_MASTER_RX = BIT(4),
	I2C_DRV_SLAVE_TX = BIT(5),
	I2C_DRV_SLAVE_RX = BIT(6),
	I2C_DRV_MASTER_TX_CMPL = BIT(7),
	I2C_DRV_MASTER_RX_CMPL = BIT(8),
	I2C_DRV_SLAVE_TX_CMPL = BIT(9),
	I2C_DRV_SLAVE_RX_CMPL = BIT(10),
}i2c_driver_state;

/**
\brief I2C Status
*/
typedef struct _i2c_status {
	uint32_t mode             : 1;        ///< Mode: 0=Slave, 1=Master
	uint32_t direction        : 1;        ///< Direction: 0=Transmitter, 1=Receiver
	uint32_t general_call     : 1;        ///< General Call indication (cleared on start of next Slave operation)
	uint32_t arbitration_lost : 1;        ///< Master lost arbitration (cleared on start of next Master operation)
	uint32_t bus_error        : 1;        ///< Bus error detected (cleared on start of next Master/Slave operation)
} i2c_status;

typedef void (*i2c_signalevent_t) (uint32_t event);  ///< Pointer to \ref NDS_I2C_SignalEvent : Signal I2C Event.

typedef struct _i2c_info
{
	struct k_sem i2c_transfer_sem;
	struct k_sem i2c_busy_sem;
	volatile i2c_driver_state	driver_state;
	uint8_t*			middleware_rx_buf;
	uint8_t*			middleware_tx_buf;
	/* flags for flow control */
	uint32_t			last_rx_data_count;// salve mode only
	uint32_t			middleware_rx_ptr;// Xfer_Data_Rd_Buf[] for middleware read out, salve mode only
	uint8_t				nack_assert;// salve mode only
	uint32_t			fifo_depth;
	uint32_t			slave_addr;
	uint32_t			xfer_wt_num;
	uint32_t			xfer_rd_num;
	uint32_t			xfered_data_wt_ptr;// write pointer
	uint32_t			xfered_data_rd_ptr;// read pointer
	uint32_t			xfered_data_rd_ptr_ow; // read pointer overwrite
	uint8_t				xfer_data_rd_buf[MAX_XFER_SZ];
	uint32_t			slave_rx_cmpl_ctrl_reg_val;
	volatile i2c_status		status;
	uint8_t				xfer_cmpl_count;
} nds_i2c_info;
