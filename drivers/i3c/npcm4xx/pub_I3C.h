/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EXPORTS_PUB_I3C_H_
#define EXPORTS_PUB_I3C_H_

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <kernel.h>

#include <common/reg/reg_def.h>
#include <common/reg/reg_access.h>

#define WAIT_SLAVE_PREPARE_RESPONSE_TIME 10 /* unit: us */

/*----------------------------- I3C --------------------------------*/
#define I3C_TRANSFER_SPEED_SDR_12p5MHZ  12500000
#define I3C_TRANSFER_SPEED_SDR_8MHZ     8000000
#define I3C_TRANSFER_SPEED_SDR_6MHZ     6000000
#define I3C_TRANSFER_SPEED_SDR_4MHZ     4000000
#define I3C_TRANSFER_SPEED_SDR_2MHZ     2000000
#define I3C_TRANSFER_SPEED_SDR_1MHZ     1000000

#define I3C_TRANSFER_SPEED_SDR_IBI      1000000

#define I3C_TRANSFER_SPEED_I2C_1MHZ     1000000
#define I3C_TRANSFER_SPEED_I2C_400KHZ   400000
#define I3C_TRANSFER_SPEED_I2C_100KHZ   100000

#define I3C_TRANSFER_SPEED_UNDEF  0

#define I3C_PAYLOAD_SIZE_MAX	256
#define IBI_PAYLOAD_SIZE_MAX	8

enum I3C_PORT {
	I3C1_IF,
	I3C2_IF,
	I3C3_IF,
	I3C4_IF,
	I3C5_IF,
	I3C6_IF,
	I3C_PORT_MAX,
};

#define I3C_PORT_Enum enum I3C_PORT

#define I3C_BUS_COUNT_MAX    I3C_PORT_MAX

#define I2C_STATIC_ADDR_DEFAULT_7BIT 0xFFU
#define I3C_DYNAMIC_ADDR_DEFAULT_7BIT 0x00U

/*#define I3C_BROADCAST_ADDR	0x7E*/

#define RX_HANDLER_MAX	3

enum I3C_DEVICE_TYPE {
	I3C_DEVICE_TYPE_PURE_I3C = 0U,
	I3C_DEVICE_TYPE_PURE_I2C = 1U,
	I3C_DEVICE_TYPE_MIXED = 2U,
};

#define I3C_DEVICE_TYPE_Enum enum I3C_DEVICE_TYPE

enum I3C_DEVICE_MODE {
	I3C_DEVICE_MODE_DISABLE            = 0U,
	I3C_DEVICE_MODE_CURRENT_MASTER     = 1U,
	I3C_DEVICE_MODE_SLAVE_ONLY         = 2U,
	I3C_DEVICE_MODE_SECONDARY_MASTER   = 3U,
};

#define I3C_DEVICE_MODE_Enum enum I3C_DEVICE_MODE

enum I3C_SLAVE_DEVICE_MODE {
	I3C_SLAVE_DEVICE_MODE_I2C,
	I3C_SLAVE_DEVICE_MODE_I3C,
};

#define I3C_SLAVE_DEVICE_MODE_Enum enum I3C_SLAVE_DEVICE_MODE

enum I3C_BUS_STATE {
	I3C_BUS_STATE_DEFAULT,
	I3C_BUS_STATE_WAIT_RESET_DONE,
	I3C_BUS_STATE_WAIT_CLEAR_DONE,
	I3C_BUS_STATE_INIT,
	I3C_BUS_STATE_IDLE,
};

#define I3C_BUS_STATE_Enum enum I3C_BUS_STATE

enum I3C_BUS_TYPE {
	I3C_BUS_TYPE_PURE_I3C = 0U,
	I3C_BUS_TYPE_PURE_I2C = 1U,
	I3C_BUS_TYPE_MIXED = 2U,
};

#define I3C_BUS_TYPE_Enum enum I3C_BUS_TYPE

enum I3C_VERSION {
	I3C_VER_1p0 = 0U,
	I3C_VER_1p1 = 1U,
};

#define I3C_VERSION_Enum enum I3C_VERSION

struct I3C_BAUDRATE {
	uint32_t i3cI2c;
	uint32_t i3cSdr;

	uint16_t tHmax;
	uint16_t tHmin;
	uint16_t tLmax;
	uint16_t tLmin;
	uint16_t tTimeout;
	bool bRunI3C;
};

#define I3C_BAUDRATE_t struct I3C_BAUDRATE

struct I3C_DEVICE_INFO;
struct I3C_DEVICE_INFO_SHORT;

/*typedef */struct I3C_BUS_INFO {
	uint8_t busno;

	I3C_BUS_TYPE_Enum busType;
	I3C_VERSION_Enum busVersion;
	I3C_BUS_STATE_Enum busState;
	struct I3C_TRANSFER_TASK *pCurrentTask;
	uint16_t timer_ms;

	uint8_t DevCount;
	struct I3C_DEVICE_INFO_SHORT *pDevList;
	struct I3C_DEVICE_INFO *pCurrentMaster;
} /* I3C_BUS_INFO_t*/;

#define I3C_BUS_INFO_t struct I3C_BUS_INFO

/*typedef */union I3C_DEVICE_ATTRIB_t {
	struct {
		uint16_t reqSETAASA : 1;
		uint16_t reqSETDASA : 1;
		uint16_t reqRSTDAA : 1;
		uint16_t reqSETHID : 1;
		uint16_t reqHotJoin : 1;
		uint16_t reqPostInit : 1;

		uint16_t doneSETAASA : 1;
		uint16_t doneSETDASA : 1;
		uint16_t doneRSTDAA : 1;
		uint16_t doneSETHID : 1;
		uint16_t doneHotJoin : 1;
		uint16_t donePostInit : 1;

		uint16_t suppMST : 1;
		uint16_t suppSLV : 1;
		uint16_t defaultMST : 1;

		uint16_t suppENTDAA : 1;
		uint16_t present: 1;
		uint16_t runI3C: 1;
	} b;
	uint16_t U16;
} /* I3C_DEVICE_ATTRIB_t */;

#define I3C_DEVICE_ATTRIB_t union I3C_DEVICE_ATTRIB_t

/*typedef */struct I3C_DEVICE_INFO_SHORT {
	uint8_t staticAddr;
	uint8_t dynamicAddr;
	uint8_t pid[6];
	uint8_t bcr;
	uint8_t dcr;

	I3C_DEVICE_ATTRIB_t attr;
	struct I3C_DEVICE_INFO_SHORT *pNextDev;
	void *pDeviceInfo;
} /* I3C_DEVICE_INFO_SHORT_t */;

#define I3C_DEVICE_INFO_SHORT_t struct I3C_DEVICE_INFO_SHORT

#define CMD_INVALID_SDR 0xFE
#define CMD_INVALID_DDRRd 0xFE
#define CMD_DEFAULT 0xFD
#define CMD_INVALID 0xFE
#define CMD_INVALID_DDRWr 0x7E

#define TIMEOUT_TYPICAL	10

#define NOT_HIF false
#define IS_HIF true

/*typedef */union cmd_t {
	uint8_t cmd8;
	uint16_t cmd16;
} /* cmd_t*/;

#define cmd_t union cmd_t

/*typedef */struct cmd_attrib {
	uint8_t endian	: 1;    /* 0b: little, 1b: bigh endian, if width != 0 */
	uint8_t width : 1;     /* 0b = 1, 1b = 2 */
	uint8_t write : 1;     /* wrtiable */
	uint8_t read  : 1;     /* readable */
} /* cmd_attrib_t */;

#define cmd_attrib_t struct cmd_attrib

/*typedef */struct I3C_REG_ITEM {
	cmd_t cmd;

	union {
		uint8_t rw;
		cmd_attrib_t attr;
	};

	uint16_t len;
	uint8_t *buf;
} /* I3C_REG_ITEM_t*/;

#define I3C_REG_ITEM_t struct I3C_REG_ITEM

enum I3C_ErrCode {
	I3C_ERR_OK = 0,
	I3C_ERR_PENDING = 1,				/* wait i3c engine to process it */
	I3C_ERR_OUT_OF_MEMORY = 2,          /* allocate memory fail */
	I3C_ERR_PARAMETER_INVALID = 4,      /* parameter invalid */
	I3C_ERR_TASK_INVALID = 8,			/* illegal task */
	I3C_ERR_MEMORY_RAN_OUT = 16,        /* allocate memory is insufficient */
	I3C_ERR_DATA_ERROR = 32,			/* crc error for SPD5118, MCTP */
	I3C_ERR_SW_TIMEOUT = 64,			/* timer timeout, or retry too many times */
	I3C_ERR_HW_NOT_SUPPORT,
	I3C_ERR_BUS_ERROR,
	I3C_ERR_MERRWARN,
	I3C_ERR_NACK,
	I3C_ERR_TERM,
	I3C_ERR_WRABT,
	I3C_ERR_ERRWARN,
	I3C_ERR_HW_TIMEOUT,
	I3C_ERR_BUS_BUSY,
	I3C_ERR_SLVSTART,
	I3C_ERR_IBI,
	I3C_ERR_MR,
	I3C_ERR_HJ,
	I3C_ERR_NACK_SLVSTART,
};

#define I3C_ErrCode_Enum enum I3C_ErrCode

enum I3C_CCC {
	CCC_BROADCAST_ENEC      = 0x00, /* Enable Events Command */
	CCC_BROADCAST_DISEC     = 0x01, /* Disable Events Command */
	CCC_BROADCAST_ENTAS0    = 0x02, /* Enter Activity State 0, 1 us */
	CCC_BROADCAST_ENTAS1    = 0x03, /* Enter Activity State 1, 100 us */
	CCC_BROADCAST_ENTAS2    = 0x04, /* Enter Activity State 2, 2 ms */
	CCC_BROADCAST_ENTAS3    = 0x05, /* Enter Activity State 3, 50 ms */
	CCC_BROADCAST_RSTDAA    = 0x06, /* Reset Dynamic Address Assignment */
	CCC_BROADCAST_ENTDAA    = 0x07, /* Enter Dynamic Address Assignment */
	CCC_BROADCAST_DEFSLVS   = 0x08,
	CCC_BROADCAST_SETMWL    = 0x09,
	CCC_BROADCAST_SETMRL    = 0x0A,
	CCC_BROADCAST_ENTTM     = 0x0B,
	CCC_BROADCAST_SETBUSCON = 0x0C,
/*    CCC_BROADCAST_rsvd      = 0x0D - 0x11, */
	CCC_BROADCAST_ENDXFER   = 0x12,
/*    CCC_BROADCAST_rsvd      = 0x13 - 0x1F, */
	CCC_BROADCAST_ENTHDR0   = 0x20,
	CCC_BROADCAST_ENTHDR1   = 0x21,
	CCC_BROADCAST_ENTHDR2   = 0x22,
	CCC_BROADCAST_ENTHDR3   = 0x23,
	CCC_BROADCAST_ENTHDR4   = 0x24,
	CCC_BROADCAST_ENTHDR5   = 0x25,
	CCC_BROADCAST_ENTHDR6   = 0x26,
	CCC_BROADCAST_ENTHDR7   = 0x27,
	CCC_BROADCAST_SETXTIME  = 0x28,
	CCC_BROADCAST_SETAASA   = 0x29, /* Set All Addresses to Static Addresses */
	CCC_BROADCAST_RSTACT    = 0x2A,
	CCC_BROADCAST_DEFGRPA   = 0x2B,
	CCC_BROADCAST_RSTGRPA   = 0x2C,
	CCC_BROADCAST_MLANE     = 0x2D,
/*    CCC_BROADCAST_rsvd      = 0x2E - 0x48, */
/*    CCC_BROADCAST_rsvd      = 0x49 - 0x4C, */
/*    CCC_BROADCAST_rsvd      = 0x4D - 0x57, */
/*    CCC_BROADCAST_rsvd      = 0x58 - 0x5B, */
/*    CCC_BROADCAST_rsvd      = 0x5C - 0x60, */
/*    CCC_BROADCAST_rsvd      = 0x61 - 0x7F, */
	CCC_BROADCAST_SETHID    = 0x61, /* JEDEC DDR5 */
	CCC_BROADCAST_DEVCTRL   = 0x62, /* JEDEC DDR5 */
	CCC_DIRECT_ENEC         = 0x80, /* Enable Events Command, Direct Set */
	CCC_DIRECT_DISEC        = 0x81, /* Disable Events Command, Direct Set */
	CCC_DIRECT_ENTAS0       = 0x82, /* Enter Activity State 0, 1 us, Direct Set */
	CCC_DIRECT_ENTAS1       = 0x83, /* Enter Activity State 1, 100 us, Direct Set */
	CCC_DIRECT_ENTAS2       = 0x84, /* Enter Activity State 2, 2 ms, Direct Set */
	CCC_DIRECT_ENTAS3       = 0x85, /* Enter Activity State 3, 50 ms, Direct Set */
	CCC_DIRECT_RSTDAA       = 0x86, /* Reset Dynamic Address Assignment, Direct Set */
	CCC_DIRECT_SETDASA      = 0x87, /* Set Dynamic Address from Static Address, Direct Set */
	CCC_DIRECT_SETNEWDA     = 0x88, /* Set New Dynamic Address, Direct Set */
	CCC_DIRECT_SETMWL       = 0x89, /* Set Max Write Length, Direct Set */
	CCC_DIRECT_SETMRL       = 0x8A, /* Set Max Read Length, Direct Set */
	CCC_DIRECT_GETMWL       = 0x8B, /* Get Max Write Length, Direct Get */
	CCC_DIRECT_GETMRL       = 0x8C, /* Get Max Read Length, Direct Get */
	CCC_DIRECT_GETPID       = 0x8D, /* Get Provisioned ID, Direct Get */
	CCC_DIRECT_GETBCR       = 0x8E, /* Get Bus Characteristics Register, Direct Get */
	CCC_DIRECT_GETDCR       = 0x8F, /* Get Device Characteristics Register, Direct Get */
	CCC_DIRECT_GETSTATUS    = 0x90, /* Get Device Status, Direct Get */
	CCC_DIRECT_GETACCMST    = 0x91,
	CCC_DIRECT_GETACCCR     = 0x91, /* Get Accept Controller Role, Direct Get */
	CCC_DIRECT_ENDXFER      = 0x92, /* Data Transfer Ending Procedure Control, Direct Set */
	CCC_DIRECT_SETBRGTGT    = 0x93, /* Set Bridge Targets, Direct Set */
	CCC_DIRECT_GETMXDS      = 0x94, /* Get Max Data Speed, Direct Get */
	CCC_DIRECT_GETCAPS      = 0x95, /* GetOptional Feature Capabilities, Direct Get */
	CCC_DIRECT_SETROUTE     = 0x96, /* Set Route, Direct Set */
/*    CCC_DIRECT_D2DXFER      = 0x97, // Device to Device Tuneling Control, not include in Basic*/
	CCC_DIRECT_SETXTIME     = 0x98, /* Set Exchange Timing Information, Direct Set */
	CCC_DIRECT_GETXTIME     = 0x99, /* Get Exchange Timing Information, Direct Get */
	CCC_DIRECT_RSTACT       = 0x9A, /* Target Reset Action, Direct Set */
	CCC_DIRECT_SETGRPA      = 0x9B, /* Set Group Address */
	CCC_DIRECT_RSTGRPA      = 0x9C, /* Reset Group Address */
	CCC_DIRECT_MLANE        = 0x9D,
/*    CCC_DIRECT_rsvd         = 0x9E - 0xBF, */
/*    CCC_DIRECT_rsvd         = 0xC0 - 0xC3, */
/*    CCC_DIRECT_rsvd         = 0xC4 - 0xD6, */
/*    CCC_DIRECT_rsvd         = 0xD7 - 0xDA, */
/*    CCC_DIRECT_rsvd         = 0xDB - 0xDF, */
/*    CCC_DIRECT_rsvd         = 0xE0 - 0xFE, */
/*    CCC_DIRECT_rsvd         = 0xFF, */
};

#define I3C_CCC_Enum enum I3C_CCC

#define ENEC_MASK_ENINT (0x01)
#define ENEC_MASK_ENMR  (0x02)
#define ENEC_MASK_ENHJ  (0x08)

#define DISEC_MASK_ENINT (0x01)
#define DISEC_MASK_ENMR  (0x02)
#define DISEC_MASK_ENHJ  (0x08)

enum I3C_TRANSFER_PROTOCOL {
	I3C_TRANSFER_PROTOCOL_I2C_WRITE         = 0x00,
	I3C_TRANSFER_PROTOCOL_I2C_READ          = 0x01,
	I3C_TRANSFER_PROTOCOL_I2C_WRITEnREAD    = 0x02,

	I3C_TRANSFER_PROTOCOL_I3C_WRITE         = 0x10,
	I3C_TRANSFER_PROTOCOL_I3C_READ          = 0x11,
	I3C_TRANSFER_PROTOCOL_I3C_WRITEnREAD    = 0x12,
	I3C_TRANSFER_PROTOCOL_I3C_W7E           = 0x13,
	I3C_TRANSFER_PROTOCOL_I3C_R7E           = 0x14,
	I3C_TRANSFER_PROTOCOL_I3C_W7EnREAD      = 0x15,

	I3C_TRANSFER_PROTOCOL_DDR_WRITE         = 0x20,
	I3C_TRANSFER_PROTOCOL_DDR_READ          = 0x21,

	I3C_TRANSFER_PROTOCOL_CCCb              = 0x40U,	/* Broadcast CCC*/
	I3C_TRANSFER_PROTOCOL_CCCw              = 0x41U,	/* Direct Write*/
	I3C_TRANSFER_PROTOCOL_CCCr              = 0x42U,	/* Direct Read*/
	I3C_TRANSFER_PROTOCOL_ENTDAA            = 0x43U,	/* ENTDAA*/

	I3C_TRANSFER_PROTOCOL_EVENT             = 0x80U,	/* for IBI/Hot-Join/Master Request*/
	I3C_TRANSFER_PROTOCOL_IBI               = 0x81U,	/* Slave's task for IBI*/
	I3C_TRANSFER_PROTOCOL_MASTER_REQUEST    = 0x82U,	/* Slave's task for Master Request*/
	I3C_TRANSFER_PROTOCOL_HOT_JOIN          = 0x84U,	/* Slave's task for Hot-Join*/
};

#define I3C_TRANSFER_PROTOCOL_Enum enum I3C_TRANSFER_PROTOCOL

#define I2C_TRANSFER_PROTOCOL(x) (((x & 0xF0) == 0x00) ? true : false)
#define I3C_TRANSFER_PROTOCOL(x) (((x & 0xF0) == 0x10) ? true : false)
#define DDR_TRANSFER_PROTOCOL(x) (((x & 0xF0) == 0x20) ? true : false)
#define CCC_TRANSFER_PROTOCOL(x) (((x & 0xF0) == 0x40) ? true : false)
	#define CCC_BROADCAST(CCC) (((CCC >= 0) && (CCC <= 0x7F)) ? true : false)
	#define CCC_DIRECT(CCC) (((CCC >= 0x80) && (CCC <= 0xFE)) ? true : false)
#define EVENT_TRANSFER_PROTOCOL(x) (((x & 0xF0) == 0x80) ? true : false)
#define SLAVE_TRANSFER_PROTOCOL(x) (((x == 0x81) || (x == 0x82) || (x == 0x84)) ? true : false)

#define WRITE_TRANSFER_PROTOCOL(x)      (((x & 0xCF) == 0x00) ? true : false)
#define READ_TRANSFER_PROTOCOL(x)       (((x & 0xCF) == 0x01) ? true : false)
#define WRITEnREAD_TRANSFER_PROTOCOL(x) (((x & 0xCF) == 0x02) ? true : false)
#define W7E_TRANSFER_PROTOCOL(x)        (((x & 0xCF) == 0x03) ? true : false)
#define R7E_TRANSFER_PROTOCOL(x)        (((x & 0xCF) == 0x04) ? true : false)
#define W7EnREAD_TRANSFER_PROTOCOL(x)   (((x & 0xCF) == 0x05) ? true : false)

enum I3C_TASK_POLICY {
	I3C_TASK_POLICY_INSERT_FIRST,
	I3C_TASK_POLICY_APPEND_LAST,
};

#define I3C_TASK_POLICY_Enum enum I3C_TASK_POLICY

enum I3C_TRANSFER_FLAG {
	I3C_TRANSFER_NORMAL             = 0x00U,
	I3C_TRANSFER_NO_STOP            = 0x01U,
	I3C_TRANSFER_REPEAT_START       = 0x02U,
	I3C_TRANSFER_NO_START           = 0x04U,
	I3C_TRANSFER_RETRY_ENABLE		= 0x08U,
	I3C_TRANSFER_RETRY_WITHOUT_STOP = 0x10U,
	I3C_TRANSFER_WORD_WIDTH         = 0x20U,
	I3C_TRANSFER_MESSAGE_MODE       = 0x40U,
	I3C_TRANSFER_NAK				= 0x100U,
};

#define I3C_TRANSFER_FLAG_Enum enum I3C_TRANSFER_FLAG

enum I3C_TRANSFER_DIR {
	I3C_TRANSFER_DIR_WRITE = 0U,
	I3C_TRANSFER_DIR_READ  = 1U,
};

#define I3C_TRANSFER_DIR_Enum enum I3C_TRANSFER_DIR

enum I3C_TRANSFER_TYPE {
	I3C_TRANSFER_TYPE_SDR    = 0U,
	I3C_TRANSFER_TYPE_I2C    = 1U,
	I3C_TRANSFER_TYPE_DDR    = 2U,
};

#define I3C_TRANSFER_TYPE_Enum enum I3C_TRANSFER_TYPE

enum I3C_IBITYPE {
	I3C_IBITYPE_None    = 0,
	I3C_IBITYPE_IBI     = 1,
	I3C_IBITYPE_MstReq  = 2,
	I3C_IBITYPE_HotJoin = 3,
};

#define I3C_IBITYPE_Enum enum I3C_IBITYPE

typedef uint32_t (*ptrI3C_RetFunc)(uint32_t TaskInfo, uint32_t ErrDetail);

struct I3C_CAPABILITY_INFO {
	uint8_t	MASTER : 1;
	uint8_t	SLAVE : 1;

	uint8_t	INT : 1;
	uint8_t	DMA : 1;
	uint8_t	OFFLINE : 1;

	uint8_t	HDR_DDR : 1;
	uint8_t	HDR_TSP : 1;
	uint8_t	HDR_TSL : 1;
	uint8_t	IBI : 1;
	uint8_t	HOT_JOIN : 1;
	uint8_t	MASTER_REQUEST : 1;

	/* Time Control */
	uint8_t	SYNC : 1;
	uint8_t	ASYNC0 : 1;
	uint8_t	ASYNC1 : 1;
	uint8_t	ASYNC2 : 1;
	uint8_t	ASYNC3 : 1;

	/* Version */
	uint8_t I3C_VER_1p0 : 1;
	uint8_t I3C_VER_1p1 : 1;

	uint8_t FIFO_TX_LEN;
	uint8_t FIFO_RX_LEN;
};

#define I3C_CAPABILITY_INFO_t struct I3C_CAPABILITY_INFO

struct I3C_DEVICE_INFO {
	struct I3C_CAPABILITY_INFO capability;
	I3C_PORT_Enum port;
	struct I3C_BUS_INFO *pOwner;    /* bus */
	struct I3C_DEVICE_INFO_SHORT *pDevInfo;

	I3C_DEVICE_MODE_Enum mode;  /* off, master on, slave only, master capable */
	bool bRunI3C;              /* current slave device working mode, 0:i2c, 1:i3c */

	/* common */
	uint8_t dynamicAddr;
	uint8_t pid[6];
	uint8_t dcr;                /* Device characteristics register information.*/
	uint8_t bcr;                /* Bus characteristics register information.*/
	uint16_t vendorID;          /* Device vendor ID (manufacture ID).*/
	uint32_t partNumber;        /* Device part number info */

	uint32_t max_rd_len;
	uint32_t max_wr_len;

	/* master config */
	bool enableOpenDrainHigh;
	bool enableOpenDrainStop;
	bool disableTimeout;
	bool ackIBI; /* For master, used to define master should ack or nack the slave request */
				  /* For slave, used to set correct protocol (with 7E)*/
	struct I3C_BAUDRATE baudrate;

	uint8_t *pRxBuf;
	uint8_t *pTxBuf;

	uint16_t rxOffset;
	uint16_t txOffset;

	uint16_t rxLen;
	uint16_t txLen;

	/* slave config */
	uint8_t staticAddr; /* 7-bit */
	bool stopSplitRead; /* 1b: master can read un-read data after STOP */
						 /* 0b: slave should reset command, RX DMA/FIFO */
	uint8_t cmdIndex;
	I3C_REG_ITEM_t *pReg;
	uint8_t regCnt;

	bool bAbort;
	volatile uint8_t task_count;
	struct I3C_TRANSFER_TASK *pTaskListHead;
	ptrI3C_RetFunc callback;

	uint8_t dma_tx_channel;
	uint8_t dma_rx_channel;

	struct k_mutex lock;
	struct k_sem ibi_complete;
};

#define I3C_DEVICE_INFO_t struct I3C_DEVICE_INFO

/*
 * initMode
 * Used to configure how to init slave device on the bus
 */
#define INITMODE_I2C        0x0000U
#define INITMODE_I3C        0x0001U
#define INITMODE_BUSRESET   0x0002U
#define INITMODE_DEVCTRL    0x0004U
#define INITMODE_SETHID     0x0008U
#define INITMODE_SETAASA    0x0010U
#define INITMODE_SETDASA    0x0020U
#define INITMODE_ENTDAA		0x0040U
#define INITMODE_POWERON    0x0080U
#define INITMODE_POST_INIT  0x0100U
#define INITMODE_RSTDAA     0x0200U

struct I3C_TASK_INFO {
	struct I3C_TRANSFER_TASK *pTask;
	I3C_ErrCode_Enum result;
	ptrI3C_RetFunc callback;

	uint8_t MasterRequest : 1;
	uint8_t bHIF : 1;
	I3C_PORT_Enum Port;

	uint32_t SwTimeout;

	void *pParentTaskInfo;

	/* used for Sensor module */
	uint8_t idx;
	uint8_t fmt;
};

#define I3C_TASK_INFO_t struct I3C_TASK_INFO

struct I3C_TRANSFER_TASK {
	struct I3C_TASK_INFO *pTaskInfo;

	I3C_TRANSFER_PROTOCOL_Enum protocol;
	uint32_t baudrate;
	uint8_t address;

	uint8_t *pWrBuf;
	uint16_t *pWrLen;

	uint8_t *pRdBuf;
	uint16_t *pRdLen;

	struct I3C_TRANSFER_FRAME *pFrameList;
	uint8_t frame_count;
	uint8_t frame_idx;

	struct I3C_TRANSFER_TASK *pNextTask;
};

#define I3C_TRANSFER_TASK_t struct I3C_TRANSFER_TASK

struct I3C_TRANSFER_FRAME {
	struct I3C_TRANSFER_TASK *pOwner;

	I3C_TRANSFER_FLAG_Enum flag;
	I3C_TRANSFER_TYPE_Enum type;
	uint32_t baudrate;

	uint8_t address;
	I3C_TRANSFER_DIR_Enum direction;
	uint8_t hdrcmd;

	uint16_t access_len;
	uint16_t access_idx;
	uint8_t *access_buf;
	uint8_t retry_count;

	struct I3C_TRANSFER_FRAME *pNextFrame;
};

#define I3C_TRANSFER_FRAME_t struct I3C_TRANSFER_FRAME

/* LSM6DSO */
enum I3C_LSM6DSO_STATE {
	I3C_LSM6DSO_STATE_DEFAULT,
	I3C_LSM6DSO_STATE_INIT,
	I3C_LSM6DSO_STATE_IDLE,
	I3C_LSM6DSO_STATE_BUSY,
	I3C_LSM6DSO_STATE_TASK_CANNOT_COMPLETE,
	I3C_LSM6DSO_STATE_DEVICE_NOT_PRESENT,
};

#define I3C_LSM6DSO_STATE_Enum enum I3C_LSM6DSO_STATE

enum LSM6DSO_POST_INIT_STATE {
	LSM6DSO_POST_INIT_STATE_Default,

	LSM6DSO_POST_INIT_STATE_1,
	LSM6DSO_POST_INIT_STATE_1_Wait,
	LSM6DSO_POST_INIT_STATE_1_Done,

	LSM6DSO_POST_INIT_STATE_2,
	LSM6DSO_POST_INIT_STATE_2_Wait,
	LSM6DSO_POST_INIT_STATE_2_Done,

	LSM6DSO_POST_INIT_STATE_3,
	LSM6DSO_POST_INIT_STATE_3_Wait,
	LSM6DSO_POST_INIT_STATE_3_Done,

	LSM6DSO_POST_INIT_STATE_END,
};

#define LSM6DSO_POST_INIT_STATE_Enum enum LSM6DSO_POST_INIT_STATE

struct LSM6DSO_DEVICE_INFO {
	I3C_DEVICE_INFO_t i3c_device;
	uint32_t baudrate;
	I3C_LSM6DSO_STATE_Enum state;
	LSM6DSO_POST_INIT_STATE_Enum post_init_state;
	uint16_t initMode;
	uint16_t rxLen;
	uint8_t rxBuf[16];
	uint8_t temp_val;
};

#define SPD5118_POST_INIT_STATE_Enum enum SPD5118_POST_INIT_STATE

enum SPD5118_POST_INIT_STATE {
	SPD5118_POST_INIT_STATE_Default,

	SPD5118_POST_INIT_STATE_Sync_Wait,
	SPD5118_POST_INIT_STATE_Sync_Done,

	SPD5118_POST_INIT_STATE_1_Wait,
	SPD5118_POST_INIT_STATE_1_Done,

	SPD5118_POST_INIT_STATE_2,
	SPD5118_POST_INIT_STATE_2_Wait,
	SPD5118_POST_INIT_STATE_2_Done,

	SPD5118_POST_INIT_STATE_3,
	SPD5118_POST_INIT_STATE_3_Wait,
	SPD5118_POST_INIT_STATE_3_Done,

	SPD5118_POST_INIT_STATE_4,
	SPD5118_POST_INIT_STATE_4_Wait,
	SPD5118_POST_INIT_STATE_4_Done,

	SPD5118_POST_INIT_STATE_5,
	SPD5118_POST_INIT_STATE_5_Wait,
	SPD5118_POST_INIT_STATE_5_Done,

	SPD5118_POST_INIT_STATE_6,
	SPD5118_POST_INIT_STATE_6_Wait,
	SPD5118_POST_INIT_STATE_6_Done,

	SPD5118_POST_INIT_STATE_7,
	SPD5118_POST_INIT_STATE_7_Wait,
	SPD5118_POST_INIT_STATE_7_Done,

	SPD5118_POST_INIT_STATE_8,
	SPD5118_POST_INIT_STATE_8_Wait,
	SPD5118_POST_INIT_STATE_8_Done,

	SPD5118_POST_INIT_STATE_9,
	SPD5118_POST_INIT_STATE_9_Wait,
	SPD5118_POST_INIT_STATE_9_Done,

	SPD5118_POST_INIT_STATE_10,
	SPD5118_POST_INIT_STATE_10_Wait,
	SPD5118_POST_INIT_STATE_10_Done,

	SPD5118_POST_INIT_STATE_11,
	SPD5118_POST_INIT_STATE_11_Wait,
	SPD5118_POST_INIT_STATE_11_Done,

	SPD5118_POST_INIT_STATE_12,
	SPD5118_POST_INIT_STATE_12_Wait,
	SPD5118_POST_INIT_STATE_12_Done,

	SPD5118_POST_INIT_STATE_13,
	SPD5118_POST_INIT_STATE_13_Wait,
	SPD5118_POST_INIT_STATE_13_Done,

	SPD5118_POST_INIT_STATE_END,
};

 #define SPD5118_STATE_Enum enum SPD5118_STATE

enum SPD5118_STATE {
	SPD5118_STATE_DEFAULT,
	SPD5118_STATE_RESET,
	SPD5118_STATE_CLEAR,
	SPD5118_STATE_IDLE,
	SPD5118_STATE_TASK_CANNOT_COMPLETE,
	SPD5118_STATE_WAIT_WR_OP,
	SPD5118_STATE_WAIT_NEXT_WR_OP,
	SPD5118_STATE_WAIT_WR_OP_END,
	SPD5118_STATE_GET_MR11,
	SPD5118_STATE_GET_MR11_END,
	SPD5118_STATE_WRITE_REG,
	SPD5118_STATE_WRITE_REG_END,
	SPD5118_STATE_WRITE_REG_NEXT,
	SPD5118_STATE_READ_REG,
	SPD5118_STATE_READ_REG_FAIL_NOT_PRESENT,
	SPD5118_STATE_READ_REG_FAIL_FORMAT,
	SPD5118_STATE_READ_REG_END,
	SPD5118_STATE_READ_REG_NEXT,
	SPD5118_STATE_GOTO_PAGE,
	SPD5118_STATE_GOTO_PAGE_CHECK,
	SPD5118_STATE_CHECK_PAGE,
	SPD5118_STATE_GOTO_PAGE_END,
	SPD5118_STATE_WRITE_EEPROM,
	SPD5118_STATE_READ_EEPROM,
	SPD5118_STATE_WRITE_ABORT_EEPROM, /* error case */

	SPD5118_STATE_DEFAULT_READ,
};

#define SPD5118_OP_Enum enum SPD5118_OP

enum SPD5118_OP {
	SPD5118_OP_PEC_SYNC,
	SPD5118_OP_REG_WRITE,
	SPD5118_OP_REG_READ,
	SPD5118_OP_REG_DEFAULT_POINTER_READ,
	SPD5118_OP_EEPROM_WRITE,
	SPD5118_OP_EEPROM_READ,
};

#define SPD5118_TASK_t struct SPD5118_TASK

struct SPD5118_TASK {
	/* used to link the next SPD5118 task if the current task is not finished */
	struct SPD5118_TASK *pNextTask;

	/* used to store task parameters */
	uint8_t address;
	SPD5118_OP_Enum op;
	bool bPEC;
	bool b2B;
	bool bRunI3C;   /* optional, used to switch i2c and i3c*/
	uint16_t offset;
	uint8_t *pWrBuf, *pRdBuf;
	uint16_t *pWrLen, *pRdLen;

	uint16_t retry_cnt;     /* used to retry */
	uint16_t timeout;       /* used to validate task timeout */
	uint16_t access_offset; /* used to record status for long read/write */
	uint8_t access_size;    /* used to validate read length */
};

#define SPD5118_DEVICE_INFO_t struct SPD5118_DEVICE_INFO

struct SPD5118_DEVICE_INFO {
	I3C_DEVICE_INFO_t i3c_device;
	uint32_t baudrate;

	uint8_t bPEC : 1;

	SPD5118_POST_INIT_STATE_Enum post_init_state;
	SPD5118_STATE_Enum state;
	uint8_t checkMask;
	uint16_t reset_timeout;

	uint8_t MR0;    /* Device Type, MSB */
	uint8_t MR1;    /* Device Type, LSB */
	uint8_t MR2;    /* Revision */
	uint8_t MR3;    /* Vendor ID, Byte 0 */
	uint8_t MR4;    /* Vendor ID, Byte 1 */
	uint8_t MR5;    /* Device Capability */
		/* [1] Internal Temperature Sensor Support */
		/* [0] Hub function support */
	uint8_t MR6;    /* Device Write Recovery Time Capability */
		/* [7:4]: 0, .., 10, 50, 100, 200, 500 */
		/* [1:0] == 00b, ns */
		/*       == 01b, us */
		/*       == 10b, ms */
		/*       == 11b, Reserved */
	uint8_t MR11;   /* I2C Legacy Mode Device Configuration*/
		/* [3] == 0b, 1 Byte addressing */
		/*     == 1b, 2 Byte addressing */
		/* [2:0], Non Volatile Memory Address Page Pointer in I2C Legacy Mode */
	uint8_t MR12;   /* Write Protection For NVM Blocks [7:0] */
	uint8_t MR13;   /* Write Protection for NVM Blocks [15:8] */
	uint8_t MR14;   /* Device Configuration - Host and Local Interface IO; */
		/* [5] Local Interface Pull Up Resistor Configuration */
	uint8_t MR18;   /* Device Configuration */
		/* [7] PEC, I3C mode only */
		/* [6] T bit Disable, I3C mode only */
		/* [5] Interface Selection */
		/* [4] Default Read Address Pointer Enable */
		/* [3:2], Default Read Pointer Starting Address */
		/* [1], Burst Length for Default Read Pointer Address for PEC Calculation */
	uint8_t MR19;   /* Clear Register MR51 Temperature Status Command */
		/* [3] CRIT_LOW, clear MR51[3] */
		/* [2] CRIT_HIGH, clear MR51[2] */
		/* [1] LOW, clear MR51[1] */
		/* [0] HIGH, clear MR51[0] */
	uint8_t MR20;   /* Clear Register MR52 Error Status Command */
		/* [7]  Write or Read Attempt while SPD Device Busy Error, Clear MR52[7] */
		/* [6]  Write Attempt to Protected NVM Block Error, Clear MR52[6] */
		/* [5]  Write Attempt to NVM Protection Register Error, Clear MR52[5] */
		/* [1]  PEC Error, Clear MR52[1] */
		/* [0]  Parity Error, Clear MR52[0] */
	uint8_t MR26;   /* TS Configuration */
		/* [0] Disable thermal sensor */
	uint8_t MR27;   /* Interrupt Configurations */
		/* [7] Clear MR48[8], MR51[3:0], MR52[7:5,3,1:0] */
		/* [4] IBI enabled for MR52[7:5, 1:0] */
		/* [3] MR27[4] = 1 & MR27[3] = 1 & MR51[3] = 1 generate IBI for CRIT_LOW */
		/* [2] MR27[4] = 1 & MR27[2] = 1 & MR51[2] = 1 generate IBI for CRIT_HIGH */
		/* [1] MR27[4] = 1 & MR27[1] = 1 & MR51[1] = 1 generate IBI for LOW */
		/* [0] MR27[4] = 1 & MR27[0] = 1 & MR51[0] = 1 generate IBI for HIGH */

	uint8_t MR28;   /* TS Temperature High Limit Configuration - Low Byte */
	uint8_t MR29;   /* TS Temperature High Limit Configuration - High Byte */
	uint8_t MR30;   /* TS Temperature Low Limit Configuration - Low Byte */
	uint8_t MR31;   /* TS Temperature Low Limit Configuration - High Byte */
	uint8_t MR32;   /* TS Critical Temperature High Limit Configuration - Low Byte */
	uint8_t MR33;   /* TS Critical Temperature High Limit Configuration - High Byte */
	uint8_t MR34;   /* TS Critical Temperature Low Limit Configuration - Low Byte */
	uint8_t MR35;   /* TS Critical Temperature Low Limit Configuration - High Byte */

	uint8_t MR48;   /* Device Status */
		/* [7] Pending IBI */
		/* [3] Write Operation Status */
		/* [2] Write Protect Override Status */
	uint8_t MR49;   /* TS Current Sensed Temperature - Low Byte */
	uint8_t MR50;   /* TS Current Sensed Temperature - High Byte */
	uint8_t MR51;   /* TS Temperature Status */
		/* [3] TS < CRIT_LOW */
		/* [2] TS > CRIT_HIGH */
		/* [1] TS < LOW */
		/* [0] TS > HIGH */
	uint8_t MR52;   /* Hub, Thermal and NVM Error Status */
		/* [7] BUSY_ERROR */
		/* [6] WR_NVM_BLK_ERROR */
		/* [5] WR_NVM_PRO_REG_ERROR */
		/* [1] PEC_ERROR */
		/* [0] PAR_ERROR */

	SPD5118_TASK_t *pOpListHead; /* op list for a specified device */

	uint16_t initMode;
};


#define LSM6DSO_DEVICE_INFO_t struct LSM6DSO_DEVICE_INFO

/* PENDINT definition */
#define I3C_PENDING_NONE	0x00
#define I3C_PENDING_MCTP	0x01

#endif /* EXPORTS_PUB_I3C_H_ */
