/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_i3c

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(npcm_i3c, CONFIG_I3C_LOG_LEVEL);

#define NPCM_I3C_CHK_TIMEOUT_US 10000 /* Timeout for checking register status */
#define I3C_SCL_PP_FREQ_MAX_MHZ 12500000
#define I3C_SCL_OD_FREQ_MAX_MHZ 4170000

#define I3C_BUS_TLOW_PP_MIN_NS 24  /* T_LOW period in push-pull mode */
#define I3C_BUS_TLOW_OD_MIN_NS 200 /* T_LOW period in open-drain mode */

#define PPBAUD_DIV_MAX  (BIT(GET_FIELD_SZ(NPCM_I3C_MCONFIG_PPBAUD)) - 1)  /* PPBAUD divider max */
#define I2CBAUD_DIV_MAX (BIT(GET_FIELD_SZ(NPCM_I3C_MCONFIG_I2CBAUD)) - 1) /* I2C divider max */

#define DAA_TGT_INFO_SZ 0x8 /* 8 bytes = PID(6) + BCR(1) + DCR(1) */

/* Default maximum time we allow for an I3C transfer */
#define I3C_TRANS_TIMEOUT_MS K_MSEC(100)

#define I3C_CLK_FREQ_48_MHZ MHZ(48)
#define I3C_CLK_FREQ_96_MHZ MHZ(96)

#define I3C_STATUS_CLR_MASK                                                                        \
	(BIT(NPCM_I3C_MSTATUS_TGTSTART) | BIT(NPCM_I3C_MSTATUS_MCTRLDONE) |                        \
	 BIT(NPCM_I3C_MSTATUS_COMPLETE) | BIT(NPCM_I3C_MSTATUS_IBIWON) |                           \
	 BIT(NPCM_I3C_MSTATUS_NOWCNTLR))

#define I3C_NPCM_HW_IDX(n) ((n & 0xFFF) >> 9)

#ifdef CONFIG_I3C_NPCM_DMA
#define I3C_NPCM_PDMA_MUX_ID(n, rnw)                                                               \
	rnw ? ((((n & 0xFFF) >> 9) * 2) + 5) : ((((n & 0xFFF) >> 9) * 2) + 6)
#endif

/* Supported I3C clock frequency */
enum npcm_i3c_clk_speed {
	NPCM_I3C_CLK_FREQ_48MHZ,
	NPCM_I3C_CLK_FREQ_96MHZ,
};

/* I3C timing configuration for each i3c/i2c speed */
struct npcm_i3c_timing_cfg {
	uint8_t ppbaud;   /* Push-Pull high period */
	uint8_t pplow;    /* Push-Pull low period */
	uint8_t odhpp;    /* Open-Drain high period */
	uint8_t odbaud;   /* Open-Drain low period */
	uint8_t i2c_baud; /* I2C period */
};

#define NPCM_I3C_CFG_SET(idx, a, b, c, d, e)                                                       \
	[idx] = {.ppbaud = a, .pplow = b, .odhpp = c, .odbaud = d, .i2c_baud = e}

/* Recommended I3C timing values are based on I3C frequency 48 or 96 MHz */
static const struct npcm_i3c_timing_cfg npcm_def_speed_cfg[] = {
	/* PP = 12.5 mhz, OD = 4.17 Mhz, i2c = 1.0 Mhz */
	NPCM_I3C_CFG_SET(NPCM_I3C_CLK_FREQ_48MHZ, 1, 0, 1, 4, 3),
	NPCM_I3C_CFG_SET(NPCM_I3C_CLK_FREQ_96MHZ, 3, 0, 1, 4, 3),
};

struct pdma_dsct_reg {
	volatile uint32_t CTL;
	volatile uint32_t SA;
	volatile uint32_t DA;
	volatile uint32_t NEXT;
};

struct npcm_i3c_config {
	/* Common I3C Driver Config */
	struct i3c_driver_config common;

	/* Pointer to controller registers. */
	struct i3c_reg *base;

	/* Clock control subsys related struct. */
	uint32_t clk_cfg;

	/* Pointer to pin control device. */
	const struct pinctrl_dev_config *pincfg;

	/* Interrupt configuration function. */
	void (*irq_config_func)(const struct device *dev);

	struct {
		uint32_t i3c_pp_scl_hz; /* I3C push pull clock frequency in Hz. */
		uint32_t i3c_od_scl_hz; /* I3C open drain clock frequency in Hz. */
		uint32_t i2c_scl_hz;    /* I2C clock frequency in Hz */
	} clocks;

#ifdef CONFIG_I3C_NPCM_DMA
	struct pdma_dsct_reg *pdma_rx;
	struct pdma_dsct_reg *pdma_tx;
#endif
};

struct npcm_i3c_data {
	struct i3c_driver_data common; /* Common i3c driver data */
	struct k_mutex lock_mutex;     /* Mutex of i3c controller */
	struct k_sem sync_sem;         /* Semaphore used for synchronization */
	struct k_sem ibi_lock_sem;     /* Semaphore used for ibi */

#ifdef CONFIG_I3C_USE_IBI
	struct {
		/* List of addresses used in the MIBIRULES register. */
		uint8_t addr[5];

		/* Number of valid addresses in MIBIRULES. */
		uint8_t num_addr;

		/* True if all addresses have MSB set. */
		bool msb;

		/*
		 * True if all target devices require mandatory byte
		 * for IBI.
		 */
		bool has_mandatory_byte;
	} ibi;
#endif

#ifdef CONFIG_I3C_NPCM_DMA
	struct pdma_dsct_reg dsct_sg[2] __aligned(4); /* use for dma, 4-bytes align */
#endif
};

struct i3c_reg {
	/* 0x000: Controller Configuration */
	volatile uint32_t MCONFIG;
	/* 0x004: Target Configuration */
	volatile uint32_t CONFIG;
	/* 0x008: Target Status */
	volatile uint32_t STATUS;
	/* 0x00C: Target I3C Control */
	volatile uint32_t CTRL;
	/* 0x010: Target Interrupt Enable Set */
	volatile uint32_t INTSET;
	/* 0x014: Target Interrupt Enable Clear */
	volatile uint32_t INTCLR;
	/* 0x018: Target Interrupt Masked */
	volatile uint32_t INTMASKED;
	/* 0x01C: Target Error and Warning */
	volatile uint32_t ERRWARN;
	/* 0x020: Target DMA Control */
	volatile uint32_t DMACTRL;
	volatile uint32_t reserved1[2];
	/* 0x02C: Target Data Control */
	volatile uint32_t DATACTRL;
	/* 0x030: Target Write Byte Data */
	volatile uint32_t WDATAB;
	/* 0x034: Target Write Byte Data as End */
	volatile uint32_t WDATABE;
	/* 0x038: Target Write Half-Word Data */
	volatile uint32_t WDATAH;
	/* 0x03C: Target Write Half-Word Data as End */
	volatile uint32_t WDATAHE;
	/* 0x040: Target Read Byte Data */
	volatile uint32_t RDATAB;
	volatile uint32_t reserved2;
	/* 0x048: Target Read Half-Word Data */
	volatile uint32_t RDATAH;
	volatile uint32_t reserved3[2];
	/* 0x054: Target Byte-Only Write Byte Data */
	volatile uint8_t WDATAB1;
	volatile uint8_t reserved4[11];
	/* 0x060: Target Capabilities */
	volatile uint32_t CAPABILITIES;
	/* 0x064: Target Dynamic Address */
	volatile uint32_t DYNADDR;
	/* 0x068: Target Maximum Limits */
	volatile uint32_t MAXLIMITS;
	/* 0x06C: Target Part Number */
	volatile uint32_t PARTNO;
	/* 0x070: Target ID Extension */
	volatile uint32_t IDEXT;
	/* 0x074: Target Vendor ID */
	volatile uint32_t VENDORID;
	/* 0x078: Target Timing Control Clock */
	volatile uint32_t TCCLOCK;
	volatile uint32_t reserved5[2];
	/* 0x084: Controller Control */
	volatile uint32_t MCTRL;
	/* 0x088: Controller Status */
	volatile uint32_t MSTATUS;
	/* 0x08C: Controller IBI Registry and Rules */
	volatile uint32_t IBIRULES;
	/* 0x090: Controller Interrupt Enable Set */
	volatile uint32_t MINTSET;
	/* 0x094: Controller Interrupt Enable Clear */
	volatile uint32_t MINTCLR;
	/* 0x098: Controller Interrupt Masked */
	volatile uint32_t MINTMASKED;
	/* 0x09C: Controller Error and Warning */
	volatile uint32_t MERRWARN;
	/* 0x0A0: Controller DMA Control */
	volatile uint32_t MDMACTRL;
	volatile uint32_t reserved6[2];
	/* 0x0AC: Controller Data Control */
	volatile uint32_t MDATACTRL;
	/* 0x0B0: Controller Write Byte Data */
	volatile uint32_t MWDATAB;
	/* 0x0B4: Controller Write Byte Data as End */
	volatile uint32_t MWDATABE;
	/* 0x0B8: Controller Write Half-Word Data */
	volatile uint32_t MWDATAH;
	/* 0x0BC: Controller Write Half-Word Data as End */
	volatile uint32_t MWDATAHE;
	/* 0x0C0: Controller Read Byte Data */
	volatile uint32_t MRDATAB;
	volatile uint32_t reserved7;
	/* 0x0C8: Controller Read Half-Word Data */
	volatile uint32_t MRDATAH;
	/* 0x0CC: Controller Byte-Only Write Byte Data */
	volatile uint8_t MWDATAB1;
	volatile uint8_t reserved8[3];
	/* 0x0D0: Controller Start or Continue SDR Message */
	volatile uint32_t MWMSG_SDR;
	/* 0x0D4: Controller Read SDR Message Data */
	volatile uint32_t MRMSG_SDR;
	/* 0x0D8: Controller Start or Continue DDR Message */
	volatile uint32_t MWMSG_DDR;
	/* 0x0DC: Controller Read DDR Message Data */
	volatile uint32_t MRMSG_DDR;
	volatile uint32_t reserved9;
	/* 0x0E4: Controller Dynamic Address */
	volatile uint32_t MDYNADDR;
	volatile uint32_t reserved10[8];
	/* 0x108: Target HDR Command Register */
	volatile uint32_t HDRCMD;
	volatile uint32_t reserved11[13];
	/* 0x140: Target Extended IBI Data Register 1 */
	volatile uint32_t IBIEXT1;
	/* 0x144: Target Extended IBI Data Register 2 */
	volatile uint32_t IBIEXT2;
	volatile uint32_t reserved12[45];
	/* 0x1FC: Target Block ID */
	volatile uint32_t ID;
};

/* I3C Controller register fields */
#define NPCM_I3C_MCONFIG_I2CBAUD        FIELD(28, 4)
#define NPCM_I3C_MCONFIG_ODHPP          24
#define NPCM_I3C_MCONFIG_ODBAUD         FIELD(16, 8)
#define NPCM_I3C_MCONFIG_PPLOW          FIELD(12, 4)
#define NPCM_I3C_MCONFIG_PPBAUD         FIELD(8, 4)
#define NPCM_I3C_MCONFIG_ODSTOP         6
#define NPCM_I3C_MCONFIG_DISTO          3
#define NPCM_I3C_MCONFIG_CTRENA         FIELD(0, 2)
#define NPCM_I3C_MCTRL_RDTERM           FIELD(16, 8)
#define NPCM_I3C_MCTRL_ADDR             FIELD(9, 7)
#define NPCM_I3C_MCTRL_DIR              8
#define NPCM_I3C_MCTRL_IBIRESP          FIELD(6, 2)
#define NPCM_I3C_MCTRL_TYPE             FIELD(4, 2)
#define NPCM_I3C_MCTRL_REQUEST          FIELD(0, 3)
#define NPCM_I3C_MSTATUS_IBIADDR        FIELD(24, 7)
#define NPCM_I3C_MSTATUS_NOWCNTLR       19
#define NPCM_I3C_MSTATUS_ERRWARN        15
#define NPCM_I3C_MSTATUS_IBIWON         13
#define NPCM_I3C_MSTATUS_TXNOTFULL      12
#define NPCM_I3C_MSTATUS_RXPEND         11
#define NPCM_I3C_MSTATUS_COMPLETE       10
#define NPCM_I3C_MSTATUS_MCTRLDONE      9
#define NPCM_I3C_MSTATUS_TGTSTART       8
#define NPCM_I3C_MSTATUS_IBITYPE        FIELD(6, 2)
#define NPCM_I3C_MSTATUS_NACKED         5
#define NPCM_I3C_MSTATUS_BETWEEN        4
#define NPCM_I3C_MSTATUS_STATE          FIELD(0, 3)
#define NPCM_I3C_IBIRULES_NOBYTE        31
#define NPCM_I3C_IBIRULES_MSB0          30
#define NPCM_I3C_IBIRULES_ADDR4         FIELD(24, 6)
#define NPCM_I3C_IBIRULES_ADDR3         FIELD(18, 6)
#define NPCM_I3C_IBIRULES_ADDR2         FIELD(12, 6)
#define NPCM_I3C_IBIRULES_ADDR1         FIELD(6, 6)
#define NPCM_I3C_IBIRULES_ADDR0         FIELD(0, 6)
#define NPCM_I3C_MINTSET_NOWMASTER      19
#define NPCM_I3C_MINTSET_ERRWARN        15
#define NPCM_I3C_MINTSET_IBIWON         13
#define NPCM_I3C_MINTSET_TXNOTFULL      12
#define NPCM_I3C_MINTSET_RXPEND         11
#define NPCM_I3C_MINTSET_COMPLETE       10
#define NPCM_I3C_MINTSET_MCTRLDONE      9
#define NPCM_I3C_MINTSET_TGTSTART       8
#define NPCM_I3C_MINTCLR_NOWMASTER      19
#define NPCM_I3C_MINTCLR_ERRWARN        15
#define NPCM_I3C_MINTCLR_IBIWON         13
#define NPCM_I3C_MINTCLR_TXNOTFULL      12
#define NPCM_I3C_MINTCLR_RXPEND         11
#define NPCM_I3C_MINTCLR_COMPLETE       10
#define NPCM_I3C_MINTCLR_MCTRLDONE      9
#define NPCM_I3C_MINTCLR_TGTSTART       8
#define NPCM_I3C_MINTMASKED_NOWMASTER   19
#define NPCM_I3C_MINTMASKED_ERRWARN     15
#define NPCM_I3C_MINTMASKED_IBIWON      13
#define NPCM_I3C_MINTMASKED_TXNOTFULL   12
#define NPCM_I3C_MINTMASKED_RXPEND      11
#define NPCM_I3C_MINTMASKED_COMPLETE    10
#define NPCM_I3C_MINTMASKED_MCTRLDONE   9
#define NPCM_I3C_MINTMASKED_TGTSTART    8
#define NPCM_I3C_MERRWARN_TIMEOUT       20
#define NPCM_I3C_MERRWARN_INVREQ        19
#define NPCM_I3C_MERRWARN_MSGERR        18
#define NPCM_I3C_MERRWARN_OWRITE        17
#define NPCM_I3C_MERRWARN_OREAD         16
#define NPCM_I3C_MERRWARN_HCRC          10
#define NPCM_I3C_MERRWARN_HPAR          9
#define NPCM_I3C_MERRWARN_TERM          4
#define NPCM_I3C_MERRWARN_WRABT         3
#define NPCM_I3C_MERRWARN_NACK          2
#define NPCM_I3C_MDMACTRL_DMAWIDTH      FIELD(4, 2)
#define NPCM_I3C_MDMACTRL_DMATB         FIELD(2, 2)
#define NPCM_I3C_MDMACTRL_DMAFB         FIELD(0, 2)
#define NPCM_I3C_MDATACTRL_RXEMPTY      31
#define NPCM_I3C_MDATACTRL_TXFULL       30
#define NPCM_I3C_MDATACTRL_RXCOUNT      FIELD(24, 5)
#define NPCM_I3C_MDATACTRL_TXCOUNT      FIELD(16, 5)
#define NPCM_I3C_MDATACTRL_RXTRIG       FIELD(6, 2)
#define NPCM_I3C_MDATACTRL_TXTRIG       FIELD(4, 2)
#define NPCM_I3C_MDATACTRL_UNLOCK       3
#define NPCM_I3C_MDATACTRL_FLUSHFB      1
#define NPCM_I3C_MDATACTRL_FLUSHTB      0
#define NPCM_I3C_MWDATAB_END_A          16
#define NPCM_I3C_MWDATAB_END_B          8
#define NPCM_I3C_MWDATAB_DATA           FIELD(0, 8)
#define NPCM_I3C_MWDATABE_DATA          FIELD(0, 8)
#define NPCM_I3C_MWDATAH_END            16
#define NPCM_I3C_MWDATAH_DATA1          FIELD(8, 8)
#define NPCM_I3C_MWDATAH_DATA0          FIELD(0, 8)
#define NPCM_I3C_MWDATAHE_DATA1         FIELD(8, 8)
#define NPCM_I3C_MWDATAHE_DATA0         FIELD(0, 8)
#define NPCM_I3C_MRDATAB_DATA           FIELD(0, 8)
#define NPCM_I3C_MRDATAH_DATA1          FIELD(8, 8)
#define NPCM_I3C_MRDATAH_DATA0          FIELD(0, 8)
#define NPCM_I3C_MWDATAB1_DATA          FIELD(0, 8)
#define NPCM_I3C_MWMSG_SDR_CONTROL_LEN  FIELD(11, 5)
#define NPCM_I3C_MWMSG_SDR_CONTROL_I2C  10
#define NPCM_I3C_MWMSG_SDR_CONTROL_END  8
#define NPCM_I3C_MWMSG_SDR_CONTROL_ADDR FIELD(1, 7)
#define NPCM_I3C_MWMSG_SDR_CONTROL_DIR  0
#define NPCM_I3C_MWMSG_SDR_DATA         FIELD(0, 16)
#define NPCM_I3C_MRMSG_SDR_DATA         FIELD(0, 16)
#define NPCM_I3C_MWMSG_DDR_CONTROL_END  14
#define NPCM_I3C_MWMSG_DDR_CONTROL_LEN  FIELD(0, 10)
#define NPCM_I3C_MWMSG_DDR_CONTROL_ADDR FIELD(9, 7)
#define NPCM_I3C_MWMSG_DDR_CONTROL_DIR  7
#define NPCM_I3C_MWMSG_DDR_CONTROL_CMD  FIELD(0, 7)
#define NPCM_I3C_MWMSG_DDR_DATA         FIELD(0, 16)
#define NPCM_I3C_MRMSG_DDR_DATA         FIELD(0, 16)
#define NPCM_I3C_MDYNADDR_DADDR         FIELD(1, 7)
#define NPCM_I3C_MDYNADDR_DAVALID       0

/* MCONFIG options */
#define MCONFIG_CTRENA_OFF        0x0
#define MCONFIG_CTRENA_ON         0x1
#define MCONFIG_CTRENA_CAPABLE    0x2
#define MCONFIG_HKEEP_EXT_SDA_SCL 0x3

/* MCTRL options */
#define MCTRL_REQUEST_NONE          0 /* None */
#define MCTRL_REQUEST_EMITSTARTADDR 1 /* Emit a START */
#define MCTRL_REQUEST_EMITSTOP      2 /* Emit a STOP */
#define MCTRL_REQUEST_IBIACKNACK    3 /* Manually ACK or NACK an IBI */
#define MCTRL_REQUEST_PROCESSDAA    4 /* Starts the DAA process */
#define MCTRL_REQUEST_FORCEEXIT     6 /* Emit HDR Exit Pattern  */
/* Emits a START with address 7Eh when a slave pulls I3C_SDA low to request an IBI */
#define MCTRL_REQUEST_AUTOIBI       7

/* ACK with mandatory byte determined by IBIRULES or ACK with no mandatory byte */
#define MCTRL_IBIRESP_ACK           0
#define MCTRL_IBIRESP_NACK          1 /* NACK */
#define MCTRL_IBIRESP_ACK_MANDATORY 2 /* ACK with mandatory byte  */
#define MCTRL_IBIRESP_MANUAL        3

enum npcm_i3c_mctrl_type {
	NPCM_I3C_MCTRL_TYPE_I3C,
	NPCM_I3C_MCTRL_TYPE_I2C,
	NPCM_I3C_MCTRL_TYPE_I3C_HDR_DDR,
};

/* MSTATUS options */
#define MSTATUS_STATE_IDLE    0x0
#define MSTATUS_STATE_TGTREQ  0x1
#define MSTATUS_STATE_NORMACT 0x3 /* SDR message mode */
#define MSTATUS_STATE_MSGDDR  0x4
#define MSTATUS_STATE_DAA     0x5
#define MSTATUS_STATE_IBIACK  0x6
#define MSTATUS_STATE_IBIRCV  0x7
#define MSTATUS_IBITYPE_NONE  0x0
#define MSTATUS_IBITYPE_IBI   0x1
#define MSTATUS_IBITYPE_CR    0x2
#define MSTATUS_IBITYPE_HJ    0x3

/* IBIRULES */
#define IBIRULES_ADDR_MSK   0x3F
#define IBIRULES_ADDR_SHIFT 0x6

/* MDMACTRL options */
#define MDMA_DMAFB_DISABLE      0x0
#define MDMA_DMAFB_EN_ONE_FRAME 0x1
#define MDMA_DMAFB_EN_MANUAL    0x2
#define MDMA_DMATB_DISABLE      0x0
#define MDMA_DMATB_EN_ONE_FRAME 0x1
#define MDMA_DMATB_EN_MANUAL    0x2

/* I3C Target register fields */
#define NPCM_I3C_CONFIG_SADDR           FIELD(25, 7)
#define NPCM_I3C_CONFIG_BAMATCH         FIELD(16, 7)
#define NPCM_I3C_CONFIG_HDRCMD          10
#define NPCM_I3C_CONFIG_OFFLINE         9
#define NPCM_I3C_CONFIG_IDRAND          8
#define NPCM_I3C_CONFIG_DDROK           4
#define NPCM_I3C_CONFIG_S0IGNORE        3
#define NPCM_I3C_CONFIG_MATCHSS         2
#define NPCM_I3C_CONFIG_NACK            1
#define NPCM_I3C_CONFIG_TGTENA          0
#define NPCM_I3C_STATUS_TIMECTRL        FIELD(30, 2)
#define NPCM_I3C_STATUS_ACTSTATE        FIELD(28, 2)
#define NPCM_I3C_STATUS_HJDIS           27
#define NPCM_I3C_STATUS_MRDIS           25
#define NPCM_I3C_STATUS_IBIDIS          24
#define NPCM_I3C_STATUS_EVDET           FIELD(20, 2)
#define NPCM_I3C_STATUS_EVENT           18
#define NPCM_I3C_STATUS_CHANDLED        17
#define NPCM_I3C_STATUS_DDRMATCH        16
#define NPCM_I3C_STATUS_ERRWARN         15
#define NPCM_I3C_STATUS_CCC             14
#define NPCM_I3C_STATUS_DACHG           13
#define NPCM_I3C_STATUS_TXNOTFULL       12
#define NPCM_I3C_STATUS_RXPEND          11
#define NPCM_I3C_STATUS_STOP            10
#define NPCM_I3C_STATUS_MATCHED         9
#define NPCM_I3C_STATUS_START           8
#define NPCM_I3C_STATUS_STHDR           6
#define NPCM_I3C_STATUS_STDAA           5
#define NPCM_I3C_STATUS_STREQWR         4
#define NPCM_I3C_STATUS_STREQRD         3
#define NPCM_I3C_STATUS_STCCCH          2
#define NPCM_I3C_STATUS_STMSG           1
#define NPCM_I3C_STATUS_STNOTSTOP       0
#define NPCM_I3C_CTRL_VENDINFO          FIELD(24, 8)
#define NPCM_I3C_CTRL_ACTSTATE          FIELD(20, 2)
#define NPCM_I3C_CTRL_PENDINT           FIELD(16, 4)
#define NPCM_I3C_CTRL_IBIDATA           FIELD(8, 8)
#define NPCM_I3C_CTRL_EXTDATA           3
#define NPCM_I3C_CTRL_EVENT             FIELD(0, 2)
#define NPCM_I3C_INTSET_EVENT           18
#define NPCM_I3C_INTSET_CHANDLED        17
#define NPCM_I3C_INTSET_DDRMATCHED      16
#define NPCM_I3C_INTSET_ERRWARN         15
#define NPCM_I3C_INTSET_CCC             14
#define NPCM_I3C_INTSET_DACHG           13
#define NPCM_I3C_INTSET_TXNOTFULL       12
#define NPCM_I3C_INTSET_RXPEND          11
#define NPCM_I3C_INTSET_STOP            10
#define NPCM_I3C_INTSET_MATCHED         9
#define NPCM_I3C_INTSET_START           8
#define NPCM_I3C_INTCLR_EVENT           18
#define NPCM_I3C_INTCLR_CHANDLED        17
#define NPCM_I3C_INTCLR_DDRMATCHED      16
#define NPCM_I3C_INTCLR_ERRWARN         15
#define NPCM_I3C_INTCLR_CCC             14
#define NPCM_I3C_INTCLR_DACHG           13
#define NPCM_I3C_INTCLR_TXNOTFULL       12
#define NPCM_I3C_INTCLR_RXPEND          11
#define NPCM_I3C_INTCLR_STOP            10
#define NPCM_I3C_INTCLR_MATCHED         9
#define NPCM_I3C_INTCLR_START           8
#define NPCM_I3C_INTMASKED_EVENT        18
#define NPCM_I3C_INTMASKED_CHANDLED     17
#define NPCM_I3C_INTMASKED_DDRMATCHED   16
#define NPCM_I3C_INTMASKED_ERRWARN      15
#define NPCM_I3C_INTMASKED_CCC          14
#define NPCM_I3C_INTMASKED_DACHG        13
#define NPCM_I3C_INTMASKED_TXNOTFULL    12
#define NPCM_I3C_INTMASKED_RXPEND       11
#define NPCM_I3C_INTMASKED_STOP         10
#define NPCM_I3C_INTMASKED_MATCHED      9
#define NPCM_I3C_INTMASKED_START        8
#define NPCM_I3C_ERRWARN_OWRITE         17
#define NPCM_I3C_ERRWARN_OREAD          16
#define NPCM_I3C_ERRWARN_S0S1           11
#define NPCM_I3C_ERRWARN_HCRC           10
#define NPCM_I3C_ERRWARN_HPAR           9
#define NPCM_I3C_ERRWARN_SPAR           8
#define NPCM_I3C_ERRWARN_INVSTART       4
#define NPCM_I3C_ERRWARN_TERM           3
#define NPCM_I3C_ERRWARN_URUNNACK       2
#define NPCM_I3C_ERRWARN_URUN           1
#define NPCM_I3C_ERRWARN_ORUN           0
#define NPCM_I3C_DMACTRL_DMAWIDTH       FIELD(4, 2)
#define NPCM_I3C_DMACTRL_DMATB          FIELD(2, 2)
#define NPCM_I3C_DMACTRL_DMAFB          FIELD(0, 2)
#define NPCM_I3C_DATACTRL_RXEMPTY       31
#define NPCM_I3C_DATACTRL_TXFULL        30
#define NPCM_I3C_DATACTRL_RXCOUNT       FIELD(24, 5)
#define NPCM_I3C_DATACTRL_TXCOUNT       FIELD(16, 5)
#define NPCM_I3C_DATACTRL_RXTRIG        FIELD(6, 2)
#define NPCM_I3C_DATACTRL_TXTRIG        FIELD(4, 2)
#define NPCM_I3C_DATACTRL_UNLOCK        3
#define NPCM_I3C_DATACTRL_FLUSHFB       1
#define NPCM_I3C_DATACTRL_FLUSHTB       0
#define NPCM_I3C_WDATAB_END_A           16
#define NPCM_I3C_WDATAB_END_B           8
#define NPCM_I3C_WDATAB_DATA            FIELD(0, 8)
#define NPCM_I3C_WDATABE_DATA           FIELD(0, 8)
#define NPCM_I3C_WDATAH_END             16
#define NPCM_I3C_WDATAH_DATA1           FIELD(8, 8)
#define NPCM_I3C_WDATAH_DATA0           FIELD(0, 8)
#define NPCM_I3C_WDATAHE_DATA1          FIELD(8, 8)
#define NPCM_I3C_WDATAHE_DATA0          FIELD(0, 8)
#define NPCM_I3C_RDATAB_DATA0           FIELD(0, 8)
#define NPCM_I3C_RDATAH_DATA1           FIELD(8, 8)
#define NPCM_I3C_RDATAH_DATA0           FIELD(0, 8)
#define NPCM_I3C_WDATAB1_DATA           FIELD(0, 8)
#define NPCM_I3C_CAPABILITIES_DMA       31
#define NPCM_I3C_CAPABILITIES_INT       30
#define NPCM_I3C_CAPABILITIES_FIFORX    FIELD(28, 2)
#define NPCM_I3C_CAPABILITIES_FIFOTX    FIELD(26, 2)
#define NPCM_I3C_CAPABILITIES_TIMECTRL  21
#define NPCM_I3C_CAPABILITIES_IBI_MR_HJ FIELD(16, 5)
#define NPCM_I3C_CAPABILITIES_CCCHANDLE FIELD(12, 4)
#define NPCM_I3C_CAPABILITIES_SADDR     FIELD(10, 2)
#define NPCM_I3C_CAPABILITIES_HDRSUPP   6
#define NPCM_I3C_CAPABILITIES_IDREG     FIELD(2, 4)
#define NPCM_I3C_CAPABILITIES_IDENA     FIELD(0, 2)
#define NPCM_I3C_DYNADDR_DADDR          FIELD(1, 7)
#define NPCM_I3C_DYNADDR_DAVALID        0
#define NPCM_I3C_MAXLIMITS_MAXWR        FIELD(16, 12)
#define NPCM_I3C_MAXLIMITS_MAXRD        FIELD(0, 12)
#define NPCM_I3C_PARTNO_PARTNO          FIELD(0, 32)
#define NPCM_I3C_IDEXT_BCR              FIELD(16, 8)
#define NPCM_I3C_IDEXT_DCR              FIELD(8, 8)
#define NPCM_I3C_VENDORID_VID           FIELD(0, 15)
#define NPCM_I3C_TCCLOCK_FREQ           FIELD(8, 8)
#define NPCM_I3C_TCCLOCK_ACCURACY       FIELD(0, 8)
#define NPCM_I3C_IBIEXT1_EXT3           FIELD(24, 8)
#define NPCM_I3C_IBIEXT1_EXT2           FIELD(16, 8)
#define NPCM_I3C_IBIEXT1_EXT1           FIELD(8, 8)
#define NPCM_I3C_IBIEXT1_MAX            FIELD(4, 3)
#define NPCM_I3C_IBIEXT1_CNT            FIELD(0, 3)
#define NPCM_I3C_IBIEXT2_EXT7           FIELD(24, 8)
#define NPCM_I3C_IBIEXT2_EXT6           FIELD(16, 8)
#define NPCM_I3C_IBIEXT2_EXT5           FIELD(8, 8)
#define NPCM_I3C_IBIEXT2_EXT4           FIELD(0, 8)
#define NPCM_I3C_HDRCMD_NEWCMD          31
#define NPCM_I3C_HDRCMD_OVFLW           30
#define NPCM_I3C_HDRCMD_CMD0            FIELD(0, 8)
#define NPCM_I3C_ID_ID                  FIELD(0, 32)

struct pdma_reg {
	/* 0x000 ~ 0x0DC: Descriptor Table Control Register 0 - 13 */
	struct pdma_dsct_reg PDMA_DSCT[14];
	volatile uint32_t reserved1[8];
	/* 0x100 ~ 0x134: Current Scatter-Gather Descriptor Table Address 0 - 13 */
	volatile uint32_t PDMA_CURSCAT[14];
	volatile uint32_t reserved2[178];
	/* 0x400: PDMA Channel Control Register */
	volatile uint32_t PDMA_CHCTL;
	/* 0x404: PDMA Stop Transfer Register */
	volatile uint32_t PDMA_STOP;
	/* 0x408: PDMA Software Request Register */
	volatile uint32_t PDMA_SWREQ;
	/* 0x40C: PDMA Request Active Flag Register */
	volatile uint32_t PDMA_TRGSTS;
	/* 0x410: PDMA Fixed Priority Setting Register */
	volatile uint32_t PDMA_PRISET;
	/* 0x414: PDMA Fixed Priority Clear Register */
	volatile uint32_t PDMA_PRICLR;
	/* 0x418: PDMA Interrupt Enable Control Register */
	volatile uint32_t PDMA_INTEN;
	/* 0x41C: PDMA PDMA Interrupt Status Register */
	volatile uint32_t PDMA_INTSTS;
	/* 0x420: PDMA Read/Write Target Abort Flag Register */
	volatile uint32_t PDMA_ABTSTS;
	/* 0x424: PDMA Transfer Done Flag Register */
	volatile uint32_t PDMA_TDSTS;
	/* 0x428: PDMA Scatter-Gather Transfer Done Flag Register */
	volatile uint32_t PDMA_SCATSTS;
	/* 0x42C: PDMA Transfer on Active Flag Register */
	volatile uint32_t PDMA_TACTSTS;
	volatile uint32_t reserved3[3];
	/* 0x43C: PDMA Scatter-Gather Descriptor Table Base Address Register */
	volatile uint32_t PDMA_SCATBA;
	volatile uint32_t reserved4[16];
	/* 0x480: PDMA Source Module Select Register 0 - 3 */
	volatile uint32_t PDMA_REQSEL[4];
};

#define NPCM_PDMA_INTSTS_TEIF            2
#define NPCM_PDMA_INTSTS_TDIF            1
#define NPCM_PDMA_INTSTS_ABTIF           0
#define NPCM_PDMA_SCATBA_16BITS          FIELD(16, 16)
#define NPCM_PDMA_REQSEL_CHANNEL(ch)     FIELD((ch * 8), 7)
#define NPCM_PDMA_DSCT_CTL_TXCNT         FIELD(16, 14)
#define NPCM_PDMA_DSCT_CTL_TXWIDTH       FIELD(12, 2)
#define NPCM_PDMA_DSCT_CTL_TX_WIDTH_8    0x0
#define NPCM_PDMA_DSCT_CTL_TX_WIDTH_16   0x1
#define NPCM_PDMA_DSCT_CTL_TX_WIDTH_32   0x2
#define NPCM_PDMA_DSCT_CTL_DAINC         FIELD(10, 2)
#define NPCM_PDMA_DSCT_CTL_DAINC_FIX     0x3
#define NPCM_PDMA_DSCT_CTL_SAINC         FIELD(8, 2)
#define NPCM_PDMA_DSCT_CTL_SAINC_FIX     0x3
#define NPCM_PDMA_DSCT_CTL_TBINTDIS      7
#define NPCM_PDMA_DSCT_CTL_BURSIZE       FIELD(4, 3)
#define NPCM_PDMA_DSCT_CTL_TXTYPE_SINGLE 2
#define NPCM_PDMA_DSCT_CTL_OPMODE        FIELD(0, 2)
#define NPCM_PDMA_DSCT_CTL_OPMODE_STOP   0x0
#define NPCM_PDMA_DSCT_CTL_OPMODE_BASIC  0x1
#define NPCM_PDMA_DSCT_CTL_OPMODE_SGM    0x2
#define NPCM_PDMA_DSCT_NEXT_DSCT_OFFSET  FIELD(2, 14)

/* n == pdma descriptor table address */
#define NPCM_PDMA_BASE(n)         (n & 0xFFFFFF00)
#define NPCM_PDMA_DSCT_IDX(n)     ((n - (n & 0xFFFFFF00)) >> 4)
#define NPCM_PDMA_CHANNEL_PER_REQ 0x4

/*
 * Power Management Controller (PMC) device registers
 */
struct pmc_reg {
	/* 0x000: Power Management Controller */
	volatile uint8_t PMCSR;
	volatile uint8_t reserved1[2];
	/* 0x003: Enable in Sleep Control */
	volatile uint8_t ENIDL_CTL;
	/* 0x004: Disable in Idle Control */
	volatile uint8_t DISIDL_CTL;
	/* 0x005: Disable in Idle Control 1 */
	volatile uint8_t DISIDL_CTL1;
	volatile uint8_t reserved2;
	/* 0x007: Power-Down Control 0 */
	volatile uint8_t PWDWN_CTL0;
	/* 0x008: Power-Down Control 1 */
	volatile uint8_t PWDWN_CTL1;
	/* 0x009: Power-Down Control 2 */
	volatile uint8_t PWDWN_CTL2;
	/* 0x00A: Power-Down Control 3 */
	volatile uint8_t PWDWN_CTL3;
	/* 0x00B: Power-Down Control 4 */
	volatile uint8_t PWDWN_CTL4;
	/* 0x00C: Power-Down Control 5 */
	volatile uint8_t PWDWN_CTL5;
	/* 0x00D: Power-Down Control 6 */
	volatile uint8_t PWDWN_CTL6;
	volatile uint8_t reserved3[3];
	/* 0x011: RAM Power-Down Control 1 */
	volatile uint8_t RAM_PD1;
	/* 0x012: RAM Power-Down Control 2 */
	volatile uint8_t RAM_PD2;
	/* 0x013: Software Reset 1 */
	volatile uint8_t SW_RST1;
	/* 0x014: RAM Power-Down Control 3  */
	volatile uint8_t RAM_PD3;
	/* 0x015: Power-Down Control 7 */
	volatile uint8_t PWDWN_CTL7;
	/* 0x016: Power-Down Control 8 */
	volatile uint8_t PWDWN_CTL8;
};

/* PMC register fields */
#define NPCM_PMCSR_DI_INSTW      0
#define NPCM_PMCSR_DHF           1
#define NPCM_PMCSR_IDLE          2
#define NPCM_PMCSR_NWBI          3
#define NPCM_PMCSR_OHFC          6
#define NPCM_PMCSR_OLFC          7
#define NPCM_DISIDL_CTL_RAM_DID  5
#define NPCM_ENIDL_CTL_LP_WK_CTL 6
#define NPCM_ENIDL_CTL_PECI_ENI  2

#define GET_POS_FIELD(pos, size)  pos
#define GET_SIZE_FIELD(pos, size) size
#define FIELD_POS(field)          GET_POS_##field
#define FIELD_SIZE(field)         GET_SIZE_##field

#define GET_FIELD_SZ(field)   _GET_FIELD_SZ_(FIELD_SIZE(field))
#define _GET_FIELD_SZ_(f_ops) f_ops

#define GET_FIELD(reg, field)           _GET_FIELD_(reg, FIELD_POS(field), FIELD_SIZE(field))
#define _GET_FIELD_(reg, f_pos, f_size) (((reg) >> (f_pos)) & ((1 << (f_size)) - 1))

#define SET_FIELD(reg, field, value) _SET_FIELD_(reg, FIELD_POS(field), FIELD_SIZE(field), value)
#define _SET_FIELD_(reg, f_pos, f_size, value)                                                     \
	((reg) = ((reg) & (~(((1 << (f_size)) - 1) << (f_pos)))) | ((value) << (f_pos)))

/* Driver convenience defines */
#define HAL_INSTANCE(dev) ((struct i3c_reg *)((const struct npcm_i3c_config *)(dev)->config)->base)

static void npcm_i3c_mutex_lock(const struct device *dev)
{
	struct npcm_i3c_data *const data = dev->data;

	k_mutex_lock(&data->lock_mutex, K_FOREVER);
}

static void npcm_i3c_mutex_unlock(const struct device *dev)
{
	struct npcm_i3c_data *const data = dev->data;

	k_mutex_unlock(&data->lock_mutex);
}

static void npcm_i3c_reset_module(const struct device *dev)
{
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	struct pmc_reg *pmc = (struct pmc_reg *)DT_REG_ADDR_BY_NAME(DT_NODELABEL(pcc), pmc);
	uint8_t index;

	index = I3C_NPCM_HW_IDX((uint32_t)i3c_inst);

	/* Reset i3c module, write 1 to the bit, then write 0 */
	pmc->SW_RST1 |= BIT(index);
	/* Require one NOP instruction cycle time */
	arch_nop();
	pmc->SW_RST1 &= ~BIT(index);
}

/*
 * brief:  Wait for status bit done and clear the status
 *
 * param[in] i3c_inst  Pointer to I3C register.
 *
 * return  0, success
 *         -ETIMEDOUT: check status timeout.
 */
static inline int npcm_i3c_status_wait_clear(struct i3c_reg *i3c_inst, uint8_t bit_offset)
{
	if (WAIT_FOR(IS_BIT_SET(i3c_inst->MSTATUS, bit_offset), NPCM_I3C_CHK_TIMEOUT_US, NULL) ==
	    false) {
		return -ETIMEDOUT;
	}

	i3c_inst->MSTATUS = BIT(bit_offset); /* W1C */

	return 0;
}

static inline uint32_t npcm_i3c_state_get(struct i3c_reg *i3c_inst)
{
	return GET_FIELD(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_STATE);
}

static inline void npcm_i3c_interrupt_all_disable(struct i3c_reg *i3c_inst)
{
	uint32_t intmask = i3c_inst->MINTSET;

	i3c_inst->MINTCLR = intmask;
}

static inline void npcm_i3c_interrupt_enable(struct i3c_reg *i3c_inst, uint32_t mask)
{
	i3c_inst->MINTSET = mask;
}

static bool npcm_i3c_has_error(struct i3c_reg *i3c_inst)
{
	if (IS_BIT_SET(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_ERRWARN)) {
		LOG_WRN("ERROR: MSTATUS 0x%08x MERRWARN 0x%08x", i3c_inst->MSTATUS,
			i3c_inst->MERRWARN);

		return true;
	}

	return false;
}

static inline void npcm_i3c_status_clear_all(struct i3c_reg *i3c_inst)
{
	uint32_t mask = I3C_STATUS_CLR_MASK;

	i3c_inst->MSTATUS = mask;
}

static inline void npcm_i3c_errwarn_clear_all(struct i3c_reg *i3c_inst)
{
	i3c_inst->MERRWARN = i3c_inst->MERRWARN;
}

static inline void npcm_i3c_fifo_flush(struct i3c_reg *i3c_inst)
{
	i3c_inst->MDATACTRL |= (BIT(NPCM_I3C_MDATACTRL_FLUSHTB) | BIT(NPCM_I3C_MDATACTRL_FLUSHFB));
}

/*
 * brief:  Send request and check the request is valid
 *
 * param[in] i3c_inst  Pointer to I3C register.
 *
 * return  0, success
 *         -ETIMEDOUT check MCTRLDONE timeout.
 *         -ENOSYS    invalid use of request.
 */
static inline int npcm_i3c_send_request(struct i3c_reg *i3c_inst, uint32_t mctrl_val)
{
	i3c_inst->MCTRL = mctrl_val;

	if (npcm_i3c_status_wait_clear(i3c_inst, NPCM_I3C_MSTATUS_MCTRLDONE) != 0) {
		return -ETIMEDOUT;
	}

	/* Check invalid use of request */
	if (IS_BIT_SET(i3c_inst->MERRWARN, NPCM_I3C_MERRWARN_INVREQ)) {
		LOG_ERR("Invalid request, merrwarn: %#x", i3c_inst->MERRWARN);
		return -ENOSYS;
	}

	return 0;
}

/* Start DAA procedure and continue the DAA with a Repeated START */
static inline int npcm_i3c_request_daa(struct i3c_reg *i3c_inst)
{
	uint32_t val = 0;
	int ret;

	/* Set IBI response NACK while processing DAA */
	SET_FIELD(val, NPCM_I3C_MCTRL_IBIRESP, MCTRL_IBIRESP_NACK);

	/* Send DAA request */
	SET_FIELD(val, NPCM_I3C_MCTRL_REQUEST, MCTRL_REQUEST_PROCESSDAA);

	ret = npcm_i3c_send_request(i3c_inst, val);
	if (ret != 0) {
		LOG_ERR("Request DAA error, %d", ret);
		return ret;
	}

	return 0;
}

/* Tell controller to start auto IBI */
static inline int npcm_i3c_request_auto_ibi(struct i3c_reg *i3c_inst)
{
	uint32_t val = 0;
	int ret;

	SET_FIELD(val, NPCM_I3C_MCTRL_IBIRESP, MCTRL_IBIRESP_ACK);
	SET_FIELD(val, NPCM_I3C_MCTRL_REQUEST, MCTRL_REQUEST_AUTOIBI);

	ret = npcm_i3c_send_request(i3c_inst, val);
	if (ret != 0) {
		LOG_ERR("Request auto ibi error, %d", ret);
		return ret;
	}

	return 0;
}

/*
 * brief:  Controller emit start and send address
 *
 * param[in] i3c_inst     Pointer to I3C register.
 * param[in] addr     Dyamic address for xfer or 0x7E for CCC command.
 * param[in] op_type  Request type.
 * param[in] is_read  Read(true) or write(false) operation.
 * param[in] read_sz  Read size.
 *
 * return  0, success
 *         else, error
 */
static int npcm_i3c_request_emit_start(struct i3c_reg *i3c_inst, uint8_t addr,
				       enum npcm_i3c_mctrl_type op_type, bool is_read,
				       size_t read_sz)
{
	uint32_t mctrl = 0;
	int ret;

	/* Set request and target address*/
	SET_FIELD(mctrl, NPCM_I3C_MCTRL_REQUEST, MCTRL_REQUEST_EMITSTARTADDR);

	/* Set operation type */
	SET_FIELD(mctrl, NPCM_I3C_MCTRL_TYPE, op_type);

	/* Set IBI response NACK in emit start */
	SET_FIELD(mctrl, NPCM_I3C_MCTRL_IBIRESP, MCTRL_IBIRESP_NACK);

	/* Set dynamic address */
	SET_FIELD(mctrl, NPCM_I3C_MCTRL_ADDR, addr);

	/* Set read(1) or write(0) */
	if (is_read) {
		mctrl |= BIT(NPCM_I3C_MCTRL_DIR);
		SET_FIELD(mctrl, NPCM_I3C_MCTRL_RDTERM, read_sz); /* Set read length */
	} else {
		mctrl &= ~BIT(NPCM_I3C_MCTRL_DIR);
	}

	ret = npcm_i3c_send_request(i3c_inst, mctrl);
	if (ret != 0) {
		LOG_ERR("Request start error, %d", ret);
		return ret;
	}

	/* Check NACK after MCTRLDONE is get */
	if (IS_BIT_SET(i3c_inst->MERRWARN, NPCM_I3C_MERRWARN_NACK)) {
		LOG_DBG("NACK");
		return -ENODEV;
	}

	return 0;
}

/*
 * brief:  Controller emit STOP.
 *
 * This emits STOP when controller is in NORMACT state.
 *
 * param[in] i3c_inst  Pointer to I3C register.
 *
 * return  0 success
 *         -ECANCELED i3c state not as expected.
 *         -ETIMEDOUT check MCTRLDONE timeout.
 *         -ENOSYS    invalid use of request.
 */
static inline int npcm_i3c_request_emit_stop(struct i3c_reg *i3c_inst)
{
	uint32_t val = 0;
	int ret;
	uint32_t i3c_state = npcm_i3c_state_get(i3c_inst);

	/* Make sure we are in a state where we can emit STOP */
	if (i3c_state == MSTATUS_STATE_IDLE || i3c_state == MSTATUS_STATE_TGTREQ) {
		LOG_ERR("Request stop state error, state= %#x", i3c_state);
		return -ECANCELED;
	}

	SET_FIELD(val, NPCM_I3C_MCTRL_REQUEST, MCTRL_REQUEST_EMITSTOP);

	ret = npcm_i3c_send_request(i3c_inst, val);
	if (ret != 0) {
		LOG_ERR("Request stop error, %d", ret);
		return ret;
	}

	return 0;
}

static inline int npcm_i3c_ibi_respond_nack(struct i3c_reg *i3c_inst)
{
	uint32_t val = 0;
	int ret;

	SET_FIELD(val, NPCM_I3C_MCTRL_IBIRESP, MCTRL_IBIRESP_NACK);
	SET_FIELD(val, NPCM_I3C_MCTRL_REQUEST, MCTRL_REQUEST_IBIACKNACK);

	ret = npcm_i3c_send_request(i3c_inst, val);
	if (ret != 0) {
		LOG_ERR("Request ibi_rsp nack error, %d", ret);
		return ret;
	}

	return 0;
}

static inline int npcm_i3c_ibi_respond_ack(struct i3c_reg *i3c_inst)
{
	uint32_t val = 0;
	int ret;

	SET_FIELD(val, NPCM_I3C_MCTRL_IBIRESP, MCTRL_IBIRESP_ACK);
	SET_FIELD(val, NPCM_I3C_MCTRL_REQUEST, MCTRL_REQUEST_IBIACKNACK);

	ret = npcm_i3c_send_request(i3c_inst, val);
	if (ret != 0) {
		LOG_ERR("Request ibi_rsp ack error %d", ret);
		return ret;
	}

	return 0;
}

/*
 * brief:  Find a registered I3C target device.
 *
 * This returns the I3C device descriptor of the I3C device
 * matching the incoming id.
 *
 * param[in] dev  Pointer to controller device driver instance.
 * param[in] id   Pointer to I3C device ID.
 *
 * return  see i3c_device_find.
 */
static inline struct i3c_device_desc *npcm_i3c_device_find(const struct device *dev,
							   const struct i3c_device_id *id)
{
	const struct npcm_i3c_config *config = dev->config;

	return i3c_dev_list_find(&config->common.dev_list, id);
}

/*
 * brief:  Perform bus recovery.
 *
 * param[in] dev  Pointer to controller device driver instance.
 *
 * return 0 success, otherwise error
 */
static int npcm_i3c_recover_bus(const struct device *dev)
{
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);

	/*
	 * If the controller is in NORMACT state, tells it to emit STOP
	 * so it can return to IDLE, or is ready to clear any pending
	 * target initiated IBIs.
	 */
	if (npcm_i3c_state_get(i3c_inst) == MSTATUS_STATE_NORMACT) {
		npcm_i3c_request_emit_stop(i3c_inst);
	};

	/* Exhaust all target initiated IBI */
	while (IS_BIT_SET(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_TGTSTART)) {
		/* Tell the controller to perform auto IBI. */
		npcm_i3c_request_auto_ibi(i3c_inst);

		if (WAIT_FOR(IS_BIT_SET(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_COMPLETE),
			     NPCM_I3C_CHK_TIMEOUT_US, NULL) == false) {
			break;
		}

		/* Once auto IBI is done, discard bytes in FIFO. */
		while (IS_BIT_SET(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_RXPEND)) {
			/* Flush FIFO as long as RXPEND is set. */
			npcm_i3c_fifo_flush(i3c_inst);
		}

		/* Emit stop */
		npcm_i3c_request_emit_stop(i3c_inst);

		/*
		 * There might be other IBIs waiting.
		 * So pause a bit to let other targets initiates
		 * their IBIs.
		 */
		k_busy_wait(100);
	}

	/* Check IDLE state */
	if (WAIT_FOR((npcm_i3c_state_get(i3c_inst) == MSTATUS_STATE_IDLE), NPCM_I3C_CHK_TIMEOUT_US,
		     NULL) == false) {
		return -EBUSY;
	}

	return 0;
}

static inline void npcm_i3c_xfer_reset(struct i3c_reg *i3c_inst)
{
	npcm_i3c_status_clear_all(i3c_inst);
	npcm_i3c_errwarn_clear_all(i3c_inst);
	npcm_i3c_fifo_flush(i3c_inst);
}

/*
 * brief:  Perform one write transaction.
 *
 * This writes all data in buf to TX FIFO or time out
 * waiting for FIFO spaces.
 *
 * param[in] i3c_inst   Pointer to controller registers.
 * param[in] buf        Buffer containing data to be sent.
 * param[in] buf_sz     Number of bytes in buf to send.
 * param[in] no_ending  True if not to signal end of write message.
 *
 * return  Number of bytes written, or negative if error.
 *
 */
static int npcm_i3c_xfer_write_fifo(struct i3c_reg *i3c_inst, uint8_t *buf, uint8_t buf_sz,
				    bool no_ending)
{
	int offset = 0;
	int remaining = buf_sz;

	while (remaining > 0) {
		/* Check tx fifo not full */
		if (WAIT_FOR(!IS_BIT_SET(i3c_inst->MDATACTRL, NPCM_I3C_MDATACTRL_TXFULL),
			     NPCM_I3C_CHK_TIMEOUT_US, NULL) == false) {
			LOG_DBG("Check tx fifo not full timed out");
			return -ETIMEDOUT;
		}

		if ((remaining > 1) || no_ending) {
			i3c_inst->MWDATAB = (uint32_t)buf[offset];
		} else {
			i3c_inst->MWDATABE = (uint32_t)buf[offset]; /* Set last byte */
		}

		offset += 1;
		remaining -= 1;
	}

	return offset;
}

/*
 * brief:  Perform read transaction.
 *
 * This reads from RX FIFO until COMPLETE bit is set in MSTATUS
 * or time out.
 *
 * param[in] i3c_inst  Pointer to controller registers.
 * param[in] buf       Buffer to store data.
 * param[in] buf_sz    Number of bytes to read.
 *
 * return  Number of bytes read, or negative if error.
 *
 */
static int npcm_i3c_xfer_read_fifo(struct i3c_reg *i3c_inst, uint8_t *buf, uint8_t rd_sz)
{
	bool is_done = false;
	int offset = 0;

	while (is_done == false) {
		/* Check message is terminated */
		if (IS_BIT_SET(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_COMPLETE)) {
			is_done = true;
		}

		/* Check I3C bus error */
		if (npcm_i3c_has_error(i3c_inst)) {
			/* Check timeout*/
			if (IS_BIT_SET(i3c_inst->MERRWARN, NPCM_I3C_MERRWARN_TIMEOUT)) {
				LOG_WRN("ERR: timeout");
			}

			i3c_inst->MERRWARN = i3c_inst->MERRWARN;

			return -EIO;
		}

		/* Check rx not empty */
		if (IS_BIT_SET(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_RXPEND)) {

			/* Receive all the data in this round.
			 * Read in a tight loop to reduce chance of losing
			 * FIFO data when the i3c speed is high.
			 */
			while (offset < rd_sz) {
				if (GET_FIELD(i3c_inst->MDATACTRL, NPCM_I3C_MDATACTRL_RXCOUNT) ==
				    0) {
					break;
				}

				buf[offset++] = (uint8_t)i3c_inst->MRDATAB;
			}
		}
	}

	return offset;
}

#ifdef CONFIG_I3C_NPCM_DMA
static int npcm_i3c_pdma_dsct(const struct device *dev, bool is_read,
			      struct pdma_dsct_reg **dsct_inst)
{
	const struct npcm_i3c_config *config = dev->config;

	if (is_read) {
		*dsct_inst = (struct pdma_dsct_reg *)config->pdma_rx;
	} else {
		*dsct_inst = (struct pdma_dsct_reg *)config->pdma_tx;
	}

	return NPCM_PDMA_DSCT_IDX((uint32_t)*dsct_inst);
}

static void npcm_i3c_ctrl_notify(const struct device *dev)
{
	struct npcm_i3c_data *const data = dev->data;

	k_sem_give(&data->sync_sem);
}

static int npcm_i3c_ctrl_wait_completion(const struct device *dev)
{
	struct npcm_i3c_data *const data = dev->data;

	return k_sem_take(&data->sync_sem, I3C_TRANS_TIMEOUT_MS);
}

static int npcm_i3c_pdma_wait_completion(const struct device *dev, bool is_read)
{
	struct pdma_reg *pdma_inst;
	struct pdma_dsct_reg *dsct_inst = NULL;
	uint8_t dsct_idx;

	dsct_idx = npcm_i3c_pdma_dsct(dev, is_read, &dsct_inst);
	if (dsct_inst == NULL) {
		LOG_ERR("dsct(%d) not exist", is_read);
		return -EINVAL;
	}

	pdma_inst = (struct pdma_reg *)NPCM_PDMA_BASE((uint32_t)dsct_inst);
	if (pdma_inst == NULL) {
		LOG_ERR("pdma base address not exist.");
		return -EINVAL;
	}

	/* Check dma transfer done */
	if (WAIT_FOR(IS_BIT_SET(pdma_inst->PDMA_TDSTS, dsct_idx), NPCM_I3C_CHK_TIMEOUT_US, NULL) ==
	    false) {
		LOG_ERR("Check dma transfer done timed out");
		return -ETIMEDOUT;
	}

	return 0;
}

static int npcm_i3c_pdma_remain_count(const struct device *dev, bool is_read)
{
	struct pdma_reg *pdma_inst;
	struct pdma_dsct_reg *dsct_inst = NULL;
	uint8_t dsct_idx;

	dsct_idx = npcm_i3c_pdma_dsct(dev, is_read, &dsct_inst);
	if (dsct_inst == NULL) {
		LOG_ERR("dsct(%d) not exist", is_read);
		return -EINVAL;
	}

	pdma_inst = (struct pdma_reg *)NPCM_PDMA_BASE((uint32_t)dsct_inst);
	if (pdma_inst == NULL) {
		LOG_ERR("pdma base address not exist.");
		return -EINVAL;
	}

	if (!IS_BIT_SET(pdma_inst->PDMA_TDSTS, dsct_idx)) {
		return GET_FIELD(dsct_inst->CTL, NPCM_PDMA_DSCT_CTL_TXCNT) + 1;
	} else {
		return 0;
	}
}

static int npcm_i3c_pdma_stop(const struct device *dev, bool is_read)
{
	struct pdma_dsct_reg *dsct_inst = NULL;
	struct pdma_reg *pdma_inst;
	uint8_t dsct_idx;
	uint32_t key;

	dsct_idx = npcm_i3c_pdma_dsct(dev, is_read, &dsct_inst);
	if (dsct_inst == NULL) {
		LOG_ERR("dsct(%d) not exist", is_read);
		return -EINVAL;
	}

	/* Get pdma base address */
	pdma_inst = (struct pdma_reg *)NPCM_PDMA_BASE((uint32_t)dsct_inst);
	if (pdma_inst == NULL) {
		LOG_ERR("pdma base address not exist.");
		return -EINVAL;
	}

	key = irq_lock();

	/* Clear transfer done flag */
	if (pdma_inst->PDMA_TDSTS & BIT(dsct_idx)) {
		pdma_inst->PDMA_TDSTS |= BIT(dsct_idx);
	}

	pdma_inst->PDMA_CHCTL &= ~BIT(dsct_idx);

	irq_unlock(key);

	return 0;
}

static int npcm_i3c_pdma_start(const struct device *dev, bool is_read)
{
	struct pdma_dsct_reg *dsct_inst = NULL;
	struct pdma_reg *pdma_inst;
	uint8_t dsct_idx;
	uint32_t key;

	dsct_idx = npcm_i3c_pdma_dsct(dev, is_read, &dsct_inst);
	if (dsct_inst == NULL) {
		LOG_ERR("dsct(%d) not exist", is_read);
		return -EINVAL;
	}

	/* Get pdma base address */
	pdma_inst = (struct pdma_reg *)NPCM_PDMA_BASE((uint32_t)dsct_inst);
	if (pdma_inst == NULL) {
		LOG_ERR("pdma base address not exist.");
		return -EINVAL;
	}

	key = irq_lock();

	/* Clear transfer done flag */
	if (pdma_inst->PDMA_TDSTS & BIT(dsct_idx)) {
		pdma_inst->PDMA_TDSTS |= BIT(dsct_idx);
	}

	pdma_inst->PDMA_CHCTL |= BIT(dsct_idx);

	irq_unlock(key);

	return 0;
}

/*
 * brief:  Configure DMA transaction.
 *
 * For DMA read, use one descriptor table to received data from target/controller.
 * For DMA write, use two descriptor tables to transfer data to target/controller.
 *
 * param[in] dev           Pointer to controller device driver instance.
 * param[in] type          i3c config type
 * param[in] rnw           read or write request.
 * param[in] buf           Buffer to store data.
 * param[in] buf_sz        Number of bytes to read.
 * param[in] no_ending     True if not to signal end of message.
 *
 * return  Successful return 0, or negative if error.
 *
 */
static int npcm_i3c_pdma_configure(const struct device *dev, enum i3c_config_type type,
				   bool is_read, uint8_t *buf, uint8_t buf_sz, bool no_ending)
{
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	struct npcm_i3c_data *data = dev->data;
	struct pdma_reg *pdma_inst;
	struct pdma_dsct_reg *dsct_inst = NULL;
	uint8_t dsct_idx;
	uint8_t i3c_mux_id;
	uint32_t src_addr;
	uint32_t dst_addr;
	uint32_t ctrl;
	uint32_t key;

	ARG_UNUSED(type);

	/* No data to be transferred */
	if ((buf == NULL) || (buf_sz == 0)) {
		return 0;
	}

	dsct_idx = npcm_i3c_pdma_dsct(dev, is_read, &dsct_inst);
	if (dsct_inst == NULL) {
		LOG_ERR("dsct(%d) not exist", is_read);
		return -EINVAL;
	}

	/* Get pdma base address */
	pdma_inst = (struct pdma_reg *)NPCM_PDMA_BASE((uint32_t)dsct_inst);
	if (pdma_inst == NULL) {
		LOG_ERR("pdma base address not exist.");
		return -EINVAL;
	}

	i3c_mux_id = I3C_NPCM_PDMA_MUX_ID((uint32_t)i3c_inst, is_read);

	key = irq_lock();

	/* Setup channel request selection */
	SET_FIELD(pdma_inst->PDMA_REQSEL[(dsct_idx / NPCM_PDMA_CHANNEL_PER_REQ)],
		  NPCM_PDMA_REQSEL_CHANNEL(dsct_idx % NPCM_PDMA_CHANNEL_PER_REQ), i3c_mux_id);

	/* PDMA support scatter-gather and basic mode, we use scatter-gather mode
	 * as default mode.
	 */
	ctrl = 0;

	/* Initial top descriptor table */
	dsct_inst->CTL = NPCM_PDMA_DSCT_CTL_OPMODE_SGM;
	dsct_inst->SA = 0x0;
	dsct_inst->DA = 0x0;
	dsct_inst->NEXT = (uint32_t)&data->dsct_sg[0];

	/* Configure scatter-gather table base MSB address. */
	pdma_inst->PDMA_SCATBA = (uint32_t)&data->dsct_sg[0];

	/* set 8 bits transfer width */
	SET_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_TXWIDTH, NPCM_PDMA_DSCT_CTL_TX_WIDTH_8);

	/* Set DMA single request type */
	ctrl |= BIT(NPCM_PDMA_DSCT_CTL_TXTYPE_SINGLE);

	/* Set mode as basic means the last descriptor */
	SET_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_OPMODE, NPCM_PDMA_DSCT_CTL_OPMODE_BASIC);

	/* For read DMA, fixed src address.
	 * For write DMA, fixed dst address
	 */
	if (is_read) {
		/* Set transfer size, TXCNT + 1 */
		SET_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_TXCNT, (buf_sz - 1));
		SET_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_SAINC, NPCM_PDMA_DSCT_CTL_SAINC_FIX);
		SET_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_DAINC, 0x0);

		src_addr = (uint32_t)&i3c_inst->MRDATAB;
		dst_addr = (uint32_t)&buf[0];
	} else {
		SET_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_DAINC, NPCM_PDMA_DSCT_CTL_DAINC_FIX);
		SET_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_SAINC, 0x0);

		src_addr = (uint32_t)&buf[0];

		/* Set transfer size, TXCNT + 1 */
		SET_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_TXCNT, (buf_sz - 1));

		if (no_ending) {
			dst_addr = (uint32_t)&i3c_inst->MWDATAB1;
		} else if (buf_sz > 1) {
			/* In this case need use second descriptor table, re-configure
			 * first descriptor SGM and (tx_length - 2), the last byte use
			 * second decriptor table.
			 */
			SET_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_OPMODE, NPCM_PDMA_DSCT_CTL_OPMODE_SGM);
			SET_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_TXCNT, (buf_sz - 2));

			dst_addr = (uint32_t)&i3c_inst->MWDATAB1;
		} else {
			dst_addr = (uint32_t)&i3c_inst->MWDATABE;
		}
	}

	memset(&data->dsct_sg, 0x0, sizeof(data->dsct_sg));

	/* Set next descriptor */
	dsct_inst->NEXT = (uint32_t)&data->dsct_sg[0];

	data->dsct_sg[0].CTL = ctrl;
	data->dsct_sg[0].SA = src_addr;
	data->dsct_sg[0].DA = dst_addr;
	data->dsct_sg[0].NEXT = 0x0;

	if (!is_read) {
		/* If first descriptor use scatter-gather mode */
		if (GET_FIELD(data->dsct_sg[0].CTL, NPCM_PDMA_DSCT_CTL_OPMODE) ==
		    NPCM_PDMA_DSCT_CTL_OPMODE_SGM) {
			/* Configure next descriptor. */
			data->dsct_sg[0].NEXT = (uint32_t)&data->dsct_sg[1];

			/* Set basic mode for last descriptor */
			SET_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_OPMODE, NPCM_PDMA_DSCT_CTL_OPMODE_BASIC);
			SET_FIELD(ctrl, NPCM_PDMA_DSCT_CTL_TXCNT, 0x0);

			data->dsct_sg[1].CTL = ctrl;
			data->dsct_sg[1].SA = (uint32_t)&buf[buf_sz - 1];
			data->dsct_sg[1].DA = (uint32_t)&i3c_inst->MWDATABE;
			data->dsct_sg[1].NEXT = 0x0;
		}
	}

	irq_unlock(key);

	return 0;
}

static int npcm_i3c_ctlr_xfer_read_fifo_dma(const struct device *dev, uint8_t addr,
					    enum npcm_i3c_mctrl_type op_type, uint8_t *buf,
					    size_t buf_sz, bool is_read, bool emit_start,
					    bool emit_stop, bool no_ending)
{
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	int ret;

	ret = npcm_i3c_pdma_configure(dev, I3C_CONFIG_CONTROLLER, true, buf, buf_sz, no_ending);
	if (ret) {
		return ret;
	}

	/* Enable PDMA before emit start */
	ret = npcm_i3c_pdma_start(dev, true);
	if (ret) {
		goto out_read_fifo_dma;
	}

	/* Enable DMA until DMA is disabled by setting DMAFB to 00 */
	SET_FIELD(i3c_inst->MDMACTRL, NPCM_I3C_MDMACTRL_DMAFB, MDMA_DMAFB_EN_MANUAL);

	/* Emit START if needed */
	if (emit_start) {
		ret = npcm_i3c_request_emit_start(i3c_inst, addr, op_type, is_read, buf_sz);
		if (ret != 0) {
			goto out_read_fifo_dma;
		}
	}

	/* No data to be transferred */
	if ((buf == NULL) || (buf_sz == 0)) {
		goto out_read_fifo_dma;
	}

	if (no_ending) {
		ret = npcm_i3c_pdma_wait_completion(dev, true);
		if (ret) {
			LOG_ERR("i3c wait dma completion timeout");
		}
	} else {
		/* Enable COMPLETE interrupt */
		i3c_inst->MINTSET |= BIT(NPCM_I3C_MINTSET_COMPLETE);

		ret = npcm_i3c_ctrl_wait_completion(dev);
		if (ret) {
			i3c_inst->MINTCLR = BIT(NPCM_I3C_MINTCLR_COMPLETE);
			LOG_ERR("i3c wait completion timeout");
		}
	}

out_read_fifo_dma:
	/* Disable DMA */
	if (GET_FIELD(i3c_inst->MDMACTRL, NPCM_I3C_MDMACTRL_DMAFB) != MDMA_DMAFB_DISABLE) {
		SET_FIELD(i3c_inst->MDMACTRL, NPCM_I3C_MDMACTRL_DMAFB, MDMA_DMAFB_DISABLE);
	}

	if (npcm_i3c_pdma_stop(dev, true)) {
		ret = -EIO;
	}

	if (ret == 0 && (buf && buf_sz)) {
		ret = npcm_i3c_pdma_remain_count(dev, true);
		if (ret >= 0) {
			ret = buf_sz - ret;
		}
	}

	/* Emit STOP if needed */
	if (emit_stop) {
		npcm_i3c_request_emit_stop(i3c_inst);
	}

	return ret;
}

static int npcm_i3c_ctlr_xfer_write_fifo_dma(const struct device *dev, uint8_t addr,
					     enum npcm_i3c_mctrl_type op_type, uint8_t *buf,
					     size_t buf_sz, bool is_read, bool emit_start,
					     bool emit_stop, bool no_ending)
{
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	uint32_t key;
	int ret;

	ret = npcm_i3c_pdma_configure(dev, I3C_CONFIG_CONTROLLER, false, buf, buf_sz, no_ending);
	if (ret) {
		return ret;
	}

	/* For write operation, we enable dma after emit start. Disable all interrupts
	 * to avoid i3c stall timeout.
	 */
	key = irq_lock();

	/* Emit START if needed */
	if (emit_start) {
		ret = npcm_i3c_request_emit_start(i3c_inst, addr, op_type, is_read, buf_sz);
		if (ret != 0) {
			irq_unlock(key);
			return ret;
		}
	}

	/* Enable PDMA after emit start */
	ret = npcm_i3c_pdma_start(dev, false);
	if (ret) {
		irq_unlock(key);
		goto out_write_fifo_dma;
	}

	/* Enable DMA until DMA is disabled by setting DMATB to 00 */
	SET_FIELD(i3c_inst->MDMACTRL, NPCM_I3C_MDMACTRL_DMATB, MDMA_DMATB_EN_MANUAL);

	/* Enable interrupts */
	irq_unlock(key);

	/* No data to be transferred */
	if ((buf == NULL) || (buf_sz == 0)) {
		goto out_write_fifo_dma;
	}

	if (no_ending) {
		ret = npcm_i3c_pdma_wait_completion(dev, false);
		if (ret) {
			LOG_ERR("i3c wait dma completion timeout");
		}
	} else {
		/* Enable COMPLETE interrupt */
		i3c_inst->MINTSET |= BIT(NPCM_I3C_MINTSET_COMPLETE);

		ret = npcm_i3c_ctrl_wait_completion(dev);
		if (ret) {
			i3c_inst->MINTCLR = BIT(NPCM_I3C_MINTCLR_COMPLETE);
			LOG_ERR("i3c wait completion timeout");
		}
	}

out_write_fifo_dma:
	/* Disable DMA */
	if (GET_FIELD(i3c_inst->MDMACTRL, NPCM_I3C_MDMACTRL_DMATB) != MDMA_DMATB_DISABLE) {
		SET_FIELD(i3c_inst->MDMACTRL, NPCM_I3C_MDMACTRL_DMATB, MDMA_DMATB_DISABLE);
	}

	if (npcm_i3c_pdma_stop(dev, false)) {
		ret = -EIO;
	}

	if (ret == 0 && (buf && buf_sz)) {
		ret = npcm_i3c_pdma_remain_count(dev, false);
		if (ret >= 0) {
			ret = buf_sz - ret;
		}
	}

	/* Emit STOP if needed */
	if (emit_stop) {
		npcm_i3c_request_emit_stop(i3c_inst);
	}

	return ret;
}

/*
 * brief:  Perform one transfer transaction by DMA.
 *
 * param[in] dev         Pointer to device driver instance.
 * param[in] addr        Target address.
 * param[in] op_type     Request type.
 * param[in] buf         Buffer for data to be sent or received.
 * param[in] buf_sz      Buffer size in bytes.
 * param[in] is_read     True if this is a read transaction, false if write.
 * param[in] emit_start  True if START is needed before read/write.
 * param[in] emit_stop   True if STOP is needed after read/write.
 * param[in] no_ending   True if not to signal end of message.
 *
 * return  Number of bytes read/written, or negative if error.
 */
static int npcm_i3c_do_one_xfer_dma(const struct device *dev, uint8_t addr,
				    enum npcm_i3c_mctrl_type op_type, uint8_t *buf, size_t buf_sz,
				    bool is_read, bool emit_start, bool emit_stop, bool no_ending)
{
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	int ret = 0;

	npcm_i3c_status_clear_all(i3c_inst);
	npcm_i3c_errwarn_clear_all(i3c_inst);

	if (is_read) {
		ret = npcm_i3c_ctlr_xfer_read_fifo_dma(dev, addr, op_type, buf, buf_sz, is_read,
						       emit_start, emit_stop, no_ending);
	} else {
		ret = npcm_i3c_ctlr_xfer_write_fifo_dma(dev, addr, op_type, buf, buf_sz, is_read,
							emit_start, emit_stop, no_ending);
	}

	if (ret < 0) {
		LOG_ERR("%s fifo fail", is_read ? "read" : "write");
		return ret;
	}

	/* Check I3C bus error */
	if (npcm_i3c_has_error(i3c_inst)) {
		LOG_ERR("I3C bus error");
		return -EIO;
	}

	if (no_ending) {
		/* Flush fifo data */
		npcm_i3c_fifo_flush(i3c_inst);
	}

	return ret;
}
#endif /* End of CONFIG_I3C_NPCM_DMA */

/*
 * brief:  Perform one transfer transaction.
 *
 * param[in] dev         Pointer to device driver instance
 * param[in] addr        Target address.
 * param[in] op_type     Request type.
 * param[in] buf         Buffer for data to be sent or received.
 * param[in] buf_sz      Buffer size in bytes.
 * param[in] is_read     True if this is a read transaction, false if write.
 * param[in] emit_start  True if START is needed before read/write.
 * param[in] emit_stop   True if STOP is needed after read/write.
 * param[in] no_ending   True if not to signal end of write message.
 *
 * return  Number of bytes read/written, or negative if error.
 */
static int npcm_i3c_do_one_xfer(const struct device *dev, uint8_t addr,
				enum npcm_i3c_mctrl_type op_type, uint8_t *buf, size_t buf_sz,
				bool is_read, bool emit_start, bool emit_stop, bool no_ending)
{
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	int ret = 0;

	npcm_i3c_status_clear_all(i3c_inst);
	npcm_i3c_errwarn_clear_all(i3c_inst);

	/* Emit START if needed */
	if (emit_start) {
		ret = npcm_i3c_request_emit_start(i3c_inst, addr, op_type, is_read, buf_sz);
		if (ret != 0) {
			goto out_do_one_xfer;
		}
	}

	/* No data to be transferred */
	if ((buf == NULL) || (buf_sz == 0)) {
		goto out_do_one_xfer;
	}

	/* Select read or write operation */
	if (is_read) {
		ret = npcm_i3c_xfer_read_fifo(i3c_inst, buf, buf_sz);
	} else {
		ret = npcm_i3c_xfer_write_fifo(i3c_inst, buf, buf_sz, no_ending);
	}

	if (ret < 0) {
		LOG_ERR("%s fifo fail", is_read ? "read" : "write");
		goto out_do_one_xfer;
	}

	/* Check message complete if is a read transaction or
	 * ending byte of a write transaction.
	 */
	if (is_read || !no_ending) {
		/* Wait message transfer complete */
		if (WAIT_FOR(IS_BIT_SET(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_COMPLETE),
			     NPCM_I3C_CHK_TIMEOUT_US, NULL) == false) {
			LOG_ERR("timed out addr 0x%02x, buf_sz %u", addr, buf_sz);

			ret = -ETIMEDOUT;
			emit_stop = true;

			goto out_do_one_xfer;
		}

		i3c_inst->MSTATUS = BIT(NPCM_I3C_MSTATUS_COMPLETE); /* W1C */
	}

	/* Check I3C bus error */
	if (npcm_i3c_has_error(i3c_inst)) {
		ret = -EIO;
		LOG_ERR("I3C bus error");
	}

out_do_one_xfer:
	/* Emit STOP if needed */
	if (emit_stop) {
		npcm_i3c_request_emit_stop(i3c_inst);
	}

	return ret;
}

/*
 * brief:  Transfer messages in I3C mode.
 *
 * see i3c_transfer
 *
 * param[in] dev       Pointer to device driver instance.
 * param[in] target    Pointer to target device descriptor.
 * param[in] msgs      Pointer to I3C messages.
 * param[in] num_msgs  Number of messages to transfers.
 *
 * return  see i3c_transfer
 */
static int npcm_i3c_transfer(const struct device *dev, struct i3c_device_desc *target,
			     struct i3c_msg *msgs, uint8_t num_msgs)
{
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	uint32_t intmask;
	int xfered_len, ret = 0;
	bool send_broadcast = true;
	bool is_xfer_done = true;

	if (msgs == NULL) {
		return -EINVAL;
	}

	if (target->dynamic_addr == 0U) {
		return -EINVAL;
	}

	npcm_i3c_mutex_lock(dev);

	/* Check bus in idle state */
	if (WAIT_FOR((npcm_i3c_state_get(i3c_inst) == MSTATUS_STATE_IDLE), NPCM_I3C_CHK_TIMEOUT_US,
		     NULL) == false) {
		LOG_ERR("xfer state error: %d", npcm_i3c_state_get(i3c_inst));
		npcm_i3c_mutex_unlock(dev);
		return -ETIMEDOUT;
	}

	/* Disable interrupt */
	intmask = i3c_inst->MINTSET;
	npcm_i3c_interrupt_all_disable(i3c_inst);

	npcm_i3c_xfer_reset(i3c_inst);

	/* Iterate over all the messages */
	for (int i = 0; i < num_msgs; i++) {

		bool is_read = (msgs[i].flags & I3C_MSG_RW_MASK) == I3C_MSG_READ;
		bool no_ending = false;

		/*
		 * Emit start if this is the first message or that
		 * the RESTART flag is set in message.
		 */
		bool emit_start =
			(i == 0) || ((msgs[i].flags & I3C_MSG_RESTART) == I3C_MSG_RESTART);

		bool emit_stop = (msgs[i].flags & I3C_MSG_STOP) == I3C_MSG_STOP;

		/*
		 * The controller requires special treatment of last byte of
		 * a write message. Since the API permits having a bunch of
		 * write messages without RESTART in between, this is just some
		 * logic to determine whether to treat the last byte of this
		 * message to be the last byte of a series of write mssages.
		 * If not, tell the write function not to treat it that way.
		 */
		if (!is_read && !emit_stop && ((i + 1) != num_msgs)) {
			bool next_is_write = (msgs[i + 1].flags & I3C_MSG_RW_MASK) == I3C_MSG_WRITE;
			bool next_is_restart =
				((msgs[i + 1].flags & I3C_MSG_RESTART) == I3C_MSG_RESTART);

			/* Check next msg is still write operation and not including Sr */
			if (next_is_write && !next_is_restart) {
				no_ending = true;
			}
		}

		/*
		 * Two ways to do read/write transfer .
		 * 1. [S] + [0x7E]    + [address] + [data] + [Sr or P]
		 * 2. [S] + [address] + [data]    + [Sr or P]
		 *
		 * Send broadcast header(0x7E) on first transfer or after a STOP,
		 * unless flag is set not to.
		 */
		if (!(msgs[i].flags & I3C_MSG_NBCH) && (send_broadcast)) {
			ret = npcm_i3c_request_emit_start(i3c_inst, I3C_BROADCAST_ADDR,
							  NPCM_I3C_MCTRL_TYPE_I3C, false, 0);
			if (ret < 0) {
				LOG_ERR("emit start of broadcast addr failed, error (%d)", ret);
				break;
			}
			send_broadcast = false;
		}

#ifdef CONFIG_I3C_NPCM_DMA
		/* Do transfer with target device */
		xfered_len = npcm_i3c_do_one_xfer_dma(
			dev, target->dynamic_addr, NPCM_I3C_MCTRL_TYPE_I3C, msgs[i].buf,
			msgs[i].len, is_read, emit_start, emit_stop, no_ending);
#else
		xfered_len = npcm_i3c_do_one_xfer(dev, target->dynamic_addr,
						  NPCM_I3C_MCTRL_TYPE_I3C, msgs[i].buf, msgs[i].len,
						  is_read, emit_start, emit_stop, no_ending);
#endif
		if (xfered_len < 0) {
			LOG_ERR("do xfer fail");
			ret = xfered_len; /* Set error code to ret */
			break;
		}

		/* Write back the total number of bytes transferred */
		msgs[i].num_xfer = xfered_len;

		if (emit_stop) {
			/* After a STOP, send broadcast header before next msg */
			send_broadcast = true;
		}

		/* Check emit stop flag including in the final msg */
		if ((i == num_msgs - 1) && (emit_stop == false)) {
			is_xfer_done = false;
		}
	}

	/* Emit stop if error occurs or stop flag not in the msg */
	if ((ret != 0) || (is_xfer_done == false)) {
		npcm_i3c_request_emit_stop(i3c_inst);
	}

	npcm_i3c_errwarn_clear_all(i3c_inst);
	npcm_i3c_status_clear_all(i3c_inst);

	npcm_i3c_interrupt_enable(i3c_inst, intmask);

	npcm_i3c_mutex_unlock(dev);

	return ret;
}

/*
 * brief:  Perform Dynamic Address Assignment.
 *
 * param[in] dev  Pointer to controller device driver instance.
 *
 * return  0 If successful.
 *         -EBUSY Bus is busy.
 *         -EIO General input / output error.
 *         -ENODEV If a provisioned ID does not match to any target devices
 *                 in the registered device list.
 *         -ENOSPC No more free addresses can be assigned to target.
 *         -ENOSYS Dynamic address assignment is not supported by
 *                 the controller driver.
 */
static int npcm_i3c_do_daa(const struct device *dev)
{
	const struct npcm_i3c_config *config = dev->config;
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	struct npcm_i3c_data *data = dev->data;
	int ret = 0;
	uint8_t rx_buf[8];
	size_t rx_count;
	uint32_t intmask;

	npcm_i3c_mutex_lock(dev);

	memset(rx_buf, 0xff, sizeof(rx_buf));

	/* Check bus in idle state */
	if (WAIT_FOR((npcm_i3c_state_get(i3c_inst) == MSTATUS_STATE_IDLE), NPCM_I3C_CHK_TIMEOUT_US,
		     NULL) == false) {
		LOG_ERR("DAA state error: %d", npcm_i3c_state_get(i3c_inst));
		npcm_i3c_mutex_unlock(dev);
		return -ETIMEDOUT;
	}

	LOG_DBG("DAA: ENTDAA");

	/* Disable interrupt */
	intmask = i3c_inst->MINTSET;
	npcm_i3c_interrupt_all_disable(i3c_inst);

	npcm_i3c_xfer_reset(i3c_inst);

	/* Emit process DAA */
	if (npcm_i3c_request_daa(i3c_inst) != 0) {
		ret = -ETIMEDOUT;
		LOG_ERR("Emit process DAA error");
		goto out_do_daa;
	}

	/* Loop until no more responses from devices */
	do {
		/* Check ERRWARN bit set */
		if (npcm_i3c_has_error(i3c_inst)) {
			ret = -EIO;
			LOG_ERR("DAA recv error");
			break;
		}

		/* Receive Provisioned ID, BCR and DCR (total 8 bytes) */
		rx_count = GET_FIELD(i3c_inst->MDATACTRL, NPCM_I3C_MDATACTRL_RXCOUNT);

		if (rx_count == DAA_TGT_INFO_SZ) {
			for (int i = 0; i < rx_count; i++) {
				rx_buf[i] = (uint8_t)i3c_inst->MRDATAB;
			}
		} else {
			/* Data count not as expected, exit DAA */
			ret = -EBADMSG;
			LOG_DBG("Rx count not as expected %d, abort DAA", rx_count);
			break;
		}

		/* Start assign dynamic address */
		if ((npcm_i3c_state_get(i3c_inst) == MSTATUS_STATE_DAA) &&
		    IS_BIT_SET(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_BETWEEN)) {
			struct i3c_device_desc *target;
			uint16_t vendor_id;
			uint32_t part_no;
			uint64_t pid;
			uint8_t dyn_addr = 0;

			/* PID[47:33] = manufacturer ID */
			vendor_id = (((uint16_t)rx_buf[0] << 8U) | (uint16_t)rx_buf[1]) & 0xFFFEU;

			/* PID[31:0] = vendor fixed falue or random value */
			part_no = (uint32_t)rx_buf[2] << 24U | (uint32_t)rx_buf[3] << 16U |
				  (uint32_t)rx_buf[4] << 8U | (uint32_t)rx_buf[5];

			/* Combine into one Provisioned ID */
			pid = (uint64_t)vendor_id << 32U | (uint64_t)part_no;

			LOG_DBG("DAA: Rcvd PID 0x%04x%08x", vendor_id, part_no);

			/* Find a usable address during ENTDAA */
			ret = i3c_dev_list_daa_addr_helper(&data->common.attached_dev.addr_slots,
							   &config->common.dev_list, pid, false,
							   true, &target, &dyn_addr);
			if (ret != 0) {
				LOG_ERR("Assign new DA error");
				break;
			}

			if (target == NULL) {
				LOG_INF("%s: PID 0x%04x%08x is not in registered device "
					"list, given dynamic address 0x%02x",
					dev->name, vendor_id, part_no, dyn_addr);
			} else {
				/* Update target descriptor */
				target->dynamic_addr = dyn_addr;
				target->bcr = rx_buf[6];
				target->dcr = rx_buf[7];
			}

			/* Mark the address as I3C device */
			i3c_addr_slots_mark_i3c(&data->common.attached_dev.addr_slots, dyn_addr);

			/*
			 * If the device has static address, after address assignment,
			 * the device will not respond to the static address anymore.
			 * So free the static one from address slots if different from
			 * newly assigned one.
			 */
			if ((target != NULL) && (target->static_addr != 0U) &&
			    (dyn_addr != target->static_addr)) {
				i3c_addr_slots_mark_free(&data->common.attached_dev.addr_slots,
							 dyn_addr);
			}

			/* Emit process DAA again to send the address to the device */
			i3c_inst->MWDATAB = dyn_addr;
			ret = npcm_i3c_request_daa(i3c_inst);
			if (ret != 0) {
				LOG_ERR("Assign DA timeout");
				break;
			}

			LOG_DBG("PID 0x%04x%08x assigned dynamic address 0x%02x", vendor_id,
				part_no, dyn_addr);

			/* Target did not accept the assigned DA, exit DAA */
			if (IS_BIT_SET(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_NACKED)) {
				ret = -EFAULT;
				LOG_DBG("TGT NACK assigned DA %#x", dyn_addr);

				/* Free the reserved DA */
				i3c_addr_slots_mark_free(&data->common.attached_dev.addr_slots,
							 dyn_addr);

				/* 0 if address has not been assigned */
				if (target != NULL) {
					target->dynamic_addr = 0;
				}

				break;
			}
		}

		/* Check all targets have been assigned DA and DAA complete */
	} while ((!IS_BIT_SET(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_COMPLETE)) &&
		 npcm_i3c_state_get(i3c_inst) != MSTATUS_STATE_IDLE);

out_do_daa:
	/* Exit DAA mode when error occurs */
	if (ret != 0) {
		npcm_i3c_request_emit_stop(i3c_inst);
	}

	/* Clear all flags. */
	npcm_i3c_errwarn_clear_all(i3c_inst);
	npcm_i3c_status_clear_all(i3c_inst);

	/* Re-Enable I3C IRQ sources. */
	npcm_i3c_interrupt_enable(i3c_inst, intmask);

	npcm_i3c_fifo_flush(i3c_inst);
	npcm_i3c_mutex_unlock(dev);

	return ret;
}

/*
 * brief:  Send Common Command Code (CCC).
 *
 * param[in] dev      Pointer to controller device driver instance.
 * param[in] payload  Pointer to CCC payload.
 *
 * return:  The same as i3c_do_ccc()
 *          0 If successful.
 *          -EBUSY Bus is busy.
 *          -EIO General Input / output error.
 *          -EINVAL Invalid valid set in the payload structure.
 *          -ENOSYS Not implemented.
 */
static int npcm_i3c_do_ccc(const struct device *dev, struct i3c_ccc_payload *payload)
{
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	uint32_t intmask;
	int xfered_len;
	int ret;

	if (dev == NULL || payload == NULL) {
		return -EINVAL;
	}

	npcm_i3c_mutex_lock(dev);

	/* Disable interrupt */
	intmask = i3c_inst->MINTSET;
	npcm_i3c_interrupt_all_disable(i3c_inst);

	/* Clear status and flush fifo */
	npcm_i3c_xfer_reset(i3c_inst);

	LOG_DBG("CCC[0x%02x]", payload->ccc.id);

	/* Write emit START and broadcast address (0x7E) */
	ret = npcm_i3c_request_emit_start(i3c_inst, I3C_BROADCAST_ADDR, NPCM_I3C_MCTRL_TYPE_I3C,
					  false, 0);
	if (ret < 0) {
		LOG_ERR("CCC[0x%02x] %s START error (%d)", payload->ccc.id,
			i3c_ccc_is_payload_broadcast(payload) ? "broadcast" : "direct", ret);

		goto out_do_ccc;
	}

	/* Write CCC command */
	npcm_i3c_status_clear_all(i3c_inst);
	npcm_i3c_errwarn_clear_all(i3c_inst);
	xfered_len =
		npcm_i3c_xfer_write_fifo(i3c_inst, &payload->ccc.id, 1, payload->ccc.data_len > 0);
	if (xfered_len < 0) {
		LOG_ERR("CCC[0x%02x] %s command error (%d)", payload->ccc.id,
			i3c_ccc_is_payload_broadcast(payload) ? "broadcast" : "direct", ret);
		ret = xfered_len;

		goto out_do_ccc;
	}

	/* Write data (defining byte or data bytes) for CCC if needed */
	if (payload->ccc.data_len > 0) {
		npcm_i3c_status_clear_all(i3c_inst);
		npcm_i3c_errwarn_clear_all(i3c_inst);
		xfered_len = npcm_i3c_xfer_write_fifo(i3c_inst, payload->ccc.data,
						      payload->ccc.data_len, false);
		if (xfered_len < 0) {
			LOG_ERR("CCC[0x%02x] %s command payload error (%d)", payload->ccc.id,
				i3c_ccc_is_payload_broadcast(payload) ? "broadcast" : "direct",
				ret);
			ret = xfered_len;

			goto out_do_ccc;
		}

		/* Write back the transferred bytes */
		payload->ccc.num_xfer = xfered_len;
	}

	/* Wait message transfer complete */
	if (WAIT_FOR(IS_BIT_SET(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_COMPLETE),
		     NPCM_I3C_CHK_TIMEOUT_US, NULL) == false) {
		ret = -ETIMEDOUT;
		LOG_DBG("Check complete timeout");
		goto out_do_ccc;
	}

	i3c_inst->MSTATUS = BIT(NPCM_I3C_MSTATUS_COMPLETE); /* W1C */

	/* For direct CCC */
	if (!i3c_ccc_is_payload_broadcast(payload)) {
		/*
		 * If there are payload(s) for each target,
		 * RESTART and then send payload for each target.
		 */
		for (int idx = 0; idx < payload->targets.num_targets; idx++) {
			struct i3c_ccc_target_payload *tgt_payload =
				&payload->targets.payloads[idx];

			bool is_read = (tgt_payload->rnw == 1U);

			xfered_len = npcm_i3c_do_one_xfer(
				dev, tgt_payload->addr, NPCM_I3C_MCTRL_TYPE_I3C, tgt_payload->data,
				tgt_payload->data_len, is_read, true, false, false);
			if (xfered_len < 0) {
				LOG_ERR("CCC[0x%02x] target payload error (%d)", payload->ccc.id,
					ret);
				ret = xfered_len;

				goto out_do_ccc;
			}

			/* Write back the total number of bytes transferred */
			tgt_payload->num_xfer = xfered_len;
		}
	}

out_do_ccc:
	npcm_i3c_request_emit_stop(i3c_inst);

	npcm_i3c_interrupt_enable(i3c_inst, intmask);

	npcm_i3c_mutex_unlock(dev);

	return ret;
}

#ifdef CONFIG_I3C_USE_IBI
/*
 * brief  Callback to service target initiated IBIs in workqueue.
 *
 * param[in] work  Pointer to k_work item.
 */
static void npcm_i3c_ibi_work(struct k_work *work)
{
	uint8_t payload[CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE];
	size_t payload_sz = 0;

	struct i3c_ibi_work *i3c_ibi_work = CONTAINER_OF(work, struct i3c_ibi_work, work);
	const struct device *dev = i3c_ibi_work->controller;
	struct npcm_i3c_data *data = dev->data;
	struct i3c_dev_attached_list *dev_list = &data->common.attached_dev;
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	struct i3c_device_desc *target = NULL;
	uint32_t ibitype, ibiaddr;
	int ret;

	k_sem_take(&data->ibi_lock_sem, K_FOREVER);

	if (npcm_i3c_state_get(i3c_inst) != MSTATUS_STATE_TGTREQ) {
		LOG_DBG("IBI work %p running not because of IBI", work);
		LOG_ERR("MSTATUS 0x%08x MERRWARN 0x%08x", i3c_inst->MSTATUS, i3c_inst->MERRWARN);

		npcm_i3c_request_emit_stop(i3c_inst);

		goto out_ibi_work;
	};

	/* Use auto IBI to service the IBI */
	npcm_i3c_request_auto_ibi(i3c_inst);

	/* Wait for target to win address arbitration (ibitype and ibiaddr) */
	if (WAIT_FOR(IS_BIT_SET(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_IBIWON),
		     NPCM_I3C_CHK_TIMEOUT_US, NULL) == false) {
		LOG_ERR("IBI work, IBIWON timeout");

		goto out_ibi_work;
	}

	ibitype = GET_FIELD(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_IBITYPE);
	ibiaddr = GET_FIELD(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_IBIADDR);

	switch (ibitype) {
	case MSTATUS_IBITYPE_IBI:
		target = i3c_dev_list_i3c_addr_find(dev_list, (uint8_t)ibiaddr);
		if (target != NULL) {
			ret = npcm_i3c_xfer_read_fifo(i3c_inst, &payload[0], sizeof(payload));
			if (ret >= 0) {
				payload_sz = (size_t)ret;
			} else {
				LOG_ERR("Error reading IBI payload");

				npcm_i3c_request_emit_stop(i3c_inst);

				goto out_ibi_work;
			}
		} else {
			/* NACK IBI coming from unknown device */
			npcm_i3c_ibi_respond_nack(i3c_inst);
		}
		break;
	case MSTATUS_IBITYPE_HJ:
		npcm_i3c_ibi_respond_ack(i3c_inst);
		npcm_i3c_request_emit_stop(i3c_inst);
		break;
	case MSTATUS_IBITYPE_CR:
		LOG_DBG("Controller role handoff not supported");
		npcm_i3c_ibi_respond_nack(i3c_inst);
		npcm_i3c_request_emit_stop(i3c_inst);
		break;
	default:
		break;
	}

	if (npcm_i3c_has_error(i3c_inst)) {
		/*
		 * If the controller detects any errors, simply
		 * emit a STOP to abort the IBI. The target will
		 * raise IBI again if so desired.
		 */
		npcm_i3c_request_emit_stop(i3c_inst);

		goto out_ibi_work;
	}

	switch (ibitype) {
	case MSTATUS_IBITYPE_IBI:
		if (target != NULL) {
			if (i3c_ibi_work_enqueue_target_irq(target, &payload[0], payload_sz) != 0) {
				LOG_ERR("Error enqueue IBI IRQ work");
			}
		}

		/* Finishing the IBI transaction */
		npcm_i3c_request_emit_stop(i3c_inst);
		break;
	case MSTATUS_IBITYPE_HJ:
		if (i3c_ibi_work_enqueue_hotjoin(dev) != 0) {
			LOG_ERR("Error enqueue IBI HJ work");
		}
		break;
	case MSTATUS_IBITYPE_CR:
		/* Not supported, for future use. */
		break;
	default:
		break;
	}

out_ibi_work:
	npcm_i3c_xfer_reset(i3c_inst);

	k_sem_give(&data->ibi_lock_sem);

	/* Re-enable target initiated IBI interrupt. */
	i3c_inst->MINTSET = BIT(NPCM_I3C_MINTSET_TGTSTART);
}

/* Set local IBI information to IBIRULES register */
static void npcm_i3c_ibi_rules_setup(struct npcm_i3c_data *data, struct i3c_reg *i3c_inst)
{
	uint32_t ibi_rules;
	int idx;

	ibi_rules = 0;

	for (idx = 0; idx < ARRAY_SIZE(data->ibi.addr); idx++) {
		uint32_t addr_6bit;

		/* Extract the lower 6-bit of target address */
		addr_6bit = (uint32_t)data->ibi.addr[idx] & IBIRULES_ADDR_MSK;

		/* Shift into correct place */
		addr_6bit <<= idx * IBIRULES_ADDR_SHIFT;

		/* Put into the temporary IBI Rules register */
		ibi_rules |= addr_6bit;
	}

	/* Enable i3c address arbitration optimization strategy. */
	if (!data->ibi.msb) {
		/* The MSB0 field is 1 if MSB is 0 */
		ibi_rules |= BIT(NPCM_I3C_IBIRULES_MSB0);
	} else {
		ibi_rules &= ~BIT(NPCM_I3C_IBIRULES_MSB0);
	}

	if (!data->ibi.has_mandatory_byte) {
		/* The NOBYTE field is 1 if there is no mandatory byte */
		ibi_rules |= BIT(NPCM_I3C_IBIRULES_NOBYTE);
	}

	/* Update the register */
	i3c_inst->IBIRULES = ibi_rules;

	LOG_DBG("MIBIRULES 0x%08x", ibi_rules);
}

static int npcm_i3c_ibi_enable(const struct device *dev, struct i3c_device_desc *target)
{
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	struct npcm_i3c_data *data = dev->data;
	struct i3c_ccc_events i3c_events;
	uint8_t idx;
	bool msb, has_mandatory_byte;
	int ret;

	/* Check target IBI request capable */
	if (!i3c_device_is_ibi_capable(target)) {
		LOG_ERR("device is not ibi capable");
		return -EINVAL;
	}

	if (data->ibi.num_addr >= ARRAY_SIZE(data->ibi.addr)) {
		/* No more free entries in the IBI Rules table */
		LOG_ERR("no more free space in the IBI rules table");
		return -ENOMEM;
	}

	/* Check whether the selected target is already in the list */
	for (idx = 0; idx < ARRAY_SIZE(data->ibi.addr); idx++) {
		if (data->ibi.addr[idx] == target->dynamic_addr) {
			LOG_ERR("selected target is already in the list");
			return -EINVAL;
		}
	}

	/* Disable controller interrupt while we configure IBI rules. */
	i3c_inst->MINTCLR = BIT(NPCM_I3C_MINTCLR_TGTSTART);

	LOG_DBG("IBI enabling for 0x%02x (BCR 0x%02x)", target->dynamic_addr, target->bcr);

	msb = (target->dynamic_addr & BIT(6)) == BIT(6); /* Check addess(7-bit) MSB enable */
	has_mandatory_byte = i3c_ibi_has_payload(target);

	/*
	 * If there are already addresses in the table, we must
	 * check if the incoming entry is compatible with
	 * the existing ones.
	 *
	 * All targets in the list should follow the same IBI rules.
	 */
	if (data->ibi.num_addr > 0) {
		/*
		 * 1. All devices in the table must all use mandatory
		 *    bytes, or do not.
		 *
		 * 2. Each address in entry only captures the lowest 6-bit.
		 *    The MSB (7th bit) is captured separated in another bit
		 *    in the register. So all addresses must have the same MSB.
		 */
		if ((has_mandatory_byte != data->ibi.has_mandatory_byte) ||
		    (msb != data->ibi.msb)) {
			ret = -EINVAL;
			LOG_ERR("New IBI does not have same mandatory byte or msb"
				" as previous IBI");
			goto out_ibi_enable;
		}

		/* Find an empty address slot */
		for (idx = 0; idx < ARRAY_SIZE(data->ibi.addr); idx++) {
			if (data->ibi.addr[idx] == 0U) {
				break;
			}
		}

		if (idx >= ARRAY_SIZE(data->ibi.addr)) {
			ret = -ENOTSUP;
			LOG_ERR("Cannot support more IBIs");
			goto out_ibi_enable;
		}
	} else {
		/*
		 * If the incoming address is the first in the table,
		 * it dictates future compatibilities.
		 */
		data->ibi.has_mandatory_byte = has_mandatory_byte;
		data->ibi.msb = msb;

		idx = 0;
	}

	data->ibi.addr[idx] = target->dynamic_addr;
	data->ibi.num_addr += 1U;

	npcm_i3c_ibi_rules_setup(data, i3c_inst);

	/* Enable target IBI event by ENEC command */
	i3c_events.events = I3C_CCC_EVT_INTR;
	ret = i3c_ccc_do_events_set(target, true, &i3c_events);
	if (ret != 0) {
		LOG_ERR("Error sending IBI ENEC for 0x%02x (%d)", target->dynamic_addr, ret);
	}

out_ibi_enable:
	if (data->ibi.num_addr > 0U) {
		/*
		 * If there is more than 1 target in the list,
		 * enable controller to raise interrupt when a target
		 * initiates IBI.
		 */
		i3c_inst->MINTSET = BIT(NPCM_I3C_MINTSET_TGTSTART);
	}

	return ret;
}

static int npcm_i3c_ibi_disable(const struct device *dev, struct i3c_device_desc *target)
{
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	struct npcm_i3c_data *data = dev->data;
	struct i3c_ccc_events i3c_events;
	int ret;
	int idx;

	if (!i3c_device_is_ibi_capable(target)) {
		LOG_ERR("device is not ibi capable");
		return -EINVAL;
	}

	for (idx = 0; idx < ARRAY_SIZE(data->ibi.addr); idx++) {
		if (target->dynamic_addr == data->ibi.addr[idx]) {
			break;
		}
	}

	if (idx == ARRAY_SIZE(data->ibi.addr)) {
		LOG_ERR("target is not in list of registered addresses");
		return -ENODEV;
	}

	/* Disable controller interrupt while we configure IBI rules. */
	i3c_inst->MINTCLR = BIT(NPCM_I3C_MINTCLR_TGTSTART);

	/* Clear the ibi rule data */
	data->ibi.addr[idx] = 0U;
	data->ibi.num_addr -= 1U;

	/* Disable disable target IBI */
	i3c_events.events = I3C_CCC_EVT_INTR;
	ret = i3c_ccc_do_events_set(target, false, &i3c_events);
	if (ret != 0) {
		LOG_ERR("Error sending IBI DISEC for 0x%02x (%d)", target->dynamic_addr, ret);
	}

	npcm_i3c_ibi_rules_setup(data, i3c_inst);

	if (data->ibi.num_addr > 0U) {
		/*
		 * Enable controller to raise interrupt when a target
		 * initiates IBI.
		 */
		i3c_inst->MINTSET = BIT(NPCM_I3C_MINTSET_TGTSTART);
	}

	return ret;
}
#endif /* CONFIG_I3C_USE_IBI */

static void npcm_i3c_isr(const struct device *dev)
{
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	uint8_t ctlr_mode;

	ctlr_mode = GET_FIELD(i3c_inst->MCONFIG, NPCM_I3C_MCONFIG_CTRENA);

	if (ctlr_mode == MCONFIG_CTRENA_ON) {
#ifdef CONFIG_I3C_NPCM_DMA
		if (IS_BIT_SET(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_COMPLETE)) {
			/* Clear COMPLETE status */
			i3c_inst->MSTATUS = BIT(NPCM_I3C_MSTATUS_COMPLETE); /* W1C */

			/* Disable COMPLETE interrupt */
			i3c_inst->MINTCLR = BIT(NPCM_I3C_MINTCLR_COMPLETE);

			npcm_i3c_ctrl_notify(dev);
			return;
		}
#endif /* CONFIG_I3C_NPCM_DMA */

#ifdef CONFIG_I3C_USE_IBI
		int ret;

		/* Target start detected */
		if (IS_BIT_SET(i3c_inst->MSTATUS, NPCM_I3C_MSTATUS_TGTSTART)) {
			/* Disable further target initiated IBI interrupt */
			i3c_inst->MINTCLR = BIT(NPCM_I3C_MINTCLR_TGTSTART);

			/* Clear TGTSTART interrupt */
			i3c_inst->MSTATUS = BIT(NPCM_I3C_MSTATUS_TGTSTART);

			/* Handle IBI in workqueue */
			ret = i3c_ibi_work_enqueue_cb(dev, npcm_i3c_ibi_work);
			if (ret < 0) {
				LOG_ERR("Enqueuing ibi work fail, ret %d", ret);
				i3c_inst->MINTSET = BIT(NPCM_I3C_MINTSET_TGTSTART);
			}
		}
#endif /* CONFIG_I3C_USE_IBI */
	}
}

static int npcm_i3c_get_scl_config(struct npcm_i3c_timing_cfg *cfg, uint32_t i3c_src_clk,
				   uint32_t pp_baudrate_hz, uint32_t od_baudrate_hz,
				   uint32_t i2c_baudrate_hz)
{
	uint32_t div;
	uint32_t i3c_ppbaud, i3c_odbaud;
	uint32_t i2c_baud_ns, i2c_baud;
	uint32_t i3c_odlow_ns;
	uint32_t i3c_pphigh_ns, i3c_pplow_ns;
	uint32_t src_clk_ns;

	if (cfg == NULL) {
		LOG_ERR("Freq config NULL");
		return -EINVAL;
	}

	if ((pp_baudrate_hz == 0) || (pp_baudrate_hz > I3C_SCL_PP_FREQ_MAX_MHZ) ||
	    (od_baudrate_hz == 0) || (od_baudrate_hz > I3C_SCL_OD_FREQ_MAX_MHZ)) {
		LOG_ERR("I3C PP_SCL should within 12.5 Mhz, input: %d", pp_baudrate_hz);
		LOG_ERR("I3C OD_SCL should within 4.17 Mhz, input: %d", od_baudrate_hz);
		return -EINVAL;
	}

	/* PPBAUD(pp-high) = number of i3c source clock period in one I3C_SCL high
	 * period for I3C push-pull operation.
	 * For example, 48Mhz = 20.8 ns, 96Mhz = 10.4 ns
	 */

	/* Source clock period */
	src_clk_ns = (uint32_t)(NSEC_PER_SEC / i3c_src_clk);

	/* Fixed PPLOW(pp-low) = 0, 50% duty cycle for push-pull */
	i3c_pphigh_ns = (uint32_t)(NSEC_PER_SEC / pp_baudrate_hz) / 2UL;

	div = i3c_pphigh_ns / src_clk_ns;
	div = (div == 0UL) ? 1UL : div;
	if (i3c_pphigh_ns % src_clk_ns != 0) {
		div++;
	}

	if (div > PPBAUD_DIV_MAX) {
		LOG_ERR("PPBAUD(%d) out of range", div);
		return -EINVAL;
	}

	/* 0x0 = one source clock period for pp-high
	 * 0x1 = two source clock period for pp-high
	 * 0x2 = three source clock period for pp-high
	 * ...
	 */
	i3c_ppbaud = div - 1UL;

	/* Record calculation result , 50% duty-cycle */
	i3c_pphigh_ns = src_clk_ns * div;
	i3c_pplow_ns = i3c_pphigh_ns;

	/* Check PP low period in spec (should be the same as PPHIGH) */
	if (i3c_pplow_ns < I3C_BUS_TLOW_PP_MIN_NS) {
		LOG_ERR("PPLOW(%d) ns out of spec", i3c_pplow_ns);
		return -EINVAL;
	}

	/* odbaud = Number of PPBAUD periods (minus 1) in one I3C_SCL low period
	 * for I3C open-drain operation.
	 */

	/* Fixed ODHPP(od-high) = 1, calculate odlow_ns value */
	i3c_odlow_ns = (uint32_t)(NSEC_PER_SEC / od_baudrate_hz) - i3c_pphigh_ns;

	/* pphigh_ns = PPBAUD periods */
	div = i3c_odlow_ns / i3c_pphigh_ns;
	div = (div == 0UL) ? 1UL : div;
	if (i3c_odlow_ns % i3c_pphigh_ns != 0) {
		div++;
	}

	/* 0x0 = one PPBAUD period
	 * 0x1 = two PPBAUD period
	 * 0x2 = three PPBAUD period
	 * ...
	 */
	i3c_odbaud = div - 1UL;

	/* Record calculation result, odhpp = pphpp */
	i3c_odlow_ns = i3c_pphigh_ns * div;

	/* Check OD low period in spec */
	if (i3c_odlow_ns < I3C_BUS_TLOW_OD_MIN_NS) {
		LOG_ERR("ODBAUD(%d) ns out of spec", i3c_odlow_ns);
		return -EINVAL;
	}

	if (i2c_baudrate_hz != 0) {
		/* Calculate i2c baudrate periods */
		i2c_baud_ns = (uint32_t)(NSEC_PER_SEC / i2c_baudrate_hz);

		/* 50% duty-cycle */
		div = i2c_baud_ns / i3c_odlow_ns;
		if (i2c_baud_ns % i3c_odlow_ns != 0) {
			div++;
		}

		/* I2CBAUD = scl-high + scl-low
		 * (I2CBAUD >> 1) + 1 ==> scl-high
		 * (I2CBAUD >> 1) + 1 + lsb bit ==> scl-low
		 */
		i2c_baud = div - (1 << 1);

		if (div > PPBAUD_DIV_MAX) {
			LOG_ERR("I2C out of range");
			return -EINVAL;
		}
	} else {
		i2c_baud = 0;
	}

	cfg->pplow = 0;
	cfg->odhpp = 1;
	cfg->ppbaud = i3c_ppbaud;
	cfg->odbaud = i3c_odbaud;
	cfg->i2c_baud = i2c_baud;

	return 0;
}

static int npcm_i3c_freq_init(const struct device *dev)
{
	const struct npcm_i3c_config *config = dev->config;
	struct npcm_i3c_data *data = dev->data;
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(pcc));
	struct i3c_config_controller *ctrl_config = &data->common.ctrl_config;

	uint32_t scl_pp = ctrl_config->scl.i3c;
	uint32_t scl_od = config->clocks.i3c_od_scl_hz;
	uint32_t scl_i2c = ctrl_config->scl.i2c;
	struct npcm_i3c_timing_cfg timing_cfg;
	uint32_t i3c_freq_rate;
	int ret;

	ret = clock_control_get_rate(clk_dev, (clock_control_subsys_t)&config->clk_cfg,
				     &i3c_freq_rate);
	if (ret != 0x0) {
		LOG_ERR("Get I3C source clock fail %d", ret);
		return -EINVAL;
	}

	LOG_DBG("SCL_PP_FREQ MAX: %d", I3C_SCL_PP_FREQ_MAX_MHZ);
	LOG_DBG("SCL_OD_FREQ MAX: %d", I3C_SCL_OD_FREQ_MAX_MHZ);
	LOG_DBG("i3c_clk_freq: %d", i3c_freq_rate);
	LOG_DBG("scl_pp: %d", scl_pp);
	LOG_DBG("scl_od: %d", scl_od);
	LOG_DBG("scl_i2c: %d", scl_i2c);
	LOG_DBG("hdr: %d", ctrl_config->supported_hdr);

	if (i3c_freq_rate == I3C_CLK_FREQ_48_MHZ) {
		timing_cfg = npcm_def_speed_cfg[NPCM_I3C_CLK_FREQ_48MHZ];
	} else if (i3c_freq_rate == I3C_CLK_FREQ_96_MHZ) {
		timing_cfg = npcm_def_speed_cfg[NPCM_I3C_CLK_FREQ_96MHZ];
	} else {
		LOG_ERR("Unsupported i3c freq for %s. freq rate: %d", dev->name, i3c_freq_rate);
		return -EINVAL;
	}

	ret = npcm_i3c_get_scl_config(&timing_cfg, i3c_freq_rate, scl_pp, scl_od, scl_i2c);
	if (ret != 0x0) {
		LOG_ERR("Adjust I3C frequency fail");
		return -EINVAL;
	}

	/* Apply SCL_PP and SCL_OD */
	SET_FIELD(i3c_inst->MCONFIG, NPCM_I3C_MCONFIG_PPBAUD, timing_cfg.ppbaud);
	SET_FIELD(i3c_inst->MCONFIG, NPCM_I3C_MCONFIG_PPLOW, timing_cfg.pplow);
	SET_FIELD(i3c_inst->MCONFIG, NPCM_I3C_MCONFIG_ODBAUD, timing_cfg.odbaud);
	SET_FIELD(i3c_inst->MCONFIG, NPCM_I3C_MCONFIG_I2CBAUD, timing_cfg.i2c_baud);
	if (timing_cfg.odhpp != 0) {
		i3c_inst->MCONFIG |= BIT(NPCM_I3C_MCONFIG_ODHPP);
	} else {
		i3c_inst->MCONFIG &= ~BIT(NPCM_I3C_MCONFIG_ODHPP);
	}

	LOG_DBG("ppbaud: %d", GET_FIELD(i3c_inst->MCONFIG, NPCM_I3C_MCONFIG_PPBAUD));
	LOG_DBG("odbaud: %d", GET_FIELD(i3c_inst->MCONFIG, NPCM_I3C_MCONFIG_ODBAUD));
	LOG_DBG("pplow: %d", GET_FIELD(i3c_inst->MCONFIG, NPCM_I3C_MCONFIG_PPLOW));
	LOG_DBG("odhpp: %d", IS_BIT_SET(i3c_inst->MCONFIG, NPCM_I3C_MCONFIG_ODHPP));
	LOG_DBG("i2c_baud: %d", GET_FIELD(i3c_inst->MCONFIG, NPCM_I3C_MCONFIG_I2CBAUD));

	return 0;
}

static int npcm_i3c_cntlr_init(const struct device *dev)
{
	const struct npcm_i3c_config *config = dev->config;
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(pcc));
	uint32_t i3c_freq_rate;
	uint8_t bamatch;
	int ret;

	/* Reset I3C module */
	npcm_i3c_reset_module(dev);

	/* Disable all interrupts */
	npcm_i3c_interrupt_all_disable(i3c_inst);

	/* Initial baudrate. */
	if (npcm_i3c_freq_init(dev) != 0x0) {
		return -EINVAL;
	}

	/* Enable main controller mode */
	SET_FIELD(i3c_inst->MCONFIG, NPCM_I3C_MCONFIG_CTRENA, MCONFIG_CTRENA_ON);
	/* Enable open-drain stop */
	i3c_inst->MCONFIG |= BIT(NPCM_I3C_MCONFIG_ODSTOP);
	/* Enable timeout */
	i3c_inst->MCONFIG &= ~BIT(NPCM_I3C_MCONFIG_DISTO);
	/* Flush tx and tx FIFO buffer */
	npcm_i3c_fifo_flush(i3c_inst);

	/* Set bus available match value in target register */
	ret = clock_control_get_rate(clk_dev, (clock_control_subsys_t)&config->clk_cfg,
				     &i3c_freq_rate);
	LOG_DBG("I3C_CLK_FREQ: %d", i3c_freq_rate);

	if (ret != 0x0) {
		LOG_ERR("Get I3C source clock fail %d", ret);
		return -EINVAL;
	}

	bamatch = DIV_ROUND_UP(i3c_freq_rate, MHZ(1));
	LOG_DBG("BAMATCH: %d", bamatch);

	SET_FIELD(i3c_inst->CONFIG, NPCM_I3C_CONFIG_BAMATCH, bamatch);

	return 0;
}

static int npcm_i3c_configure(const struct device *dev, enum i3c_config_type type, void *config)
{
	struct npcm_i3c_data *dev_data = dev->data;
	struct i3c_config_controller *cntlr_cfg = config;

	if (type == I3C_CONFIG_CONTROLLER) {
		/*
		 * Check for valid configuration parameters.
		 * Currently, must be the primary controller.
		 */
		if ((cntlr_cfg->is_secondary) || (cntlr_cfg->scl.i3c == 0U)) {
			return -EINVAL;
		}

		/* Save requested config to dev */
		(void)memcpy(&dev_data->common.ctrl_config, cntlr_cfg, sizeof(*cntlr_cfg));

		/* Controller init */
		return npcm_i3c_cntlr_init(dev);
	}

	LOG_ERR("Support controller mode only");
	return -EINVAL;
}

static int npcm_i3c_config_get(const struct device *dev, enum i3c_config_type type, void *config)
{
	struct npcm_i3c_data *data = dev->data;

	if ((type != I3C_CONFIG_CONTROLLER) || (config == NULL)) {
		return -EINVAL;
	}

	(void)memcpy(config, &data->common.ctrl_config, sizeof(data->common.ctrl_config));

	return 0;
}

static int npcm_i3c_init(const struct device *dev)
{
	const struct npcm_i3c_config *config = dev->config;
	struct npcm_i3c_data *data = dev->data;
	struct i3c_config_controller *ctrl_config = &data->common.ctrl_config;
	const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(pcc));
	int ret;

	/* Check clock device ready */
	if (!device_is_ready(clk_dev)) {
		LOG_ERR("%s Clk device not ready", clk_dev->name);
		return -ENODEV;
	}

	/* Set I3C_PD operational */
	ret = clock_control_on(clk_dev, (clock_control_subsys_t)&config->clk_cfg);
	if (ret < 0) {
		LOG_ERR("Turn on I3C clock fail %d", ret);
		return ret;
	}

	/* Apply pin-muxing */
	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("Apply pinctrl fail %d", ret);
		return ret;
	}

	k_mutex_init(&data->lock_mutex);
	k_sem_init(&data->sync_sem, 0, 1);
	k_sem_init(&data->ibi_lock_sem, 1, 1);

	ret = i3c_addr_slots_init(dev);
	if (ret != 0) {
		LOG_ERR("Addr slots init fail %d", ret);
		return ret;
	}

	ctrl_config->is_secondary = false; /* Currently can only act as primary controller. */
	ctrl_config->supported_hdr = 0U;   /* HDR mode not supported at the moment. */
	ctrl_config->scl.i3c = config->clocks.i3c_pp_scl_hz; /* Set I3C frequency */
	ctrl_config->scl.i2c = config->clocks.i2c_scl_hz;    /* Set I2C frequency */

	ret = npcm_i3c_configure(dev, I3C_CONFIG_CONTROLLER, ctrl_config);
	if (ret != 0) {
		LOG_ERR("Apply i3c_configure() fail %d", ret);
		return ret;
	}

	/* Just in case the bus is not in idle. */
	ret = npcm_i3c_recover_bus(dev);
	if (ret != 0) {
		LOG_ERR("Apply i3c_recover_bus() fail %d", ret);
		return ret;
	}

	/* Configure interrupt */
	config->irq_config_func(dev);

	/* Check I3C target device exist in device tree */
	if (config->common.dev_list.num_i3c > 0) {
		/* Perform bus initialization */
		ret = i3c_bus_init(dev, &config->common.dev_list);
		if (ret != 0) {
			LOG_ERR("Apply i3c_bus_init() fail %d", ret);
			return ret;
		}
	}

	return 0;
}

static int npcm_i3c_i2c_api_configure(const struct device *dev, uint32_t dev_config)
{
	return -ENOSYS;
}

static int npcm_i3c_i2c_api_transfer(const struct device *dev, struct i2c_msg *msgs,
				     uint8_t num_msgs, uint16_t addr)
{
	struct i3c_reg *i3c_inst = HAL_INSTANCE(dev);
	uint32_t intmask;
	int xfered_len, ret;
	bool is_xfer_done = true;

	npcm_i3c_mutex_lock(dev);

	if (WAIT_FOR((npcm_i3c_state_get(i3c_inst) == MSTATUS_STATE_IDLE), NPCM_I3C_CHK_TIMEOUT_US,
		     NULL) == false) {
		LOG_ERR("xfer state error: %d", npcm_i3c_state_get(i3c_inst));
		npcm_i3c_mutex_unlock(dev);
		return -ETIMEDOUT;
	}

	/* Disable interrupt */
	intmask = i3c_inst->MINTSET;
	npcm_i3c_interrupt_all_disable(i3c_inst);

	npcm_i3c_xfer_reset(i3c_inst);

	/* Iterate over all the messages */
	for (int i = 0; i < num_msgs; i++) {
		bool is_read = (msgs[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ;
		bool no_ending = false;

		/*
		 * Emit start if this is the first message or that
		 * the RESTART flag is set in message.
		 */
		bool emit_start =
			(i == 0) || ((msgs[i].flags & I2C_MSG_RESTART) == I2C_MSG_RESTART);

		bool emit_stop = (msgs[i].flags & I2C_MSG_STOP) == I2C_MSG_STOP;

		/*
		 * The controller requires special treatment of last byte of
		 * a write message. Since the API permits having a bunch of
		 * write messages without RESTART in between, this is just some
		 * logic to determine whether to treat the last byte of this
		 * message to be the last byte of a series of write mssages.
		 * If not, tell the write function not to treat it that way.
		 */
		if (!is_read && !emit_stop && ((i + 1) != num_msgs)) {
			bool next_is_write = (msgs[i + 1].flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE;
			bool next_is_restart =
				((msgs[i + 1].flags & I2C_MSG_RESTART) == I2C_MSG_RESTART);

			if (next_is_write && !next_is_restart) {
				no_ending = true;
			}
		}

#ifdef CONFIG_I3C_NPCM_DMA
		/* Do transfer with target device */
		xfered_len = npcm_i3c_do_one_xfer_dma(dev, addr, NPCM_I3C_MCTRL_TYPE_I2C,
						      msgs[i].buf, msgs[i].len, is_read, emit_start,
						      emit_stop, no_ending);
#else
		xfered_len = npcm_i3c_do_one_xfer(dev, addr, NPCM_I3C_MCTRL_TYPE_I2C, msgs[i].buf,
						  msgs[i].len, is_read, emit_start, emit_stop,
						  no_ending);
#endif
		if (xfered_len < 0) {
			LOG_ERR("do xfer fail");
			ret = xfered_len;
			goto out_xfer_i2c_stop_unlock;
		}

		/* Check emit stop flag including in the final msg */
		if ((i == num_msgs - 1) && (emit_stop == false)) {
			is_xfer_done = false;
		}
	}

	ret = 0;

out_xfer_i2c_stop_unlock:
	/* Emit stop if error occurs or stop flag not in the msg */
	if ((ret != 0) || (is_xfer_done == false)) {
		npcm_i3c_request_emit_stop(i3c_inst);
	}

	npcm_i3c_errwarn_clear_all(i3c_inst);
	npcm_i3c_status_clear_all(i3c_inst);

	npcm_i3c_interrupt_enable(i3c_inst, intmask);
	npcm_i3c_mutex_unlock(dev);

	return ret;
}

static const struct i3c_driver_api npcm_i3c_driver_api = {
	.i2c_api.configure = npcm_i3c_i2c_api_configure,
	.i2c_api.transfer = npcm_i3c_i2c_api_transfer,
	.i2c_api.recover_bus = npcm_i3c_recover_bus,
	.configure = npcm_i3c_configure,
	.config_get = npcm_i3c_config_get,
	.recover_bus = npcm_i3c_recover_bus,
	.do_daa = npcm_i3c_do_daa,
	.do_ccc = npcm_i3c_do_ccc,
	.i3c_device_find = npcm_i3c_device_find,
	.i3c_xfers = npcm_i3c_transfer,
#ifdef CONFIG_I3C_USE_IBI
	.ibi_enable = npcm_i3c_ibi_enable,
	.ibi_disable = npcm_i3c_ibi_disable,
#endif
};

#define I3C_NPCM_DEVICE(inst)                                                                      \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static void npcm_i3c_config_func_##inst(const struct device *dev)                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), npcm_i3c_isr,         \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	};                                                                                         \
                                                                                                   \
	static struct i3c_device_desc npcm_i3c_device_array_##inst[] =                             \
		I3C_DEVICE_ARRAY_DT_INST(inst);                                                    \
                                                                                                   \
	static struct i3c_i2c_device_desc npcm_i3c_i2c_device_array_##inst[] =                     \
		I3C_I2C_DEVICE_ARRAY_DT_INST(inst);                                                \
                                                                                                   \
	static const struct npcm_i3c_config npcm_i3c_config_##inst = {                             \
		.base = (struct i3c_reg *)DT_INST_REG_ADDR(inst),                                  \
		.clk_cfg = DT_INST_PHA(inst, clocks, clk_cfg),                                     \
		.irq_config_func = npcm_i3c_config_func_##inst,                                    \
		.common.dev_list.i3c = npcm_i3c_device_array_##inst,                               \
		.common.dev_list.num_i3c = ARRAY_SIZE(npcm_i3c_device_array_##inst),               \
		.common.dev_list.i2c = npcm_i3c_i2c_device_array_##inst,                           \
		.common.dev_list.num_i2c = ARRAY_SIZE(npcm_i3c_i2c_device_array_##inst),           \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		.clocks.i3c_pp_scl_hz = DT_INST_PROP_OR(inst, i3c_scl_hz, 0),                      \
		.clocks.i3c_od_scl_hz = DT_INST_PROP_OR(inst, i3c_od_scl_hz, 0),                   \
		.clocks.i2c_scl_hz = DT_INST_PROP_OR(inst, i2c_scl_hz, 0),                         \
		IF_ENABLED(CONFIG_I3C_NPCM_DMA, ( \
			.pdma_rx = (struct pdma_dsct_reg *)DT_INST_REG_ADDR_BY_IDX(inst, 1), \
			))                                                \
				    IF_ENABLED(CONFIG_I3C_NPCM_DMA, ( \
			.pdma_tx = (struct pdma_dsct_reg *)DT_INST_REG_ADDR_BY_IDX(inst, 2), \
			)) };      \
                                                                                                   \
	static struct npcm_i3c_data npcm_i3c_data_##inst;                                          \
	DEVICE_DT_INST_DEFINE(inst, npcm_i3c_init, NULL, &npcm_i3c_data_##inst,                    \
			      &npcm_i3c_config_##inst, POST_KERNEL,                                \
			      CONFIG_I3C_CONTROLLER_INIT_PRIORITY, &npcm_i3c_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I3C_NPCM_DEVICE)
