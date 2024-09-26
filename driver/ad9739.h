#ifndef _AD9739_H_
#define _AD9739_H_

// ***************************************************************************************
// ***************************** Include Files *******************************************
// ***************************************************************************************
#include "platform_drivers.h"

#define AD9739_READ (1 << 7)
#define AD9739_WRITE (0 << 7)

/* Registers */
#define AD9739_REG_MODE 0x00
#define AD9739_REG_POWER_DOWN 0x01
#define AD9739_REG_CNT_CLK_DIS 0x02
#define AD9739_REG_IRQ_EN 0x03
#define AD9739_REG_IRQ_REQ 0x04
#define AD9739_REG_FSC_1 0x06
#define AD9739_REG_FSC_2 0x07
#define AD9739_REG_DEC_CNT 0x08
#define AD9739_REG_LVDS_STAT1 0x0C
#define AD9739_REG_LVDS_REC_CNT1 0x10
#define AD9739_REG_LVDS_REC_CNT2 0x11
#define AD9739_REG_LVDS_REC_CNT3 0x12
#define AD9739_REG_LVDS_REC_CNT4 0x13
#define AD9739_REG_LVDS_REC_CNT5 0x14
#define AD9739_REG_LVDS_REC_STAT1 0x19
#define AD9739_REG_LVDS_REC_STAT2 0x1A
#define AD9739_REG_LVDS_REC_STAT3 0x1B
#define AD9739_REG_LVDS_REC_STAT4 0x1C
#define AD9739_REG_LVDS_REC_STAT9 0x21
#define AD9739_REG_CROSS_CNT1 0x22
#define AD9739_REG_CROSS_CNT2 0x23
#define AD9739_REG_PHS_DET 0x24
#define AD9739_REG_MU_DUTY 0x25
#define AD9739_REG_MU_CNT1 0x26
#define AD9739_REG_MU_CNT2 0x27
#define AD9739_REG_MU_CNT3 0x28
#define AD9739_REG_MU_CNT4 0x29
#define AD9739_REG_MU_STAT1 0x2A
#define AD9739_REG_PART_ID 0x35
#define AD9739_CHIP_ID 0x20

/* AD9739_REG_MODE definitions, address 0x00 */
#define AD9739_MODE_SDIO_DIR ((1 << 7) | (1 << 0))
#define AD9739_MODE_LSB ((1 << 6) | (1 << 1))
#define AD9739_MODE_RESET ((1 << 5) | (1 << 2))

/* AD9739_REG_POWER_DOWN definitions, address 0x01 */
#define AD9739_POWER_DOWN_LVDS_DRVR_PD (1 << 5)
#define AD9739_POWER_DOWN_LVDS_RCVR_PD (1 << 4)
#define AD9739_POWER_DOWN_CLK_RCVR_PD (1 << 1)
#define AD9739_POWER_DOWN_DAC_BIAS_PD (1 << 0)

/* AD9739_REG_CNT_CLK_DIS definitions, address 0x02 */
#define AD9739_CNT_CLK_DIS_CLKGEN_PD (1 << 3)
#define AD9739_CNT_CLK_DIS_REC_CNT_CLK (1 << 1)
#define AD9739_CNT_CLK_DIS_MU_CNT_CLK (1 << 0)

/* AD9739_REG_IRQ_EN definitions, address 0x03 */
#define AD9739_IRQ_EN_MU_LST_EN (1 << 3)
#define AD9739_IRQ_EN_MU_LCK_EN (1 << 2)
#define AD9739_IRQ_EN_RCV_LST_EN (1 << 1)
#define AD9739_IRQ_EN_RCV_LCK_EN (1 << 0)

/* AD9739_REG_IRQ_REQ definitions, address 0x04 */
#define AD9739_IRQ_REQ_MU_LST_IRQ (1 << 3)
#define AD9739_IRQ_REQ_MU_LCK_IRQ (1 << 2)
#define AD9739_IRQ_REQ_RCV_LST_IRQ (1 << 1)
#define AD9739_IRQ_REQ_RCV_LCK_IRQ (1 << 0)

/* AD9739_REG_FSC_1 definitions, address 0x06 */
#define AD9739_FSC_1_FSC_1(x) (((x)&0xFF) << 0)

/* AD9739_REG_FSC_2 definitions, address 0x07 */
#define AD9739_FSC_2_FSC_2(x) (((x)&0x3) << 0)
#define AD9739_FSC_2_Sleep (1 << 7)

/* AD9739_REG_DEC_CNT definitions, address 0x08 */
#define AD9739_DEC_CNT_DAC_DEC(x) (((x)&0x3) << 0)

/* AD9739_DEC_CNT_DAC_DEC(x) option. */
#define AD9739_DAC_DEC_NORMAL_BASEBAND 0

/* AD9739_DEC_CNT_DAC_DEC(x) option. */
#define AD9739_DAC_DEC_MIX_MODE 2

/* AD9739_REG_LVDS_STAT1 definitions, address 0x0C */
#define AD9739_LVDS_STAT1_DCI_PRE_PH0 (1 << 2)
#define AD9739_LVDS_STAT1_DCI_PST_PH0 (1 << 0)

/* AD9739_REG_LVDS_REC_CNT1 definitions, address 0x10 */
#define AD9739_LVDS_REC_CNT1_RCVR_FLG_RST (1 << 2)
#define AD9739_LVDS_REC_CNT1_RCVR_LOOP_ON (1 << 1)
#define AD9739_LVDS_REC_CNT1_RCVR_CNT_ENA (1 << 0)

/* AD9739_REG_LVDS_REC_CNT2 definitions, address 0x11 */
#define AD9739_LVDS_REC_CNT2_SMP_DEL(x) (((x)&0x3) << 6)

/* AD9739_REG_LVDS_REC_CNT3 definitions, address 0x12 */
#define AD9739_LVDS_REC_CNT3_SMP_DEL(x) (((x)&0xFF) << 0)

/* AD9739_REG_LVDS_REC_CNT4 definitions, address 0x13 */
#define AD9739_LVDS_REC_CNT4_DCI_DEL(x) (((x)&0xF) << 4)
#define AD9739_LVDS_REC_CNT4_FINE_DEL_SKEW(x) (((x)&0xF) << 0)

/* AD9739_REG_LVDS_REC_CNT5 definitions, address 0x14 */
#define AD9739_LVDS_REC_CNT5_DCI_DEL(x) (((x)&0x3F) << 0)

/* AD9739_REG_LVDS_REC_STAT1 definitions, address 0x19 */
#define AD9739_LVDS_REC_STAT1_SMP_DEL(x) (((x)&0x3) << 6)

/* AD9739_REG_LVDS_REC_STAT2 definitions, address 0x1A */
#define AD9739_LVDS_REC_STAT2_SMP_DEL(x) (((x)&0xFF) << 0)

/* AD9739_REG_LVDS_REC_STAT3 definitions, address 0x1B */
#define AD9739_LVDS_REC_STAT3_DCI_DEL(x) (((x)&0x3) << 6)

/* AD9739_REG_LVDS_REC_STAT4 definitions, address 0x1C */
#define AD9739_LVDS_REC_STAT4_DCI_DEL(x) (((x)&0xFF) << 0)

/* AD9739_REG_LVDS_REC_STAT9 definitions, address 0x21 */
#define AD9739_LVDS_REC_STAT9_RCVR_TRK_ON (1 << 3)
#define AD9739_LVDS_REC_STAT9_RCVR_FE_ON (1 << 2)
#define AD9739_LVDS_REC_STAT9_RCVR_LST (1 << 1)
#define AD9739_LVDS_REC_STAT9_RCVR_LCK (1 << 0)

/* AD9739_REG_CROSS_CNT1 definitions, address 0x22 */
#define AD9739_CROSS_CNT1_DIR_P (1 << 4)
#define AD9739_CROSS_CNT1_CLKP_OFFSET(x) (((x)&0xF) << 0)

/* AD9739_REG_CROSS_CNT2 definitions, address 0x23 */
#define AD9739_CROSS_CNT2_DIR_N (1 << 4)
#define AD9739_CROSS_CNT2_CLKN_OFFSET(x) (((x)&0xF) << 0)

/* AD9739_REG_PHS_DET definitions, address 0x24 */
#define AD9739_PHS_DET_CMP_BST (1 << 5)
#define AD9739_PHS_DET_PHS_DET_AUTO_EN (1 << 4)

/* AD9739_REG_MU_DUTY definitions, address 0x25 */
#define AD9739_MU_DUTY_MU_DUTY_AUTO_EN (1 << 7)

/* AD9739_REG_MU_CNT1 definitions, address 0x26 */
#define AD9739_MU_CNT1_SLOPE (1 << 6)
#define AD9739_MU_CNT1_MODE(x) (((x)&0x3) << 4)
#define AD9739_MU_CNT1_READ (1 << 3)
#define AD9739_MU_CNT1_GAIN(x) (((x)&0x3) << 1)
#define AD9739_MU_CNT1_ENABLE (1 << 0)

/* AD9739_REG_MU_CNT2 definitions, address 0x27 */
#define AD9739_MU_CNT2_MUDEL (1 << 7)
#define AD9739_MU_CNT2_SRCH_MODE(x) ((((x)&0x3) << 5))
#define AD9739_MU_CNT2_SET_PHS(x) ((((x)&0x1F) << 0))

/* AD9739_REG_MU_CNT3 definitions, address 0x28 */
#define AD9739_MU_CNT3_MUDEL(x) ((((x)&0xFF) << 0))

/* AD9739_REG_MU_CNT4 definitions, address 0x29 */
#define AD9739_MU_CNT4_SEARCH_TOL (1 << 7)
#define AD9739_MU_CNT4_RETRY (1 << 6)
#define AD9739_MU_CNT4_CONTRST (1 << 5)
#define AD9739_MU_CNT4_GUARD(x) ((((x)&0x1F) << 0))

/* AD9739_REG_MU_STAT1 definitions, address 0x2A */
#define AD9739_MU_STAT1_MU_LST (1 << 1)
#define AD9739_MU_STAT1_MU_LKD (1 << 0)

/* AD9739_REG_PART_ID definitions, address 0x35 */
#define AD9739_PART_ID_PART_ID(x) ((((x)&0xFF) << 0))

typedef struct ad9739_config_t
{
	const char *name;
	spi_mode_t spi_mode;
	uint8_t chip_select;
	uint32_t spi_deviceID;
	uint32_t gpio_id;
	uint8_t gpio_reset;
	uint8_t gpio_irq;
	float full_scale_current;
	uint8_t common_mode_voltage_dacclk_p;
	uint8_t common_mode_voltage_dacclk_n;
} ad9739_config_t;

typedef struct ad9739Device_t
{
	spi_desc_t *spi_desc;
	gpio_desc_t *gpio_reset;
	gpio_desc_t *gpio_irq;
	ad9739_config_t init;
} ad9739Device_t;

/*! Writes a value to the selected register. */
extern int32_t ad9739_write(ad9739Device_t *device, uint8_t addr, uint8_t data);

/*! Reads the value of the selected register. */
extern int32_t ad9739_read(ad9739Device_t *device, uint8_t addr, uint8_t *data);

/*! Resets the device. */
extern int32_t ad9739_reset(ad9739Device_t *device);

/*! Sets the full-scale output current for the DAC.  */
extern float ad9739_dac_fs_current(ad9739Device_t *device, float fs_val);

/*! Sets the normal baseband mode or mix-mode. */
extern int32_t ad9739_operation_mode(ad9739Device_t *device, uint8_t mode);

/*! Powers down LVDS interface and TxDAC. */
extern int32_t ad9739_power_down(ad9739Device_t *device, uint8_t pwr_config);

/*! Delay for a number of fdata clock cycles. */
extern int32_t delay_fdata_cycles(uint32_t cycles);

/*! Initializes the AD9739. */
extern int32_t ad9739_init(ad9739Device_t *device);

extern int32_t ad9739_config(ad9739Device_t *device);

/*! Free the resources allocated by ad9739_init(). */
extern void ad9739_remove(ad9739Device_t *device);

#endif
