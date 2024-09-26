// ***************************************************************************************
// ***************************** Include Files *******************************************
// ***************************************************************************************
#include "ad9739.h"

#define FDATA 2400 // for 2.4 GSPS

// ***************************************************************************************
// * @brief Writes a value to the selected register.
// *
// * @param dev   - The device structure.
// * @param addr  - The address of the register to write to.
// * @param data  - The value to write to the register.
// *
// * @return Returns 0 in case of success or negative error code.
// ***************************************************************************************
int32_t ad9739_write(ad9739Device_t *device, uint8_t addr, uint8_t data)
{
	int32_t Status;
	uint8_t buf[2];

	buf[0] = AD9739_WRITE | (0x7F & addr);
	buf[1] = data;
	Status = spi_write_and_read(device->spi_desc, buf, NULL, (uint8_t)sizeof(buf));

	//	buf[0] = AD9739_READ | (0x7F & addr);
	//	buf[1] = 0x00;
	//	Status |= spi_write_and_read(device->spi_desc, buf, buf, (uint8_t)sizeof(buf));

	if (Status != XST_SUCCESS)
		return XST_FAILURE;
	else
		return XST_SUCCESS;
}

// ***************************************************************************************
// * @brief Reads the value of the selected register.
// *
// * @param dev   - The device structure.
// * @param addr  - The address of the register to read.
// * @param data  - The value read from the register.
// *
// * @return registerValue - The register's value or negative error code.
// ***************************************************************************************
int32_t ad9739_read(ad9739Device_t *device, uint8_t addr, uint8_t *data)
{
	uint8_t buf[2];
	int32_t Status;

	*data = 0x0;

	buf[0] = AD9739_READ | (0x7F & addr);
	buf[1] = 0x0;
	Status = spi_write_and_read(device->spi_desc, buf, buf, (uint8_t)sizeof(buf));

	*data = buf[1];

	if (Status != XST_SUCCESS)
		return XST_FAILURE;
	else
		return XST_SUCCESS;
}

// ***************************************************************************************
// * @brief Resets the device.
// *
// * @param dev - The device structure.
// *
// * @return Returns negative error code or 0 in case of success.
// ***************************************************************************************
int32_t ad9739_reset(ad9739Device_t *device)
{
	int32_t Status;

	gpio_write(device->gpio_reset, GPIO_HIGH);
	mdelay(100);
	gpio_write(device->gpio_reset, GPIO_LOW);
	mdelay(100);

	/* Software reset to default SPI values. */
	Status = ad9739_write(device, AD9739_REG_MODE, AD9739_MODE_RESET);
	if (Status != XST_SUCCESS)
	{
		return Status;
	}

	/* Clear the reset bit. */
	Status = ad9739_write(device, AD9739_REG_MODE, 0x00);
	if (Status != XST_SUCCESS)
		return XST_FAILURE;
	else
		return XST_SUCCESS;
}

// ***************************************************************************************
// * @brief Powers down LVDS interface and TxDAC.
// *
// * @param dev        - The device structure.
// * @param pwr_config - Selects the modules to be powered-down.
// * Example: AD9739_POWER_DOWN_LVDS_DRVR_PD | AD9739_POWER_DOWN_LVDS_RCVR_PD
// *
// * @return Returns negative error code or 0 in case of success.
// ***************************************************************************************
int32_t ad9739_power_down(ad9739Device_t *device, uint8_t pwr_config)
{
	int32_t Status;
	uint8_t reg_data;

	/* Write to register if pwrConfig contains valid parameters. Else read
	 * and return the register value.
	 * */
	if ((pwr_config & ((1 << 7) | (1 << 6) | (1 << 3) | (1 << 2))) == 0)
	{
		Status = ad9739_write(device, AD9739_REG_POWER_DOWN, pwr_config);
	}
	else
	{
		ad9739_read(device, AD9739_REG_POWER_DOWN, &reg_data);
		return reg_data;
	}

	return Status;
}

// ***************************************************************************************
// * @brief Sets the normal baseband mode or mix-mode.
// *
// * @param dev  - The device structure.
// * @param mode - mode of operation.
// * 		 Example:
// *			AD9739_DAC_DEC_MIX_MODE - mix-mode.
// * 			AD9739_DAC_DEC_NORMAL_BASEBAND - normal baseband mode;
// *
// * @return Returns negative error code or 0 in case of success.
// ***************************************************************************************
int32_t ad_serdes_clk(ad9739Device_t *device, uint8_t mode)
{
	int32_t Status;
	uint8_t reg_data;

	if ((mode == AD9739_DAC_DEC_NORMAL_BASEBAND) | (mode == AD9739_DAC_DEC_MIX_MODE))
	{
		Status = ad9739_write(device, AD9739_REG_DEC_CNT, AD9739_DEC_CNT_DAC_DEC(mode));
	}
	else
	{
		ad9739_read(device, AD9739_REG_DEC_CNT, &reg_data);
		return reg_data;
	}
	return Status;
}

// ***************************************************************************************
// * @brief Sets the full-scale output current for the DAC.
// *
// * @param dev    - The device structure.
// * @param fs_val - The desired full-scale output current. Accepted values:
// *		   8.7 to 32.7 (mA) and 0. When fs_val is set to 0 the DAC
// *		   output is disabled(sleep).
// *
// * @return Returns the actual set full-scale current or negative error code.
// ***************************************************************************************
float ad9739_dac_fs_current(ad9739Device_t *device, float fs_val)
{
	int32_t Status;
	float actual_fs = 0x0;
	int16_t reg_data = 0x0;
	uint8_t reg_data_r = 0x0;

	if ((fs_val >= 8.7) && (fs_val <= 31.7))
	{
		reg_data = (int16_t)((fs_val - 8.58) / 0.0226);
		Status = ad9739_write(device, AD9739_REG_FSC_1, (reg_data & 0xFF));
		if (Status != XST_SUCCESS)
			return (float)Status;

		Status = ad9739_write(device, AD9739_REG_FSC_2, ((reg_data & 0x300) >> 8));
		if (Status != XST_SUCCESS)
			return (float)Status;
	}
	else if (fs_val == 0)
	{
		Status = ad9739_write(device, AD9739_REG_FSC_2, AD9739_FSC_2_Sleep);
		if (Status != XST_SUCCESS)
			return (float)Status;
	}
	else
	{
		ad9739_read(device, AD9739_REG_FSC_1, &reg_data_r);
		if ((int8_t)reg_data_r < 0)
			return -1;

		ad9739_read(device, AD9739_REG_FSC_2, &reg_data_r);
		if ((int8_t)reg_data_r < 0)
			return (float)reg_data_r;

		reg_data |= (reg_data_r & 0x3) << 8;
		actual_fs = (float)(reg_data * 0.0226 + 8.58);
		return actual_fs;
	}

	return (float)Status;
}

// ***************************************************************************************
// * @brief Delay for a number of fdata clock cycles.
// *
// * @param cycles - Number of cycles to wait for.
// *
// * @return Returns negative error code or 0 in case of success.
// ***************************************************************************************
int32_t delay_fdata_cycles(uint32_t cycles)
{
	uint32_t us;
	uint32_t delay;
	volatile uint32_t count;

	/* 10 ns */
	uint8_t cpu_clk_period = 10;

	/* 100 cpu clocks in a microsecond */
	uint32_t cpu_clks_in_a_us = 1000 / cpu_clk_period;

	/* convert DAC cycles to microseconds delay */
	us = (cycles / FDATA);

	/* add 9-10 microseconds to the delay as a precaution */
	delay = ((us + 9) * cpu_clks_in_a_us);
	for (count = 0; count < delay; count++)
	{
		asm("nop");
	}

	return XST_SUCCESS;
}

// ***************************************************************************************
// * @brief Initializes the AD9739A.
// *
// * @param device     - The device structure.
// * @param init_param - The structure that contains the device initial
// * 		       parameters.
// *
// * @return Returns negative error code or 0 in case of success.
// ***************************************************************************************
int32_t ad9739_init(ad9739Device_t *device)
{
	int32_t Status = 0;
	uint8_t chip_id = 0;

	spi_init_param_t spi_param;

	Status = gpio_init(&device->gpio_reset, device->init.gpio_id, device->init.gpio_reset);
	Status = gpio_init(&device->gpio_irq, device->init.gpio_id, device->init.gpio_irq);
	if (Status != XST_SUCCESS)
	{
		xil_printf("\r\n[E]: %s GPIO initialize Failed.\n\r", device->init.name);
		return XST_FAILURE;
	}

	spi_param.id = device->init.spi_deviceID;
	spi_param.mode = device->init.spi_mode;
	spi_param.chip_select = device->init.chip_select;
	Status = spi_init(&device->spi_desc, &spi_param);
	if (Status != XST_SUCCESS)
	{
		xil_printf("\r\n[E]: %s SPI initialize Failed.\n\r", device->init.name);
		return XST_FAILURE;
	}

	/* Set 4-wire SPI, MSB first. */
	Status = ad9739_write(device, AD9739_REG_MODE, 0x00);
	if (Status != XST_SUCCESS)
	{
		return Status;
	}

	/* Reset*/
	Status = ad9739_reset(device);
	if (Status != XST_SUCCESS)
		return Status;

	/* Device ID */
	ad9739_read(device, AD9739_REG_PART_ID, &chip_id);
	if (chip_id == AD9739_CHIP_ID)
	{
		xil_printf("\r\n[I]: %s Get Part ID: 0x%02x\n\r", device->init.name, chip_id);
	}
	else
	{
		xil_printf("\r\n[E]: %s Get Part ID: 0x%02x\n\r", device->init.name, chip_id);
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

int32_t ad9739_config(ad9739Device_t *device)
{
	float fret = 0;
	int32_t Status = 0;
	uint8_t dll_loop_locked = 0;
	uint8_t dll_loop_lock_counter = 0;
	uint8_t ad9739_reg_mu_stat1_buf = 0;
	uint8_t ad9739_reg_lvds_rec_stat9_buf;

	/* Reset*/
	Status = ad9739_reset(device);
	if (Status != XST_SUCCESS)
		return Status;

	/* Set the common-mode voltage of DACCLK_P and DACCLK_N inputs. */
	Status = ad9739_write(device, AD9739_REG_CROSS_CNT1, AD9739_CROSS_CNT1_CLKP_OFFSET(device->init.common_mode_voltage_dacclk_p));
	if (Status != XST_SUCCESS)
	{
		return Status;
	}

	Status = ad9739_write(device, AD9739_REG_CROSS_CNT2, AD9739_CROSS_CNT2_CLKN_OFFSET(device->init.common_mode_voltage_dacclk_n));
	if (Status != XST_SUCCESS)
	{
		return Status;
	}

	/* MU CONTROLLER Setup*/
	/* Phase detector enable and boost bias bits. */
	Status = ad9739_write(device, AD9739_REG_PHS_DET, AD9739_PHS_DET_CMP_BST | AD9739_PHS_DET_PHS_DET_AUTO_EN);
	if (Status != XST_SUCCESS)
	{
		return Status;
	}

	Status = ad9739_write(device, AD9739_REG_MU_DUTY, AD9739_MU_DUTY_MU_DUTY_AUTO_EN);
	if (Status != XST_SUCCESS)
	{
		return Status;
	}

	/* Set the MU Controller search direction to UP and the target phase to 4. */
	Status = ad9739_write(device, AD9739_REG_MU_CNT2, AD9739_MU_CNT2_SRCH_MODE(1) | AD9739_MU_CNT2_SET_PHS(4));
	if (Status != XST_SUCCESS)
	{
		return Status;
	}

	/* Set the MU delay value at witch the controller begins its search to 216. */
	Status = ad9739_write(device, AD9739_REG_MU_CNT3, AD9739_MU_CNT3_MUDEL(0x6C));
	if (Status != XST_SUCCESS)
	{
		return Status;
	}

	do
	{
		/* Set: find the exact targeted phase, retry the search until
		 * correct value is found, continue if desired phase is not
		 * found, guard band set to optimal value: 0xB.
		 * */
		Status = ad9739_write(device, AD9739_REG_MU_CNT4, AD9739_MU_CNT4_SEARCH_TOL | AD9739_MU_CNT4_RETRY | AD9739_MU_CNT4_GUARD(0xB));
		if (Status != XST_SUCCESS)
		{
			return Status;
		}

		/* Set the MU controller tracking gain to the recommended value of 0x1. */
		Status = ad9739_write(device, AD9739_REG_MU_CNT1, AD9739_MU_CNT1_GAIN(0x1));
		if (Status != XST_SUCCESS)
		{
			return Status;
		}

		/* Enable the MU controller. */
		Status = ad9739_write(device, AD9739_REG_MU_CNT1, AD9739_MU_CNT1_GAIN(0x1) | AD9739_MU_CNT1_ENABLE);
		delay_fdata_cycles(180000);
		dll_loop_lock_counter++;
		ad9739_read(device, AD9739_REG_MU_STAT1, &ad9739_reg_mu_stat1_buf);
		if (ad9739_reg_mu_stat1_buf == AD9739_MU_STAT1_MU_LKD)
		{
			dll_loop_locked = 1;
		}
	} while ((dll_loop_lock_counter <= 10) && (dll_loop_locked == 0));

	if (dll_loop_locked == 0)
	{
		xil_printf("\r\n[E]: %s DLL unlocked1.\n\r", device->init.name);
		return XST_FAILURE;
	}

	/* Set FINE_DEL_SKEW to 2 for a larger DCI sampling window. */
	Status = ad9739_write(device, AD9739_REG_LVDS_REC_CNT4, AD9739_LVDS_REC_CNT4_FINE_DEL_SKEW(2) | AD9739_LVDS_REC_CNT4_DCI_DEL(0x7));
	if (Status != XST_SUCCESS)
	{
		return Status;
	}

	dll_loop_locked = 0;
	dll_loop_lock_counter = 0;

	do
	{
		/* Disable the data Rx controller before enabling it. */
		Status = ad9739_write(device, AD9739_REG_LVDS_REC_CNT1, 0x00);
		if (Status != XST_SUCCESS)
		{
			return Status;
		}

		/* Enable the data Rx controller for loop and IRQ. */
		Status = ad9739_write(device, AD9739_REG_LVDS_REC_CNT1, AD9739_LVDS_REC_CNT1_RCVR_LOOP_ON);
		if (Status != XST_SUCCESS)
		{
			return Status;
		}

		/* Enable the data Rx controller for search and track mode. */
		Status = ad9739_write(device, AD9739_REG_LVDS_REC_CNT1, AD9739_LVDS_REC_CNT1_RCVR_LOOP_ON | AD9739_LVDS_REC_CNT1_RCVR_CNT_ENA);
		if (Status != XST_SUCCESS)
		{
			return Status;
		}

		delay_fdata_cycles(135000);
		dll_loop_lock_counter++;
		ad9739_read(device, AD9739_REG_LVDS_REC_STAT9, &ad9739_reg_lvds_rec_stat9_buf);

		if (ad9739_reg_lvds_rec_stat9_buf == (AD9739_LVDS_REC_STAT9_RCVR_LCK | AD9739_LVDS_REC_STAT9_RCVR_TRK_ON))
		{
			dll_loop_locked = 1;
		}

	} while ((dll_loop_lock_counter <= 10) && (dll_loop_locked == 0));

	if (dll_loop_locked == 0)
	{
		xil_printf("\r\n[E]: %s DLL unlocked2.\n\r", device->init.name);
		return XST_FAILURE;
	}

	fret = ad9739_dac_fs_current(device, device->init.full_scale_current);
	if (fret < 0)
	{
		return (int32_t)fret;
	}

	return XST_SUCCESS;
}

// ***************************************************************************************
// * @brief Free the resources allocated by ad9739_init().
// *
// * @param dev - The device structure.
// *
// * @return SUCCESS in case of success, negative error code otherwise.
// ***************************************************************************************
void ad9739_remove(ad9739Device_t *device)
{
	spi_remove(device->spi_desc);
	gpio_remove(device->gpio_reset);
	gpio_remove(device->gpio_irq);
}
