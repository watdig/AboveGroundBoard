/*
 * vl53l0x.c
 *
 *  Created on: Feb 2, 2025
 *      Author: Victor Kalenda
 */

// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

#include <stdint.h>
#include "main.h" // Change it for your requirements.
#include "string.h"
#include "vl53l0x.h"

uint8_t i2c_tx_buffer[I2C_BUFFER_SIZE];
uint8_t i2c_rx_buffer[I2C_BUFFER_SIZE];

uint8_t vl53l0x_address;
uint32_t i2c_tx_time = 0;
uint32_t i2c_rx_time = 0;
volatile uint8_t i2c_tx_int = 1;
volatile uint8_t i2c_rx_int = 1;
volatile uint8_t i2c_err_int = 0;

extern uint16_t holding_register_database[NUM_HOLDING_REGISTERS];
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_tx_int = 1;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_rx_int = 1;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_tx_int = 1;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_rx_int = 1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	// Do something
	i2c_err_int = 1;
	__HAL_I2C_DISABLE_IT(&hi2c1, I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI |
										 I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI);
}

uint8_t g_i2cAddr = ADDRESS_DEFAULT;
uint16_t g_ioTimeout = 0;  // no timeout
uint8_t g_isTimeout = 0;
uint16_t g_timeoutStartMs;
uint8_t g_stopVariable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
uint32_t g_measTimBudUs;

#define I2C_TIMEOUT 100 // I2C timeout in ms
#define I2C_READ 1
#define I2C_WRITE 0
I2C_HandleTypeDef VL53L0X_I2C_Handler; // I2C handler
uint8_t msgBuffer[4];
HAL_StatusTypeDef i2cStat;


int8_t vl53l0x_get_spad_info(uint8_t *count, uint8_t *type_is_aperture);
int8_t vl53l0x_get_sequence_step_enables(SequenceStepEnables * enables);
int8_t vl53l0x_get_sequence_step_timeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);
int8_t vl53l0x_single_reference_calibration(uint8_t vhv_init_byte);
static uint16_t vl53l0x_decode_timeout(uint16_t value);
static uint16_t vl53l0x_encode_timeout(uint16_t timeout_mclks);
static uint32_t vl53l0x_timeout_mclks_to_us(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t vl53l0x_timeout_us_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

//---------------------------------------------------------
// I2C communication Functions
//---------------------------------------------------------
// Write an 8-bit register
int8_t writeReg(uint8_t reg, uint8_t value)
{
	i2c_tx_buffer[0] = value; // Assign the value to the buffer.
	return handle_i2c_error(HAL_I2C_Mem_Write(&hi2c1, vl53l0x_address | I2C_WRITE, reg, 1, i2c_tx_buffer, 1, I2C_TIMEOUT));
}

// Write a 16-bit register
int8_t writeReg16Bit(uint8_t reg, uint16_t value)
{
	memcpy(i2c_tx_buffer, &value, 2); // Assign the value to the buffer.
	return handle_i2c_error(HAL_I2C_Mem_Write(&hi2c1, vl53l0x_address | I2C_WRITE, reg, 1, i2c_tx_buffer, 2, I2C_TIMEOUT));
}

// Write a 32-bit register
int8_t writeReg32Bit(uint8_t reg, uint32_t value)
{

  memcpy(i2c_tx_buffer, &value, 4); // Assign the value to the buffer.
  return handle_i2c_error(HAL_I2C_Mem_Write(&hi2c1, vl53l0x_address | I2C_WRITE, reg, 1, i2c_tx_buffer, 4, I2C_TIMEOUT));

}

// Read an 8-bit register
int8_t readReg(uint8_t reg, uint8_t* value)
{
	int8_t status = HAL_I2C_Mem_Read(&hi2c1, vl53l0x_address | I2C_READ, reg, 1, i2c_rx_buffer, 1, I2C_TIMEOUT);
	(*value) = i2c_rx_buffer[0];
	return handle_i2c_error(status);
}

// Read a 16-bit register
int8_t readReg16Bit(uint8_t reg, uint16_t* value)
{
	int8_t status = HAL_I2C_Mem_Read(&hi2c1, vl53l0x_address | I2C_READ, reg, 1, i2c_rx_buffer, 2, I2C_TIMEOUT);
	memcpy(value, i2c_rx_buffer, 2);
	return handle_i2c_error(status);
}

// Read a 32-bit register
int8_t readReg32Bit(uint8_t reg, uint32_t* value)
{
	int8_t status = HAL_I2C_Mem_Read(&hi2c1, vl53l0x_address | I2C_READ, reg, 1, i2c_rx_buffer, 4, I2C_TIMEOUT);
	memcpy(value, i2c_rx_buffer, 4);
	return handle_i2c_error(status);
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
int8_t writeMulti(uint8_t reg, uint8_t const *src, uint8_t count)
{
	memcpy(i2c_tx_buffer, src, 4);
	return handle_i2c_error(HAL_I2C_Mem_Write(&hi2c1, vl53l0x_address | I2C_WRITE, reg, 1, msgBuffer, count, I2C_TIMEOUT));
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
int8_t readMulti(uint8_t reg, uint8_t* dst, uint8_t count)
{
	 return handle_i2c_error(HAL_I2C_Mem_Read(&hi2c1, vl53l0x_address | I2C_READ, reg, 1, dst, count, I2C_TIMEOUT));
}


// Public Methods //////////////////////////////////////////////////////////////

int8_t vl53l0x_set_address(uint8_t new_addr)
{
  int8_t status = writeReg(I2C_SLAVE_DEVICE_ADDRESS, (new_addr>>1) & 0x7F );
  vl53l0x_address = new_addr;
  return status;
}

uint8_t vl53l0x_get_address()
{
  return vl53l0x_address;
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is HAL_OK or not given, the sensor is configured for 2V8
// mode.
int8_t vl53l0x_init(uint8_t io_2v8)
{
	int8_t status = HAL_OK;
	uint8_t reg_val = 0;

	// Reset buffers
	for(uint8_t i = 0; i < I2C_BUFFER_SIZE; i++)
	{
		i2c_tx_buffer[i] = 0;
		i2c_rx_buffer[i] = 0;
	}
	vl53l0x_address = ADDRESS_DEFAULT;

	// Data Initialization
	// sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
	if (io_2v8)
	{
		status = readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &reg_val);
		if(status != HAL_OK){return status;}
		status = writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, (reg_val | 0x01)); // set bit 0
		if(status != HAL_OK){return status;}
		reg_val = 0;
	}

	// "Set I2C standard mode"
	status = writeReg(0x88, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x80, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0xFF, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x00, 0x00);
	if(status != HAL_OK){return status;}
	status = readReg(0x91, &g_stopVariable);
	if(status != HAL_OK){return status;}
	status = writeReg(0x00, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0xFF, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x80, 0x00);
	if(status != HAL_OK){return status;}

	// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	status = readReg(MSRC_CONFIG_CONTROL, &reg_val);
	if(status != HAL_OK){return status;}
	status = writeReg(MSRC_CONFIG_CONTROL, reg_val | 0x12);
	if(status != HAL_OK){return status;}
	reg_val = 0;

	// set final range signal rate limit to 0.25 MCPS (million counts per second)
	status = vl53l0x_set_signal_rate_limit(0.25);
	if(status != HAL_OK){return status;}

	status = writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);
	if(status != HAL_OK){return status;}

	// Static Initialization
	uint8_t spad_count;
	uint8_t spad_type_is_aperture;
	status = vl53l0x_get_spad_info(&spad_count, &spad_type_is_aperture);
	if(status != HAL_OK){return status;}

	// The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
	// the API, but the same data seems to be more easily readable from
	// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	uint8_t ref_spad_map[6];
	status = readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
	if(status != HAL_OK){return status;}

	// -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)
	status = writeReg(0xFF, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	if(status != HAL_OK){return status;}
	status = writeReg(0xFF, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
	if(status != HAL_OK){return status;}

	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
	uint8_t spads_enabled = 0;

	for (uint8_t i = 0; i < 48; i++)
	{
		if (i < first_spad_to_enable || spads_enabled == spad_count)
		{
			// This bit is lower than the first one that should be enabled, or
			// (reference_spad_count) bits have already been enabled, so zero this bit
			ref_spad_map[i / 8] &= ~(1 << (i % 8));
		}
		else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
		{
			spads_enabled++;
		}
	}

	status = writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
	if(status != HAL_OK){return status;}
	// -- VL53L0X_set_reference_spads() end

	// -- VL53L0X_load_tuning_settings() begin
	// DefaultTuningSettings from vl53l0x_tuning.h
	status = writeReg(0xFF, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x00, 0x00);
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x09, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x10, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x11, 0x00);
	if(status != HAL_OK){return status;}

	status = writeReg(0x24, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x25, 0xFF);
	if(status != HAL_OK){return status;}
	status = writeReg(0x75, 0x00);
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x4E, 0x2C);
	if(status != HAL_OK){return status;}
	status = writeReg(0x48, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x30, 0x20);
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x30, 0x09);
	if(status != HAL_OK){return status;}
	status = writeReg(0x54, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x31, 0x04);
	if(status != HAL_OK){return status;}
	status = writeReg(0x32, 0x03);
	if(status != HAL_OK){return status;}
	status = writeReg(0x40, 0x83);
	if(status != HAL_OK){return status;}
	status = writeReg(0x46, 0x25);
	if(status != HAL_OK){return status;}
	status = writeReg(0x60, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x27, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x50, 0x06);
	if(status != HAL_OK){return status;}
	status = writeReg(0x51, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x52, 0x96);
	if(status != HAL_OK){return status;}
	status = writeReg(0x56, 0x08);
	if(status != HAL_OK){return status;}
	status = writeReg(0x57, 0x30);
	if(status != HAL_OK){return status;}
	status = writeReg(0x61, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x62, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x64, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x65, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x66, 0xA0);
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x22, 0x32);
	if(status != HAL_OK){return status;}
	status = writeReg(0x47, 0x14);
	if(status != HAL_OK){return status;}
	status = writeReg(0x49, 0xFF);
	if(status != HAL_OK){return status;}
	status = writeReg(0x4A, 0x00);
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x7A, 0x0A);
	if(status != HAL_OK){return status;}
	status = writeReg(0x7B, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x78, 0x21);
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x23, 0x34);
	if(status != HAL_OK){return status;}
	status = writeReg(0x42, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x44, 0xFF);
	if(status != HAL_OK){return status;}
	status = writeReg(0x45, 0x26);
	if(status != HAL_OK){return status;}
	status = writeReg(0x46, 0x05);
	if(status != HAL_OK){return status;}
	status = writeReg(0x40, 0x40);
	if(status != HAL_OK){return status;}
	status = writeReg(0x0E, 0x06);
	if(status != HAL_OK){return status;}
	status = writeReg(0x20, 0x1A);
	if(status != HAL_OK){return status;}
	status = writeReg(0x43, 0x40);
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x34, 0x03);
	if(status != HAL_OK){return status;}
	status = writeReg(0x35, 0x44);
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x31, 0x04);
	if(status != HAL_OK){return status;}
	status = writeReg(0x4B, 0x09);
	if(status != HAL_OK){return status;}
	status = writeReg(0x4C, 0x05);
	if(status != HAL_OK){return status;}
	status = writeReg(0x4D, 0x04);
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x44, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x45, 0x20);
	if(status != HAL_OK){return status;}
	status = writeReg(0x47, 0x08);
	if(status != HAL_OK){return status;}
	status = writeReg(0x48, 0x28);
	if(status != HAL_OK){return status;}
	status = writeReg(0x67, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x70, 0x04);
	if(status != HAL_OK){return status;}
	status = writeReg(0x71, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x72, 0xFE);
	if(status != HAL_OK){return status;}
	status = writeReg(0x76, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x77, 0x00);
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x0D, 0x01);
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x80, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x01, 0xF8);
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x8E, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x00, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0xFF, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x80, 0x00);
	if(status != HAL_OK){return status;}

	// -- VL53L0X_load_tuning_settings() end

	// "Set interrupt config to new sample ready"
	// -- VL53L0X_SetGpioConfig() begin

	status = writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
	if(status != HAL_OK){return status;}
	status = readReg(GPIO_HV_MUX_ACTIVE_HIGH, &reg_val);
	if(status != HAL_OK){return status;}
	status = writeReg(GPIO_HV_MUX_ACTIVE_HIGH, reg_val & ~0x10); // active low
	if(status != HAL_OK){return status;}
	reg_val = 0;
	status = writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
	if(status != HAL_OK){return status;}

	// -- VL53L0X_SetGpioConfig() end

//	status = vl53l0x_get_measurement_timing_budget(&g_measTimBudUs);
//	if(status != HAL_OK){return status;}

	// "Disable MSRC and TCC by default"
	// MSRC = Minimum Signal Rate Check
	// TCC = Target CentreCheck
	// -- VL53L0X_SetSequenceStepEnable() begin
	status = writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
	if(status != HAL_OK){return status;}
	// -- VL53L0X_SetSequenceStepEnable() end

	// "Recalculate timing budget"
//	status = vl53l0x_set_measurement_timing_budget(g_measTimBudUs);
//	if(status != HAL_OK){return status;}

	// VL53L0X_StaticInit() end

	// VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

	// -- VL53L0X_perform_vhv_calibration() begin
	status = writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
	if(status != HAL_OK){return status;}
	status = vl53l0x_single_reference_calibration(0x40);
	if(status != HAL_OK){return status;}
	// -- VL53L0X_perform_vhv_calibration() end

	// -- VL53L0X_perform_phase_calibration() begin
	status = writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
	if(status != HAL_OK){return status;}
	status = vl53l0x_single_reference_calibration(0x00);
	if(status != HAL_OK){return status;}
	// -- VL53L0X_perform_phase_calibration() end

	// "restore the previous Sequence Config"
	status = writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
	if(status != HAL_OK){return status;}
	// VL53L0X_PerformRefCalibration() end

	return HAL_OK;
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
int8_t vl53l0x_set_signal_rate_limit(float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return HAL_ERROR; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  return writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
}

// Get the return signal rate limit check value in MCPS
int8_t vl53l0x_get_signal_rate_limit(float* limit_Mcps)
{
	uint16_t reg_val = 0;
	int8_t status = readReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, &reg_val);
	(*limit_Mcps) = (float) (reg_val / (1 << 7));
	return status;
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
int8_t vl53l0x_set_measurement_timing_budget(uint32_t budget_us)
{
	int8_t status = HAL_OK;
	SequenceStepEnables enables;
	SequenceStepTimeouts timeouts;

	uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
	uint16_t const EndOverhead        = 960;
	uint16_t const MsrcOverhead       = 660;
	uint16_t const TccOverhead        = 590;
	uint16_t const DssOverhead        = 690;
	uint16_t const PreRangeOverhead   = 660;
	uint16_t const FinalRangeOverhead = 550;

	uint32_t const MinTimingBudget = 20000;

	if (budget_us < MinTimingBudget) { return HAL_ERROR; }

	uint32_t used_budget_us = StartOverhead + EndOverhead;

	status = vl53l0x_get_sequence_step_enables(&enables);
	if(status != HAL_OK){return status;}
	status = vl53l0x_get_sequence_step_timeouts(&enables, &timeouts);
	if(status != HAL_OK){return status;}

	if(enables.tcc)
	{
		used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}

	if(enables.dss)
	{
		used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	}
	else if(enables.msrc)
	{
		used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}

	if(enables.pre_range)
	{
		used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}

	if(enables.final_range)
	{
		used_budget_us += FinalRangeOverhead;

		// "Note that the final range timeout is determined by the timing
		// budget and the sum of all other timeouts within the sequence.
		// If there is no room for the final range timeout, then an error
		// will be set. Otherwise the remaining time will be applied to
		// the final range."

		if(used_budget_us > budget_us)
		{
			// "Requested timeout too big."
			return HAL_ERROR;
		}

		uint32_t final_range_timeout_us = budget_us - used_budget_us;

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

		// "For the final range timeout, the pre-range timeout
		//  must be added. To do this both final and pre-range
		//  timeouts must be expressed in macro periods MClks
		//  because they have different vcsel periods."

		uint16_t final_range_timeout_mclks = vl53l0x_timeout_us_to_mclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

		if (enables.pre_range)
		{
			final_range_timeout_mclks += timeouts.pre_range_mclks;
		}

		status = writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, vl53l0x_encode_timeout(final_range_timeout_mclks));
		if(status != HAL_OK){return status;}
		// set_sequence_step_timeout() end

		g_measTimBudUs = budget_us; // store for internal reuse
	}
	return status;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
int8_t vl53l0x_get_measurement_timing_budget(uint32_t* budget_us)
{
	int8_t status = HAL_OK;
	SequenceStepEnables enables;
	SequenceStepTimeouts timeouts;

	uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
	uint16_t const EndOverhead        = 960;
	uint16_t const MsrcOverhead       = 660;
	uint16_t const TccOverhead        = 590;
	uint16_t const DssOverhead        = 690;
	uint16_t const PreRangeOverhead   = 660;
	uint16_t const FinalRangeOverhead = 550;

	// "Start and end overhead times always present"
	(*budget_us) = StartOverhead + EndOverhead;

	status = vl53l0x_get_sequence_step_enables(&enables);
	if(status != HAL_OK){return status;}
	status = vl53l0x_get_sequence_step_timeouts(&enables, &timeouts);
	if(status != HAL_OK){return status;}

	if (enables.tcc)
	{
		(*budget_us) += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}

	if (enables.dss)
	{
		(*budget_us) += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	}
	else if (enables.msrc)
	{
		(*budget_us) += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}

	if (enables.pre_range)
	{
		(*budget_us) += (timeouts.pre_range_us + PreRangeOverhead);
	}

	if (enables.final_range)
	{
		(*budget_us) += (timeouts.final_range_us + FinalRangeOverhead);
	}

	g_measTimBudUs = (*budget_us); // store for internal reuse
	return status;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
int8_t vl53l0x_set_vcsel_pulse_period(vcselPeriodType type, uint8_t period_pclks)
{
	int8_t status = HAL_OK;
	uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

	SequenceStepEnables enables;
	SequenceStepTimeouts timeouts;

	status = vl53l0x_get_sequence_step_enables(&enables);
	if(status != HAL_OK){return status;}
	status = vl53l0x_get_sequence_step_timeouts(&enables, &timeouts);
	if(status != HAL_OK){return status;}

	// "Apply specific settings for the requested clock period"
	// "Re-calculate and apply timeouts, in macro periods"

	// "When the VCSEL period for the pre or final range is changed,
	// the corresponding timeout must be read from the device using
	// the current VCSEL period, then the new VCSEL period can be
	// applied. The timeout then must be written back to the device
	// using the new VCSEL period.
	//
	// For the MSRC timeout, the same applies - this timeout being
	// dependant on the pre-range vcsel period."


	if (type == VcselPeriodPreRange)
	{
		// "Set phase check limits"
		switch (period_pclks)
		{
			case 12:
			{
				status = writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
				break;
			}
			case 14:
			{
				status = writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
				break;
			}
			case 16:
			{
				status = writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
				break;
			}
			case 18:
			{
				status = writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
				break;
			}
			default:
			{
				// invalid period
				return HAL_ERROR;
			}
		}
		if(status != HAL_OK){return status;}
		status = writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if(status != HAL_OK){return status;}

		// apply new VCSEL period
		status = writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
		if(status != HAL_OK){return status;}

		// update timeouts
		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)
		uint16_t new_pre_range_timeout_mclks = vl53l0x_timeout_us_to_mclks(timeouts.pre_range_us, period_pclks);

		status = writeReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, vl53l0x_encode_timeout(new_pre_range_timeout_mclks));
		if(status != HAL_OK){return status;}
		// set_sequence_step_timeout() end

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)
		uint16_t new_msrc_timeout_mclks = vl53l0x_timeout_us_to_mclks(timeouts.msrc_dss_tcc_us, period_pclks);
		status = writeReg(MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
		if(status != HAL_OK){return status;}
		// set_sequence_step_timeout() end
	}
	else if (type == VcselPeriodFinalRange)
	{
		switch (period_pclks)
		{
			case 8:
			{
				status = writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
				if(status != HAL_OK){return status;}
				status = writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
				if(status != HAL_OK){return status;}
				status = writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
				if(status != HAL_OK){return status;}
				status = writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
				if(status != HAL_OK){return status;}
				status = writeReg(0xFF, 0x01);
				if(status != HAL_OK){return status;}
				status = writeReg(ALGO_PHASECAL_LIM, 0x30);
				if(status != HAL_OK){return status;}
				status = writeReg(0xFF, 0x00);
				break;
			}
			case 10:
			{
				status = writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
				if(status != HAL_OK){return status;}
				status = writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
				if(status != HAL_OK){return status;}
				status = writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				if(status != HAL_OK){return status;}
				status = writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
				if(status != HAL_OK){return status;}
				status = writeReg(0xFF, 0x01);
				if(status != HAL_OK){return status;}
				status = writeReg(ALGO_PHASECAL_LIM, 0x20);
				if(status != HAL_OK){return status;}
				status = writeReg(0xFF, 0x00);
				break;
			}
			case 12:
			{
				status = writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
				if(status != HAL_OK){return status;}
				status = writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
				if(status != HAL_OK){return status;}
				status = writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				if(status != HAL_OK){return status;}
				status = writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
				if(status != HAL_OK){return status;}
				status = writeReg(0xFF, 0x01);
				if(status != HAL_OK){return status;}
				status = writeReg(ALGO_PHASECAL_LIM, 0x20);
				if(status != HAL_OK){return status;}
				status = writeReg(0xFF, 0x00);
				break;
			}
			case 14:
			{
				status = writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
				if(status != HAL_OK){return status;}
				status = writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
				if(status != HAL_OK){return status;}
				status = writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				if(status != HAL_OK){return status;}
				status = writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
				if(status != HAL_OK){return status;}
				status = writeReg(0xFF, 0x01);
				if(status != HAL_OK){return status;}
				status = writeReg(ALGO_PHASECAL_LIM, 0x20);
				if(status != HAL_OK){return status;}
				status = writeReg(0xFF, 0x00);
				break;
			}
			default:
			{
				// invalid period
				return HAL_ERROR;
			}
		}
		if(status != HAL_OK){return status;}

		// apply new VCSEL period
		status = writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
		if(status != HAL_OK){return status;}

		// update timeouts
		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

		// "For the final range timeout, the pre-range timeout
		//  must be added. To do this both final and pre-range
		//  timeouts must be expressed in macro periods MClks
		//  because they have different vcsel periods."
		uint16_t new_final_range_timeout_mclks = vl53l0x_timeout_us_to_mclks(timeouts.final_range_us, period_pclks);

		if (enables.pre_range)
		{
			new_final_range_timeout_mclks += timeouts.pre_range_mclks;
		}
		status = writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, vl53l0x_encode_timeout(new_final_range_timeout_mclks));
		if(status != HAL_OK){return status;}
		// set_sequence_step_timeout end
	}
	else
	{
		// invalid type
		return HAL_ERROR;
	}

	// "Finally, the timing budget must be re-applied"
	status = vl53l0x_set_measurement_timing_budget(g_measTimBudUs);
	if(status != HAL_OK){return status;}


	// "Perform the phase calibration. This is needed after changing on vcsel period."
	// VL53L0X_perform_phase_calibration() begin

	uint8_t sequence_config = 0;
	status = readReg(SYSTEM_SEQUENCE_CONFIG, &sequence_config);
	if(status != HAL_OK){return status;}
	status = writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
	if(status != HAL_OK){return status;}
	status = vl53l0x_single_reference_calibration(0x0);
	if(status != HAL_OK){return status;}
	status = writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);
	// VL53L0X_perform_phase_calibration() end

	return status;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
int8_t vl53l0x_get_vcsel_pulse_period(vcselPeriodType type, uint8_t* period_pclks)
{
	int8_t status = HAL_OK;
	uint8_t reg_value = 0;
	if (type == VcselPeriodPreRange)
	{
		status = readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, &reg_value);
		if(status != HAL_OK){return status;}
		(*period_pclks) = decodeVcselPeriod(reg_value);
	}
	else if (type == VcselPeriodFinalRange)
	{
		status = readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, &reg_value);
		if(status != HAL_OK){return status;}
		(*period_pclks) = decodeVcselPeriod(reg_value);
	}
	else
	{
		return HAL_ERROR;
	}
	return status;
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
int8_t vl53l0x_start_continuous(uint32_t period_ms)
{
	int8_t status = HAL_OK;

	status = writeReg(0x80, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0xFF, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x00, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x91, g_stopVariable);
	if(status != HAL_OK){return status;}
	status = writeReg(0x00, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0xFF, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x80, 0x00);
	if(status != HAL_OK){return status;}

	if(period_ms != 0)
	{
		// continuous timed mode

		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
		uint16_t osc_calibrate_val = 0;
		status = readReg16Bit(OSC_CALIBRATE_VAL, &osc_calibrate_val);

		if(osc_calibrate_val != 0)
		{
			period_ms *= osc_calibrate_val;
		}

		status = writeReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
		if(status != HAL_OK){return status;}
		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

		status = writeReg(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
	}
	else
	{
		// continuous back-to-back mode
		status = writeReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
	}
	return status;
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
int8_t vl53l0x_stop_continuous()
{
	int8_t status = HAL_OK;
	status = writeReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x00, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x91, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x00, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0xFF, 0x00);

	return status;
}

// Returns a range reading in millimeters when continuous mode is active
// (vl53l0x_read_range_single() also calls this function after starting a
// single-shot range measurement)
// extraStats provides additional info for this measurment. Set to 0 if not needed.
int8_t vl53l0x_read_range_continuous( statInfo_t_VL53L0X *extraStats, uint16_t* range_mm)
{
	int8_t status = HAL_OK;
	uint8_t reg_val = 0;
	startTimeout();
	while((reg_val & 0x07) == 0)
	{
		status = readReg(RESULT_INTERRUPT_STATUS, &reg_val);
		if(status != HAL_OK){return status;}
		if(checkTimeoutExpired())
		{
			g_isTimeout = HAL_OK;
			return HAL_ERROR;
		}
	}
	if(extraStats == 0)
	{
		// assumptions: Linearity Corrective Gain is 1000 (default);
		// fractional ranging is not enabled
		status = readReg16Bit(RESULT_RANGE_STATUS + 10, range_mm);
		if(status != HAL_OK){return status;}
	}
	else
	{
		// Register map starting at 0x14
		//     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
		//    5A 06 BC 04 00 85 00 38 00 19 06 B6 00 00 00 00
		//   0: Ranging status, uint8_t
		//   1: ???
		// 2,3: Effective SPAD return count, uint16_t, fixpoint8.8
		//   4: 0 ?
		//   5: ???
		// 6,7: signal count rate [mcps], uint16_t, fixpoint9.7
		// 8,9: AmbientRateRtnMegaCps  [mcps], uint16_t, fixpoimt9.7
		// 10,11: uncorrected distance [mm], uint16_t
		status = readMulti(0x14, i2c_rx_buffer, 12);
		if(status != HAL_OK){return status;}

		extraStats->rangeStatus =  i2c_rx_buffer[0]>>3;
		extraStats->spadCnt     = (i2c_rx_buffer[2]<<8) | i2c_rx_buffer[3];
		extraStats->signalCnt   = (i2c_rx_buffer[6]<<8) | i2c_rx_buffer[7];
		extraStats->ambientCnt  = (i2c_rx_buffer[8]<<8) | i2c_rx_buffer[9];
		(*range_mm)             = (i2c_rx_buffer[10]<<8) | i2c_rx_buffer[11];
		extraStats->rawDistance = (*range_mm);
	}
	status = writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
	return status;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
// extraStats provides additional info for this measurment. Set to 0 if not needed.
int8_t vl53l0x_read_range_single(statInfo_t_VL53L0X *extraStats, uint16_t* range_mm)
{
	int8_t status = HAL_OK;
	uint8_t reg_val = 1;
	status = writeReg(0x80, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0xFF, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x00, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x91, g_stopVariable);
	if(status != HAL_OK){return status;}
	status = writeReg(0x00, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0xFF, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x80, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(SYSRANGE_START, 0x01);
	if(status != HAL_OK){return status;}

	// "Wait until start bit has been cleared"
	startTimeout();
	while(reg_val & 0x01)
	{
		status = readReg(SYSRANGE_START, &reg_val);
		if(status != HAL_OK){return status;}
		if (checkTimeoutExpired())
		{
			g_isTimeout = HAL_OK;
			return HAL_ERROR;
		}
	}
	return vl53l0x_read_range_continuous(extraStats, range_mm);
}

// Did a timeout occur in one of the read functions since the last call to
// vl53l0x_timeout()?
uint8_t vl53l0x_timeout()
{
  uint8_t tmp = g_isTimeout;
  g_isTimeout = HAL_ERROR;
  return tmp;
}

void vl53l0x_set_timeout(uint16_t timeout)
{
  g_ioTimeout = timeout;
}

uint16_t vl53l0x_get_timeout()
{
  return g_ioTimeout;
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
int8_t vl53l0x_get_spad_info(uint8_t * count, uint8_t * type_is_aperture)
{
	int8_t status = HAL_OK;
	uint8_t reg_val = 0;

	status = writeReg(0x80, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0xFF, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x00, 0x00);
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x06);
	if(status != HAL_OK){return status;}
	status = readReg(0x83, &reg_val);
	if(status != HAL_OK){return status;}
	status = writeReg(0x83, reg_val | 0x04);
	if(status != HAL_OK){return status;}
	reg_val = 0;
	status = writeReg(0xFF, 0x07);
	if(status != HAL_OK){return status;}
	status = writeReg(0x81, 0x01);
	if(status != HAL_OK){return status;}

	status = writeReg(0x80, 0x01);
	if(status != HAL_OK){return status;}

	status = writeReg(0x94, 0x6b);
	if(status != HAL_OK){return status;}
	status = writeReg(0x83, 0x00);
	if(status != HAL_OK){return status;}

	startTimeout();
	while(reg_val == 0x00)
	{
		status = readReg(0x83, &reg_val);
		if(status != HAL_OK){return status;}
		if (checkTimeoutExpired())
		{
			return HAL_ERROR;
		}
	}
	status = writeReg(0x83, 0x01);
	if(status != HAL_OK){return status;}
	status = readReg(0x92, &reg_val);
	if(status != HAL_OK){return status;}

	*count = reg_val & 0x7f;
	*type_is_aperture = (reg_val >> 7) & 0x01;
	reg_val = 0;

	status = writeReg(0x81, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0xFF, 0x06);
	if(status != HAL_OK){return status;}
	status = readReg(0x83, &reg_val);
	if(status != HAL_OK){return status;}
	status = writeReg(0x83, reg_val & ~0x04);
	if(status != HAL_OK){return status;}
	status = writeReg(0xFF, 0x01);
	if(status != HAL_OK){return status;}
	status = writeReg(0x00, 0x01);
	if(status != HAL_OK){return status;}

	status = writeReg(0xFF, 0x00);
	if(status != HAL_OK){return status;}
	status = writeReg(0x80, 0x00);

	return status;
}

// Get sequence step enables
// based on VL53L0X_vl53l0x_get_sequence_step_enables()
int8_t vl53l0x_get_sequence_step_enables(SequenceStepEnables * enables)
{
	int8_t status = HAL_OK;
	uint8_t sequence_config = 0;
	status = readReg(SYSTEM_SEQUENCE_CONFIG, &sequence_config);
	if(status != HAL_OK){return status;}

	enables->tcc          = (sequence_config >> 4) & 0x1;
	enables->dss          = (sequence_config >> 3) & 0x1;
	enables->msrc         = (sequence_config >> 2) & 0x1;
	enables->pre_range    = (sequence_config >> 6) & 0x1;
	enables->final_range  = (sequence_config >> 7) & 0x1;

	return status;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
int8_t vl53l0x_get_sequence_step_timeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
	int8_t status = HAL_OK;
	uint8_t reg_val_8 = 0;
	uint16_t reg_val_16 = 0;

	status = vl53l0x_get_vcsel_pulse_period(VcselPeriodPreRange, &reg_val_8);
	if(status != HAL_OK){return status;}
	timeouts->pre_range_vcsel_period_pclks = reg_val_8;

	status = readReg(MSRC_CONFIG_TIMEOUT_MACROP, &reg_val_8);
	if(status != HAL_OK){return status;}
	timeouts->msrc_dss_tcc_mclks = reg_val_8 + 1;
	timeouts->msrc_dss_tcc_us =
	vl53l0x_timeout_mclks_to_us(timeouts->msrc_dss_tcc_mclks,
						   timeouts->pre_range_vcsel_period_pclks);

	status = readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &reg_val_16);
	if(status != HAL_OK){return status;}
	timeouts->pre_range_mclks = vl53l0x_decode_timeout(reg_val_16);
	timeouts->pre_range_us =
	vl53l0x_timeout_mclks_to_us(timeouts->pre_range_mclks,
						   timeouts->pre_range_vcsel_period_pclks);

	status = vl53l0x_get_vcsel_pulse_period(VcselPeriodFinalRange, &reg_val_8);
	if(status != HAL_OK){return status;}
	timeouts->final_range_vcsel_period_pclks = reg_val_8;

	status = readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &reg_val_16);
	if(status != HAL_OK){return status;}
	timeouts->final_range_mclks = vl53l0x_decode_timeout(reg_val_16);

	if (enables->pre_range)
	{
		timeouts->final_range_mclks -= timeouts->pre_range_mclks;
	}

	timeouts->final_range_us =
	vl53l0x_timeout_mclks_to_us(timeouts->final_range_mclks,
						   timeouts->final_range_vcsel_period_pclks);
	return status;
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t vl53l0x_decode_timeout(uint16_t reg_val)
{
	// format: "(LSByte * 2^MSByte) + 1"
	return (uint16_t)((reg_val & 0x00FF) <<
		 (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t vl53l0x_encode_timeout(uint16_t timeout_mclks)
{
	// format: "(LSByte * 2^MSByte) + 1"
	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;

	if(timeout_mclks > 0)
	{
		ls_byte = timeout_mclks - 1;

		while((ls_byte & 0xFFFFFF00) > 0)
		{
			ls_byte >>= 1;
			ms_byte++;
		}

		return (ms_byte << 8) | (ls_byte & 0xFF);
	}
	else
	{
		return 0;
	}
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t vl53l0x_timeout_mclks_to_us(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
	uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

	return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t vl53l0x_timeout_us_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
	uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

	return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// based on VL53L0X_perform_single_ref_calibration()
int8_t vl53l0x_single_reference_calibration(uint8_t vhv_init_byte)
{
	int8_t status = HAL_OK;
	uint8_t reg_val = 0;

	status = writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP
	if(status != HAL_OK){return status;}

	startTimeout();
	while((reg_val & 0x07) == 0)
	{
		status = readReg(RESULT_INTERRUPT_STATUS, &reg_val);
		if(checkTimeoutExpired())
		{
			return HAL_ERROR;
		}
	}

	status = writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
	if(status != HAL_OK){return status;}

	status = writeReg(SYSRANGE_START, 0x00);

	return HAL_OK;
}

int8_t handle_i2c_error(int8_t status)
{

	if(status != HAL_OK)
	{
		int8_t reset_status = HAL_OK;
		holding_register_database[I2C_ERRORS] |= hi2c1.ErrorCode;
		// Attempt to reset the peripheral
		reset_status = i2c_reset();
		if(reset_status != HAL_OK)
		{
			// I2C Fatal Error
			holding_register_database[I2C_ERRORS] |= 1U << 10U;
			holding_register_database[I2C_SHUTDOWN] = 1;
		}
	}
	return status;
}

int8_t i2c_reset()
{
	int8_t status = HAL_OK;
	status |= HAL_I2C_DeInit(&hi2c1);
	__I2C1_FORCE_RESET();
	HAL_Delay(100);
	__I2C1_RELEASE_RESET();
	status = HAL_I2C_Init(&hi2c1);
	status |= HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
	status |= HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
	return status;
}
