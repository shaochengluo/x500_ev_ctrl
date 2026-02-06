/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "LSM6DS3.hpp"

using namespace time_literals;

static inline int16_t le16_to_i16(uint8_t lsb, uint8_t msb)
{
	return (int16_t)((uint16_t)msb << 8u | (uint16_t)lsb);
}

// LSM6DS3 SPI framing: cmd = (addr << 1) | R/W  (R/W: 1=read, 0=write)
static inline uint8_t spi_cmd_read(uint8_t reg)  { return (uint8_t)((reg << 1) | 1); }
static inline uint8_t spi_cmd_write(uint8_t reg) { return (uint8_t)((reg << 1) | 0); }

LSM6DS3::LSM6DS3(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

LSM6DS3::~LSM6DS3()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int LSM6DS3::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool LSM6DS3::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void LSM6DS3::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void LSM6DS3::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("sample interval: %u us (%.1f Hz)", (unsigned)_sample_interval_us, 1e6 / (double)_sample_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

int LSM6DS3::probe()
{
	const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != WHO_AM_I_ID) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void LSM6DS3::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// Software reset
		RegisterWrite(Register::CTRL3_C, CTRL3_C_BIT::SW_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(50_ms);
		break;

	case STATE::WAIT_FOR_RESET: {
		// Wait until device responds and reset bit cleared
		const uint8_t whoami = RegisterRead(Register::WHO_AM_I);
		const uint8_t ctrl3 = RegisterRead(Register::CTRL3_C);

		if ((whoami == WHO_AM_I_ID) && ((ctrl3 & CTRL3_C_BIT::SW_RESET) == 0)) {
			// Disable I2C (recommended for SPI-only wiring)
			RegisterWrite(Register::CTRL4_C, CTRL4_C_BIT::I2C_DISABLE);

			_state = STATE::CONFIGURE;
			ScheduleDelayed(10_ms);

		} else {
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(50_ms);

			} else {
				ScheduleDelayed(10_ms);
			}
		}
		break;
	}

	case STATE::CONFIGURE:
		if (Configure()) {
			_state = STATE::READ;
			ScheduleOnInterval(_sample_interval_us, _sample_interval_us);

		} else {
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;
			} else {
				PX4_DEBUG("Configure failed, retrying");
			}
			ScheduleDelayed(50_ms);
		}
		break;

	case STATE::READ: {
		bool success = false;

		if (ReadAndPublish(now)) {
			success = true;
			if (_failure_count > 0) {
				_failure_count--;
			}
		} else {
			_failure_count++;
			if (_failure_count > 10) {
				Reset();
				return;
			}
		}

		if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 200_ms) {
			if (RegisterCheck(_register_cfg[_checked_register])) {
				_last_config_check_timestamp = now;
				_checked_register = (_checked_register + 1) % size_register_cfg;
			} else {
				perf_count(_bad_register_perf);
				Reset();
				return;
			}

		} else {
			// update temperature ~1 Hz
			if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
				UpdateTemperature();
				_temperature_update_timestamp = now;
			}
		}
		break;
	}
	}
}

void LSM6DS3::ConfigureSampleRate(int sample_rate_hz)
{
	// Bound to sensor ODR
	const int rate = math::constrain(sample_rate_hz, 1, (int)GYRO_RATE);
	_sample_interval_us = (uint32_t)math::max(1.0f, roundf(1e6f / (float)rate));
}

bool LSM6DS3::Configure()
{
	// set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	// Gyroscope configuration: 2000 dps, sensitivity 70 mdps/LSB
	_px4_gyro.set_scale(math::radians(70.f / 1000.f));
	_px4_gyro.set_range(math::radians(2000.f));

	// Accelerometer configuration: 16 g, sensitivity 0.488 mg/LSB
	_px4_accel.set_scale(0.488f * (CONSTANTS_ONE_G / 1000.f));
	_px4_accel.set_range(16.f * CONSTANTS_ONE_G);

	return success;
}

bool LSM6DS3::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint8_t LSM6DS3::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = spi_cmd_read((uint8_t)reg);
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void LSM6DS3::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { spi_cmd_write((uint8_t)reg), value };
	transfer(cmd, cmd, sizeof(cmd));
}

void LSM6DS3::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	const uint8_t val = (orig_val & ~clearbits) | setbits;
	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

bool LSM6DS3::ReadAndPublish(const hrt_abstime &timestamp_sample)
{
	// Burst read: OUTX_L_G (0x22) .. OUTZ_H_XL (0x2D) => 12 bytes
	struct TransferBuffer {
		uint8_t cmd{spi_cmd_read((uint8_t)Register::OUTX_L_G)};
		uint8_t data[12]{};
	} buffer{};

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	const int16_t gx = le16_to_i16(buffer.data[0], buffer.data[1]);
	const int16_t gy = le16_to_i16(buffer.data[2], buffer.data[3]);
	const int16_t gz = le16_to_i16(buffer.data[4], buffer.data[5]);

	const int16_t ax = le16_to_i16(buffer.data[6], buffer.data[7]);
	const int16_t ay = le16_to_i16(buffer.data[8], buffer.data[9]);
	const int16_t az = le16_to_i16(buffer.data[10], buffer.data[11]);

	// Publish as FIFO messages with one sample (keeps the driver structure similar to LSM9DS1)
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = 1;
	gyro.dt = SAMPLE_DT_US;
	gyro.x[0] = gx;
	gyro.y[0] = gy;
	gyro.z[0] = (gz == INT16_MIN) ? INT16_MAX : -gz;

	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 1;
	accel.dt = SAMPLE_DT_US;
	accel.x[0] = ax;
	accel.y[0] = ay;
	accel.z[0] = (az == INT16_MIN) ? INT16_MAX : -az;

	_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf));
	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf));

	_px4_gyro.updateFIFO(gyro);
	_px4_accel.updateFIFO(accel);

	return true;
}

void LSM6DS3::UpdateTemperature()
{
	struct TransferBuffer {
		uint8_t cmd{spi_cmd_read((uint8_t)Register::OUT_TEMP_L)};
		uint8_t OUT_TEMP_L{0};
		uint8_t OUT_TEMP_H{0};
	} buffer{};

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	const int16_t raw = le16_to_i16(buffer.OUT_TEMP_L, buffer.OUT_TEMP_H);
	// LSM6DS3: 0 LSB (typ.) at 25C; datasheet specifies sensitivity in LSB/°C (commonly 16 LSB/°C)
	const float temperature = (raw / 16.0f) + 25.0f;

	if (PX4_ISFINITE(temperature)) {
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
	}
}
