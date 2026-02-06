/****************************************************************************
 *
 *   Copyright (c) 2020-2026 PX4 Development Team. All rights reserved.
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

#include "LSM6DSO.hpp"

#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/px4_config.h>

using namespace time_literals;

LSM6DSO::LSM6DSO(const I2CSPIDriverConfig &config) :
    	 SPI(config),
    	 I2CSPIDriver(config),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	_px4_accel.set_scale(CONSTANTS_ONE_G); // default, overridden by set_range()
	_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
}

LSM6DSO::~LSM6DSO()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int LSM6DSO::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	// kick off the driver state machine
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();

	return PX4_OK;
}


int LSM6DSO::probe()
{
	// Some PX4 versions call probe() before init(), so ensure the SPI bus is initialized here.
	const int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != WHO_AM_I_ID) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void LSM6DSO::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

void LSM6DSO::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// CTRL3_C: Software reset
		RegisterSetAndClearBits(Register::CTRL3_C, CTRL3_C_BIT::SW_RESET, 0);
		_reset_timestamp = now;
		_failure_count = 0;
		_checked_register = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(10_ms); // allow reset time
		break;

	case STATE::WAIT_FOR_RESET: {
		// Wait until SW_RESET clears (datasheet: bit auto-clears)
		const uint8_t ctrl3 = RegisterRead(Register::CTRL3_C);

		if ((ctrl3 & CTRL3_C_BIT::SW_RESET) == 0) {
			_state = STATE::CONFIGURE;
			ScheduleNow();

		} else if (now - _reset_timestamp > 100_ms) {
			PX4_DEBUG("reset timeout");
			_state = STATE::RESET;
			ScheduleDelayed(10_ms);
		} else {
			ScheduleDelayed(10_ms);
		}
		break;
	}

	case STATE::CONFIGURE:
		if (Configure()) {
			_state = STATE::READ;
			ScheduleDelayed(1_ms);
		} else {
			_state = STATE::RESET;
			ScheduleDelayed(10_ms);
		}
		break;

	case STATE::READ:
		if (!ReadData()) {
			_failure_count++;

			if (_failure_count > 10) {
				_state = STATE::RESET;
				ScheduleDelayed(10_ms);
				break;
			}
		} else {
			_failure_count = 0;
		}

		// Periodic register check (optional)
		if (now - _last_config_check_timestamp > 100_ms) {
			_last_config_check_timestamp = now;
			if (!RegisterCheck(_register_cfg[_checked_register])) {
				perf_count(_bad_register_perf);
				_state = STATE::CONFIGURE;
				ScheduleNow();
				break;
			}
			_checked_register = (_checked_register + 1) % size_register_cfg;
		}

		ScheduleDelayed(DEFAULT_SAMPLE_INTERVAL_US);
		break;
	}
}

bool LSM6DSO::Configure()
{
	bool success = true;

	for (auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
		const uint8_t v = RegisterRead(reg_cfg.reg);

		// Verify "set bits"
		if (reg_cfg.set_bits && ((v & reg_cfg.set_bits) != reg_cfg.set_bits)) {
			success = false;
		}

		// Verify "clear bits"
		if (reg_cfg.clear_bits && ((v & reg_cfg.clear_bits) != 0)) {
			success = false;
		}
	}

	// Set scaling (datasheet: accel 0.488 mg/LSB @ ±16g; gyro 70 mdps/LSB @ ±2000 dps)
	const float accel_scale = (0.488f * 1e-3f) * CONSTANTS_ONE_G; // mg -> g -> m/s^2
	const float gyro_scale  = math::radians(0.07f); // 70 mdps = 0.07 dps -> rad/s

	_px4_accel.set_scale(accel_scale);
	_px4_accel.set_range(16.f * CONSTANTS_ONE_G);

	_px4_gyro.set_scale(gyro_scale);
	_px4_gyro.set_range(math::radians(2000.f));

	return success;
}

bool LSM6DSO::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;
	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)",
			  (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)",
			  (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

static inline int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (int16_t)((uint16_t)msb << 8u | lsb);
}

bool LSM6DSO::ReadData()
{
	// Burst read starting at OUT_TEMP_L (0x20):
	// TEMP(2) + G(6) + XL(6) = 14 bytes
	struct TransferBuffer {
		uint8_t cmd;
		uint8_t data[14];
	} __attribute__((packed)) buffer{};

	buffer.cmd = (static_cast<uint8_t>(Register::OUT_TEMP_L) << 1) | SPI_READ_BIT;

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, sizeof(buffer)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	const int16_t temp_raw  = combine(buffer.data[1], buffer.data[0]);

	const int16_t gx = combine(buffer.data[3], buffer.data[2]);
	const int16_t gy = combine(buffer.data[5], buffer.data[4]);
	const int16_t gz = combine(buffer.data[7], buffer.data[6]);

	const int16_t ax = combine(buffer.data[9],  buffer.data[8]);
	const int16_t ay = combine(buffer.data[11], buffer.data[10]);
	const int16_t az = combine(buffer.data[13], buffer.data[12]);

	(void)temp_raw; // temperature not published here (optional)

	// Sensor frame assumed +x forward, +y left, +z up.
	// Publish right-handed NED-compatible body frame: x forward, y right, z down.
	const float gyro_x = (float)gx;
	const float gyro_y = (float)-gy;
	const float gyro_z = (float)-gz;

	const float accel_x = (float)ax;
	const float accel_y = (float)-ay;
	const float accel_z = (float)-az;

	_px4_gyro.update(timestamp_sample, gyro_x, gyro_y, gyro_z);
	_px4_accel.update(timestamp_sample, accel_x, accel_y, accel_z);

	return true;
}

uint8_t LSM6DSO::RegisterRead(Register reg)
{
	uint8_t tx[2] {};
	uint8_t rx[2] {};

	tx[0] = (static_cast<uint8_t>(reg) << 1) | SPI_READ_BIT;
	tx[1] = 0;

	if (transfer(tx, rx, sizeof(tx)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	return rx[1];
}

void LSM6DSO::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t tx[2] {};
	uint8_t rx[2] {};

	tx[0] = (static_cast<uint8_t>(reg) << 1) | SPI_WRITE_BIT;
	tx[1] = value;

	if (transfer(tx, rx, sizeof(tx)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
	}
}

void LSM6DSO::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	const uint8_t new_val = (orig_val | setbits) & ~clearbits;

	if (new_val != orig_val) {
		RegisterWrite(reg, new_val);
	}
}
