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

#pragma once

#include "ST_LSM6DSO_Registers.hpp"

#include <drivers/device/spi.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>

class LSM6DSO final : public device::SPI, public I2CSPIDriver<LSM6DSO>
{
public:
	LSM6DSO(const I2CSPIDriverConfig &config);
	~LSM6DSO() override;

	static void print_usage();

	int init() override;
	void print_status() override;

	void RunImpl();

private:
	// I2CSPIDriver
	int probe() override;

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		READ,
	} _state{STATE::RESET};

	struct register_config_t {
		Register reg;
		uint8_t set_bits;
		uint8_t clear_bits;
	};

	// Choose a conservative default ODR for external IMU polling to keep CPU load reasonable.
	// You can raise to ODR_416HZ or ODR_833HZ once detection is stable.
	static constexpr uint8_t DEFAULT_XL_CFG = CTRL1_XL_BIT::ODR_208HZ | CTRL1_XL_BIT::FS_16G;
	static constexpr uint8_t DEFAULT_G_CFG  = CTRL2_G_BIT::ODR_208HZ | CTRL2_G_BIT::FS_2000DPS;
	static constexpr uint32_t DEFAULT_SAMPLE_INTERVAL_US = 1000000 / 208; // ~4807 us

	static constexpr uint8_t size_register_cfg{3};
	register_config_t _register_cfg[size_register_cfg] {
		{ Register::CTRL3_C,  CTRL3_C_BIT::BDU | CTRL3_C_BIT::IF_INC,  CTRL3_C_BIT::SW_RESET | CTRL3_C_BIT::SIM },
		{ Register::CTRL1_XL, DEFAULT_XL_CFG, 0 },
		{ Register::CTRL2_G,  DEFAULT_G_CFG,  0 },
	};

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	int _failure_count{0};
	uint8_t _checked_register{0};

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);
	bool RegisterCheck(const register_config_t &reg_cfg);

	bool Configure();
	bool ReadData();
};
