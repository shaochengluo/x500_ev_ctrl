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

/**
 * @file LSM6DS3.hpp
 *
 * Driver for the ST LSM6DS3 connected via SPI (Accel + Gyro).
 */

#pragma once

#include "ST_LSM6DS3_Registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace ST_LSM6DS3;

class LSM6DS3 : public device::SPI, public I2CSPIDriver<LSM6DS3>
{
public:
	LSM6DS3(const I2CSPIDriverConfig &config);
	~LSM6DS3() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// Sensor configuration (selected by this driver)
	static constexpr float SAMPLE_DT_US{1e6f / ST_LSM6DS3::G_ODR};
	static constexpr float GYRO_RATE{ST_LSM6DS3::G_ODR};
	static constexpr float ACCEL_RATE{ST_LSM6DS3::LA_ODR};

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();
	bool Configure();
	void ConfigureSampleRate(int sample_rate_hz);

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	bool ReadAndPublish(const hrt_abstime &timestamp_sample);
	void UpdateTemperature();

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};

	hrt_abstime _reset_timestamp{0};	hrt_abstime _last_config_check_timestamp{0};	hrt_abstime _temperature_update_timestamp{0};
	int _failure_count{0};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		READ,
	} _state{STATE::RESET};

	uint32_t _sample_interval_us{1200}; // default ~833 Hz

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{4};
	register_config_t _register_cfg[size_register_cfg] {
		// Register              | Set bits, Clear bits
		{ Register::CTRL1_XL,   CTRL1_XL_BIT::ODR_XL_833HZ | CTRL1_XL_BIT::FS_XL_16G, 0 },
		{ Register::CTRL2_G,    CTRL2_G_BIT::ODR_G_833HZ | CTRL2_G_BIT::FS_G_2000DPS, 0 },
		{ Register::CTRL3_C,    CTRL3_C_BIT::BDU | CTRL3_C_BIT::IF_INC, CTRL3_C_BIT::SW_RESET },
		{ Register::CTRL4_C,    CTRL4_C_BIT::I2C_DISABLE, 0 },
	};
};
