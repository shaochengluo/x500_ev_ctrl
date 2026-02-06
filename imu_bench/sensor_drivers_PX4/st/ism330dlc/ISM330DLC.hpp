#pragma once

#include "ISM330DLC_Registers.hpp"

#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/spi.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>

#ifndef DRV_IMU_DEVTYPE_ST_ISM330DLC
#define DRV_IMU_DEVTYPE_ST_ISM330DLC 0x56
#endif

class ISM330DLC final : public device::SPI, public I2CSPIDriver<ISM330DLC>
{
public:
	ISM330DLC(const I2CSPIDriverConfig &config);
	~ISM330DLC() override;

	int init() override;
	void print_status() override;

private:
	// i2c_spi_buses.h calls this via I2CSPIDriver::Run()
	void RunImpl();

	enum class State : uint8_t {
		RESET = 0,
		CONFIGURE,
		READ
	};

	State _state{State::RESET};
	hrt_abstime _reset_timestamp{0};

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_transfer{nullptr};
	perf_counter_t _bad_register{nullptr};

	float _accel_lsb_to_m_s2{0.f};
	float _gyro_lsb_to_rad_s{0.f};

	// low-level SPI helpers
	int read_reg(uint8_t reg, uint8_t &val);
	int write_reg(uint8_t reg, uint8_t val);
	int read_block(uint8_t start_reg, uint8_t *buf, uint32_t len);

	int probe();
	int reset();
	int configure();
	void update_scales();
};
