#include "ISM330DLC.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <mathlib/mathlib.h>

using namespace ISM330DLC_REG;

ISM330DLC::ISM330DLC(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	_px4_accel.set_device_type(DRV_IMU_DEVTYPE_ST_ISM330DLC);
	_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_ST_ISM330DLC);

	_bad_transfer = perf_alloc(PC_COUNT, MODULE_NAME": bad transfer");
	_bad_register = perf_alloc(PC_COUNT, MODULE_NAME": bad register");
}

ISM330DLC::~ISM330DLC()
{
	stop();
	perf_free(_bad_transfer);
	perf_free(_bad_register);
}

int ISM330DLC::init()
{
	// Initialize the SPI base (configures bus + CS)
	const int ret = SPI::init();

	if (ret != PX4_OK) {
		PX4_ERR("SPI::init failed (%d)", ret);
		return ret;
	}

	// Start state machine
	_state = State::RESET;
	_reset_timestamp = hrt_absolute_time();
	ScheduleNow();
	return PX4_OK;
}

void ISM330DLC::stop()
{
	ScheduleClear();
}

void ISM330DLC::print_status()
{
	PX4_INFO("WHO_AM_I: 0x%02x", _whoami);
	perf_print_counter(_bad_transfer);
	perf_print_counter(_bad_register);
}

int ISM330DLC::probe()
{
	uint8_t who = 0;

	if (read_reg(WHO_AM_I, who) != PX4_OK) {
		perf_count(_bad_transfer);
		return -EIO;
	}

	_whoami = who;

	if (who != WHOAMI_EXPECTED) {
		perf_count(_bad_register);
		return -ENODEV;
	}

	return PX4_OK;
}

int ISM330DLC::reset()
{
	// CTRL3_C layout is shared across this ST family:
	// bit0 SW_RESET, bit2 IF_INC, bit6 BDU
	const uint8_t ctrl3_common = CTRL3_C_IF_INC | CTRL3_C_BDU;

	// Set SW_RESET (self-clears)
	const int ret = write_reg(CTRL3_C, static_cast<uint8_t>(ctrl3_common | CTRL3_C_SW_RESET));

	if (ret != PX4_OK) {
		perf_count(_bad_transfer);
		return ret;
	}

	_reset_timestamp = hrt_absolute_time();
	return PX4_OK;
}

int ISM330DLC::configure()
{
	// Re-assert IF_INC and BDU after reset
	int ret = write_reg(CTRL3_C, static_cast<uint8_t>(CTRL3_C_IF_INC | CTRL3_C_BDU));

	if (ret != PX4_OK) {
		perf_count(_bad_transfer);
		return ret;
	}

	// Default configuration: ODR 104 Hz, Accel ±4g, Gyro ±2000 dps
	// CTRL1_XL: ODR_XL=104Hz (0x40), FS_XL=4g (0x08)
	// CTRL2_G : ODR_G=104Hz  (0x40), FS_G =2000dps (0x0C)
	ret = write_reg(CTRL1_XL, 0x48);

	if (ret != PX4_OK) {
		perf_count(_bad_transfer);
		return ret;
	}

	ret = write_reg(CTRL2_G, 0x4C);

	if (ret != PX4_OK) {
		perf_count(_bad_transfer);
		return ret;
	}

	update_scales();

	// 1 kHz polling; actual publication gated on STATUS_REG
	ScheduleOnInterval(1000);
	return PX4_OK;
}

void ISM330DLC::update_scales()
{
	// Accel: m/s^2 per LSB = FS(g) * g / 32768
	const float g = CONSTANTS_ONE_G;
	const float fs_g = 4.f;
	_accel_lsb_to_m_s2 = (fs_g * g) / 32768.f;

	// Gyro: 2000 dps -> 70 mdps/LSB (0.07 dps/LSB) on this family
	const float dps_per_lsb = 0.07f;
	_gyro_lsb_to_rad_s = math::radians(dps_per_lsb);
}

int ISM330DLC::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t tx[2]{};
	uint8_t rx[2]{};
	// bit7=1 read
	tx[0] = static_cast<uint8_t>(reg | 0x80);
	tx[1] = 0;

	const int ret = transfer(tx, rx, sizeof(tx));

	if (ret == PX4_OK) {
		val = rx[1];
	}

	return ret;
}

int ISM330DLC::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t tx[2]{};
	// bit7=0 write
	tx[0] = static_cast<uint8_t>(reg & 0x7F);
	tx[1] = val;
	return transfer(tx, nullptr, sizeof(tx));
}

int ISM330DLC::read_block(uint8_t start_reg, uint8_t *buf, uint32_t len)
{
	if (len == 0 || buf == nullptr) {
		return PX4_OK;
	}

	// small stack buffer; typical len is 12
	if (len > 32) {
		return -EINVAL;
	}

	uint8_t tx[33]{};
	uint8_t rx[33]{};

	// bit7=1 read, bit6=1 auto-increment
	tx[0] = static_cast<uint8_t>(start_reg | 0xC0);

	for (uint32_t i = 1; i < (len + 1); i++) {
		tx[i] = 0;
	}

	const int ret = transfer(tx, rx, static_cast<unsigned>(len + 1));

	if (ret == PX4_OK) {
		for (uint32_t i = 0; i < len; i++) {
			buf[i] = rx[i + 1];
		}
	}

	return ret;
}

void ISM330DLC::RunImpl()
{
	switch (_state) {
	case State::RESET: {
		// Probe first: if sensor isn't present, back off and retry without crashing the FC
		if (probe() != PX4_OK) {
			// Try again in 200 ms
			ScheduleDelayed(200_ms);
			return;
		}

		// Kick reset
		if (reset() != PX4_OK) {
			ScheduleDelayed(200_ms);
			return;
		}

		_state = State::WAIT_RESET;
		ScheduleDelayed(50_ms);
		return;
	}

	case State::WAIT_RESET: {
		// Allow SW_RESET to complete. Datasheet specifies it auto-clears.
		if ((hrt_absolute_time() - _reset_timestamp) < 50_ms) {
			ScheduleDelayed(10_ms);
			return;
		}

		_state = State::CONFIGURE;
		ScheduleNow();
		return;
	}

	case State::CONFIGURE: {
		if (configure() != PX4_OK) {
			_state = State::RESET;
			ScheduleDelayed(200_ms);
			return;
		}

		_state = State::READ;
		return;
	}

	case State::READ:
	default:
		break;
	}

	// READ state: gated on STATUS_REG data-ready
	uint8_t status = 0;

	if (read_reg(STATUS_REG, status) != PX4_OK) {
		perf_count(_bad_transfer);
		return;
	}

	const bool gyro_ready = (status & STATUS_GDA);
	const bool accel_ready = (status & STATUS_XLDA);

	if (!gyro_ready && !accel_ready) {
		return;
	}

	uint8_t buf[12]{};

	if (read_block(OUTX_L_G, buf, sizeof(buf)) != PX4_OK) {
		perf_count(_bad_transfer);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (gyro_ready) {
		const int16_t gx = (int16_t)((buf[1] << 8) | buf[0]);
		const int16_t gy = (int16_t)((buf[3] << 8) | buf[2]);
		const int16_t gz = (int16_t)((buf[5] << 8) | buf[4]);

		_px4_gyro.update(now,
				 gx * _gyro_lsb_to_rad_s,
				 gy * _gyro_lsb_to_rad_s,
				 gz * _gyro_lsb_to_rad_s);
	}

	if (accel_ready) {
		const int16_t ax = (int16_t)((buf[7] << 8) | buf[6]);
		const int16_t ay = (int16_t)((buf[9] << 8) | buf[8]);
		const int16_t az = (int16_t)((buf[11] << 8) | buf[10]);

		_px4_accel.update(now,
				 ax * _accel_lsb_to_m_s2,
				 ay * _accel_lsb_to_m_s2,
				 az * _accel_lsb_to_m_s2);
	}
}
