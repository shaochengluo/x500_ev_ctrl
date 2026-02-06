#include "ISM330DLC.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

// Safe default SPI speed for external cabling; ISM330DLC supports higher, but start conservative
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10 MHz

void ISM330DLC::print_usage()
{
	PRINT_MODULE_USAGE_NAME("ism330dlc", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int ism330dlc_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = ISM330DLC;
	BusCLIArguments cli{false, true};
	cli.default_spi_frequency = SPI_SPEED;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (enum Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_IMU_DEVTYPE_ST_ISM330DLC);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
