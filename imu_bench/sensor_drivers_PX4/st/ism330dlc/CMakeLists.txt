px4_add_module(
	MODULE drivers__imu__st__ism330dlc
	MAIN ism330dlc
	COMPILE_FLAGS
		-Wno-shadow
	SRCS
		ISM330DLC.cpp
		ism330dlc_main.cpp
	DEPENDS
		drivers_device
)
