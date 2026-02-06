/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ST_LSM6DS3_Registers.hpp
 *
 * ST LSM6DS3 registers (SPI).
 */

#pragma once

#include <cstddef>
#include <cstdint>

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace ST_LSM6DS3
{
// Pixhawk external SPI IMU: keep conservative; board-level SPI can go higher if wiring is short.
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10 MHz

// Identification
static constexpr uint8_t WHO_AM_I_ID = 0x69;

// Chosen defaults for this driver (ODR must be configured; power-on default is typically power-down)
static constexpr uint32_t LA_ODR = 833; // Hz
static constexpr uint32_t G_ODR  = 833; // Hz

enum class Register : uint8_t {
	WHO_AM_I   = 0x0F,

	CTRL1_XL   = 0x10,
	CTRL2_G    = 0x11,
	CTRL3_C    = 0x12,
	CTRL4_C    = 0x13,

	OUT_TEMP_L = 0x20,
	OUT_TEMP_H = 0x21,

	OUTX_L_G   = 0x22,
	OUTX_H_G   = 0x23,
	OUTY_L_G   = 0x24,
	OUTY_H_G   = 0x25,
	OUTZ_L_G   = 0x26,
	OUTZ_H_G   = 0x27,

	OUTX_L_XL  = 0x28,
	OUTX_H_XL  = 0x29,
	OUTY_L_XL  = 0x2A,
	OUTY_H_XL  = 0x2B,
	OUTZ_L_XL  = 0x2C,
	OUTZ_H_XL  = 0x2D,
};

// CTRL1_XL (0x10)
// ODR_XL [7:4], FS_XL [3:2]
// FS_XL encoding per datasheet: 00=2g, 01=16g, 10=4g, 11=8g
enum CTRL1_XL_BIT : uint8_t {
	ODR_XL_833HZ = Bit6 | Bit5 | Bit4, // 0b0111xxxx -> 833 Hz
	FS_XL_16G    = Bit2,               // 01 in [3:2]
};

// CTRL2_G (0x11)
// ODR_G [7:4], FS_G [3:2]
enum CTRL2_G_BIT : uint8_t {
	ODR_G_833HZ  = Bit6 | Bit5 | Bit4,
	FS_G_2000DPS = Bit3 | Bit2,
};

// CTRL3_C (0x12)
// BDU bit6, IF_INC bit2, SW_RESET bit0
enum CTRL3_C_BIT : uint8_t {
	BDU      = Bit6,
	IF_INC   = Bit2,
	SW_RESET = Bit0,
};

// CTRL4_C (0x13)
// I2C_disable bit2
enum CTRL4_C_BIT : uint8_t {
	I2C_DISABLE = Bit2,
};

} // namespace ST_LSM6DS3
