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

#include <cstdint>

// LSM6DSO SPI protocol (datasheet DS12140 Rev 3, Section 5.1.2):
//  - bit0 = R/W (1=READ, 0=WRITE)
//  - bit1..7 = register address AD[6:0]
//  - MSB first
static constexpr uint8_t SPI_READ_BIT  = 0x01;
static constexpr uint8_t SPI_WRITE_BIT = 0x00;

static constexpr uint8_t WHO_AM_I_ID = 0b01101100; // WHO_AM_I (0x0F) fixed to 0x6C

enum class Register : uint8_t {
	FUNC_CFG_ACCESS   = 0x01,

	FIFO_CTRL1        = 0x07,
	FIFO_CTRL2        = 0x08,
	FIFO_CTRL3        = 0x09,
	FIFO_CTRL4        = 0x0A,
	FIFO_STATUS1      = 0x3A,
	FIFO_STATUS2      = 0x3B,

	INT1_CTRL         = 0x0D,
	INT2_CTRL         = 0x0E,
	WHO_AM_I          = 0x0F,

	CTRL1_XL          = 0x10,
	CTRL2_G           = 0x11,
	CTRL3_C           = 0x12,
	CTRL4_C           = 0x13,
	CTRL5_C           = 0x14,
	CTRL6_C           = 0x15,
	CTRL7_G           = 0x16,
	CTRL8_XL          = 0x17,
	CTRL9_XL          = 0x18,
	CTRL10_C          = 0x19,

	STATUS_REG        = 0x1E,

	OUT_TEMP_L        = 0x20,
	OUT_TEMP_H        = 0x21,

	OUTX_L_G          = 0x22,
	OUTX_H_G          = 0x23,
	OUTY_L_G          = 0x24,
	OUTY_H_G          = 0x25,
	OUTZ_L_G          = 0x26,
	OUTZ_H_G          = 0x27,

	OUTX_L_A          = 0x28,
	OUTX_H_A          = 0x29,
	OUTY_L_A          = 0x2A,
	OUTY_H_A          = 0x2B,
	OUTZ_L_A          = 0x2C,
	OUTZ_H_A          = 0x2D,
};

// CTRL1_XL bits (ODR_XL[3:0] in bits 7:4, FS_XL[1:0] in bits 3:2)
namespace CTRL1_XL_BIT
{
static constexpr uint8_t ODR_POWERDOWN = 0x00 << 4;
static constexpr uint8_t ODR_12_5HZ    = 0x01 << 4;
static constexpr uint8_t ODR_26HZ      = 0x02 << 4;
static constexpr uint8_t ODR_52HZ      = 0x03 << 4;
static constexpr uint8_t ODR_104HZ     = 0x04 << 4;
static constexpr uint8_t ODR_208HZ     = 0x05 << 4;
static constexpr uint8_t ODR_416HZ     = 0x06 << 4;
static constexpr uint8_t ODR_833HZ     = 0x07 << 4;
// FS_XL (with XL_FS_MODE=0 default): 00=2g, 10=4g, 11=8g, 01=16g
static constexpr uint8_t FS_2G   = 0x00 << 2;
static constexpr uint8_t FS_16G  = 0x01 << 2;
static constexpr uint8_t FS_4G   = 0x02 << 2;
static constexpr uint8_t FS_8G   = 0x03 << 2;
}

// CTRL2_G bits (ODR_G[3:0] in bits 7:4, FS_G[1:0] in bits 3:2)
namespace CTRL2_G_BIT
{
static constexpr uint8_t ODR_POWERDOWN = 0x00 << 4;
static constexpr uint8_t ODR_12_5HZ    = 0x01 << 4;
static constexpr uint8_t ODR_26HZ      = 0x02 << 4;
static constexpr uint8_t ODR_52HZ      = 0x03 << 4;
static constexpr uint8_t ODR_104HZ     = 0x04 << 4;
static constexpr uint8_t ODR_208HZ     = 0x05 << 4;
static constexpr uint8_t ODR_416HZ     = 0x06 << 4;
static constexpr uint8_t ODR_833HZ     = 0x07 << 4;
// FS_G: 00=250dps, 01=500dps, 10=1000dps, 11=2000dps
static constexpr uint8_t FS_250DPS  = 0x00 << 2;
static constexpr uint8_t FS_500DPS  = 0x01 << 2;
static constexpr uint8_t FS_1000DPS = 0x02 << 2;
static constexpr uint8_t FS_2000DPS = 0x03 << 2;
}

// CTRL3_C bits
namespace CTRL3_C_BIT
{
static constexpr uint8_t BOOT     = 1 << 7;
static constexpr uint8_t BDU      = 1 << 6;
static constexpr uint8_t H_LACTIVE= 1 << 5;
static constexpr uint8_t PP_OD    = 1 << 4;
static constexpr uint8_t SIM      = 1 << 3; // 1=3-wire, 0=4-wire
static constexpr uint8_t IF_INC   = 1 << 2; // address auto-increment
static constexpr uint8_t SW_RESET = 1 << 0;
}

// STATUS_REG bits
namespace STATUS_REG_BIT
{
static constexpr uint8_t XLDA = 1 << 0; // accel new data
static constexpr uint8_t GDA  = 1 << 1; // gyro new data
static constexpr uint8_t TDA  = 1 << 2; // temp new data
}
