#pragma once

#include <cstdint>

namespace ISM330DLC_REG {

static constexpr uint8_t WHO_AM_I   = 0x0F;
static constexpr uint8_t CTRL1_XL   = 0x10;
static constexpr uint8_t CTRL2_G    = 0x11;
static constexpr uint8_t CTRL3_C    = 0x12;
static constexpr uint8_t STATUS_REG = 0x1E;
static constexpr uint8_t OUTX_L_G   = 0x22; // 6 bytes gyro (x,y,z)
static constexpr uint8_t OUTX_L_XL  = 0x28; // 6 bytes accel (x,y,z)

static constexpr uint8_t WHOAMI_EXPECTED = 0x6A;

// CTRL3_C bits (common across ST LSM6/ISM330 family)
static constexpr uint8_t CTRL3_C_BOOT      = (1u << 7);
static constexpr uint8_t CTRL3_C_BDU       = (1u << 6);
static constexpr uint8_t CTRL3_C_H_LACTIVE = (1u << 5);
static constexpr uint8_t CTRL3_C_PP_OD     = (1u << 4);
static constexpr uint8_t CTRL3_C_SIM       = (1u << 3);
static constexpr uint8_t CTRL3_C_IF_INC    = (1u << 2);
static constexpr uint8_t CTRL3_C_BLE       = (1u << 1);
static constexpr uint8_t CTRL3_C_SW_RESET  = (1u << 0);

// STATUS_REG bits
static constexpr uint8_t STATUS_XLDA = (1u << 0);
static constexpr uint8_t STATUS_GDA  = (1u << 1);

// Helpers: default ODR/FS (104 Hz, accel ±4 g, gyro ±2000 dps)
static constexpr uint8_t CTRL1_XL_ODR_104HZ = (0x4u << 4);
static constexpr uint8_t CTRL1_XL_FS_4G     = (0x2u << 2);
static constexpr uint8_t CTRL1_XL_BW_100HZ  = (0x0u << 0);
static constexpr uint8_t CTRL1_XL_DEFAULT   = (CTRL1_XL_ODR_104HZ | CTRL1_XL_FS_4G | CTRL1_XL_BW_100HZ);

static constexpr uint8_t CTRL2_G_ODR_104HZ  = (0x4u << 4);
static constexpr uint8_t CTRL2_G_FS_2000DPS = (0x3u << 2);
static constexpr uint8_t CTRL2_G_DEFAULT    = (CTRL2_G_ODR_104HZ | CTRL2_G_FS_2000DPS);

} // namespace ISM330DLC_REG
