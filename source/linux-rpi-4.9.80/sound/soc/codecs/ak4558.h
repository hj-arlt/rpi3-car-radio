/*
 * AK4558.h  --  AK4558 Soc Audio driver
 *
 * Copyright 2017
 *
 * Author: H.J.Arlt <hj.arlt@online.de>
 *
 * Based on ak4535.c by Richard Purdie
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _AK4558_H
#define _AK4558_H

/* AK4558 register space */

#define AK4558_I2C_ADDR    0x11   // 7bit , 0x22 8bit
/* register 0 Power */
#define AK4558_RESET       0x00
#define AK4558_PWR_UP      0x1F

/* register 1 PLL */
#define PLL_MASK           0x1E   // default TDM128
#define PLL_PMPLL          0x01
#define PLL_256FS          0x00
#define PLL_128FS          0x02
#define PLL_64FS           0x04
#define PLL_32FS           0x06

/* register 2 DAC TDM */
#define DAC_L1R2           0x00   // default TDM128

/* register 3 CTRL1 */
#define AK4558_TDM         0xC0
#define AK4558_DIF         0x38
#define AK4558_MUTE        0x01
#define I2S_32BIT          0x38   // >=64fs
#define I2S_16BIT          0x18   // >=32fs
#define I2S_24BIT          0x18   // >=48fs
#define MSB_24BIT          0x08   // >=48fs
#define MSB_32BIT          0x30   // >=64fs

/* register 1 PLL Mode */
#define PMPLL_ON           0x01
#define AK4558_PLL         0x1E
#define BICK_256FS         0x00   // D4:1
#define BICK_128FS         0x02   // D4:1
#define BICK_64FS          0x04
#define BICK_32FS          0x06

/* register 4 CTRL2 */
#define AUTO_CLK           0x01
#define NORM_SPEED         (0 << 1)   // DFS1:0
#define MCK_512FS          (2 << 3)   // default, no auto
#define MCK_256FS          (0 << 3)   // MCKS1:0

/* register 5 Filter DAC */
#define BCKO_64FS          0x02  // master only
#define AK4558_FS          0x78
#define FS_8_13K           0x00
#define FS_12_27K          0x10
#define FS_24_54K          0x20
#define FS_48_108K         0x30
#define FS_96_216K         0x40

/* register 6 Mode Control */
#define DEEM_MASK          0x03
#define DEEM_44100         0x00
#define DEEM_OFF           0x01
#define DEEM_48000         0x02
#define DEEM_32000         0x03

#define REG_PWR           0
#define REG_PLL           1
#define REG_DAC           2
#define REG_CTRL1         3
#define REG_CTRL2         4
#define REG_FILTER_DAC    5
#define REG_MODCTL        6
#define REG_FILTER_ADC    7
#define REG_OUTL          8
#define REG_OUTR          9

#endif
