/****************************************************************************
 * include/nuttx/sensors/lis2mdl.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s; /* Forward reference */

typedef int (*nau7802_attach)(xcpt_t, FAR void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lis2mdl_register
 *
 * Description:
 *   Register the LIS2MDL device as a UORB sensor.
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the LIS2MDL.
 *   addr    - The I2C address of the LIS2MDL. Should always be 0x1e.
 *   devno   - The device number to use for the topic (i.e. /dev/mag0)
 *   attach  - A function which is called by this driver to attach the
 *             LIS2MDL interrupt handler to an IRQ. Pass NULL to operate
 *             in polling mode. This function should return 0 on succes
 *             and a negated error code otherwise.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

// The NAU7802 supports two modes, 0-100kHz and 0-400kHz
#ifndef CONFIG_SENSORS_NAU7802_I2C_FREQUENCY
#define CONFIG_SENSORS_NAU7802_I2C_FREQUENCY                                 \
  100000 // is this 100kHz? we'll find out I guess
#endif

#ifndef CONFIG_SENSORS_NAU7802_THREAD_STACKSIZE
#define CONFIG_SENSORS_NAU7802_THREAD_STACKSIZE                              \
  10000 // ACTUAL TODO THIS IS THE FIRST NUMBER I CAN THINK OF
#endif

#define REG_PU_CTRL 0x00 // power up control
#define REG_CTRL_1 0x01  // control/config reg 1
#define REG_CTRL_2 0x02  // control/config reg 2

#define REG_GCAL1_B3 0x6 // gain calibration registers
#define REG_GCAL1_B2 0x7
#define REG_GCAL1_B1 0x8
#define REG_GCAL1_B0 0x9

#define REG_ADCO_B2 0x12 // data bit 23 to 16
#define REG_ADCO_B1 0x13 // data bit 15 to 8
#define REG_ADCO_B0 0x14 // data bit 7 to 0
#define REG_ADC 0x15     // ADC / chopper control
#define REG_PGA 0x1B     // PGA control
#define REG_POWER 0x1C   // power control

// Bits for the PU_CTRL register
#define BIT_RR 0x0    // register reset
#define BIT_PUD 0x1   // power up digital
#define BIT_PUA 0x2   // power up analog
#define BIT_PUR 0x3   // power up ready
#define BIT_CS 0x4    // cycle start
#define BIT_CR 0x5    // cycle ready
#define BIT_AVVDS 0x7 // AVDDS source select

// Bits for the CTRL_2 register
#define CAL_START 0x2
#define CAL_ERR 0x3 // THIS FUCKER IS COMPLETELY UNDOCUMENTED

typedef enum nau7802_ord_e
{
  ORD_10HZ,
  ORD_20HZ,
  ORD_40HZ,
  ORD_80HZ,
  ORD_320HZ,
};

static const uint32_t ODR_TO_INTERVAL[] = {[ORD_10HZ] = 100000,
                                           [ORD_20HZ] = 50000,
                                           [ORD_40HZ] = 25000,
                                           [ORD_80HZ] = 12500,
                                           [ORD_320HZ] = 3125};

typedef enum
{
  SPS_10 = 0,
  SPS_20 = 1,
  SPS_40 = 2,
  SPS_80 = 3,
  SPS_320 = 7
} nau7802_sample_rate_e;

typedef enum
{
  LDO_V_4V5,
  LDO_V_4V2,
  LDO_V_3V9,
  LDO_V_3V6,
  LDO_V_3V3,
  LDO_V_3V0,
  LDO_V_2V7,
  LDO_V_2V4,
  LDO_V_EXTERNAL
} nau7802_ldo_voltage_enum;

typedef enum
{
  GAIN_1,
  GAIN_2,
  GAIN_4,
  GAIN_8,
  GAIN_16,
  GAIN_32,
  GAIN_64,
  GAIN_128
} nau7802_gain_e;

typedef enum
{
  CALMOD_INTERNAL = 0,
  CALMOD_OFFSET = 2,
  CALMOD_GAIN = 3
} nau7802_calibration_mode_e;

int nau7802_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr);
