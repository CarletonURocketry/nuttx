/****************************************************************************
 * include/nuttx/sensors/nau7802.h
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

/* Valid calibration modes */
typedef enum
{
  CALMOD_INTERNAL = 0,
  CALMOD_OFFSET = 2,
  CALMOD_GAIN = 3
} nau7802_calibration_mode_e;

/* Valid LDO voltage settings */
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

/* Valid gain settings */
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

/* Valid sample rates */
typedef enum
{
  SPS_10 = 0,
  SPS_20 = 1,
  SPS_40 = 2,
  SPS_80 = 3,
  SPS_320 = 7
} nau7802_sample_rate_e;

/* Valid ODR settings*/
typedef enum nau7802_ord_e
{
  ORD_10HZ,
  ORD_20HZ,
  ORD_40HZ,
  ORD_80HZ,
  ORD_320HZ,
};
/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nau7802_register
 *
 * Description:
 *   Register the nau7802 device as a UORB sensor.
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the nau7802.
 *   addr    - The I2C address of the nau7802. Should always be 0x2a.
 *   devno   - The device number to use for the topic (i.e. /dev/mag0)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nau7802_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr);
