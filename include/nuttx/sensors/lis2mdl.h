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

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_SENSORS_LIS2MDL_SPI
struct spi_dev_s; /* Forward reference */
#else
struct i2c_master_s; /* Forward reference */
#endif

typedef int (*lis2mdl_attach)(xcpt_t, FAR void *arg);

struct lis2mdl_config_s
{
#ifdef CONFIG_SENSORS_LIS2MDL_SPI
  FAR struct spi_dev_s *spi;
  int spi_devid;
#else
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
#endif
};

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
 *   config  - SPI or I2C interface configuration.
 *   devno   - The device number to use for the topic (i.e. /dev/mag0)
 *   attach  - A function which is called by this driver to attach the
 *             LIS2MDL interrupt handler to an IRQ. Pass NULL to operate
 *             in polling mode. This function should return 0 on success
 *             and a negated error code otherwise.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lis2mdl_register(FAR struct lis2mdl_config_s *config, int devno,
                     lis2mdl_attach attach);
