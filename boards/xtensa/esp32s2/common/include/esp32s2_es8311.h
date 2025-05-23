/****************************************************************************
 * boards/xtensa/esp32s2/common/include/esp32s2_es8311.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
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

#ifndef __BOARDS_XTENSA_ESP32S2_COMMON_INCLUDE_ESP32S2_ES8311_H
#define __BOARDS_XTENSA_ESP32S2_COMMON_INCLUDE_ESP32S2_ES8311_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_es8311_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the ES8311 device. This function will register the driver
 *   as /dev/audio/pcm[x] where x is determined by the I2S port number.
 *
 * Input Parameters:
 *   i2c_port  - The I2C port used for the device;
 *   i2c_addr  - The I2C address used by the device;
 *   i2c_freq  - The I2C frequency used for the device.
 *
 * Returned Value:
 *   Zero is returned on success. Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int esp32s2_es8311_initialize(int i2c_port, uint8_t i2c_addr, int i2c_freq);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_XTENSA_ESP32S2_COMMON_INCLUDE_ESP32S2_ES8311_H */
