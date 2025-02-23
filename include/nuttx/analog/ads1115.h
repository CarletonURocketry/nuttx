/****************************************************************************
 * include/nuttx/sensors/ads1115.h
 *
 * Contributed by Jia Lin
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

#ifndef __INCLUDE_NUTTX_ANALOG_ADS1115_H
#define __INCLUDE_NUTTX_ANALOG_ADS1115_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/analog/ioctl.h>
#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands
 *
 */

#define ANIOC_ADS1115_SET_OS _ANIOC(AN_ADS1115_FIRST + 0)
#define ANIOC_ADS1115_SET_PGA _ANIOC(AN_ADS1115_FIRST + 1)
#define ANIOC_ADS1115_SET_MODE _ANIOC(AN_ADS1115_FIRST + 2)
#define ANIOC_ADS1115_SET_DR _ANIOC(AN_ADS1115_FIRST + 3)
#define ANIOC_ADS1115_SET_COMP_MODE _ANIOC(AN_ADS1115_FIRST + 4)
#define ANIOC_ADS1115_SET_COMP_POL _ANIOC(AN_ADS1115_FIRST + 5)
#define ANIOC_ADS1115_SET_COMP_LAT _ANIOC(AN_ADS1115_FIRST + 6)
#define ANIOC_ADS1115_SET_COMP_QUEUE _ANIOC(AN_ADS1115_FIRST + 7)
#define ANIOC_ADS1115_READ_CHANNEL _ANIOC(AN_ADS1115_FIRST + 8)

/* Operational status / Single-shot conversion start*/

enum ads1115_os_e {
  ADS1115_OS1 = 0u, /* When Writing: No Effect*/
  ADS1115_OS2
};

/* Input Multiplexer configuration */

enum ads1115_mux_e {
  ADS1115_MUX1 = 0u, /* AINP = AIN0 and AINN = AIN1 (default) */
  ADS1115_MUX2,      /* AINP = AIN0 and AINN = AIN3 */
  ADS1115_MUX3,      /* AINP = AIN1 and AINN = AIN3 */
  ADS1115_MUX4,      /* AINP = AIN2 and AINN = AIN3 */
  ADS1115_MUX5,      /* AINP = AIN0 and AINN = GND */
  ADS1115_MUX6,      /* AINP = AIN1 and AINN = GND */
  ADS1115_MUX7,      /* AINP = AIN2 and AINN = GND */
  ADS1115_MUX8,      /* AINP = AIN3 and AINN = GND */
};

/* Programmable gain amplifier configuration */

enum ads1115_pga_e {
  ADS1115_PGA1 = 0u, /* FSR = ±6.144V */
  ADS1115_PGA2,      /* FSR = ±4.096V */
  ADS1115_PGA3,      /* FSR = ±2.048V (default) */
  ADS1115_PGA4,      /* FSR = ±1.024V */
  ADS1115_PGA5,      /* FSR = ±0.512V */
  ADS1115_PGA6,      /* FSR = ±0.256V */
  ADS1115_PGA7,      /* FSR = ±0.256V */
  ADS1115_PGA8,      /* FSR = ±0.256V */
};

/* Device Operating Mode */

enum ads1115_mode_e {
  ADS1115_MODE1 = 0u, /* Continuous conversion mode */
  ADS1115_MODE2       /* Power-down single-shot mode (default) */
};

/* Data rate */

enum ads1115_dr_e {
  ADS1115_DR1 = 0u, /* 8 SPS */
  ADS1115_DR2,      /* 16 SPS */
  ADS1115_DR3,      /* 32 SPS */
  ADS1115_DR4,      /* 64 SPS */
  ADS1115_DR5,      /* 128 SPS (default) */
  ADS1115_DR6,      /* 250 SPS */
  ADS1115_DR7,      /* 475 SPS */
  ADS1115_DR8,      /* 860 SPS */
};

/* Comparator mode */

enum adss1115_comp_mode_e {
  ADS1115_COMP_MODE1 = 0u, /* Traditional comparator (default) */
  ADS1115_COMP_MODE2       /* Window comparator */
};

/* Comparator Polarity */

enum ads1115_comp_pol_e {
  ADS1115_COMP_POL1 = 0u, /* Active low (default) */
  ADS1115_COMP_POL2       /* Active high */
};

/* Latching comparator */

enum ads1115_comp_lat_e {
  ADS1115_COMP_LAT1 = 0u, /* Nonlatching comparator. The ALERT/RDY pin does not
                              latch when asserted (default) */
  ADS1115_COMP_LAT2       /* Latching comparator. See datasheet.*/
};

/* Comparator queue and disable */

enum ads1115_comp_queue_e {
  ADS1115_COMP_QUEUE1 = 0u, /* Assert after one conversion */
  ADS1115_COMP_QUEUE2,      /* Assert after two conversions */
  ADS1115_COMP_QUEUE3,      /* Assert after four conversions */
  ADS1115_COMP_QUEUE4,      /* Disable comparator and set ALERT/RDY pin to
                                       high-impedance (default) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct adc_dev_s *ads1115_initialize(FAR struct i2c_master_s *i2c,
                                         uint8_t addr);

#endif /* __INCLUDE_NUTTX_ANALOG_ADS1115_H */
