/****************************************************************************
 * arch/arm/src/at32/at32_pm.h
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

#ifndef __ARCH_ARM_SRC_AT32_AT32_PM_H
#define __ARCH_ARM_SRC_AT32_AT32_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: at32_pmstop
 *
 * Description:
 *   Enter STOP mode.
 *
 * Input Parameters:
 *   lpds - true: To further reduce power consumption in Stop mode, put the
 *          internal voltage regulator in low-power mode using the LPDS bit
 *          of the Power control register (PWR_CR).
 *
 * Returned Value:
 *   Zero means that the STOP was successfully entered and the system has
 *   been re-awakened.  The internal voltage regulator is back to its
 *   original state.  Otherwise, STOP mode did not occur and a negated
 *   errno value is returned to indicate the cause of the failure.
 *
 ****************************************************************************/

int at32_pmstop(bool lpds);

/****************************************************************************
 * Name: at32_pmstandby
 *
 * Description:
 *   Enter STANDBY mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, this function will not return (STANDBY mode can only be
 *   terminated with a reset event).  Otherwise, STANDBY mode did not occur
 *   and a negated errno value is returned to indicate the cause of the
 *   failure.
 *
 ****************************************************************************/

int at32_pmstandby(void);

/****************************************************************************
 * Name: at32_pmsleep
 *
 * Description:
 *   Enter SLEEP mode.
 *
 * Input Parameters:
 *   sleeponexit - true:  SLEEPONEXIT bit is set when the WFI instruction is
 *                        executed, the MCU enters Sleep mode as soon as it
 *                        exits the lowest priority ISR.
 *               - false: SLEEPONEXIT bit is cleared, the MCU enters Sleep
 *                        mode as soon as WFI or WFE instruction is executed.
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void at32_pmsleep(bool sleeponexit);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_AT32_AT32_PM_H */
