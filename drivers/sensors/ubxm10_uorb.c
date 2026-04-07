/****************************************************************************
 * drivers/sensors/ubxm10_uorb.c
 *
 * NOTE: EXPERIMENTAL DRIVER for the U-Blox M10 GNSS Chip
 *
 * Contributed by Carleton University InSpace
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

#include <nuttx/sensors/ubxm10.h>
#include <nuttx/sensors/ioctl.h>

static int ubxm10_control(FAR struct gnss_lowerhalf_s *lower, FAR struct file *filep, int cmd, unsigned long arg) {


    FAR ubxm10_dev_s *dev = container_of(lower, FAR ubxm10_dev_s, lower);
    const ubx_cmd_id_t *ubx_cmd_id;

    /* TEMPORARY UNTIL IMPL THE INTERVAL AND BAUD RATE COMMANDS. THOSE USE UBX_CFG_VALSET */
    uint8_t ubx_payload[4]; /* Init 4 byte payload. The only msg we are sending is UBX_CFG_RST for the power as of now and that takes only 4 bytes. */
    
    switch(cmd) {

        case SNIOC_HOT_START:
            ubx_cmd_id = &UBX_CFG_RST;

            /* Hot start payload - clear nothing */
            ubx_payload[0] = 0x00; /* navBbrMask low byte */
            ubx_payload[1] = 0x00; /* navBbrMask high byte */
            ubx_payload[2] = 0x00; /* resetMode */
            ubx_payload[3] = 0x00; /* reserved */

            break;

        case SNIOC_WARM_START:
            ubx_cmd_id = &UBX_CFG_RST;

            /* Warm start payload - clear ephemeris only */
            /* NOTE: Payload is in little-endian byte order, meaning low bytes are fed first then the high byte.
            For example the navBbrMask here actually comes out to 0x0001 but the low 0x01 is fed before the high 0x00. */
            ubx_payload[0] = 0x01; /* navBbrMask low byte */
            ubx_payload[1] = 0x00; /* navBbrMask high byte */
            ubx_payload[2] = 0x00; /* resetMode */
            ubx_payload[3] = 0x00; /* reserved */
            
            break;

        case SNIOC_COLD_START:
            ubx_cmd_id = &UBX_CFG_RST;

            /* Cold start payload - clear everything */
            ubx_payload[0] = 0xFF; /* navBbrMask low byte */
            ubx_payload[1] = 0xFF; /* navBbrMask high byte */
            ubx_payload[2] = 0x00; /* resetMode */
            ubx_payload[3] = 0x00; /* reserved */
            
            break;
        
        // case SNIOC_SET_INTERVAL:
            
        // case SNIOC_SET_BAUD:

    }


    /* With the UBX_CFG_RST commands, no ack is guaranteed so we just send it off and hope for the best */

    /* Note that we need to add the sync words, length, payload, checksum into one frame. */


    return 0;
}