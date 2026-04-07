/****************************************************************************
 * drivers/sensors/ubm10.h
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

#ifndef __DRIVERS_GPS_UBX_M10_H
#define __DRIVERS_GPS_UBX_M10_H

#include <nuttx/config.h>
#include <nuttx/sensors/gnss.h>

#define UBM10_BAUD_RATE 38400
#define UBM10_THREAD_STACK_SIZE 10000

/* Depending on the start byte we decide which protocol we should be parsing. */
#define UBX_PROTOCOL_SYNC_BYTE_1 0xB5
#define UBX_PROTOCOL_SYNC_BYTE_2 0x62
#define NMEA_PROTOCOL_START_BYTE 0x24 /* '$' in Hex */

#define UBX_PROTOCOL_ACK_RETRY_COUNT 5
#define MINMEA_MAX_LENGTH 256


typedef struct {
    uint8_t cls;
    uint8_t id;
} ubx_msg_id;

/* UBX Acknowledge Messages, outputs */
static const ubx_msg_id UBX_ACK_ACK = { 0x5, 0x01 };
static const ubx_msg_id UBX_ACK_NAK = { 0x5, 0x00 };

/* UBX Configuration Messages*/


/* Need to figure out how to send to uorb */

/* Private functions or something like that */
/* Init module */
/* Send command */
/* Parse response */

/* Public functions */
/* Register module */

#endif