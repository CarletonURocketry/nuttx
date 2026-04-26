/****************************************************************************
 * drivers/sensors/ubxm10.h
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

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/gnss.h>

#define UBXM10_BAUD_RATE 38400
#define UBXM10_THREAD_STACK_SIZE 10000

/* Depending on the start byte we decide which protocol we should be parsing. */
#define UBX_PROTOCOL_SYNC_BYTE_1 0xB5
#define UBX_PROTOCOL_SYNC_BYTE_2 0x62
#define NMEA_PROTOCOL_START_BYTE 0x24 /* '$' in Hex */

#define UBX_PROTOCOL_ACK_RETRY_COUNT 5
#define UBX_PROTOCOL_BUFFER_MAX_LENGTH 256

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct
{
    uint8_t cls;
    uint8_t id;
} ubx_cmd_id_t;

typedef struct
{
    FAR struct file uart;          /* UART interface to get data */
    struct gnss_lowerhalf_s lower; /* GNSS lower-half */
    bool enabled;                  /* Enabled state */
    char buffer[UBX_PROTOCOL_BUFFER_MAX_LENGTH]; /* UART read buffer */
    mutex_t lock;                  /* Device lock */
    sem_t run;                     /* Start/stop kthread */
} ubxm10_dev_s;

/****************************************************************************
 * Public Constants
 ****************************************************************************/

/* UBX Acknowledge Messages, outputs */
static const ubx_cmd_id_t UBX_ACK_ACK = { 0x05, 0x01 };
static const ubx_cmd_id_t UBX_ACK_NAK = { 0x05, 0x00 };

/* UBX Configuration Messages */
static const ubx_cmd_id_t UBX_CFG_RST = { 0x06, 0x04 }; /* Reset and power config */
static const ubx_cmd_id_t UBX_CFG_VALSET = { 0x06, 0x8A }; /* Set config value */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ubxm10_create_frame
 *
 * Description:
 *   Builds a UBX frame into out_frame and returns the total frame size.
 *   out_frame must be at least (payload_len + 8) bytes.
 ****************************************************************************/

int ubxm10_create_frame(const ubx_cmd_id_t *ubx_cmd_id,
                        const uint8_t *payload,
                        uint16_t payload_len,
                        uint8_t *out_frame);

/****************************************************************************
 * Name: ubxm10_register
 *
 * Description:
 *   Register the UBX M10 GNSS driver.
 *
 * Arguments:
 *    uartpath  -  Path to the UART character driver connected to the module
 *    devno     -  Device number for the GNSS topic
 ****************************************************************************/

int ubxm10_register(FAR const char *uartpath, int devno);

#endif /* __DRIVERS_GPS_UBX_M10_H */
