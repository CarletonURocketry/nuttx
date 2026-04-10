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

/* Builds the UBX payload frame into out_frame and the total frame size.
 * out_frame must be at least (payload_len + 8) bytes.
 * Frame format: [sync1][sync2][class][id][len_lo][len_hi][payload...][ck_a][ck_b]
 */
int ubxm10_create_frame(const ubx_cmd_id_t *ubx_cmd_id, const uint8_t *payload, uint16_t payload_len, uint8_t *out_frame) {
    int i;
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    int frame_size = 6 + payload_len + 2;

    /* Sync bytes */
    out_frame[0] = UBX_PROTOCOL_SYNC_BYTE_1;
    out_frame[1] = UBX_PROTOCOL_SYNC_BYTE_2;

    /* Class and ID */
    out_frame[2] = ubx_cmd_id->cls;
    out_frame[3] = ubx_cmd_id->id;

    /* Payload length (little-endian) */
    out_frame[4] = (uint8_t)(payload_len & 0xFF);
    out_frame[5] = (uint8_t)((payload_len >> 8) & 0xFF);

    /* Copy payload */
    for (i = 0; i < payload_len; i++)
    {
        out_frame[6 + i] = payload[i];
    }

    /* 8-bit Fletcher checksum over class, id, length, and payload. Start at i=2 since sync bytes shouldnt be included in checksum. */
    for (i = 2; i < 6 + payload_len; i++)
    {
        ck_a += out_frame[i];
        ck_b += ck_a;
    }

    out_frame[6 + payload_len] = ck_a;
    out_frame[7 + payload_len] = ck_b;

    return frame_size;
}

static int ubxm10_control(FAR struct gnss_lowerhalf_s *lower, FAR struct file *filep, int cmd, unsigned long arg) {

    FAR ubxm10_dev_s *dev = container_of(lower, FAR ubxm10_dev_s, lower);
    uint8_t frame[UBX_PROTOCOL_BUFFER_MAX_LENGTH];
    int frame_len;

    switch(cmd) {

        case SNIOC_HOT_START:
        {
            /* Hot start payload - clear nothing */
            uint8_t payload[4] = { 0x00, 0x00, 0x00, 0x00};

            frame_len = ubxm10_create_frame(&UBX_CFG_RST, payload, 4, frame);

            /* UBX-CFG-RST does not guarantee an ACK, so just send it */
            return file_write(&dev->uart, frame, frame_len);
        }

        case SNIOC_WARM_START:
        {
            /* Warm start payload - clear ephemeris only (navBbrMask = 0x0001 little-endian) */
            uint8_t payload[4] = { 0x01, 0x00, 0x00, 0x00 };
            frame_len = ubxm10_create_frame(&UBX_CFG_RST, payload, 4, frame);

            /* UBX-CFG-RST does not guarantee an ACK, so just send it */
            return file_write(&dev->uart, frame, frame_len);
        }

        case SNIOC_COLD_START:
        {
            /* Cold start payload - clear everything (navBbrMask = 0xFFFF little-endian) */

            uint8_t payload[4] = { 0xFF, 0xFF, 0x00, 0x00} ;
            frame_len = ubxm10_create_frame(&UBX_CFG_RST, payload, 4, frame);

            /* UBX-CFG-RST does not guarantee an ACK, so just send it */
            return file_write(&dev->uart, frame, frame_len);
        }

        // case SNIOC_SET_INTERVAL:
        // case SNIOC_SET_BAUD:

        default:
            return -ENOTTY;
    }

    
}