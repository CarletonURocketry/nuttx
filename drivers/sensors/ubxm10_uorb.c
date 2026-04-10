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

/****************************************************************************
 * Name: ubxm10_wait_ack
 *
 * Description:
 *   Reads from UART looking for a UBX-ACK-ACK or UBX-ACK-NAK response
 *   matching the class/id of the command that was sent.
 *
 * Returns:
 *   0          - ACK received
 *  -EIO        - NAK received
 *  -ETIMEDOUT  - No ACK/NAK after retries
 ****************************************************************************/

static int ubxm10_wait_ack(ubxm10_dev_s *dev, const ubx_cmd_id_t *sent_cmd)
{
    uint8_t byte;
    int retries;
    int index;
    uint8_t header[6];
    uint8_t ack_payload[2];
    uint8_t ck[2];
    int err;

    for (retries = 0; retries < UBX_PROTOCOL_ACK_RETRY_COUNT; retries++)
    {
        index = 0;

        /* Parse through bytes looking for a full UBX header */
        while (index < 6){
            err = file_read(&dev->uart, &byte, 1);
            if (err <= 0) {
                break;
            }

            switch (index) {
                case 0:
                    if (byte == UBX_PROTOCOL_SYNC_BYTE_1) {
                        header[0] = byte;
                        index = 1;
                    }

                    break;

                case 1:
                    if (byte == UBX_PROTOCOL_SYNC_BYTE_2) {
                        header[1] = byte;
                        index = 2;
                    } else {
                        index = 0;
                    }

                    break;

                default:
                    header[index] = byte;
                    index++;
                    break;
            }
        }

        if (index < 6) {
            continue;
        }

        /* Check if this is an ACK class */
        if (header[2] != UBX_ACK_ACK.cls) {
            continue;
        }

        /* Read the 2-byte payload (class and id being acknowledged) */
        err = file_read(&dev->uart, ack_payload, 2);

        if (err < 2) {
            continue;
        }

        /* Read and discard the 2 checksum bytes */
        file_read(&dev->uart, ck, 2);

        /* Check if this ACK/NAK is for the command we sent */
        if (ack_payload[0] != sent_cmd->cls ||
            ack_payload[1] != sent_cmd->id) {
            continue;
        }

        /* Match found - check if ACK or NAK */
        if (header[3] == UBX_ACK_ACK.id) {
            sninfo("UBX ACK received\n");

            return 0;
        } else if (header[3] == UBX_ACK_NAK.id) {
            snerr("UBX NAK received\n");

            return -EIO;
        }
    }

    snerr("Timed out waiting for UBX ACK\n");
    return -ETIMEDOUT;
}

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
    int write_ret;

    switch(cmd) {

        case SNIOC_HOT_START:
        {
            /* Hot start payload - clear nothing */
            uint8_t payload[4] = { 0x00, 0x00, 0x00, 0x00};

            frame_len = ubxm10_create_frame(&UBX_CFG_RST, payload, 4, frame);

            nxmutex_lock(&dev->lock);
            /* UBX-CFG-RST does not guarantee an ACK, so just send it */
            write_ret = file_write(&dev->uart, frame, frame_len);
            nxmutex_unlock(&dev->lock);
        }

        case SNIOC_WARM_START:
        {
            /* Warm start payload - clear ephemeris only (navBbrMask = 0x0001 little-endian) */
            uint8_t payload[4] = { 0x01, 0x00, 0x00, 0x00 };
            frame_len = ubxm10_create_frame(&UBX_CFG_RST, payload, 4, frame);

            nxmutex_lock(&dev->lock);
            /* UBX-CFG-RST does not guarantee an ACK, so just send it */
            write_ret = file_write(&dev->uart, frame, frame_len);
            nxmutex_unlock(&dev->lock);
        }

        case SNIOC_COLD_START:
        {
            /* Cold start payload - clear everything (navBbrMask = 0xFFFF little-endian) */

            uint8_t payload[4] = { 0xFF, 0xFF, 0x00, 0x00} ;
            frame_len = ubxm10_create_frame(&UBX_CFG_RST, payload, 4, frame);

            nxmutex_lock(&dev->lock);
            /* UBX-CFG-RST does not guarantee an ACK, so just send it */
            write_ret = file_write(&dev->uart, frame, frame_len);
            nxmutex_unlock(&dev->lock);
        }

        case SNIOC_SET_INTERVAL:
        {

            /* UBX-CFG-VALSET Set Interval Payload */

            uint8_t payload[9] = { 
                0x00, 0x01, 0x00, /* Message version, ram only, reserved. */
                0x01, 0x00, 0x21, 0x30, /* Key ID for CFG-RATE-MEAS in little endian. */
                (uint8_t)(arg & 0xFF), /* Value in ms low byte */
                (uint8_t)((arg >> 8) & 0xFF), /* Value in ms high byte */
            };

            frame_len = ubxm10_create_frame(&UBX_CFG_VALSET, payload, 9, frame);

            nxmutex_lock(&dev->lock);

            write_ret = file_write(&dev->uart, frame, frame_len);

            if (write_ret < 0) {
                nxmutex_unlock(&dev->lock);
                snerr("Failed to send SET_INTERVAL frame\n");
                return write_ret;
            }

            int ack_ret = ubxm10_wait_ack(dev, &UBX_CFG_VALSET);
            nxmutex_unlock(&dev->lock);

            return ack_ret;
        }

        // case SNIOC_SET_BAUD:

        default:
            return -ENOTTY;
    }

    if (write_ret < 0)
    {
        snerr("Failed to send command frame to device\n");
        return write_ret;
    }

    return 0;

    
}