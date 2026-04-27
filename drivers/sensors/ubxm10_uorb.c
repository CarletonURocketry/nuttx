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

#include <nuttx/config.h>
#include <nuttx/nuttx.h>
#include <debug.h>

#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/sensors/gnss.h>
#include <nuttx/sensors/ioctl.h>

#include <nuttx/sensors/ubxm10.h>

#ifndef CONFIG_SENSORS_UBXM10_THREAD_STACKSIZE
#define CONFIG_SENSORS_UBXM10_THREAD_STACKSIZE UBXM10_THREAD_STACK_SIZE
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ubxm10_control(FAR struct gnss_lowerhalf_s *lower,
                          FAR struct file *filep, int cmd,
                          unsigned long arg);
static int ubxm10_activate(FAR struct gnss_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable);
static int ubxm10_set_interval(FAR struct gnss_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gnss_ops_s g_gnss_ops =
{
  .control      = ubxm10_control,
  .activate     = ubxm10_activate,
  .set_interval = ubxm10_set_interval,
};

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

#ifdef CONFIG_SERIAL_TERMIOS
/****************************************************************************
 * Name: ubxm10_set_host_baud
 *
 * Description:
 *   Updates the host UART baud rate via termios after the module has been
 *   told (via UBX-CFG-VALSET) to switch its own UART baud rate.
 ****************************************************************************/

static int ubxm10_set_host_baud(ubxm10_dev_s *dev, int baud)
{
    struct termios opt;
    int err;

    err = file_ioctl(&dev->uart, TCGETS, &opt);
    if (err < 0)
    {
        snwarn("Couldn't get interface settings: %d\n", err);
        return err;
    }

    cfmakeraw(&opt);

    switch (baud)
    {
        case 9600:
        case 19200:
        case 38400:
        case 57600:
        case 115200:
        case 230400:
        case 460800:
            cfsetispeed(&opt, baud);
            cfsetospeed(&opt, baud);
            break;

        default:
            snerr("Invalid baud rate: %d\n", baud);
            return -EINVAL;
    }

    err = file_ioctl(&dev->uart, TCSETS, &opt);
    if (err < 0)
    {
        snerr("Couldn't set host UART baud: %d\n", err);
    }

    return err;
}
#endif

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
            break;
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
            break;
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
            break;
        }

        case SNIOC_SET_INTERVAL:
        {

            /* UBX-CFG-VALSET Set Interval Payload */

            uint8_t payload[10] = { 
                0x00, 0x01, 0x00, 0x00, /* Message version, ram only, reserved, reserved. */
                0x01, 0x00, 0x21, 0x30, /* Key ID for CFG-RATE-MEAS in little endian. */
                (uint8_t)(arg & 0xFF), /* Value in ms low byte */
                (uint8_t)((arg >> 8) & 0xFF), /* Value in ms high byte */
            };

            frame_len = ubxm10_create_frame(&UBX_CFG_VALSET, payload, 10, frame);

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

        case SNIOC_SET_BAUD:
        {
            /* UBX-CFG-VALSET with CFG-UART1-BAUDRATE key (U4 value, 4 bytes) */
            uint8_t payload[12] = {
                0x00, 0x01, 0x00, 0x00, /* version, layers=RAM, reserved */
                0x01, 0x00, 0x52, 0x40, /* CFG-UART1-BAUDRATE key (0x40520001 LE) */
                (uint8_t)(arg & 0xFF),
                (uint8_t)((arg >> 8) & 0xFF),
                (uint8_t)((arg >> 16) & 0xFF),
                (uint8_t)((arg >> 24) & 0xFF),
            };

            frame_len = ubxm10_create_frame(&UBX_CFG_VALSET, payload, 12, frame);

            nxmutex_lock(&dev->lock);
            write_ret = file_write(&dev->uart, frame, frame_len);
            if (write_ret < 0)
            {
                nxmutex_unlock(&dev->lock);
                snerr("Failed to send SET_BAUD frame\n");
                return write_ret;
            }

            int baud_ack = ubxm10_wait_ack(dev, &UBX_CFG_VALSET);
            nxmutex_unlock(&dev->lock);

            if (baud_ack < 0)
            {
                return baud_ack;
            }

#ifdef CONFIG_SERIAL_TERMIOS
            /* Give module time to switch before changing our own UART */
            nxsig_usleep(20000);
            return ubxm10_set_host_baud(dev, (int)arg);
#else
            return -ENOSYS;
#endif
        }

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

/****************************************************************************
 * Name: ubxm10_activate
 *
 * Description:
 *   Enable or disable the GNSS module. When enabled, posts the run
 *   semaphore so the polling kthread starts reading from UART.
 ****************************************************************************/

static int ubxm10_activate(FAR struct gnss_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
    FAR ubxm10_dev_s *dev = container_of(lower, FAR ubxm10_dev_s, lower);

    if (enable && !dev->enabled)
    {
        nxsem_post(&dev->run);
        dev->enabled = true;
    }
    else if (!enable && dev->enabled)
    {
        dev->enabled = false;
    }

    return 0;
}

/****************************************************************************
 * Name: ubxm10_set_interval
 *
 * Description:
 *   Sets the measurement interval of the UBX M10 by sending a UBX-CFG-VALSET
 *   with the CFG-RATE-MEAS key. period_us is converted to ms.
 ****************************************************************************/

static int ubxm10_set_interval(FAR struct gnss_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us)
{
    FAR ubxm10_dev_s *dev = container_of(lower, FAR ubxm10_dev_s, lower);
    uint32_t period_ms = *period_us / 1000;
    uint8_t frame[UBX_PROTOCOL_BUFFER_MAX_LENGTH];
    int frame_len;
    int write_ret;
    int ack_ret;

    if (period_ms < 25 || period_ms > 65535)
    {
        return -EINVAL;
    }

    uint8_t payload[10] = {
        0x00, 0x01, 0x00, 0x00,        /* version, layers=RAM, reserved */
        0x01, 0x00, 0x21, 0x30,        /* CFG-RATE-MEAS key (little-endian) */
        (uint8_t)(period_ms & 0xFF),
        (uint8_t)((period_ms >> 8) & 0xFF),
    };

    frame_len = ubxm10_create_frame(&UBX_CFG_VALSET, payload, 10, frame);

    nxmutex_lock(&dev->lock);
    write_ret = file_write(&dev->uart, frame, frame_len);
    if (write_ret < 0)
    {
        nxmutex_unlock(&dev->lock);
        return write_ret;
    }

    ack_ret = ubxm10_wait_ack(dev, &UBX_CFG_VALSET);
    nxmutex_unlock(&dev->lock);
    return ack_ret;
}

/****************************************************************************
 * Name: ubxm10_thread
 *
 * Description:
 *   Kernel thread that polls the UART, then pushes raw bytes to the GNSS
 *   upper-half for NMEA parsing.
 ****************************************************************************/

static int ubxm10_thread(int argc, FAR char *argv[])
{
    FAR ubxm10_dev_s *dev =
        (FAR ubxm10_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
    ssize_t bw;
    int err;

    for (;;)
    {
        /* Wait until enabled by ubxm10_activate */
        if (!dev->enabled)
        {
            err = nxsem_wait(&dev->run);
            if (err < 0)
            {
                snerr("Couldn't wait on semaphore\n");
                continue;
            }
        }

        nxmutex_lock(&dev->lock);
        bw = file_read(&dev->uart, dev->buffer, sizeof(dev->buffer));

        if (bw <= 0)
        {
            snerr("No data on UART: %d\n", (int)bw);
            nxmutex_unlock(&dev->lock);
            continue;
        }

        /* Push raw bytes to GNSS upper-half for NMEA parsing */
        dev->lower.push_data(dev->lower.priv, dev->buffer, bw, true);

        nxmutex_unlock(&dev->lock);
    }

    return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubxm10_register
 *
 * Description:
 *   Register the UBX M10 GNSS driver.
 *
 * Arguments:
 *    uartpath  -  Path to the UART character driver connected to the module
 *    devno     -  Device number for the GNSS topic (e.g. /dev/uorb/sensor_gnss0)
 ****************************************************************************/

int ubxm10_register(FAR const char *uartpath, int devno)
{
    FAR ubxm10_dev_s *priv = NULL;
    int err;
    uint32_t nbuffers[SENSOR_GNSS_IDX_GNSS_MAX];
    FAR char *argv[2];
    char arg1[32];

    DEBUGASSERT(uartpath != NULL);

    priv = kmm_zalloc(sizeof(ubxm10_dev_s));
    if (priv == NULL)
    {
        snerr("Failed to allocate UBX M10 driver instance.\n");
        return -ENOMEM;
    }

    err = nxmutex_init(&priv->lock);
    if (err < 0)
    {
        snerr("Failed to initialize mutex: %d\n", err);
        goto free_mem;
    }

    err = nxsem_init(&priv->run, 0, 0);
    if (err < 0)
    {
        snerr("Failed to initialize semaphore: %d\n", err);
        goto destroy_mutex;
    }

    err = file_open(&priv->uart, uartpath, O_RDWR | O_CLOEXEC);
    if (err < 0)
    {
        snerr("Failed to open UART %s: %d\n", uartpath, err);
        goto destroy_sem;
    }

#ifdef CONFIG_SERIAL_TERMIOS
    /* Match the host UART to the module's default baud so reads/writes
     * work before any runtime SET_BAUD is issued.
     */
    err = ubxm10_set_host_baud(priv, UBXM10_BAUD_RATE);
    if (err < 0)
    {
        snwarn("Failed to set initial baud rate: %d\n", err);
    }
#endif

    priv->lower.ops = &g_gnss_ops;
    priv->lower.priv = priv;
    priv->enabled = false;

    nbuffers[SENSOR_GNSS_IDX_GNSS] = 2;
    nbuffers[SENSOR_GNSS_IDX_GNSS_SATELLITE] = 1;
    nbuffers[SENSOR_GNSS_IDX_GNSS_MEASUREMENT] = 1;
    nbuffers[SENSOR_GNSS_IDX_GNSS_CLOCK] = 1;
    nbuffers[SENSOR_GNSS_IDX_GNSS_GEOFENCE] = 1;

    err = gnss_register(&priv->lower, devno, nbuffers,
                        SENSOR_GNSS_IDX_GNSS_MAX);
    if (err < 0)
    {
        snerr("Failed to register GNSS driver: %d\n", err);
        goto close_file;
    }

    snprintf(arg1, sizeof(arg1), "%p", priv);
    argv[0] = arg1;
    argv[1] = NULL;

    err = kthread_create("ubxm10_thread", SCHED_PRIORITY_DEFAULT,
                         CONFIG_SENSORS_UBXM10_THREAD_STACKSIZE,
                         ubxm10_thread, argv);
    if (err < 0)
    {
        snerr("Failed to create ubxm10 kthread: %d\n", err);
        goto sensor_unreg;
    }

    sninfo("Registered UBX M10 driver on %s\n", uartpath);
    return 0;

sensor_unreg:
    gnss_unregister(&priv->lower, devno);
close_file:
    file_close(&priv->uart);
destroy_sem:
    nxsem_destroy(&priv->run);
destroy_mutex:
    nxmutex_destroy(&priv->lock);
free_mem:
    kmm_free(priv);

    return err;
}