/****************************************************************************
 * drivers/wireless/lpwan/rn2483/rn2483.c
 *
 * Contributed by Carleton University InSpace
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <nuttx/wireless/ioctl.h>
#include <nuttx/wireless/lpwan/rn2483.h>
#include <nuttx/wireless/lpwan/sx127x.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Frequency for communication in Hz */

#ifndef CONFIG_LPWAN_RN2483_FREQ
#define CONFIG_LPWAN_RN2483_FREQ 433050000
#endif /* CONFIG_LPWAN_RN2483_FREQ */

#if (CONFIG_LPWAN_RN2483_FREQ < 433000000 ||                                 \
     (CONFIG_LPWAN_RN2483_FREQ > 434800000 &&                                \
      CONFIG_LPWAN_RN2483_FREQ < 863000000) ||                               \
     CONFIG_LPWAN_RN2483_FREQ > 870000000)
#error                                                                       \
    "RN2483 frequency has to be between 433000000 and 434800000, or between 863000000 and 870000000, in Hz"
#endif /* Bandwidth value check */

/* Sync word */

#ifndef CONFIG_LPWAN_RN2483_SYNC
#define CONFIG_LPWAN_RN2483_SYNC 0x43
#endif /* CONFIG_LPWAN_RN2483_SYNC */

/* Preamble length */

#ifndef CONFIG_LPWAN_RN2483_PRLEN
#define CONFIG_LPWAN_RN2483_PRLEN 6
#endif /* CONFIG_LPWAN_RN2483_PRLEN */

/* Bandwidth in kHz */

#ifndef CONFIG_LPWAN_RN2483_BW
#define CONFIG_LPWAN_RN2483_BW 500
#endif /* CONFIG_LPWAN_RN2483_BW */

#if (CONFIG_LPWAN_RN2483_BW != 125 && CONFIG_LPWAN_RN2483_BW != 250 &&       \
     CONFIG_LPWAN_RN2483_BW != 500)
#error "RN2483 bandwidth can only be one of 125, 250 and 500kHz"
#endif /* Bandwidth value check */

/* Spread factor */

#ifndef CONFIG_LPWAN_RN2483_SPREAD
#define CONFIG_LPWAN_RN2483_SPREAD 7
#endif /* CONFIG_LPWAN_RN2483_SPREAD */

#if !(CONFIG_LPWAN_RN2483_SPREAD <= 12 && CONFIG_LPWAN_RN2483_SPREAD >= 7)
#error "RN2483 spread factor must be between 7 and 12, inclusive."
#endif /* Power value check */

/* Modulation in LoRA mode. If not LoRa, FSK is selected. */

#ifndef CONFIG_LPWAN_RN2483_LORA
#define CONFIG_LPWAN_RN2483_LORA 1
#endif /* CONFIG_LPWAN_RN2483_LORA */

/* Transmit output power. TODO: what unit? */

#ifndef CONFIG_LPWAN_RN2483_POWER
#define CONFIG_LPWAN_RN2483_POWER 15
#endif /* CONFIG_LPWAN_RN2483_POWER */

#if !(CONFIG_LPWAN_RN2483_POWER <= 15 && CONFIG_LPWAN_RN2483_POWER >= -3)
#error "RN2483 power must be between -3 and 15, inclusive."
#endif /* Power value check */

/* Coding rate */

#ifndef CONFIG_LPWAN_RN2483_CR
#define CONFIG_LPWAN_RN2483_CR 7
#endif /* CONFIG_LPWAN_RN2483_CR */

#if !(CONFIG_LPWAN_RN2483_CR <= 8 && CONFIG_LPWAN_RN2483_CR >= 5)
#error "RN2483 coding rate must be between 5 and 8, inclusive."
#endif /* Power value check */

/* IQ invert */

#ifndef CONFIG_LPWAN_RN2483_IQI
#define CONFIG_LPWAN_RN2483_IQI 0
#endif /* CONFIG_LPWAN_RN2483_IQI */

/* Cyclic redundancy check */

#ifndef CONFIG_LPWAN_RN2483_CRC
#define CONFIG_LPWAN_RN2483_CRC 1
#endif /* CONFIG_LPWAN_RN2483_CRC */

/****************************************************************************
 * Private
 ****************************************************************************/

/* Contains configuration parameters for RN2483. */

struct rn2483_config_s
{
  enum rn2483_cr_e cr;   /* Coding rate */
  enum rn2483_mod_e mod; /* Modulation, either FSK or LoRa */
  uint32_t freq;         /* TX/RX frequency in Hz */
  uint16_t bw;           /* Bandwidth in kHz */
  uint16_t prlen;        /* Preamble length */
  bool crc;              /* Cyclic redundancy check, true for enabled */
  bool iqi;              /* IQ invert, true for enabled */
  int8_t pwr;            /* Transmit output power */
  uint8_t sf;            /* Spread factor */
  uint8_t sync;          /* Synchronization word */
};

/* Represents the RN2483 device */

struct rn2483_dev_s
{
  FAR struct file uart; /* Underlying UART interface */
  bool receiving; /* Mark radio as receiving between read calls in case buffer
                     was too small */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked;                 /* True means driver is unlinked */
#endif                           /* CONFIG_DISABLE_PSEUDOFS_OPERATIONS */
  struct rn2483_config_s config; /* Radio parameters */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  int16_t crefs; /* Number of open references. */
#endif           // CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  mutex_t devlock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

// TODO: add granularity for enabling/disabling TX/RX support separately

static int rn2483_open(FAR struct file *filep);
static int rn2483_close(FAR struct file *filep);
static ssize_t rn2483_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static ssize_t rn2483_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static int rn2483_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int rn2483_unlink(FAR struct inode *inode);

static int rn2483_radio_set_cr(FAR struct rn2483_dev_s *priv,
                               enum rn2483_cr_e cr);
static int rn2483_radio_set_mod(FAR struct rn2483_dev_s *priv,
                                enum rn2483_mod_e mod);
static int rn2483_radio_set_freq(FAR struct rn2483_dev_s *priv,
                                 uint32_t freq);
static int rn2483_radio_set_bw(FAR struct rn2483_dev_s *priv, uint16_t bw);
static int rn2483_radio_set_prlen(FAR struct rn2483_dev_s *priv,
                                  uint16_t prlen);
static int rn2483_radio_set_crc(FAR struct rn2483_dev_s *priv, bool crc);
static int rn2483_radio_set_iqi(FAR struct rn2483_dev_s *priv, bool iqi);
static int rn2483_radio_set_pwr(FAR struct rn2483_dev_s *priv, int8_t pwr);
static int rn2483_radio_set_sf(FAR struct rn2483_dev_s *priv, uint8_t sf);
static int rn2483_radio_set_sync(FAR struct rn2483_dev_s *priv, uint8_t sync);
static int rn2483_read_response(FAR struct rn2483_dev_s *priv, char *buf,
                                size_t buf_len);
static int rn2483_set_config(FAR struct rn2483_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_rn2483fops = {
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    .open = rn2483_open,
    .close = rn2483_close,
#else
    .open = NULL,
    .close = NULL,
#endif
    .read = rn2483_read,
    .write = rn2483_write,
    .seek = NULL,
    .ioctl = rn2483_ioctl,
    .mmap = NULL,
    .truncate = NULL,
    .poll = NULL,
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    .unlink = rn2483_unlink,
#endif
};

/* String representation of modulation */

static const char *MODULATIONS[] = {
    [RN2483_MOD_FSK] = "fsk",
    [RN2483_MOD_LORA] = "lora",
};

/* String representation of coding rates */

static const char *CODING_RATES[] = {
    [RN2483_CR_4_5] = "4/5",
    [RN2483_CR_4_6] = "4/6",
    [RN2483_CR_4_7] = "4/7",
    [RN2483_CR_4_8] = "4/8",
};

#define SUCCESS "SUCCESS"
#define FAILURE "FAILURE"

/* Each buffer will be one more than needed to include null terminating
 * character if we want */

#define READ_BUF_LEN 17
#define CR_WRITE_BUF_LEN 19
#define MOD_WRITE_BUF_LEN 21
#define FREQ_WRITE_BUF_LEN 27
#define BW_WRITE_BUF_LEN 19
#define PRLEN_WRITE_BUF_LEN 24
#define CRC_WRITE_BUF_LEN 20
#define IQI_WRITE_BUF_LEN 20
#define PWR_WRITE_BUF_LEN 19
#define SF_WRITE_BUF_LEN 19
#define SYNC_WRITE_BUF_LEN 34
#define DUMMY_READ_BUFFER_LEN 36

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rn2483_open
 *
 * Description:
 *   This function is called whenever the RN2483 device is opened.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rn2483_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rn2483_dev_s *priv = inode->i_private;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  nxmutex_unlock(&priv->devlock);
  return 0;
}
#endif

/****************************************************************************
 * Name: rn2483_close
 *
 * Description:
 *   This routine is called when the RN2483 device is closed.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rn2483_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rn2483_dev_s *priv = inode->i_private;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If the count has decremented to zero and the driver has been unlinked,
   * then free memory now.
   */

  if (priv->crefs <= 0 && priv->unlinked)
    {
      file_close(&priv->uart);
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return 0;
    }

  nxmutex_unlock(&priv->devlock);
  return 0;
}
#endif

/****************************************************************************
 * Name: rn2483_read
 *
 * Description:
 *     Character driver interface to RN2483 for receiving data.
 *
 ****************************************************************************/

static ssize_t rn2483_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rn2483_dev_s *priv = inode->i_private;
  ssize_t length = 0;
  ssize_t received;
  int err;
  char response[30] = {0}; // TODO: shortest length?

  /* If file position is non-zero but we're marked as no longer receiving,
   * then the whole transmission has been consumed. Set a filepos of 0 to
   * signal EOF.
   */

  if (priv->receiving == false && filep->f_pos > 0)
    {
      filep->f_pos = 0;
      return 0;
    }

  /* We're setting a new receive call, mark as receiving */

  priv->receiving = true;

  /* Get exclusive access */

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      priv->receiving = false;
      return err;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Do not allow operations on unlinked drivers. */

      priv->receiving = false;
      nxmutex_unlock(&priv->devlock);
      return 0;
    }
#endif

  /* Must send 'mac pause' command before every receive command */

  char pause[] = "mac pause\r\n";
  length = file_write(&priv->uart, pause, sizeof(pause));
  if (length < 0)
    {
      priv->receiving = false;
      nxmutex_unlock(&priv->devlock);
      return length;
    }

  /* Check how the radio responded to MAC pause */

  length = rn2483_read_response(priv, response, sizeof(response) - 1);
  if (length < 0)
    {
      priv->receiving = false;
      nxmutex_unlock(&priv->devlock);
      return length;
    }

  /* We should be pausing for the max duration */

  if (!strstr(response, "4294967245"))
    {
      priv->receiving = false;
      nxmutex_unlock(&priv->devlock);
      return -EIO;
    }

  /* Get into continuous receive */

  char rx[] = "radio rx 0\r\n";
  length = file_write(&priv->uart, rx, sizeof(rx));
  if (length < 0)
    {
      priv->receiving = false;
      nxmutex_unlock(&priv->devlock);
      return length;
    }

  /* Check how the radio responded to entering RX mode */

  length = rn2483_read_response(priv, response, sizeof(response) - 1);
  if (length < 0)
    {
      priv->receiving = false;
      nxmutex_unlock(&priv->devlock);
      return length;
    }

  if (strstr(response, "invalid_param"))
    {
      priv->receiving = false;
      nxmutex_unlock(&priv->devlock);
      return -EINVAL;
    }
  else if (strstr(response, "busy"))
    {
      priv->receiving = false;
      nxmutex_unlock(&priv->devlock);
      return -EAGAIN;
    }
  else if (!strstr(response, "ok"))
    {
      /* This means our program and radio are out of sync */

      priv->receiving = false;
      nxmutex_unlock(&priv->devlock);
      return -EIO;
    }

  /* If we are, we're successfully in RX mode. This means another UART read
   * will block until we actually receive something or there was a receive
   * error.
   * We'll receive our data in ASCII hexadecimal characters, which we need to
   * convert to binary to return to the user.
   * We'll read enough characters to see if the message starts with
   * radio_rx or radio_err. If there was an error we'll return and inform the
   * caller. If there was something received, we'll read in groups of two
   * characters (one byte in ASCII hex) and convert the ASCII byte into a real
   * one to put in the user buffer. Once we see a newline or we reach the end
   * of the user buffer we'll stop.
   */

  /* Read transmission */

  memset(response, 0, sizeof(response));

  /* 9 is the length of either 'radio_err' or 'radio_tx ' (with trailing
   * space) */

  for (uint8_t pos = 0; pos < 10; pos++)
    {
      length = file_read(&priv->uart, &response[pos], 1);
      if (length < 0)
        {
          priv->receiving = false;
          nxmutex_unlock(&priv->devlock);
          return length;
        }
    }

  /* Check if we got radio_err, radio_tx or neither */

  if (strstr(response, "radio_err"))
    {
      priv->receiving = false;
      nxmutex_unlock(&priv->devlock);
      return -EIO;
    }
  else if (!strstr(response, "radio_rx "))
    {
      /* The driver is out of sync with the radio */

      priv->receiving = false;
      nxmutex_unlock(&priv->devlock);
      return -EIO;
    }

  /* If we're here, we were able to detect `radio_tx `
   * Now we convert each byte from ASCII to binary in the user's buffer.
   */

  char ascii_byte[3] = {0}; /* Space for null terminator for strtoul */
  received = 0;

  while (received < buflen)
    {

      /* We should be guaranteed to read in multiples of two because we cannot
       * receive half-bytes, and the terminating sequence is \r\n.
       */

      for (uint8_t i = 0; i < 2; i++)
        {
          length = file_read(&priv->uart, &ascii_byte[i], 1);
          if (length < 0)
            {
              priv->receiving = false;
              nxmutex_unlock(&priv->devlock);
              return length;
            }
        }

      /* Check if we got terminating characters that terminate receive */

      if (ascii_byte[0] == '\r' && ascii_byte[1] == '\n')
        {
          priv->receiving = false;
          break;
        }

      /* Convert to binary */

      buffer[received] = (uint8_t)strtoul(ascii_byte, NULL, 16);
      received++;
    }

  filep->f_pos += received;
  nxmutex_unlock(&priv->devlock);
  return received;
}

/****************************************************************************
 * Name: rn2483_write
 *
 * Description:
 *     Write interface for transmitting over the RN2483.
 ****************************************************************************/

static ssize_t rn2483_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{ // // TODO actually transmit

  FAR struct inode *inode = filep->f_inode;
  FAR struct rn2483_dev_s *priv = inode->i_private;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  return -ENOSYS;
}

/****************************************************************************
 * Name: rn2483_ioctl
 ****************************************************************************/

static int rn2483_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rn2483_dev_s *priv = inode->i_private;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Do not allow operations on unlinked drivers. */

      nxmutex_unlock(&priv->devlock);
      return -ENODEV;
    }
#endif

  /* Handle command */
  switch (cmd)
    {

    /*  Set Radio modulation */
    case WLIOC_SETRADIOMOD:
      {
        enum rn2483_mod_e *ptr = (enum rn2483_mod_e *)(arg);
        if (*ptr != RN2483_MOD_FSK && *ptr != RN2483_MOD_LORA)
          {
            err = -EINVAL;
            break;
          }

        err = rn2483_radio_set_mod(priv, *ptr);
        if (!err)
          {
            priv->config.mod = *ptr;
          }
        break;
      }
    /*  Get Radio modulation */
    case WLIOC_GETRADIOMOD:
      {
        *((enum rn2483_mod_e *)(arg)) =
            priv->config.mod; // Returns modulation in pointer
        break;
      }

    /*  Set Radio frequency */
    case WLIOC_SETRADIOFREQ:
      {
        uint32_t *ptr = (uint32_t *)(arg);
        if ((*ptr < 433000000 || (*ptr > 434800000 && *ptr < 863000000) ||
             *ptr > 870000000))
          {
            err = -EINVAL;
            break;
          }

        err = rn2483_radio_set_freq(priv, *ptr);
        if (!err)
          {
            priv->config.freq = *ptr;
          }
        break;
      }
    /*  Get Radio frequency */
    case WLIOC_GETRADIOFREQ:
      {
        *((uint32_t *)(arg)) =
            priv->config.freq; // Returns frequency in pointer
        break;
      }

    /*  Set Radio power */
    case WLIOC_SETRADIOPWR:
      {
        int8_t *ptr = (int8_t *)(arg);
        if (*ptr <= 15 && *ptr >= -3)
          {
            err = -EINVAL;
            break;
          }

        err = rn2483_radio_set_pwr(priv, *ptr);
        if (!err)
          {
            priv->config.pwr = *ptr;
          }
        break;
      }
    /*  Get Radio power */
    case WLIOC_GETRADIOPWR:
      {
        *((int8_t *)(arg)) = priv->config.pwr; // Returns power in pointer
        break;
      }

    /*  Set Radio spread factor */
    case WLIOC_SETRADIOSF:
      {
        uint8_t *ptr = (uint8_t *)(arg);
        if (*ptr <= 12 && *ptr >= 7)
          {
            err = -EINVAL;
            break;
          }

        err = rn2483_radio_set_sf(priv, *ptr);
        if (!err)
          {
            priv->config.sf = *ptr;
          }
        break;
      }

    /*  Get Radio spread factor */
    case WLIOC_GETRADIOSF:
      {
        *((uint8_t *)(arg)) =
            priv->config.sf; // Returns spread factor in pointer
        break;
      }

    /*  Set Radio preamble length */
    case WLIOC_SETRADIOPRLEN:
      {
        uint16_t *ptr = (uint16_t *)(arg);
        if (*ptr < 0 || *ptr > 65535)
          {
            err = -EINVAL;
            break;
          }

        err = rn2483_radio_set_prlen(priv, *ptr);
        if (!err)
          {
            priv->config.prlen = *ptr;
          }
        break;
      }

    /*  Get Radio preamble length */
    case WLIOC_GETRADIOPRLEN:
      {
        *((uint16_t *)(arg)) =
            priv->config.prlen; // Returns preamble length in pointer
        break;
      }

    /*  Set Radio CRC Header */
    case WLIOC_SETRADIOCRC:
      {
        bool *ptr = (bool *)(arg);

        err = rn2483_radio_set_crc(priv, *ptr);
        if (!err)
          {
            priv->config.crc = *ptr;
          }
        break;
      }
    /*  Get Radio CRC Header */
    case WLIOC_GETRADIOCRC:
      {
        *((bool *)(arg)) = priv->config.crc; // Returns CRC Header in pointer
        break;
      }

    /*  Set Radio Invert IQ */
    case WLIOC_SETRADIOIQI:
      {
        bool *ptr = (bool *)(arg);

        err = rn2483_radio_set_iqi(priv, *ptr);
        if (!err)
          {
            priv->config.iqi = *ptr;
          }
        break;
      }
    /*  Get Radio Invert IQ */
    case WLIOC_GETRADIOIQI:
      {
        *((bool *)(arg)) = priv->config.iqi; // Returns Invert IQ in pointer
        break;
      }

    /*  Set Radio Coding Rate */
    case WLIOC_SETRADIOCR:
      {
        enum rn2483_cr_e *ptr = (enum rn2483_cr_e *)(arg);
        if (*ptr <= 8 && *ptr >= 5)
          {
            err = -EINVAL;
            break;
          }

        err = rn2483_radio_set_cr(priv, *ptr);
        if (!err)
          {
            priv->config.cr = *ptr;
          }
        break;
      }
    /*  Get Radio Coding Rate */
    case WLIOC_GETRADIOCR:
      {
        *((enum rn2483_cr_e *)(arg)) =
            priv->config.cr; // Returns Coding Rate in pointer
        break;
      }

    /*  Set Radio sync word */
    case WLIOC_SETRADIOSYNC:
      {
        uint8_t *ptr = (uint8_t *)(arg);

        err = rn2483_radio_set_sync(priv, *ptr);
        if (!err)
          {
            priv->config.sync = *ptr;
          }
        break;
      }
    /*  Get Radio sync word */
    case WLIOC_GETRADIOSYNC:
      {
        *((uint8_t *)(arg)) =
            priv->config.sync; // Returns sync word in pointer
        break;
      }

    /*  Set Radio bandwidth */
    case WLIOC_SETRADIOBW:
      {
        uint16_t *ptr = (uint16_t *)(arg);
        if (*ptr != 125 && *ptr != 250 && *ptr != 500)
          {
            err = -EINVAL;
            break;
          }

        err = rn2483_radio_set_bw(priv, *ptr);
        if (!err)
          {
            priv->config.bw = *ptr;
          }
        break;
      }
    /*  Get Radio bandwidth */
    case WLIOC_GETRADIOBW:
      {
        *((uint16_t *)(arg)) =
            priv->config.bw; // Returns bandwidth in pointer
        break;
      }

    default:
      {
        /*If we got here than an invalid command was entered*/
        err = -EINVAL;
        break;
      }
    }

  nxmutex_unlock(&priv->devlock);
  return err;
}

/****************************************************************************
 * Name: rn2483_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rn2483_unlink(FAR struct inode *inode)
{
  FAR struct rn2483_dev_s *priv;
  int err;

  DEBUGASSERT(inode->i_private != NULL);
  priv = inode->i_private;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Are there open references to the driver data structure? */

  if (priv->crefs <= 0)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return 0;
    }

  /* No. Just mark the driver as unlinked and free the resources when
   * the last client closes their reference to the driver.
   */

  priv->unlinked = true;
  nxmutex_unlock(&priv->devlock);
  return OK;
}
#endif

/****************************************************************************
 * Name: rn2483_read_response
 ****************************************************************************/

static int rn2483_read_response(FAR struct rn2483_dev_s *priv, char *buf,
                                size_t buf_len)
{
  size_t read = 0;
  ssize_t fread;
  char last = '\0';
  while (read < buf_len)
    {
      fread = file_read(&priv->uart, &buf[read], 1);
      if (last == '\r' && buf[read] == '\n')
        {
          break;
        }
      last = buf[read];

      if (fread < 0) return fread;

      read++;
    }

  return fread;
}

/****************************************************************************
 * Name: rn2483_check_response
 ****************************************************************************/

static int rn2483_check_response(char *read_buffer)
{
  if (strstr(read_buffer, "ok") != NULL)
    {
      return 0;
    }
  else if (strstr(read_buffer, "invalid_param") != NULL)
    {
      return -EINVAL;
    }
  else
    {
      return -EINVAL;
    }
}

/****************************************************************************
 * Name: rn2483_set_cr
 ****************************************************************************/

static int rn2483_radio_set_cr(FAR struct rn2483_dev_s *priv,
                               enum rn2483_cr_e cr)
{
  char write_buffer[CR_WRITE_BUF_LEN];
  char read_buffer[READ_BUF_LEN] = {0};
  ssize_t err;
  ssize_t written;

  written = snprintf(write_buffer, sizeof(write_buffer),
                     "radio set cr %s\r\n", CODING_RATES[cr]);

  err = file_write(&priv->uart, write_buffer, written);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_read_response(priv, read_buffer, sizeof(read_buffer) - 1);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_check_response(read_buffer);
  return err;
}

static int rn2483_radio_set_mod(FAR struct rn2483_dev_s *priv,
                                enum rn2483_mod_e mod)
{
  char write_buffer[MOD_WRITE_BUF_LEN];
  char read_buffer[READ_BUF_LEN] = {0};
  ssize_t err;
  ssize_t written;

  written = snprintf(write_buffer, sizeof(write_buffer),
                     "radio set mod %s\r\n", MODULATIONS[mod]);

  err = file_write(&priv->uart, write_buffer, written);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_read_response(priv, read_buffer, sizeof(read_buffer) - 1);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_check_response(read_buffer);
  return err;
}

static int rn2483_radio_set_freq(FAR struct rn2483_dev_s *priv, uint32_t freq)
{

  char write_buffer[FREQ_WRITE_BUF_LEN];
  char read_buffer[READ_BUF_LEN] = {0};
  ssize_t err;
  ssize_t written;

  written = snprintf(write_buffer, sizeof(write_buffer),
                     "radio set freq %ld\r\n", freq);

  err = file_write(&priv->uart, write_buffer, written);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_read_response(priv, read_buffer, sizeof(read_buffer) - 1);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_check_response(read_buffer);
  return err;
}

static int rn2483_radio_set_bw(FAR struct rn2483_dev_s *priv, uint16_t bw)
{

  char write_buffer[BW_WRITE_BUF_LEN];
  char read_buffer[READ_BUF_LEN] = {0};
  ssize_t err;
  ssize_t written;

  written =
      snprintf(write_buffer, sizeof(write_buffer), "radio set bw %d\r\n", bw);

  err = file_write(&priv->uart, write_buffer, written);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_read_response(priv, read_buffer, sizeof(read_buffer) - 1);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_check_response(read_buffer);
  return err;
}

static int rn2483_radio_set_prlen(FAR struct rn2483_dev_s *priv,
                                  uint16_t prlen)
{
  char write_buffer[PRLEN_WRITE_BUF_LEN];
  char read_buffer[READ_BUF_LEN] = {0};
  ssize_t err;
  ssize_t written;

  written = snprintf(write_buffer, sizeof(write_buffer),
                     "radio set prlen %d\r\n", prlen);

  err = file_write(&priv->uart, write_buffer, written);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_read_response(priv, read_buffer, sizeof(read_buffer) - 1);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_check_response(read_buffer);
  return err;
}

static int rn2483_radio_set_crc(FAR struct rn2483_dev_s *priv, bool crc)
{

  char write_buffer[CRC_WRITE_BUF_LEN];
  char read_buffer[READ_BUF_LEN] = {0};
  char *data = (crc) ? "on" : "off";
  ssize_t err;
  ssize_t written;

  written = snprintf(write_buffer, sizeof(write_buffer),
                     "radio set crc %s\r\n", data);

  err = file_write(&priv->uart, write_buffer, written);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_read_response(priv, read_buffer, sizeof(read_buffer) - 1);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_check_response(read_buffer);
  return err;
}

static int rn2483_radio_set_iqi(FAR struct rn2483_dev_s *priv, bool iqi)
{

  char write_buffer[IQI_WRITE_BUF_LEN];
  char read_buffer[READ_BUF_LEN] = {0};
  char *data = (iqi) ? "on" : "off";
  ssize_t err;
  ssize_t written;

  written = snprintf(write_buffer, sizeof(write_buffer),
                     "radio set iqi %s\r\n", data);

  err = file_write(&priv->uart, write_buffer, written);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_read_response(priv, read_buffer, sizeof(read_buffer) - 1);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_check_response(read_buffer);
  return err;
}

static int rn2483_radio_set_pwr(FAR struct rn2483_dev_s *priv, int8_t pwr)
{

  char write_buffer[PWR_WRITE_BUF_LEN];
  char read_buffer[READ_BUF_LEN] = {0};
  ssize_t err;
  ssize_t written;

  written = snprintf(write_buffer, sizeof(write_buffer),
                     "radio set pwr %d\r\n", pwr);

  err = file_write(&priv->uart, write_buffer, written);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_read_response(priv, read_buffer, sizeof(read_buffer) - 1);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_check_response(read_buffer);
  return err;
}

static int rn2483_radio_set_sf(FAR struct rn2483_dev_s *priv, uint8_t sf)
{

  char write_buffer[SF_WRITE_BUF_LEN];
  char read_buffer[READ_BUF_LEN] = {0};
  ssize_t err;
  ssize_t written;

  written = snprintf(write_buffer, sizeof(write_buffer),
                     "radio set sf sf%d\r\n", sf);

  err = file_write(&priv->uart, write_buffer, written);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_read_response(priv, read_buffer, sizeof(read_buffer) - 1);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_check_response(read_buffer);
  return err;
}

static int rn2483_radio_set_sync(FAR struct rn2483_dev_s *priv, uint8_t sync)
{

  char read_buffer[READ_BUF_LEN] = {0};
  char write_buffer[SYNC_WRITE_BUF_LEN];
  ssize_t err;
  ssize_t written;

  written = snprintf(write_buffer, sizeof(write_buffer),
                     "radio set sync %d\r\n", sync);

  err = file_write(&priv->uart, write_buffer, written);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_read_response(priv, read_buffer, sizeof(read_buffer) - 1);
  if (err < 0)
    {
      return err;
    }

  err = rn2483_check_response(read_buffer);
  return err;
}

static int rn2483_set_config(FAR struct rn2483_dev_s *priv)
{
  int err = 0;

  /* Set modulation; FSK or LoRa */

  err = rn2483_radio_set_mod(priv, priv->config.mod);
  if (err < 0)
    {
      return err;
    }

  /*  Set frequency in Hz */

  err = rn2483_radio_set_freq(priv, priv->config.freq);
  if (err < 0)
    {
      return err;
    }

  /* Set coding rate */

  err = rn2483_radio_set_cr(priv, priv->config.cr);
  if (err < 0)
    {
      return err;
    }

  /* Set bandwidth in kHz */

  err = rn2483_radio_set_bw(priv, priv->config.bw);
  if (err < 0)
    {
      return err;
    }

  /* Set preamble length */

  err = rn2483_radio_set_prlen(priv, priv->config.prlen);
  if (err < 0)
    {
      return err;
    }

  /* Set IQ invert */

  err = rn2483_radio_set_iqi(priv, priv->config.iqi);
  if (err < 0)
    {
      return err;
    }

  /* Set cyclic redundancy check */

  err = rn2483_radio_set_crc(priv, priv->config.crc);
  if (err < 0)
    {
      return err;
    }

  /* Set transmit output power */

  err = rn2483_radio_set_pwr(priv, priv->config.pwr);
  if (err < 0)
    {
      return err;
    }

  /* Set spread factor */

  err = rn2483_radio_set_sf(priv, priv->config.sf);
  if (err < 0)
    {
      return err;
    }

  /* Set synchronization word */

  err = rn2483_radio_set_sync(priv, priv->config.sync);
  if (err < 0)
    {
      return err;
    }

  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rn2483_register
 *
 * Description:
 *   Register the RN2483 LoRa driver.
 *
 ****************************************************************************/

int rn2483_register(FAR const char *devpath, FAR const char *uartpath)
{

  FAR struct rn2483_dev_s *priv = NULL;
  int err = 0;

  DEBUGASSERT(uartpath != NULL);

  /* Initialize the device structure */

  priv = kmm_zalloc(sizeof(struct rn2483_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance.\n");
      return -ENOMEM;
    }

  /* Initialize mutex */

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      // TODO: what logging macro needs to be used
      // snerr("ERROR: Failed to register SHT4X driver: %d\n", err);
      kmm_free(priv);
      return err;
    }

  /* Get access to underlying UART interface */

  err = file_open(&priv->uart, uartpath, O_RDWR | O_CLOEXEC);
  if (err < 0)
    {
      // TODO log error
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return err;
    }

  priv->receiving = false;

  /* Set configuration options. */

#if CONFIG_LPWAN_RN2483_CR == 5
  priv->config.cr = RN2483_CR_4_5;
#elif CONFIG_LPWAN_RN2483_CR == 6
  priv->config.cr = RN2483_CR_4_6;
#elif CONFIG_LPWAN_RN2483_CR == 7
  priv->config.cr = RN2483_CR_4_7;
#elif CONFIG_LPWAN_RN2483_CR == 8
  priv->config.cr = RN2483_CR_4_8;
#endif

#if CONFIG_LPWAN_RN2483_LORA
  priv->config.mod = RN2483_MOD_LORA;
#else
  priv->config.mod = RN2483_MOD_FSK;
#endif

  priv->config.freq = CONFIG_LPWAN_RN2483_FREQ;
  priv->config.bw = CONFIG_LPWAN_RN2483_BW;
  priv->config.prlen = CONFIG_LPWAN_RN2483_PRLEN;
  priv->config.iqi = CONFIG_LPWAN_RN2483_IQI;
  priv->config.crc = CONFIG_LPWAN_RN2483_CRC;
  priv->config.pwr = CONFIG_LPWAN_RN2483_POWER;
  priv->config.sf = CONFIG_LPWAN_RN2483_SPREAD;
  priv->config.sync = CONFIG_LPWAN_RN2483_SYNC;

  /* struct pollfd *poll_array; */
  /* poll_array->fd = (int)&priv->uart; */
  /* poll_array->events = POLLIN;   */
  /* int polls_set = file_poll(&priv->uart, poll_array, 1); */
  /* if (polls_set < 0) */
  /* { */
  /*   return polls_set; */
  /* } */

  /* Dummy read to get rid of version string that sometimes is caught on boot
   * TODO
   */

  /* Set configuration parameters on the radio via commands */

  err = rn2483_set_config(priv);
  if (err < 0)
    {
      file_close(&priv->uart);
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return err;
    }

  /* Register the character driver */

  err = register_driver(devpath, &g_rn2483fops, 0666, priv);
  if (err < 0)
    {
      wlerr("ERROR: Failed to register RN2483 driver: %d\n", err);
      file_close(&priv->uart);
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return err;
    }

  return err;
}
