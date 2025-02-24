/****************************************************************************
 * drivers/sensors/mcp9600_uorb.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/nuttx.h>
#include <nuttx/signal.h>

#include <assert.h>
#include <debug.h>
#include <endian.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ads1115.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

#if defined(CONFIG_ADC_ADS1115)

#ifndef CONFIG_ADS1115_I2C_FREQUENCY
#define CONFIG_ADS1115_I2C_FREQUENCY 100000
#endif

#ifndef CONFIG_ADC_ADS1115_DEFAULT_CONFIG
#define CONFIG_ADC_ADS1115_DEFEAULT_CONFIG 0x8083
#endif

#define ADS1115_NUM_CHANNELS 8

#define ADS1115_CMD_BYTES_OS (1 << 15)
#define ADS1115_CMD_BYTES_MUX_SHIFT 12
#define ADS1115_CMD_BYTES_MUX_MASK (7 << ADS1115_CMD_BYTES_MUX_SHIFT)
#define ADS1115_CMD_BYTES_PGA_SHIFT 9
#define ADS1115_CMD_BYTES_PGA_MASK (7 << ADS1115_CMD_BYTES_PGA_SHIFT)
#define ADS1115_CMD_BYTES_MODE_MASK (1 << 8)
#define ADS1115_CMD_BYTES_DR_SHIFT 5
#define ADS1115_CMD_BYTES_DR_MASK (7 << ADS1115_CMD_BYTES_DR_SHIFT)
#define ADS1115_CMD_BYTES_COMP_MODE_MASK (1 << 4)
#define ADS1115_CMD_BYTES_COMP_POL_MASK (1 << 3)
#define ADS1115_CMD_BYTES_COMP_LAT_MASK (1 << 2)
#define ADS1115_CMD_BYTES_COMP_QUE_SHIFT 0
#define ADS1115_CMD_BYTES_COMP_QUE_MASK (3 << ADS1115_CMD_BYTES_COMP_QUE_SHIFT)

#define ADS1115_CONVERSION_REGISTER 0x00;
#define ADS1115_CONFIG_REGISTER 0x01;
#define ADS1115_LO_THRESH_REGISTER 0x02;
#define ADS1115_HI_THRESH_REGISTER 0x03;

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ads1115_dev_s {
  FAR struct i2c_master_s *i2c;
  FAR const struct adc_callback_s *cb;
  uint8_t addr;

  /* Current configuration of ADC. */
  uint16_t cmdbyte;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ads1115_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback);
static void ads1115_reset(FAR struct adc_dev_s *dev);
static int ads1115_setup(FAR struct adc_dev_s *dev);
static void ads1115_shutdown(FAR struct adc_dev_s *dev);
static void ads1115_rxint(FAR struct adc_dev_s *dev, bool enable);
static int ads1115_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_ads1115ops = {
    .ao_bind = ads1115_bind,
    .ao_reset = ads1115_reset,
    .ao_setup = ads1115_setup,
    .ao_shutdown = ads1115_shutdown,
    .ao_rxint = ads1115_rxint,
    .ao_ioctl = ads1115_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads1115_readchannel
 *
 * Description:
 *   Reads a conversion from the ADC.
 *
 * Input Parameters:
 *
 *
 *
 *
 *
 *
 *
 *
 ****************************************************************************/

static int ads1115_readchannel(FAR struct ads1115_dev_s *priv,
                               FAR struct adc_msg_s *msg) {
  int ret = OK;

  if (priv == NULL || msg == NULL) {
    ret = -EINVAL;
  } else {
    struct i2c_msg_s i2cmsg[2];
    struct i2c_msg_s i2cmsg_read[1];

    uint8_t channel = msg->am_channel;
    uint16_t channel_bits = channel << ADS1115_CMD_BYTES_MUX_SHIFT;
    priv->cmdbyte &= ~(ADS1115_CMD_BYTES_MUX_MASK);
    priv->cmdbyte |= channel_bits;

    uint16_t cmdbyte = htobe16(priv->cmdbyte);

    ainfo("cmdbyte: %d\n", priv->cmdbyte);

    uint8_t config_register = ADS1115_CONFIG_REGISTER;

    /* Write the configuration register into the address pointer */
    i2cmsg[0].frequency = CONFIG_ADS1115_I2C_FREQUENCY;
    i2cmsg[0].addr = priv->addr;
    i2cmsg[0].flags = 0;
    i2cmsg[0].buffer = (FAR uint8_t *)(&config_register);
    i2cmsg[0].length = sizeof(config_register);

    /* Actually write into the register */
    i2cmsg[1].frequency = CONFIG_ADS1115_I2C_FREQUENCY;
    i2cmsg[1].addr = priv->addr;
    i2cmsg[1].flags = I2C_M_NOSTART;
    i2cmsg[1].buffer = (FAR uint8_t *)&cmdbyte;
    i2cmsg[1].length = sizeof(cmdbyte);

    ret = I2C_TRANSFER(priv->i2c, i2cmsg, 2);
    if (ret < 0) {
      aerr("ADS1115 I2C transfer failed: %d\n", ret);
    }

    /* We want to read the configuration register */
    i2cmsg_read[0].frequency = CONFIG_ADS1115_I2C_FREQUENCY;
    i2cmsg_read[0].addr = priv->addr;
    i2cmsg_read[0].flags = I2C_M_READ;

    uint16_t buf;
    i2cmsg_read[0].buffer = (FAR uint8_t *)(&buf);
    i2cmsg_read[0].length = sizeof(buf);

    do {
      ret = I2C_TRANSFER(priv->i2c, i2cmsg_read, 1);
      if (ret < 0) {
        aerr("ADS1115 I2C transfer failed: %d\n", ret);
      }
    } while ((betoh16(buf) & ADS1115_CMD_BYTES_OS) == 0);

    ainfo("config register: %d\n", betoh16(buf));

    uint8_t conversion_register = ADS1115_CONVERSION_REGISTER;

    struct i2c_msg_s i2cmsg2[2];

    /* Write the conversion register to the address pointer */
    i2cmsg2[0].frequency = CONFIG_ADS1115_I2C_FREQUENCY;
    i2cmsg2[0].addr = priv->addr;
    i2cmsg2[0].flags = 0;
    i2cmsg2[0].buffer = (FAR uint8_t *)(&conversion_register);
    i2cmsg2[0].length = sizeof(conversion_register);

    ret = I2C_TRANSFER(priv->i2c, i2cmsg2, 1);
    if (ret < 0) {
      aerr("ADS1115 I2C transfer failed: %d\n", ret);
    }

    int16_t buf2;

    /* We want to read the conversion register */
    i2cmsg_read[0].frequency = CONFIG_ADS1115_I2C_FREQUENCY;
    i2cmsg_read[0].addr = priv->addr;
    i2cmsg_read[0].flags = I2C_M_READ;
    i2cmsg_read[0].buffer = (FAR uint8_t *)(&buf2);
    i2cmsg_read[0].length = sizeof(buf2);

    ret = I2C_TRANSFER(priv->i2c, i2cmsg_read, 1);
    if (ret < 0) {
      aerr("ADS1115 I2C transfer failed: %d\n", ret);
    }

    ainfo("output: %d\n", betoh16(buf2));

    msg->am_data = (uint32_t)betoh16(buf2);
  }
  return ret;
}

/****************************************************************************
 * Name: ads1115_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int ads1115_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback) {
  FAR struct ads1115_dev_s *priv = (FAR struct ads1115_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return 0;
}

/****************************************************************************
 * Name: ads1115_reset
 *
 * Description:
 *   Reset the ADC device. This is called when the ADC device is closed.
 *
 ****************************************************************************/
static void ads1115_reset(FAR struct adc_dev_s *dev) {
  FAR struct ads1115_dev_s *priv = (FAR struct ads1115_dev_s *)dev->ad_priv;

  priv->cmdbyte = CONFIG_ADC_ADS1115_DEFAULT_CONFIG;
}

/****************************************************************************
 * Name: ads1115_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.
 *
 ****************************************************************************/
static int ads1115_setup(FAR struct adc_dev_s *dev) { return OK; }

/****************************************************************************
 * Name: ads1115_shutdown
 *
 * Description:
 *   Disable the ADC. This method is called when the ADC device is closed.
 *
 ****************************************************************************/
static void ads1115_shutdown(FAR struct adc_dev_s *dev) {}

/****************************************************************************
 * Name: ads1115_rxint
 *
 * Description:
 *   Needed for ADC upper-half compatibility but conversion interrupts
 *   are not supported by the ADS1115.
 *
 ****************************************************************************/
static void ads1115_rxint(FAR struct adc_dev_s *dev, bool enable) {}

/****************************************************************************
 * Name: ads1115_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 ****************************************************************************/
static int ads1115_ioctl(FAR struct adc_dev_s *dev, int cmd,
                         unsigned long arg) {
  FAR struct ads1115_dev_s *priv = (FAR struct ads1115_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd) {
  case ANIOC_TRIGGER: {
    struct adc_msg_s msg;

    for (uint8_t i = 0; i < ADS1115_NUM_CHANNELS && (ret == 0); i++) {
      msg.am_channel = i;
      ainfo("Channel: %d\n", i);
      ret = ads1115_readchannel(priv, &msg);
      if (ret == 0) {
        priv->cb->au_receive(dev, i, msg.am_data);
      } else {
        aerr("Error reading channel %d with error %d \n", i, ret);
      }
    }
  } break;

  case ANIOC_ADS1115_SET_OS: {
    if (arg == ADS1115_OS1) {
      priv->cmdbyte |= ADS1115_CMD_BYTES_OS;
    } else if (arg == ADS1115_OS2) {
      priv->cmdbyte &= ~ADS1115_CMD_BYTES_OS;
    } else {
      ret = -EINVAL;
    }
  } break;

  case ANIOC_ADS1115_SET_PGA: {
    if (arg > ADS1115_PGA1 && arg < ADS1115_PGA8) {
      priv->cmdbyte &= ~ADS1115_CMD_BYTES_PGA_MASK;
      priv->cmdbyte |= arg << ADS1115_CMD_BYTES_PGA_SHIFT;
    } else {
      ret = -EINVAL;
    }
  } break;

  case ANIOC_ADS1115_SET_MODE: {
    if (arg == ADS1115_MODE1) {
      priv->cmdbyte &= ~ADS1115_CMD_BYTES_MODE_MASK;
    } else if (arg == ADS1115_MODE2) {
      priv->cmdbyte |= ADS1115_CMD_BYTES_MODE_MASK;
    } else {
      ret = -EINVAL;
    }
  } break;

  case ANIOC_ADS1115_SET_DR: {
    if (arg > ADS1115_DR1 && arg < ADS1115_DR8) {
      priv->cmdbyte &= ~ADS1115_CMD_BYTES_DR_MASK;
      priv->cmdbyte |= arg << ADS1115_CMD_BYTES_DR_SHIFT;
    } else {
      ret = -EINVAL;
    }
  } break;

  case ANIOC_ADS1115_SET_COMP_MODE: {
    if (arg == ADS1115_COMP_MODE1) {
      priv->cmdbyte &= ~ADS1115_CMD_BYTES_COMP_MODE_MASK;
    } else if (arg == ADS1115_COMP_MODE2) {
      priv->cmdbyte |= ADS1115_CMD_BYTES_COMP_MODE_MASK;
    } else {
      ret = -EINVAL;
    }
  } break;

  case ANIOC_ADS1115_SET_COMP_LAT: {
    if (arg == ADS1115_COMP_LAT1) {
      priv->cmdbyte &= ~ADS1115_CMD_BYTES_COMP_LAT_MASK;
    } else if (arg == ADS1115_COMP_LAT2) {
      priv->cmdbyte |= ADS1115_CMD_BYTES_COMP_LAT_MASK;
    } else {
      ret = -EINVAL;
    }
  } break;

  case ANIOC_ADS1115_SET_COMP_QUEUE: {
    if (arg > ADS1115_COMP_QUEUE1 && arg < ADS1115_COMP_QUEUE4) {
      priv->cmdbyte &= ~ADS1115_CMD_BYTES_COMP_QUE_MASK;
      priv->cmdbyte |= arg << ADS1115_CMD_BYTES_COMP_QUE_SHIFT;
    } else {
      ret = -EINVAL;
    }
  } break;

  case ANIOC_ADS1115_READ_CHANNEL: {
    FAR struct adc_msg_s *msg = (FAR struct adc_msg_s *)arg;
    ret = ads1115_readchannel(priv, msg);
  } break;
  }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads1115_initialize
 *
 * Description:
 *  Initialize the selected ADC
 *
 * Input Parameters:
 *  i2c - Pointer to a valid I2C master struct.
 *  addr - I2C device address.
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 * ****************************************************************************/

FAR struct adc_dev_s *ads1115_initialize(FAR struct i2c_master_s *i2c,
                                         uint8_t addr) {
  FAR struct ads1115_dev_s *priv;
  FAR struct adc_dev_s *adcdev;

  DEBUGASSERT(i2c != NULL);

  priv = kmm_malloc(sizeof(struct ads1115_dev_s));
  if (priv == NULL) {
    aerr("ERROR: Failed to allocate ads1115_dev_s instance\n");
    free(priv);
    return NULL;
  }

  priv->cb = NULL;
  priv->i2c = i2c;
  priv->addr = addr;
  priv->cmdbyte = CONFIG_ADC_ADS1115_DEFAULT_CONFIG;

  adcdev = kmm_malloc(sizeof(struct adc_dev_s));
  if (adcdev == NULL) {
    aerr("ERROR: Failed to allocate adc_dev_s instance\n");
    return NULL;
  }

  memset(adcdev, 0, sizeof(struct adc_dev_s));
  adcdev->ad_ops = &g_ads1115ops;
  adcdev->ad_priv = priv;

  return adcdev;
}

#endif /* CONFIG_ADC_ADS1115 */