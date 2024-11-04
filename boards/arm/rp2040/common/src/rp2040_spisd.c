/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_spisd.c
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

#include <debug.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/partition.h>
#include <nuttx/mmcsd.h>

#include "rp2040_gpio.h"
#include "rp2040_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_RP2040_SPISD_SLOT_NO
#define CONFIG_RP2040_SPISD_SLOT_NO 0
#endif

typedef struct {
  int partition_num;
  uint8_t err;
} partition_state_t;

static void partition_handler(struct partition_s *part, void *arg) {
  partition_state_t *partition_handler_state = (partition_state_t *)arg;

  char devname[] = "/dev/mmcsd0p0";

  if (partition_handler_state->partition_num < 10 &&
      part->index == partition_handler_state->partition_num) {
    finfo("Num of sectors: %d \n", part->nblocks);
    devname[sizeof(devname) - 2] = partition_handler_state->partition_num + 48;
    register_blockpartition(devname, 0, "/dev/mmcsd0", part->firstblock,
                            part->nblocks);
    partition_handler_state->err = 0;
  }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_spisd_initialize
 *
 * Description:
 *   Initialize the SPI-based SD card.
 *
 ****************************************************************************/

int board_spisd_initialize(int minor, int bus) {
  int ret;
  struct spi_dev_s *spi;

  /* Initialize spi device */

  spi = rp2040_spibus_initialize(bus);
  if (!spi) {
    ferr("ERROR: Failed to initialize spi%d.\n", bus);
    return -ENODEV;
  }

  /* Pull up RX */

#ifdef CONFIG_RP2040_SPI0
  if (bus == 0) {
    rp2040_gpio_set_pulls(CONFIG_RP2040_SPI0_RX_GPIO, true, false);
  }
#endif

#ifdef CONFIG_RP2040_SPI1
  if (bus == 1) {
    rp2040_gpio_set_pulls(CONFIG_RP2040_SPI1_RX_GPIO, true, false);
  }
#endif

  /* Get the SPI driver instance for the SD chip select */

  finfo("Initializing SPI for the MMC/SD slot\n");

  ret = mmcsd_spislotinitialize(minor, CONFIG_RP2040_SPISD_SLOT_NO, spi);
  if (ret < 0) {
    ferr("ERROR: Failed to bind SPI device to MMC/SD slot %d: %d\n",
         CONFIG_RP2040_SPISD_SLOT_NO, ret);
    return ret;
  }

  /*Partitioning and mounting code*/

  static partition_state_t partitions[] = {
      {.partition_num = 0, .err = ENOENT},
      {.partition_num = 1, .err = ENOENT},
  };

  for (int i = 0; i < 2; i++) {
    parse_block_partition("/dev/mmcsd0", partition_handler, &partitions[i]);
    if (partitions[i].err == ENOENT) {
      fwarn("Partition %d did not register \n", partitions[i].partition_num);
    } else {
      finfo("Partition %d registered! \n", partitions[i].partition_num);
    }
  }

  ret = nx_mount("/dev/mmcsd0p0", "/mnt/sd0p0", "vfat", 666, NULL);
  /*Invalid Superblock, means it's not formatted as vfat*/
  if (ret == -EINVAL) {
    // for now do nothing, in the future try to format it as fat 32.
    ;
  }
  if (ret) {
    ferr("ERROR: Could not mount fat partition %d: \n", ret);
    return ret;
  }

  /*Mount our littlefs partition, and format it if it's corrupted*/
  ret = nx_mount("dev/mmcsd0p1", "/mnt/sd0p1", "littlefs", 666, "autoformat");
  if (ret) {
    ferr("ERROR: could not mount littlefs partition %d: \n", ret);
    return ret;
  }

  return OK;
}

/****************************************************************************
 * Name: board_spisd_status
 *
 * Description:
 *   Get the status whether SD Card is present or not.
 *   This function is called only from rp2040_spi.c.
 *
 * Returned Value:
 *   Return SPI_STATUS_PRESENT if SD Card is present. Otherwise, return 0.
 *
 ****************************************************************************/

uint8_t board_spisd_status(struct spi_dev_s *dev, uint32_t devid) {
  uint8_t ret = 0;

  if (devid == SPIDEV_MMCSD(0)) {
    /* Card detection is not supported yet */

    ret = SPI_STATUS_PRESENT;
  }

  return ret;
}
