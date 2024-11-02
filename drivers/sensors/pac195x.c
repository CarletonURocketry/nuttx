#include <nuttx/config.h>

#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <time.h>
#include "pac195x.h"; //temp
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/pac195x.h>
#include <nuttx/random.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define return_err(err) \
    if (err != EOK)     \
    return err

// add to config file
#ifndef CONFIG_PAC195x_I2C_FREQUENCY
#define CONFIG_PAC195x_I2C_FREQUENCY 400000
#endif

// add to config file
#define CONFIG_PAC195x_CHANNEL_1 1
#define CONFIG_PAC195x_CHANNEL_2 1
#define CONFIG_PAC195x_SAMPLE_MODE SAMPLE_1024_SPS_AD
/* Commands and Registers */

typedef enum
{

    REFRESH = 0x00,             /**< Refreshes the PAC195X. */
    CTRL = 0x01,                /**< Control register for configuring sampling mode and ALERT pins. */
    ACC_COUNT = 0x02,           /**< Accumulator count for all channels. */
    VACCN = 0x03,               /**< Accumulator outputs for channels 1-4. Spans until 0x06. */
    VBUSN = 0x07,               /**< V_BUS measurements for channels 1-4. Spans until 0x0A. */
    VSENSEN = 0x0B,             /**< V_SENSE measurements for channels 1-4. Spans until 0x0E. */
    VBUSN_AVG = 0x0F,           /**< Rolling average of the 8 most recent V_BUS_n measurements, (n: 1-4). Spans until 0x12. */
    VSENSEN_AVG = 0x13,         /**< Rolling average of the 8 most recent V_SENSE_n measurements, (n: 1-4). Spans until 0x16. */
    VPOWERN = 0x17,             /**< V_SENSE * V_BUS for channels 1-4. Spans until 0x1A. */
    SMBUS_SETTINGS = 0x1C,      /**< Activate SMBus functionality, I/O data for R/W on I/O pins. */
    NEG_PWR_FSR = 0x1D,         /**< Configuration control for bidirectional current. */
    REFRESH_G = 0x1E,           /**< Refresh response to General Call Adddress. */
    REFRESH_V = 0x1F,           /**< Refreshes V_BUS and V_SENSE data only, no accumulator reset. */
    SLOW = 0x20,                /**< Status and control for SLOW pin functions. */
    CTRL_ACT = 0x21,            /**< Currently active value of CTRL register. */
    NEG_PWR_FSR_ACT = 0x22,     /**< Currently active value of NEG_PWR register. */
    CTRL_LAT = 0x23,            /**< Latched active value of CTRL register. */
    NWG_PWR_FSR_LAT = 0x24,     /**< Latched active value of NEG_PWR register. */
    ACCUM_CONFIG = 0x25,        /**< Enable V_SENSE and V_BUS accumulation. */
    ALERT_STATUS = 0x26,        /**< Reads to see what triggered ALERT. */
    SLOW_ALERT1 = 0x27,         /**< Assigns specific ALERT to ALERTn/SLOW. */
    GPIO_ALERT2 = 0x28,         /**< Assigns specific ALERT to ALERTn/I/O. */
    ACC_FULLNESS_LIMITS = 0x29, /**< ACC and ACC Count fullness limits. */
    OC_LIMITN = 0x30,           /**< OC limit for channels 1-4. Spans until 0x33. */
    UC_LIMITN = 0x34,           /**< UC limit for channels 1-4. Spans until 0x37. */
    OP_LIMITN = 0x38,           /**< OP limit for channels 1-4. Spans until 0x3B. */
    OV_LIMITN = 0x3C,           /**< OV limit for channels 1-4. Spans until 0x3F. */
    UV_LIMITN = 0x40,           /**< UV limit for channels 1-4. Spans until 0x43. */
    OC_LIMIT_NSAMPLES = 0x44,   /**< Consecutive OC samples over threshold for ALERT. */
    UC_LIMIT_NSAMPLES = 0x45,   /**< Consecutive UC samples over threshold for ALERT. */
    OP_LIMIT_NSAMPLES = 0x46,   /**< Consecutive OP samples over threshold for ALERT. */
    OV_LIMIT_NSAMPLES = 0x47,   /**< Consecutive OV samples over threshold for ALERT. */
    UV_LIMIT_NSAMPLES = 0x48,   /**< Consecutive UV samples over threshold for ALERT. */
    ALERT_ENABLE = 0x49,        /**< ALERT enable. */
    ACCUM_CONFIG_ACT = 0x50,    /**< Currently active value of ACCUM_CONFIG register. */
    ACCUM_CONFIG_LAT = 0x51,    /**< Currently latched value of ACCUM_CONFIG register. */
    PRODUCT_ID = 0xFD,          /**< The register containing the product ID. */
    MANUFACTURER_ID = 0xFE,     /**< The register containing the manufacturer ID of 0x54. */
    REVISION_ID = 0xFF,         /**< The register containing the revision ID. Initial release is 0x02. */
} pac195x_reg_t;

// sampling modes for pac195x

/****************************************************************************
 * Private
 ****************************************************************************/

struct pac195x_dev_s
{
    FAR struct i2c_master_s *i2c; /* I2C interface.CONFIG_PAC195x_I2C_FREQUENCY */
    uint8_t addr;                 /* I2C address. */
    mutex_t devlock;
    // TODO ID struct?
    int16_t crefs; /*Number of open instances*/
    unit8_t mau_id;
    uint8_t prod_id;
    uint8_t rev_id;
    // TODO data struct
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int pac195x_send_byte(FAR struct pac195x_dev_s *priv, uint8_t reg addr);
static int pac195x_write_byte(FAR struct pac195x_dev_s *priv, uint8_t reg_addr,
                              FAR uint8_t *data);
static int pac195x_receive_byte(FAR struct pac195x_dev_s *priv,
                                FAR uint8_t *data);
static int pac195x_read_byte(FAR struct pac195x_dev_s *priv, uint8_t reg_addr,
                             FAR uint8_t *data);
static int pac195x_block_read(FAR struct pac195x_dev_s *priv, uint8_t reg_addr,
                              size_t nbytes, FAR uint8_t *data);
static int pac195x_block_write(FAR struct pac195x_dev_s *priv,
                               uint8_t reg_addr, size_t nbytes,
                               FAR uint8_t *data);

/* Character driver methods */

static int pac195x_open(FAR struct file *filep);
static int pac195x_close(FAR struct file *filep);
static ssize_t pac195x_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);

static ssize_t pac195x_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static int pac195x_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pac195xfops =
    {
        .open = pac195x_open,
        .close = pac195x_close,
        .read = pac195x_read,
        .write = NULL,
        .seek = NULL,
        .ioctl = pac195x_ioctl,
        .mmap = NULL,
        .truncate = NULL,
        .poll = NULL,
};

static int pac195x_send_byte(FAR struct pac195x_dev_s *priv, uint8_t reg addr)
{
    struct i2c_msg_s msg;
    int ret;

    msg.frequency = CONFIG_PAC195x_I2C_FREQUENCY;
    msg.addr = priv->addr;
    msg.buffer = addr;
    msg.length = 1;

    ret = i2c_send_t(priv->i2c, msg, 1);
}

static int pac195x_write_byte(FAR struct pac195x_dev_s *priv, uint8_t reg_addr, FAR uint8_t *data)
{
    struct i2c_msg_s msg;
    int ret;
    uint8_t buffer[2];

    buffer[0] = reg_address;
    buffer[1] = *data;

    msg.frequency = CONFIG_PAC195x_I2C_FREQUENCY;
    msg.addr = priv->addr;
    msg.flags = 0;
    msg.buffer = buffer;
    msg.length = 2;

    ret = i2c_send_t(priv->i2c, msg, 1);

    return ret;
}

static int pac195x_receive_byte(FAR struct pac195x_dev_s *priv, far uint8_t *data)
{
    struct i2c_msg_s;
    msg.frequency = CONFIG_PAC195x_I2C_FREQUENCY;
    msg.addr = priv->addr;
    msg.flags = I2C_M_READ;
    msg.buffer = data;
    msg.length = 1;

    ret = i2c_send_t(priv->i2c, msg, 1);

    return ret;
}

static int pac195x_read_byte(FAR struct pac195x_dev_s *priv, uint8_t reg_addr, FAR uint8_t *data)
{
    struct i2c_msg_s msg[2];
    int ret;

    msg[0].frequency = CONFIG_PAC195x_I2C_FREQUENCY;
    msg[0].addr = priv->addr;
    msg[0].flags = 0;
    msg[0].buffer = &reg_addr;
    msg[0].length = 1;

    msg[1].frequency = CONFIG_PAC195x_I2C_FREQUENCY;
    msg[1].addr = priv->addr;
    msg[1].flags = I2C_M_READ;
    msg[1].buffer = data;
    msg[1].length = 1;

    ret = i2c_send_t(priv->i2c, msg, 2);

    return ret;
}

static int pac195x_block_read(FAR struct pac195x_dev_s *priv, uint8_t reg_addr, size_t nbytes, FAR uint8_t *data)
{
    struct i2c_msg_s msg[2];
    int ret;

    msg[0].frequency = CONFIG_PAC195x_I2C_FREQUENCY;
    msg[0].addr = priv->addr;
    msg[0].flags = 0;
    msg[0].buffer = &reg_addr;
    msg[0].length = 1;

    msg[1].frequency = CONFIG_PAC195x_I2C_FREQUENCY;
    msg[1].addr = priv->addr;
    msg[1].flags = I2C_M_READ;
    msg[1].buffer = data;
    msg[1].length = nbytes

        ret = i2c_send_t(priv->i2c, msg, 2);

    return ret;
}

static int pac195x_block_write(FAR struct pac195x_dev_s *priv, uint8_t reg_addr, size_t nbytes, FAR uint8_t *data)
{
    struct i2c_msg_s msg;
    int ret;
    uint8_t buffer[nbytes + 1];
    buffer[0] = reg_addr;

    memcpy(&buffer[1], data, nbytes);
    msg.frequency = CONFIG_PAC195x_I2C_FREQUENCY;
    msg.addr = priv->addr;
    msg.flags = 0;
    msg.buffer = buffer;
    msg.length = 2;

    ret = i2c_send_t(priv->i2c, msg, 1);

    return ret;
}

/****************************************************************************
 * Name: pac195x_open
 *
 * Description:
 *   This function is called whenever the SHT3x device is opened.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int pac195x_open(FAR struct file *filep)
{
    FAR struct inode *inode = filep->f_inode;
    FAR struct sht3x_dev_s *priv = inode->i_private;
    int ret;

    /* Get exclusive access */
    ret = nxmutex_lock(&priv->devlock);
    if (ret < 0)
    {
        return ret;
    }

    /* Increment the count of open references on the driver */

    priv->crefs++;
    DEBUGASSERT(priv->crefs > 0);
    nxmutex_unlock(&priv->devlock);

    // get ID values and TOD: Debug staments.. check against config file expected ID?
    priv->rev_id = pac195x_get_rev_id(&priv, REVISION_ID);
    priv->mau_id = pac195x_get_manu_id(&priv, MANUFACTURER_ID);
    priv->prod_id = pac195x_get_prod_id(&priv, PRODUCT_ID);

    // apply config TODO: add debug satements
    if (CONFIG_PAC195x_CHANNEL_1)
    {
        int err = pac195x_toggle_channel(&priv, CHANNEL1, true);
        if (err != 0)
            printf("%s", "Channel 1 INIT FAILURE");
    }
    if (CONFIG_PAC195x_CHANNEL_2)
    {
        int err = pac195x_toggle_channel(&priv, CHANNEL1, true);
        if (err != EOK)
            printf("%s", "Channel 1 INIT FAILURE");
    }

    // set sample mode
    int err = pac195x_set_sample_mode(&priv, CONFIG_PAC195x_SAMPLE_MODE);
    if (err != EOK)
        printf("%s", "Channel 2 INIT FAILURE");

    // refresh and sleep 1ms:
    err = pac195x_refresh(&priv) if (err != EOK) printf("%s", "REFRESH FAILED");
}
#endif

/****************************************************************************
 * Name: pac195x_get_manu_id
 *
 * Description:
 *   Reads chip ID into 'id', should allways be 0x54
 *
 ****************************************************************************/

int pac195x_get_manu_id(FAR struct pac195x_dev_s *priv, uint8_t *id)
{
    return pac195x_read_byte(loc, MANUFACTURER_ID, id);
}

/****************************************************************************
 * Name: pac195x_get_manu_id
 *
 * Description:
 *    Reads the product ID from the PAC195X into `id`. The value is chip model dependent.
 *
 ****************************************************************************/

int pac195x_get_prod_id(FAR struct pac195x_dev_s *priv, uint8_t *id)
{
    return pac195x_read_byte(priv, PRODUCT_ID, id);
}

/****************************************************************************
 * Name: pac195x_refresh
 *
 * Description:
 *    Reads the revision ID from the PAC195X into `id`. Should be 0x02.
 *
 ****************************************************************************/

int pac195x_get_rev_id(FAR struct pac195x_dev_s *priv, uint8_t *id)
{
    return pac195x_read_byte(priv, REVISION_ID, id);
}

/****************************************************************************
 * Name: pac195x_get_rev_id
 *
 * Description:
 *    Sends The refresh command to the PAC195x sensor
 *
 ****************************************************************************/
int pac195x_refresh(FAR struct pac195x_dev_s *priv)
{
    return pac195x_send_byte(priv, REFRESH);
}

/****************************************************************************
 * Name: pac195x_refresh_v
 *
 * Description:
 *    Sends the refresh v command to the PAC195X.
 *    Same as refresh except accumulators and accumulator count are not reset.
 *
 ****************************************************************************/

int pac195x_refresh_v(FAR struct pac195x_dev_s *priv)
{
    return pac195x_send_byte(priv, REFRESH_V);
}

/****************************************************************************
 * Name: pac195x_set_sample_mode
 *
 * Description:
 *    Set the sampling mode for the PAC195x.
 *
 ****************************************************************************/

int pac195x_set_sample_mode(FAR struct pac195x_dev_s *priv, pac195x_sm_e mode)
{
    uint8_t ctrl_reg;
    int err = pac195x_read_byte(priv, CTRL, &ctrl_reg);
    return_err(err);

    ctrl_reg &= ~(0xF0); // Clear upper 4 bits
    ctrl_reg |= mode;    // Set the mode
    err = pac195x_write_byte(priv, CTRL, ctrl_reg);
    return err;
}

/****************************************************************************
 * Name: pac195x_toggle_channel
 *
 * Description:
 *    Enable/disable channels for sampling on the PAC195X.
 *
 ****************************************************************************/

int pac195x_toggle_channel(FAR struct pac195x_dev_s *priv, pac195x_channel_e channel, bool enable)
{

    uint8_t buf[2]; // Buffer for 16 bit message

    // Read the current CTRL register values (2 bytes)
    int err = pac195x_block_read(priv, CTRL, 2, buf);

    return_err(err);

    // Modify the control register to enable or disable the channels
    if (enable)
    {
        buf[1] &= ~(channel << 4); // Clear the relevant bits to enable channels (0 enables)
    }
    else
    {
        buf[1] |= (channel << 4); // Set the relevant bits to disable channels (1 disables)
    }
    err = pac195x_block_write(priv, CTRL, 2, buf);
    return err;
}

/****************************************************************************
 * Name: pac195x_get_16b_channel
 *
 * Description:
 *     Generic function for reading 2 bit values from a specific channel number.
 *
 ****************************************************************************/

static int pac195x_get_16b_channel(FAR struct pac195x_dev_s *priv, uint8_t addr, uint8_t n, uint16_t *val)
{
    if (n > 4 || n < 1)
        return EINVAL; // Invalid channel number

    int8_t buf[2]; // Buffer for the 16-bit response
    // Read 2 bytes from the given address (starting at reg_addr for channel n)
    int err = pac195x_block_read(priv, addr + (n - 1), 2, buf);
    return_err(err);

    // Combine the two bytes into a 16-bit value
    *val = (uint16_t)((buf[0] << 8) | buf[1]); // bit shift the 8 bit 'higher' value by 8, OR for other 8 bits

    return err;
}

/****************************************************************************
 * Name: pac195x_get_vsensen
 *
 * Description:
 *     Get the V_SENSE for channels 1-4
 *
 ****************************************************************************/

int pac195x_get_vsensen(FAR struct pac195x_dev_s *priv, uint8_t n, uint16_t *val)
{
    return pac195x_get_16b_channel(priv, VSENSEN, n, val);
}

/****************************************************************************
 * Name: pac195x_get_vbusn
 *
 * Description:
 *     Get the V_BUS measurements for channels 1-4.
 *
 ****************************************************************************/

int pac195x_get_vbusn(FAR struct pac195x_dev_s *priv, uint8_t n, uint16_t *val)
{
    return pac195x_get_16b_channel(priv, VBUSN, n, val);
}

/****************************************************************************
 * Name: pac195x_get_vbusnavg
 *
 * Description:
 *     Get the V_BUS_AVG measurements for channels 1-4.
 *
 ****************************************************************************/

int pac195x_get_vbusnavg(FAR struct pac195x_dev_s *priv, uint8_t n, uint16_t *val)
{
    return pac195x_get_16b_channel(priv, VBUSN_AVG, n, val);
}

/****************************************************************************
 * Name: pac195x_get_vsensenavg
 *
 * Description:
 *     Get the V_BUS_AVG measurements for channels 1-4.
 *
 ****************************************************************************/

int pac195x_get_vsensen(FAR struct pac195x_dev_s *priv, uint8_t n, uint16_t *val)
{
    return pac195x_get_16b_channel(priv, VSENSEN_AVG, n, val);
}

/****************************************************************************
 * Name: pac195x_register
 *
 * Description:
 *   Register the pac195x character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the pac195x
 *   addr    - The I2C address of the pac195x.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pac195x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr)
{
    FAR struct pac195x_dev_s *priv;
    int ret;

    DEBUGASSERT(i2c != NULL);

    /* Initialize the device structure */

    priv = kmm_zalloc(sizeof(struct pac195x_dev_s));
    if (priv == NULL)
    {
        snerr("ERROR: Failed to allocate instance\n");
        return -ENOMEM;
    }

    priv->i2c = i2c;
    priv->addr = addr;

    nxmutex_init(&priv->devlock);

    /* Register the character driver */

    ret = register_driver(devpath, &g_pac195xfops, 0666, priv);
    if (ret < 0)
    {
        snerr("ERROR: Failed to register driver: %d\n", ret);
        nxmutex_destroy(&priv->devlock);
        kmm_free(priv);
    }

    return ret;
}
