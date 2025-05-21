/****************************************************************************
 * drivers/src/analog/ads131.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>

#include <nuttx/analog/ads131.h>
#include <nuttx/analog/adc.h>
#include <nuttx/arch.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/kthread.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_DRV_ADS131_DEBUG)
#define atrace ainfo
#else
#define atrace _none
#endif /* CONFIG_DRV_ADS131_DEBUG */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ads131_dev_s
{
  FAR struct ads131_config_s *config;
  struct spi_dev_s *spi; /* Cached SPI device reference */
  int devno;
  int refs;
  mutex_t devlock;
  uint32_t frequency;
  struct gpio_dev_s *reset_gpio;

  int irq;
  FAR const struct adc_callback_s *cb;
  struct work_s work;

  /* Array used to recall device register map configurations */

  uint16_t register_map[ADS131_NUM_REGISTERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ads131_get_samples(struct adc_dev_s *dev, int32_t *samples,
                              size_t count);

static uint16_t ads131_combine_bytes(uint8_t upper_byte,
                                     uint8_t lower_byte);
static int32_t ads131_sign_extend(const uint8_t data_bytes[]);
static uint16_t ads131_is_reset_ok(struct adc_dev_s *dev);
static int ads131_bind(struct adc_dev_s *dev,
                       const struct adc_callback_s *callback);
static int ads131_setup(struct adc_dev_s *dev);
static void ads131_reset(struct adc_dev_s *dev);
static void ads131_shutdown(struct adc_dev_s *dev);
static void ads131_rxint(struct adc_dev_s *dev, bool enable);
static int ads131_ioctl(FAR struct adc_dev_s *dev, int cmd,
                        unsigned long arg);

/* Interrupt handling */
static void ads131_worker(FAR void *arg);
static int  ads131_interrupt(int irq, void *context, FAR void *arg);

static uint8_t ads131_build_spi_msg(struct ads131_dev_s *priv,
                                    const uint16_t opcode_array[],
                                    uint8_t num_opcodes,
                                    uint8_t *byte_array);
static uint16_t ads131_enforce_selected_device_modes(uint16_t data);
static int ads131_reset_device(struct adc_dev_s *dev);
static void ads131_adc_startup(struct adc_dev_s *dev);
static void ads131_restore_register_defaults(struct ads131_dev_s *priv);
static void ads131_write_single_register(struct adc_dev_s *dev,
                                         uint8_t address, uint16_t data);
static uint8_t ads131_get_word_length(struct ads131_dev_s *priv);
static uint16_t ads131_get_reg_val(struct ads131_dev_s *priv,
                                   uint8_t address);
static uint16_t ads131_read_single_register(struct adc_dev_s *dev,
                                            uint8_t address);
static int ads131_trigger_sample(struct ads131_dev_s *priv);

#define SPI_DEV SPIDEV_ADC(priv->devno)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Array of SPI word lengths */

/* clang-format off */

const static uint8_t g_wlength_byte_values[] =
  {
    2, 3, 4, 4
  };

/* Analog lower-half bindings */

static const struct adc_ops_s ads131_ops_s =
  {
    ads131_bind,     /* ao_bind */
    ads131_reset,    /* ao_reset */
    ads131_setup,    /* ao_setup */
    ads131_shutdown, /* ao_shutdown */
    ads131_rxint,    /* ao_rxint */
    ads131_ioctl     /* ao_read */
  };

/* clang-format on */

/****************************************************************************
 * Private Register Macros
 ****************************************************************************/

/** Returns Number of Channels */

#define ADS131_CHANCNT(priv)                                               \
  ((uint8_t)((ads131_get_reg_val(priv, ADS131_ID_ADDRESS)                  \
              & ADS131_ID_CHANCNT_MASK)                                    \
             >> 8))

/** Revision ID bits */

#define ADS131_REVISION_ID(priv)                                           \
  ((uint8_t)((ads131_get_reg_val(priv, ADS131_ID_ADDRESS)                  \
              & ADS131_ID_REVID_MASK)                                      \
             >> 0))

/** Returns true if SPI interface is locked */

#define ADS131_SPI_LOCKED(priv)                                            \
  ((bool)(ads131_get_reg_val(priv, ADS131_STATUS_ADDRESS)                  \
          & ADS131_STATUS_LOCK_LOCKED))

/** Returns SPI Communication Word Format */

#define ADS131_WLENGTH(priv)                                               \
  ((uint8_t)((ads131_get_reg_val(priv, ADS131_MODE_ADDRESS)                \
              & ADS131_STATUS_WLENGTH_MASK)                                \
             >> 8))

/** Returns true if Register Map CRC byte enable bit is set */

#define ADS131_REGMAP_CRC_ENABLED(priv)                                    \
  ((bool)(ads131_get_reg_val(priv, ADS131_MODE_ADDRESS)                    \
          & ADS131_MODE_REG_CRC_EN_ENABLED))

/** Returns true if SPI CRC byte enable bit is set */

#define ADS131_SPI_CRC_ENABLED(priv)                                       \
  ((bool)(ads131_get_reg_val(priv, ADS131_MODE_ADDRESS)                    \
          & ADS131_MODE_RX_CRC_EN_ENABLED))

/** Returns false for CCITT and true for ANSI CRC type */

#define ADS131_SPI_CRC_TYPE(priv)                                          \
  ((bool)(ads131_get_reg_val(priv, ADS131_MODE_ADDRESS)                    \
          & ADS131_MODE_CRC_TYPE_MASK))

/** Data rate register field setting */

#define ADS131_OSR_INDEX(priv)                                             \
  ((uint8_t)((ads131_get_reg_val(priv, ADS131_CLOCK_ADDRESS)               \
              & ADS131_CLOCK_OSR_MASK)                                     \
             >> 2))

/** Data rate register field setting */

#define ADS131_POWER_MODE(priv)                                            \
  ((uint8_t)((ads131_get_reg_val(priv, ADS131_CLOCK_ADDRESS)               \
              & ADS131_CLOCK_PWR_MASK)                                     \
             >> 0))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief read status cmd-register
 *
 * @return uint16_t EOK if ok
 */

static uint16_t ads131_is_reset_ok(struct adc_dev_s *dev)
{
  uint16_t result = ads131_read_single_register(dev, ADS131_STATUS_ADDRESS);

  bool ret = (bool)((result & ADS131_STATUS_RESET_RESET_OCCURRED) != 0);

  if (!ret)
    {
      aerr("%s: ERROR - Status register = 0x%" PRIx16, __func__, result);
    }

  return (ret ? OK : -EIO);
}

static int ads131_bind(struct adc_dev_s *dev,
                       const struct adc_callback_s *callback)
{
  FAR struct ads131_dev_s *priv = (FAR struct ads131_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

static int ads131_trigger_sample(struct ads131_dev_s *priv)
{
  /* Need four words for ADS131M02:
   * - Status response
   * - Channel 1 data
   * - Channel 2 data
   * - CRC
   */

  uint8_t bytes_per_word = g_wlength_byte_values[ADS131_WLENGTH(priv)];
  uint8_t words_in_frame = ADS131_CHANCNT(priv) + 2;
  uint8_t bytes_in_frame = words_in_frame * bytes_per_word;

  static uint8_t *sample_request_frame = NULL;

  if (!sample_request_frame)
    {
      sample_request_frame = kmm_zalloc(bytes_in_frame);
    }

  int flags = enter_critical_section();
  SPI_SELECT(priv->spi, SPI_DEV, true);
  SPI_SNDBLOCK(priv->spi, sample_request_frame, bytes_in_frame);
  SPI_SELECT(priv->spi, SPI_DEV, false);
  leave_critical_section(flags);

  return OK;
}

static int ads131_setup(struct adc_dev_s *dev)
{
  struct ads131_dev_s *priv = (struct ads131_dev_s *)dev->ad_priv;

  atrace("%s: Setting up.  Refs = %" PRId16, __func__, priv->refs);
  int ret = nxmutex_lock(&priv->devlock);
  if (ret != OK)
    {
      return ret;
    }

  ret = irq_attach(priv->irq, ads131_interrupt, NULL);
  if (ret != OK)
    {
      return ret;
    }

  if (!priv->refs)
    {
      ads131_reset(dev);
      ret = OK;
    }

  if (ret != OK)
    {
      nxmutex_unlock(&priv->devlock);

      aerr("%s: ERROR resetting device (%" PRId16 ")", __func__, ret);

      return ret;
    }

  priv->refs++;

  atrace("%s: Set up.  Refs = %" PRId16, __func__, priv->refs);
  return nxmutex_unlock(&priv->devlock);
}

static void ads131_reset(struct adc_dev_s *dev)
{
  struct ads131_dev_s *priv = (struct ads131_dev_s *)dev->ad_priv;
  struct spi_dev_s *spi = priv->spi;
  int ret = OK;

  SPI_SETMODE(spi, SPIDEV_MODE1);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, priv->frequency);

#ifdef CONFIG_SPI_DELAY_CONTROL
  SPI_SETDELAY(
      spi, 0, 0, 0, 0); /* No delay to de-assert CS between reads */
#endif

#if 0
  /* Handle reset either via GPIO-based reset or command based reset. */

  if (priv->reset_gpio)
    {
      priv->reset_gpio->gp_ops->go_write(priv->reset_gpio, false);
      up_udelay(1);
      priv->reset_gpio->gp_ops->go_write(priv->reset_gpio, true);
      up_udelay(1);
      priv->reset_gpio->gp_ops->go_write(priv->reset_gpio, false);
      up_udelay(250);
      priv->reset_gpio->gp_ops->go_write(priv->reset_gpio, true);
    }
  else
    {
      ret = ads131_reset_device(dev);
    }
#endif

  /* REGISTER DUMP */
  for (int i = 0; i <= 0x12; i++) {
      if (i != 5)
        ads131_read_single_register(dev, i);
  }

  if (ret == OK)
    {
      ads131_adc_startup(dev);
    }

  return /*ret*/;
}

/****************************************************************************
 *
 * Start up sequence for the ADS131M0x.
 *
 * \fn void adcStartup(struct spi_dev_s *spi)
 *
 * Before calling this function, the device must be powered,
 * the SPI/GPIO pins of the MCU must have already been configured,
 * and (if applicable) the external clock source should be provided to
 *CLKIN.
 *
 * \return None.
 *
 ****************************************************************************/

static void ads131_adc_startup(struct adc_dev_s *dev)
{
  struct ads131_dev_s *priv = (struct ads131_dev_s *)dev->ad_priv;
  DEBUGASSERT(dev);

  /* Define initial register settings */

  ads131_write_single_register(
      dev, ADS131_CLOCK_ADDRESS,
      (ADS131_CLOCK_DEFAULT & ~ADS131_CLOCK_OSR_MASK)
          | ADS131_CLOCK_OSR_16384);

  /* (REQUIRED) Configure MODE register settings
   * NOTE: This function call is required here for this particular code
   * implementation to work.
   * This function will enforce the MODE register settings as selected in
   * the 'ads131m0x.h' header file.
   */

  ads131_write_single_register(
      dev, ADS131_MODE_ADDRESS, ADS131_MODE_DEFAULT);

  // DC BLOCK - 1/4
  //ads131_write_single_register(
  //    dev, ADS131_THRSHLD_LSB_ADDRESS, ADS131_THRSHLD_LSB_DEFAULT | 0x01);

  // INPUT MUX - Positive DC Test
  //ads131_write_single_register(
  //    dev, ADS131_CH0_CFG_ADDRESS, ADS131_CH0_CFG_DEFAULT | 0x02);

  // INPUT MUX - Negative DC Test
  //ads131_write_single_register(
  //    dev, ADS131_CH1_CFG_ADDRESS, ADS131_CH1_CFG_DEFAULT | 0x03);

  // CH1/2 - INPUT GAIN - 0
  ads131_write_single_register(
      dev, ADS131_GAIN1_ADDRESS, ADS131_GAIN1_DEFAULT | 0x00);

  /* Trigger initial sample read. */

  ads131_trigger_sample(priv);
}

/****************************************************************************
 *
 *  Writes data to a single register.
 *
 *  \fn void ads131_write_single_register(struct adc_dev_s *dev,
 *                                        uint8_t address, uint16_t data)
 *
 *  \param address is the address of the register to write to.
 *  \param data is the value to write.
 *
 *  This command will be ignored if device registers are locked.
 *
 *  \return None.
 *
 ****************************************************************************/

static void ads131_write_single_register(struct adc_dev_s *dev,
                                         uint8_t address, uint16_t data)
{
  struct ads131_dev_s *priv = (struct ads131_dev_s *)dev->ad_priv;
  struct spi_dev_s *spi = priv->spi;

  /* Check that the register address is in range */

  DEBUGASSERT(address < ADS131_NUM_REGISTERS);

  /* (OPTIONAL) Enforce certain register field values when
   * writing MODE register to fix the operation mode
   */

  if (ADS131_MODE_ADDRESS == address)
    {
      data = ads131_enforce_selected_device_modes(data);
    }

#ifdef ADS131_ENABLE_CRC_IN
  /* 3 words, up to 4 bytes each = 12 bytes max */

  /* clang-format off */

  uint8_t tx_data[12] =
    {
      0
    };

  uint8_t rx_data[12] =
    {
      0
    };

#else
  /* 2 words, up to 4 bytes long = 8 bytes max */

  uint8_t tx_data[8] =
    {
      0
    };

  uint8_t rx_data[8] =
    {
      0
    };

  /* clang-format on */

#endif
  uint16_t opcodes[2];
  opcodes[0] = ADS131_OPCODE_WREG | (((uint16_t)address) << 7);
  opcodes[1] = data;
  uint8_t num_bytes = ads131_build_spi_msg(priv, &opcodes[0], 2, tx_data);

  /* Send command */

  SPI_SELECT(spi, SPI_DEV, true);
  SPI_EXCHANGE(spi, tx_data, rx_data, num_bytes);
  SPI_SELECT(spi, SPI_DEV, false);

  /* Update internal array */

  priv->register_map[address] = data;

  atrace("%s: Register 0x%x: 0x%x", __func__, address,
         priv->register_map[address]);

  /* (RECOMMENDED) Read back register to confirm register write
   * was successful
   */

  uint16_t read_data = ads131_read_single_register(dev, address);

  if (read_data != data)
    {
      aerr("%s: ERROR: setting mode failed ( \
          expected: 0x%" PRIx16 ", read: 0x%" PRIx16 ")",
           __func__, data, read_data);
    }

  /* NOTE: Enabling CRC words in SPI command will NOT prevent an invalid W
   */
}

/****************************************************************************
 *
 *  Modifies MODE register data to maintain device operation according to
 *  preselected mode(s) (RX_CRC_EN, WLENGTH, etc.).
 *
 *  \fn uint16_t enforce_selected_device_mode(uint16_t data)
 *
 *  \param data uint16_t register data.
 *
 *  \return uint16_t modified register data.
 *
 ****************************************************************************/

static uint16_t ads131_enforce_selected_device_modes(uint16_t data)
{
  /* Enforce RX_CRC_EN setting */

#ifdef ADS131_ENABLE_CRC_IN
  /* When writing MODE register, ensure RX_CRC_EN bit is ALWAYS set */

  data |= ADS131_MODE_RX_CRC_EN_ENABLED;
#else
  /* When writing MODE register, ensure RX_CRC_EN bit is NEVER set */

  data &= ~ADS131_MODE_RX_CRC_EN_ENABLED;
#endif /* ENABLE_CRC_IN */

  /* Enforce WLENGH setting */

#ifdef ADS131_WORD_LENGTH_24BIT
  /* When writing MODE register, ensure WLENGTH bits are ALWAYS set to 01b
   */

  data = (data & ~ADS131_MODE_WLENGTH_MASK) | ADS131_MODE_WLENGTH_24BIT;
#elif defined ADS131_WORD_LENGTH_32BIT_SIGN_EXTEND
  /* When writing MODE register, ensure WLENGH bits are ALWAYS set to 11b */

  data = (data & ~ADS131_MODE_WLENGTH_MASK)
         | ADS131_MODE_WLENGTH_32BIT_MSB_SIGN_EXT;
#elif defined ADS131_WORD_LENGTH_32BIT_ZERO_PADDED
  /* When writing MODE register, ensure WLENGH bits are ALWAYS set to 10b */

  data = (data & ~ADS131_MODE_WLENGTH_MASK)
         | ADS131_MODE_WLENGTH_32BIT_LSB_ZEROES;
#elif defined ADS131_WORD_LENGTH_16BIT_TRUNCATED
  /* When writing MODE register, ensure WLENGH bits are ALWAYS set to 00b */

  data = (data & ~ADS131_MODE_WLENGTH_MASK) | ADS131_MODE_WLENGTH_16BIT;
#endif

  /* Enforce DRDY_FMT setting */

#ifdef ADS131_DRDY_FMT_PULSE
  /* When writing MODE register, ensure DRDY_FMT bit is ALWAYS set */

  data = (data & ~ADS131_MODE_DRDY_FMT_MASK)
         | ADS131_MODE_DRDY_FMT_NEG_PULSE_FIXED_WIDTH;
#else
  /* When writing MODE register, ensure DRDY_FMT bit is NEVER set */

  data = (data & ~ADS131_MODE_DRDY_FMT_MASK)
         | ADS131_MODE_DRDY_FMT_LOGIC_LOW;
#endif

  /* Enforce CRC_TYPE setting */

#ifdef ADS131_CRC_CCITT
  /* When writing MODE register, ensure CRC_TYPE bit is NEVER set */

  data = (data & ~ADS131_STATUS_CRC_TYPE_MASK)
         | ADS131_STATUS_CRC_TYPE_16BIT_CCITT;
#elif defined ADS131_CRC_ANSI
  /* When writing MODE register, ensure CRC_TYPE bit is ALWAYS set */

  data = (data & ~ADS131_STATUS_CRC_TYPE_MASK)
         | ADS131_STATUS_CRC_TYPE_16BIT_ANSI;
#endif

  /* Return modified register data */

  return data;
}

/****************************************************************************
 *
 * Updates the priv->register_map[] array to its default values.
 *
 * \fn void ads131_restore_register_defaults(struct ads131_dev_s *priv)
 *
 * NOTES:
 * - If the MCU keeps a copy of the ADS131M0x register settings in memory,
 * then it is important to ensure that these values remain in sync with the
 * actual hardware settings. In order to help facilitate this, this function
 * should be called after powering up or resetting the device (either by
 * hardware pin control or SPI software command).
 *
 * - Reading back all of the registers after resetting the device can
 * accomplish the same result; however, this might be problematic if the
 * device was previously in CRC mode or the WLENGTH was modified, since
 * resetting the device exits these modes. If the MCU is not aware of this
 * mode change, then read register commands will return invalid data due to
 * the expectation of data appearing in a different byte position.
 *
 * \return None.
 ****************************************************************************/

static void ads131_restore_register_defaults(struct ads131_dev_s *priv)
{
  /* NOTE: This a read-only register */

  priv->register_map[ADS131_ID_ADDRESS]
      = ADS131_ID_DEFAULT(ADS131_NUM_CHANNELS);
  priv->register_map[ADS131_STATUS_ADDRESS] = ADS131_STATUS_DEFAULT;
  priv->register_map[ADS131_MODE_ADDRESS] = ADS131_MODE_DEFAULT;
  priv->register_map[ADS131_CLOCK_ADDRESS] = ADS131_CLOCK_DEFAULT;
  priv->register_map[ADS131_GAIN1_ADDRESS] = ADS131_GAIN1_DEFAULT;
  priv->register_map[ADS131_GAIN2_ADDRESS] = ADS131_GAIN2_DEFAULT;
  priv->register_map[ADS131_CFG_ADDRESS] = ADS131_CFG_DEFAULT;
  priv->register_map[ADS131_THRSHLD_MSB_ADDRESS]
      = ADS131_THRSHLD_MSB_DEFAULT;
  priv->register_map[ADS131_THRSHLD_LSB_ADDRESS]
      = ADS131_THRSHLD_LSB_DEFAULT;
  priv->register_map[ADS131_CH0_CFG_ADDRESS] = ADS131_CH0_CFG_DEFAULT;
  priv->register_map[ADS131_CH0_OCAL_MSB_ADDRESS]
      = ADS131_CH0_OCAL_MSB_DEFAULT;
  priv->register_map[ADS131_CH0_OCAL_LSB_ADDRESS]
      = ADS131_CH0_OCAL_LSB_DEFAULT;
  priv->register_map[ADS131_CH0_GCAL_MSB_ADDRESS]
      = ADS131_CH0_GCAL_MSB_DEFAULT;
  priv->register_map[ADS131_CH0_GCAL_LSB_ADDRESS]
      = ADS131_CH0_GCAL_LSB_DEFAULT;
  priv->register_map[ADS131_CH1_CFG_ADDRESS] = ADS131_CH1_CFG_DEFAULT;
  priv->register_map[ADS131_CH1_OCAL_MSB_ADDRESS]
      = ADS131_CH1_OCAL_MSB_DEFAULT;
  priv->register_map[ADS131_CH1_OCAL_LSB_ADDRESS]
      = ADS131_CH1_OCAL_LSB_DEFAULT;
  priv->register_map[ADS131_CH1_GCAL_MSB_ADDRESS]
      = ADS131_CH1_GCAL_MSB_DEFAULT;
  priv->register_map[ADS131_CH1_GCAL_LSB_ADDRESS]
      = ADS131_CH1_GCAL_LSB_DEFAULT;
  priv->register_map[ADS131_REGMAP_CRC_ADDRESS] = ADS131_REGMAP_CRC_DEFAULT;
}

/****************************************************************************
 *
 *  Resets the device.
 *
 *  \fn void ads131_reset_device(void)
 *
 *  NOTE: This function does not capture DOUT data, but it could be modified
 *  to do so.
 *
 *  \return None.
 *
 ****************************************************************************/

static int ads131_reset_device(struct adc_dev_s *dev)
{
  DEBUGASSERT(dev);

  int ret = OK;

  struct ads131_dev_s *priv = (struct ads131_dev_s *)dev->ad_priv;
  struct spi_dev_s *spi = priv->spi;

  atrace("%s: Resetting Device.  Refs = %" PRId16, __func__, priv->refs);

  /* Build TX and RX byte array */

  /* No CRC: 1 word, up to 4 bytes long = 4 bytes max.
   * With CRC: 2 words, up to 4 bytes each = 8 bytes max
   */

  /* clang-format off */

  uint8_t tx_data[16] =
    {
      0
    };

  /* Only needed if capturing data */

  uint8_t rx_data[16] =
    {
      0
    };

  /* clang-format on */

  uint16_t opcode = ADS131_OPCODE_RESET;

  uint8_t num_bytes = ads131_build_spi_msg(priv, &opcode, 1, tx_data);

  uint8_t bytes_per_word = g_wlength_byte_values[ADS131_WLENGTH(priv)];
  uint8_t words_in_frame = ADS131_CHANCNT(priv) + 2;

  /* Send the opcode (and CRC word, if enabled) */

  SPI_SELECT(spi, SPI_DEV, true);
  SPI_EXCHANGE(
      spi, tx_data, rx_data, num_bytes + (words_in_frame * bytes_per_word));
  SPI_SELECT(spi, SPI_DEV, false);

  /* tSRLRST delay, ~1ms with 2.048 MHz fCLK */

  usleep(1000);

  /* Send null command to read the command response */

  opcode = ADS131_OPCODE_NULL;
  num_bytes = ads131_build_spi_msg(priv, &opcode, 1, tx_data);

  SPI_SELECT(spi, SPI_DEV, true);
  SPI_EXCHANGE(spi, tx_data, rx_data, num_bytes);
  SPI_SELECT(spi, SPI_DEV, false);

  uint16_t adc_response = ads131_combine_bytes(rx_data[0], rx_data[1]);

  /* NOTE: The ADS131M0x's response word should be (0xff20 | CHANCNT),
   * if the response is 0x0011 (acknowledge of RESET command), then device
   * did not receive a full SPI frame and the reset did not occur!
   */

  if (adc_response != (0xff20 | ADS131_CHANCNT(priv)))
    {
      aerr("%s: ERROR - Device reset failed. 0x%" PRIx16, __func__,
           adc_response);
    }

  /* Update register setting array to keep software in sync with device */

  ads131_restore_register_defaults(priv);

  /* Check STATUS register that reset was successful. */

  if ((ret = ads131_is_reset_ok(dev)) != OK)
    {
      aerr("%s: ERROR - Device reset failed.", __func__);
      return ret;
    }

  /* Write to MODE register to enforce mode settings */

  ads131_write_single_register(
      dev, ADS131_MODE_ADDRESS, ADS131_MODE_DEFAULT);

  return ret;
}

static void ads131_shutdown(struct adc_dev_s *dev)
{
  struct ads131_dev_s *priv = (struct ads131_dev_s *)dev->ad_priv;
  nxmutex_lock(&priv->devlock);

  atrace("%s: Shutting down.  Refs = %" PRId16, __func__, priv->refs);

  DEBUGASSERT(priv->refs);
  priv->refs--;

  nxmutex_unlock(&priv->devlock);
}

static void ads131_rxint(struct adc_dev_s *dev, bool enable)
{
  FAR struct ads131_dev_s *priv = (FAR struct ads131_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);

  if (enable)
    {
      //up_enable_irq(priv->irq);
    }
  else
    {
      //up_disable_irq(priv->irq);
    }
}

static int ads131_ioctl(FAR struct adc_dev_s *dev, int cmd,
                        unsigned long arg)
{
  int ret;
  struct ads131_dev_s *priv = (struct ads131_dev_s *)dev->ad_priv;

  switch (cmd)
    {
      case ANIOC_ADS131_READ_CHANNELS:
        int32_t *samples = (int32_t *)arg;
        nxmutex_lock(&priv->devlock);

#if 0
        uint16_t drdy;
        do {
          drdy = ads131_read_single_register(dev, ADS131_STATUS_ADDRESS) & 0x03;
        } while(drdy != 0x03);
#endif

        ret = ads131_get_samples(dev, samples, ADS131_NUM_CHANNELS);
        nxmutex_unlock(&priv->devlock);
        break;

      case ANIOC_TRIGGER:
        nxmutex_lock(&priv->devlock);
        ret = ads131_trigger_sample(priv);
        nxmutex_unlock(&priv->devlock);
        break;

      default:
        ret = -EINVAL;
    }

  return ret;
}

/****************************************************************************
 *
 * Takes a 16-bit word and returns the most-significant byte.
 *
 *  \fn uint8_t ads131_upper_byte(uint16_t temp_word)
 *
 *  \param temp_word is the original 16-bit word.
 *
 *  \return 8-bit most-significant byte.
 ****************************************************************************/

uint8_t ads131_upper_byte(uint16_t temp_word)
{
  uint8_t msb;
  msb = (uint8_t)((temp_word >> 8) & 0x00ff);

  return msb;
}

/****************************************************************************
 *  Takes a 16-bit word and returns the least-significant byte.
 *
 *  \fn uint8_t ads131_lower_byte(uint16_t temp_word)
 *
 *  \param temp_word is the original 16-bit word.
 *
 *  \return 8-bit least-significant byte.
 ****************************************************************************/

uint8_t ads131_lower_byte(uint16_t temp_word)
{
  uint8_t lsb;
  lsb = (uint8_t)(temp_word & 0x00ff);

  return lsb;
}

/****************************************************************************
 *  Builds SPI TX data arrays according to number of opcodes provided and
 *  currently programmed device word length.
 *
 *  \fn uint8_t ads131_build_spi_msg(struct ads131_dev_s *priv,
 *          const uint16_t opcode_array[],
 *          uint8_t num_opcodes,
 *          uint8_t byte_array[])
 *
 *  \param opcode_array[] pointer to an array of 16-bit opcodes to use in
 *                        the SPI command.
 *  \param num_opcodes the number of opcodes provided in opcode_array[].
 *  \param byte_array[] pointer to an array of 8-bit SPI bytes to send to
 *                      the device.
 *
 *  NOTE: The calling function must ensure it reserves sufficient memory for
 *        byte_array[]!
 *
 *  \return number of bytes added to byte_array[].
 ****************************************************************************/

static uint8_t ads131_build_spi_msg(struct ads131_dev_s *priv,
                                    const uint16_t opcode_array[],
                                    uint8_t num_opcodes,
                                    uint8_t *byte_array)
{
  /* Frame size = opcode word(s) + optional CRC word
   * Number of bytes per word = 2, 3, or 4
   * Total bytes = bytes per word * number of words
   */

  uint8_t num_words = num_opcodes + (ADS131_SPI_CRC_ENABLED(priv) ? 1 : 0);
  uint8_t byte_per_word = ads131_get_word_length(priv);
  uint8_t num_bytes = num_words * byte_per_word;

  int i;
  for (i = 0; i < num_opcodes; i++)
    {
      /* NOTE: Be careful not to accidentally overflow the array here.
       * The array and opcodes are defined in the calling function, so
       * we are trusting that no mistakes were made in the calling function!
       */

      byte_array[(i * byte_per_word) + 0]
          = ads131_upper_byte(opcode_array[i]);
      byte_array[(i * byte_per_word) + 1]
          = ads131_lower_byte(opcode_array[i]);
    }

#ifdef ENABLE_CRC_IN
  /* Calculate CRC and put it into TX array */

  uint16_t crc_word = ads131_calc_ctc(&byte_array[0], num_bytes, 0xffff);
  byte_array[(i * byte_per_word) + 0] = ads131_upper_byte(crc_word);
  byte_array[(i * byte_per_word) + 1] = ads131_lower_byte(crc_word);
#endif

  return num_bytes;
}

/****************************************************************************
 *  Getter function to access priv->register_map array from outside of this
 *  module.
 *
 *  \fn uint16_t ads131_get_reg_val(
 *          struct ads131_dev_s *priv, uint8_t address)
 *
 *  NOTE: The internal priv->register_map arrays stores the last known
 *  register value, since the last read or write operation to that register.
 *  This function does not communicate with the device to retrieve the
 *  current register value.
 *  For the most up-to-date register data or retrieving the value of a
 *  hardware controlled register, it is recommend to use
 *  ads131_read_single_register() to read the current register value.
 *
 *  \return unsigned 16-bit register value.
 ****************************************************************************/

static uint16_t ads131_get_reg_val(struct ads131_dev_s *priv,
                                   uint8_t address)
{
  DEBUGASSERT(address < ADS131_NUM_REGISTERS);
  return priv->register_map[address];
}

/****************************************************************************
 *  Returns the ADS131M0x configured word length used for SPI communication.
 *
 *  \fn uint8_t ads131_get_word_length(struct ads131_dev_s *priv)
 *
 *  NOTE: It is important that the MODE register value stored in
 *  priv->register_map[] remains in sync with the device.
 *  If these values get out of sync then SPI communication may fail!
 *
 *  \return SPI word byte length (2, 3, or 4)
 ****************************************************************************/

static uint8_t ads131_get_word_length(struct ads131_dev_s *priv)
{
  return g_wlength_byte_values[ADS131_WLENGTH(priv)];
}

/****************************************************************************
 *
 *  Takes two 8-bit words and returns a concatenated 16-bit word.
 *
 *  \fn uint16_t ads131_combine_bytes(uint8_t upper_byte, uint8_t
 *lower_byte)
 *
 *  \param upper_byte is the 8-bit value that will become the
 *                    MSB of the 16-bit word.
 *  \param lower_byte is the 8-bit value that will become the
 *                    LSB of the 16-bit word.
 *
 *  \return concatenated 16-bit word.
 *
 ****************************************************************************/

static uint16_t ads131_combine_bytes(uint8_t upper_byte, uint8_t lower_byte)
{
  uint16_t combined_value;
  combined_value = ((uint16_t)upper_byte << 8) | ((uint16_t)lower_byte);

  return combined_value;
}

/****************************************************************************
 *
 *  Sends the LOCK command and verifies that registers are locked.
 *
 *  \fn bool ads131_lock_regs(struct adc_dev_s *dev)
 *
 *  \return boolean to indicate if an error occurred
 *          (0 = no error; 1 = error)
 *
 ****************************************************************************/

bool ads131_lock_regs(struct adc_dev_s *dev)
{
  struct ads131_dev_s *priv = (struct ads131_dev_s *)dev->ad_priv;
  struct spi_dev_s *spi = priv->spi;
  bool b_lock_error;

  /* Build TX and RX byte array */

  /* clang-format off */

#ifdef ENABLE_CRC_IN
  /* 2 words, up to 4 bytes each = 8 bytes max */

  uint8_t tx_data[8] =
    {
      0
    };

  uint8_t rx_data[8] =
    {
      0
    };

#else
  /* 1 word, up to 4 bytes long = 4 bytes max */

  uint8_t tx_data[4] =
    {
      0
    };

  uint8_t rx_data[4] =
    {
      0
    };

  /* clang-format on */

#endif
  uint16_t opcode = ADS131_OPCODE_LOCK;
  uint8_t num_bytes = ads131_build_spi_msg(priv, &opcode, 1, tx_data);

  /* Send command */

  SPI_SELECT(spi, SPI_DEV, true);
  SPI_EXCHANGE(spi, tx_data, rx_data, num_bytes);
  SPI_SELECT(spi, SPI_DEV, false);

  /* (OPTIONAL) Check for SPI errors by sending the NULL command and
   * checking STATUS
   */

  /* Read back the STATUS register and check if LOCK bit is set...
   */

  ads131_read_single_register(dev, ADS131_STATUS_ADDRESS);

  if (!ADS131_SPI_LOCKED(priv))
    {
      b_lock_error = true;
    }

  /* (OPTIONAL) Error handler */

  if (b_lock_error)
    {
      /* TODO - Insert error handler function call here... */
    }

  return b_lock_error;
}

/****************************************************************************
 *
 *  Sends the UNLOCK command and verifies that registers are unlocked
 *
 *  \fn bool ads131_unlock_regs(struct adc_dev_s *dev)
 *
 *  \return boolean to indicate if an error occurred
 *          (0 = no error; 1 = error)
 *
 ****************************************************************************/

bool ads131_unlock_regs(struct adc_dev_s *dev)
{
  struct ads131_dev_s *priv = (struct ads131_dev_s *)dev->ad_priv;
  struct spi_dev_s *spi = priv->spi;
  bool b_unlock_error;

  /* Build TX and RX byte array */

#ifdef ENABLE_CRC_IN
  /* 2 words, up to 4 bytes each = 8 bytes max */

  /* clang-format off */

  uint8_t tx_data[8] =
    {
      0
    };

  uint8_t rx_data[8] =
    {
      0
    };

#else
  /* 1 word, up to 4 bytes long = 4 bytes max */

  uint8_t tx_data[4] =
    {
      0
    };

  uint8_t rx_data[4] =
    {
      0
    };

  /* clang-format on */

#endif
  uint16_t opcode = ADS131_OPCODE_UNLOCK;
  uint8_t num_bytes = ads131_build_spi_msg(priv, &opcode, 1, tx_data);

  /* Send command */

  SPI_SELECT(spi, SPI_DEV, true);
  SPI_EXCHANGE(spi, tx_data, rx_data, num_bytes);
  SPI_SELECT(spi, SPI_DEV, false);

  /* (OPTIONAL) Check for SPI errors by sending the NULL command and
   * checking STATUS
   */

  /* (OPTIONAL) Read the STATUS register and check if LOCK bit is cleared.
   */

  ads131_read_single_register(dev, ADS131_STATUS_ADDRESS);
  if (ADS131_SPI_LOCKED(priv))
    {
      b_unlock_error = true;
    }

  /* If the STATUS register is NOT read back,
   * then make sure to manually update the global register map variable...
   */

  /* priv->register_map[ADS131_STATUS_ADDRESS]  &=
   *   !ADS131_STATUS_LOCK_LOCKED;
   */

  /* (OPTIONAL) Error handler */

  if (b_unlock_error)
    {
      /* TODO - Insert error handler function call here... */
    }

  return b_unlock_error;
}

/****************************************************************************
 *  Reads the contents of a single register at the specified address.
 *
 *  \fn uint16_t ads131_read_single_register(
 *          struct adc_dev_s *dev, uint8_t address)
 *
 *  \param address is the 8-bit address of the register to read.
 *
 *  \return Returns the 8-bit register read result.
 ****************************************************************************/

static uint16_t ads131_read_single_register(struct adc_dev_s *dev,
                                            uint8_t address)
{
  struct ads131_dev_s *priv = (struct ads131_dev_s *)dev->ad_priv;
  struct spi_dev_s *spi = priv->spi;

  /* Check that the register address is in range */

  DEBUGASSERT(address < ADS131_NUM_REGISTERS);

  /* Build TX and RX byte array */

  /* clang-format off */

#ifdef ADS131_ENABLE_CRC_IN
  /* 2 words, up to 4 bytes each = 8 bytes max */

  uint8_t tx_data[8] =
    {
      0
    };

  uint8_t rx_data[8] =
    {
      0
    };

#else
  /* 1 word, up to 4 bytes long = 4 bytes max  */

  uint8_t tx_data[8] =
    {
      0
    };

  uint8_t rx_data[8] =
    {
      0
    };

#endif

  uint16_t opcode[2] =
    {
      (ADS131_OPCODE_RREG | (((uint16_t) address) << 7)),
      ADS131_OPCODE_NULL
    };

  /* clang-format on */

  uint8_t num_bytes = ads131_build_spi_msg(priv, &opcode[0], 1, tx_data);

  /* [FRAME 1] Send RREG command */

  SPI_SELECT(spi, SPI_DEV, true);
  SPI_EXCHANGE(spi, tx_data, rx_data, num_bytes);
  SPI_SELECT(spi, SPI_DEV, false);

  /* [FRAME 2] Send NULL command to retrieve the register data */

  num_bytes = ads131_build_spi_msg(priv, &opcode[1], 1, tx_data);

  SPI_SELECT(spi, SPI_DEV, true);
  SPI_EXCHANGE(spi, tx_data, rx_data, num_bytes);
  SPI_SELECT(spi, SPI_DEV, false);

  priv->register_map[address]
      = ads131_combine_bytes(rx_data[0], rx_data[1]);

  atrace("%s: Register 0x%x: 0x%x", __func__, address,
         priv->register_map[address]);

  return priv->register_map[address];
}

/****************************************************************************
 *
 *  Combines ADC data bytes into a single signed 32-bit word.
 *
 *  \fn int32_t combineDataBytes(const uint8_t data_bytes[])
 *
 *  \param data_bytes pointer to uint8_t[] where first element is the MSB.
 *
 *  \return Returns the signed-extend 32-bit result.
 *
 ****************************************************************************/

static int32_t ads131_sign_extend(const uint8_t data_bytes[])
{
#ifdef ADS131_WORD_LENGTH_24BIT
  int32_t upper_byte = ((int32_t)data_bytes[0] << 24);
  int32_t middle_byte = ((int32_t)data_bytes[1] << 16);
  int32_t lower_byte = ((int32_t)data_bytes[2] << 8);

  /* Right-shift of signed data maintains signed bit */

  return (((int32_t)(upper_byte | middle_byte | lower_byte)) >> 8);

#elif defined ADS131_WORD_LENGTH_32BIT_SIGN_EXTEND
  int32_t sign_byte = ((int32_t)data_bytes[0] << 24);
  int32_t upper_byte = ((int32_t)data_bytes[1] << 16);
  int32_t middle_byte = ((int32_t)data_bytes[2] << 8);
  int32_t lower_byte = ((int32_t)data_bytes[3] << 0);

  return (sign_byte | upper_byte | middle_byte | lower_byte);

#elif defined ADS131_WORD_LENGTH_32BIT_ZERO_PADDED
  int32_t upper_byte = ((int32_t)data_bytes[0] << 24);
  int32_t middle_byte = ((int32_t)data_bytes[1] << 16);
  int32_t lower_byte = ((int32_t)data_bytes[2] << 8);

  /* Right-shift of signed data maintains signed bit */

  return (((int32_t)(upper_byte | middle_byte | lower_byte)) >> 8);

#elif defined ADS131_WORD_LENGTH_16BIT_TRUNCATED
  int32_t upper_byte = ((int32_t)data_bytes[0] << 24);
  int32_t lower_byte = ((int32_t)data_bytes[1] << 16);

  /* Right-shift of signed data maintains signed bit */

  return (((int32_t)(upper_byte | lower_byte)) >> 16);
#endif
}

/****************************************************************************
 *
 *  Reads ADC data.
 *
 *  \fn bool ads131_read_data(struct adc_dev_s *dev,
 *          ads131_adc_channel_data_s *adc_data)
 *
 *  \param *adc_data points to an ads131_adc_channel_data_s structure
 *
 *  NOTE: Should be called after /DRDY goes low,
 *        and not during a /DRDY falling edge!
 *
 *  \return Returns true if the CRC-OUT of the data read detects an error.
 *
 ****************************************************************************/

bool ads131_read_data(struct adc_dev_s *dev,
                      struct ads131_adc_channel_data_s *adc_data)
{
  struct ads131_dev_s *priv = (struct ads131_dev_s *)dev->ad_priv;
  struct spi_dev_s *spi = priv->spi;

  /* clang-format off */

  uint8_t rx_data[16] =
    {
      0
    };

  uint8_t bytes_per_word = ads131_get_word_length(priv);

#ifdef ENABLE_CRC_IN
  /* Build CRC word (only if "RX_CRC_EN" register bit is enabled) */

  uint8_t tx_data[16] =
    {
      0
    };

  /* clang-format on */

  uint16_t crc_word_in
      = ads131_calc_ctc(&tx_data[0], bytes_per_word * 2, 0xffff);
  tx_crc[0] = ads131_upper_byte(crc_word_in);
  tx_crc[1] = ads131_lower_byte(crc_word_in);
#endif

  /* Send NULL word, receive:
   *  - 2 bytes status register + 0x00 padding
   *  - Channel 0 result
   *  - Channel 1 result
   *  - 2 bytes CRC + 0x00 padding
   */

  SPI_SELECT(spi, SPI_DEV, true);
  SPI_RECVBLOCK(spi, rx_data, bytes_per_word * 4u);
  SPI_SELECT(spi, SPI_DEV, false);

  adc_data->response = ads131_combine_bytes(rx_data[0], rx_data[1]);

  /* (OPTIONAL) Do something with the response (STATUS) word.
   * ...Here we only use the response for calculating the CRC-OUT
   */

  /* uint16_t crc_word = ads131_calc_ctc(
   *    &rx_data[0], bytes_per_word, 0xffff);
   */

  /* (OPTIONAL) Ignore CRC error checking */

  uint16_t crc_word = 0;

  /* 2nd word, receive channel 1 data */

  adc_data->channel0 = ads131_sign_extend(&rx_data[3]);

  /* crc_word = ads131_calc_ctc(&rx_data[0], bytes_per_word, crc_word); */

  /* 3rd word, receive channel 2 data */

  adc_data->channel1 = ads131_sign_extend(&rx_data[6]);

  /* crc_word = ads131_calc_ctc(&rx_data[0], bytes_per_word, crc_word); */

  /* The next word, CRC data */

  adc_data->crc = ads131_combine_bytes(rx_data[9], rx_data[10]);

  /* NOTE: If we continue calculating the CRC with a matching CRC,
   * the result should be zero.
   * Any non-zero result will indicate a mismatch.
   */

  /* crc_word = ads131_calc_ctc(&rx_data[0], bytes_per_word, crc_word); */

  /* Returns true when a CRC error occurs */

  return ((bool)crc_word);
}

static int ads131_get_samples(struct adc_dev_s *dev, int32_t *samples,
                              size_t count)
{
  struct ads131_adc_channel_data_s adc_data;

  ads131_read_data(dev, &adc_data);

  samples[0] = adc_data.channel0;
  samples[1] = adc_data.channel1;

  atrace("%s: ADC Channel 0: 0x%" PRIx32 ". Channel 1: 0x%" PRIx32 ".",
         __func__, adc_data.channel0, adc_data.channel1);

  return OK;
}

static void ads131_worker(FAR void *arg)
{
  syslog(LOG_ERR, "ads131_worker");
}

static int ads131_interrupt(int irq, FAR void *context, FAR void *arg)
{
  syslog(LOG_ERR, "ads131_interrupt");
  //FAR struct ads131_dev_s *priv =
  //  (FAR struct ads131_dev_s *)g_adcdev.ad_priv;
//
  //DEBUGASSERT(priv != NULL);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct adc_dev_s *ads131_initialize(struct ads131_config_s *config, unsigned int devno)
{
  int ret;

  atrace("%s Entered.", __func__);

  DEBUGASSERT(config->spi != NULL);

  struct ads131_dev_s *ads131_priv
      = (struct ads131_dev_s *)kmm_zalloc(sizeof(struct ads131_dev_s));
  if (!ads131_priv)
    {
      aerr("failed to allocate memory for struct ads131_dev_s");
      return NULL;
    }

  struct adc_dev_s *dev
      = (struct adc_dev_s *)kmm_zalloc(sizeof(struct adc_dev_s));
  if (!dev)
    {
      aerr("failed to allocate memory for struct adc_dev_s");
      kmm_free(ads131_priv);
      return NULL;
    }

  ads131_priv->config = config;
  ads131_priv->spi = config->spi;
  ads131_priv->devno = devno;
  ads131_priv->frequency = config->frequency;
  ads131_priv->reset_gpio = 0;

  ret = nxmutex_init(&ads131_priv->devlock);
  if (ret != OK)
    {
      aerr("nxmutex_init failed %d", ret);
      kmm_free(ads131_priv);
      kmm_free(dev);
      return NULL;
    }

  dev->ad_priv = ads131_priv;
  dev->ad_ops = &ads131_ops_s;

  //DEBUGASSERT(ads131_priv->config->irq_attach != NULL);
  //ads131_priv->config->irq_attach(ads131_priv->config, ads131_interrupt, ads131_priv);

  //DEBUGASSERT(ads131_priv->config->irq_enable != NULL);
  //ads131_priv->config->irq_enable(ads131_priv->config, false);

  ads131_restore_register_defaults(ads131_priv);

  return dev;
}
