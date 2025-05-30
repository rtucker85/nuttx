/****************************************************************************
 * drivers/include/analog/ads131.h
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_ANALOG_ADS131_H
#define __INCLUDE_NUTTX_ANALOG_ADS131_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

//#include <analog/ads131.h>
#include <nuttx/analog/adc.h>
#include <nuttx/config.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>
#include <nuttx/analog/ioctl.h>
#include <stdbool.h>

//#include <m1/ioctl.h>

/****************************************************************************
 * Macros and Constants Definitions
 ****************************************************************************/

#define ANIOC_ADS131_READ_CHANNELS  _ANIOC(AN_ADS131_FIRST + 0)
#define ANIOC_ADS131_TRIGGER_SAMPLE _ANIOC(AN_ADS131_FIRST + 1)

#define ADS131_NUM_CHANNELS         (2)

#define ADS131_NUM_REGISTERS        ((uint8_t)64)

/* Enable this define statement to use CRC on DIN... */

/* #define ADS131_ENABLE_CRC_IN */

/* Pick one (and only one) mode to use:
 *   ADS131_WORD_LENGTH_16BIT_TRUNCATED
 *   ADS131_WORD_LENGTH_24BIT
 *   ADS131_WORD_LENGTH_32BIT_SIGN_EXTEND
 *   ADS131_WORD_LENGTH_32BIT_ZERO_PADDED
 */

#define ADS131_WORD_LENGTH_24BIT

/****************************************************************************
 *
 * SPI command opcodes
 *
 ****************************************************************************/

#define ADS131_OPCODE_NULL    ((uint16_t)0x0000)
#define ADS131_OPCODE_RESET   ((uint16_t)0x0011)
#define ADS131_OPCODE_RREG    ((uint16_t)0xA000)
#define ADS131_OPCODE_WREG    ((uint16_t)0x6000)
#define ADS131_OPCODE_STANDBY ((uint16_t)0x0022)
#define ADS131_OPCODE_WAKEUP  ((uint16_t)0x0033)
#define ADS131_OPCODE_LOCK    ((uint16_t)0x0555)
#define ADS131_OPCODE_UNLOCK  ((uint16_t)0x0655)

/****************************************************************************
 *
 * Register macros
 *
 ****************************************************************************/

/* NOTE: Whenever possible, macro names (defined below) were derived from
 * datasheet defined names; however, updates to documentation may cause
 * mismatches between names defined here in example code from those shown
 * in the device datasheet.
 */

/* Register 0x00 (ID) definition - READ ONLY
 * -----------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |
 * -----------------------------------------------------
 * |                    RESERVED[3:0]                  |
 * -----------------------------------------------------
 *
 * -----------------------------------------------------
 * |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |
 * -----------------------------------------------------
 * |                    CHANCNT[3:0]                   |
 * -----------------------------------------------------
 *
 * -----------------------------------------------------
 * |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |
 * -----------------------------------------------------
 * |                    REVID[7:0]
 * -----------------------------------------------------
 *
 * -----------------------------------------------------
 * |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------
 *                      REVID[7:0]                     |
 * -----------------------------------------------------
 *
 */

/* ID register address & default value */

#define ADS131_ID_ADDRESS     ((uint8_t)0x00)

/* NOTE: ID May change with future device revisions! */

#define ADS131_ID_DEFAULT(CHANNEL_COUNT)                                   \
  ((uint16_t)0x2000 | (CHANNEL_COUNT << 8))

/* RESERVED field mask */

#define ADS131_ID_RESERVED_MASK                    ((uint16_t)0xF000)

/* CHANCNT field mask & values */

#define ADS131_ID_CHANCNT_MASK                     ((uint16_t)0x0F00)
#define ADS131_ID_CHANCNT_2CH                      ((uint16_t)0x0002 << 8)
#define ADS131_ID_CHANCNT_4CH                      ((uint16_t)0x0004 << 8)
#define ADS131_ID_CHANCNT_6CH                      ((uint16_t)0x0006 << 8)
#define ADS131_ID_CHANCNT_8CH                      ((uint16_t)0x0008 << 8)

/* REVID field mask & values */

#define ADS131_ID_REVID_MASK                       ((uint16_t)0x00FF)
#define ADS131_ID_REVID_REVA                       ((uint16_t)0x0000 << 0)

/* Register 0x01 (STATUS) definition - READ ONLY
 * -----------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |
 * -----------------------------------------------------
 * |    LOCK    |  F_RESYNC  |   REG_MAP  |   CRC_ERR  |
 * -----------------------------------------------------
 *
 * -----------------------------------------------------
 * |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |
 * -----------------------------------------------------
 * |  CRC_TYPE  |    RESET   |       WLENGTH[1:0]      |
 * -----------------------------------------------------
 *
 * -----------------------------------------------------
 * |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |
 * -----------------------------------------------------
 * |    DRDY7   |    DRDY6   |    DRDY5   |    DRDY4   |
 * -----------------------------------------------------
 *
 * -----------------------------------------------------
 * |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------
 * |    DRDY3   |    DRDY2   |    DRDY1   |    DRDY0   |
 * -----------------------------------------------------
 *
 *  NOTE 1: Bits 0 through 7 are hardware controlled.
 *          Reading these values multiple times may return different
 * results. NOTE 2: Bits 0 through 4 are RESERVED on the ADS131M04. These
 * bits will always read 0.
 */

/* STATUS register address & default value */

#define ADS131_STATUS_ADDRESS                      ((uint8_t)0x01)
#define ADS131_STATUS_DEFAULT                      ((uint16_t)0x0500)

/* LOCK field mask & values */

#define ADS131_STATUS_LOCK_MASK                    ((uint16_t)0x8000)
#define ADS131_STATUS_LOCK_UNLOCKED                ((uint16_t)0x0000 << 15)
#define ADS131_STATUS_LOCK_LOCKED                  ((uint16_t)0x0001 << 15)

/* F_RESYNC field mask & values */

#define ADS131_STATUS_F_RESYNC_MASK                ((uint16_t)0x4000)
#define ADS131_STATUS_F_RESYNC_NO_FAULT            ((uint16_t)0x0000 << 14)
#define ADS131_STATUS_F_RESYNC_FAULT               ((uint16_t)0x0001 << 14)

/* REG_MAP field mask & values */

#define ADS131_STATUS_REG_MAP_MASK                 ((uint16_t)0x2000)
#define ADS131_STATUS_REG_MAP_NO_CHANGE_CRC        ((uint16_t)0x0000 << 13)
#define ADS131_STATUS_REG_MAP_CHANGED_CRC          ((uint16_t)0x0001 << 13)

/* CRC_ERR field mask & values */

#define ADS131_STATUS_CRC_ERR_MASK                 ((uint16_t)0x1000)
#define ADS131_STATUS_CRC_ERR_NO_CRC_ERROR         ((uint16_t)0x0000 << 12)
#define ADS131_STATUS_CRC_ERR_INPUT_CRC_ERROR      ((uint16_t)0x0001 << 12)

/* CRC_TYPE field mask & values */

#define ADS131_STATUS_CRC_TYPE_MASK                ((uint16_t)0x0800)
#define ADS131_STATUS_CRC_TYPE_16BIT_CCITT         ((uint16_t)0x0000 << 11)
#define ADS131_STATUS_CRC_TYPE_16BIT_ANSI          ((uint16_t)0x0001 << 11)

/* RESET field mask & values */

#define ADS131_STATUS_RESET_MASK                   ((uint16_t)0x0400)
#define ADS131_STATUS_RESET_NO_RESET               ((uint16_t)0x0000 << 10)
#define ADS131_STATUS_RESET_RESET_OCCURRED         ((uint16_t)0x0001 << 10)

/* WLENGTH field mask & values */

#define ADS131_STATUS_WLENGTH_MASK                 ((uint16_t)0x0300)
#define ADS131_STATUS_WLENGTH_16BIT                ((uint16_t)0x0000 << 8)
#define ADS131_STATUS_WLENGTH_24BIT                ((uint16_t)0x0001 << 8)
#define ADS131_STATUS_WLENGTH_32BIT_LSB_ZEROES     ((uint16_t)0x0002 << 8)
#define ADS131_STATUS_WLENGTH_32BIT_MSB_SIGN_EXT   ((uint16_t)0x0003 << 8)

/* DRDY1 field mask & values */

#define ADS131_STATUS_DRDY1_MASK                   ((uint16_t)0x0002)
#define ADS131_STATUS_DRDY1_NO_NEW_DATA            ((uint16_t)0x0000 << 1)
#define ADS131_STATUS_DRDY1_NEW_DATA               ((uint16_t)0x0001 << 1)

/* DRDY0 field mask & values */

#define ADS131_STATUS_DRDY0_MASK                   ((uint16_t)0x0001)
#define ADS131_STATUS_DRDY0_NO_NEW_DATA            ((uint16_t)0x0000 << 0)
#define ADS131_STATUS_DRDY0_NEW_DATA               ((uint16_t)0x0001 << 0)

/* Register 0x02 (MODE) definition
 *
 * -----------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |
 * -----------------------------------------------------
 * |      RESERVED0[1:0]     | REG_CRC_EN |  RX_CRC_EN |
 * -----------------------------------------------------
 *
 * -----------------------------------------------------
 * |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |
 * -----------------------------------------------------
 * |  CRC_TYPE  |    RESET   |       WLENGTH[1:0]      |
 * -----------------------------------------------------
 *
 * -----------------------------------------------------
 * |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |
 * -----------------------------------------------------
 * |             RESERVED1[2:0]           |   TIMEOUT  |
 * -----------------------------------------------------
 *
 * -----------------------------------------------------
 * |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------
 * |       DRDY_SEL[1:0]     |  DRDY_HiZ  |  DRDY_FMT  |
 * -----------------------------------------------------
 *
 */

/* MODE register address & default value */

#define ADS131_MODE_ADDRESS                        ((uint8_t)0x02)
#define ADS131_MODE_DEFAULT                        ((uint16_t)0x0110)

/* RESERVED0 field mask */

#define ADS131_MODE_RESERVED0_MASK                 ((uint16_t)0xC000)

/* REG_CRC_EN field mask & values */

#define ADS131_MODE_REG_CRC_EN_MASK                ((uint16_t)0x2000)
#define ADS131_MODE_REG_CRC_EN_DISABLED            ((uint16_t)0x0000 << 13)
#define ADS131_MODE_REG_CRC_EN_ENABLED             ((uint16_t)0x0001 << 13)

/* RX_CRC_EN field mask & values */

#define ADS131_MODE_RX_CRC_EN_MASK                 ((uint16_t)0x1000)
#define ADS131_MODE_RX_CRC_EN_DISABLED             ((uint16_t)0x0000 << 12)
#define ADS131_MODE_RX_CRC_EN_ENABLED              ((uint16_t)0x0001 << 12)

/* CRC_TYPE field mask & values */

#define ADS131_MODE_CRC_TYPE_MASK                  ((uint16_t)0x0800)
#define ADS131_MODE_CRC_TYPE_16BIT_CCITT           ((uint16_t)0x0000 << 11)
#define ADS131_MODE_CRC_TYPE_16BIT_ANSI            ((uint16_t)0x0001 << 11)

/* RESET field mask & values */

#define ADS131_MODE_RESET_MASK                     ((uint16_t)0x0400)
#define ADS131_MODE_RESET_NO_RESET                 ((uint16_t)0x0000 << 10)
#define ADS131_MODE_RESET_RESET_OCCURRED           ((uint16_t)0x0001 << 10)

/* WLENGTH field mask & values */

#define ADS131_MODE_WLENGTH_MASK                   ((uint16_t)0x0300)
#define ADS131_MODE_WLENGTH_16BIT                  ((uint16_t)0x0000 << 8)
#define ADS131_MODE_WLENGTH_24BIT                  ((uint16_t)0x0001 << 8)
#define ADS131_MODE_WLENGTH_32BIT_LSB_ZEROES       ((uint16_t)0x0002 << 8)
#define ADS131_MODE_WLENGTH_32BIT_MSB_SIGN_EXT     ((uint16_t)0x0003 << 8)

/* RESERVED1 field mask */

#define ADS131_MODE_RESERVED1_MASK                 ((uint16_t)0x00E0)

/* TIMEOUT field mask & values */

#define ADS131_MODE_TIMEOUT_MASK                   ((uint16_t)0x0010)
#define ADS131_MODE_TIMEOUT_DISABLED               ((uint16_t)0x0000 << 4)
#define ADS131_MODE_TIMEOUT_ENABLED                ((uint16_t)0x0001 << 4)

/* DRDY_SEL field mask & values */

#define ADS131_MODE_DRDY_SEL_MASK                  ((uint16_t)0x000C)
#define ADS131_MODE_DRDY_SEL_MOST_LAGGING          ((uint16_t)0x0000 << 2)
#define ADS131_MODE_DRDY_SEL_LOGIC_OR              ((uint16_t)0x0001 << 2)
#define ADS131_MODE_DRDY_SEL_MOST_LEADING          ((uint16_t)0x0002 << 2)

/* DRDY_HiZ field mask & values */

#define ADS131_MODE_DRDY_HiZ_MASK                  ((uint16_t)0x0002)
#define ADS131_MODE_DRDY_HiZ_LOGIC_HIGH            ((uint16_t)0x0000 << 1)
#define ADS131_MODE_DRDY_HiZ_HIGH_IMPEDANCE        ((uint16_t)0x0001 << 1)

/* DRDY_FMT field mask & values */

#define ADS131_MODE_DRDY_FMT_MASK                  ((uint16_t)0x0001)
#define ADS131_MODE_DRDY_FMT_LOGIC_LOW             ((uint16_t)0x0000 << 0)
#define ADS131_MODE_DRDY_FMT_NEG_PULSE_FIXED_WIDTH ((uint16_t)0x0001 << 0)

/* CLOCK register address & default value */

#define ADS131_CLOCK_ADDRESS                       ((uint8_t)0x03)
#define ADS131_CLOCK_DEFAULT                       ((uint16_t)0x030E)

#define ADS131_CLOCK_OSR_MASK                      ((uint16_t)0x001C)
#define ADS131_CLOCK_OSR_128                       ((uint16_t)0x0000 << 2)
#define ADS131_CLOCK_OSR_256                       ((uint16_t)0x0001 << 2)
#define ADS131_CLOCK_OSR_512                       ((uint16_t)0x0002 << 2)
#define ADS131_CLOCK_OSR_1024                      ((uint16_t)0x0003 << 2)
#define ADS131_CLOCK_OSR_2048                      ((uint16_t)0x0004 << 2)
#define ADS131_CLOCK_OSR_4096                      ((uint16_t)0x0005 << 2)
#define ADS131_CLOCK_OSR_8192                      ((uint16_t)0x0006 << 2)
#define ADS131_CLOCK_OSR_16384                     ((uint16_t)0x0007 << 2)

/* GAIN1 register address & default value */

#define ADS131_GAIN1_ADDRESS                       ((uint8_t)0x04)
#define ADS131_GAIN1_DEFAULT                       ((uint16_t)0x0000)

/* GAIN2 register address & default value */

#define ADS131_GAIN2_ADDRESS                       ((uint8_t)0x05)
#define ADS131_GAIN2_DEFAULT                       ((uint16_t)0x0000)

/* CFG register address & default value */

#define ADS131_CFG_ADDRESS                         ((uint8_t)0x06)
#define ADS131_CFG_DEFAULT                         ((uint16_t)0x0600)

/* THRSHLD_MSB register address & default value */

#define ADS131_THRSHLD_MSB_ADDRESS                 ((uint8_t)0x07)
#define ADS131_THRSHLD_MSB_DEFAULT                 ((uint16_t)0x0000)

/* THRSHLD_LSB register address & default value */

#define ADS131_THRSHLD_LSB_ADDRESS                 ((uint8_t)0x08)
#define ADS131_THRSHLD_LSB_DEFAULT                 ((uint16_t)0x0000)

/* CH0_CFG register address & default value */

#define ADS131_CH0_CFG_ADDRESS                     ((uint8_t)0x09)
#define ADS131_CH0_CFG_DEFAULT                     ((uint16_t)0x0000)

/* CH0_OCAL_MSB register address & default value */

#define ADS131_CH0_OCAL_MSB_ADDRESS                ((uint8_t)0x0A)
#define ADS131_CH0_OCAL_MSB_DEFAULT                ((uint16_t)0x0000)

/* CH0_OCAL_LSB register address & default value */

#define ADS131_CH0_OCAL_LSB_ADDRESS                ((uint8_t)0x0B)
#define ADS131_CH0_OCAL_LSB_DEFAULT                ((uint16_t)0x0000)

/* CH0_GCAL_MSB register address & default value */

#define ADS131_CH0_GCAL_MSB_ADDRESS                ((uint8_t)0x0C)
#define ADS131_CH0_GCAL_MSB_DEFAULT                ((uint16_t)0x8000)

/* CH0_GCAL_LSB register address & default value */

#define ADS131_CH0_GCAL_LSB_ADDRESS                ((uint8_t)0x0D)
#define ADS131_CH0_GCAL_LSB_DEFAULT                ((uint16_t)0x0000)

/* CH1_CFG register address & default value */

#define ADS131_CH1_CFG_ADDRESS                     ((uint8_t)0x0E)
#define ADS131_CH1_CFG_DEFAULT                     ((uint16_t)0x0000)

/* CH1_OCAL_MSB register address & default value */

#define ADS131_CH1_OCAL_MSB_ADDRESS                ((uint8_t)0x0F)
#define ADS131_CH1_OCAL_MSB_DEFAULT                ((uint16_t)0x0000)

/* CH1_OCAL_LSB register address & default value */

#define ADS131_CH1_OCAL_LSB_ADDRESS                ((uint8_t)0x10)
#define ADS131_CH1_OCAL_LSB_DEFAULT                ((uint16_t)0x0000)

/* CH1_GCAL_MSB register address & default value */

#define ADS131_CH1_GCAL_MSB_ADDRESS                ((uint8_t)0x11)
#define ADS131_CH1_GCAL_MSB_DEFAULT                ((uint16_t)0x8000)

/* CH1_GCAL_LSB register address & default value */

#define ADS131_CH1_GCAL_LSB_ADDRESS                ((uint8_t)0x12)
#define ADS131_CH1_GCAL_LSB_DEFAULT                ((uint16_t)0x0000)

/* REGMAP_CRC register address & default value */

#define ADS131_REGMAP_CRC_ADDRESS                  ((uint8_t)0x3E)
#define ADS131_REGMAP_CRC_DEFAULT                  ((uint16_t)0x0000)

/****************************************************************************
 * Types Declarations
 ****************************************************************************/

struct ads131_config_s
{
  struct spi_dev_s *spi;
  uint32_t frequency;
  CODE int  (*irq_attach)(FAR struct ads131_config_s * state, xcpt_t isr,
                          FAR void *arg);
  CODE void (*irq_enable)(FAR const struct ads131_config_s *state,
                          bool enable);
};

struct ads131_adc_channel_data_s
{
  uint16_t response;
  int32_t channel0;
  int32_t channel1;
  uint16_t crc;
};

/****************************************************************************
 * Name: ads131_initialize
 *
 * Description:
 *   Initialize the selected adc port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple adc interfaces)
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *ads131_initialize(struct ads131_config_s *config, unsigned int devno);

#endif /* CONFIG_SPI && CONFIG_MOTEC_DRV_ADS131 */