=======
ADS7038
=======

The TI ADS7038 (and the automotive ADS7038-Q1) is an 8-channel, 12-bit,
1 MSPS SAR ADC on SPI.  Its analog supply, AVDD, is also the reference, so
code 4095 is AVDD.

The driver is an ADC lower half for the common ADC upper half
(``adc_register()``).  ``ANIOC_TRIGGER`` converts each enabled channel once,
in ascending order, and queues one ``struct adc_msg_s`` per channel with the
AIN number as ``am_channel``.  ``ANIOC_GET_NCHANNELS`` returns the number of
enabled channels.

The device is driven in on-the-fly mode, so converting ``n`` channels takes
``n + 1`` SPI frames.  Every result carries its channel ID, which the driver
checks; a mismatch fails the trigger with ``-EIO``.  At initialization the
driver resets the device, checks that a written register reads back (there is
no ID register), calibrates the offset and sets every input as analog.

The driver does not support the device's averaging, GPIOs, window
comparator, CRC or autonomous modes.

Driver Interface
----------------

.. code-block:: c

   #include <nuttx/analog/ads7038.h>

   FAR struct adc_dev_s *ads7038_initialize(FAR struct spi_dev_s *spi,
                                            uint32_t devid,
                                            uint8_t chanmask);

``devid`` is the device's ID for ``SPI_SELECT()``, and bit ``n`` of
``chanmask`` enables AIN\ ``n``.  It returns ``NULL`` if the device does not
respond.

.. code-block:: c

   FAR struct adc_dev_s *adc;

   adc = ads7038_initialize(spi, SPIDEV_ADC(0), 0x0f);   /* AIN0-AIN3 */
   if (adc != NULL)
     {
       ret = adc_register("/dev/adc0", adc);
     }

Configuration
-------------

* ``CONFIG_ADC_ADS7038``: the driver (needs ``CONFIG_SPI_EXCHANGE``).
* ``CONFIG_ADS7038_FREQUENCY``: SPI clock, up to 60 MHz.  SPI mode 0.
