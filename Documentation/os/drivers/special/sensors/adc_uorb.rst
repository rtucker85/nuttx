===================
ADC to uORB bridge
===================

``drivers/sensors/adc_uorb.c`` publishes the channels of an ADC, scaled to
real units, as :doc:`uorb </os/drivers/special/sensors/sensors_uorb>`
topics.  The ADC driver stays generic.  The board says what each input is
wired to, so the scaling lives with the board.

The bridge takes the ADC lower half, the ``struct adc_dev_s`` an ADC
driver's initialize function returns, in place of ``adc_register()``.  It
needs a lower half that converts on ``ANIOC_TRIGGER``.

.. code-block:: c

   #include <nuttx/sensors/adc_uorb.h>

   static const struct adc_uorb_chan_s g_chans[] =
   {
     /* channel, type, devno, gain, offset */

     { 0, SENSOR_TYPE_CURRENT, 0, 6.23f, 0.0f },   /* 1 k IPROPI, 6230:1 */
     { 2, SENSOR_TYPE_VOLTAGE, 0, 13.27f, 0.0f },  /* 47 k / 3.83 k */
   };

   static const struct adc_uorb_config_s g_config =
   {
     .adc      = adc,           /* from e.g. ads7038_initialize() */
     .vref     = 3.3f,
     .bits     = 12,
     .nchans   = 2,
     .chans    = g_chans,
     .rawdevno = 0,             /* /dev/uorb/adc_raw0, or -1 */
   };

   ret = adc_uorb_register(&g_config);

Each channel publishes ``gain * f(volts) + offset`` on
``sensor_<type><devno>``, where ``volts`` is the voltage at the ADC input
and ``f`` is the channel's ``convert`` function, or the identity when it is
NULL.  Use ``convert`` for a non-linear input such as a thermistor.  Values
are in the topic's SI units: V, A, degC.  The single-value topics are
supported: voltage, current, temperature, ambient temperature, humidity,
power and resistance.

All channels are converted together in one scan, so values from one scan
share a timestamp.  The scan runs on the low-priority work queue at the
shortest interval any active topic asks for.  The bridge sets the ADC up
(``ao_setup``) when the first topic is activated and shuts it down when the
last one is deactivated, as the ADC character driver does on open and
close.  The ADC may return results from an interrupt; publishing happens in
a second work item.

* ``SNIOC_SET_CALIBVALUE`` sets a channel's gain and offset, and
  ``SNIOC_CALIBRATE`` reads them back.  The argument is a
  ``struct adc_uorb_calib_s *``.  Both are lost at reset.
* With ``rawdevno >= 0``, ``/dev/uorb/adc_raw<n>`` publishes
  ``struct adc_uorb_raw_s``: the raw code of every configured channel from
  each scan.

* ``CONFIG_SENSORS_ADC_UORB``: the bridge (needs ``CONFIG_ADC`` and
  ``CONFIG_SCHED_LPWORK``).
* ``CONFIG_SENSORS_ADC_UORB_MIN_INTERVAL``: shortest scan interval, in
  microseconds (default 1000).
