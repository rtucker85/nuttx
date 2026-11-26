=======
LSM6DSV
=======

The LSM6DSV is a 6-axis IMU with a 3-axis gyroscope and a 3-axis
accelerometer by STMicroelectronics.  The driver also accepts the
LSM6DSV320X (``WHO_AM_I`` 0x73), which has the same accelerometer and
gyroscope plus a high-g accelerometer (up to 320 g), published as a second
accelerometer topic, and whose SFLP sensor fusion block's game rotation
vector is published as a rotation topic; on it the gyroscope's largest
scale is 2000 dps.  This
driver supports it on SPI only
and uses the :doc:`uorb </os/drivers/special/sensors/sensors_uorb>`
interface.

All sensors are batched into the device FIFO at their sample rates.  A
kernel thread drains the FIFO every ``CONFIG_SENSORS_LSM6DSV_POLL_INTERVAL``
microseconds and publishes every sample, so sample rates above the system
tick rate do not lose data.  Each sample is timestamped with the time the
FIFO was read, less one sample period for each newer sample of the same
sensor.  Data-ready interrupts are not used.

.. warning::
   The driver does not implement the device's self-test, calibration
   offsets, embedded functions other than the LSM6DSV320X's SFLP game
   rotation vector (pedometer, tilt, SFLP gravity and gyroscope bias,
   SFLP on the LSM6DSV), the
   low-power modes or the I2C/I3C interfaces.

Application Programming Interface
=================================

.. code-block:: c

   #include <nuttx/sensors/lsm6dsv.h>

   int lsm6dsv_register_uorb(int devno, int hgdevno, int rotdevno,
                             FAR struct spi_dev_s *spi, uint32_t devid);

Registering the driver checks ``WHO_AM_I`` (0x70, or 0x73 for the
LSM6DSV320X), resets the device and
registers two uORB topics under ``/dev/uorb/``: ``sensor_accel<n>`` (m/s²)
and ``sensor_gyro<n>`` (rad/s), where ``n`` is ``devno``.  On the
LSM6DSV320X, if ``hgdevno`` is not negative, the high-g accelerometer is
also registered as ``sensor_accel<hgdevno>`` (m/s²), and if ``rotdevno`` is
not negative the SFLP game rotation vector as ``sensor_rotation<rotdevno>``.
``devid`` is the device's ID for ``SPI_SELECT()``.

.. code-block:: c

   ret = lsm6dsv_register_uorb(0, 1, 1, spi, SPIDEV_IMU(0));
   if (ret < 0)
     {
       syslog(LOG_ERR, "Couldn't register LSM6DSV: %d\n", ret);
     }

Each topic's sample rate follows ``set_interval()``: the device rate at or
above the one requested, 7.5 Hz to 7.68 kHz (480 Hz to 7.68 kHz for the
high-g accelerometer).  ``SNIOC_SET_SCALE_XL`` sets the full scale, in g
for the accelerometer (2, 4, 8, 16) and the high-g accelerometer (32, 64,
128, 256, 320), and in degrees per second for the gyroscope (125 to 4000).
The accelerometer and gyroscope samples carry the die temperature.

The game rotation vector is a unit quaternion (x, y, z, w) from the
accelerometer and gyroscope only, in the IMU's axes: yaw is relative and
drifts, and ``status`` is -1.  Its rate is 15 Hz to 480 Hz.  While it runs,
the accelerometer and gyroscope run at least at its rate whether or not
their topics are active; the filter is reset each time the topic is
activated.

Configuration
=============

* ``CONFIG_SENSORS_LSM6DSV``: the driver.
* ``CONFIG_SENSORS_LSM6DSV_SPI_FREQUENCY``: SPI clock, up to 10 MHz.
* ``CONFIG_SENSORS_LSM6DSV_INTERVAL``: initial sample period of the
  sensors.
* ``CONFIG_SENSORS_LSM6DSV_POLL_INTERVAL``: FIFO drain period.  The FIFO
  holds 512 samples of all the sensors together.
* ``CONFIG_SENSORS_LSM6DSV_ACCEL_FS``, ``CONFIG_SENSORS_LSM6DSV_GYRO_FS``
  and ``CONFIG_SENSORS_LSM6DSV_HG_FS``: initial full scales.
