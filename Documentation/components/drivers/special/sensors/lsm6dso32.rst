LSM6DSO32
=========

Contributed by Carleton University InSpace.

The LSM6DSO32 is an inertial measurement unit that contains an accelerometer and
gyroscope.

Application Programming Interface
=================================

The header file for the LSM6DSO32 interface can be included using:

.. code-block:: c

   #include <nuttx/sensors/lsm6dso32.h>

The LSM6DSO32 registration function allows the driver to be registered as a
POSIX character driver.

The standard POSIX `read()` function will return the 3 axes measurements of both
acceleration and angular velocity in plain-text, which is useful when
debugging/testing the driver using `cat` from the shell.

The `write()` operation is not implemented for this sensor.

Specific operations the sensor offers can be performed via the POSIX `ioctl`
operation. The supported commands are:

 * :c:macro:`SNIOC_WHO_AM_I`

.. c:macro:: SNIOC_WHO_AM_I

This command reads the `WHO_AM_I` register of the sensor. The register value is
returned in the argument to the command, which must be a `uint8_t` pointer.

.. code-block:: c

  uint8_t whoami = 0;
  err = ioctl(sensor, SNIOC_WHO_AM_I, &whoami);
