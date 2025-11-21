Flight Controller Sample
========================

This sample implements a flight controller skeleton with attitude estimation
using a complementary filter. The design is modular, with a clean main file
that initializes separate tasks for different functionality.

Features
--------

- **Modular Architecture**: Clean separation of concerns with independent tasks
- **Attitude Estimation**: Complementary filter combining accelerometer and gyroscope
- **Configurable Frequency**: Attitude estimator runs at configurable frequency (default 200Hz)
- **Thread-Safe**: Mutex-protected attitude data access

Architecture
------------

The flight controller consists of:

- **main.c**: Clean initialization and task coordination
- **attitude_estimator.c**: Attitude estimation using complementary filter
  - Runs as independent thread at configurable frequency
  - Combines BMI088 accelerometer and gyroscope data
  - Provides thread-safe attitude access

Configuration
-------------

The attitude estimator frequency can be configured via Kconfig:

.. code-block:: console

   CONFIG_ATTITUDE_ESTIMATOR_FREQUENCY=200

Default frequency is 200Hz. The complementary filter alpha value (gyro weight)
can be adjusted in ``attitude_estimator.c``:

.. code-block:: c

   #define COMPLEMENTARY_FILTER_ALPHA 0.98f  /* 0-1, higher = trust gyro more */

Requirements
------------

- BMI088 sensor configured in device tree (aliases: ``bmi088-accel`` and ``bmi088-gyro``)
- I2C bus enabled

Building and Running
--------------------

Build the sample:

.. code-block:: console

   west build -b <board> samples/flight_controller

Run the sample:

.. code-block:: console

   west flash

Sample Output
-------------

The application will output attitude estimates:

.. code-block:: console

   [00:00:00.000,000] <inf> main: Flight Controller Starting...
   [00:00:00.000,000] <inf> attitude_estimator: Attitude estimator initialized (frequency: 200 Hz)
   [00:00:00.000,000] <inf> attitude_estimator: Attitude estimator started
   [00:00:00.000,000] <inf> main: Flight Controller Initialized
   [00:00:00.100,000] <inf> main: Attitude - Roll: 0.001, Pitch: -0.002, Yaw: 0.000 (rad)
   [00:00:00.200,000] <inf> main: Attitude - Roll: 0.002, Pitch: -0.001, Yaw: 0.001 (rad)

Extending the Flight Controller
--------------------------------

To add new functionality:

1. Create a new module (e.g., ``motor_controller.c``)
2. Add initialization in ``main.c``
3. Create a task/thread if needed
4. Follow the same pattern as ``attitude_estimator.c``

Example:

.. code-block:: c

   // In main.c
   ret = motor_controller_init();
   if (ret < 0) {
       LOG_ERR("Failed to initialize motor controller: %d", ret);
       return ret;
   }
   
   ret = motor_controller_start();
   if (ret < 0) {
       LOG_ERR("Failed to start motor controller: %d", ret);
       return ret;
   }

