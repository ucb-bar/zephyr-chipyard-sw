BMI088 Sensor Test Sample
==========================

This sample application demonstrates how to use the BMI088 sensor driver
to read accelerometer, gyroscope, and temperature data.

The application:
- Initializes the BMI088 accelerometer and gyroscope devices
- Periodically reads and displays sensor data
- Shows accelerometer values in m/s²
- Shows gyroscope values in rad/s
- Shows temperature in °C

Requirements
------------

The device tree must define aliases for the BMI088 sensor:
- ``bmi088_accel`` for the accelerometer
- ``bmi088_gyro`` for the gyroscope

Building and Running
--------------------

Build the sample using west:

.. code-block:: console

   west build -b <board> samples/bmi088_test

Run the sample:

.. code-block:: console

   west flash

Sample Output
-------------

The application will output sensor readings every second:

.. code-block:: console

   BMI088 Sensor Test Application
   ==============================

   Accelerometer device ready: BMI088_ACCEL
   Gyroscope device ready: BMI088_GYRO

   Accelerometer - X: 0.123456 m/s^2, Y: -0.234567 m/s^2, Z: 9.810000 m/s^2
   Gyroscope     - X: 0.000000 rad/s, Y: 0.000000 rad/s, Z: 0.000000 rad/s
   Temperature    - 25.500000 °C
   ---

