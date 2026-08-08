PMW3901 Optical Flow Sensor Test
================================

This sample demonstrates how to use the PMW3901 optical flow sensor with Zephyr.

Hardware Setup
--------------

The PMW3901 is connected via SPI with the following pin mappings on ESP32C6:

- SCK:  GPIO6
- MOSI: GPIO7
- MISO: GPIO18
- NCS:  GPIO19

Optional pins:
- CamRST (active low, resets the sensor): GPIO2
- LED_N: GPIO3

Building and Running
-------------------

Console note (ESP32-C6)
-----------------------

The default console on ``esp32c6_devkitc_hpcore`` is **UART0 on GPIO16/17**.
If you’re watching the USB Serial/JTAG CDC-ACM port (often ``/dev/ttyACM*``)
instead, it can look like Zephyr “hangs before the banner” even when it booted.

This sample does **not** override ``zephyr,console``.

Build the sample:

.. code-block:: bash

   west build -b esp32c6_devkitc_hpcore samples/riskybird/pmw3901_test

Incremental bring-up overlays
-----------------------------

Baseline boot (no special overlay) uses only the board defaults:

Enable PMW3901 wiring using SPI bitbang (recommended first step while debugging):

.. code-block:: bash

   west build -b esp32c6_devkitc_hpcore samples/riskybird/pmw3901_test -p auto \
     -- -DDTC_OVERLAY_FILE=boards/esp32c6_devkitc_hpcore_pmw3901_bitbang.overlay \
        -DOVERLAY_CONFIG=pmw3901_bitbang.conf

Flash the sample:

.. code-block:: bash

   west flash

The sample will continuously read and display motion data from the PMW3901 sensor.
