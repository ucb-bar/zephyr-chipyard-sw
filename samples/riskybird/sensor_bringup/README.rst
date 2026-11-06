ADS7128 I2C ADC Expander Connection Test
#########################################

This sample validates that the ADS7128 I2C ADC expander is properly
connected and accessible on the I2C bus.

Overview
********

The ADS7128 is an 8-channel, 12-bit analog-to-digital converter (ADC) with
an I²C interface. This sample performs basic connectivity tests by:

1. Verifying the device responds to I2C communication at address 0x17
2. Reading several configuration registers to validate register access
3. Confirming the device is ready for use

Hardware Requirements
*********************

- ESP32C6 DevKitC board (or compatible ESP32C6 board)
- ADS7128 I2C ADC expander connected to I2C bus
- ADS7128 configured at I2C address 0x17 (via ADDR pin configuration)

I2C Connections
**************

The ADS7128 should be connected to the I2C bus:
- SDA: GPIO15 (custom ESP32C6 PCB I2C0 SDA)
- SCL: GPIO14 (custom ESP32C6 PCB I2C0 SCL)
- VCC: 2.35V to 5.5V (AVDD and DVDD)
- GND: Ground
- ADDR: Configured for I2C address 0x17

Building and Running
*********************

Build the sample:

.. code-block:: bash

   west build -b esp32c6_devkitc_hpcore samples/riskybird/ads7128_test

Flash and monitor:

.. code-block:: bash

   west flash
   west espressif monitor

Expected Output
***************

The sample will output test results for each register read operation:

.. code-block:: console

   ADS7128 I2C Expander Connection Test
   ====================================

   I2C device ready: I2C_0

   Test 1: Basic I2C presence check...
          PASS: Device ACKed at address 0x17

   Test 2: Reading GENERAL_CFG register (0x01)...
          PASS: Read GENERAL_CFG = 0xXX

   ...

   ADS7128 connected successfully!
   All register read tests passed.
   Device is ready for use.

References
**********

- `ADS7128 Datasheet <https://www.ti.com/lit/ds/sbas868a/sbas868a.pdf>`_
  (SBAS868A - May 2019, Revised May 2020)
