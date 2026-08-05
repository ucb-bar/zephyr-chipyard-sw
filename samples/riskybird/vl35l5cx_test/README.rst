.. zephyr:code-sample:: vl35l5cx_test
   :name: VL53L5CX minimal test

   Minimal bring-up sample for VL53L5CX using ADS7128 as an I2C GPIO expander.
   ADS7128 GPIO1 is used to enable the first VL53L5CX sensor, then the sample
   initializes the sensor and prints distances periodically.
