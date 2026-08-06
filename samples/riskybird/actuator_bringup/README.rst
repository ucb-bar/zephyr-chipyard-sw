RiskyBird Actuator Bring-up
###########################

Sequentially spins each of the four brushed motors for ~1 second at a low PWM
duty, to verify the motor drive channels. Motors are low-side N-FET driven
(Q1-Q4 = SI2302); the ESP32-C6 PWMs the gate (active-high) via LEDC.

Motor map
*********

===== ========= ============= =========
Motor Gate net  ESP32-C6 GPIO LEDC ch
===== ========= ============= =========
M1    /MOTOR1   GPIO21        CH0
M2    /MOTOR2   GPIO20        CH1
M3    /MOTOR3   GPIO23        CH2
M4    /MOTOR4   GPIO22        CH3
===== ========= ============= =========

Each gate is also wired to the FPGA (JB1) through a parallel 47R, with a 10k
pulldown holding the FET off until driven.

Safety / power
**************

- **Remove propellers.** Duty is low (``SPIN_DUTY_PCT``, default 10 %) — enough to
  spin, not to fly — but run it bare.
- Motors run off **+BATT**, so the board must be **battery-powered** (or a bench
  supply on the +BATT input). On USB alone with no battery, +BATT is only weakly
  cap-charged and the motors will barely move.

Tuning
******

Edit ``src/main.c``: ``SPIN_DUTY_PCT`` (start duty), ``PWM_PERIOD_NS`` (frequency,
default 20 kHz), ``SPIN_MS`` (on-time per motor), ``GAP_MS`` (pause between).

Building and Running
********************

.. code-block:: bash

   cd zephyr_ws
   west build -b esp32c6_devkitc/esp32c6/hpcore ../samples/riskybird/actuator_bringup \
     -- -DEXTRA_DTC_OVERLAY_FILE=../samples/riskybird/usb_console.overlay
   west flash --esp-device /dev/ttyACM0

Runs **one pass** across the four motors, then stops (all off) until the board is
reset. Expected output::

   riskybird actuator bring-up (sequential motor spin)
   *** REMOVE PROPELLERS ***  duty=10%  freq=20000 Hz  1000 ms each

     M1 (GPIO21)  ON  (10%)
     M1 (GPIO21)  off
     M2 (GPIO20)  ON  (10%)
     M2 (GPIO20)  off
     M3 (GPIO23)  ON  (10%)
     M3 (GPIO23)  off
     M4 (GPIO22)  ON  (10%)
     M4 (GPIO22)  off

   Pass complete — all motors off. Reset the board to run again.
