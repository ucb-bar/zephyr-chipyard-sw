# riskybird v3 — Programming & Flashing Guide (ESP32-C6)

Practical guide for building and flashing the ESP32-C6 firmware on the riskybird
v3 main board, **including what a factory (never-flashed) board looks like** so
its behavior isn't mistaken for a hardware fault.

## Board / chip facts

- MCU: **ESP32-C6-MINI-1/U** (U14) = **ESP32-C6FH4**, single RISC-V HP core,
  **4 MB** on-package flash.
- Programming/console link: the chip's **built-in USB Serial/JTAG** peripheral,
  exposed on the board's **USB-C** port. On Linux it enumerates as
  `303a:1001 "USB JTAG/serial debug unit"` → `/dev/ttyACM*`.
- No external USB-UART bridge — `esptool` talks to the USB Serial/JTAG directly.
- Boot strap: **GPIO9** (BOOT) has the board's BOOT button (SW4) + a 0.1 µF cap
  and relies on the chip's internal pull-up (straps high = normal SPI boot).
  **EN/reset**: R40 (10 k pull-up) + caps + RESET button (SW1).

## Initial (factory / never-flashed) state — EXPECTED, not a fault

A blank board has erased flash (`0xFF`). On power-up the ROM/bootloader finds no
valid app and the RTC watchdog resets the chip about **every ~2 s**, so:

- `lsusb` shows the ESP device **appearing and disappearing** every ~2–3 s.
- Desktop (e.g. KDE) shows repeated "device connected" pop-ups.
- On the console you'd see `invalid header: 0xffffffff` repeating.

**This is normal for an unprogrammed board.** It stops the moment valid firmware
is flashed. (See the board bring-up log, BU-003.)

## Toolchain / environment (one-time per shell)

From the repo root:

```bash
source scripts/activate_conda.sh          # activates the tools/miniforge3 'zephyr' env (west, esptool, pyserial)
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr     # use the Zephyr SDK (NOT set_envvars.sh, which is the RISC-V chipyard cross-compile)
export ZEPHYR_SDK_INSTALL_DIR="$PWD/tools-manual/zephyr-sdk-1.0.0-beta1"
export ZEPHYR_BASE="$PWD/zephyr_ws/zephyr"
```

> Do **not** `source scripts/set_envvars.sh` for the ESP32 target — that selects
> the `riscv64-unknown-elf` cross-compiler used for the RISC-V chipyard SoC, not
> the esp32c6 build. The esp32c6 build uses the Zephyr SDK (`riscv64-zephyr-elf`).

## Build

The west workspace root is `zephyr_ws/`. Board target:
`esp32c6_devkitc/esp32c6/hpcore`.

```bash
cd zephyr_ws
west build -p always -b esp32c6_devkitc/esp32c6/hpcore ../samples/riskybird/sensor_bringup \
  -- -DEXTRA_DTC_OVERLAY_FILE=../samples/riskybird/usb_console.overlay
```

**Console over USB:** the hpcore default `zephyr,console` is **UART0 (physical
pins)**, which is *not* the USB-C port. Add `usb_console.overlay` (as above) to
route console + shell to the USB Serial/JTAG so output shows on `/dev/ttyACM*`.
Without it the board can look like it "hangs before the banner" when it actually
booted fine.

## Flash

```bash
west flash --esp-device /dev/ttyACM0
```

Or with esptool directly (chip is 4 MB):

```bash
esptool --chip esp32c6 --port /dev/ttyACM0 --baud 460800 \
  --before default-reset --after hard-reset \
  write-flash -u --flash-mode dio --flash-freq 80m --flash-size 4MB \
  0x0 build/zephyr/zephyr.bin
```

> **Blank-board gotcha:** while a never-flashed board is in the ~2 s reset loop,
> `/dev/ttyACM0` blinks in and out, so the flasher may fail to open the port on
> the first try. Just **retry** — once esptool grabs the port it forces download
> mode and holds the chip, and the flash completes. (A short shell retry loop
> works well.)

## Manual download mode (recovery)

If auto-reset-to-download ever fails: **hold BOOT (SW4), tap RESET (SW1), release
BOOT.** The chip parks in download mode (USB stays enumerated, looping stops),
then flash normally.

## Verify a good boot

After flashing:

- The enumerate/disconnect loop **stops**; the USB device stays stably present.
- Console banner over `/dev/ttyACM*`, e.g.:

  ```
  *** Booting Zephyr OS build 90047eb47fdb ***
  Hello World! esp32c6_devkitc/esp32c6/hpcore
  ```

Read the console (no extra monitor needed):

```bash
cat /dev/ttyACM0        # or: west espressif monitor
```

## Sensor bring-up quick reference (`sensor_bringup`)

- On-board **VL53L1X ToF** is powered/enabled through the **ADS7128 I/O expander
  GPIO6** (XSHUT), then reprogrammed `0x29 → 0x30` and initialized. Distance
  prints every 500 ms.
- The **PMW3901** optical-flow sensor and the four external **VL53L5CX** ToF
  breakouts may be unpopulated — the app detects them absent (invalid chip ID /
  not on the bus) and continues; the on-board ToF still works.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| USB device appears/disappears every ~2 s, KDE pop-ups | Blank flash (factory state). Flash firmware. |
| Boots but no console output on `/dev/ttyACM*` | Default console is UART0; build with `usb_console.overlay`. |
| `esptool: could not open /dev/ttyACM0` on a blank board | Port is cycling in the reset loop — retry, or enter manual download mode. |
| `flash_size 8MB is larger than 4MB` warning | Harmless for small images; chip is ESP32-C6FH4 (4 MB). Use `--flash-size 4MB`. |
