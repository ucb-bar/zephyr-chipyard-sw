# rose_flight_controller bench/debug tools

Host-side Python helpers for bringing up riskybird v3 on real hardware. They talk to the
board over the ESP32-C6 USB Serial/JTAG CDC port (`/dev/ttyACM*`). Need `pyserial`
(and `matplotlib` + `numpy` for the plot) — the repo's conda `zephyr` env has them:

    source scripts/activate_conda.sh

## serial_capture.py
Timestamped capture of the live serial stream to stdout (+ optional file). Reconnects
across USB re-enumeration, so start it then reset/flash the board.

    python tools/serial_capture.py [seconds] [port] [out.txt]

## flightlog_dump.py
Dump + per-flight analysis of the on-board flash flight log. Flash the DUMP build first:

    west build -b esp32c6_devkitc/esp32c6/hpcore samples/rose_flight_controller -d build_dump -- \
        -DROSE_USE_PID=1 -DEXTRA_CPPFLAGS="-DROSE_FLIGHTLOG_DUMP=1"
    west flash -d build_dump
    python tools/flightlog_dump.py [port] [out.csv]

It resets the board via DTR/RTS while attached (the dump prints ~0.5 s after boot), saves
the CSV, segments it into flights, and prints airborne means: est roll/pitch and the
per-corner duty + roll (RIGHT-LEFT) / pitch (FRONT-REAR) asymmetries — how the deterministic
hover drift was diagnosed. The log is append-mode and survives resets (erase with
`-DROSE_FLIGHTLOG_ERASE=1`).

## live_tof_plot.py
Live 8x8 heatmaps of the 4 side VL53L5CX wall sensors, for occlusion mapping + facing
checks. Flash a `-DROSE_BUMPER_GRID=1` build (streams `GRID <name>: ...` lines), then:

    python tools/live_tof_plot.py [port]

Near=red, far=green, grey=invalid/occluded. Each title shows the derived rows-0-1 valid
average (the wall distance the controller uses). Close the window (or Ctrl-C) to stop.
