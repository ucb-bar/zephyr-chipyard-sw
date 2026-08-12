# Playing music with RiskyBird's motors

Investigation (side-quest) into making RiskyBird v3 play music with its props/motors — the
floppy-drive / Crazyflie-jukebox trick. **Investigation only; no firmware has been written yet.**

## TL;DR
- **Technique: drive the LEDC PWM *at the note frequency*.** Switch the motor FET at, e.g., 440 Hz
  and the coil/rotor/frame vibrates at 440 Hz and radiates it — like a floppy stepper "singing" at
  its step rate. The rotor's inertia low-passes the torque ripple, so **the prop barely spins while
  the coil buzzes at pitch** → clean, controllable notes at *sub-liftoff* duty.
- The Crazyflie uses the **same 7×16 mm coreless motors** and does exactly this, so it's directly
  transferable. Practical range **C4–B7 (~262 Hz – 4 kHz)**, octave-shift anything outside.
- On this firmware it's a ~40-line `motors_play_song()` next to `motors_boot_chirp()`, gated behind a
  build flag, using **`pwm_set_dt()`** (period+pulse) instead of `pwm_set_pulse_dt()` (duty only).
- **As shipped it's monophonic** (all 4 LEDC channels share `timer <0>`); 4-voice chords need a
  one-line-per-channel devicetree edit (one timer per channel — the C6 has exactly 4).

## 1. Mechanism / acoustics
Two candidate techniques:
- **(a) PWM frequency = note frequency** *(chosen)*. The winding current pulses at the note
  frequency; the pulsating electromagnetic force (cogging/reluctance + winding movement + magneto-
  striction) vibrates the motor body, the PCB frame, and the prop at that pitch. **Pitch = electrical
  drive frequency, not prop RPM** — the rotor inertia low-passes the torque, so the prop barely
  responds while the coil sings, and notes above the mechanical bandwidth still sound as thinner coil
  whine. Audible well below liftoff duty.
- **(b) Amplitude-modulate thrust at audio rate** *(rejected)*. Keep the 20 kHz carrier, wobble duty
  at the note frequency so prop RPM/air pulses at pitch. A 7×16 coreless motor+prop has a mechanical
  time constant of tens of ms → usable modulation bandwidth only ~tens of Hz. Fine for a slow rhythm,
  useless for melody, and wastes thrust (louder = more spin).

## 2. Feasibility on this board (verified against the driver)
Driver `zephyr_ws/zephyr/drivers/pwm/pwm_led_esp32.c`:
- **Per-note frequency retune works.** `pwm_set()` takes period/pulse in ns; `freq ≈ 1e9/period_ns`.
  The LEDC driver reconfigures the timer only when the frequency actually changes (early-returns if
  unchanged) — a note change is a brief glitch that reads as a clean note onset; same-pitch duty
  changes are glitch-free.
- **Range/resolution is not the bottleneck.** C6 LEDC = 20-bit timer on 80 MHz; the prescaler stays
  legal and duty resolution stays high across the whole audio band. Electrically ~40 Hz to >20 kHz.
  The limit is mechanical/acoustic, not the LEDC.
- **Coreless reality.** Low notes (<~130 Hz) tend to cog/rattle and spin the prop → octave up. High
  notes (>~4 kHz) are audible but thin. Crazyflie treats **C4–B7** as the reliable window on these
  same motors — adopt that.

## 3. Implementation plan
Lives beside `motors_boot_chirp()` in `src/main.cpp` (real-target branch), called from `main()` next
to the existing `motors_boot_chirp();` — runs once at boot, **before** the control loop, while
disarmed (same safe window). Gate behind `-DROSE_PLAY_SONG=1`.

**The one API change:** `pwm_set_pulse_dt()` reuses the fixed 20 kHz DT period (duty only). To change
pitch, use **`pwm_set_dt(&motors[i], period_ns, pulse_ns)`** (Zephyr explicitly supports runtime
period changes this way). `period_ns = 1e9/freq`, `pulse_ns = period_ns * duty`.

```c
/* Bench-only motor music (-DROSE_PLAY_SONG=1). Runs once at boot, disarmed, before the
 * control loop -- same safe window as motors_boot_chirp(). Drives LEDC at the note frequency
 * so the coil/frame sings at pitch. Keep duty low (audible well below liftoff). Props off. */
#ifndef MUSIC_MAX_DUTY
#define MUSIC_MAX_DUTY 0.10f
#endif
struct note { uint16_t freq_hz; uint16_t ms; };   /* freq 0 = rest */

static void motor_tone(int i, uint16_t freq_hz)
{
	if (freq_hz == 0) { pwm_set_pulse_dt(&motors[i], 0); return; }
	uint32_t period_ns = 1000000000u / freq_hz;
	uint32_t pulse_ns  = (uint32_t)(period_ns * MUSIC_MAX_DUTY);
	pwm_set_dt(&motors[i], period_ns, pulse_ns);   /* NOT _pulse_dt: lets us change frequency */
}

static void motors_play_song(void)
{
	static const struct note song[] = { /* ...note table... */ };
	for (size_t n = 0; n < ARRAY_SIZE(song); n++) {
		for (int i = 0; i < NACTIONS; i++) motor_tone(i, song[n].freq_hz);
		k_msleep(song[n].ms);
		for (int i = 0; i < NACTIONS; i++) pwm_set_pulse_dt(&motors[i], 0);  /* note gap */
		k_msleep(20);
	}
	for (int i = 0; i < NACTIONS; i++) pwm_set_pulse_dt(&motors[i], 0);
}
```
Self-timed with `k_msleep` like `motors_boot_chirp` (doesn't touch the control loop). If you later want
music *during* flight, advance a note index off `k_uptime_get()` inside the loop and set duty in
`send_control()` instead. 1 kHz loop granularity (1 ms) is far finer than any note needs.

## 4. Polyphony
- **As shipped = monophonic (unison).** Overlay `esp32c6_devkitc_hpcore.overlay` puts all 4 channels on
  `timer = <0>`; the driver refuses two channels sharing a timer at different frequencies
  (`-EINVAL`, "Timer can't be shared and different frequency requested"). You still get per-motor
  **duty = per-motor volume** (antiphonal/"stereo" effects) — just one pitch at a time.
- **4-voice chords:** give each motor its own timer (C6 LEDC = 6 channels but 4 timers; frequency is
  per-timer, duty per-channel):
  ```
  channel0@0 { reg = <0x0>; timer = <0>; };
  channel1@1 { reg = <0x1>; timer = <1>; };
  channel2@2 { reg = <0x2>; timer = <2>; };
  channel3@3 { reg = <0x3>; timer = <3>; };
  ```
  → exactly 4 independent voices (the Crazyflie model). Then `motor_tone(i, freq)` per motor gives
  true chords; allocate voices round-robin or melody/bass-priority.
- **Volume** = duty (roughly monotonic until thrust matters → cap it). **Range** C4–B7, octave-shift
  outliers. **Tempo** granularity effectively unlimited.

## 5. Safety
- **Props spin and make thrust — restrained bench demo only.** Run props-off while tuning notes/volume;
  if props on, restrain the frame. Technique (a) is audible well below liftoff, so keep duty low.
- Give the demo its **own** low `MUSIC_MAX_DUTY` (10–15% max) and always end by writing 0 to all
  channels. It runs at boot before the control loop (like the chirp), bypassing `g_armed` by driving
  PWM directly — so don't wire music into an armed/flying build for a first pass.
- Build-flag it (`-DROSE_PLAY_SONG=1`) so flight/production builds compile it to a no-op, like the
  existing `ROSE_START_PULSE_MS` / autoflight guards.

## 6. Prior art
- Bitcraze — *Fun Friday: Play music on your Crazyflie* (crazyflie-jukebox; MIDI → per-motor notes;
  4 voices; C4–B7 octave-shift; same 7×16 motors): https://www.bitcraze.io/2026/03/fun-friday-project-play-music-on-your-crazyflie/
- Crazyflie Jukebox demo: https://www.youtube.com/watch?v=H25dcKOpLaY
- Crazyflie sound module: https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/sound_cf2.c
- Floppotron / musical floppy drives (the (a) technique): https://en.wikipedia.org/wiki/Floppotron
- ESP32 LEDC freq-change caveat: https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/api-reference/peripherals/ledc.html

## Key files
- `samples/rose_flight_controller/src/main.cpp` — `send_control()`, `motors_boot_chirp()` (template),
  `motors[]` / `MOTOR_MAX_DUTY`, the boot call site.
- `samples/rose_flight_controller/boards/esp32c6_devkitc_hpcore.overlay` — `ledc0` channels (all on
  `timer <0>`), `pwm_motors` 50000 ns (20 kHz) period.
- `zephyr_ws/zephyr/drivers/pwm/pwm_led_esp32.c` — shared-timer rejection, freq-change reconfigure.
- `zephyr_ws/zephyr/include/zephyr/drivers/pwm.h` — `pwm_set_dt()` runtime-period API.
