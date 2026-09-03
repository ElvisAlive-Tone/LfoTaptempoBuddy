# LFO Taptempo Buddy

Small but powerful ATtiny402 based tap-tempo LFO firmware. Output is a waveform (Sin, Triangle, Pulse, Ramp up,
Ramp down, Random) at the Speed set by the `Speed Pot` or by tapping. Multiple Random algorithms.
Leslie like speed change on long tap (faster, or slower when already fast). Intended for tremolo and similar analog modulation effects.

This is an adaptation of [Hydra Delay Taptempo Buddy](https://github.com/ElvisAlive-Tone/HydraDelayTaptempoBuddy).

**Tip:** You can use my [Simple Serial UPDI programmer](https://github.com/ElvisAlive-Tone/updipcb) to program the microcontroller for this project.

## Features

- LFO Speed can be controlled by `Speed Pot` or tapped by `Tap Button`. Period range is 50 ms (~20 Hz) to 2000 ms (~0.5 Hz).
- `LED` always blinks at the LFO Speed with 50% duty cycle, locked to LFO phase (both pot and tap control).
- `Tap Button` does four jobs:
  - Two or more taps — Speed from tapping.
  - Hold from rest (over 500 ms) — Leslie.
  - Short tap, then hold (over 500 ms) — Random algorithm.
  - Hold and flip `Shape` — alt bank (Ramp up / Ramp down / Random).
- Tap `Tap Button` at least two times to switch from `Speed Pot` control to `Tap Button` control.
  - First tap aligns LFO and `LED` to the downbeat without changing Speed.
  - Second tap must follow within 2.5 s after the first one.
  - Subsequent tap times are averaged until tapping finishes.
  - Tapping finishes if the next tap is not performed for 3 periods of the current Speed, but never later than 2.5 s.
  - Very slow subsequent taps (gap between 2 s and 2.5 s) still count; only that interval is clamped to 2 s. Faster taps are left as measured.
- Move `Speed Pot` to take control from tap tempo or latched Leslie: the pot is ignored until its speed matches the current sounding Speed (catch-up), then control passes to the pot with no jump. In normal pot mode the LFO tracks the pot continuously.
- `Shape` on-off-on switch selects the LFO waveform:
  - Flip **without** holding `Tap Button` — normal bank: Sin, Triangle, Pulse.
  - Flip **while holding** `Tap Button` — alt bank: Ramp up, Ramp down, Random.
  - Hold-flip only changes the shape bank. Speed, Leslie, and Random algorithm stay as they are. The press still lines up LFO phase.
  - Move the switch **without** holding Tap to return to Sin / Triangle / Pulse.
  - With Leslie latched, wait for Speed to settle after release, then press `Tap Button` again before hold-flip for the alt bank.
- Random shape algorithm (Hybrid / S&H / Wander) is cycled with a **short tap then one long press** of the `Tap Button` (hold the second press over 500 ms). Works in any shape; the sound changes when Random is selected. The `LED` still blinks 1 / 2 / 3 on any shape so you can see it was stored.
  - `LED` blinks **1 / 2 / 3** times (Hybrid / S&H / Wander) to indicate the selected algorithm.
  - The same blink is shown on power-up when Random is selected. See [Random shape algorithm](#random-shape-algorithm).
- **Leslie** — long `Tap Button` press from rest (over 500 ms, no short tap before it):
  - **While held:** Speed glides toward **2×** (half period, min 50 ms) when period is **300 ms or longer**, or toward **½** (double period, max 2000 ms) when already faster. `Speed Pot` sets glide time (~0.3–8 s for a full leg), not LFO period. Tap is occupied — no tap tempo, Random cycle, or hold-flip. Foot-down at the start of the hold does **not** phase-sync once the ramp begins at 500 ms.
  - **Release at the Leslie target** (2× or ½) → **latch** there; pre-Leslie Speed is remembered but not written to EEPROM.
  - **Release before target** → glide back to the saved origin without latching; glide continues if you lift your foot; press again to ramp toward Leslie.
  - **While latched** (foot up):
    - Short tap — downbeat align only; stay latched.
    - Tap session (two or more taps) — new Speed; Leslie cancelled.
    - Short tap, then long hold — Random cycle; stay latched.
    - `Speed Pot` catch-up — Leslie cancelled (see above).
    - Long hold from rest — unlatch; glide to saved origin (early release finishes foot up; press again during glide to head back toward Leslie).
- Current `Speed Pot`/`Tap Button` control state, tapped-in Speed, Random algorithm, and shape bank are preserved over power-off. Latched Leslie Speed is **not** stored — boot restores the last non-Leslie Speed.
- Trimmer or fixed resistor to set LED brightness.
- UPDI pins to re-program the soldered microcontroller.

## Project Content

- `firmware.hex` - firmware binary
- `firmware/` - VSCode/[PlatformIO](https://docs.platformio.org/en/latest/platforms/atmelmegaavr.html) project with firmware
- `LFOBuddy.dch` - schematics
- `LFOBuddy-rev1_gerber.zip` - Gerber file for PCB fabrication
- `LFOBuddy.dip` - PCB design file

Schematics and PCB design file can be opened/edited by [DipTrace](https://diptrace.com/).

## Wiring the module

- Plan module and controls (`Tap Button`, `Shape` on-off-on, `Speed Pot`, `LED`) placement. Use long enough wires.
- Power and output:
  - `GND` - ground
  - `OUT` - LFO voltage
  - `VCC` - 5V power, but can run on 3.3V also
  - You can use a connector to disconnect module easily for programming, spacing on PCB is 2.54mm for it.
- `Speed Pot` - connect pot's 1, 2 and 3 lugs to the module's `P1`, `P2` and `P3`.
  Use `B` type pot, ideally `B10k`. Higher voltage is higher LFO Speed (shorter LFO period). The firmware uses a **logarithmic** map so more of the pot's travel sits in the ~200–800 ms range.
- `Tap Button` - connect momentary button to the module's `TAP` pads.
- `LED` - connect LED to the module's `L+` and `L-`. Use `TL` trimmer to set LED's brightness. Used `2k` value should
  be OK for the most LED types, if too small for your LED, use higher trimmer value, or connect additional resistor
  in series. Alternatively use fixed value resistor `RL` if you figured out exact value and wanna to save some space.
- `Shape` switch - SPDT **on-off-on**. Common to `SH`. Throws to `SHG` and `SHV`. Left of `/` is the normal bank (no hold), right is the alt bank (hold Tap).
  ```
    SHV ---- throw     Pulse    /  Random
                |
    SH  ---- common    Triangle /  Ramp down
                |
    SHG ---- throw     Sin      /  Ramp up
  ```

### LFO voltage `OUT` wiring

Analog `OUT` is the LFO control voltage PWM:

- PCB contains simple filter to smooth it out. Cut frequency should be 100–200Hz, so eg. R=1kohm and C=1uF is good into high-Z load. So buffer it, or adjust filter for small-Z loads.
- Maximal amplitude is VCC (5V or 3.3V), so scaling may be necessary fo rnext circuitry
- You have to implement LFO `Depth` pot if you want it.

Exact circuitry consuming this module output depends on how do you want to drive the rest of the pedal circuit - VACTROL LED/s, lamp, JFETs etc.

## Building module

Module schematics:

<img src="img/schematics.png" width="600px" alt="Module schematics">

**Note:** you can omit U2 and bridge its pins 1 to 3 if you have 5V or 3.3V from the main pedal board.

PCB BOM:

| Markings           | Value             | PCB packaging type                                    |
| ------------------ | ----------------- | ----------------------------------------------------- |
| R1, R2, R4         | 1k                | 1206                                                  |
| R5, R6             | 10k               | 1206                                                  |
| C1, C3, C4         | 100n              | 1206                                                  |
| C2                 | 1u                | 5mm                                                   |
| TL                 | 2k or 5k          | 3362 trimmer                                          |
| RL (instead of TL) | matching LED      | 1206                                                  |
| U1                 | ATtiny 402 or 412 | SOIC-8                                                |
| U2                 | 7805              | TO-92                                                 |
| UPDI               |                   | 3 pins header connector (male or female, it's on you) |

External components:

| Markings     | Value                  |
| ------------ | ---------------------- |
| `Speed Pot`  | B10k                   |
| LED          | any color and size LED |
| `Tap Button` | any momentary switch   |
| `Shape`      | SPDT on-off-on         |

PCB:

<img src="img/pcb.png" width="300px" alt="PCB">

## Functionality Tweaking

### LFO period range

Source code contains constants for the LFO period range and tap timeout. Change them and rebuild if you want a different Speed span.

```c
const uint16_t c_lfo_max = 2000;   // slowest LFO period [ms] (~0.5 Hz) - Speed pot on minimum
const uint16_t c_lfo_min = 50;     // fastest LFO period [ms] (~20 Hz) - Speed pot on maximum
const uint16_t c_lfo_range = 1950; // c_lfo_max - c_lfo_min [ms]
const uint16_t c_tap_end_max = 2500; // max wait for the next tap [ms]; must be > c_lfo_max
```

PWM carrier is 20 kHz, range `0..c_pwm_max` (999). The analog LFO appears after the RC filter on `OUT`.

### Speed pot mapping

`pot_to_period()` uses a 64-step log LUT (`period_log_lut` in `firmware/src/main.c`) so more physical pot travel covers ~200–800 ms. Use a linear **B** pot; catch-up and tap/pot handoff use the same map.

```c
const uint16_t c_pot_catchup_ms = 40; // minimum catch-up window [ms]; also 5% of sounding period
```

### Leslie Speed

Holding `Tap button` (from rest, over 500 ms) ramps Speed by a multiplier. **Latch** only if you release **at** the 2×/½ target; release earlier and Speed glides home without latching.

The multiplier is `c_leslie_speed` in `firmware/src/main.c`. Default is 2. Use 3 for 3x, and so on — integer only.

- If the current period is **at or above** `c_leslie_slowdown_ms`, hold speeds up: period becomes `period / c_leslie_speed`, clamped at `c_lfo_min` (50 ms).
- If the current period is **below** `c_leslie_slowdown_ms` (already high Speed), hold slows down: period becomes `period * c_leslie_speed`, clamped at `c_lfo_max` (2000 ms).

`c_leslie_slowdown_ms` defaults to 300 (~3.3 Hz).

`Speed Pot` sets the **total glide time** for a full 2×/½ ramp leg (constant rate over the distance), not the LFO period:

```c
const uint16_t c_leslie_ramp_min_ms = 300;  // Speed pot max — ~0.3 s full glide
const uint16_t c_leslie_ramp_max_ms = 8000; // Speed pot min — ~8 s full glide
```

The map is quadratic on the slow end of the pot so fine control sits in the longer ramps. Examples at 500 ms → 250 ms (speed-up): ~8 s with pot at minimum, ~2.2 s at mid, ~0.3 s at maximum.

**Latched Leslie:** sitting at 2×/½ after release at the target. Short tap only phase-aligns (foot down). Committed tap tempo (two or more taps), pot catch-up, or a long hold from rest (unlatch) exits Leslie. Leslie and unlatch long holds do **not** phase-shift — any foot-down sync from the start of the hold is undone when the glide begins at 500 ms. Unlatch ramps home while held; releasing the foot early still finishes the glide to the saved origin. On any return ramp (initial or unlatch), the glide continues foot up and a new press steers back toward the Leslie target. Power-off does not save the latched rate.

### Random shape algorithm

Random still follows the tapped or pot Speed, with one new random level per LFO cycle. What it _does_ with that level is the algorithm:

- **Hybrid** — S&H when period is 400 ms or shorter, Wander when LFO is slower (default).
- **S&H** — jumps to a random level and sits there until the next cycle (classic stepped random).
- **Wander** — glides smoothly from the last level to the next over the cycle (no stairs; nicer when the LFO is slow).

**How to switch:** tap once (short), then press and hold (longer than 500 ms). That is not a new Speed, and it is not Leslie.
You can do this tap pattern with any shape selected. The sound only changes when Random is selected; the LED still blinks the new algorithm on any shape.

The LED then blinks the new algorithm. The choice is stored in EEPROM, so it survives power-off:

- 1 blink — Hybrid
- 2 blinks — S&H
- 3 blinks — Wander

The same blinks happen at power-up, but only if Random shape is already selected (so other shapes do not look like an algorithm change).

To always blink on power-up, set `ANNOUNCE_RANDOM_ON_BOOT_ALWAYS` to `1` in `firmware/src/main.c`.

A blank chip starts in Hybrid. To change that default, edit `LFO_RANDOM_MODE` in the same file.

The 400 ms Hybrid algorithm split is defined in `c_random_hybrid_ms` (400 ms and faster -> S&H in Hybrid).

After the 1/2/3 announcement blinks there is a trailing dark so they do not blend into the LFO tempo LED:

```c
#define C_RANDOM_ANNOUNCE_DARK_MS 700     // dark after algorithm blinks so they don't blend into tempo LED
```

## Compiling firmware

I'm using VS Code with Platform IO extension. You have to have `Atmel megaAVR` Platform installed in the Platform IO.

`platformio.ini` is committed in the `firmware/` folder with all the basic settings,
including [pymcuprog](https://github.com/microchip-pic-avr-tools/pymcuprog) related settings for
my [Simple Serial UPDI programmer](https://github.com/ElvisAlive-Tone/updipcb).

Open the `firmware/` folder in VS Code and it should work.

## Programming the microcontroller

Easiest way is to set the microcontroller up before soldering it to the PCB.

If you want to change firmware later, you can use UPDI pins on the module, where `GND` is middle, `UPDI` left, `VCC` right.

Be careful, **never program it with 5V VCC when the module is connected to a 3.3V chip like FV-1 DCP** - you can damage the DSP or other chips.

I recommend to always disconnect it for programming, using a connector on the `GND` / `OUT` / `VCC` connection.

## Changelog

Revision string of each version is baked into firmware binary.

- v1.0 - `rev_1` - work in progress
  - initial release

## Changes from Hydra Delay Taptempo Buddy

Functional changes (also marked by `MOD:` in the source):

- runs at 5V power supply by default, not 3.3V
- PWM is an LFO waveform, not a DC delay-time / Speed voltage.
- Hydra's digital "Tempo Division Switch" (`DIV`) changed to 3 state analog LFO Shape on pads `SH` / `SHG` / `SHV`.
- LED always blinks, locked to LFO phase (including pot-control mode).
- First tap aligns LFO/LED to the downbeat without changing Speed.
- Tap-session timeout capped at 2.5 s (first-tap window too). Hydra used uncapped 3× tempo and a 1.5 s first-tap window.
- LFO period range 50-2000 ms instead of Hydra delay 150-920 ms.
- Long-press is a latching Leslie ramp (2× faster while held, or 2× slower if period is under 300 ms; latch only when released at the target; early release or unlatch glides home, continuing foot up; `Speed Pot` sets glide time while held).
- Speed pot log map and catch-up takeover from tap or latched Leslie.
- Short tap then long press cycles Random algorithm (Hybrid / S&H / Wander); LED blinks 1-3 times, stored in EEPROM, announced on power-up if Random shape is selected.

## License

© 2026 ElvisAlive Tone. This work is openly licensed via [CC BY-NC-SA](https://creativecommons.org/licenses/by-nc-sa/4.0/)
