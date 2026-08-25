# LFO Taptempo Buddy

ATtiny402 tap-tempo LFO firmware. PWM output is a waveform (sin, triangle, ramp up,
ramp down, pulse, random) at the speed set by the `Speed Pot` or by tapping. Intended
for tremolo and similar analog modulation.

Finished in **ToDo** 2026

This is an adaptation of [Hydra Delay Taptempo Buddy](https://github.com/ElvisAlive-Tone/HydraDelayTaptempoBuddy).
Hardware (PCB, schematics, BOM) is the same module.

**Tip:** You can use my [Simple Serial UPDI programmer](https://github.com/ElvisAlive-Tone/updipcb) to program u-controller for this project.

## Features

- LFO speed can be controlled by `Speed Pot` or tapped by `Tap Button`. Period range is 50 ms (~20 Hz) to 2000 ms (~0.5 Hz).
- `LED` always blinks at the LFO rate with 50% duty cycle, locked to the waveform (both pot and tap control).
- Tap `Tap Button` at least two times to switch from `Speed Pot` control to `Tap Button` control.
  - First tap aligns LFO and `LED` to the downbeat without changing rate.
  - Second tap must follow within 2.5 s after the first one.
  - Subsequent tap times are averaged until tapping finishes.
  - Tapping finishes if the next tap is not performed for 3 periods of the current tempo, but never later than 2.5s.
  - A gap between 2s and 2.5s still counts as a tap; the period is then clamped to 2s.
- `Shape` rotary switch (on the old `Tap to Head` / `DIV` pin) selects the LFO waveform:
  - sin, triangle, ramp up, ramp down, pulse, random.
  - Random algorithm (S&H / wander / hybrid) is cycled with a **short tap then a long press** (hold the second press over 500 ms). Works in any shape; takes effect when Random is selected.
    `LED` blinks **1 / 2 / 3** times (S&H / wander / hybrid) and the choice is stored in EEPROM.
    The same blink is shown on power-up when the rotary is already on Random. See [Random shape algorithm](#random-shape-algorithm).
- Long press from rest (over 500 ms, no short tap before it) is a Leslie ramp:
  - While held, speed ramps up to 2x (half period, clamped at 50 ms) and stays there.
  - On release, speed ramps back to the original rate.
  - Press again during the slowdown to speed up again.
  - Ramping velocity depends on `Speed Pot` (higher = faster ramp).
- Move `Speed Pot` at least 5% to switch control back to it.
- Current `Speed Pot` or `Tap Button` control state, together with the tapped-in tempo, is preserved over power-off.
- Trimmer or fixed resistor to set LED brightness.
- UPDI pins to re-program soldered u-controller.

## Project Content

- `firmware.hex` - firmware binary
- `firmware/` - VSCode/[PlatformIO](https://docs.platformio.org/en/latest/platforms/atmelmegaavr.html) project with firmware
- `LFOBuddy.dch` - schematics
- `LFOBuddy-rev1_gerber.zip` - Gerber file for PCB fabrication
- `LFOBuddy.dip` - PCB design file

Schematics and PCB design file can be opened/edited by [DipTrace](https://diptrace.com/).

## Wiring the module

**TODO** PCB. 

- Analog `OUT` is now the LFO control voltage PWM:
  - it needs on-board filter to smooth it out, with 100–200Hz cut frequency (eg. RC 10kohm/100nF into high-Z load)
  - it has max amplitude of 3.3V, so may need some scalling, depending how you drive rest of the circuit (VACTROL LEDs, JFETs etc)
- Plan module and controls (`Tap Button`, `Shape` rotary, `Speed Pot`, `LED`) placement. Use long enough wires.
- Power and output:
  - `GND` - ground
  - `OUT` - LFO voltage
  - `3V3` - 3.3 V power
  - If you reuse a Hydra-style connector on a delay PCB: closest-to-edge square pad `GND`, center `OUT`, third `3V3`. A connector makes it easy to disconnect for programming.
- `Speed` pot - connect pot's 1, 2 and 3 lugs to the module's `P1`, `P2` and `P3`.
  Use `B` type pot, from `B10k` up to `B100k`. Higher voltage is higher Speed (shorter LFO period).
- `Tap Button` - connect momentary button to the module's `TAP` pads.
- `LED` - connect LED to the module's `L+` and `L-`. Use `TL` trimmer to set LED's brightness. Used `2k` value should
  be OK for the most LED types, if too small for your LED, use higher trimmer value, or connect additional resistor
  in series. Alternatively use fixed value resistor `RL` if you figured out exact value and wanna to save some space.
- `Shape` switch - 6-position rotary with 5 equal resistors (e.g. 10k) as a divider. Switch common to `DIV` (PA2 / AIN2).

```
  3V3 ---- pos0  sin        ADC ~1023
        R
       ---- pos1  triangle   ADC ~818
        R
       ---- pos2  ramp up    ADC ~614
        R
       ---- pos3  ramp down  ADC ~409
        R
       ---- pos4  pulse      ADC ~205
        R
  GND ---- pos5  random     ADC ~0
```

On the Hydra buddy PCB, `D4` / `D2` were the two sides of the old on/on Head switch (VCC / GND). You can use those pads as the 3V3 and GND ends of the ladder.

## Building module

**TODO** Module schematics:

<img src="img/schematics.png" width="600px" alt="Module schematics">

PCB BOM:

**TODO** update for new schematic and PCB

| Markings           | Value             | PCB packaging type                                    |
| ------------------ | ----------------- | ----------------------------------------------------- |
| R1, R2             | 1k                | 1206                                                  |
| R4                 | 10k               | 1206                                                  |
| C1, C3, C4         | 100n              | 1206                                                  |
| C2                 | 10u               | 5,3mm                                                 |
| TL                 | 2k                | 3362 trimmer                                          |
| RL (instead of TL) | matching LED      | 1206                                                  |
| U1                 | ATtiny 402 or 412 | SOIC-8                                                |
| UPDI               |                   | 3 pins header connector (male or female, it's on you) |

External components:

| Markings     | Value                                      |
| ------------ | ------------------------------------------ |
| `Speed Pot`  | B10k - B100k                               |
| LED          | any color and size LED                     |
| `Tap Button` | any momentary switch                       |
| `Shape`      | 6-position rotary                          |

PCB:

<img src="img/pcb.png" width="300px" alt="PCB">

## Functionality Tweaking

### LFO period range

Source code contains constants for the LFO period range and tap timeout. Change them and rebuild if you want a different speed span.

```c
const uint16_t c_lfo_max = 2000;   // slowest LFO period [ms] (~0.5 Hz) - Speed pot on minimum
const uint16_t c_lfo_min = 50;     // fastest LFO period [ms] (~20 Hz) - Speed pot on maximum
const uint16_t c_lfo_range = 1950; // c_lfo_max - c_lfo_min [ms]
const uint16_t c_tap_end_max = 2500; // max wait for the next tap [ms]; must be > c_lfo_max
```

PWM carrier is 20 kHz, range `0..c_pwm_max` (999). The analog LFO appears after the RC filter on `OUT`.

### Random shape algorithm

Random still follows the tapped or pot speed: one new random target per LFO cycle. What it *does* with that target is the mode:

- **S&H** — jumps to a random level and sits there until the next beat (classic stepped random).
- **Wander** — glides smoothly from the last level to the next over the cycle (no stairs; nicer when the LFO is slow).
- **Hybrid** — S&H when the period is shorter than 400 ms, wander when it is slower.

**How to switch:** tap once (short), then press and hold (longer than 500 ms). That is not a new tempo, and it is not Leslie — Leslie is a hold *without* a short tap first. You can do the gesture with any shape selected; it only matters when the rotary is on Random.

The LED then blinks the new mode. The choice is stored in EEPROM, so it survives power-off:

- 1 blink — S&H
- 2 blinks — wander
- 3 blinks — hybrid

The same blinks happen at power-up, but only if the rotary is already on Random (so other shapes do not look like a mode change). To always blink on boot, set `ANNOUNCE_RANDOM_ON_BOOT_ALWAYS` to `1` in `firmware/src/main.c`.

A blank chip starts in S&H. To change that default, edit `LFO_RANDOM_MODE` in the same file. 

The 400 ms hybrid split is `c_random_hybrid_ms`.

## Compiling firmware

I'm using VS Code with Platform IO extension. You have to have `Atmel megaAVR` Platform installed in the Platform IO.

`platformio.ini` file is commited in `firmware` folder with all the basic settings,
including [pymcuprog](https://github.com/microchip-pic-avr-tools/pymcuprog) related settings for
my [Simple Serial UPDI programmer](https://github.com/ElvisAlive-Tone/updipcb).

So you can just open the folder in VS Code and it should work.

## Programming u-controller

Easiest way is to set u-controller up before soldering it to the PCB.

If you want to change firmware later, you can use UPDI pins on the module, where `GND` is middle, `UPDI` left, `VCC` right.

Be carefull, **never program it with 5V VCC when the module is connected to a 3.3V pedal**
(Hydra / FV-1 and similar) - you can damage the DSP or other chips.

I recommend to always disconnect it for programming, using a connector on the `GND` / `OUT` / `3V3` connection.

## Changes from Hydra Delay Taptempo Buddy

Functional changes (also marked by `MOD:` in the source):

- PWM is an LFO waveform, not a DC delay-time / Speed voltage.
- "Tempo Division Switch" pin is analog LFO shape select (rotary + resistor ladder, 6 voltage layers) instead of digital Head 2 / Head 4.
- LED always blinks, locked to LFO phase (including pot-control mode).
- First tap aligns LFO/LED to the downbeat without changing rate.
- Tap-session timeout capped at 2.5 s (first-tap window too). Hydra used uncapped 3× tempo and a 1.5 s first-tap window.
- LFO period range 50-2000 ms instead of Hydra delay 150-920 ms.
- Long-press is a Leslie ramp (2× speed while held, ramp back on release; Speed pot sets ramp velocity), not a bounce through the whole delay range.
- Short tap then long press cycles Random algorithm (S&H / wander / hybrid); LED blinks 1-3 times; stored in EEPROM.

## License

© 2026 ElvisAlive Tone. This work is openly licensed via [CC BY-NC-SA](https://creativecommons.org/licenses/by-nc-sa/4.0/)
