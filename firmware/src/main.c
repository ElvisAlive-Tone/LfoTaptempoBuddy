/*
 * LFO Taptempo Buddy
 *
 * ATtiny402 tap-tempo LFO firmware. PWM output is a waveform (sin, triangle,
 * ramp up, ramp down, pulse, random) at the speed set by the Speed pot or by
 * tapping. Intended for tremolo and similar analog modulation.
 *
 * Adaptation of Hydra Delay Taptempo Buddy:
 * https://github.com/ElvisAlive-Tone/HydraDelayTaptempoBuddy
 *
 * August 2026
 * by Vlastimil Elias from Elvis Alive Tone
 *
 * This work is openly licensed via CC BY-NC-SA.
 * https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode
 *
 * MODs vs Hydra Delay Taptempo Buddy (marked in source by `MOD:`):
 * - PWM is an LFO waveform, not a DC delay-time / Speed voltage
 * - "Tempo Division Switch" pin is analog LFO shape select: on-off-on
 *   (GND / VCC/2 / VCC) for sin, triangle, pulse; hold Tap while flipping
 *   for ramp up, ramp down, random. Bank stored in EEPROM.
 * - LED always blinks, locked to LFO phase (including pot-control mode)
 * - First tap aligns LFO/LED to the downbeat without changing rate
 * - Tap-session timeout capped at c_tap_end_max (first-tap window too)
 * - LFO period range 50-2000 ms instead of Hydra delay 150-920 ms
 * - Long-press is a Leslie ramp (2x faster while held, or 2x slower if already
 *   fast; ramp back on release; Speed pot sets ramp velocity), not a bounce
 *   through the whole delay range
 * - Short tap then long press cycles Random algorithm (hybrid / S&H / wander);
 *   LED blinks 1-3 times to show the mode; stored in EEPROM
 *
 * Pinout:
 * 1: VDD/VCC
 * 2: Momentary Tap Tempo Button Digital Input
 * 3: LED+ Output
 * 4: Speed Potentiometer Analog Input
 * 5: LFO Shape on-off-on Analog Input (AIN2) // MOD: was digital Head 2/4
 * 6: Reset/UPDI
 * 7: PWM Output // MOD: LFO waveform (was DC Speed voltage)
 * 8: GND
 *
 * Shape switch (PA2 / AIN2): on-off-on, common to DIV.
 * Center (off) is VCC/2 via two equal resistors (e.g. 10k) from 3V3 to GND.
 *
 *   GND  ----  sin            /  ramp up    (hold Tap while flipping)
 *   1/2  ----  triangle       /  ramp down
 *   VCC  ----  pulse          /  random
 *
 * Flip without Tap: normal bank. Flip with Tap held: alt bank. Bank is in EEPROM.
 * Hold-flip is not a new Speed, Leslie, or Random-algorithm change (same as Hydra head bank).
 * The Tap press still aligns LFO phase.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 20000000UL
#include <util/delay.h>

#define PWM_PIN 3 // PWM is the analog LFO after RC filtering
#define TAP_PIN 6
#define LED_PIN 7
#define POT_PIN 1           // higher voltage higher Speed -> shorter LFO period
#define DIV_PIN 2           // MOD: analog shape select (AIN2); on-off-on GND / VCC/2 / VCC
#define DEBOUNCE_TIME 900   // Tap tempo button debounce time [us]
#define POT_MAX_VALUE 0x3FF // 10bit ADC max value to convert pot

// MOD: LFO shapes from on-off-on layers on DIV_PIN (bank from Tap-while-flip, EEPROM)
#define LFO_SIN 0
#define LFO_TRI 1
#define LFO_RAMP_UP 2
#define LFO_RAMP_DOWN 3
#define LFO_PULSE 4
#define LFO_RANDOM 5

// Which algorithm the Random shape (LFO_RANDOM) uses at runtime (`lfo_random_mode`).
// LFO_RANDOM_MODE is the default when EEPROM is empty / invalid.
// Cycle at runtime: short tap, then long press (see main tap handler).
//   LFO_RANDOM_HYBRID - S&H when period < c_random_hybrid_ms, wander when slower (default; 1 blink)
//   LFO_RANDOM_SNH    - stepped sample-and-hold: new level once per cycle, held until the next (2 blinks)
//   LFO_RANDOM_WANDER - linear glide from the last value to the next over each cycle (smooth; 3 blinks)
#define LFO_RANDOM_HYBRID 0
#define LFO_RANDOM_SNH 1
#define LFO_RANDOM_WANDER 2
#define LFO_RANDOM_MODE LFO_RANDOM_HYBRID
// 0 = announce Random hybrid/S&H/wander on power-up only if that shape is selected
// 1 = always announce on power-up (any shape)
#define ANNOUNCE_RANDOM_ON_BOOT_ALWAYS 0
// Trailing dark after 1/2/3 announce blinks. Literal in announce_random_mode — _delay_ms needs a compile-time constant.
#define C_RANDOM_ANNOUNCE_DARK_MS 700

#define SHAPE_BANK_NORMAL 0 // sin / triangle / pulse
#define SHAPE_BANK_ALT 1    // ramp up / ramp down / random

const uint16_t EEPROM_TAP = 0x1400;    // one byte to store `tap` variable
const uint16_t EEPROM_TEMPO = 0x1401;  // two bytes to store `mstempo` variable
const uint16_t EEPROM_RANDOM = 0x1403; // one byte to store `lfo_random_mode`
const uint16_t EEPROM_BANK = 0x1404;   // one byte to store shape bank (0=normal, 1=alt)

// MOD: LFO period range (was Hydra delay 150-920 ms)
const uint16_t c_pwm_max = 999;
const uint16_t c_lfo_max = 2000;   // slowest LFO period [ms] (~0.5 Hz) - Speed pot on minimum
const uint16_t c_lfo_min = 50;     // fastest LFO period [ms] (~20 Hz) - Speed pot on maximum
const uint16_t c_lfo_range = 1950; // c_lfo_max - c_lfo_min [ms]
// MOD: cap on wait-for-next-tap (Hydra used uncapped 3*tempo and a 1.5 s first-tap window)
const uint16_t c_tap_end_max = 2500;
// Used only when lfo_random_mode is LFO_RANDOM_HYBRID: period [ms] below this → S&H, slower → wander.
const uint16_t c_random_hybrid_ms = 400;
// Leslie hold: Speed multiplier (2 = twice as fast = half period). Integer; 3 = 3x, etc.
const uint8_t c_leslie_speed = 2;
// Leslie: if current period [ms] is below this, hold slows (× multiplier) instead of speeds (/ multiplier).
const uint16_t c_leslie_slowdown_ms = 300;

// On-off-on shape switch: midpoints between 0, ~512, 1023
const uint16_t c_shape_mid_lo = 256;
const uint16_t c_shape_mid_hi = 768;

// Firmware revision info in uC binary code for further reference.
// Must be volatile so the compiler keeps it.
// Starts by `rev_` to find it easily in the binary - kept in sync with the README.md changelog.
volatile char revision[] = "rev_1";

volatile uint16_t pot;         // current Pot value is stored here from ADC by interrupt handler
volatile uint16_t divsw;       // current shape-switch ADC value (AIN2)
volatile uint8_t adc_shape_ok; // 1 after AIN2 has been sampled at least once
static volatile uint8_t adc_mux_settle; // 1 = discard next result (S/H cap still settling after MUX change)
volatile uint16_t pwm = 500;   // current PWM sample (0..c_pwm_max), written by LFO in TCA ISR
volatile uint16_t ms;          // time [ms] counter for Tap button pressed length and delayed `tap` reset in EEPROM respectively. Incremented in TCA interrupt.

volatile uint16_t lfo_phase = 0; // 16-bit phase accumulator, one wrap = one LFO cycle
volatile uint16_t lfo_inc = 131; // added to phase every 1 ms (65536/500 ≈ 131)
volatile uint8_t lfo_shape = LFO_SIN;
volatile uint8_t lfo_random_mode = LFO_RANDOM_MODE;
volatile uint8_t led_follow_lfo = 1; // 0 while announcing random mode with LED blinks

// random LFO state - used only from TCA ISR
static uint16_t lfsr = 0xACE1u;
static uint8_t rnd_from = 128;
static uint8_t rnd_to = 128;
static uint8_t rand_armed = 1;

// Fuse settings left at device defaults; programmed via platformio.ini if needed.
// FUSES = {
// .WDTCFG = 0x00,  // WDTCFG {PERIOD=OFF, WINDOW=OFF} - DEFAULT VALUE
// .BODCFG = 0x00,  // BODCFG {SLEEP=DIS, ACTIVE=DIS, SAMPFREQ=1KHz, LVL=BODLEVEL0} - DEFAULT VALUE
// .OSCCFG = 0x02,  // OSCCFG {FREQSEL=20MHZ, OSCLOCK=CLEAR} - DEFAULT VALUE
// .TCD0CFG = 0x00, // TCD0CFG {CMPA=CLEAR, CMPB=CLEAR, CMPC=CLEAR, CMPD=CLEAR, CMPAEN=CLEAR, CMPBEN=CLEAR, CMPCEN=CLEAR, CMPDEN=CLEAR} - DEFAULT VALUE
// .SYSCFG0 = 0xF6, // SYSCFG0 {EESAVE=CLEAR, RSTPINCFG=GPIO, CRCSRC=NOCRC} - DEFAULT VALUE
// .SYSCFG1 = 0x07, // SYSCFG1 {SUT=64MS} - DEFAULT VALUE
// .APPEND = 0x00,  // APPEND {APPEND=User range:  0x0 - 0xFF} - DEFAULT VALUE
// .BOOTEND = 0x00, // BOOTEND {BOOTEND=User range:  0x0 - 0xFF} - DEFAULT VALUE
//};

void IO_Init(void)
{
    // PWM & LED as outputs
    PORTA.DIRSET = (1 << PWM_PIN) | (1 << LED_PIN);
    // pull up for tap button only
    PORTA_PIN6CTRL |= PORT_PULLUPEN_bm; // Tap tempo button
    PORTA_PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTA_PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc; // Pot pin
}

// configure ADC
void ADC_Config(void)
{
    // reference to VCC with the right sampling capacitor
    ADC0.CTRLC = ADC_SAMPCAP_bm | ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV32_gc;
    // ADC0.CTRLB = ADC_SAMPNUM_ACC32_gc;
    // Start with ADC channel 1 for Tempo Pot
    ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
    // enable result ready interrupt
    ADC0.INTCTRL = ADC_RESRDY_bm;
    // 10bits freerun & enable
    ADC0.CTRLA |= ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc | ADC_FREERUN_bm;
    adc_mux_settle = 1; // discard first conversion after enable
    // Start conversion
    ADC0.COMMAND = ADC_STCONV_bm;
}

// interrupt handler for ADC
// ADC mux: pot (AIN1) and on-off-on shape switch (AIN2)
// Discard the first conversion after each MUX change — the sample cap still
// holds the previous channel (shape sits at ~VCC/2, which pulled pot toward center).
ISR(ADC0_RESRDY_vect)
{
    if (adc_mux_settle)
    {
        adc_mux_settle = 0;
    }
    else
    {
        switch (ADC0.MUXPOS)
        {
        case ADC_MUXPOS_AIN1_gc:
            pot = ADC0.RES;
            ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc;
            adc_mux_settle = 1;
            break;

        case ADC_MUXPOS_AIN2_gc:
            divsw = ADC0.RES;
            adc_shape_ok = 1;
            ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
            adc_mux_settle = 1;
            break;

        default:
            ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
            adc_mux_settle = 1;
            break;
        }
    }

    ADC0.INTFLAGS = ADC_RESRDY_bm;
}

// configure ms timer for tap tempo and 20 kHz PWM carrier
void TCA_Config(void)
{
    // Enable PWM output 0 (PA3), single slope PWM
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm;
    // Set period to 1000 (20kHz for 20MHz clock)
    TCA0.SINGLE.PER = c_pwm_max;
    // set initial ratio to 50%
    TCA0.SINGLE.CMP0 = 500;
    // Enable overflow interrupt
    TCA0.SINGLE.INTCTRL |= TCA_SINGLE_OVF_bm;
    // Enable timer
    TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm;
}

static inline uint8_t rnd8(void)
{
    uint8_t lsb = lfsr & 1;
    lfsr >>= 1;
    if (lsb)
    {
        lfsr ^= 0xB400u;
    }
    return (uint8_t)lfsr;
}

// Pick a new random target once per LFO cycle (phase wrap).
static void random_on_wrap(uint16_t phase)
{
    if (phase < lfo_inc)
    {
        if (rand_armed)
        {
            rnd_from = rnd_to;
            rnd_to = rnd8();
            rand_armed = 0;
        }
    }
    else
    {
        rand_armed = 1;
    }
}

// Linear glide from rnd_from to rnd_to over the cycle (smooth wander).
static uint8_t random_wander(uint16_t phase)
{
    int16_t d = (int16_t)rnd_to - (int16_t)rnd_from;
    return (uint8_t)(rnd_from + (((int32_t)d * (phase >> 8)) >> 8));
}

// Stepped sample-and-hold: hold rnd_to until the next wrap.
static uint8_t random_snh(void)
{
    return rnd_to;
}

// S&H at higher speeds, wander at lower speeds (see c_random_hybrid_ms).
static uint8_t random_hybrid(uint16_t phase)
{
    if (lfo_inc >= (uint16_t)(65536UL / c_random_hybrid_ms))
    {
        return random_snh();
    }
    return random_wander(phase);
}

static uint8_t lfo_random(uint16_t phase)
{
    random_on_wrap(phase);
    if (lfo_random_mode == LFO_RANDOM_WANDER)
    {
        return random_wander(phase);
    }
    if (lfo_random_mode == LFO_RANDOM_HYBRID)
    {
        return random_hybrid(phase);
    }
    return random_snh();
}

// 8-bit waveform from 16-bit phase. Simple integer math, parabola sine, LFSR random.
static uint8_t lfo_wave8(uint16_t phase, uint8_t shape)
{
    uint8_t p = phase >> 8;

    switch (shape)
    {
    case LFO_TRI:
    {
        // fold 0..127 then scale to 0..255 (<<1 only reached 254)
        uint8_t t = (p & 0x80) ? (uint8_t)~p : p;
        return (uint8_t)(((uint16_t)t * 255) / 127);
    }

    case LFO_RAMP_UP:
        return p;

    case LFO_RAMP_DOWN:
        return (uint8_t)~p;

    case LFO_PULSE:
        return (phase & 0x8000) ? 255 : 0;

    case LFO_RANDOM:
        return lfo_random(phase);

    default: // LFO_SIN - parabola approximation, no table, no sin()
    {
        uint8_t x = p & 0x7F;
        uint8_t y = ((uint16_t)x * (128 - x)) >> 5; // 0..128
        if (phase & 0x8000)
        {
            return (y >= 128) ? 0 : (uint8_t)(128 - y);
        }
        return (y >= 128) ? 255 : (uint8_t)(128 + y);
    }
    }
}

static inline uint16_t wave8_to_pwm(uint8_t w)
{
    return (uint16_t)(((uint32_t)w * c_pwm_max) / 255);
}

// interrupt handler for TCA timer - time counters, LFO sample, PWM output
ISR(TCA0_OVF_vect)
{
    // set pwm output value
    TCA0.SINGLE.CMP0BUF = pwm;

    // count ms
    static uint8_t count;
    count++;
    // this interrupt is called at 20kHz frequency
    if (count >= 20)
    {
        count = 0;
        ms++;

        // MOD: 1 kHz LFO tick (Hydra wrote a DC pwm value from the main loop)
        lfo_phase += lfo_inc;
        pwm = wave8_to_pwm(lfo_wave8(lfo_phase, lfo_shape));

        if (led_follow_lfo)
        {
            // MOD: LED follows LFO phase for pot and tap (Hydra was solid-on in pot mode)
            if (lfo_phase & 0x8000)
            {
                PORTA.OUTCLR = (1 << LED_PIN);
            }
            else
            {
                PORTA.OUTSET = (1 << LED_PIN);
            }
        }
    }

    // Clear interrupt flag
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

// Tap button debounce with 900us time - returns 1 if Tap button is currently pressed
uint8_t debounce(void)
{
    if (!(PORTA.IN & (1 << TAP_PIN)))
    {
        _delay_us(DEBOUNCE_TIME);
        if (!(PORTA.IN & (1 << TAP_PIN)))
        {
            return (1);
        }
    }
    return (0);
}

// Atomic 16-bit access vs ISR (AVR is 8-bit). Same cli/sei style as HydraDelayTaptempoBuddy.
static inline uint16_t read_u16(volatile uint16_t *p)
{
    cli();
    uint16_t v = *p;
    sei();
    return v;
}

static inline void write_u16(volatile uint16_t *p, uint16_t v)
{
    cli();
    *p = v;
    sei();
}

// Copy 16-bit ADC samples; ISR can otherwise tear pot/divsw across two bytes
static inline void snap_adc(uint16_t *p, uint16_t *d)
{
    cli();
    *p = pot;
    *d = divsw;
    sei();
}

static inline uint16_t adc_delta(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

// On-off-on: GND / ~VCC/2 / VCC → layer 0 / 1 / 2
uint8_t layer_from_adc(uint16_t adc)
{
    if (adc < c_shape_mid_lo)
    {
        return 0;
    }
    if (adc < c_shape_mid_hi)
    {
        return 1;
    }
    return 2;
}

// MOD: layer + bank → shape (normal: sin/tri/pulse, alt: ramp up/down/random)
uint8_t shape_from_layer(uint8_t layer, uint8_t bank)
{
    if (bank == SHAPE_BANK_ALT)
    {
        if (layer == 0)
        {
            return LFO_RAMP_UP;
        }
        if (layer == 1)
        {
            return LFO_RAMP_DOWN;
        }
        return LFO_RANDOM;
    }
    if (layer == 0)
    {
        return LFO_SIN;
    }
    if (layer == 1)
    {
        return LFO_TRI;
    }
    return LFO_PULSE;
}

// one blink of LED (150 ms on, 150 ms off); caller must set led_follow_lfo = 0
void blink(void)
{
    PORTA.OUTSET = (1 << LED_PIN);
    _delay_ms(150);
    PORTA.OUTCLR = (1 << LED_PIN);
    _delay_ms(150);
}

// LED: 1 blink = hybrid, 2 = S&H, 3 = wander
void announce_random_mode(uint8_t mode)
{
    led_follow_lfo = 0;
    PORTA.OUTCLR = (1 << LED_PIN);
    _delay_ms(250);
    for (uint8_t i = 0; i <= mode; i++)
    {
        blink();
    }
    _delay_ms(C_RANDOM_ANNOUNCE_DARK_MS);
    led_follow_lfo = 1;
}

// persist NVM Page changes into EEPROM
void eeprom_persist(void)
{
    while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm)
        ; // Wait for EEPROM not busy.
    // disable interrupts as next code is CPU cycle sensitive
    cli();
    CPU_CCP = CCP_SPM_gc; // Unlock NVMCTRL.CTRLA write protection.
    NVMCTRL.CTRLA = NVMCTRL_CMD_PAGEERASEWRITE_gc;
    // reenable interrupts
    sei();
}

// PAGEERASEWRITE erases the whole EEPROM page — write tap, tempo, random mode and shape bank together
void eeprom_save(uint8_t tap_val, uint16_t tempo_val, uint8_t rnd_mode, uint8_t bank)
{
    *(uint8_t *)(EEPROM_TAP) = tap_val;
    *(uint16_t *)(EEPROM_TEMPO) = tempo_val;
    *(uint8_t *)(EEPROM_RANDOM) = rnd_mode;
    *(uint8_t *)(EEPROM_BANK) = bank;
    eeprom_persist();
}

// MOD: Speed pot to LFO period [ms] - higher pot = faster LFO = shorter period
uint16_t pot_to_period(uint16_t p)
{
    return (uint16_t)(c_lfo_max - ((uint32_t)p * c_lfo_range) / POT_MAX_VALUE);
}

// MOD: phase increment so one wrap happens in `period_ms` milliseconds (1 kHz tick)
uint16_t lfo_inc_from_ms(uint16_t period_ms)
{
    if (period_ms < c_lfo_min)
    {
        period_ms = c_lfo_min;
    }
    else if (period_ms > c_lfo_max)
    {
        period_ms = c_lfo_max;
    }
    return (uint16_t)(65536UL / period_ms);
}

// MOD: write LFO rate from main loop; 16-bit access must be atomic vs ISR
void set_lfo_period(uint16_t period_ms, uint8_t restart)
{
    uint16_t inc = lfo_inc_from_ms(period_ms);
    cli();
    lfo_inc = inc;
    if (restart)
    {
        lfo_phase = 0;
    }
    sei();
}

// MOD: align LFO/LED to downbeat without changing rate (first tap)
void lfo_restart_phase(void)
{
    write_u16(&lfo_phase, 0);
}

// MOD: keep LFO period in c_lfo_min .. c_lfo_max
uint16_t period_range(uint16_t period)
{
    if (period > c_lfo_max)
    {
        return c_lfo_max;
    }
    else if (period < c_lfo_min)
    {
        return c_lfo_min;
    }
    else
    {
        return period;
    }
}

int main(void)
{
    // Unlocking protected registers and setting main clock to 20MHz
    CPU_CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0;
    CLKCTRL.MCLKLOCK |= CLKCTRL_LOCKEN_bm;

    // init other values
    uint8_t currentstate = 0;       // Current state of Tap button in the cycle of main loop
    uint8_t laststate = 0;          // Laststate of Tap button
    uint8_t nbtap = 0;              // Number of subsequent taps during current tapping
    uint8_t tapping = 0;            // Tapping currently in progress (1) or not (0)
    uint16_t divtempo = 500;        // Current LFO period in [ms]
    uint16_t previouspot;           // Previous Time Pot value to be able to detect change 0-1024
    uint8_t eeprom_reset_tap = 0;   // Flag for delayed `tap` reset in EEPROM
    uint8_t first_tap_released = 0; // 1 after a short first tap; next long press cycles random mode
    uint8_t second_down = 0;        // 1 while the press after that short tap is held
    uint16_t pending_interval = 0;  // gap [ms] from first tap to that second press
    uint8_t shape_selecting = 0;    // 1 if this Tap press flipped the shape switch — not tempo/Leslie/Random cycle
    uint8_t shape_bank = SHAPE_BANK_NORMAL;
    uint8_t shape_layer = 0;

    // read values from EEPROM
    while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm) // Wait for EEPROM not busy.
        ;
    uint8_t tap = *(uint8_t *)(EEPROM_TAP);         // Current control status - 1 for tap tempo, 0 for pot control
    uint16_t mstempo = *(uint16_t *)(EEPROM_TEMPO); // Currently tapped in tempo in ms
    lfo_random_mode = *(uint8_t *)(EEPROM_RANDOM);
    shape_bank = *(uint8_t *)(EEPROM_BANK);
    // check values from EEPROM and patch them just in case
    if (mstempo < c_lfo_min || mstempo > c_lfo_max)
    {
        mstempo = 500;
    }
    if (tap > 1)
    {
        tap = 0;
    }
    if (lfo_random_mode > LFO_RANDOM_WANDER)
    {
        lfo_random_mode = LFO_RANDOM_MODE;
    }
    if (shape_bank > SHAPE_BANK_ALT)
    {
        shape_bank = SHAPE_BANK_NORMAL;
    }

    // enable interrupts
    sei();
    IO_Init();
    TCA_Config();
    ADC_Config();

    // Wait until AIN2 has been sampled — `divsw` starts at 0 (GND = sin / ramp-up).
    ms = 0;
    while (!adc_shape_ok && read_u16(&ms) < 100)
        ;
    _delay_ms(20); // a couple more muxed conversions after the first AIN2 sample

    // Initialize from values stored in EEPROM / pot *before* any LED announce,
    // otherwise the blinks run at the default 500 ms LFO instead of the saved rate.
    uint16_t pot_now;
    uint16_t div_now;
    snap_adc(&pot_now, &div_now);
    shape_layer = layer_from_adc(div_now);
    lfo_shape = shape_from_layer(shape_layer, shape_bank);
    if (tap == 1)
    {
        divtempo = period_range(mstempo);
        mstempo = divtempo;
        set_lfo_period(divtempo, 0);
    }
    else
    {
        divtempo = pot_to_period(pot_now);
        set_lfo_period(divtempo, 0);
    }
    previouspot = pot_now;

    if (lfo_shape == LFO_RANDOM || ANNOUNCE_RANDOM_ON_BOOT_ALWAYS)
    {
        announce_random_mode(lfo_random_mode);
    }

    // Main loop
    while (1)
    {
        snap_adc(&pot_now, &div_now);
        currentstate = debounce();

        // Shape: on-off-on. Flip without Tap = normal bank; flip with Tap held = alt bank.
        // Use the same debounced Tap as the rest of the loop so a bounce cannot stick shape_selecting.
        {
            uint8_t layer = layer_from_adc(div_now);
            if (layer != shape_layer)
            {
                uint8_t tap_held = currentstate;
                shape_layer = layer;
                shape_bank = tap_held ? SHAPE_BANK_ALT : SHAPE_BANK_NORMAL;
                lfo_shape = shape_from_layer(shape_layer, shape_bank);
                eeprom_save(tap, mstempo, lfo_random_mode, shape_bank);
                if (tap_held)
                {
                    // this press is only a shape-bank gesture — not tempo, Leslie, or Random cycle
                    // (first-tap phase align is fine; shape is typically flipped between songs)
                    shape_selecting = 1;
                    nbtap = 0;
                    tapping = 0;
                    first_tap_released = 0;
                    second_down = 0;
                }
            }
        }

        // TIME POT handling
        // if pot move of more than 5%, changing to pot control
        if ((tap == 1 && tapping == 0 && adc_delta(previouspot, pot_now) >= 50) || (tap == 0 && tapping == 0 && adc_delta(previouspot, pot_now) >= 10))
        {
            divtempo = pot_to_period(pot_now);
            set_lfo_period(divtempo, 0);
            previouspot = pot_now;
            // delay EEPROM change due to power-off pot value changes
            if (tap == 1)
            {
                // set flag and reset ms counter for delayed EEPROM write
                eeprom_reset_tap = 1;
                ms = 0;
            }
            tap = 0;
        }

        // handle delayed `tap` reset in EEPROM due to potentional power-off Pot value changes
        if (eeprom_reset_tap == 1)
        {
            // check if tap still 0
            if (tap == 0)
            {
                //  check delay elapsed
                if (read_u16(&ms) > 1000)
                {
                    // write change to EEPROM
                    eeprom_save(0, mstempo, lfo_random_mode, shape_bank);
                    eeprom_reset_tap = 0;
                }
            }
            else
            {
                eeprom_reset_tap = 0;
            }
        }

        // TAP button handling
        if (currentstate == 0 && laststate == 0) // Tap button keeps off
        {
            if (shape_selecting)
            {
                shape_selecting = 0;
            }
            if (nbtap > 1) // if too long between taps, persist values and resets tapping process
            {
                // 3 cycles of current tempo, but never more than c_tap_end_max
                if (read_u16(&ms) > (3 * mstempo) || read_u16(&ms) > c_tap_end_max)
                {
                    // write changed values into EEPROM
                    eeprom_save(1, mstempo, lfo_random_mode, shape_bank);

                    // reset state machine
                    tap = 1;
                    previouspot = pot_now;
                    ms = 0;
                    nbtap = 0;
                    tapping = 0;
                    first_tap_released = 0;
                    second_down = 0;
                }
            }
            else if (nbtap == 1 && read_u16(&ms) > c_tap_end_max) // single tap align only; window must fit c_lfo_max
            {
                ms = 0;
                nbtap = 0;
                tapping = 0;
                first_tap_released = 0;
                second_down = 0;
            }
        }
        else if (currentstate == 0 && laststate == 1) // Tap button just released
        {
            laststate = 0;
            previouspot = pot_now;
            if (shape_selecting)
            {
                shape_selecting = 0;
                nbtap = 0;
                tapping = 0;
                ms = 0;
                first_tap_released = 0;
                second_down = 0;
            }
            else if (nbtap == 1 && !first_tap_released && !second_down && read_u16(&ms) < 500)
            {
                first_tap_released = 1; // short first tap done; a following long press will cycle random mode
            }
            else if (second_down)
            {
                // short second tap: commit tempo from the gap before this press
                mstempo = pending_interval;
                divtempo = period_range(mstempo);
                mstempo = divtempo;
                set_lfo_period(divtempo, 1);
                nbtap = 2;
                tap = 1;
                previouspot = pot_now;
                ms = 0;
                second_down = 0;
                first_tap_released = 0;
            }
        }
        else if (currentstate == 1 && laststate == 0) // Tap button just pressed
        {
            if (nbtap == 0) // MOD: first tap aligns to downbeat, keep current rate
            {
                ms = 0;
                nbtap++;
                laststate = 1;
                tapping = 1;
                first_tap_released = 0;
                second_down = 0;
                lfo_restart_phase();
            }
            else if (nbtap == 1 && first_tap_released && read_u16(&ms) >= 50)
            {
                // press after a short tap: wait to see if it is a short tap (tempo) or long (random mode)
                pending_interval = read_u16(&ms);
                if (TCA0.SINGLE.CNT >= 500)
                {
                    pending_interval++;
                }
                ms = 0;
                laststate = 1;
                second_down = 1;
            }
            else if (nbtap > 1 && read_u16(&ms) >= 50) // further taps: average tempo
            {
                uint16_t interval = read_u16(&ms);
                if (TCA0.SINGLE.CNT >= 500)
                {
                    interval++;
                }

                mstempo = (mstempo + interval) / 2;
                divtempo = (divtempo + mstempo) / 2;
                divtempo = period_range(divtempo);
                mstempo = divtempo;
                set_lfo_period(divtempo, 1);

                nbtap++;
                laststate = 1;
                ms = 0;
                tap = 1;
                previouspot = pot_now;
            }
        }
        else if (currentstate == 1 && laststate == 1) // Tap button keeps on
        {
            if (shape_selecting)
            {
                // Hold-flip already consumed this press; wait for release
            }
            else if (read_u16(&ms) >= 500)
            {
                if (second_down)
                {
                    // MOD: short tap then long press cycles Random algorithm
                    lfo_random_mode++;
                    if (lfo_random_mode > LFO_RANDOM_WANDER)
                    {
                        lfo_random_mode = LFO_RANDOM_HYBRID;
                    }
                    eeprom_save(tap, mstempo, lfo_random_mode, shape_bank);
                    announce_random_mode(lfo_random_mode);
                    while (debounce())
                    {
                    }
                    second_down = 0;
                    first_tap_released = 0;
                    nbtap = 0;
                    tapping = 0;
                    laststate = 0;
                    previouspot = pot_now;
                }
                else
                {
                    // Hold: ramp by c_leslie_speed. Below c_leslie_slowdown_ms → slower (×);
                    // otherwise faster (/). Release: ramp back to the original period.
                    // Speed pot sets how fast that change happens (higher = faster ramp).
                    uint16_t ramp_origin = divtempo;
                    uint16_t ramp_target;
                    if (ramp_origin < c_leslie_slowdown_ms)
                    {
                        uint32_t slow = (uint32_t)ramp_origin * c_leslie_speed;
                        if (slow > c_lfo_max)
                        {
                            ramp_target = c_lfo_max;
                        }
                        else
                        {
                            ramp_target = (uint16_t)slow;
                        }
                    }
                    else
                    {
                        ramp_target = ramp_origin / c_leslie_speed;
                        if (ramp_target < c_lfo_min)
                        {
                            ramp_target = c_lfo_min;
                        }
                    }
                    uint8_t to_target = 1;

                    for (;;)
                    {
                        uint8_t held = !(PORTA.IN & (1 << TAP_PIN));
                        if (to_target)
                        {
                            if (!held)
                            {
                                to_target = 0; // released - start returning
                            }
                            else if (divtempo < ramp_target)
                            {
                                divtempo++;
                            }
                            else if (divtempo > ramp_target)
                            {
                                divtempo--;
                            }
                        }
                        else
                        {
                            if (held)
                            {
                                to_target = 1; // pressed again during return
                            }
                            else if (divtempo < ramp_origin)
                            {
                                divtempo++;
                            }
                            else if (divtempo > ramp_origin)
                            {
                                divtempo--;
                            }
                            else
                            {
                                break; // back at original rate, button up
                            }
                        }

                        set_lfo_period(divtempo, 0);

                        snap_adc(&pot_now, &div_now);

                        // higher Speed pot = faster ramping (shorter wait per 1 ms period step)
                        for (uint16_t i = 0; i < ((POT_MAX_VALUE - pot_now) * 2); i++)
                        {
                            _delay_us(1);
                        }
                    }
                    previouspot = pot_now;
                    // Leslie ate this press — do not treat it as a tap session
                    nbtap = 0;
                    tapping = 0;
                    first_tap_released = 0;
                    second_down = 0;
                    laststate = 0;
                    ms = 0;
                }
            }
        }
    }
}
