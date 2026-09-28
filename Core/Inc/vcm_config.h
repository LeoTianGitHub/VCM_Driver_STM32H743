/**
 * @file    vcm_config.h
 * @brief   VCM current loop - PI + R/L FF, three-level PWM @ 50 kHz
 *
 * Competitor-style: short +/-Vbus pulses, rest of the period Vcoil=0 (LS freewheel).
 * PWM outputs stay enabled (no idle clamp).
 */
#ifndef VCM_CONFIG_H
#define VCM_CONFIG_H

#include <stdint.h>

#define VCM_SYSCLK_HZ           480000000UL
#define VCM_HRTIM_CLOCK_HZ      480000000UL

#define VCM_PWM_FREQ_HZ         50000UL
#define VCM_CTRL_FREQ_HZ        50000UL
#define VCM_PWM_PERIOD          ((uint16_t)(VCM_HRTIM_CLOCK_HZ / VCM_PWM_FREQ_HZ)) /* 9600 */
#define VCM_PWM_REPETITION      ((uint16_t)(VCM_PWM_FREQ_HZ / VCM_CTRL_FREQ_HZ - 1U))
#define VCM_PWM_TS_S            (1.0f / (float)VCM_CTRL_FREQ_HZ)

#define VCM_DT_PRESCALER        HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV8
#define VCM_DT_RISING           0U
#define VCM_DT_FALLING          0U

#define VCM_MOD_MAX             0.48f

/* 0 = bipolar 0.5+/-m; 1 = three-level low-side freewheel (D=2*|m|) */
#define VCM_PWM_MODE_BIPOLAR       0U
#define VCM_PWM_MODE_TRILEVEL      1U
#define VCM_PWM_MODE               VCM_PWM_MODE_TRILEVEL

/*
 * Dual-pulse three-level, one period: +V pulse → coast → -V pulse → coast.
 * Both pulses always stay >= dither; m only lengthens one side.
 * Net Vcoil=(Da-Db)*Vbus=2*m*Vbus. Dither = 1 us (needed for small-I linearity).
 */
#define VCM_TRI_PULSE_NS           1000U
#define VCM_TRI_DITHER_D           ((float)VCM_TRI_PULSE_NS * 1.0e-9f * \
                                 (float)VCM_PWM_FREQ_HZ) /* 0.050 @ 50 kHz */

#define VCM_I_MAX_A             10.0f
#define VCM_I_OCP_A             12.0f
#define VCM_OCP_CONFIRM_SAMPLES 4U

#define VCM_ADC_MID             32768.0f
#define VCM_ADC_VREF_V          3.0f
#define VCM_ADC_COUNTS_PER_V    (VCM_ADC_MID / VCM_ADC_VREF_V)

/* IFB: RS + TPA8001 → ADC1. Adjust GAIN_TRIM after clamp meter vs g_vcm.ifb_a */
#define VCM_IFB_RS_OHM          0.020f
#define VCM_IFB_TPA8001_GAIN    8.2f
#define VCM_IFB_GAIN_TRIM       1.0f
#define VCM_IFB_V_PER_A         (VCM_IFB_GAIN_TRIM * VCM_IFB_TPA8001_GAIN * VCM_IFB_RS_OHM)
#define VCM_IFB_COUNTS_PER_A    (VCM_IFB_V_PER_A * VCM_ADC_COUNTS_PER_V)
#define VCM_IFB_POLARITY        (-1.0f)

/* IREF: ±20 V * op-amp G=0.15 → ±3 V ADC = ±10 A */
#define VCM_IREF_VIN_FULL_V     20.0f
#define VCM_IREF_DIFF_GAIN      0.15f
#define VCM_IREF_GAIN_TRIM      1.0f
#define VCM_IREF_V_PER_A        (VCM_IREF_GAIN_TRIM * VCM_IREF_DIFF_GAIN * \
                                 VCM_IREF_VIN_FULL_V / VCM_I_MAX_A) /* 0.3 V/A */
#define VCM_IREF_COUNTS_PER_A   (VCM_IREF_V_PER_A * VCM_ADC_COUNTS_PER_V)
#define VCM_IREF_POLARITY       (-1.0f)

/* PI: ~3 kHz crossover @ 50 kHz (zero ~320 Hz, PM ~55°).
 * One gain set at every current. Analog RC (C32/C33) is ~14.5 kHz, above this loop.
 * PI follows î, not the raw 2-pt IFB. */
#define VCM_KP                  0.23f
#define VCM_KI                  460.0f
#define VCM_I_INTEGRAL_LIM      0.30f

/* Plant FF: bipolar Vcoil ≈ 2·m·Vbus. L-FF off while tuning PI.
 * Live/UART override uses the same 1st-order as analog RC (C32/C33 =
 * 49.9 Ω × 220 nF → fc≈14.5 kHz) so Live steps look like analog. */
#define VCM_COIL_L_H            0.001188f
#define VCM_COIL_R_OHM          3.62f
#define VCM_VBUS_V              48.0f
#define VCM_VBUS_MIN_V          24.0f
#define VCM_VBUS_MAX_V          48.0f
#define VCM_FF_SCALE            1.0f
#define VCM_L_FF_SCALE          0.0f
#define VCM_FF_MOD_PER_A        (VCM_FF_SCALE * (VCM_COIL_R_OHM + VCM_IFB_RS_OHM) / \
                                 (2.0f * VCM_VBUS_V))
#define VCM_L_FF_MOD_PER_A      (VCM_L_FF_SCALE * VCM_COIL_L_H / (2.0f * VCM_VBUS_V))
#define VCM_IREF_OVR_LPF_A      0.838f   /* 1-exp(-2*pi*14500/50000); Live-tunable */
#define VCM_L_FF_DIDT_MAX       4000.0f

/*
 * Current observer on the 2-point duration-weighted IFB mean.
 *   î ← î + (Ts/L)(2·m·Vbus − R·î) + α(ifb_avg − î)
 * m is last applied (HRTIM preload = voltage that produced this IFB).
 *
 * 8 kHz (α=0.63) was a no-op for buzzing: each sample is 63% raw IFB, so PI
 * still chases CSA/dither aliases. HF current must come from the voltage
 * model; IFB only trims DC. fo=1 kHz.
 * Live: g_vcm.i_obs_a  0.061≈500 Hz  0.096≈800 Hz  0.118≈1 kHz  0.63≈8 kHz.
 */
#define VCM_I_OBS_HZ            1000.0f
#define VCM_I_OBS_A             0.118f   /* 1-exp(-2*pi*1000/50000) */
#define VCM_I_OBS_R_OHM         (VCM_COIL_R_OHM + VCM_IFB_RS_OHM)
#define VCM_I_OBS_TS_OVER_L     (VCM_PWM_TS_S / VCM_COIL_L_H)

/*
 * 0: keep PWM at standstill (competitor-style; no software dead zone).
 * 1: clamp after |IREF| < ENTER for DEB samples (quiet idle).
 */
#define VCM_IDLE_CLAMP_EN          0U
#define VCM_IDLE_ENTER_A           0.020f
#define VCM_IDLE_EXIT_A            0.035f
#define VCM_IDLE_ENTER_DEB         5000U  /* 100 ms @ 50 kHz */

/* No error/IREF deadband: PI stays closed through zero (small-current linear).
 * Quiet idle is UART 'Z' only. 2-sample mean feeds the observer, PI follows î. */

/* 0: enable goes straight to RUN (no IFB/IREF offset cal, no 2 ms open loop).
 * 1: measure offsets for VCM_IFB_CAL_SAMPLES with m=0 before closing PI. */
#define VCM_ENABLE_CAL_EN          0U
#define VCM_IFB_CAL_SAMPLES        100U   /* ~2 ms @ 50 kHz; used only if ENABLE_CAL */
#define VCM_IREF_OFFSET_MAX_A      0.050f

#define VCM_ADC_TRIG_EDGE_MARGIN   480U   /* 1 us: coast S&H away from PWM edges */
#define VCM_ADC_ON_EDGE_MARGIN     48U    /* 100 ns: ON-pulse S&H off the switching edge */
#define VCM_ADC_CONV_GUARD         720U   /* ~1.5 us: sample + conv before next trig */
#define VCM_ADC_KERNEL_HZ          24000000UL  /* PLL2P 96 MHz / ASYNC_DIV4 */
#define VCM_ADC_SMP_CYCLES         16.5f       /* CubeMX 16.5; S&H at end of this */
#define VCM_ADC_SMP_DELAY          ((uint16_t)(VCM_ADC_SMP_CYCLES * \
                                 ((float)VCM_HRTIM_CLOCK_HZ / (float)VCM_ADC_KERNEL_HZ) + 0.5f)) /* 330 */
#define VCM_ADC_MIN_COAST          ((uint16_t)(2U * VCM_ADC_TRIG_EDGE_MARGIN + \
                                 VCM_ADC_SMP_DELAY)) /* ~2.7 us quiet window */
#define VCM_ADC_N                  2U     /* + and − slots; duration-weighted → observer */

/* ADCTRG1 = TA CMP2 | TB CMP3. DMA length 2.
 * Prefer coast midpoint. If coast < MIN_COAST, sample inside the MOS ON pulse
 * (series CSA is valid while conducting) with the whole S&H window inset from
 * both PWM edges. Trigger is advanced by SMP_DELAY so S&H ends on the hold. */

/* Gain-cal: UART force current (clamp meter) and rolling mean window */
#define VCM_CAL_FORCE_A            0.50f
#define VCM_CAL_AVG_ALPHA          (1.0f / 2550.0f) /* ~51 ms EMA @ 50 kHz */

#define VCM_DRV_EN_Pin             GPIO_PIN_0
#define VCM_DRV_EN_GPIO_Port       GPIOB
#define VCM_DRV_EN_ACTIVE_LEVEL    GPIO_PIN_RESET
#define VCM_FAULT_Pin              GPIO_PIN_1
#define VCM_FAULT_GPIO_Port        GPIOB
#define VCM_FAULT_ACTIVE_LEVEL     GPIO_PIN_SET
#define VCM_FAULT_INACTIVE_LEVEL   GPIO_PIN_RESET
#define VCM_PROCESS_LED_Pin        GPIO_PIN_3
#define VCM_PROCESS_LED_GPIO_Port  GPIOE

#define VCM_EN_GLITCH_SAMPLES   20U  /* ~400 us @ 50 kHz */
#define VCM_EN_OFF_DEBOUNCE_MS  2U
#define VCM_CAL_REUSE_MS        5000U

#define VCM_DIAG_EN             1
#define VCM_DIAG_STEP_A         0.20f
#define VCM_DIAG_DONE_BAND      0.10f
#define VCM_DIAG_TIMEOUT_TICKS  1000U  /* 20 ms @ 50 kHz */
#define VCM_DIAG_UART_MARK      0

/* UART cal cmds (ASCII; avoid 0x05 / 0xA0 used by IAP) */
#define VCM_UART_CMD_GAIN       ((uint8_t)'G')  /* print iref/ifb means */
#define VCM_UART_CMD_FORCE      ((uint8_t)'F')  /* override IREF = CAL_FORCE_A */
#define VCM_UART_CMD_ZERO       ((uint8_t)'Z')  /* override IREF = 0 */
#define VCM_UART_CMD_ANALOG     ((uint8_t)'A')  /* clear override, use ADC */
#define VCM_UART_CMD_BIPOLAR    ((uint8_t)'B')  /* pwm_mode = bipolar */
#define VCM_UART_CMD_TRILEVEL   ((uint8_t)'T')  /* pwm_mode = three-level */

/* Host frames: "$" ... "\n" @ 115200. Avoid 0x05 / 0xA0 (IAP).
 *   $I  identify     $P  telemetry     $D  dump params
 *   $S kp=230 ki=460000 obs=118 lpf=838 ff=37916 vbus=48000 r=3620 l=1188
 *      (kp/obs/lpf/vbus/r ×1000, ki ×1000, ff ×1e6, l = µH)
 *   $R <mA>  IREF override (PI on)   $Z $A $F $B $T $C $G as lines too
 *   $W  save RAM params to FLASH Sector3   $L reload   $E factory erase
 */
#define VCM_HOST_PROTO          2U
#define VCM_HOST_LINE_MAX       128U

#endif /* VCM_CONFIG_H */
