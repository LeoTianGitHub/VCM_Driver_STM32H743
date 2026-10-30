/**
 * @file    vcm_config.h
 * @brief   VCM current loop - PI + R/L FF, three-level PWM @ 50 kHz
 *
 * Structure resonance fn≈820 Hz. Competitor-style short
 * +/-Vbus pulses, rest of period Vcoil=0. PWM stays enabled (no idle clamp).
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

/* Bipolar compensation, two edges. $K measures this width.
 * $S dt=<ns>, 0 disables it. HRTIM dead-time registers stay 0.
 * Tri-level does not use this value. Both pulses already stay at
 * VCM_TRI_CM_NS, which covers the gate edge. */
#define VCM_DRV_DT_NS              400U
#define VCM_DRV_DT_NS_MAX          1000U
#define VCM_DRV_DT_HOLD_A          0.080f
/* Second current-loop step, applied to the −pulse of the same period.
 * The 50 kHz tick (notch, gain zone, capture, diag) stays on VCM_PWM_TS_S. */
#define VCM_HALF_TS_S              (VCM_PWM_TS_S * 0.5f)
/* Both tri-level pulses stay at least this wide while the current loop
 * is closed, including at zero command. 600 ns is above the 200 ns gate
 * edge, so blanking cannot swallow either pulse. At 48 V the ripple is
 * about 24 mA peak to peak. The command only lengthens one side.
 * Open-loop hold (UART Z) keeps both pulses off. */
#define VCM_TRI_CM_NS              600U
/* $K: bipolar, t_d forced to 0, PI at ±0.2 A then ±0.6 A. Axis must be still. */
#define VCM_DT_CAL_I1_A            0.20f
#define VCM_DT_CAL_I2_A            0.60f
#define VCM_DT_CAL_SETTLE          1000U   /* 20 ms @ 50 kHz */
#define VCM_DT_CAL_AVG             2500U   /* valid IFB samples, 50 ms */
#define VCM_DT_CAL_AVG_MAX         5000U   /* abort if samples do not arrive */

#define VCM_MOD_MAX             0.48f

/* 0 = bipolar 0.5+/-m; 1 = three-level low-side freewheel (D=2*|m|) */
#define VCM_PWM_MODE_BIPOLAR       0U
#define VCM_PWM_MODE_TRILEVEL      1U
#define VCM_PWM_MODE               VCM_PWM_MODE_TRILEVEL

/*
 * Three-level, one period: +V pulse, coast, -V pulse, coast.
 * Closed loop keeps both pulses at least VCM_TRI_CM_NS. The current
 * command lengthens one side, so the average voltage is 2*m*Vbus.
 * There is no extra dead-time term on this waveform.
 * After the first coast sample, the −pulse is recomputed with VCM_HALF_TS_S
 * if that edge has not yet started. Feedback for the 50 kHz step is the
 * mean of the two coast samples.
 */

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

/* Single-zone PI @ 50 kHz: fc≈3.9 kHz (zero ~32 Hz).
 * Default: Kp on the observer, Ki on the sampled current so Ke·v is trimmed
 * at steady speed. Host $S fdbk=0 puts both terms on the 2-point IFB average. */
#define VCM_KP                  0.30f
#define VCM_KI                  300.0f
#define VCM_I_INTEGRAL_LIM      0.30f

/* Two-zone PI. Motion uses Kp/Ki. After the current command has been
 * quiet for ENTER, both gains scale by GZ_SCALE. Exit on command slope
 * only — not on current magnitude or current error. The integrator is
 * shifted by (s0-s1)*Kp*e so modulation does not step. */
#define VCM_GZ_EN_DEFAULT          0U
#define VCM_GZ_SCALE_DEFAULT       0.35f
#define VCM_GZ_REF_A               0.0004f   /* command LPF, τ≈50 ms @ 50 kHz */
#define VCM_GZ_ENTER_A_S           8.0f      /* slow-command slope, A/s */
#define VCM_GZ_EXIT_A_S            40.0f
#define VCM_GZ_ENTER_TICKS         2000U     /* 40 ms @ 50 kHz */
#define VCM_GZ_EXIT_TICKS          100U      /* 2 ms @ 50 kHz */

/* PI feedback: 0 = Kp and Ki on ifb_avg. 1 = Kp on ifb_hat, Ki on ifb_avg. */
#define VCM_PI_FDBK_AVG         0U
#define VCM_PI_FDBK_OBS         1U
#define VCM_PI_FDBK_DEFAULT     VCM_PI_FDBK_OBS

/* Plant FF: bipolar Vcoil ≈ 2·m·Vbus. L-FF off while tuning PI.
 * Live/UART override uses the same 1st-order as analog RC (C32/C33 =
 * 49.9 Ω × 220 nF → fc≈14.5 kHz) so Live steps look like analog. */
#define VCM_COIL_L_H            0.001188f
#define VCM_COIL_R_OHM          3.62f
#define VCM_VBUS_V              48.0f
#define VCM_VBUS_MIN_V          24.0f
#define VCM_VBUS_MAX_V          48.0f
/* R-FF off. SCALE=1 cancels coil R, and the PI (tuned with R in the
 * plant) then rings the ~820 Hz mode — audible squeal. ê still removes
 * the Ke·v that was left in the integrator at the end of a move.
 * A Vbus/R change without an explicit ff= keeps the saved ff. */
#define VCM_FF_SCALE            0.0f
#define VCM_FF_MOD_PER_A        0.0f
#define VCM_L_FF_SCALE          0.0f
#define VCM_L_FF_MOD_PER_A      (VCM_L_FF_SCALE * VCM_COIL_L_H / (2.0f * VCM_VBUS_V))
#define VCM_IREF_OVR_LPF_A      0.838f   /* 1-exp(-2*pi*14500/50000); Live-tunable */
#define VCM_L_FF_DIDT_MAX       4000.0f

/*
 * Current observer on the period-average IFB.
 *   î ← î + (Ts/L)(2·m·Vbus − R·î) + α(ifb_avg − î)
 * Default fo≈500 Hz. α is the host milli step 0.061 (displays ~500 Hz).
 * HF still comes from the voltage model. 1-exp(-2*pi*500/50000) = 0.06090.
 */
#define VCM_I_OBS_HZ            500.0f
#define VCM_I_OBS_A             0.0609f  /* host obs=61 @ 50 kHz */
#define VCM_I_OBS_R_OHM         (VCM_COIL_R_OHM + VCM_IFB_RS_OHM)
#define VCM_I_OBS_TS_OVER_L     (VCM_PWM_TS_S / VCM_COIL_L_H)

/* ê estimates Ke·v and is added as ê/(2·Vbus) while |Iref| ≥ I_MIN.
 * Below that gate it is not applied and bleeds off, so idle noise cannot
 * wind a voltage into the coil. Bandwidth ~250 Hz: fast enough to drop
 * when the axis stops, slow enough not to chase the current-sample hash.
 * Innovation LPF 500 Hz: 1-exp(-2*pi*500/50000) = 0.06090.
 * Idle bleed ~40 Hz: 1-exp(-2*pi*40/50000) = 0.005013. */
/* Off: ê chased IFB hash into audible squeal. Re-enable only after
 * pair sampling and idle Ki are quiet on the real axis. */
#define VCM_E_OBS_EN            0U
#define VCM_E_OBS_HZ            250.0f
#define VCM_E_OBS_IN_A          0.06090f
#define VCM_E_OBS_I_MIN         0.050f
#define VCM_E_OBS_BLEED_A       0.005013f

/*
 * 0: keep PWM at standstill (competitor-style; no software dead zone).
 * 1: clamp after |IREF| < ENTER for DEB samples (quiet idle).
 */
#define VCM_IDLE_CLAMP_EN          0U
#define VCM_IDLE_ENTER_A           0.020f
#define VCM_IDLE_EXIT_A            0.035f
#define VCM_IDLE_ENTER_DEB         5000U  /* 100 ms @ 50 kHz */

/* Kp stays closed at every current. Ki ignores |e_i| below the ADC noise
 * floor so a constant-force hold does not walk modulation on IFB hash.
 * Quiet idle is UART 'Z' only. */
/* 0 = off. Was 0.10 A while IFB hash walked Ki; hardware RC made it redundant. */
#define VCM_KI_ERR_DB_A            0.0f
/* Kp feedback EMA @ 50 kHz. 1.0 = off (pass-through). Hardware RC on
 * TPA8001→ADC already kills the HF hash; keep this at 1 so Kp BW returns.
 * Set ≈0.0247 (~200 Hz) only if the analog filter is missing. */
#define VCM_KP_FDBK_LPF_A          1.0f

/* 0: enable goes straight to RUN (no IFB/IREF offset cal, no 2 ms open loop).
 * 1: measure offsets for VCM_IFB_CAL_SAMPLES with m=0 before closing PI. */
#define VCM_ENABLE_CAL_EN          0U
#define VCM_IFB_CAL_SAMPLES        100U   /* ~2 ms @ 50 kHz; used only if ENABLE_CAL */
#define VCM_IREF_OFFSET_MAX_A      0.050f

#define VCM_ADC_CONV_GUARD         720U   /* ~1.5 us: sample + conversion */
#define VCM_ADC_KERNEL_HZ          24000000UL  /* PLL2P 96 MHz / ASYNC_DIV4 */
#define VCM_ADC_SMP_CYCLES         16.5f       /* CubeMX 16.5; S&H at end of this */
#define VCM_ADC_SMP_DELAY          ((uint16_t)(VCM_ADC_SMP_CYCLES * \
                                 ((float)VCM_HRTIM_CLOCK_HZ / (float)VCM_ADC_KERNEL_HZ) + 0.5f)) /* 330 */
/* S&H length at the ADC kernel clock. The window is centered in the zero
 * vector after the active pulse, and the whole window stays >= 1 µs from
 * both the previous edge and the next one. Phase follows the pulse width. */
#define VCM_ADC_EDGE_GUARD_NS      1000U
#define VCM_ADC_EDGE_GUARD_CNT     ((uint16_t)(((uint32_t)VCM_ADC_EDGE_GUARD_NS * \
                                   (VCM_HRTIM_CLOCK_HZ / 1000000UL)) / 1000UL)) /* 480 */
#define VCM_ADC_ZERO_MIN_CNT       ((uint16_t)(VCM_ADC_EDGE_GUARD_CNT + \
                                   VCM_ADC_SMP_DELAY + VCM_ADC_EDGE_GUARD_CNT))
/* Trailing zero must be at least ZERO_MIN. Bipolar later edge = T/2 + |m|·T. */
#define VCM_BIP_MOD_MAX            (((0.5f * (float)VCM_PWM_PERIOD) - \
                                   (float)VCM_ADC_ZERO_MIN_CNT) / (float)VCM_PWM_PERIOD)
/* Tri-level second coast must leave two sample windows. The pulse floor
 * and the zero-region width are applied in VCM_ModMax. */
#define VCM_ADC_N                  2U     /* two IFB conversions per PWM period */
/* Min time between the two ADC triggers so the first conversion finishes. */
#define VCM_ADC_TRIG_GAP           VCM_ADC_CONV_GUARD

/* ADCTRG1 = Timer A CMP2 | CMP4. DMA length 2, MemInc on.
 * Pair validity comes from DMA TC latch (not REP-time NDTR==2).
 * CMP4 must stay ≤ PERIOD-CONV_GUARD so the 2nd conversion ends before REP. */

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
 *   $S kp=300 ki=300000 obs=61 lpf=838 ff=0 vbus=48000 r=3620 l=1188 fdbk=1 dt=400 n0=0 n1=0 n2=0 n3=0 gz=0 gs=350
 *      (kp/obs/lpf/vbus/r ×1000, ki ×1000, ff ×1e6, l = µH,
 *       fdbk 0=Kp and Ki on IFB, 1=Kp on î and Ki on IFB,
 *       dt = two-edge bipolar compensation ns, 0 = off,
 *       n0..n3 = current-command notch Hz, 0 = off,
 *       gz 0=one PI zone 1=standstill scale, gs = standstill scale ×1000)
 *   $R <mA>  IREF override (PI on)   $Z $A $F $B $T $C $G as lines too
 *   $K  measure bipolar gate-driver t_d and store it in RAM
 *   $Q <baud>  switch USART1. Reply BAUD at the old rate, then retune.
 *      115200, 230400, 460800, 921600, 1500000, 2000000, 3000000, 6000000.
 *      6000000 is fck/16 with PLL2Q = 96 MHz and oversampling 16.
 *   $U id=<n> hz=<Hz> mode=0|1 n=<count>   capture iref and ifb into RAM
 *      mode 0 = one shot, mode 1 = continuous.
 *      $U stop id=<n> ends that session only. A stop with no id still stops.
 *      Hz is snapped to an integer divisor of the control rate.
 *      Continuous Hz is capped by the live UART baud (8N1, both currents).
 *   $W  save RAM params to Bank2 Sector3   $L reload   $E factory erase
 *
 * Version in $I / $D: hw=<board> fw=<major>.<minor>.<patch>
 */
#define VCM_HOST_PROTO          3U
#define VCM_HOST_LINE_MAX       192U
#define VCM_HW_VERSION          1U   /* PCB / hardware revision */
#define VCM_FW_VERSION_MAJOR    1U
#define VCM_FW_VERSION_MINOR    4U
#define VCM_FW_VERSION_PATCH    67U

#endif /* VCM_CONFIG_H */
