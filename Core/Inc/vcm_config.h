/**
 * @file    vcm_config.h
 * @brief   Wire-feed current loop - PI + R/L FF, three-level PWM @ 20 kHz
 *
 * 1 A plant, 20 mΩ shunt (same sense as Z; ADC span is ~1/10 of ±10 A).
 * Wide bilateral cm so ADC S&H sits inside each +/-Vbus pulse. Symmetric
 * ripple is above the mechanics. PWM stays enabled (no idle clamp).
 */
#ifndef VCM_CONFIG_H
#define VCM_CONFIG_H

#include <stdint.h>

#define VCM_SYSCLK_HZ           480000000UL
#define VCM_HRTIM_CLOCK_HZ      480000000UL

#define VCM_PWM_FREQ_HZ         20000UL
#define VCM_CTRL_FREQ_HZ        20000UL
#define VCM_PWM_PERIOD          ((uint16_t)(VCM_HRTIM_CLOCK_HZ / VCM_PWM_FREQ_HZ)) /* 24000 */
#define VCM_PWM_REPETITION      ((uint16_t)(VCM_PWM_FREQ_HZ / VCM_CTRL_FREQ_HZ - 1U))
#define VCM_PWM_TS_S            (1.0f / (float)VCM_CTRL_FREQ_HZ)

#define VCM_DT_PRESCALER        HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV8
#define VCM_DT_RISING           0U
#define VCM_DT_FALLING          0U

/* Dead-time volt-second compensation. BIPOLAR ONLY — see VCM_DrvDtTrim().
 * Bipolar: the leg difference is one pulse, so sign(i)·t_d lands directly as
 * volt-second error. A hard sign switch at zero leaves a band where the trim
 * is frozen on a stale sign, so the loop must fight a constant error in the
 * wrong direction — that is the +I / −I asymmetry measured on the bench.
 * Ramping linearly over ±VCM_DRV_DT_BAND_A keeps it continuous through zero.
 * Tri-level must NOT use this value: there the same t_d applies to both the
 * +pulse and the −pulse, so it is common-mode and cancels in the difference.
 * Applying 400 ns differentially would inject ~0.27 A (0.010 m).
 * $K measures t_d; HRTIM dead-time registers stay 0. */
#define VCM_DRV_DT_NS              400U
#define VCM_DRV_DT_NS_MAX          1000U
#define VCM_DRV_DT_BAND_A          0.040f  /* linear sign transition, ±A */
/* 1: after I+ pulse sample, re-run PI and reshape this period's −pulse.
 * 0: only the 20 kHz REP PI; −pulse stays as preloaded at period start. */
#define VCM_HALF_PI_EN             0U
#define VCM_HALF_TS_S              (VCM_PWM_TS_S * 0.5f)
/* Both tri-level pulses stay at least this wide while the current loop
 * is closed, including at zero command. Must fit one S&H window (≈3.7 µs
 * with 1.5 µs edge guards). Default 4 µs → ~210 mA p-p @ 48 V / 0.91 mH;
 * symmetric, above the mechanics. Host $S cm=<ns>. UART Z holds both off. */
#define VCM_TRI_CM_NS              4000U
#define VCM_TRI_CM_NS_MIN          4000U
#define VCM_TRI_CM_NS_MAX          20000U
/* $K: bipolar, t_d forced to 0, PI at ±0.2 A then ±0.6 A. Axis must be still. */
#define VCM_DT_CAL_I1_A            0.20f
#define VCM_DT_CAL_I2_A            0.60f
#define VCM_DT_CAL_SETTLE          400U    /* 20 ms @ 20 kHz */
#define VCM_DT_CAL_AVG             1000U   /* valid IFB samples, 50 ms */
#define VCM_DT_CAL_AVG_MAX         2000U   /* abort if samples do not arrive */

#define VCM_MOD_MAX             0.48f

/* 0 = bipolar 0.5+/-m; 1 = three-level low-side freewheel (D=2*|m|) */
#define VCM_PWM_MODE_BIPOLAR       0U
#define VCM_PWM_MODE_TRILEVEL      1U
#define VCM_PWM_MODE               VCM_PWM_MODE_TRILEVEL

/*
 * Three-level, one period: +V pulse, coast, -V pulse, coast.
 * Closed loop keeps both pulses at least g_vcm.tri_cm_ns. The current
 * command lengthens one side, so the average voltage is 2*m*Vbus.
 * There is no extra dead-time term on this waveform.
 * Feedback for the 20 kHz step is the mean of the two mid-pulse samples
 * (period-average proxy). Half-PI off: −pulse not reshaped mid-period.
 */

#define VCM_I_MAX_A             1.0f
#define VCM_I_OCP_A             1.2f
#define VCM_OCP_CONFIRM_SAMPLES 4U

#define VCM_ADC_MID             32768.0f
#define VCM_ADC_VREF_V          3.0f
#define VCM_ADC_COUNTS_PER_V    (VCM_ADC_MID / VCM_ADC_VREF_V)

/* IFB: RS + TPA8001 → ADC1. 20 mΩ × 8.2 = 0.164 V/A (board shunt). */
#define VCM_IFB_RS_OHM          0.020f
#define VCM_IFB_TPA8001_GAIN    8.2f
#define VCM_IFB_GAIN_TRIM       1.0f
#define VCM_IFB_V_PER_A         (VCM_IFB_GAIN_TRIM * VCM_IFB_TPA8001_GAIN * VCM_IFB_RS_OHM)
#define VCM_IFB_COUNTS_PER_A    (VCM_IFB_V_PER_A * VCM_ADC_COUNTS_PER_V)
#define VCM_IFB_POLARITY        (-1.0f)

/* IREF: ±20 V * op-amp G=0.15 → ±3 V ADC = ±1 A */
#define VCM_IREF_VIN_FULL_V     20.0f
#define VCM_IREF_DIFF_GAIN      0.15f
#define VCM_IREF_GAIN_TRIM      1.0f
#define VCM_IREF_V_PER_A        (VCM_IREF_GAIN_TRIM * VCM_IREF_DIFF_GAIN * \
                                 VCM_IREF_VIN_FULL_V / VCM_I_MAX_A) /* 3.0 V/A */
#define VCM_IREF_COUNTS_PER_A   (VCM_IREF_V_PER_A * VCM_ADC_COUNTS_PER_V)
#define VCM_IREF_POLARITY       (-1.0f)

/* Single-zone PI @ 20 kHz. Default fdbk=0: Kp/Ki on mid-pulse IFB average. */
#define VCM_KP                  0.05f
#define VCM_KI                  0.0f
/* Must sit at or above VCM_ModMax(): at steady state err→0, so m ≈ integral
 * and an integral clamped below the modulator ceiling makes every command
 * above that point unreachable no matter how large Kp is. Tri-level ModMax
 * is 0.5 − cm_duty − coast at the defaults; keep lim above ModMax. */
#define VCM_I_INTEGRAL_LIM      0.40f
/* Back-calc anti-windup gain on the PI(+R-FF) path only. 1 = full unwind;
 * <1 softens recovery. L-FF one-sample spikes are excluded from AW (see PiStep). */
#define VCM_PI_AW_K             0.25f

/* Two-zone PI. Motion uses Kp/Ki. After the current command has been
 * quiet for ENTER, both gains scale by GZ_SCALE. Exit on command slope
 * only — not on current magnitude or current error. The integrator is
 * shifted by (s0-s1)*Kp*e so modulation does not step. */
#define VCM_GZ_EN_DEFAULT          0U
#define VCM_GZ_SCALE_DEFAULT       0.35f
#define VCM_GZ_REF_A               0.001f    /* command LPF, τ≈50 ms @ 20 kHz */
#define VCM_GZ_ENTER_A_S           8.0f      /* slow-command slope, A/s */
#define VCM_GZ_EXIT_A_S            40.0f
#define VCM_GZ_ENTER_TICKS         800U      /* 40 ms @ 20 kHz */
#define VCM_GZ_EXIT_TICKS          40U       /* 2 ms @ 20 kHz */

/* PI feedback: 0 = Kp and Ki on ifb_avg. 1 = Kp and Ki on ifb_hat (î). */
#define VCM_PI_FDBK_AVG         0U
#define VCM_PI_FDBK_OBS         1U
#define VCM_PI_FDBK_DEFAULT     VCM_PI_FDBK_AVG

/* Plant FF: bipolar Vcoil ≈ 2·m·Vbus. L-FF off while tuning PI. */
#define VCM_COIL_L_H            0.000912f /* 912 µH */
#define VCM_COIL_R_OHM          5.42f
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
/* Command (analog IREF + Live/UART) 1st-order LPF @ 20 kHz.
 * Default fo≈3 kHz. 1-exp(-2*pi*3000/20000)≈0.607. Host $S lpf= ; 1.0=off. */
#define VCM_IREF_LPF_HZ         3000.0f
#define VCM_IREF_OVR_LPF_A      0.607f
#define VCM_L_FF_DIDT_MAX       4000.0f

/*
 * Current observer on the period-average IFB.
 *   î ← î + (Ts/L)(2·m·Vbus − R·î) + α(ifb_avg − î)
 * Default fo≈500 Hz. 1-exp(-2*pi*500/20000)≈0.1456.
 */
#define VCM_I_OBS_HZ            500.0f
#define VCM_I_OBS_A             0.1456f  /* host obs=146 @ 20 kHz; DCM keeps this */
/* CCM floor: mild IFB innovation; mid-pulse IFB is period-avg proxy. */
#define VCM_I_OBS_A_CCM         0.15f
#define VCM_I_OBS_R_OHM         (VCM_COIL_R_OHM + VCM_IFB_RS_OHM)
#define VCM_I_OBS_TS_OVER_L     (VCM_PWM_TS_S / VCM_COIL_L_H)

/* ê estimates Ke·v (still off on this branch). Scale α for 20 kHz. */
#define VCM_E_OBS_EN            0U
#define VCM_E_OBS_HZ            120.0f
#define VCM_E_OBS_IN_A          0.1456f
#define VCM_E_OBS_I_MIN         0.050f
#define VCM_E_OBS_BLEED_A       0.0125f

/*
 * 0: keep PWM at standstill (competitor-style; no software dead zone).
 * 1: clamp after |IREF| < ENTER for DEB samples (quiet idle).
 */
#define VCM_IDLE_CLAMP_EN          0U
#define VCM_IDLE_ENTER_A           0.020f
#define VCM_IDLE_EXIT_A            0.035f
#define VCM_IDLE_ENTER_DEB         2000U  /* 100 ms @ 20 kHz */

/*
 * DCM FF when coast IFB is untrustworthy. Tri-level only.
 * Hysteresis: ENTER < EXIT so mid-band cannot flap.
 * Enter when |iref| < ENTER and |ifb| < ENTER_IA.
 * Leave when |iref| > EXIT (iref-only; coast IFB must not gate exit) or FORCE.
 * DCM voltage: m = I·R/(2V)·FF_SCALE + sign(I)·DT_MOD_{P|N} + offset.
 * While dcm=1, ApplyDuty forces cm=0 (no dual min pulses) — PMAC hold was
 * squealing with m≈0 + bilateral cm; CCM keeps cm for sampling.
 * EXIT / EXIT_ERR live in g_vcm (host $S dx= / de=); de= kept for host compat.
 */
#define VCM_DCM_EN                 1U
/* Runtime default for g_vcm.dcm_en ($S den=). 0: PMAC closed-loop stays quiet
 * (CCM+PI). 1: only for open-loop small-I clamp tests. */
#define VCM_DCM_EN_DEFAULT         0U
/* FF through ~100 mA (clamp-matched). Old EXIT 50 mA handed off to PI on
 * low coast IFB → clamp ≈ cmd+25 mA at 50/60/70. */
#define VCM_DCM_ENTER_A            0.10f
#define VCM_DCM_ENTER_IA           0.08f
#define VCM_DCM_EXIT_A             0.12f
#define VCM_DCM_EXIT_ERR_A         0.01f
#define VCM_DCM_EXIT_A_MIN         0.050f
#define VCM_DCM_EXIT_A_MAX         2.000f
#define VCM_DCM_EXIT_ERR_A_MIN     0.010f
#define VCM_DCM_EXIT_ERR_A_MAX     0.500f
#define VCM_DCM_FORCE_A            0.250f
#define VCM_DCM_DEB                20U    /* 1 ms @ 20 kHz */
#define VCM_DCM_FF_SCALE           1.80f
#define VCM_DCM_DT_MOD_P           0.00008f
#define VCM_DCM_DT_MOD_N           0.00003f
#define VCM_DCM_OFFSET_MAX         0.02f
#define VCM_DCM_IREF_ZERO_A        0.002f

/* Was ±25 mA CCM IFB→clamp trim; disabled — PI tracks IFB as-is. */
#define VCM_IFB_CCM_BIAS_A         0.0f
#define VCM_IFB_CCM_BIAS_MIN_A     0.005f

/* Kp stays closed at every current. Ki ignores |e_i| below the ADC noise
 * floor so a constant-force hold does not walk modulation on IFB hash.
 * Quiet idle is UART 'Z' only. */
/* 0 = off. Was 0.10 A while IFB hash walked Ki; hardware RC made it redundant. */
#define VCM_KI_ERR_DB_A            0.0f

/* $O / background IFB zero: reject a capture larger than this. */
#define VCM_IREF_OFFSET_MAX_A      0.050f

/*
 * Background IFB zero-offset estimate.
 * The offset is only observable where the coil current is physically zero.
 * VCM_Stop() stops the HRTIM outputs but not the counters, so the ADC
 * trigger pairs keep arriving with the bridge disarmed. Skip the L/R
 * decay (~0.33 ms) and average raw IFB across off-windows: no dedicated
 * blind time at enable, and the estimate refreshes on every disable, so
 * temperature drift is removed without any closed-loop tracking.
 */
#define VCM_IFB_OFF_WAIT           100U     /* 5 ms @ 20 kHz after the bridge opens */
#define VCM_IFB_OFF_SAMPLES        200U     /* 10 ms of accumulated off-time */
#define VCM_IFB_OFFSET_MAX_A       0.150f   /* reject a bogus capture; real |b| ≤ ~30 mA */

/* $O: one-shot IREF zero capture. The host guarantees the analog IREF
 * input is 0 V for the whole window; nothing is inferred from the reading. */
#define VCM_IREF_ZERO_SAMPLES      200U     /* 10 ms @ 20 kHz */
#define VCM_IREF_ZERO_TIMEOUT_MS   500U

/* A stuck DMA pair leaves the loop on the voltage model only, with stale
 * current for OCP. 200 ticks = 10 ms without a fresh pair is a fault. */
#define VCM_PAIR_MISS_FAULT        200U

#define VCM_ADC_CONV_GUARD         720U   /* ~1.5 us: sample + conversion */
#define VCM_ADC_KERNEL_HZ          24000000UL  /* PLL2P 96 MHz / ASYNC_DIV4 */
#define VCM_ADC_SMP_CYCLES         16.5f       /* CubeMX 16.5; S&H at end of this */
#define VCM_ADC_SMP_DELAY          ((uint16_t)(VCM_ADC_SMP_CYCLES * \
                                 ((float)VCM_HRTIM_CLOCK_HZ / (float)VCM_ADC_KERNEL_HZ) + 0.5f)) /* 330 */
/* S&H window: edge guard + sample + edge guard. Tri-level places it in the
 * middle of each active pulse; bipolar still uses the two zero vectors. */
#define VCM_ADC_EDGE_GUARD_NS      1500U
#define VCM_ADC_EDGE_GUARD_CNT     ((uint16_t)(((uint32_t)VCM_ADC_EDGE_GUARD_NS * \
                                   (VCM_HRTIM_CLOCK_HZ / 1000000UL)) / 1000UL)) /* 720 @ 1.5 µs */
#define VCM_ADC_ZERO_MIN_CNT       ((uint16_t)(VCM_ADC_EDGE_GUARD_CNT + \
                                   VCM_ADC_SMP_DELAY + VCM_ADC_EDGE_GUARD_CNT)) /* ≈1770 ≈3.7 µs */
/* Tri-level coasts no longer host S&H; keep a short freewheel gap only. */
#define VCM_TRI_COAST_MIN_NS       1000U
#define VCM_TRI_COAST_MIN_CNT      ((uint16_t)(((uint32_t)VCM_TRI_COAST_MIN_NS * \
                                   (VCM_HRTIM_CLOCK_HZ / 1000000UL)) / 1000UL)) /* 480 */
/* Minimum zero-vector width for ONE S&H window (bipolar). */
#define VCM_BIP_MOD_MAX            (((0.5f * (float)VCM_PWM_PERIOD) - \
                                   (float)VCM_ADC_ZERO_MIN_CNT) / (float)VCM_PWM_PERIOD)
#define VCM_ADC_N                  2U     /* two IFB conversions per PWM period */
/* Min time between the two ADC triggers so the first conversion finishes. */
#define VCM_ADC_TRIG_GAP           VCM_ADC_CONV_GUARD

/* ADCTRG1 = Timer A CMP2 | CMP4. DMA length 2, MemInc on.
 * Pair validity comes from DMA TC latch (not REP-time NDTR==2).
 * CMP4 must stay ≤ PERIOD-CONV_GUARD so the 2nd conversion ends before REP.
 * Tri-level: one S&H in each active pulse [0,cnt_a) and [start_b,end_b).
 * Mean ≈ period average. Pulse floor = cm ≥ ZERO_MIN. Short DCM/hold
 * pulses fall back to coast windows. Bipolar unchanged (two zero vectors).
 * Pair miss holds last ifb_avg (no mid-edge dma[0]). */

/* Gain-cal: UART force current (clamp meter) and rolling mean window */
#define VCM_CAL_FORCE_A            0.50f
#define VCM_CAL_AVG_ALPHA          (1.0f / 1020.0f) /* ~51 ms EMA @ 20 kHz */

#define VCM_DRV_EN_Pin             GPIO_PIN_0
#define VCM_DRV_EN_GPIO_Port       GPIOB
#define VCM_DRV_EN_ACTIVE_LEVEL    GPIO_PIN_RESET
#define VCM_FAULT_Pin              GPIO_PIN_1
#define VCM_FAULT_GPIO_Port        GPIOB
#define VCM_FAULT_ACTIVE_LEVEL     GPIO_PIN_SET
#define VCM_FAULT_INACTIVE_LEVEL   GPIO_PIN_RESET
#define VCM_PROCESS_LED_Pin        GPIO_PIN_3
#define VCM_PROCESS_LED_GPIO_Port  GPIOE

#define VCM_EN_GLITCH_SAMPLES   8U   /* ~400 us @ 20 kHz */
#define VCM_EN_OFF_DEBOUNCE_MS  2U

#define VCM_DIAG_EN             1
#define VCM_DIAG_STEP_A         0.20f
#define VCM_DIAG_DONE_BAND      0.10f
#define VCM_DIAG_TIMEOUT_TICKS  400U   /* 20 ms @ 20 kHz */
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
 *   $S kp=50 ki=0 obs=146 lpf=607 ff=0 vbus=48000 r=5420 l=912 fdbk=0 dt=400 cm=4000 den=0 dx=120 de=10 n0=0 n1=0 n2=0 n3=0 gz=0 gs=350
 *      (kp/obs/lpf/vbus/r ×1000, ki ×1000, ff ×1e6, l = µH,
 *       lpf = command IREF LPF α, 0.607≈3 kHz @ 20 kHz, 1.0 = off,
 *       fdbk 0=Kp and Ki on IFB, 1=Kp and Ki on î,
 *       dt = two-edge bipolar compensation ns, 0 = off,
 *       cm = tri-level min pulse ns (both sides), 4000..20000, default 4000,
 *       den 0=DCM off (CCM PI) 1=allow DCM FF; dx = exit |iref| mA; de= compat,
 *       n0..n3 = current-command notch Hz, 0 = off,
 *       gz 0=one PI zone 1=standstill scale, gs = standstill scale ×1000)
 *   $R ma=<mA>;s=<seq>  IREF override (PI on). ';' ends the mA field so a
 *      glued trailing digit cannot turn 10 into 100. s= is echoed in OK.
 *   $Z $A $F $B $T $C $G as lines too
 *   $K  measure bipolar gate-driver t_d and store it in RAM
 *   $O  one-shot IREF zero capture. The host must hold the analog IREF
 *      input at 0 V for the whole window (10 ms). Runs in the ISR only
 *      while a fresh ADC pair arrives, so it also serves as a link probe.
 *      Reply "IREFZ <mA>", or IREFZ ERR override|range|nosample.
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
#define VCM_HOST_PROTO          5U
#define VCM_HOST_LINE_MAX       192U
#define VCM_HW_VERSION          1U   /* PCB / hardware revision */
#define VCM_FW_VERSION_MAJOR    1U
#define VCM_FW_VERSION_MINOR    4U
#define VCM_FW_VERSION_PATCH    201U /* wire-feed 1 A / 20 mΩ IFB @ 20 kHz */

#endif /* VCM_CONFIG_H */