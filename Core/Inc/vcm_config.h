/**

 * @file    vcm_config.h

 * @brief   VCM PWM current-loop parameters (STM32H743 + HRTIM)

 *

 * Flash: app linked at 0x08020000 for IAP_UartSTM32H7 (see iap_app.h).
 *
 * Operating envelope (this board/firmware):
 *   Bus VS = 48 V, command/loop full-scale I_MAX = 10 A (OCP 12 A).
 *   Z-VCM: L≈1188 µH, R≈3.62 Ω; series shunt RS=20 mΩ.
 *
 * Sense (rev):
 *   IFB  — coil series RS + TPA8001-SOAR → ADC1 differential
 *   IREF — ±10 V cmd via TPA2672 (G=0.15) → ADC2 differential
 */

#ifndef VCM_CONFIG_H

#define VCM_CONFIG_H



#include <stdint.h>



/* Clock: HSI 64 MHz → PLL1P 480 MHz SYSCLK; HRTIM1CLK = CPUCLK = 480 MHz */

#define VCM_SYSCLK_HZ           480000000UL

#define VCM_HRTIM_CLOCK_HZ      480000000UL

/* PWM 50 kHz (cut MOS switching loss vs 200 kHz); current loop same rate. */
#define VCM_PWM_FREQ_HZ         50000UL
#define VCM_CTRL_FREQ_HZ        50000UL
#define VCM_PWM_PERIOD          ((uint16_t)(VCM_HRTIM_CLOCK_HZ / VCM_PWM_FREQ_HZ)) /* 9600 */
/* REP = f_pwm/f_ctrl − 1 → 0 = IRQ every PWM period */
#define VCM_PWM_REPETITION      ((uint16_t)(VCM_PWM_FREQ_HZ / VCM_CTRL_FREQ_HZ - 1U))
#define VCM_PWM_TS_S            (1.0f / (float)VCM_CTRL_FREQ_HZ)



/* Dead-time: EG2132 provides ~150–350 ns; MCU DT = 0 */

#define VCM_DT_PRESCALER        HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV8

#define VCM_DT_RISING           0U

#define VCM_DT_FALLING          0U

#define VCM_MOD_DT_COMP         0.01f

#define VCM_DT_COMP_HYST_A      0.08f



#define VCM_MOD_MAX             0.45f

#define VCM_DUTY_MIN_COUNTS     ((uint16_t)(VCM_PWM_PERIOD * 0.05f))

#define VCM_DUTY_MAX_COUNTS     ((uint16_t)(VCM_PWM_PERIOD * 0.95f))



#define VCM_I_MAX_A             10.0f

#define VCM_I_OCP_A             12.0f

/* Raw IFB spike reject: 4 samples @ 50 kHz control ≈ 80 µs */
#define VCM_OCP_CONFIRM_SAMPLES 4U



/* 16-bit differential ADC, mid-scale = 0 Vdiff; VREF+ = 3.0 V */

#define VCM_ADC_MID             32768.0f

#define VCM_ADC_VREF_V          3.0f

#define VCM_ADC_COUNTS_PER_V    (VCM_ADC_MID / VCM_ADC_VREF_V)



/*

 * IFB: RS=20 mΩ, TPA8001 G=8.2 (±250 mV in → diff out)

 *   Vshunt/A = 0.02 V, Vadc_diff/A = 8.2*0.02 = 0.164 V

 *   At 10 A: Vshunt=0.20 V; at I_OCP 12 A: 0.24 V (within ±250 mV in)

 */

#define VCM_IFB_RS_OHM          0.020f

#define VCM_IFB_TPA8001_GAIN    8.2f

#define VCM_IFB_V_PER_A         (VCM_IFB_TPA8001_GAIN * VCM_IFB_RS_OHM)

#define VCM_IFB_COUNTS_PER_A    (VCM_IFB_V_PER_A * VCM_ADC_COUNTS_PER_V)

/* Flip to -1.0f if closed-loop sign is inverted vs coil current */

#define VCM_IFB_POLARITY        (-1.0f)



/*

 * IREF: host ±10 V → ±I_MAX via TPA2672 → ADC2 diff.

 * Stage A G=0.15 + stage B invert about Vocm → Vdiff = 0.30 * Vin

 * (±10 V → ±3.0 V at VREF=3 V = full-scale)

 */

#define VCM_IREF_VIN_FULL_V     10.0f

#define VCM_IREF_FRONT_GAIN     0.30f

#define VCM_IREF_V_PER_A        (VCM_IREF_FRONT_GAIN * VCM_IREF_VIN_FULL_V / VCM_I_MAX_A)

#define VCM_IREF_COUNTS_PER_A   (VCM_IREF_V_PER_A * VCM_ADC_COUNTS_PER_V)

/* +1: host +cmd → +iref. -1: reverse without swapping motor leads.
 * Do NOT flip only PWM/m or only IFB — that breaks closed-loop sign. */
#define VCM_IREF_POLARITY       (-1.0f)



/* 50 kHz PWM/loop @ 48 V: m → Vcoil ≈ 2*m*Vbus. */
#define VCM_KP                  0.08f
#define VCM_KI                  400.0f
#define VCM_I_INTEGRAL_LIM      VCM_MOD_MAX

/* Plant FF defaults (copied into g_vcm at VCM_Init — Live Expressions can change).
 * Symmetric L·di/dt: same SCALE/ALPHA/lim for rise and fall. */
#define VCM_COIL_R_OHM          3.62f
#define VCM_COIL_L_H            0.001188f
#define VCM_VBUS_V              48.0f
#define VCM_FF_ENABLE           1
#define VCM_FF_SCALE            0.70f
#define VCM_FF_ALPHA            0.25f
#define VCM_FF_MOD_PER_A        (VCM_FF_SCALE * (VCM_COIL_R_OHM + VCM_IFB_RS_OHM) / (2.0f * VCM_VBUS_V))
#define VCM_FF_L_ENABLE         1
#define VCM_FF_L_SCALE          1.00f
/* Post-filter on m_ff_l (1.0 = no lag). Do NOT prefilter iref before d/dt —
 * that caused large phase lag at 3 kHz when scale was raised. */
#define VCM_FF_L_IREF_ALPHA     1.00f
/* lim 0.15 → |V|≈14 V → 0.5 A theoretical ~40 µs; LPF/α → ~80–120 µs both ways. */
#define VCM_FF_L_MOD_MAX        0.15f

/* Series shunt: edge-glitch reject into PI.
 * ALPHA at 50 kHz: 0.24 → τ≈83 µs (same as 0.12 @ 100 kHz). */
#define VCM_IFB_SPIKE_A         5.0f
#define VCM_IFB_PI_ALPHA        0.24f

/* Hold PI when |iref-ifb| is within ADC/ripple (steady-state hiss).
 * Large steps (|err|>>deadband) are unchanged. */
#define VCM_I_ERR_DEADBAND_A    0.04f

/* Same mod LPF on rise and fall. */
#define VCM_MOD_LPF_ALPHA       0.40f

/* Only used if L-FF finished and a trickle of current remains. */
#define VCM_ZERO_BRAKE_KP_SCALE 0.25f



/* 0 = use ADC2 command; 1 = iref_override_a (debug) */

#define VCM_IREF_OVERRIDE_DEFAULT    0

#define VCM_IREF_OVERRIDE_A_DEFAULT  0.0f

#define VCM_MOD_OVERRIDE_DEFAULT     0

#define VCM_MOD_OVERRIDE_A_DEFAULT   0.0f



#define VCM_IFB_CAL_SAMPLES        512U

#define VCM_IREF_I_HOLD_A          0.05f

/* Slow leak only while |iref| is small but not in hard zero-hold. */
#define VCM_I_LEAK_PER_PERIOD      0.05f

/* Same slew limit both directions (was rise=MOD_MAX, fall=0.05 → asymmetric). */
#define VCM_MOD_SLEW_PER_PERIOD    0.08f

/* |iref| below this: brake residual current to 0, then snap integral/FF. */
#define VCM_IREF_NEAR_ZERO_A       0.05f

/* Legacy: huge |ifb| with zero command (sense fault). Still clears integral. */
#define VCM_IFB_UNEXPECTED_A       3.5f



/* Keep ADC trigger away from PWM edges @ PERIOD=9600 (~1 µs) */
#define VCM_ADC_TRIG_EDGE_MARGIN   480U



/* DRV_EN active-low (board pull-up) */

#define VCM_DRV_EN_Pin             GPIO_PIN_0

#define VCM_DRV_EN_GPIO_Port       GPIOB

#define VCM_DRV_EN_ACTIVE_LEVEL    GPIO_PIN_RESET

#define VCM_FAULT_Pin              GPIO_PIN_1

#define VCM_FAULT_GPIO_Port        GPIOB

#define VCM_FAULT_ACTIVE_LEVEL     GPIO_PIN_SET

#define VCM_FAULT_INACTIVE_LEVEL   GPIO_PIN_RESET

#define VCM_PROCESS_LED_Pin        GPIO_PIN_3

#define VCM_PROCESS_LED_GPIO_Port  GPIOE



#endif /* VCM_CONFIG_H */


