/**

 * @file    vcm_config.h

 * @brief   VCM PWM current-loop parameters (STM32H743 + HRTIM)

 *

 * Flash: app linked at 0x08020000 for IAP_UartSTM32H7 (see iap_app.h).
 *
 * Operating envelope (this board/firmware):
 *   Bus VS = 12 V, command/loop full-scale I_MAX = 1.0 A (OCP 1.2 A).
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

#define VCM_PWM_FREQ_HZ         100000UL

#define VCM_PWM_PERIOD          ((uint16_t)(VCM_HRTIM_CLOCK_HZ / VCM_PWM_FREQ_HZ)) /* 4800 */

#define VCM_PWM_TS_S            (1.0f / (float)VCM_PWM_FREQ_HZ)



/* Dead-time: EG2132 provides ~150–350 ns; MCU DT = 0 */

#define VCM_DT_PRESCALER        HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV8

#define VCM_DT_RISING           0U

#define VCM_DT_FALLING          0U

#define VCM_MOD_DT_COMP         0.01f

#define VCM_DT_COMP_HYST_A      0.008f



#define VCM_MOD_MAX             0.45f

#define VCM_DUTY_MIN_COUNTS     ((uint16_t)(VCM_PWM_PERIOD * 0.05f))

#define VCM_DUTY_MAX_COUNTS     ((uint16_t)(VCM_PWM_PERIOD * 0.95f))



#define VCM_I_MAX_A             1.0f

#define VCM_I_OCP_A             1.2f



/* 16-bit differential ADC, mid-scale = 0 Vdiff; VREF+ = 3.0 V */

#define VCM_ADC_MID             32768.0f

#define VCM_ADC_VREF_V          3.0f

#define VCM_ADC_COUNTS_PER_V    (VCM_ADC_MID / VCM_ADC_VREF_V)



/*

 * IFB: RS=200 mΩ, TPA8001 G=8.2 (±250 mV in → diff out)

 *   Vshunt/A = 0.2 V, Vadc_diff/A = 8.2*0.2 = 1.64 V

 *   At 1 A: headroom to ±250 mV in / ±(8.2*0.25) out ≈ OK for I_OCP

 */

#define VCM_IFB_RS_OHM          0.200f

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

#define VCM_IREF_POLARITY       (1.0f)



/* 100 kHz loop: Kp sets rise; with R-drop FF, Ki only trims model error. */
#define VCM_KP                  1.0f
#define VCM_KI                  2000.0f
#define VCM_I_INTEGRAL_LIM      VCM_MOD_MAX

/* Plant FF: L≈915 µH, Rcoil≈5.4 Ω; bus 12 V; bipolar Vcoil ≈ 2*m*Vbus.
 * Rated use: max 1 A @ 12 V input (see VCM_I_MAX_A / VCM_VBUS_V).
 * SCALE<1 derates FF; ALPHA LPF softens m_ff on iref steps. */
#define VCM_COIL_R_OHM          5.4f
#define VCM_VBUS_V              12.0f
#define VCM_FF_ENABLE           1
#define VCM_FF_SCALE            0.70f
#define VCM_FF_ALPHA            0.05f
#define VCM_FF_MOD_PER_A        (VCM_FF_SCALE * (VCM_COIL_R_OHM + VCM_IFB_RS_OHM) / (2.0f * VCM_VBUS_V))

/* Series shunt: light edge-glitch reject into PI (not freewheel-gated) */
#define VCM_IFB_SPIKE_A         0.50f
#define VCM_IFB_PI_ALPHA        0.45f



/* 0 = use ADC2 command; 1 = iref_override_a (debug) */

#define VCM_IREF_OVERRIDE_DEFAULT    0

#define VCM_IREF_OVERRIDE_A_DEFAULT  0.0f

#define VCM_MOD_OVERRIDE_DEFAULT     0

#define VCM_MOD_OVERRIDE_A_DEFAULT   0.0f



#define VCM_IFB_CAL_SAMPLES        512U

#define VCM_IREF_I_HOLD_A          0.005f

#define VCM_I_LEAK_PER_PERIOD      0.05f

/* ~same real-time slew as 0.05 @ 20 kHz after fsw×5 */
#define VCM_MOD_SLEW_PER_PERIOD    0.01f

#define VCM_IREF_NEAR_ZERO_A       0.02f

#define VCM_IFB_UNEXPECTED_A       0.35f



/* Keep ADC trigger away from PWM edges (series sense still rings at switch) */

/* ~5% of PERIOD@100 kHz; keep clear of PWM edges */
#define VCM_ADC_TRIG_EDGE_MARGIN   240U



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


