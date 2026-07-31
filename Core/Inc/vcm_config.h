/**
 * @file    vcm_config.h
 * @brief   Single-axis VCM PWM current-loop parameters (STM32H743 + HRTIM)
 *
 * Flash: app linked at 0x08020000 for IAP_UartSTM32H7 (see iap_app.h).
 */
#ifndef VCM_CONFIG_H
#define VCM_CONFIG_H

#include <stdint.h>

/* Clock: HSI 64 MHz → PLL1P 480 MHz SYSCLK, HCLK 240 MHz
 * HRTIM1CLK = CPUCLK = 480 MHz (RCC_HRTIM1CLK_CPUCLK, H7 max; no DLL/MUL32) */
#define VCM_SYSCLK_HZ           480000000UL
#define VCM_HRTIM_CLOCK_HZ      480000000UL
/* 20 kHz PWM (period 24000 @ 480 MHz HRTIM; audible but better small-I window) */
#define VCM_PWM_FREQ_HZ         20000UL
/* H7 HRTIM only supports DIV1/2/4 (no MUL32 like G4) */
#define VCM_PWM_PERIOD          ((uint16_t)(VCM_HRTIM_CLOCK_HZ / VCM_PWM_FREQ_HZ)) /* 24000 */
#define VCM_PWM_TS_S            (1.0f / (float)VCM_PWM_FREQ_HZ)

/* Dead-time: fDTG = fHRTIM/8 = 60 MHz.
 * EG2132 already inserts ~150–350 ns; MCU DT = 0 (chip provides the gap). */
#define VCM_DT_PRESCALER        HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV8
#define VCM_DT_RISING           0U
#define VCM_DT_FALLING          0U

/* Software dead-time compensation: m_pwm = m_pi + sign(mod)*m_dt
 * m_dt ≈ 2*td/Tsw with td≈250 ns @ 20 kHz → ~0.01 */
#define VCM_MOD_DT_COMP         0.01f
#define VCM_DT_COMP_HYST_A      0.008f

#define VCM_MOD_MAX             0.45f
#define VCM_DUTY_MIN_COUNTS     ((uint16_t)(VCM_PWM_PERIOD * 0.05f))
#define VCM_DUTY_MAX_COUNTS     ((uint16_t)(VCM_PWM_PERIOD * 0.95f))

#define VCM_I_MAX_A             1.0f
#define VCM_I_OCP_A             1.2f
#define VCM_ADC_MID             32768.0f
#define VCM_ADC_COUNTS_PER_A    32768.0f

#define VCM_KP                  0.02f
#define VCM_KI                  200.0f
#define VCM_I_INTEGRAL_LIM      VCM_MOD_MAX

/* Debug: 1 = ignore ADC command, use g_vcm.iref_override_a (default 0 A) */
#ifndef VCM_IREF_OVERRIDE_DEFAULT
#define VCM_IREF_OVERRIDE_DEFAULT  1
#endif
#define VCM_IREF_OVERRIDE_A_DEFAULT  0.0f

#ifndef VCM_FORCE_ZERO_MOD
#define VCM_FORCE_ZERO_MOD         0
#endif

/* Debug open-loop: 1 = start with mod_override_en (Live Watch can change anytime) */
#ifndef VCM_MOD_OVERRIDE_DEFAULT
#define VCM_MOD_OVERRIDE_DEFAULT   0
#endif
#define VCM_MOD_OVERRIDE_A_DEFAULT 0.0f

#define VCM_IFB_CAL_SAMPLES        512U

#define VCM_IREF_I_HOLD_A          0.005f
#define VCM_I_LEAK_PER_PERIOD      0.05f

/* Real-time slew ≈ 1000 /s (0.05 per period @ 20 kHz) */
#define VCM_MOD_SLEW_PER_PERIOD    0.05f
#define VCM_IREF_NEAR_ZERO_A       0.02f
#define VCM_IFB_UNEXPECTED_A       0.35f

/* Time-based (HRTIM ticks @ 480 MHz) */
#define VCM_ADC_TRIG_EDGE_MARGIN   240U
#define VCM_ADC_ACTIVE_MIN_COUNTS  720U

/* GPIO control (LQFP144) — match main.h Cube labels
 * DRV_EN (PB0): external enable input (active low). Wire to host EN / driver EN. */
#define VCM_DRV_EN_Pin          GPIO_PIN_0
#define VCM_DRV_EN_GPIO_Port    GPIOB
#define VCM_DRV_EN_ACTIVE_LEVEL GPIO_PIN_SET
#define VCM_FAULT_Pin           GPIO_PIN_1
#define VCM_FAULT_GPIO_Port     GPIOB
#define VCM_FAULT_ACTIVE_LEVEL  GPIO_PIN_SET
#define VCM_FAULT_INACTIVE_LEVEL GPIO_PIN_RESET

#define VCM_PROCESS_LED_Pin       GPIO_PIN_3
#define VCM_PROCESS_LED_GPIO_Port GPIOE

/* ISR scope: 512 samples @ 20 kHz ≈ 25.6 ms */
#ifndef VCM_SCOPE_ENABLE
#define VCM_SCOPE_ENABLE          1
#endif
#define VCM_SCOPE_LEN             512U

#endif /* VCM_CONFIG_H */
