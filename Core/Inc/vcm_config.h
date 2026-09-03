/**
 * @file    vcm_config.h
 * @brief   VCM current loop - PI + R/L FF, tri-level or bipolar PWM
 *
 * Default: three-level (active leg chops, idle leg low-side ON).
 * Set VCM_PWM_MODE to BIPOLAR for A/B compare. No command LPF.
 */
#ifndef VCM_CONFIG_H
#define VCM_CONFIG_H

#include <stdint.h>

#define VCM_SYSCLK_HZ           480000000UL
#define VCM_HRTIM_CLOCK_HZ      480000000UL

#define VCM_PWM_FREQ_HZ         100000UL
#define VCM_CTRL_FREQ_HZ        100000UL
#define VCM_PWM_PERIOD          ((uint16_t)(VCM_HRTIM_CLOCK_HZ / VCM_PWM_FREQ_HZ)) /* 4800 */
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

/* Sign hysteresis for tri-level leg swap (modulation units) */
#define VCM_TRI_SIGN_ON_M          0.004f
#define VCM_TRI_SIGN_OFF_M         0.002f
/* Below this |m|, stay bipolar so AC zero-cross stays continuous */
#define VCM_TRI_BIPOLAR_M          0.003f

/*
 * IREF map: host ±20 V -> op-amp G=0.15 -> ±3 V at ADC -> ±1 A.
 */
#define VCM_I_MAX_A             1.0f
#define VCM_I_CMD_MAX_A         1.0f
#define VCM_I_OCP_A             1.2f
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

/* IREF: ±20 V * 0.15 → ±3 V ADC = ±1 A */
#define VCM_IREF_VIN_FULL_V     20.0f
#define VCM_IREF_DIFF_GAIN      0.15f   /* front-end op-amp gain */
#define VCM_IREF_GAIN_TRIM      1.0f
#define VCM_IREF_V_PER_A        (VCM_IREF_GAIN_TRIM * VCM_IREF_DIFF_GAIN * \
                                 VCM_IREF_VIN_FULL_V / VCM_I_MAX_A) /* 3.0 V/A */
#define VCM_IREF_COUNTS_PER_A   (VCM_IREF_V_PER_A * VCM_ADC_COUNTS_PER_V)
#define VCM_IREF_POLARITY       (-1.0f)

/* PI trim around plant FF */
#define VCM_KP                  0.30f
#define VCM_KI                  600.0f
#define VCM_I_INTEGRAL_LIM      0.30f

/* Plant FF: bipolar Vcoil ≈ 2·m·Vbus. No LPF on FF. */
#define VCM_COIL_L_H            0.000912f /* 912 µH */
#define VCM_COIL_R_OHM          5.42f
#define VCM_VBUS_V              48.0f
#define VCM_FF_SCALE            1.0f
#define VCM_L_FF_SCALE          0.85f
#define VCM_FF_MOD_PER_A        (VCM_FF_SCALE * (VCM_COIL_R_OHM + VCM_IFB_RS_OHM) / \
                                 (2.0f * VCM_VBUS_V))
#define VCM_L_FF_MOD_PER_A      (VCM_L_FF_SCALE * VCM_COIL_L_H / (2.0f * VCM_VBUS_V))

/* True standstill only — do not trip on 200 Hz zero-cross */
#define VCM_IDLE_ENTER_A           0.020f
#define VCM_IDLE_EXIT_A            0.035f
#define VCM_IDLE_ENTER_DEB         10000U /* 100 ms @ 100 kHz */

#define VCM_IFB_CAL_SAMPLES        200U   /* ~2 ms @ 100 kHz */
#define VCM_IREF_OFFSET_MAX_A      0.050f

#define VCM_ADC_TRIG_EDGE_MARGIN   480U

/* Gain-cal: UART force current (clamp meter) and rolling mean window */
#define VCM_CAL_FORCE_A            0.50f
#define VCM_CAL_AVG_ALPHA          (1.0f / 5120.0f) /* ~51 ms EMA @ 100 kHz */

#define VCM_DRV_EN_Pin             GPIO_PIN_0
#define VCM_DRV_EN_GPIO_Port       GPIOB
#define VCM_DRV_EN_ACTIVE_LEVEL    GPIO_PIN_RESET
#define VCM_FAULT_Pin              GPIO_PIN_1
#define VCM_FAULT_GPIO_Port        GPIOB
#define VCM_FAULT_ACTIVE_LEVEL     GPIO_PIN_SET
#define VCM_FAULT_INACTIVE_LEVEL   GPIO_PIN_RESET
#define VCM_PROCESS_LED_Pin        GPIO_PIN_3
#define VCM_PROCESS_LED_GPIO_Port  GPIOE

#define VCM_EN_GLITCH_SAMPLES   8U
#define VCM_EN_OFF_DEBOUNCE_MS  2U
#define VCM_CAL_REUSE_MS        5000U

#define VCM_DIAG_EN             1
#define VCM_DIAG_STEP_A         0.20f
#define VCM_DIAG_DONE_BAND      0.10f
#define VCM_DIAG_TIMEOUT_TICKS  2000U  /* 20 ms @ 100 kHz */
#define VCM_DIAG_UART_MARK      0

/* UART cal cmds (ASCII; avoid 0x05 / 0xA0 used by IAP) */
#define VCM_UART_CMD_GAIN       ((uint8_t)'G')  /* print iref/ifb means */
#define VCM_UART_CMD_FORCE      ((uint8_t)'F')  /* override IREF = CAL_FORCE_A */
#define VCM_UART_CMD_ZERO       ((uint8_t)'Z')  /* override IREF = 0 */
#define VCM_UART_CMD_ANALOG     ((uint8_t)'A')  /* clear override, use ADC */
#define VCM_UART_CMD_BIPOLAR    ((uint8_t)'B')  /* pwm_mode = bipolar */
#define VCM_UART_CMD_TRILEVEL   ((uint8_t)'T')  /* pwm_mode = three-level */

#endif /* VCM_CONFIG_H */
