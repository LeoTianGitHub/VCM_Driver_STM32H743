/**
 * @file    vcm_ctrl.h
 * @brief   VCM current-loop API - observer + PI + R/L FF, tri-level PWM
 */
#ifndef VCM_CTRL_H
#define VCM_CTRL_H

#include "main.h"
#include "vcm_config.h"
#include <stdbool.h>

/* Place current-loop hot path in ITCMRAM (copied from Flash at reset). */
#if defined(__GNUC__)
#define VCM_ITCM __attribute__((section(".itcm_text")))
#else
#define VCM_ITCM
#endif

typedef enum
{
  VCM_STATE_INIT = 0,
  VCM_STATE_IDLE = 1,
  /* Former on-enable CALIB; numbering frozen for $P / host StateName. */
  VCM_STATE_RESERVED_2 = 2,
  VCM_STATE_RUN = 3,
  VCM_STATE_FAULT = 4
} VCM_State_t;

typedef struct
{
  float iref_a;
  float iref_offset_a;
  float ifb_a;
  float ifb_avg_a;     /* 2-pt duration-weighted mean (scope / OCP / UART G) */
  float ifb_hat_a;     /* observer current → PI */
  float ifb_filt_a;    /* Live: alias of ifb_hat_a */
  float ifb_ip_a;      /* tri: after +pulse. bipolar: leading zero (ripple min) */
  float ifb_im_a;      /* tri: after −pulse. bipolar: trailing zero (ripple max) */
  float ifb_raw_a;
  float ifb_offset_a;
  float mod;
  float mod_ff;
  float integral;
  float kp;
  float ki;
  float ff_mod_per_a;
  float iref_lpf_a;    /* command LPF α; 0.314 ≈ 3 kHz @ 50 kHz; 1.0 = off */
  float i_obs_a;       /* observer α; 0.0609 ≈ 500 Hz @ 50 kHz */
  float vbus_v;        /* plant Vbus; observer + R-FF */
  float coil_l_h;      /* plant L */
  float coil_r_ohm;    /* plant R (coil; Rs added in observer/FF) */
  float iref_override_a;
  float cal_iref_mean;      /* EMA for UART 'G' */
  float cal_ifb_mean;
  float ifb_off_est_a;      /* last background IFB zero-offset estimate */
  uint32_t pair_miss_cnt;   /* total missed DMA pairs (diagnostic) */
  uint32_t fault_flags;
  uint32_t adc1_ovr_cnt;
  uint32_t adc2_ovr_cnt;
  uint16_t adc_ifb;
  uint16_t adc_iref;
  VCM_State_t state;
  uint8_t adc_active_valid;
  uint8_t adc_pair_ok;        /* 1 = fresh DMA-TC pair (I+ and I−) consumed */
  uint8_t pwm_armed;
  uint8_t iref_override_en;
  uint8_t iref_hold_en;       /* 1 = UART 'Z': PI off, equal dither */
  uint8_t pi_fdbk;            /* VCM_PI_FDBK_*: 0=ifb_avg 1=î for Kp and Ki */
  uint8_t pwm_mode;           /* VCM_PWM_MODE_*; Live Expr / UART B|T */
  uint16_t drv_dt_ns;         /* dead-time compensation width, ns; 0 = off */
  uint16_t tri_cm_ns;         /* tri-level min pulse both sides, ns */
  uint16_t notch_hz[4];       /* current-command notches, Hz; 0 = off */
  float gz_scale;             /* standstill Kp/Ki scale, 0.05..1 */
  float dcm_exit_a;           /* DCM exit |iref|; $S dx= */
  float dcm_exit_err_a;       /* DCM exit |iref−ifb|; $S de= (compat) */
  uint8_t gz_en;              /* 0 = one zone, 1 = lower gain when command is quiet */
  uint8_t gz_hold;            /* 1 = standstill scale is applied this period */
  uint8_t dcm_en;             /* 0 = force CCM PI; 1 = allow DCM FF; $S den= */
  uint8_t dcm;                /* 1 = DCM FF active (tri-level small-I) */
  uint32_t isr_ticks;
  uint32_t start_count;
  uint32_t stop_count;
  uint32_t exti_count;
  uint32_t fault_count;
  uint32_t en_glitch_count;
  uint32_t diag_ticks;
  uint32_t diag_timeouts;
  float diag_iref_at_step;
  float diag_ifb_at_step;
  float diag_ifb_final;
} VCM_Handle_t;

extern VCM_Handle_t g_vcm;
extern HRTIM_HandleTypeDef hhrtim;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern UART_HandleTypeDef huart1;

void VCM_Init(void);
void VCM_PlantSyncFf(void);
/** Rebuild command-notch biquad coeffs (main loop / $S / NV — not from the ISR). */
void VCM_NotchSync(void);
void VCM_HRTIM_InitTimers(void);
void VCM_Start(void);
void VCM_Stop(void);
void VCM_ClearFault(void);
void VCM_ServiceFaultClear(void);
int VCM_AdcStart(void);
void VCM_CurrentLoop_IRQHandler(void);
/* Timer A CMP3: second PI step onto the tri-level −pulse. */
void VCM_CurrentLoop_HalfIRQ(void);
void VCM_EnterFault(uint32_t flag);
void VCM_ServiceDrvEnable(void);
bool VCM_IsDrvEnActive(void);
float VCM_AdcToIfbAmpere(uint16_t raw);
float VCM_AdcToIrefAmpere(uint16_t raw);

/** Handle ASCII cal cmds: G/F/Z/A. Returns 1 if consumed. */
uint8_t VCM_ServiceUartCmd(uint8_t cmd);
/** Byte from USART1: legacy G/F/Z/A/B/T or "$...\\n" host frames. */
void VCM_ServiceUartByte(uint8_t b);
/** Pop one USART1 byte. Also keeps bytes received while a capture line is transmitting. */
uint8_t VCM_UartRxPop(uint8_t *out);
void VCM_UartRxISR(void);
void VCM_UartRxArm(void);
void VCM_PollVersionQuery(void);
void VCM_BootAnnounce(void);
/** Run deferred $W/$E Flash erase/program from main loop (not from the UART parser). */
void VCM_NvPoll(void);

#endif /* VCM_CTRL_H */
