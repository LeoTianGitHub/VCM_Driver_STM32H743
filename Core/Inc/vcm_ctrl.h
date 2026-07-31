/**
 * @file    vcm_ctrl.h
 * @brief   VCM current-loop API (HRTIM H-bridge)
 */
#ifndef VCM_CTRL_H
#define VCM_CTRL_H

#include "main.h"
#include "vcm_config.h"
#include <stdbool.h>

typedef enum
{
  VCM_STATE_INIT = 0,
  VCM_STATE_IDLE,
  VCM_STATE_CALIB, /* averaging ifb offset at 50% duty */
  VCM_STATE_RUN,
  VCM_STATE_FAULT
} VCM_State_t;

typedef struct
{
  float iref_a;
  float ifb_a;       /* offset-corrected instantaneous (scope) */
  float ifb_pi_a;    /* 2-period average used by PI only */
  float ifb_raw_a;   /* raw ADC current before offset subtract */
  float ifb_offset_a;
  float mod;
  float integral;
  float kp;
  float ki;
  uint16_t adc_ifb;
  uint16_t adc_iref;
  uint8_t adc_active_valid; /* 1 = last ApplyDuty had wide enough active diagonal for shunt */
  VCM_State_t state;
  uint32_t fault_flags;
  uint32_t adc1_ovr_cnt;
  uint32_t adc2_ovr_cnt;
  volatile uint8_t iref_override_en;
  volatile float iref_override_a;
  /* Open-loop duty: when mod_override_en!=0, skip PI and apply mod_override */
  volatile uint8_t mod_override_en;
  volatile float mod_override;
  uint16_t ifb_cal_count;
  float ifb_cal_acc;
} VCM_Handle_t;

extern VCM_Handle_t g_vcm;
extern HRTIM_HandleTypeDef hhrtim; /* CubeMX handle in main.c */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

#if VCM_SCOPE_ENABLE
typedef struct
{
  float ifb_a;
  float iref_a;
  float mod;
  uint16_t adc_ifb;
  uint16_t adc_iref;
  uint8_t state;   /* VCM_State_t */
  uint8_t en;      /* DRV_EN active */
  uint8_t fault;   /* fault_flags != 0 */
  uint8_t ovr;     /* bit0=ADC1 OVR, bit1=ADC2 OVR at sample time */
} VCM_ScopeSample_t;

extern volatile VCM_ScopeSample_t g_vcm_scope[VCM_SCOPE_LEN];
extern volatile uint32_t g_vcm_scope_wr;      /* next write index */
extern volatile uint32_t g_vcm_scope_count;   /* total samples since arm/reset */
extern volatile uint8_t g_vcm_scope_frozen;   /* 1 = stop recording */
extern volatile uint32_t g_vcm_scope_capture_n; /* 0 = continuous; else freeze after N */

void VCM_Scope_Reset(void);
void VCM_Scope_Freeze(void);
void VCM_Scope_Resume(void);
/* One-shot: clear buffer, record n samples (clamped to VCM_SCOPE_LEN), then freeze */
void VCM_Scope_Capture(uint32_t n);
#endif

void VCM_Init(void);
void VCM_HRTIM_InitTimers(void);
void VCM_Start(void);
void VCM_Stop(void);
void VCM_SetIref(float iref_a);
void VCM_ClearFault(void);
void VCM_ServiceFaultClear(void);
int VCM_AdcStart(void);
void VCM_CurrentLoop_IRQHandler(void);
void VCM_EnterFault(uint32_t flag);
void VCM_ServiceDrvEnable(void);
bool VCM_IsDrvEnActive(void);
/* Re-run ifb zero calibration (50% duty average). Safe to call from debugger while RUN/IDLE. */
void VCM_CalibrateIfbOffset(void);

float VCM_AdcToAmpere(uint16_t raw);

#endif /* VCM_CTRL_H */
