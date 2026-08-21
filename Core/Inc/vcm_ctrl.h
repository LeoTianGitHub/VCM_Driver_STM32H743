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
  VCM_STATE_CALIB,
  VCM_STATE_RUN,
  VCM_STATE_FAULT
} VCM_State_t;

typedef struct
{
  float iref_a;
  float ifb_a;
  float ifb_pi_a;
  float ifb_raw_a;
  float ifb_offset_a;
  float mod;
  float mod_ff;
  float mod_ff_l;
  float integral;
  float kp;
  float ki;
  float ff_mod_per_a;
  float ff_alpha;
  float ff_l_scale;
  float ff_l_iref_alpha;
  float ff_l_mod_max;
  float mod_lpf_alpha;
  float mod_slew_per_period;
  float iref_override_a;
  float mod_override;
  float ifb_cal_acc;
  uint32_t fault_flags;
  uint32_t adc1_ovr_cnt;
  uint32_t adc2_ovr_cnt;
  uint16_t adc_ifb;
  uint16_t adc_iref;
  uint16_t ifb_cal_count;
  VCM_State_t state;
  uint8_t adc_active_valid;
  uint8_t iref_override_en;
  uint8_t mod_override_en;
} VCM_Handle_t;

extern VCM_Handle_t g_vcm;
extern HRTIM_HandleTypeDef hhrtim;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

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
void VCM_CalibrateIfbOffset(void);
float VCM_AdcToIfbAmpere(uint16_t raw);
float VCM_AdcToIrefAmpere(uint16_t raw);

#endif /* VCM_CTRL_H */
