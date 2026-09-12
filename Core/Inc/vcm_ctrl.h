/**
 * @file    vcm_ctrl.h
 * @brief   VCM current-loop API - PI + R/L FF, tri-level / bipolar PWM
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
  float iref_offset_a;
  float ifb_a;
  float ifb_avg_a;     /* period mean of I+/I- → PI */
  float ifb_ip_a;
  float ifb_im_a;
  float ifb_raw_a;
  float ifb_offset_a;
  float iref_cal_acc;
  float mod;
  float mod_ff;
  float integral;
  float kp;
  float ki;
  float ff_mod_per_a;
  float ifb_cal_acc;
  float iref_override_a;
  float cal_iref_mean;      /* EMA for UART 'G' */
  float cal_ifb_mean;
  uint32_t fault_flags;
  uint32_t adc1_ovr_cnt;
  uint32_t adc2_ovr_cnt;
  uint16_t adc_ifb;
  uint16_t adc_iref;
  uint16_t ifb_cal_count;
  VCM_State_t state;
  uint8_t adc_active_valid;
  uint8_t adc_pair_ok;
  uint8_t pwm_armed;
  uint8_t calib_valid;
  uint8_t iref_override_en;
  uint8_t pi_steady;          /* 1 = hold: reduced PI, P@400Hz I@50Hz */
  uint8_t pwm_mode;           /* VCM_PWM_MODE_*; Live Expr / UART B|T */
  uint32_t isr_ticks;
  uint32_t start_count;
  uint32_t stop_count;
  uint32_t calib_count;
  uint32_t calib_reuse_count;
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
void VCM_HRTIM_InitTimers(void);
void VCM_Start(void);
void VCM_Stop(void);
void VCM_ClearFault(void);
void VCM_ServiceFaultClear(void);
int VCM_AdcStart(void);
void VCM_CurrentLoop_IRQHandler(void);
void VCM_EnterFault(uint32_t flag);
void VCM_ServiceDrvEnable(void);
bool VCM_IsDrvEnActive(void);
float VCM_AdcToIfbAmpere(uint16_t raw);
float VCM_AdcToIrefAmpere(uint16_t raw);

/** Handle ASCII cal cmds: G/F/Z/A. Returns 1 if consumed. */
uint8_t VCM_ServiceUartCmd(uint8_t cmd);

#endif /* VCM_CTRL_H */
