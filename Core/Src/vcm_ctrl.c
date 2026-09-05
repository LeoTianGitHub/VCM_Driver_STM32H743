/**
 * @file    vcm_ctrl.c
 * @brief   HRTIM H-bridge + PI + R/L FF + tri-level / bipolar PWM
 */
#include "vcm_ctrl.h"
#include "vcm_config.h"
#include <stdint.h>

VCM_Handle_t g_vcm;

static volatile uint16_t vcm_adc1_dma __attribute__((section(".dma_buffer"), aligned(32)));
static volatile uint16_t vcm_adc2_dma __attribute__((section(".dma_buffer"), aligned(32)));

static void VCM_HRTIM_ApplyDuty(float m);
static uint16_t VCM_AdcTrigQuiet(uint16_t edge_cnt);
static void VCM_ConfigHalfBridge(uint32_t timer_idx, uint32_t output1, uint32_t output2);
static void VCM_FaultPin_Set(bool fault_active);
static void VCM_LoopStateReset(void);
static float VCM_Absf(float x);
static float VCM_PiStep(float ref, float fdbk);
static void VCM_HRTIM_SetChop(uint32_t timer_idx, uint8_t chop);
static void VCM_UartWrite(const char *s);
static void VCM_UartWriteI32(int32_t v);
static void VCM_UartWriteMilli(float a);
#if VCM_DIAG_EN
static void VCM_DiagService(void);
#endif

static uint16_t vcm_ocp_hits;
static uint8_t vcm_chop_a = 0xFFU;
static uint8_t vcm_chop_b = 0xFFU;
static uint16_t vcm_idle_deb;
static float vcm_iref_z1;
static int8_t vcm_tri_sign; /* +1: A chops / B low; -1: B chops / A low; 0: unset */
static uint32_t vcm_cal_done_ms;
static uint32_t vcm_en_off_ms;
static uint16_t vcm_en_off_samples;

#if VCM_DIAG_EN
static float vcm_diag_iref_prev;
static float vcm_diag_target;
static float vcm_diag_start;
static uint32_t vcm_diag_t;
static uint8_t vcm_diag_state;
#endif

static float VCM_Absf(float x)
{
  return (x >= 0.0f) ? x : -x;
}

static void VCM_LoopStateReset(void)
{
  g_vcm.integral = 0.0f;
  g_vcm.mod = 0.0f;
  g_vcm.mod_ff = 0.0f;
  g_vcm.iref_a = 0.0f;
  g_vcm.pwm_armed = 0U;
  vcm_idle_deb = 0U;
  vcm_iref_z1 = 0.0f;
  vcm_tri_sign = 0;
}

/*
 * m = Kp*e + Ki*∫e + R·I_ff + L·di/dt_ff
 * Long-idle clamp for true standstill only (not AC zero-cross).
 */
static float VCM_PiStep(float ref, float fdbk)
{
  float err;
  float m_ff;
  float m_unsat;
  float m;
  float di_dt;
  float ref_abs = VCM_Absf(ref);

  if (g_vcm.pwm_armed != 0U)
  {
    if (ref_abs < VCM_IDLE_ENTER_A)
    {
      if (vcm_idle_deb < 0xFFFFU)
      {
        vcm_idle_deb++;
      }
      if (vcm_idle_deb >= VCM_IDLE_ENTER_DEB)
      {
        g_vcm.pwm_armed = 0U;
        g_vcm.integral = 0.0f;
        g_vcm.mod_ff = 0.0f;
        vcm_idle_deb = 0U;
        vcm_iref_z1 = ref;
        return 0.0f;
      }
    }
    else
    {
      vcm_idle_deb = 0U;
    }
  }
  else
  {
    if (ref_abs > VCM_IDLE_EXIT_A)
    {
      g_vcm.pwm_armed = 1U;
      g_vcm.integral = 0.0f;
      vcm_idle_deb = 0U;
      vcm_iref_z1 = ref;
    }
    else
    {
      g_vcm.integral = 0.0f;
      g_vcm.mod_ff = 0.0f;
      vcm_iref_z1 = ref;
      return 0.0f;
    }
  }

  err = ref - fdbk;
  di_dt = (ref - vcm_iref_z1) * (float)VCM_CTRL_FREQ_HZ;
  vcm_iref_z1 = ref;

  m_ff = (ref * g_vcm.ff_mod_per_a) + (di_dt * VCM_L_FF_MOD_PER_A);
  g_vcm.mod_ff = m_ff;

  m_unsat = (g_vcm.kp * err) + g_vcm.integral + m_ff;

  if (!(((m_unsat >= VCM_MOD_MAX) && (err > 0.0f)) ||
        ((m_unsat <= -VCM_MOD_MAX) && (err < 0.0f))))
  {
    g_vcm.integral += g_vcm.ki * err * VCM_PWM_TS_S;
  }
  if (g_vcm.integral > VCM_I_INTEGRAL_LIM)
  {
    g_vcm.integral = VCM_I_INTEGRAL_LIM;
  }
  if (g_vcm.integral < -VCM_I_INTEGRAL_LIM)
  {
    g_vcm.integral = -VCM_I_INTEGRAL_LIM;
  }

  m = (g_vcm.kp * err) + g_vcm.integral + m_ff;
  if (m > VCM_MOD_MAX)
  {
    m = VCM_MOD_MAX;
  }
  if (m < -VCM_MOD_MAX)
  {
    m = -VCM_MOD_MAX;
  }
  return m;
}

float VCM_AdcToIfbAmpere(uint16_t raw)
{
  return VCM_IFB_POLARITY * ((float)raw - VCM_ADC_MID) / VCM_IFB_COUNTS_PER_A;
}

float VCM_AdcToIrefAmpere(uint16_t raw)
{
  return VCM_IREF_POLARITY * ((float)raw - VCM_ADC_MID) / VCM_IREF_COUNTS_PER_A;
}

static void VCM_FaultPin_Set(bool fault_active)
{
  HAL_GPIO_WritePin(VCM_FAULT_GPIO_Port, VCM_FAULT_Pin,
                    fault_active ? VCM_FAULT_ACTIVE_LEVEL : VCM_FAULT_INACTIVE_LEVEL);
}

int VCM_AdcStart(void)
{
  vcm_adc1_dma = (uint16_t)VCM_ADC_MID;
  vcm_adc2_dma = (uint16_t)VCM_ADC_MID;

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&vcm_adc1_dma, 1U) != HAL_OK)
  {
    return -1;
  }
  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&vcm_adc2_dma, 1U) != HAL_OK)
  {
    return -1;
  }

  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(hadc2.DMA_Handle, DMA_IT_TC | DMA_IT_HT);
  HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);

  return 0;
}

static uint16_t VCM_AdcTrigQuiet(uint16_t edge_cnt)
{
  const uint16_t period = VCM_PWM_PERIOD;
  const uint16_t mar = VCM_ADC_TRIG_EDGE_MARGIN;
  uint16_t on_len;
  uint16_t off_len;
  uint16_t trig;

  if (edge_cnt > (2U * mar))
  {
    on_len = (uint16_t)(edge_cnt - (2U * mar));
  }
  else
  {
    on_len = 0U;
  }

  if (edge_cnt < (period - (2U * mar)))
  {
    off_len = (uint16_t)(period - edge_cnt - (2U * mar));
  }
  else
  {
    off_len = 0U;
  }

  if (on_len >= off_len)
  {
    trig = (on_len > 0U) ? (uint16_t)(mar + (on_len / 2U)) : (uint16_t)(period / 2U);
  }
  else
  {
    trig = (uint16_t)(edge_cnt + mar + (off_len / 2U));
  }

  if (trig < mar)
  {
    trig = mar;
  }
  if (trig > (uint16_t)(period - mar))
  {
    trig = (uint16_t)(period - mar);
  }
  return trig;
}

static void VCM_HRTIM_SetChop(uint32_t timer_idx, uint8_t chop)
{
  uint8_t *prev;
  HRTIM_Timerx_TypeDef *tim;

  prev = (timer_idx == HRTIM_TIMERINDEX_TIMER_A) ? &vcm_chop_a : &vcm_chop_b;
  if (*prev == chop)
  {
    return;
  }
  *prev = chop;

  tim = &hhrtim.Instance->sTimerxRegs[timer_idx];
  if (chop != 0U)
  {
    tim->SETx1R = HRTIM_SET1R_PER;
    tim->RSTx1R = HRTIM_RST1R_CMP1;
  }
  else
  {
    tim->SETx1R = 0U;
    tim->RSTx1R = HRTIM_RST1R_PER;
    tim->RSTx1R = HRTIM_RST1R_PER | HRTIM_RST1R_SRT;
  }
}

/*
 * Bipolar: da=0.5+m, db=0.5-m, Vcoil ≈ 2*m*Vbus.
 * Three-level: active leg duty D=2*|m|, idle leg CMP=duty_min with chop ON
 * so complementary low-side stays ON (sync freewheel). Never SetChop(0) while armed.
 */
static void VCM_HRTIM_ApplyDuty(float m)
{
  uint16_t cmp_a;
  uint16_t cmp_b;
  uint16_t adc_trig;
  uint16_t edge;
  uint16_t pwm_cnt;
  float da;
  float db;
  float mag;
  uint8_t use_tri;
  const uint16_t duty_min = 60U;
  const uint16_t duty_max = (uint16_t)(VCM_PWM_PERIOD - duty_min);

  if (m > VCM_MOD_MAX)
  {
    m = VCM_MOD_MAX;
  }
  if (m < -VCM_MOD_MAX)
  {
    m = -VCM_MOD_MAX;
  }

  if (g_vcm.pwm_armed == 0U)
  {
    vcm_tri_sign = 0;
    VCM_HRTIM_SetChop(HRTIM_TIMERINDEX_TIMER_A, 0U);
    VCM_HRTIM_SetChop(HRTIM_TIMERINDEX_TIMER_B, 0U);
    cmp_a = duty_min;
    cmp_b = duty_min;
    adc_trig = VCM_AdcTrigQuiet(duty_min);
    g_vcm.adc_active_valid = 1U;
    __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
    __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, adc_trig);
    __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp_b);
    return;
  }

  mag = (m >= 0.0f) ? m : -m;
  use_tri = 0U;
  if (g_vcm.pwm_mode == VCM_PWM_MODE_TRILEVEL)
  {
    if (vcm_tri_sign > 0)
    {
      if (m < -VCM_TRI_SIGN_OFF_M)
      {
        vcm_tri_sign = -1;
      }
    }
    else if (vcm_tri_sign < 0)
    {
      if (m > VCM_TRI_SIGN_OFF_M)
      {
        vcm_tri_sign = 1;
      }
    }
    else if (mag >= VCM_TRI_SIGN_ON_M)
    {
      vcm_tri_sign = (m >= 0.0f) ? 1 : -1;
    }

    if ((vcm_tri_sign != 0) && (mag >= VCM_TRI_BIPOLAR_M))
    {
      use_tri = 1U;
    }
  }
  else
  {
    vcm_tri_sign = 0;
  }

  if (use_tri != 0U)
  {
    /* D = 2*|m| keeps same average Vcoil scale as bipolar. */
    pwm_cnt = (uint16_t)((2.0f * mag * (float)VCM_PWM_PERIOD) + 0.5f);
    if (pwm_cnt < duty_min)
    {
      pwm_cnt = duty_min;
    }
    if (pwm_cnt > duty_max)
    {
      pwm_cnt = duty_max;
    }

    /* Both legs stay in chop mode: idle CMP=duty_min => HS off, LS ON. */
    VCM_HRTIM_SetChop(HRTIM_TIMERINDEX_TIMER_A, 1U);
    VCM_HRTIM_SetChop(HRTIM_TIMERINDEX_TIMER_B, 1U);

    if (vcm_tri_sign > 0)
    {
      cmp_a = pwm_cnt;
      cmp_b = duty_min;
    }
    else
    {
      cmp_a = duty_min;
      cmp_b = pwm_cnt;
    }

    adc_trig = VCM_AdcTrigQuiet(pwm_cnt);
    g_vcm.adc_active_valid = 1U;
    __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
    __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, adc_trig);
    __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp_b);
    return;
  }

  da = 0.5f + m;
  db = 0.5f - m;
  cmp_a = (uint16_t)((da * (float)VCM_PWM_PERIOD) + 0.5f);
  cmp_b = (uint16_t)((db * (float)VCM_PWM_PERIOD) + 0.5f);
  if (cmp_a < duty_min)
  {
    cmp_a = duty_min;
  }
  if (cmp_b < duty_min)
  {
    cmp_b = duty_min;
  }
  if (cmp_a > duty_max)
  {
    cmp_a = duty_max;
  }
  if (cmp_b > duty_max)
  {
    cmp_b = duty_max;
  }

  VCM_HRTIM_SetChop(HRTIM_TIMERINDEX_TIMER_A, 1U);
  VCM_HRTIM_SetChop(HRTIM_TIMERINDEX_TIMER_B, 1U);

  edge = (cmp_a < cmp_b) ? cmp_a : cmp_b;
  adc_trig = VCM_AdcTrigQuiet(edge);

  g_vcm.adc_active_valid = 1U;
  __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
  __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, adc_trig);
  __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp_b);
}

void VCM_Stop(void)
{
  g_vcm.stop_count++;
  HAL_HRTIM_WaveformOutputStop(&hhrtim,
                               HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                               HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
  VCM_LoopStateReset();
  vcm_ocp_hits = 0U;
  g_vcm.ifb_cal_count = 0U;
  g_vcm.ifb_cal_acc = 0.0f;
  VCM_HRTIM_ApplyDuty(0.0f);
  if (g_vcm.state != VCM_STATE_FAULT)
  {
    g_vcm.state = VCM_STATE_IDLE;
  }
}

void VCM_ClearFault(void)
{
  g_vcm.fault_flags = 0U;
  g_vcm.state = VCM_STATE_IDLE;
  vcm_ocp_hits = 0U;
  VCM_FaultPin_Set(false);
}

void VCM_ServiceFaultClear(void)
{
  if ((g_vcm.state == VCM_STATE_FAULT) && !VCM_IsDrvEnActive())
  {
    VCM_ClearFault();
  }
}

void VCM_EnterFault(uint32_t flag)
{
  g_vcm.fault_count++;
  g_vcm.calib_valid = 0U;
  g_vcm.fault_flags |= flag;
  g_vcm.state = VCM_STATE_FAULT;
  VCM_FaultPin_Set(true);
  VCM_Stop();
}

bool VCM_IsDrvEnActive(void)
{
  return (HAL_GPIO_ReadPin(VCM_DRV_EN_GPIO_Port, VCM_DRV_EN_Pin) == VCM_DRV_EN_ACTIVE_LEVEL);
}

void VCM_ServiceDrvEnable(void)
{
  if (!VCM_IsDrvEnActive())
  {
    /*
     * Time-based debounce. The 50 kHz ISR tears the drive down faster (~160 us,
     * VCM_EN_GLITCH_SAMPLES); this path only matters if the ISR is not running.
     * Without the debounce, a single glitch here restarts IFB recalibration
     * and the drive is blind while it runs.
     */
    if ((g_vcm.state == VCM_STATE_RUN) || (g_vcm.state == VCM_STATE_CALIB))
    {
      if (vcm_en_off_ms == 0U)
      {
        vcm_en_off_ms = HAL_GetTick();
      }
      else if ((HAL_GetTick() - vcm_en_off_ms) >= VCM_EN_OFF_DEBOUNCE_MS)
      {
        VCM_Stop();
      }
    }
    return;
  }
  vcm_en_off_ms = 0U;

  if ((g_vcm.state == VCM_STATE_IDLE) && (g_vcm.fault_flags == 0U))
  {
    VCM_Start();
  }
}

void VCM_Start(void)
{
  if (g_vcm.state == VCM_STATE_FAULT)
  {
    return;
  }
  if (!VCM_IsDrvEnActive())
  {
    return;
  }

  g_vcm.start_count++;

  VCM_LoopStateReset();
  g_vcm.ifb_cal_count = 0U;
  g_vcm.ifb_cal_acc = 0.0f;
  g_vcm.iref_cal_acc = 0.0f;
  vcm_chop_a = 0xFFU;
  vcm_chop_b = 0xFFU;
  VCM_HRTIM_ApplyDuty(0.0f);

  if (HAL_HRTIM_WaveformCountStart_IT(&hhrtim,
                                      HRTIM_TIMERID_MASTER |
                                      HRTIM_TIMERID_TIMER_A |
                                      HRTIM_TIMERID_TIMER_B) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputStart(&hhrtim,
                                    HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                                    HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2) != HAL_OK)
  {
    Error_Handler();
  }

  vcm_chop_a = 0xFFU;
  vcm_chop_b = 0xFFU;
  VCM_HRTIM_ApplyDuty(0.0f);

  /*
   * Fast re-enable: the offsets drift slowly with temperature, so a brief enable
   * drop does not justify another 2.56 ms of clamped PWM. Reuse them and go
   * straight to RUN; set VCM_CAL_REUSE_MS 0 to always recalibrate.
   */
  if ((VCM_CAL_REUSE_MS > 0U) && (g_vcm.calib_valid != 0U) &&
      ((HAL_GetTick() - vcm_cal_done_ms) < (uint32_t)VCM_CAL_REUSE_MS))
  {
    g_vcm.calib_reuse_count++;
    g_vcm.state = VCM_STATE_RUN;
    return;
  }

  g_vcm.state = VCM_STATE_CALIB;
}

#if VCM_DIAG_EN
/*
 * Read-only observation: how many ISR ticks (20 us each @ 50 kHz) the loop needs from an
 * IREF step — as the firmware sees it — until the current settles inside
 * VCM_DIAG_DONE_BAND. Nothing here feeds back into the loop.
 *   small number  -> loop and plant are fine, the delay is upstream
 *   large number  -> the lag really is inside the loop/plant
 */
static void VCM_DiagService(void)
{
  float step;
  float band;

  g_vcm.isr_ticks++;

  step = g_vcm.iref_a - vcm_diag_iref_prev;
  if (step < 0.0f)
  {
    step = -step;
  }
  vcm_diag_iref_prev = g_vcm.iref_a;

  if (vcm_diag_state == 0U)
  {
    if (step > VCM_DIAG_STEP_A)
    {
      vcm_diag_state = 1U;
      vcm_diag_t = 0U;
      vcm_diag_start = g_vcm.ifb_a;
      vcm_diag_target = g_vcm.iref_a;
      g_vcm.diag_iref_at_step = g_vcm.iref_a;
      g_vcm.diag_ifb_at_step = g_vcm.ifb_a;
#if VCM_DIAG_UART_MARK
      /* Scope trigger on the existing USART1 TX pin: marks the exact instant the
       * firmware saw the command step (~87 µs frame at 115200, no new pin). */
      if ((huart1.Instance->ISR & USART_ISR_TXE_TXFNF) != 0U)
      {
        huart1.Instance->TDR = 0x55U;
      }
#endif
    }
    return;
  }

  vcm_diag_t++;

  band = vcm_diag_target - vcm_diag_start;
  if (band < 0.0f)
  {
    band = -band;
  }
  band *= VCM_DIAG_DONE_BAND;

  if (VCM_Absf(g_vcm.ifb_a - vcm_diag_target) <= band)
  {
    g_vcm.diag_ticks = vcm_diag_t;
    g_vcm.diag_ifb_final = g_vcm.ifb_a;
    vcm_diag_state = 0U;
  }
  else if (vcm_diag_t >= VCM_DIAG_TIMEOUT_TICKS)
  {
    g_vcm.diag_timeouts++;
    g_vcm.diag_ticks = vcm_diag_t;
    g_vcm.diag_ifb_final = g_vcm.ifb_a;
    vcm_diag_state = 0U;
  }
}
#endif /* VCM_DIAG_EN */

void VCM_CurrentLoop_IRQHandler(void)
{
  float m;
  float iref;

  if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR) != 0U)
  {
    g_vcm.adc1_ovr_cnt++;
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
  }
  if (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_OVR) != 0U)
  {
    g_vcm.adc2_ovr_cnt++;
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_OVR);
  }

  __DSB();
  g_vcm.adc_ifb  = vcm_adc1_dma;
  g_vcm.adc_iref = vcm_adc2_dma;

  g_vcm.ifb_raw_a = VCM_AdcToIfbAmpere(g_vcm.adc_ifb);
  g_vcm.ifb_a = g_vcm.ifb_raw_a - g_vcm.ifb_offset_a;

  if (g_vcm.iref_override_en != 0U)
  {
    iref = g_vcm.iref_override_a;
  }
  else
  {
    iref = VCM_AdcToIrefAmpere(g_vcm.adc_iref) - g_vcm.iref_offset_a;
  }
  if (iref > VCM_I_CMD_MAX_A)
  {
    iref = VCM_I_CMD_MAX_A;
  }
  if (iref < -VCM_I_CMD_MAX_A)
  {
    iref = -VCM_I_CMD_MAX_A;
  }
  g_vcm.iref_a = iref;

  /* Rolling means for UART gain report (no effect on loop). */
  g_vcm.cal_iref_mean += VCM_CAL_AVG_ALPHA * (g_vcm.iref_a - g_vcm.cal_iref_mean);
  g_vcm.cal_ifb_mean += VCM_CAL_AVG_ALPHA * (g_vcm.ifb_a - g_vcm.cal_ifb_mean);

  if (!VCM_IsDrvEnActive())
  {
    /*
     * DRV_EN is active-low on a long host wire. Require it to stay inactive for
     * VCM_EN_GLITCH_SAMPLES before tearing the drive down: a one-sample glitch
     * used to cost a full 2.56 ms recalibration with the bridge clamped, and
     * back-to-back glitches look exactly like an ms-scale dead zone.
     */
    if (vcm_en_off_samples < 0xFFFFU)
    {
      vcm_en_off_samples++;
    }
    if ((vcm_en_off_samples > VCM_EN_GLITCH_SAMPLES) &&
        ((g_vcm.state == VCM_STATE_RUN) || (g_vcm.state == VCM_STATE_CALIB)))
    {
      VCM_Stop();
    }
    return;
  }
  if (vcm_en_off_samples != 0U)
  {
    /* Recovered before the debounce expired: glitch, keep driving. */
    g_vcm.en_glitch_count++;
    vcm_en_off_samples = 0U;
  }

  if ((g_vcm.state != VCM_STATE_RUN) && (g_vcm.state != VCM_STATE_CALIB))
  {
    return;
  }

  if ((g_vcm.ifb_a > VCM_I_OCP_A) || (g_vcm.ifb_a < -VCM_I_OCP_A))
  {
    if (vcm_ocp_hits < 0xFFFFU)
    {
      vcm_ocp_hits++;
    }
    if (vcm_ocp_hits >= VCM_OCP_CONFIRM_SAMPLES)
    {
      VCM_EnterFault(2U);
      return;
    }
  }
  else
  {
    vcm_ocp_hits = 0U;
  }

  if (g_vcm.state == VCM_STATE_CALIB)
  {
    g_vcm.ifb_cal_acc += g_vcm.ifb_raw_a;
    g_vcm.iref_cal_acc += VCM_AdcToIrefAmpere(g_vcm.adc_iref);
    g_vcm.ifb_cal_count++;
    VCM_LoopStateReset();
    VCM_HRTIM_ApplyDuty(0.0f);

    if (g_vcm.ifb_cal_count >= VCM_IFB_CAL_SAMPLES)
    {
      float iref_mean = g_vcm.iref_cal_acc / (float)g_vcm.ifb_cal_count;

      g_vcm.ifb_offset_a = g_vcm.ifb_cal_acc / (float)g_vcm.ifb_cal_count;
      if (VCM_Absf(iref_mean) <= VCM_IREF_OFFSET_MAX_A)
      {
        g_vcm.iref_offset_a = iref_mean;
      }
      else
      {
        g_vcm.iref_offset_a = 0.0f;
      }
      g_vcm.ifb_cal_acc = 0.0f;
      g_vcm.iref_cal_acc = 0.0f;
      g_vcm.ifb_cal_count = 0U;
      VCM_LoopStateReset();
      g_vcm.state = VCM_STATE_RUN;
      g_vcm.calib_count++;
      g_vcm.calib_valid = 1U;
      vcm_cal_done_ms = HAL_GetTick();
    }
    return;
  }

#if VCM_DIAG_EN
  VCM_DiagService();
#endif

  m = VCM_PiStep(g_vcm.iref_a, g_vcm.ifb_a);
  g_vcm.mod = m;
  VCM_HRTIM_ApplyDuty(m);
}

void VCM_Init(void)
{
  g_vcm.iref_a = 0.0f;
  g_vcm.ifb_a = 0.0f;
  g_vcm.ifb_raw_a = 0.0f;
  g_vcm.ifb_offset_a = 0.0f;
  g_vcm.iref_offset_a = 0.0f;
  g_vcm.iref_cal_acc = 0.0f;
  g_vcm.pwm_armed = 0U;
  g_vcm.pwm_mode = VCM_PWM_MODE;
  g_vcm.iref_override_en = 0U;
  g_vcm.iref_override_a = 0.0f;
  g_vcm.cal_iref_mean = 0.0f;
  g_vcm.cal_ifb_mean = 0.0f;
  g_vcm.mod_ff = 0.0f;
  vcm_ocp_hits = 0U;
  VCM_LoopStateReset();
  g_vcm.kp = VCM_KP;
  g_vcm.ki = VCM_KI;
  g_vcm.ff_mod_per_a = VCM_FF_MOD_PER_A;
  g_vcm.state = VCM_STATE_IDLE;
  g_vcm.fault_flags = 0U;
  g_vcm.adc1_ovr_cnt = 0U;
  g_vcm.adc2_ovr_cnt = 0U;
  g_vcm.adc_active_valid = 0U;
  g_vcm.ifb_cal_count = 0U;
  g_vcm.ifb_cal_acc = 0.0f;
  g_vcm.calib_valid = 0U;
  g_vcm.isr_ticks = 0U;
  g_vcm.start_count = 0U;
  g_vcm.stop_count = 0U;
  g_vcm.calib_count = 0U;
  g_vcm.calib_reuse_count = 0U;
  g_vcm.exti_count = 0U;
  g_vcm.fault_count = 0U;
  g_vcm.en_glitch_count = 0U;
  g_vcm.diag_ticks = 0U;
  g_vcm.diag_timeouts = 0U;
  g_vcm.diag_iref_at_step = 0.0f;
  g_vcm.diag_ifb_at_step = 0.0f;
  g_vcm.diag_ifb_final = 0.0f;
  vcm_cal_done_ms = 0U;
  vcm_en_off_ms = 0U;
  vcm_en_off_samples = 0U;
#if VCM_DIAG_EN
  vcm_diag_state = 0U;
  vcm_diag_t = 0U;
  vcm_diag_iref_prev = 0.0f;
  vcm_diag_start = 0.0f;
  vcm_diag_target = 0.0f;
#endif

  VCM_FaultPin_Set(false);
  VCM_HRTIM_ApplyDuty(0.0f);
}

static void VCM_UartWrite(const char *s)
{
  while (*s != '\0')
  {
    while ((huart1.Instance->ISR & USART_ISR_TXE_TXFNF) == 0U)
    {
    }
    huart1.Instance->TDR = (uint32_t)((uint8_t)*s);
    s++;
  }
}

static void VCM_UartWriteI32(int32_t v)
{
  char buf[12];
  uint32_t u;
  uint8_t n = 0U;
  uint8_t i;

  if (v < 0)
  {
    VCM_UartWrite("-");
    u = (uint32_t)(-v);
  }
  else
  {
    u = (uint32_t)v;
  }
  if (u == 0U)
  {
    VCM_UartWrite("0");
    return;
  }
  while (u > 0U)
  {
    buf[n++] = (char)('0' + (u % 10U));
    u /= 10U;
  }
  for (i = n; i > 0U; i--)
  {
    char c[2] = {buf[i - 1U], '\0'};
    VCM_UartWrite(c);
  }
}

/* Print amperes as signed milliamp integer: +1234 = 1.234 A */
static void VCM_UartWriteMilli(float a)
{
  int32_t ma = (int32_t)((a * 1000.0f) + ((a >= 0.0f) ? 0.5f : -0.5f));
  VCM_UartWriteI32(ma);
}

/*
 * Gain-cal UART (115200):
 *   G — dump EMA iref/ifb (mA) and raw ADC for clamp/scope对照
 *   F — force IREF = VCM_CAL_FORCE_A (default 0.5 A)
 *   Z — force IREF = 0
 *   A — clear force, use analog IREF
 *
 * After F: clamp_meter_A / firmware_ifb_mA → next IFB_GAIN_TRIM.
 * After known Vin: (Vin/20*1) / firmware_iref → IREF_GAIN_TRIM.
 */
uint8_t VCM_ServiceUartCmd(uint8_t cmd)
{
  if (cmd == VCM_UART_CMD_GAIN)
  {
    VCM_UartWrite("CAL iref_mA=");
    VCM_UartWriteMilli(g_vcm.cal_iref_mean);
    VCM_UartWrite(" ifb_mA=");
    VCM_UartWriteMilli(g_vcm.cal_ifb_mean);
    VCM_UartWrite(" adc_iref=");
    VCM_UartWriteI32((int32_t)g_vcm.adc_iref);
    VCM_UartWrite(" adc_ifb=");
    VCM_UartWriteI32((int32_t)g_vcm.adc_ifb);
    VCM_UartWrite(" ov=");
    VCM_UartWriteI32((int32_t)g_vcm.iref_override_en);
    VCM_UartWrite(" armed=");
    VCM_UartWriteI32((int32_t)g_vcm.pwm_armed);
    VCM_UartWrite(" mode=");
    VCM_UartWriteI32((int32_t)g_vcm.pwm_mode);
    VCM_UartWrite("\r\n");
    return 1U;
  }
  if (cmd == VCM_UART_CMD_FORCE)
  {
    g_vcm.iref_override_a = VCM_CAL_FORCE_A;
    g_vcm.iref_override_en = 1U;
    VCM_UartWrite("FORCE ");
    VCM_UartWriteMilli(VCM_CAL_FORCE_A);
    VCM_UartWrite(" mA cmd\r\n");
    return 1U;
  }
  if (cmd == VCM_UART_CMD_ZERO)
  {
    g_vcm.iref_override_a = 0.0f;
    g_vcm.iref_override_en = 1U;
    VCM_UartWrite("FORCE 0\r\n");
    return 1U;
  }
  if (cmd == VCM_UART_CMD_ANALOG)
  {
    g_vcm.iref_override_en = 0U;
    VCM_UartWrite("ANALOG\r\n");
    return 1U;
  }
  if (cmd == VCM_UART_CMD_BIPOLAR)
  {
    g_vcm.pwm_mode = VCM_PWM_MODE_BIPOLAR;
    vcm_tri_sign = 0;
    VCM_UartWrite("PWM BIPOLAR\r\n");
    return 1U;
  }
  if (cmd == VCM_UART_CMD_TRILEVEL)
  {
    g_vcm.pwm_mode = VCM_PWM_MODE_TRILEVEL;
    vcm_tri_sign = 0;
    VCM_UartWrite("PWM TRILEVEL\r\n");
    return 1U;
  }
  return 0U;
}

static void VCM_ConfigHalfBridge(uint32_t timer_idx, uint32_t output1, uint32_t output2)
{
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg = {0};

  pTimeBaseCfg.Period = VCM_PWM_PERIOD;
  pTimeBaseCfg.RepetitionCounter = (timer_idx == HRTIM_TIMERINDEX_TIMER_A)
                                       ? VCM_PWM_REPETITION
                                       : 0x00U;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV1;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim, timer_idx, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }

  pTimerCfg.InterruptRequests = (timer_idx == HRTIM_TIMERINDEX_TIMER_A) ? HRTIM_TIM_IT_REP : HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_MASTER;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim, timer_idx, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }

  pCompareCfg.CompareValue = VCM_PWM_PERIOD / 2U;
  pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  pCompareCfg.AutoDelayedTimeout = 0;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, timer_idx, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }

  if (timer_idx == HRTIM_TIMERINDEX_TIMER_A)
  {
    pCompareCfg.CompareValue = VCM_PWM_PERIOD / 4U;
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, timer_idx, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
    {
      Error_Handler();
    }
  }

  pDeadTimeCfg.Prescaler = VCM_DT_PRESCALER;
  pDeadTimeCfg.RisingValue = VCM_DT_RISING;
  pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  pDeadTimeCfg.FallingValue = VCM_DT_FALLING;
  pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim, timer_idx, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }

  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim, timer_idx, output1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }

  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim, timer_idx, output2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
}

void VCM_HRTIM_InitTimers(void)
{
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_ADCTriggerCfgTypeDef adc_trig = {0};

  pTimeBaseCfg.Period = VCM_PWM_PERIOD;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV1;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }

  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }

  VCM_ConfigHalfBridge(HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, HRTIM_OUTPUT_TA2);
  VCM_ConfigHalfBridge(HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, HRTIM_OUTPUT_TB2);

  adc_trig.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
  adc_trig.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP2;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim, HRTIM_ADCTRIGGER_1, &adc_trig) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim_handle, uint32_t TimerIdx)
{
  (void)hhrtim_handle;
  if (TimerIdx == HRTIM_TIMERINDEX_TIMER_A)
  {
    VCM_CurrentLoop_IRQHandler();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == VCM_DRV_EN_Pin)
  {
    g_vcm.exti_count++;
    VCM_ServiceDrvEnable();
  }
}