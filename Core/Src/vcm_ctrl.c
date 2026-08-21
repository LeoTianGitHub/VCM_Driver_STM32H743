/**
 * @file    vcm_ctrl.c
 * @brief   HRTIM H-bridge PWM + PI current loop (STM32H743)
 */
#include "vcm_ctrl.h"
#include "vcm_config.h"

VCM_Handle_t g_vcm;

/* DMA1 cannot access DTCM — place sample buffers in AXI SRAM (RAM_D1) */
static volatile uint16_t vcm_adc1_dma __attribute__((section(".dma_buffer"), aligned(32)));
static volatile uint16_t vcm_adc2_dma __attribute__((section(".dma_buffer"), aligned(32)));

static void VCM_HRTIM_ApplyDuty(uint16_t cmp_a, uint16_t cmp_b);
static void VCM_ConfigHalfBridge(uint32_t timer_idx, uint32_t output1, uint32_t output2);
static void VCM_FaultPin_Set(bool fault_active);

static float vcm_ifb_z1;
static float vcm_mod_ff_f;
static float vcm_mod_lpf_f;
static float vcm_iref_ff_f;
static float vcm_iref_ff_z1;
static uint8_t vcm_ifb_pi_primed;
static uint8_t vcm_ff_l_primed;
static uint16_t vcm_ocp_hits;

static void VCM_FfStateReset(void)
{
  vcm_mod_ff_f = 0.0f;
  vcm_iref_ff_f = 0.0f;
  vcm_iref_ff_z1 = 0.0f;
  vcm_ff_l_primed = 0U;
  g_vcm.mod_ff = 0.0f;
  g_vcm.mod_ff_l = 0.0f;
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

  /* Armed for HRTIM ADCTRG1: one conversion per PWM period into DMA buffer */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&vcm_adc1_dma, 1U) != HAL_OK)
  {
    return -1;
  }
  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&vcm_adc2_dma, 1U) != HAL_OK)
  {
    return -1;
  }

  /* HAL_ADC_Start_DMA uses HAL_DMA_Start_IT: circular DMA @ PWM rate must not
   * enable TC/HT IRQs or main/SysTick starve. Samples are read in the 50 kHz
   * VCM_CurrentLoop_IRQHandler — TC/HT IRQs are not needed. */
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(hadc2.DMA_Handle, DMA_IT_TC | DMA_IT_HT);
  HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);

  return 0;
}

static void VCM_HRTIM_ApplyDuty(uint16_t cmp_a, uint16_t cmp_b)
{
  uint16_t adc_trig;

  if (cmp_a < VCM_DUTY_MIN_COUNTS)
  {
    cmp_a = VCM_DUTY_MIN_COUNTS;
  }
  if (cmp_a > VCM_DUTY_MAX_COUNTS)
  {
    cmp_a = VCM_DUTY_MAX_COUNTS;
  }
  if (cmp_b < VCM_DUTY_MIN_COUNTS)
  {
    cmp_b = VCM_DUTY_MIN_COUNTS;
  }
  if (cmp_b > VCM_DUTY_MAX_COUNTS)
  {
    cmp_b = VCM_DUTY_MAX_COUNTS;
  }

  /* Coil-series RS: I valid all period. Prefer PERIOD/4 so that at m≈0
   * (both edges at PERIOD/2) the sample is not on the switching instant. */
  adc_trig = (uint16_t)(VCM_PWM_PERIOD / 4U);
  g_vcm.adc_active_valid = 1U;

  __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
  __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, adc_trig);
  __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp_b);
}

void VCM_SetIref(float iref_a)
{
  if (iref_a > VCM_I_MAX_A)
  {
    iref_a = VCM_I_MAX_A;
  }
  if (iref_a < -VCM_I_MAX_A)
  {
    iref_a = -VCM_I_MAX_A;
  }
  g_vcm.iref_override_a = iref_a;
  g_vcm.iref_a = iref_a;
}

void VCM_Stop(void)
{
  HAL_HRTIM_WaveformOutputStop(&hhrtim,
                               HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                               HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
  g_vcm.integral = 0.0f;
  g_vcm.mod = 0.0f;
  VCM_FfStateReset();
  vcm_mod_lpf_f = 0.0f;
  vcm_ocp_hits = 0U;
  g_vcm.ifb_pi_a = 0.0f;
  vcm_ifb_z1 = 0.0f;
  vcm_ifb_pi_primed = 0U;
  g_vcm.ifb_cal_count = 0U;
  g_vcm.ifb_cal_acc = 0.0f;
  VCM_HRTIM_ApplyDuty((uint16_t)(VCM_PWM_PERIOD / 2U), (uint16_t)(VCM_PWM_PERIOD / 2U));
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
  /* Host must deassert enable (DRV_EN inactive) to acknowledge / clear fault latch */
  if ((g_vcm.state == VCM_STATE_FAULT) && !VCM_IsDrvEnActive())
  {
    VCM_ClearFault();
  }
}

void VCM_EnterFault(uint32_t flag)
{
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
    if ((g_vcm.state == VCM_STATE_RUN) || (g_vcm.state == VCM_STATE_CALIB))
    {
      VCM_Stop();
    }
    return;
  }

  if ((g_vcm.state == VCM_STATE_IDLE) && (g_vcm.fault_flags == 0U))
  {
    VCM_Start();
  }
}

void VCM_CalibrateIfbOffset(void)
{
  if (g_vcm.state == VCM_STATE_FAULT)
  {
    return;
  }
  g_vcm.ifb_cal_count = 0U;
  g_vcm.ifb_cal_acc = 0.0f;
  g_vcm.integral = 0.0f;
  g_vcm.mod = 0.0f;
  vcm_mod_lpf_f = 0.0f;
  VCM_HRTIM_ApplyDuty((uint16_t)(VCM_PWM_PERIOD / 2U), (uint16_t)(VCM_PWM_PERIOD / 2U));

  if (g_vcm.state == VCM_STATE_IDLE)
  {
    /* Same bring-up path as Start, but end state will be CALIB then RUN */
    VCM_Start();
    return;
  }
  g_vcm.state = VCM_STATE_CALIB;
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

  g_vcm.integral = 0.0f;
  g_vcm.mod = 0.0f;
  VCM_FfStateReset();
  vcm_mod_lpf_f = 0.0f;
  g_vcm.iref_a = 0.0f;
  g_vcm.ifb_cal_count = 0U;
  g_vcm.ifb_cal_acc = 0.0f;
  VCM_HRTIM_ApplyDuty((uint16_t)(VCM_PWM_PERIOD / 2U), (uint16_t)(VCM_PWM_PERIOD / 2U));

  /* Start Master|A|B together: one MCR write keeps legs + ADC timebase aligned */
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

  /* Average ifb at 50% before enabling PI (removes static sense offset) */
  g_vcm.state = VCM_STATE_CALIB;
}

void VCM_CurrentLoop_IRQHandler(void)
{
  bool en_active;
  float err;
  float m;
  float m_ff;
  float m_ff_l;
  float m_unsat;
  float dm;
  float iref_abs;
  uint16_t cmp_a;
  uint16_t cmp_b;

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

  if (g_vcm.iref_override_en != 0U)
  {
    g_vcm.iref_a = g_vcm.iref_override_a;
  }
  else
  {
    g_vcm.iref_a = VCM_AdcToIrefAmpere(g_vcm.adc_iref);
  }
  if (g_vcm.iref_a > VCM_I_MAX_A)
  {
    g_vcm.iref_a = VCM_I_MAX_A;
  }
  if (g_vcm.iref_a < -VCM_I_MAX_A)
  {
    g_vcm.iref_a = -VCM_I_MAX_A;
  }

  /* Bidirectional series sense: signed coil current (minus zero calib). */
  g_vcm.ifb_a = g_vcm.ifb_raw_a - g_vcm.ifb_offset_a;

  /* Light EMA; reject only huge single-period spikes (switch ringing). */
  {
    float d = g_vcm.ifb_a - g_vcm.ifb_pi_a;
    if (d < 0.0f)
    {
      d = -d;
    }
    if ((vcm_ifb_pi_primed == 0U) || (d <= VCM_IFB_SPIKE_A))
    {
      if (vcm_ifb_pi_primed == 0U)
      {
        g_vcm.ifb_pi_a = g_vcm.ifb_a;
        vcm_ifb_pi_primed = 1U;
      }
      else
      {
        g_vcm.ifb_pi_a += VCM_IFB_PI_ALPHA * (g_vcm.ifb_a - g_vcm.ifb_pi_a);
      }
      vcm_ifb_z1 = g_vcm.ifb_a;
    }
  }

  en_active = VCM_IsDrvEnActive();
  if (!en_active)
  {
    if ((g_vcm.state == VCM_STATE_RUN) || (g_vcm.state == VCM_STATE_CALIB))
    {
      VCM_Stop();
    }
    return;
  }

  if ((g_vcm.state != VCM_STATE_RUN) && (g_vcm.state != VCM_STATE_CALIB))
  {
    return;
  }

  /* Hard OCP on corrected feedback (debounce: ignore single-edge spikes) */
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

  /* ---- Zero-offset calibration at 50% duty ---- */
  if (g_vcm.state == VCM_STATE_CALIB)
  {
    g_vcm.integral = 0.0f;
    g_vcm.mod = 0.0f;
    vcm_mod_lpf_f = 0.0f;
    g_vcm.ifb_pi_a = g_vcm.ifb_a;
    vcm_ifb_z1 = g_vcm.ifb_a;
    VCM_HRTIM_ApplyDuty((uint16_t)(VCM_PWM_PERIOD / 2U), (uint16_t)(VCM_PWM_PERIOD / 2U));

    g_vcm.ifb_cal_acc += g_vcm.ifb_raw_a;
    g_vcm.ifb_cal_count++;
    if (g_vcm.ifb_cal_count >= VCM_IFB_CAL_SAMPLES)
    {
      g_vcm.ifb_offset_a = g_vcm.ifb_cal_acc / (float)g_vcm.ifb_cal_count;
      g_vcm.ifb_cal_acc = 0.0f;
      g_vcm.ifb_cal_count = 0U;
      g_vcm.integral = 0.0f;
      g_vcm.mod = 0.0f;
      VCM_FfStateReset();
      vcm_mod_lpf_f = 0.0f;
      g_vcm.ifb_pi_a = 0.0f;
      vcm_ifb_z1 = 0.0f;
      vcm_ifb_pi_primed = 0U;
      g_vcm.state = VCM_STATE_RUN;
    }
    return;
  }

  /* Open-loop modulation (optional) */
  if (g_vcm.mod_override_en != 0U)
  {
    m = g_vcm.mod_override;
    if (m > VCM_MOD_MAX)
    {
      m = VCM_MOD_MAX;
    }
    if (m < -VCM_MOD_MAX)
    {
      m = -VCM_MOD_MAX;
    }
    g_vcm.integral = 0.0f;
    g_vcm.mod = m;
    vcm_mod_lpf_f = m;
    vcm_ff_l_primed = 0U;
    g_vcm.mod_ff_l = 0.0f;
  }
  else
  {
    iref_abs = (g_vcm.iref_a >= 0.0f) ? g_vcm.iref_a : -g_vcm.iref_a;

    /* Series sense: ifb_pi tracks continuously (adc_active_valid always 1). */
    err = g_vcm.iref_a - g_vcm.ifb_pi_a;

    if (iref_abs < VCM_IREF_NEAR_ZERO_A)
    {
      float ifb_abs = (g_vcm.ifb_pi_a >= 0.0f) ? g_vcm.ifb_pi_a : -g_vcm.ifb_pi_a;

      /* iref→0 while |ifb| still large: let L-FF brake alone (err=0).
       * Stacking −ifb*Kp on top of L-FF caused overshoot; freezing L-FF
       * immediately made the fall ~L/R (~300 µs). */
      g_vcm.integral = 0.0f;
      if (ifb_abs <= VCM_I_ERR_DEADBAND_A)
      {
        err = 0.0f;
        vcm_mod_ff_f = 0.0f;
        vcm_mod_lpf_f = 0.0f;
      }
      else if (ifb_abs > VCM_IFB_UNEXPECTED_A)
      {
        err = -g_vcm.ifb_pi_a * VCM_ZERO_BRAKE_KP_SCALE;
      }
      else
      {
        err = 0.0f;
      }
    }
    else
    {
      float err_abs = (err >= 0.0f) ? err : -err;
      if (err_abs < VCM_I_ERR_DEADBAND_A)
      {
        err = 0.0f;
      }
    }

#if VCM_FF_ENABLE
    /* Resistive drop FF + LPF: avoid instant m jump on iref step (undershoot). */
    if (iref_abs < VCM_IREF_NEAR_ZERO_A)
    {
      m_ff = 0.0f;
      vcm_mod_ff_f = 0.0f;
    }
    else
    {
      float alpha_r = g_vcm.ff_alpha;
      if (alpha_r < 0.0f)
      {
        alpha_r = 0.0f;
      }
      if (alpha_r > 1.0f)
      {
        alpha_r = 1.0f;
      }
      m_ff = g_vcm.iref_a * g_vcm.ff_mod_per_a;
      vcm_mod_ff_f += alpha_r * (m_ff - vcm_mod_ff_f);
      m_ff = vcm_mod_ff_f;
    }
#if VCM_FF_L_ENABLE
    /* L·di/dt FF: differentiate raw iref (min phase lag), optional LPF on m_ff_l.
     * Prefiltering iref before d/dt lagged the FF voltage and showed up as
     * current lagging the command once scale was raised. */
    {
      float ifb_abs = (g_vcm.ifb_pi_a >= 0.0f) ? g_vcm.ifb_pi_a : -g_vcm.ifb_pi_a;

      if ((iref_abs < VCM_IREF_NEAR_ZERO_A) && (ifb_abs <= VCM_I_ERR_DEADBAND_A))
      {
        vcm_iref_ff_f = 0.0f;
        vcm_iref_ff_z1 = g_vcm.iref_a;
        m_ff_l = 0.0f;
      }
      else if (vcm_ff_l_primed == 0U)
      {
        vcm_iref_ff_f = 0.0f;
        vcm_iref_ff_z1 = g_vcm.iref_a;
        vcm_ff_l_primed = 1U;
        m_ff_l = 0.0f;
      }
      else
      {
        float diref;
        float m_l_raw;
        float alpha_l = g_vcm.ff_l_iref_alpha;
        float lim_l = g_vcm.ff_l_mod_max;

        if (alpha_l < 0.0f)
        {
          alpha_l = 0.0f;
        }
        if (alpha_l > 1.0f)
        {
          alpha_l = 1.0f;
        }
        if (lim_l < 0.0f)
        {
          lim_l = 0.0f;
        }
        if (lim_l > VCM_MOD_MAX)
        {
          lim_l = VCM_MOD_MAX;
        }

        diref = (g_vcm.iref_a - vcm_iref_ff_z1) * (float)VCM_CTRL_FREQ_HZ;
        vcm_iref_ff_z1 = g_vcm.iref_a;
        m_l_raw = g_vcm.ff_l_scale * VCM_COIL_L_H * diref / (2.0f * VCM_VBUS_V);
        if (m_l_raw > lim_l)
        {
          m_l_raw = lim_l;
        }
        if (m_l_raw < -lim_l)
        {
          m_l_raw = -lim_l;
        }

        /* alpha=1 → m_ff_l follows raw (least lag); lower alpha softens noise. */
        vcm_iref_ff_f += alpha_l * (m_l_raw - vcm_iref_ff_f);
        m_ff_l = vcm_iref_ff_f;
      }
    }
    g_vcm.mod_ff_l = m_ff_l;
    m_ff += m_ff_l;
#else
    g_vcm.mod_ff_l = 0.0f;
#endif
#else
    m_ff = 0.0f;
    vcm_mod_ff_f = 0.0f;
    g_vcm.mod_ff_l = 0.0f;
#endif
    g_vcm.mod_ff = m_ff;

    m_unsat = g_vcm.kp * err + g_vcm.integral + m_ff;
    if (iref_abs < VCM_IREF_NEAR_ZERO_A)
    {
      /* Integral already snapped above; do not re-accumulate on brake err. */
    }
    else if (iref_abs < VCM_IREF_I_HOLD_A)
    {
      g_vcm.integral *= (1.0f - VCM_I_LEAK_PER_PERIOD);
    }
    else if (!(((m_unsat >= VCM_MOD_MAX) && (err > 0.0f)) ||
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

    m = g_vcm.kp * err + g_vcm.integral + m_ff;
    if (m > VCM_MOD_MAX)
    {
      m = VCM_MOD_MAX;
    }
    if (m < -VCM_MOD_MAX)
    {
      m = -VCM_MOD_MAX;
    }

    {
      float alpha_m = g_vcm.mod_lpf_alpha;
      float slew = g_vcm.mod_slew_per_period;

      if (alpha_m < 0.0f)
      {
        alpha_m = 0.0f;
      }
      if (alpha_m > 1.0f)
      {
        alpha_m = 1.0f;
      }
      if (slew < 0.0f)
      {
        slew = 0.0f;
      }
      if (slew > VCM_MOD_MAX)
      {
        slew = VCM_MOD_MAX;
      }

      vcm_mod_lpf_f += alpha_m * (m - vcm_mod_lpf_f);
      m = vcm_mod_lpf_f;

      dm = m - g_vcm.mod;
      if (dm > slew)
      {
        dm = slew;
      }
      if (dm < -slew)
      {
        dm = -slew;
      }
      m = g_vcm.mod + dm;
      g_vcm.mod = m;
    }
  }

  /* Dead-time compensation follows applied mod (same reason as ifb sign) */
  {
    float m_pwm = m;
    if (m > VCM_DT_COMP_HYST_A)
    {
      m_pwm += VCM_MOD_DT_COMP;
    }
    else if (m < -VCM_DT_COMP_HYST_A)
    {
      m_pwm -= VCM_MOD_DT_COMP;
    }
    if (m_pwm > VCM_MOD_MAX)
    {
      m_pwm = VCM_MOD_MAX;
    }
    if (m_pwm < -VCM_MOD_MAX)
    {
      m_pwm = -VCM_MOD_MAX;
    }
    cmp_a = (uint16_t)((0.5f + m_pwm) * (float)VCM_PWM_PERIOD);
    cmp_b = (uint16_t)((0.5f - m_pwm) * (float)VCM_PWM_PERIOD);
  }
  VCM_HRTIM_ApplyDuty(cmp_a, cmp_b);
}

void VCM_Init(void)
{
  g_vcm.iref_a = 0.0f;
  g_vcm.ifb_a = 0.0f;
  g_vcm.ifb_pi_a = 0.0f;
  g_vcm.ifb_raw_a = 0.0f;
  g_vcm.ifb_offset_a = 0.0f;
  vcm_ifb_z1 = 0.0f;
  vcm_ifb_pi_primed = 0U;
  vcm_ocp_hits = 0U;
  g_vcm.mod = 0.0f;
  VCM_FfStateReset();
  vcm_mod_lpf_f = 0.0f;
  g_vcm.integral = 0.0f;
  g_vcm.kp = VCM_KP;
  g_vcm.ki = VCM_KI;
  g_vcm.ff_mod_per_a = VCM_FF_MOD_PER_A;
  g_vcm.ff_alpha = VCM_FF_ALPHA;
  g_vcm.ff_l_scale = VCM_FF_L_SCALE;
  g_vcm.ff_l_iref_alpha = VCM_FF_L_IREF_ALPHA;
  g_vcm.ff_l_mod_max = VCM_FF_L_MOD_MAX;
  g_vcm.mod_lpf_alpha = VCM_MOD_LPF_ALPHA;
  g_vcm.mod_slew_per_period = VCM_MOD_SLEW_PER_PERIOD;
  g_vcm.state = VCM_STATE_IDLE;
  g_vcm.fault_flags = 0U;
  g_vcm.adc1_ovr_cnt = 0U;
  g_vcm.adc2_ovr_cnt = 0U;
  g_vcm.adc_active_valid = 0U;
  g_vcm.iref_override_en = (uint8_t)VCM_IREF_OVERRIDE_DEFAULT;
  g_vcm.iref_override_a = VCM_IREF_OVERRIDE_A_DEFAULT;
  g_vcm.mod_override_en = (uint8_t)VCM_MOD_OVERRIDE_DEFAULT;
  g_vcm.mod_override = VCM_MOD_OVERRIDE_A_DEFAULT;
  g_vcm.ifb_cal_count = 0U;
  g_vcm.ifb_cal_acc = 0.0f;

  VCM_FaultPin_Set(false);
  VCM_HRTIM_ApplyDuty((uint16_t)(VCM_PWM_PERIOD / 2U), (uint16_t)(VCM_PWM_PERIOD / 2U));
}

static void VCM_ConfigHalfBridge(uint32_t timer_idx, uint32_t output1, uint32_t output2)
{
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg = {0};

  pTimeBaseCfg.Period = VCM_PWM_PERIOD;
  /* Timer A: REP IRQ at VCM_CTRL_FREQ_HZ; Timer B: no REP IRQ */
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
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_MASTER; /* preload with Master */
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER; /* lock phase to Master */
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

  /* Timer A CMP2: ADC active-vector trigger (updated in ApplyDuty) */
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

  /* High-side reference PWM; low-side from dead-time unit (SET/RESET must be NONE) */
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

  /* Master: common timebase so Timer A/B stay phase-locked (ADC trig uses both cmp) */
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

  /* MX_HRTIM_Init() must have run first (handle + GPIO AF). */
  VCM_ConfigHalfBridge(HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, HRTIM_OUTPUT_TA2);
  VCM_ConfigHalfBridge(HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, HRTIM_OUTPUT_TB2);

  /* HRTIM ADCTRG1 → ADC1/ADC2 external trigger, once per PWM on TimerA CMP2 */
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
    VCM_ServiceDrvEnable();
  }
}
