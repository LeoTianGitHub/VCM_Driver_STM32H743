/**
 * @file    vcm_ctrl.c
 * @brief   HRTIM H-bridge + current observer + PI + R/L FF + tri-level PWM
 */
#include "vcm_ctrl.h"
#include "vcm_config.h"
#include "vcm_nv.h"
#include <stdint.h>

VCM_Handle_t g_vcm;

static volatile uint16_t vcm_adc1_dma[VCM_ADC_N] __attribute__((section(".dma_buffer"), aligned(32)));
static volatile uint16_t vcm_adc2_dma __attribute__((section(".dma_buffer"), aligned(32)));

static void VCM_HRTIM_ApplyDuty(float m);
static void VCM_DtCalAbort(void);
static void VCM_DtCalSample(void);
static void VCM_CapPush(void);
static void VCM_CapArm(const char *p, const char *end);
static void VCM_CapStop(void);
static float VCM_DtCalAmp(uint8_t step);
static void VCM_HRTIM_SetAdcPhase(uint16_t z0, uint16_t z1);
static void VCM_HRTIM_SetAdcPhasePair(uint16_t z0a, uint16_t z1a,
                                      uint16_t z0b, uint16_t z1b);
static void VCM_ConfigHalfBridge(uint32_t timer_idx, uint32_t output1, uint32_t output2);
static void VCM_FaultPin_Set(bool fault_active);
static void VCM_LoopStateReset(void);
static float VCM_Absf(float x);
static float VCM_PiStep(float ref, float fdbk_p, float fdbk_i, float ts, uint8_t full_rate);
static void VCM_IobsUpdate(float ifb, uint8_t meas_ok);
static void VCM_HRTIM_SetChop(uint32_t timer_idx, uint8_t chop);
static void VCM_UartWrite(const char *s);
static void VCM_UartFlush(void);
static void VCM_UartWriteI32(int32_t v);
static void VCM_UartWriteMilli(float a);
#if VCM_DIAG_EN
static void VCM_DiagService(void);
#endif

static volatile uint8_t vcm_nv_req; /* 1 = $W, 2 = $E */
#define VCM_CAP_SAMPLES      16384U
#define VCM_CAP_PAIRS_LINE   8U
/* "CAP d=" + 16×"-32767" + 15 commas + CRLF. 8N1 byte time is baud/10. */
#define VCM_CAP_LINE_BYTES   119U
static int16_t vcm_cap_buf[VCM_CAP_SAMPLES * 2U] __attribute__((section(".dma_buffer"), aligned(32)));
static volatile uint8_t vcm_cap_run;
static volatile uint8_t vcm_cap_done;
static volatile uint8_t vcm_cap_mode;
static volatile uint32_t vcm_cap_div;
static volatile uint32_t vcm_cap_div_cnt;
static volatile uint32_t vcm_cap_n;
static volatile uint32_t vcm_cap_wr;
static volatile uint32_t vcm_cap_rd;
static volatile uint32_t vcm_cap_ovf;
static volatile uint32_t vcm_cap_hz;
static volatile uint32_t vcm_cap_id;
static uint16_t vcm_ocp_hits;
static uint8_t vcm_chop_a = 0xFFU;
static uint8_t vcm_chop_b = 0xFFU;
static uint16_t vcm_idle_deb;
static float vcm_iref_z1;
static int8_t vcm_tri_sign; /* +1: A chops / B low; -1: B chops / A low; 0: unset */
static int8_t vcm_dt_sign;  /* bipolar compensation sign; 0 until |ifb| exceeds hold */
static float vcm_pi_ref;     /* notched command from the 50 kHz step */
static float vcm_base_i0;    /* previous period's first-coast sample */
static float vcm_base_avg;   /* previous period's coast average */
static volatile uint32_t vcm_pair_seq;  /* even: dma pair below is stable */
static volatile uint16_t vcm_pair_raw0;
static volatile uint16_t vcm_pair_raw1;
static uint32_t vcm_pair_taken;
static uint8_t vcm_base_ok;
static uint8_t vcm_half_done;   /* 1 = the −pulse step integrated VCM_HALF_TS_S */
static uint8_t vcm_rep_closed;  /* 1 = this period's 50 kHz PI ran */
static uint8_t vcm_next_valid;  /* next period has a tri-level −pulse */
static uint8_t vcm_act_valid;
static uint16_t vcm_next_cnt_a;
static uint16_t vcm_next_start_b;
static uint16_t vcm_next_dcm;
static uint16_t vcm_next_b1;
static uint16_t vcm_next_b2;
static uint16_t vcm_next_cmp3;
static uint16_t vcm_act_cnt_a;
static uint16_t vcm_act_start_b;
static uint16_t vcm_act_dcm;
static uint16_t vcm_act_cmp3;
static uint16_t vcm_act_b1;
static uint16_t vcm_act_b2;
static uint8_t vcm_run_bip = 1U; /* 1 = bipolar waveform this period */
static uint8_t vcm_gz_hold;
static uint8_t vcm_gz_seed;
static uint16_t vcm_gz_tick;
static float vcm_gz_ref;
static float vcm_gz_ref_z;
static float vcm_gz_s = 1.0f;
static float vcm_e_hat;      /* Ke·v estimate, volts */
static float vcm_e_apply;    /* volts actually added into m; 0 inside the gate */
static float vcm_e_innov_lp;
static float vcm_kp_fdbk_lp; /* Kp feedback after ~200 Hz EMA */
typedef struct
{
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
  uint16_t hz;
} VCM_Notch_t;
static VCM_Notch_t vcm_notch[4];
static uint8_t vcm_dt_cal;   /* 0 idle, 1 running */
static uint8_t vcm_dt_step;
static uint8_t vcm_dt_phase;
static uint16_t vcm_dt_tick;
static uint16_t vcm_dt_nsamp;
static float vcm_dt_sum_m[4];
static float vcm_dt_sum_i[4];
static uint16_t vcm_dt_saved_ns;
static uint8_t vcm_dt_saved_mode;
static uint8_t vcm_dt_saved_ov;
static uint8_t vcm_dt_saved_hold;
static float vcm_dt_saved_iref;
static volatile uint8_t vcm_dt_msg; /* 1 = result, 2 = abort, 3 = fit */
static volatile int32_t vcm_dt_ns_out;
static volatile int32_t vcm_dt_ns_pos;
static volatile int32_t vcm_dt_ns_neg;
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

void VCM_PlantSyncFf(void)
{
  float vb = g_vcm.vbus_v;
  if (vb < 1.0f)
  {
    vb = 1.0f;
  }
  g_vcm.ff_mod_per_a = VCM_FF_SCALE * (g_vcm.coil_r_ohm + VCM_IFB_RS_OHM) /
                       (2.0f * vb);
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
  vcm_dt_sign = 0;
  vcm_base_ok = 0U;
  vcm_half_done = 0U;
  vcm_rep_closed = 0U;
  vcm_next_valid = 0U;
  vcm_act_valid = 0U;
  vcm_run_bip = 1U;
  vcm_gz_hold = 0U;
  vcm_gz_seed = 0U;
  vcm_gz_tick = 0U;
  vcm_gz_s = 1.0f;
  g_vcm.gz_hold = 0U;
  {
    uint32_t i;
    for (i = 0U; i < 4U; i++)
    {
      vcm_notch[i].hz = 0xFFFFU;
      vcm_notch[i].x1 = 0.0f;
      vcm_notch[i].x2 = 0.0f;
      vcm_notch[i].y1 = 0.0f;
      vcm_notch[i].y2 = 0.0f;
    }
  }
  g_vcm.ifb_avg_a = 0.0f;
  g_vcm.ifb_hat_a = 0.0f;
  g_vcm.ifb_filt_a = 0.0f;
  g_vcm.ifb_ip_a = 0.0f;
  g_vcm.ifb_im_a = 0.0f;
  vcm_e_hat = 0.0f;
  vcm_e_apply = 0.0f;
  vcm_e_innov_lp = 0.0f;
  vcm_kp_fdbk_lp = 0.0f;
}

/*
 * Luenberger observer: voltage model from last applied m, correct with 2-pt IFB.
 * Skip the innovation if this period's pair is missing (predict only).
 * ê is the Ke·v term missing from that model. It is applied in VCM_PiStep
 * so the integrator does not have to store it through a stop.
 */
static void VCM_IobsUpdate(float ifb, uint8_t meas_ok)
{
  float i = g_vcm.ifb_hat_a;
  float a = g_vcm.i_obs_a;
  float l = g_vcm.coil_l_h;
  float vb = g_vcm.vbus_v;
  float r = g_vcm.coil_r_ohm + VCM_IFB_RS_OHM;
  float v;
  float ts_l;
  float i_pred;

  if (l < 1.0e-4f)
  {
    l = 1.0e-4f;
  }
  if (vb < 1.0f)
  {
    vb = 1.0f;
  }
  v = 2.0f * g_vcm.mod * vb;
#if (VCM_E_OBS_EN != 0U)
  v -= vcm_e_apply;
#endif
  ts_l = VCM_PWM_TS_S / l;
  i_pred = i + (ts_l * (v - (r * i)));
  if (meas_ok != 0U)
  {
    float innov = ifb - i_pred;

    if (a < 0.0f)
    {
      a = 0.0f;
    }
    if (a > 1.0f)
    {
      a = 1.0f;
    }
#if (VCM_E_OBS_EN != 0U)
    if ((g_vcm.pwm_armed != 0U) && (g_vcm.iref_hold_en == 0U))
    {
      float e_max;

      vcm_e_innov_lp += VCM_E_OBS_IN_A * (innov - vcm_e_innov_lp);
      if (VCM_Absf(g_vcm.iref_a) < VCM_E_OBS_I_MIN)
      {
        vcm_e_hat -= VCM_E_OBS_BLEED_A * vcm_e_hat;
        vcm_e_apply = 0.0f;
      }
      else
      {
        /* innov ≈ (Ts/L)·(e_hat − Ke·v). Subtracting pulls ê onto Ke·v. */
        vcm_e_hat -= (6.2831853f * VCM_E_OBS_HZ) * l * vcm_e_innov_lp;
        e_max = 0.90f * vb;
        if (vcm_e_hat > e_max)
        {
          vcm_e_hat = e_max;
        }
        else if (vcm_e_hat < -e_max)
        {
          vcm_e_hat = -e_max;
        }
        vcm_e_apply = vcm_e_hat;
      }
    }
    else
    {
      vcm_e_hat = 0.0f;
      vcm_e_apply = 0.0f;
      vcm_e_innov_lp = 0.0f;
    }
#endif
    i = i_pred + (a * innov);
  }
  else
  {
    i = i_pred;
  }
  if (i > VCM_I_OCP_A)
  {
    i = VCM_I_OCP_A;
  }
  else if (i < -VCM_I_OCP_A)
  {
    i = -VCM_I_OCP_A;
  }
  g_vcm.ifb_hat_a = i;
  g_vcm.ifb_filt_a = i;
}

/* w0 stays below pi/2 for f0 <= 10 kHz at 50 kHz. */
static float VCM_Sin(float x)
{
  float x2 = x * x;
  return x * (1.0f - x2 * (0.16666667f - x2 * (0.008333333f - x2 * 0.0001984127f)));
}

static float VCM_Cos(float x)
{
  float x2 = x * x;
  return 1.0f - x2 * (0.5f - x2 * (0.041666667f - x2 * 0.0013888889f));
}

/* Up to 4 series notches on the current command. 0 Hz bypasses that stage.
 * Q=5. The null sits on the requested Hz at the control sample rate.
 * The current feedback is left alone: notching it moves the squeal to a shoulder. */
static void VCM_NotchDesign(VCM_Notch_t *n, uint16_t hz)
{
  float w0;
  float cw;
  float sw;
  float alpha;
  float a0;

  n->hz = hz;
  n->x1 = 0.0f;
  n->x2 = 0.0f;
  n->y1 = 0.0f;
  n->y2 = 0.0f;
  if ((hz < 50U) || (hz > 10000U))
  {
    n->b0 = 1.0f;
    n->b1 = 0.0f;
    n->b2 = 0.0f;
    n->a1 = 0.0f;
    n->a2 = 0.0f;
    return;
  }
  w0 = 6.2831853f * ((float)hz / (float)VCM_CTRL_FREQ_HZ);
  cw = VCM_Cos(w0);
  sw = VCM_Sin(w0);
  alpha = sw / 10.0f;
  a0 = 1.0f + alpha;
  n->b0 = 1.0f / a0;
  n->b1 = (-2.0f * cw) / a0;
  n->b2 = n->b0;
  n->a1 = n->b1;
  n->a2 = (1.0f - alpha) / a0;
}

static float VCM_NotchRun(VCM_Notch_t *n, float x)
{
  float y;

  if ((n->hz < 50U) || (n->hz > 10000U))
  {
    return x;
  }
  y = (n->b0 * x) + (n->b1 * n->x1) + (n->b2 * n->x2)
      - (n->a1 * n->y1) - (n->a2 * n->y2);
  n->x2 = n->x1;
  n->x1 = x;
  n->y2 = n->y1;
  n->y1 = y;
  return y;
}

static float VCM_IfbNotch(float x)
{
  uint32_t i;

  for (i = 0U; i < 4U; i++)
  {
    if (vcm_notch[i].hz != g_vcm.notch_hz[i])
    {
      VCM_NotchDesign(&vcm_notch[i], g_vcm.notch_hz[i]);
    }
    x = VCM_NotchRun(&vcm_notch[i], x);
  }
  return x;
}

/* Duty of one minimum tri-level pulse. */
static float VCM_TriCmDuty(void)
{
  return ((float)VCM_TRI_CM_NS * (float)VCM_PWM_FREQ_HZ) * 1.0e-9f;
}

static float VCM_ModMax(void)
{
  float coast;
  float mmax;

  if (vcm_run_bip != 0U)
  {
    return VCM_BIP_MOD_MAX;
  }
  coast = (float)VCM_ADC_ZERO_MIN_CNT / (float)VCM_PWM_PERIOD;
  mmax = 0.5f - VCM_TriCmDuty() - coast;
  if (mmax > VCM_MOD_MAX)
  {
    mmax = VCM_MOD_MAX;
  }
  if (mmax < 0.02f)
  {
    mmax = 0.02f;
  }
  return mmax;
}

/*
 * m = s·Kp·e + Ki_state + R·I_ff + L·di/dt_ff
 * s = 1 while the command is moving. After it stays quiet, s = gz_scale
 * on both Kp and Ki. Idle clamp optional. Back-calculation anti-windup.
 */
static float VCM_GainZoneScale(float ref, float err)
{
  float s1;
  float di;
  float adi;

  if (vcm_gz_seed == 0U)
  {
    vcm_gz_ref = ref;
    vcm_gz_ref_z = ref;
    vcm_gz_seed = 1U;
  }
  vcm_gz_ref += VCM_GZ_REF_A * (ref - vcm_gz_ref);
  di = (vcm_gz_ref - vcm_gz_ref_z) * (float)VCM_CTRL_FREQ_HZ;
  vcm_gz_ref_z = vcm_gz_ref;
  adi = VCM_Absf(di);

  if (g_vcm.gz_en == 0U)
  {
    vcm_gz_hold = 0U;
    vcm_gz_tick = 0U;
    s1 = 1.0f;
  }
  else if (vcm_gz_hold != 0U)
  {
    if (adi > VCM_GZ_EXIT_A_S)
    {
      if (vcm_gz_tick < 0xFFFFU)
      {
        vcm_gz_tick++;
      }
      if (vcm_gz_tick >= VCM_GZ_EXIT_TICKS)
      {
        vcm_gz_hold = 0U;
        vcm_gz_tick = 0U;
      }
    }
    else
    {
      vcm_gz_tick = 0U;
    }
    s1 = (vcm_gz_hold != 0U) ? g_vcm.gz_scale : 1.0f;
  }
  else
  {
    if (adi < VCM_GZ_ENTER_A_S)
    {
      if (vcm_gz_tick < 0xFFFFU)
      {
        vcm_gz_tick++;
      }
      if (vcm_gz_tick >= VCM_GZ_ENTER_TICKS)
      {
        vcm_gz_hold = 1U;
        vcm_gz_tick = 0U;
      }
    }
    else
    {
      vcm_gz_tick = 0U;
    }
    s1 = (vcm_gz_hold != 0U) ? g_vcm.gz_scale : 1.0f;
  }

  if (s1 < 0.05f)
  {
    s1 = 0.05f;
  }
  if (s1 > 1.0f)
  {
    s1 = 1.0f;
  }
  if (s1 != vcm_gz_s)
  {
    g_vcm.integral += (vcm_gz_s - s1) * g_vcm.kp * err;
    if (g_vcm.integral > VCM_I_INTEGRAL_LIM)
    {
      g_vcm.integral = VCM_I_INTEGRAL_LIM;
    }
    if (g_vcm.integral < -VCM_I_INTEGRAL_LIM)
    {
      g_vcm.integral = -VCM_I_INTEGRAL_LIM;
    }
    vcm_gz_s = s1;
  }
  g_vcm.gz_hold = ((g_vcm.gz_en != 0U) && (vcm_gz_hold != 0U)) ? 1U : 0U;
  return vcm_gz_s;
}

/*
 * m = Kp*e_p + Ki*∫e_i + R·I_ff + L·di/dt_ff
 * Observer mode: Kp on î, Ki on sampled IFB so Ke·v is trimmed at speed.
 * fdbk=0 passes the same sampled current for both.
 * Back-calculation anti-windup uses the applied modulation.
 */
static float VCM_PiStep(float ref, float fdbk_p, float fdbk_i, float ts, uint8_t full_rate)
{
  float err_p;
  float err_i;
  float m_ff;
  float m_unsat;
  float m;
  float di_dt;
  float kp;
  float ki;
  float m_max;
  float ref_abs = VCM_Absf(ref);

  if (ts < 1.0e-7f)
  {
    ts = 1.0e-7f;
  }

#if VCM_IDLE_CLAMP_EN
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
#else
  g_vcm.pwm_armed = 1U;
  vcm_idle_deb = 0U;
  (void)ref_abs;
  /* UART 'Z' sets iref_hold_en. Live/F writing 0 A still runs PI. */
  if ((g_vcm.iref_hold_en != 0U) &&
      ((g_vcm.iref_override_en == 0U) || (g_vcm.iref_override_a != 0.0f)))
  {
    g_vcm.iref_hold_en = 0U;
  }
  if (g_vcm.iref_hold_en != 0U)
  {
    g_vcm.integral = 0.0f;
    g_vcm.mod_ff = 0.0f;
    if (full_rate != 0U)
    {
      vcm_iref_z1 = ref;
    }
    return 0.0f;
  }
#endif

  /* Kp sees a ~200 Hz copy of î/IFB; Ki keeps the raw sample + deadband. */
  if (full_rate != 0U)
  {
    float a = VCM_KP_FDBK_LPF_A;
    if (a < 0.0f)
    {
      a = 0.0f;
    }
    if (a > 1.0f)
    {
      a = 1.0f;
    }
    vcm_kp_fdbk_lp += a * (fdbk_p - vcm_kp_fdbk_lp);
  }
  err_p = ref - vcm_kp_fdbk_lp;
  err_i = ref - fdbk_i;
  /* Force hold: ADC floor ~±20–40 mA. Do not let Ki wind that into mod. */
  if (VCM_Absf(err_i) < VCM_KI_ERR_DB_A)
  {
    err_i = 0.0f;
  }
  if (full_rate != 0U)
  {
    float zone = VCM_GainZoneScale(ref, err_p);
    kp = g_vcm.kp * zone;
    ki = g_vcm.ki * zone;
  }
  else
  {
    /* Gain-zone ticks stay at 50 kHz. The half step only reuses the scale. */
    kp = g_vcm.kp * vcm_gz_s;
    ki = g_vcm.ki * vcm_gz_s;
  }

  if (full_rate != 0U)
  {
    di_dt = (ref - vcm_iref_z1) * (float)VCM_CTRL_FREQ_HZ;
    vcm_iref_z1 = ref;
  }
  else
  {
    di_dt = 0.0f;
  }
  if (di_dt > VCM_L_FF_DIDT_MAX)
  {
    di_dt = VCM_L_FF_DIDT_MAX;
  }
  else if (di_dt < -VCM_L_FF_DIDT_MAX)
  {
    di_dt = -VCM_L_FF_DIDT_MAX;
  }

  {
    float l_ff;
    float vb = g_vcm.vbus_v;
    if (vb < 1.0f)
    {
      vb = 1.0f;
    }
    l_ff = VCM_L_FF_SCALE * g_vcm.coil_l_h / (2.0f * vb);
    m_ff = (ref * g_vcm.ff_mod_per_a) + (di_dt * l_ff);
#if (VCM_E_OBS_EN != 0U)
    m_ff += vcm_e_apply / (2.0f * vb);
#endif
  }
  g_vcm.mod_ff = m_ff;

  m_unsat = (kp * err_p) + g_vcm.integral + m_ff;
  m_max = VCM_ModMax();
  if (m_unsat > m_max)
  {
    m = m_max;
  }
  else if (m_unsat < -m_max)
  {
    m = -m_max;
  }
  else
  {
    m = m_unsat;
  }

  g_vcm.integral += (ki * err_i * ts) + (m - m_unsat);
  if (g_vcm.integral > VCM_I_INTEGRAL_LIM)
  {
    g_vcm.integral = VCM_I_INTEGRAL_LIM;
  }
  if (g_vcm.integral < -VCM_I_INTEGRAL_LIM)
  {
    g_vcm.integral = -VCM_I_INTEGRAL_LIM;
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

/* Place the S&H window in the middle of zero-vector [z0, z1]. The whole
 * window stays at least 1 µs from both edges. CMP2 is the trigger, which
 * is one S&H time before the hold instant. */
static uint16_t VCM_AdcTrigForZero(uint16_t z0, uint16_t z1)
{
  uint32_t guard = (uint32_t)VCM_ADC_EDGE_GUARD_CNT;
  uint32_t win = (uint32_t)VCM_ADC_SMP_DELAY;
  uint32_t center;
  uint32_t trig;
  uint32_t hold;
  uint32_t min_trig;
  uint32_t max_hold;

  if (z1 <= (uint16_t)(z0 + 2U))
  {
    z1 = (uint16_t)(z0 + 2U);
  }
  center = ((uint32_t)z0 + (uint32_t)z1) / 2U;
  if (win < 2U)
  {
    win = 2U;
  }
  hold = center + (win / 2U);
  trig = hold - win;
  min_trig = (uint32_t)z0 + guard;
  max_hold = (uint32_t)z1 - guard;
  if ((min_trig + win) > max_hold)
  {
    center = ((uint32_t)z0 + (uint32_t)z1) / 2U;
    if (center > win)
    {
      trig = center - (win / 2U);
    }
    else
    {
      trig = 2U;
    }
  }
  else
  {
    if (trig < min_trig)
    {
      trig = min_trig;
    }
    hold = trig + win;
    if (hold > max_hold)
    {
      hold = max_hold;
      trig = hold - win;
    }
  }
  if (trig < 2U)
  {
    trig = 2U;
  }
  if (trig >= (uint32_t)VCM_PWM_PERIOD)
  {
    trig = (uint32_t)VCM_PWM_PERIOD - 2U;
  }
  return (uint16_t)trig;
}

static void VCM_HRTIM_WriteAdcTrigs(uint16_t trig_a, uint16_t trig_b)
{
  uint16_t gap = VCM_ADC_TRIG_GAP;
  /* Second conversion must finish before PER/REP. PERIOD-2 left Tconv hanging
   * across the rollover → NDTR stuck at 1 and pair stayed 0 forever. */
  uint16_t max_b = (uint16_t)(VCM_PWM_PERIOD - VCM_ADC_CONV_GUARD);

  if (gap < 2U)
  {
    gap = 2U;
  }
  if (max_b <= (uint16_t)(2U + gap))
  {
    max_b = (uint16_t)(2U + gap);
  }
  if (trig_b < (uint16_t)(trig_a + gap))
  {
    trig_b = (uint16_t)(trig_a + gap);
  }
  if (trig_b > max_b)
  {
    trig_b = max_b;
  }
  if (((uint32_t)trig_a + (uint32_t)gap) > (uint32_t)trig_b)
  {
    if (trig_b > gap)
    {
      trig_a = (uint16_t)(trig_b - gap);
    }
    else
    {
      trig_a = 2U;
    }
  }
  if (trig_a < 2U)
  {
    trig_a = 2U;
  }
  __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, trig_a);
  __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_4, trig_b);
}

/* Two S&H windows inside one zero vector, split around its center.
 * Bipolar has only the trailing coast; both triggers stay there so the
 * two-deep DMA still completes once per period. */
static void VCM_HRTIM_SetAdcPhase(uint16_t z0, uint16_t z1)
{
  uint16_t trig = VCM_AdcTrigForZero(z0, z1);
  uint16_t half = (uint16_t)(VCM_ADC_TRIG_GAP / 2U);
  uint16_t t0 = (trig > half) ? (uint16_t)(trig - half) : 2U;
  uint16_t t1 = (uint16_t)(trig + (uint16_t)(VCM_ADC_TRIG_GAP - half));

  VCM_HRTIM_WriteAdcTrigs(t0, t1);
}

/* One S&H in each coast. CMP2 is the earlier coast, CMP4 the later one. */
static void VCM_HRTIM_SetAdcPhasePair(uint16_t z0a, uint16_t z1a,
                                      uint16_t z0b, uint16_t z1b)
{
  uint16_t t0 = VCM_AdcTrigForZero(z0a, z1a);
  uint16_t t1 = VCM_AdcTrigForZero(z0b, z1b);
  uint16_t swap;

  if (t0 > t1)
  {
    swap = t0;
    t0 = t1;
    t1 = swap;
  }
  VCM_HRTIM_WriteAdcTrigs(t0, t1);
}

static void VCM_HRTIM_SetAdcPhaseIdle(void)
{
  VCM_HRTIM_SetAdcPhase((uint16_t)(VCM_PWM_PERIOD / 2U), VCM_PWM_PERIOD);
}

int VCM_AdcStart(void)
{
  uint32_t n;

  for (n = 0U; n < VCM_ADC_N; n++)
  {
    vcm_adc1_dma[n] = (uint16_t)VCM_ADC_MID;
  }
  vcm_adc2_dma = (uint16_t)VCM_ADC_MID;

  vcm_pair_seq = 0U;
  vcm_pair_taken = 0U;
  vcm_pair_raw0 = (uint16_t)VCM_ADC_MID;
  vcm_pair_raw1 = (uint16_t)VCM_ADC_MID;

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)vcm_adc1_dma, VCM_ADC_N) != HAL_OK)
  {
    return -1;
  }
  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&vcm_adc2_dma, 1U) != HAL_OK)
  {
    return -1;
  }

  /* ADC1: TC = both coast samples latched. HT unused. ADC2: poll only. */
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(hadc1.DMA_Handle, DMA_IT_TC);
  __HAL_DMA_DISABLE_IT(hadc2.DMA_Handle, DMA_IT_TC | DMA_IT_HT);
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);

  return 0;
}

/* DMA length-2 TC: I+ then I− are both in memory. Do not run PI here —
 * latch only; the 50 kHz REP consumes the snapshot. */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  uint32_t seq;

  if ((hadc == NULL) || (hadc->Instance != ADC1))
  {
    return;
  }
  seq = vcm_pair_seq + 1U;
  vcm_pair_seq = seq; /* odd = copy in progress */
  __DMB();
  vcm_pair_raw0 = vcm_adc1_dma[0];
  vcm_pair_raw1 = vcm_adc1_dma[1];
  __DMB();
  vcm_pair_seq = seq + 1U; /* even = stable */
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

static void VCM_HRTIM_InvalidateChop(void)
{
  vcm_chop_a = 0xFFU;
  vcm_chop_b = 0xFFU;
}

/* HS pulse at start of period: high [0, high_cnt). */
static void VCM_HRTIM_PulseStart(uint32_t timer_idx, uint16_t high_cnt)
{
  HRTIM_Timerx_TypeDef *tim = &hhrtim.Instance->sTimerxRegs[timer_idx];
  tim->SETx1R = HRTIM_SET1R_PER;
  tim->RSTx1R = HRTIM_RST1R_CMP1;
  __HAL_HRTIM_SETCOMPARE(&hhrtim, timer_idx, HRTIM_COMPAREUNIT_1, high_cnt);
}

/* HS pulse at [start, start+high_cnt). Reset on CMP2, never on PER. */
static void VCM_HRTIM_PulseWindow(uint32_t timer_idx, uint16_t start, uint16_t high_cnt)
{
  uint16_t end;
  HRTIM_Timerx_TypeDef *tim = &hhrtim.Instance->sTimerxRegs[timer_idx];
  const uint16_t period = VCM_PWM_PERIOD;

  if (start < 1U)
  {
    start = 1U;
  }
  if (high_cnt < 1U)
  {
    high_cnt = 1U;
  }
  if (((uint32_t)start + (uint32_t)high_cnt) >= (uint32_t)(period - 1U))
  {
    high_cnt = (uint16_t)(period - start - 1U);
  }
  end = (uint16_t)(start + high_cnt);
  tim->SETx1R = HRTIM_SET1R_CMP1;
  tim->RSTx1R = HRTIM_RST1R_CMP2;
  __HAL_HRTIM_SETCOMPARE(&hhrtim, timer_idx, HRTIM_COMPAREUNIT_1, start);
  __HAL_HRTIM_SETCOMPARE(&hhrtim, timer_idx, HRTIM_COMPAREUNIT_2, end);
}

/* High side stays off, so the complementary low side freewheels. */
static void VCM_HRTIM_PulseOff(uint32_t timer_idx)
{
  HRTIM_Timerx_TypeDef *tim = &hhrtim.Instance->sTimerxRegs[timer_idx];

  tim->SETx1R = 0U;
  tim->RSTx1R = HRTIM_RST1R_PER | HRTIM_RST1R_SRT;
}

/* HRTIM counts for the external gate-driver blanking. 500 ns → 240 counts. */
static uint16_t VCM_DrvDtCounts(void)
{
  uint32_t ns = g_vcm.drv_dt_ns;

  if (ns > VCM_DRV_DT_NS_MAX)
  {
    ns = VCM_DRV_DT_NS_MAX;
  }
  return (uint16_t)((ns * (VCM_HRTIM_CLOCK_HZ / 1000000UL)) / 1000UL);
}

/* Keep the last sign through zero so compensation does not flip with noise. */
static int8_t VCM_DrvDtSign(float i_a)
{
  if (i_a > VCM_DRV_DT_HOLD_A)
  {
    vcm_dt_sign = 1;
  }
  else if (i_a < -VCM_DRV_DT_HOLD_A)
  {
    vcm_dt_sign = -1;
  }
  return vcm_dt_sign;
}

/* User bipolar stays bipolar. Tri-level stays tri-level. */
static void VCM_SelectRunMode(float iref)
{
  (void)iref;
  vcm_run_bip = (g_vcm.pwm_mode != VCM_PWM_MODE_TRILEVEL) ? 1U : 0U;
}

/*
 * Three-level: both pulses stay on in closed loop. m lengthens one side.
 * UART Z (open-loop hold) leaves both off.
 * Bipolar 0.5+/-m only if pwm_mode is bipolar.
 */
static void VCM_HRTIM_ApplyDuty(float m)
{
  uint16_t cmp_a;
  uint16_t cmp_b;
  uint16_t cnt_a;
  uint16_t cnt_b;
  uint16_t room;
  uint16_t remain;
  uint16_t start_b;
  float da_f;
  float db_f;
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
    VCM_HRTIM_SetChop(HRTIM_TIMERINDEX_TIMER_A, 0U);
    VCM_HRTIM_SetChop(HRTIM_TIMERINDEX_TIMER_B, 0U);
    cmp_a = duty_min;
    cmp_b = duty_min;
    g_vcm.adc_active_valid = 1U;
    vcm_next_valid = 0U;
  __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
  __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp_b);
    VCM_HRTIM_SetAdcPhase(cmp_a, VCM_PWM_PERIOD);
    return;
  }

  if (vcm_run_bip == 0U)
  {
    float dcm;
    uint16_t dcm_cnt;
    uint16_t coast_min = VCM_ADC_ZERO_MIN_CNT;
    float m_dt = m;

    dcm = (g_vcm.iref_hold_en != 0U) ? 0.0f : VCM_TriCmDuty();
    dcm_cnt = (uint16_t)(dcm * (float)VCM_PWM_PERIOD + 0.5f);
    /* Both pulses stay at least dcm. m lengthens one side. */
    if (m_dt >= 0.0f)
    {
      da_f = dcm + (2.0f * m_dt);
      db_f = dcm;
    }
    else
    {
      da_f = dcm;
      db_f = dcm - (2.0f * m_dt);
    }
    cnt_a = (uint16_t)(da_f * (float)VCM_PWM_PERIOD + 0.5f);
    cnt_b = (uint16_t)(db_f * (float)VCM_PWM_PERIOD + 0.5f);
    if (cnt_a < dcm_cnt)
    {
      cnt_a = dcm_cnt;
    }
    if (cnt_b < dcm_cnt)
    {
      cnt_b = dcm_cnt;
    }
    if (cnt_a > duty_max)
    {
      cnt_a = duty_max;
    }
    if (cnt_b > duty_max)
    {
      cnt_b = duty_max;
    }
    if ((uint32_t)cnt_a + (uint32_t)cnt_b >
        (uint32_t)VCM_PWM_PERIOD - (2U * (uint32_t)coast_min))
    {
      room = (uint16_t)(VCM_PWM_PERIOD - (2U * coast_min));
      if (cnt_a >= cnt_b)
      {
        cnt_b = dcm_cnt;
        cnt_a = (uint16_t)(room - cnt_b);
      }
      else
      {
        cnt_a = dcm_cnt;
        cnt_b = (uint16_t)(room - cnt_a);
      }
    }

    remain = (uint16_t)(VCM_PWM_PERIOD - cnt_a - cnt_b);
    start_b = (uint16_t)(cnt_a + (remain / 2U));
    if (start_b <= cnt_a)
    {
      start_b = (uint16_t)(cnt_a + duty_min);
    }

    VCM_HRTIM_InvalidateChop();
    if (cnt_a > 0U)
    {
      VCM_HRTIM_PulseStart(HRTIM_TIMERINDEX_TIMER_A, cnt_a);
    }
    else
    {
      VCM_HRTIM_PulseOff(HRTIM_TIMERINDEX_TIMER_A);
    }
    if (cnt_b > 0U)
    {
      VCM_HRTIM_PulseWindow(HRTIM_TIMERINDEX_TIMER_B, start_b, cnt_b);
    }
    else
    {
      VCM_HRTIM_PulseOff(HRTIM_TIMERINDEX_TIMER_B);
    }
    g_vcm.adc_active_valid = 1U;
    {
      uint16_t z_hi0 = cnt_a;
      uint16_t z_hi1 = start_b;
      uint16_t z_lo0 = (uint16_t)(start_b + cnt_b);
      uint16_t z_lo1 = VCM_PWM_PERIOD;

      if (z_hi1 <= z_hi0)
      {
        z_hi1 = (uint16_t)(z_hi0 + 2U);
      }
      if (z_lo0 >= VCM_PWM_PERIOD)
      {
        z_lo0 = (uint16_t)(VCM_PWM_PERIOD - 2U);
      }
      VCM_HRTIM_SetAdcPhasePair(z_hi0, z_hi1, z_lo0, z_lo1);
      {
        uint16_t t0 = VCM_AdcTrigForZero(z_hi0, z_hi1);
        uint32_t cmp3 = (uint32_t)t0 + (uint32_t)VCM_ADC_CONV_GUARD;

        if (cmp3 >= (uint32_t)(VCM_PWM_PERIOD - 2U))
        {
          cmp3 = (uint32_t)VCM_PWM_PERIOD - 2U;
        }
        if (cmp3 < 2U)
        {
          cmp3 = 2U;
        }
        __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_A,
                               HRTIM_COMPAREUNIT_3, (uint16_t)cmp3);
        vcm_next_cmp3 = (uint16_t)cmp3;
      }
    }
    vcm_next_cnt_a = cnt_a;
    vcm_next_start_b = start_b;
    vcm_next_dcm = dcm_cnt;
    vcm_next_b1 = (uint16_t)__HAL_HRTIM_GETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_B,
                                                   HRTIM_COMPAREUNIT_1);
    vcm_next_b2 = (uint16_t)__HAL_HRTIM_GETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_B,
                                                   HRTIM_COMPAREUNIT_2);
    vcm_next_valid = (cnt_b > 0U) ? 1U : 0U;
    return;
  }

  vcm_next_valid = 0U;

  /* Leg difference = 2·m·T + sign(i)·dt. dt is the two-edge compensation. */
  {
    int32_t diff;
    int32_t half;
    int32_t ca;
    int32_t cb;
    float rounded = (2.0f * m * (float)VCM_PWM_PERIOD) + ((m >= 0.0f) ? 0.5f : -0.5f);

    diff = (int32_t)rounded;
    diff += (int32_t)VCM_DrvDtSign(g_vcm.ifb_a) * (int32_t)VCM_DrvDtCounts();
    half = (int32_t)(VCM_PWM_PERIOD / 2U);
    ca = half + (diff / 2);
    cb = half - (diff - (diff / 2));
    if (ca < (int32_t)duty_min)
    {
      ca = (int32_t)duty_min;
    }
    if (cb < (int32_t)duty_min)
    {
      cb = (int32_t)duty_min;
    }
    if (ca > (int32_t)duty_max)
    {
      ca = (int32_t)duty_max;
    }
    if (cb > (int32_t)duty_max)
    {
      cb = (int32_t)duty_max;
    }
    cmp_a = (uint16_t)ca;
    cmp_b = (uint16_t)cb;
  }

  VCM_HRTIM_InvalidateChop();
  VCM_HRTIM_SetChop(HRTIM_TIMERINDEX_TIMER_A, 1U);
  VCM_HRTIM_SetChop(HRTIM_TIMERINDEX_TIMER_B, 1U);

  g_vcm.adc_active_valid = 1U;
  __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
  __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, cmp_b);
  {
    uint16_t z0 = (cmp_a > cmp_b) ? cmp_a : cmp_b;
    VCM_HRTIM_SetAdcPhase(z0, VCM_PWM_PERIOD);
  }
}

static float VCM_DtCalAmp(uint8_t step)
{
  if (step == 0U)
  {
    return VCM_DT_CAL_I1_A;
  }
  if (step == 1U)
  {
    return VCM_DT_CAL_I2_A;
  }
  if (step == 2U)
  {
    return -VCM_DT_CAL_I1_A;
  }
  return -VCM_DT_CAL_I2_A;
}

static void VCM_DtCalRestore(uint8_t use_result, uint16_t ns)
{
  g_vcm.pwm_mode = vcm_dt_saved_mode;
  g_vcm.iref_override_en = vcm_dt_saved_ov;
  g_vcm.iref_hold_en = vcm_dt_saved_hold;
  g_vcm.iref_override_a = vcm_dt_saved_iref;
  g_vcm.drv_dt_ns = (use_result != 0U) ? ns : vcm_dt_saved_ns;
  vcm_dt_cal = 0U;
  VCM_HRTIM_SetAdcPhaseIdle();
}

static void VCM_DtCalAbort(void)
{
  if (vcm_dt_cal == 0U)
  {
    return;
  }
  VCM_DtCalRestore(0U, 0U);
  vcm_dt_msg = 2U;
}

static int32_t VCM_DtCalNs(float b)
{
  float ns = VCM_Absf(b) * (2.0f * VCM_PWM_TS_S) * 1.0e9f;
  if (ns < 0.0f)
  {
    ns = 0.0f;
  }
  if (ns > (float)VCM_DRV_DT_NS_MAX)
  {
    ns = (float)VCM_DRV_DT_NS_MAX;
  }
  return (int32_t)(ns + 0.5f);
}

static void VCM_DtCalFinish(void)
{
  float ninv = 1.0f / (float)VCM_DT_CAL_AVG;
  float m0 = vcm_dt_sum_m[0] * ninv;
  float i0 = vcm_dt_sum_i[0] * ninv;
  float m1 = vcm_dt_sum_m[1] * ninv;
  float i1 = vcm_dt_sum_i[1] * ninv;
  float m2 = vcm_dt_sum_m[2] * ninv;
  float i2 = vcm_dt_sum_i[2] * ninv;
  float m3 = vcm_dt_sum_m[3] * ninv;
  float i3 = vcm_dt_sum_i[3] * ninv;
  float d_pos = i1 - i0;
  float d_neg = i3 - i2;
  float b_pos;
  float b_neg;
  int32_t ns_pos;
  int32_t ns_neg;
  int32_t ns;

  if ((VCM_Absf(d_pos) < 0.05f) || (VCM_Absf(d_neg) < 0.05f) ||
      (VCM_Absf(m0) > 0.40f) || (VCM_Absf(m1) > 0.40f) ||
      (VCM_Absf(m2) > 0.40f) || (VCM_Absf(m3) > 0.40f) ||
      (VCM_Absf(i0) < 0.05f) || (VCM_Absf(i1) < 0.05f) ||
      (VCM_Absf(i2) < 0.05f) || (VCM_Absf(i3) < 0.05f))
  {
    VCM_DtCalRestore(0U, 0U);
    vcm_dt_msg = 3U;
    return;
  }

  b_pos = ((m0 * i1) - (m1 * i0)) / d_pos;
  b_neg = ((m2 * i3) - (m3 * i2)) / d_neg;
  ns_pos = VCM_DtCalNs(b_pos);
  ns_neg = VCM_DtCalNs(b_neg);
  ns = (ns_pos + ns_neg + 1) / 2;
  if (ns > (int32_t)VCM_DRV_DT_NS_MAX)
  {
    ns = (int32_t)VCM_DRV_DT_NS_MAX;
  }
  vcm_dt_ns_pos = ns_pos;
  vcm_dt_ns_neg = ns_neg;
  vcm_dt_ns_out = ns;
  VCM_DtCalRestore(1U, (uint16_t)ns);
  vcm_dt_msg = 1U;
}

static void VCM_DtCalSample(void)
{
  if (vcm_dt_cal == 0U)
  {
    return;
  }
  if (g_vcm.state != VCM_STATE_RUN)
  {
    VCM_DtCalAbort();
    return;
  }

  if (vcm_dt_phase == 0U)
  {
    vcm_dt_tick++;
    if (vcm_dt_tick >= VCM_DT_CAL_SETTLE)
    {
      vcm_dt_phase = 1U;
      vcm_dt_tick = 0U;
      vcm_dt_nsamp = 0U;
      vcm_dt_sum_m[vcm_dt_step] = 0.0f;
      vcm_dt_sum_i[vcm_dt_step] = 0.0f;
    }
    return;
  }

  vcm_dt_tick++;
  if (g_vcm.adc_pair_ok != 0U)
  {
    vcm_dt_sum_m[vcm_dt_step] += g_vcm.mod;
    vcm_dt_sum_i[vcm_dt_step] += g_vcm.ifb_a;
    vcm_dt_nsamp++;
  }
  if (vcm_dt_nsamp >= VCM_DT_CAL_AVG)
  {
    if (vcm_dt_step >= 3U)
    {
      VCM_DtCalFinish();
      return;
    }
    vcm_dt_step++;
    vcm_dt_phase = 0U;
    vcm_dt_tick = 0U;
    g_vcm.iref_override_a = VCM_DtCalAmp(vcm_dt_step);
    return;
  }
  if (vcm_dt_tick >= VCM_DT_CAL_AVG_MAX)
  {
    VCM_DtCalAbort();
  }
}

void VCM_Stop(void)
{
  VCM_DtCalAbort();
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
     * Time-based debounce. The 50 kHz ISR tears the drive down faster (~400 us,
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
    /* Stay in the main loop so a version query can still be answered. */
    g_vcm.fault_flags |= 8U;
    g_vcm.state = VCM_STATE_FAULT;
    return;
  }
  if (HAL_HRTIM_WaveformOutputStart(&hhrtim,
                                    HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                                    HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2) != HAL_OK)
  {
    g_vcm.fault_flags |= 8U;
    g_vcm.state = VCM_STATE_FAULT;
    return;
  }

  vcm_chop_a = 0xFFU;
  vcm_chop_b = 0xFFU;
  VCM_HRTIM_ApplyDuty(0.0f);

#if VCM_ENABLE_CAL_EN
  /*
   * Fast re-enable: the offsets drift slowly with temperature, so a brief enable
   * drop does not justify another 2 ms of m=0. Reuse them and go straight to
   * RUN; set VCM_CAL_REUSE_MS 0 to always recalibrate.
   */
  if ((VCM_CAL_REUSE_MS > 0U) && (g_vcm.calib_valid != 0U) &&
      ((HAL_GetTick() - vcm_cal_done_ms) < (uint32_t)VCM_CAL_REUSE_MS))
  {
    g_vcm.calib_reuse_count++;
    g_vcm.state = VCM_STATE_RUN;
    return;
  }

  g_vcm.state = VCM_STATE_CALIB;
#else
  /* Offsets stay 0 (Init). Close PI immediately so PMAC analog is not open-loop. */
  g_vcm.state = VCM_STATE_RUN;
#endif
}

#if VCM_DIAG_EN
/*
 * Read-only observation: how many ISR ticks (~20 us each @ 50 kHz) the loop needs from an
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
  float i_samp;
  uint16_t raw;

  /* The repetition event already loaded the shadow programmed last period. */
  vcm_act_cnt_a = vcm_next_cnt_a;
  vcm_act_start_b = vcm_next_start_b;
  vcm_act_dcm = vcm_next_dcm;
  vcm_act_cmp3 = vcm_next_cmp3;
  vcm_act_b1 = vcm_next_b1;
  vcm_act_b2 = vcm_next_b2;
  vcm_act_valid = vcm_next_valid;
  vcm_rep_closed = 0U;

  if (vcm_dt_cal != 0U)
  {
    g_vcm.pwm_mode = VCM_PWM_MODE_BIPOLAR;
    g_vcm.drv_dt_ns = 0U;
    g_vcm.iref_override_en = 1U;
    g_vcm.iref_hold_en = 0U;
    g_vcm.iref_override_a = VCM_DtCalAmp(vcm_dt_step);
  }

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
  g_vcm.adc_iref = vcm_adc2_dma;
  {
    uint32_t seq;
    uint16_t raw0;
    uint16_t raw1;
    float i0;
    float i1;

    /* Prefer the DMA-TC snapshot: both coasts finished. NDTR alone races. */
    seq = vcm_pair_seq;
    raw0 = vcm_pair_raw0;
    raw1 = vcm_pair_raw1;
    __DMB();
    if ((seq == vcm_pair_seq) && ((seq & 1U) == 0U) &&
        (seq != 0U) && (seq != vcm_pair_taken))
    {
      vcm_pair_taken = seq;
      raw = raw0;
      g_vcm.adc_ifb = raw0;
      i0 = VCM_AdcToIfbAmpere(raw0) - g_vcm.ifb_offset_a;
      i1 = VCM_AdcToIfbAmpere(raw1) - g_vcm.ifb_offset_a;
      i_samp = 0.5f * (i0 + i1);
      g_vcm.ifb_ip_a = i0;
      g_vcm.ifb_im_a = i1;
      g_vcm.adc_pair_ok = 1U;
      g_vcm.ifb_raw_a = i_samp + g_vcm.ifb_offset_a;
      g_vcm.ifb_a = i_samp;
      g_vcm.ifb_avg_a = i_samp;
    }
    else
    {
      /* Missed TC this period: still publish the latest dma[0] so iavg moves. */
      g_vcm.adc_pair_ok = 0U;
      raw = vcm_adc1_dma[0];
      g_vcm.adc_ifb = raw;
      i0 = VCM_AdcToIfbAmpere(raw) - g_vcm.ifb_offset_a;
      i_samp = i0;
      g_vcm.ifb_raw_a = i_samp + g_vcm.ifb_offset_a;
      g_vcm.ifb_a = i_samp;
      g_vcm.ifb_avg_a = i_samp;
      g_vcm.ifb_ip_a = i0;
    }
  }

  if (g_vcm.iref_override_en != 0U)
  {
    iref = g_vcm.iref_override_a;
  }
  else
  {
    iref = VCM_AdcToIrefAmpere(g_vcm.adc_iref) - g_vcm.iref_offset_a;
  }
  if (iref > VCM_I_MAX_A)
  {
    iref = VCM_I_MAX_A;
  }
  if (iref < -VCM_I_MAX_A)
  {
    iref = -VCM_I_MAX_A;
  }
  /* Live/UART F: 1st-order matching analog C32/C33. Analog path: no extra LPF. */
  if (g_vcm.iref_override_en != 0U)
  {
    float a = g_vcm.iref_lpf_a;
    if (a < 0.02f)
    {
      a = 0.02f;
    }
    if (a > 1.0f)
    {
      a = 1.0f;
    }
    iref = g_vcm.iref_a + (a * (iref - g_vcm.iref_a));
  }
  g_vcm.iref_a = iref;

  /* Rolling means for UART gain report (no effect on loop). */
  g_vcm.cal_iref_mean += VCM_CAL_AVG_ALPHA * (g_vcm.iref_a - g_vcm.cal_iref_mean);
  g_vcm.cal_ifb_mean += VCM_CAL_AVG_ALPHA * (g_vcm.ifb_avg_a - g_vcm.cal_ifb_mean);

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
    /* Capture must keep recording while the bridge is off, or the host plot stays empty. */
    VCM_CapPush();
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
    VCM_CapPush();
    return;
  }

  {
    if ((i_samp > VCM_I_OCP_A) || (i_samp < -VCM_I_OCP_A))
    {
      if (vcm_ocp_hits < 0xFFFFU)
      {
        vcm_ocp_hits++;
      }
      if (vcm_ocp_hits >= VCM_OCP_CONFIRM_SAMPLES)
      {
        VCM_EnterFault(2U);
        VCM_CapPush();
        return;
      }
    }
    else
    {
      vcm_ocp_hits = 0U;
    }
  }

  /* Bipolar sample is the center of the trailing zero vector, where the
   * current is flat at the ripple peak. Subtract half the inductive ramp
   * to recover the period average. Last period's m is still in g_vcm.mod. */
  if ((vcm_run_bip != 0U) &&
      (g_vcm.pwm_armed != 0U) &&
      (g_vcm.adc_pair_ok != 0U))
  {
    float vb = g_vcm.vbus_v;
    float l = g_vcm.coil_l_h;
    float half;

    if (vb < 1.0f)
    {
      vb = 1.0f;
    }
    if (l < 1.0e-4f)
    {
      l = 1.0e-4f;
    }
    half = vb * VCM_Absf(g_vcm.mod) * VCM_PWM_TS_S / l;
    if (g_vcm.mod < 0.0f)
    {
      half = -half;
    }
    i_samp -= half;
    g_vcm.ifb_a = i_samp;
    g_vcm.ifb_avg_a = i_samp;
  }

  /* Tri-level samples one coast after each pulse. Their mean is the
   * period average, so no peak correction is applied. */

#if VCM_ENABLE_CAL_EN
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
#endif

#if VCM_DIAG_EN
  VCM_DiagService();
#endif

  VCM_IobsUpdate(g_vcm.ifb_avg_a, g_vcm.adc_pair_ok);
  VCM_SelectRunMode(g_vcm.iref_a);
  {
    float ref = VCM_IfbNotch(g_vcm.iref_a);
    float fdbk_p = (g_vcm.pi_fdbk == VCM_PI_FDBK_AVG) ? g_vcm.ifb_avg_a
                                                      : g_vcm.ifb_hat_a;
    float fdbk_i = ((g_vcm.pi_fdbk == VCM_PI_FDBK_AVG) || (g_vcm.adc_pair_ok != 0U))
                       ? g_vcm.ifb_avg_a
                       : fdbk_p;
    float ts = (vcm_half_done != 0U) ? VCM_HALF_TS_S : VCM_PWM_TS_S;

    vcm_half_done = 0U;
    vcm_pi_ref = ref;
    vcm_base_ok = 0U;
    if (g_vcm.adc_pair_ok != 0U)
    {
      vcm_base_i0 = g_vcm.ifb_ip_a;
      vcm_base_avg = g_vcm.ifb_avg_a;
      vcm_base_ok = 1U;
    }
    m = VCM_PiStep(ref, fdbk_p, fdbk_i, ts, 1U);
  }
  g_vcm.mod = m;
  vcm_rep_closed = 1U;
  VCM_HRTIM_ApplyDuty(m);
  VCM_DtCalSample();
  VCM_CapPush();
}

/* −pulse for a +pulse that has already been scheduled. The coasts stay
 * long enough for the two samples. */
static uint8_t VCM_TriSecondPulse(float m_dt, uint16_t cnt_a, uint16_t dcm_cnt,
                                  uint16_t *start_b, uint16_t *cnt_b)
{
  float da;
  float db;
  float dcm;
  uint16_t cb;
  uint16_t sb;
  uint16_t remain;
  uint32_t cb_max;
  const uint16_t coast_min = VCM_ADC_ZERO_MIN_CNT;

  if ((cnt_a < 1U) || (start_b == NULL) || (cnt_b == NULL))
  {
    return 0U;
  }
  da = (float)cnt_a / (float)VCM_PWM_PERIOD;
  dcm = (float)dcm_cnt / (float)VCM_PWM_PERIOD;
  db = da - (2.0f * m_dt);
  if (db < dcm)
  {
    db = dcm;
  }
  cb = (uint16_t)(db * (float)VCM_PWM_PERIOD + 0.5f);
  if (cb < dcm_cnt)
  {
    cb = dcm_cnt;
  }
  if (cb < 1U)
  {
    return 0U;
  }
  if ((uint32_t)VCM_PWM_PERIOD <= ((uint32_t)cnt_a + (2U * (uint32_t)coast_min)))
  {
    return 0U;
  }
  cb_max = (uint32_t)VCM_PWM_PERIOD - (uint32_t)cnt_a - (2U * (uint32_t)coast_min);
  if ((uint32_t)cb > cb_max)
  {
    cb = (uint16_t)cb_max;
  }
  if (cb < dcm_cnt)
  {
    return 0U;
  }
  remain = (uint16_t)(VCM_PWM_PERIOD - cnt_a - cb);
  sb = (uint16_t)(cnt_a + (remain / 2U));
  if ((sb <= cnt_a) || (((uint32_t)sb + (uint32_t)cb) >= (uint32_t)(VCM_PWM_PERIOD - 1U)))
  {
    return 0U;
  }
  *start_b = sb;
  *cnt_b = cb;
  return 1U;
}

/* Push Timer B's −pulse into the active registers now. The preload is put
 * back afterwards so the next master update still loads the following period. */
static void VCM_TimerBApplyNow(uint16_t start, uint16_t high_cnt)
{
  HRTIM_Timerx_TypeDef *tim = &hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B];
  uint32_t cr = tim->TIMxCR;
  uint16_t end = (uint16_t)(start + high_cnt);

  tim->TIMxCR = cr & ~HRTIM_TIMCR_PREEN;
  (void)tim->TIMxCR;
  tim->CMP1xR = start;
  tim->CMP2xR = end;
  tim->TIMxCR = cr | HRTIM_TIMCR_PREEN;
  (void)tim->TIMxCR;
  __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, vcm_next_b1);
  __HAL_HRTIM_SETCOMPARE(&hhrtim, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, vcm_next_b2);
}

void VCM_CurrentLoop_HalfIRQ(void)
{
  uint16_t now;
  uint16_t due;
  uint16_t start_b;
  uint16_t cnt_b;
  float i0;
  float fdbk_p;
  float fdbk_i;
  float m;
  float m_ach;
  float m_back;
  float integ;
  float ff_saved;
  uint16_t end_b;
  const uint16_t late = VCM_ADC_EDGE_GUARD_CNT; /* 1 µs: this CMP3 is stale */
  const uint16_t edge = 96U;                    /* 200 ns: leave the compare match alone */

  if ((vcm_rep_closed == 0U) || (vcm_act_valid == 0U) || (vcm_run_bip != 0U))
  {
    return;
  }
  if ((g_vcm.pwm_armed == 0U) || (g_vcm.iref_hold_en != 0U) || (vcm_dt_cal != 0U))
  {
    return;
  }
  if ((g_vcm.state != VCM_STATE_RUN) || (vcm_base_ok == 0U) || (hadc1.DMA_Handle == NULL))
  {
    return;
  }

  due = vcm_act_cmp3;
  now = (uint16_t)hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CNTxR;
  /* Wrapped into the next period: this flag belongs to the period that ended. */
  if (((uint32_t)now + 2U) < (uint32_t)due)
  {
    return;
  }
  if ((uint32_t)now > ((uint32_t)due + (uint32_t)late))
  {
    return;
  }
  if (((uint32_t)now + (uint32_t)edge) >= (uint32_t)vcm_act_start_b)
  {
    return;
  }
  /* NDTR == 1: the first coast conversion is in dma[0], the second has not started. */
  if (__HAL_DMA_GET_COUNTER(hadc1.DMA_Handle) != 1U)
  {
    return;
  }

  __DSB();
  i0 = VCM_AdcToIfbAmpere(vcm_adc1_dma[0]) - g_vcm.ifb_offset_a;
  fdbk_i = vcm_base_avg + (i0 - vcm_base_i0);
  if (g_vcm.pi_fdbk == VCM_PI_FDBK_AVG)
  {
    fdbk_p = fdbk_i;
  }
  else
  {
    fdbk_p = g_vcm.ifb_hat_a + (i0 - vcm_base_i0);
  }

  integ = g_vcm.integral;
  ff_saved = g_vcm.mod_ff;
  m = VCM_PiStep(vcm_pi_ref, fdbk_p, fdbk_i, VCM_HALF_TS_S, 0U);
  if (VCM_TriSecondPulse(m, vcm_act_cnt_a, vcm_act_dcm, &start_b, &cnt_b) == 0U)
  {
    g_vcm.integral = integ;
    g_vcm.mod_ff = ff_saved;
    return;
  }
  now = (uint16_t)hhrtim.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CNTxR;
  if ((((uint32_t)now + (uint32_t)edge) >= (uint32_t)start_b) ||
      (((uint32_t)now + (uint32_t)edge) >= (uint32_t)vcm_act_start_b))
  {
    g_vcm.integral = integ;
    g_vcm.mod_ff = ff_saved;
    return;
  }

  m_ach = ((float)vcm_act_cnt_a - (float)cnt_b) / (2.0f * (float)VCM_PWM_PERIOD);
  m_back = m_ach;
  {
    float extra = m_back - m;
    float quantum = 1.0f / (float)VCM_PWM_PERIOD;

    if ((extra > quantum) || (extra < -quantum))
    {
      g_vcm.integral += extra;
  if (g_vcm.integral > VCM_I_INTEGRAL_LIM)
  {
    g_vcm.integral = VCM_I_INTEGRAL_LIM;
  }
  if (g_vcm.integral < -VCM_I_INTEGRAL_LIM)
  {
    g_vcm.integral = -VCM_I_INTEGRAL_LIM;
  }
    }
  }
  end_b = (uint16_t)(start_b + cnt_b);
  if ((start_b != vcm_act_b1) || (end_b != vcm_act_b2))
  {
    /* g_vcm.mod stays the 50 kHz command. The observer keeps one 20 µs step. */
    VCM_TimerBApplyNow(start_b, cnt_b);
  }
  vcm_half_done = 1U;
}

void VCM_Init(void)
{
  g_vcm.iref_a = 0.0f;
  g_vcm.ifb_a = 0.0f;
  g_vcm.ifb_avg_a = 0.0f;
  g_vcm.ifb_hat_a = 0.0f;
  g_vcm.ifb_filt_a = 0.0f;
  g_vcm.ifb_ip_a = 0.0f;
  g_vcm.ifb_im_a = 0.0f;
  g_vcm.ifb_raw_a = 0.0f;
  g_vcm.ifb_offset_a = 0.0f;
  g_vcm.adc_pair_ok = 0U;
  vcm_pair_seq = 0U;
  vcm_pair_taken = 0U;
  vcm_pair_raw0 = (uint16_t)VCM_ADC_MID;
  vcm_pair_raw1 = (uint16_t)VCM_ADC_MID;
  g_vcm.iref_offset_a = 0.0f;
  g_vcm.iref_cal_acc = 0.0f;
  g_vcm.pwm_armed = 0U;
  g_vcm.pwm_mode = VCM_PWM_MODE;
  g_vcm.iref_override_en = 0U;
  g_vcm.iref_hold_en = 0U;
  g_vcm.iref_override_a = 0.0f;
  g_vcm.cal_iref_mean = 0.0f;
  g_vcm.cal_ifb_mean = 0.0f;
  g_vcm.mod_ff = 0.0f;
  vcm_ocp_hits = 0U;
  VCM_LoopStateReset();
  g_vcm.kp = VCM_KP;
  g_vcm.ki = VCM_KI;
  g_vcm.ff_mod_per_a = VCM_FF_MOD_PER_A;
  g_vcm.iref_lpf_a = VCM_IREF_OVR_LPF_A;
  g_vcm.i_obs_a = VCM_I_OBS_A;
  g_vcm.vbus_v = VCM_VBUS_V;
  g_vcm.coil_l_h = VCM_COIL_L_H;
  g_vcm.coil_r_ohm = VCM_COIL_R_OHM;
  g_vcm.pi_fdbk = VCM_PI_FDBK_DEFAULT;
  g_vcm.drv_dt_ns = VCM_DRV_DT_NS;
  g_vcm.notch_hz[0] = 0U;
  g_vcm.notch_hz[1] = 0U;
  g_vcm.notch_hz[2] = 0U;
  g_vcm.notch_hz[3] = 0U;
  g_vcm.gz_en = VCM_GZ_EN_DEFAULT;
  g_vcm.gz_scale = VCM_GZ_SCALE_DEFAULT;
  g_vcm.gz_hold = 0U;
  (void)VCM_NvLoad();
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

/* Capture TX holds the main loop for a whole line. The 1-byte RX register
 * overruns in that time, so "$U stop" never arrived and capture kept going. */
#define VCM_UART_RXQ 128U
static uint8_t vcm_uart_rxq[VCM_UART_RXQ];
static uint16_t vcm_uart_rx_w;
static uint16_t vcm_uart_rx_r;
/* Bytes seen before the host parser is ready (UID blink). 0x07 is answered
 * immediately; everything else is replayed on the normal path. */
static uint8_t vcm_uart_hold[16];
static uint8_t vcm_uart_hold_n;
static uint8_t vcm_uart_hold_r;

static void VCM_UartRxPull(void)
{
  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != 0U)
  {
    uint8_t b = (uint8_t)(huart1.Instance->RDR & 0xFFU);
    uint16_t next = (uint16_t)((vcm_uart_rx_w + 1U) % VCM_UART_RXQ);

    if (next != vcm_uart_rx_r)
    {
      vcm_uart_rxq[vcm_uart_rx_w] = b;
      vcm_uart_rx_w = next;
    }
  }
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != 0U)
  {
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
  }
}

static uint8_t VCM_UartPopHw(uint8_t *out)
{
  uint32_t cr1 = huart1.Instance->CR1;

  huart1.Instance->CR1 = cr1 & ~USART_CR1_RXNEIE;
  VCM_UartRxPull();
  huart1.Instance->CR1 = cr1;
  if (vcm_uart_rx_r == vcm_uart_rx_w)
  {
    return 0U;
  }
  *out = vcm_uart_rxq[vcm_uart_rx_r];
  vcm_uart_rx_r = (uint16_t)((vcm_uart_rx_r + 1U) % VCM_UART_RXQ);
  return 1U;
}

uint8_t VCM_UartRxPop(uint8_t *out)
{
  if (vcm_uart_hold_r < vcm_uart_hold_n)
  {
    *out = vcm_uart_hold[vcm_uart_hold_r];
    vcm_uart_hold_r++;
    if (vcm_uart_hold_r >= vcm_uart_hold_n)
    {
      vcm_uart_hold_r = 0U;
      vcm_uart_hold_n = 0U;
    }
    return 1U;
  }
  return VCM_UartPopHw(out);
}

void VCM_UartRxISR(void)
{
  VCM_UartRxPull();
}

void VCM_UartRxArm(void)
{
  if (huart1.Instance == NULL)
  {
    return;
  }
  HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  SET_BIT(huart1.Instance->CR1, USART_CR1_RXNEIE);
}

/* IAP tool sends 0x07 and waits for one version line. */
void VCM_PollVersionQuery(void)
{
  uint8_t b;

  while (VCM_UartPopHw(&b) != 0U)
  {
    if (b == 0x07U)
    {
      VCM_UartWrite("1.");
      VCM_UartWriteI32((int32_t)VCM_FW_VERSION_MAJOR);
      VCM_UartWrite(".");
      VCM_UartWriteI32((int32_t)VCM_FW_VERSION_MINOR);
      VCM_UartWrite(".");
      VCM_UartWriteI32((int32_t)VCM_FW_VERSION_PATCH);
      VCM_UartWrite("\r\n");
      VCM_UartFlush();
    }
    else if (vcm_uart_hold_n < (uint8_t)sizeof(vcm_uart_hold))
    {
      vcm_uart_hold[vcm_uart_hold_n] = b;
      vcm_uart_hold_n++;
    }
  }
}

static void VCM_UartWrite(const char *s)
{
  while (*s != '\0')
  {
    uint32_t spins = 2000000U;
    while (((huart1.Instance->ISR & USART_ISR_TXE_TXFNF) == 0U) && (spins > 0U))
    {
      VCM_UartRxPull();
      spins--;
    }
    if (spins == 0U)
    {
      return;
    }
    huart1.Instance->TDR = (uint32_t)((uint8_t)*s);
    s++;
  }
}

static void VCM_UartFlush(void)
{
  uint32_t spins = 2000000U;

  while (((huart1.Instance->ISR & USART_ISR_TC) == 0U) && (spins > 0U))
  {
    VCM_UartRxPull();
    spins--;
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

static void VCM_UartWriteMicro(float a)
{
  int32_t u = (int32_t)((a * 1000000.0f) + ((a >= 0.0f) ? 0.5f : -0.5f));
  VCM_UartWriteI32(u);
}

static float VCM_Clampf(float x, float lo, float hi)
{
  if (x < lo)
  {
    return lo;
  }
  if (x > hi)
  {
    return hi;
  }
  return x;
}

static int32_t VCM_ParseI32(const char **pp, const char *end)
{
  const char *p = *pp;
  int32_t s = 1;
  int32_t v = 0;

  if (p >= end)
  {
    return 0;
  }
  if (*p == '-')
  {
    s = -1;
    p++;
  }
  else if (*p == '+')
  {
    p++;
  }
  while ((p < end) && (*p >= '0') && (*p <= '9'))
  {
    v = (v * 10) + (int32_t)(*p - '0');
    p++;
  }
  *pp = p;
  return s * v;
}

static void VCM_HostTelem(void)
{
  VCM_UartWrite("TL iref_ma=");
  VCM_UartWriteMilli(g_vcm.iref_a);
  VCM_UartWrite(" ihat_ma=");
  VCM_UartWriteMilli(g_vcm.ifb_hat_a);
  VCM_UartWrite(" iavg_ma=");
  VCM_UartWriteMilli(g_vcm.ifb_avg_a);
  VCM_UartWrite(" ip_ma=");
  VCM_UartWriteMilli(g_vcm.ifb_ip_a);
  VCM_UartWrite(" im_ma=");
  VCM_UartWriteMilli(g_vcm.ifb_im_a);
  VCM_UartWrite(" mod_e6=");
  VCM_UartWriteMicro(g_vcm.mod);
  VCM_UartWrite(" ff_e6=");
  VCM_UartWriteMicro(g_vcm.mod_ff);
  VCM_UartWrite(" ev_mv=");
  VCM_UartWriteMilli(vcm_e_hat);
  VCM_UartWrite(" kp_e3=");
  VCM_UartWriteMilli(g_vcm.kp);
  VCM_UartWrite(" ki_e3=");
  VCM_UartWriteMilli(g_vcm.ki);
  VCM_UartWrite(" obs_e3=");
  VCM_UartWriteMilli(g_vcm.i_obs_a);
  VCM_UartWrite(" lpf_e3=");
  VCM_UartWriteMilli(g_vcm.iref_lpf_a);
  VCM_UartWrite(" fdbk=");
  VCM_UartWriteI32((int32_t)g_vcm.pi_fdbk);
  VCM_UartWrite(" gz=");
  VCM_UartWriteI32((int32_t)g_vcm.gz_en);
  VCM_UartWrite(" gzh=");
  VCM_UartWriteI32((int32_t)g_vcm.gz_hold);
  VCM_UartWrite(" ov=");
  VCM_UartWriteI32((int32_t)g_vcm.iref_override_en);
  VCM_UartWrite(" hold=");
  VCM_UartWriteI32((int32_t)g_vcm.iref_hold_en);
  VCM_UartWrite(" armed=");
  VCM_UartWriteI32((int32_t)g_vcm.pwm_armed);
  VCM_UartWrite(" mode=");
  VCM_UartWriteI32((int32_t)g_vcm.pwm_mode);
  VCM_UartWrite(" state=");
  VCM_UartWriteI32((int32_t)g_vcm.state);
  VCM_UartWrite(" fault=");
  VCM_UartWriteI32((int32_t)g_vcm.fault_flags);
  VCM_UartWrite(" pair=");
  VCM_UartWriteI32((int32_t)g_vcm.adc_pair_ok);
  VCM_UartWrite(" ndtr=");
  VCM_UartWriteI32((hadc1.DMA_Handle != NULL)
                       ? (int32_t)__HAL_DMA_GET_COUNTER(hadc1.DMA_Handle)
                       : -1);
  VCM_UartWrite(" adc_ifb=");
  VCM_UartWriteI32((int32_t)g_vcm.adc_ifb);
  VCM_UartWrite(" adc_iref=");
  VCM_UartWriteI32((int32_t)g_vcm.adc_iref);
  VCM_UartWrite(" ovr1=");
  VCM_UartWriteI32((int32_t)g_vcm.adc1_ovr_cnt);
  VCM_UartWrite(" ovr2=");
  VCM_UartWriteI32((int32_t)g_vcm.adc2_ovr_cnt);
  VCM_UartWrite(" ticks=");
  VCM_UartWriteI32((int32_t)g_vcm.isr_ticks);
  VCM_UartWrite("\r\n");
}

static void VCM_HostWriteVersion(void)
{
  VCM_UartWrite("hw=");
  VCM_UartWriteI32((int32_t)VCM_HW_VERSION);
  VCM_UartWrite(" fw=");
  VCM_UartWriteI32((int32_t)VCM_FW_VERSION_MAJOR);
  VCM_UartWrite(".");
  VCM_UartWriteI32((int32_t)VCM_FW_VERSION_MINOR);
  VCM_UartWrite(".");
  VCM_UartWriteI32((int32_t)VCM_FW_VERSION_PATCH);
}

void VCM_BootAnnounce(void)
{
  if (huart1.Instance == NULL)
  {
    return;
  }
  VCM_UartWrite("1.");
  VCM_UartWriteI32((int32_t)VCM_FW_VERSION_MAJOR);
  VCM_UartWrite(".");
  VCM_UartWriteI32((int32_t)VCM_FW_VERSION_MINOR);
  VCM_UartWrite(".");
  VCM_UartWriteI32((int32_t)VCM_FW_VERSION_PATCH);
  VCM_UartWrite("\r\n");
  VCM_UartFlush();
}

static void VCM_HostDump(void)
{
  VCM_UartWrite("PR ");
  VCM_HostWriteVersion();
  VCM_UartWrite(" proto=");
  VCM_UartWriteI32((int32_t)VCM_HOST_PROTO);
  VCM_UartWrite(" pwm=");
  VCM_UartWriteI32((int32_t)VCM_PWM_FREQ_HZ);
  VCM_UartWrite(" imax_ma=");
  VCM_UartWriteMilli(VCM_I_MAX_A);
  VCM_UartWrite(" kp_e3=");
  VCM_UartWriteMilli(g_vcm.kp);
  VCM_UartWrite(" ki_e3=");
  VCM_UartWriteMilli(g_vcm.ki);
  VCM_UartWrite(" obs_e3=");
  VCM_UartWriteMilli(g_vcm.i_obs_a);
  VCM_UartWrite(" lpf_e3=");
  VCM_UartWriteMilli(g_vcm.iref_lpf_a);
  VCM_UartWrite(" ff_e6=");
  VCM_UartWriteMicro(g_vcm.ff_mod_per_a);
  VCM_UartWrite(" vbus_e3=");
  VCM_UartWriteMilli(g_vcm.vbus_v);
  VCM_UartWrite(" r_e3=");
  VCM_UartWriteMilli(g_vcm.coil_r_ohm);
  VCM_UartWrite(" l_e6=");
  VCM_UartWriteMicro(g_vcm.coil_l_h);
  VCM_UartWrite(" fdbk=");
  VCM_UartWriteI32((int32_t)g_vcm.pi_fdbk);
  VCM_UartWrite(" dt_ns=");
  VCM_UartWriteI32((int32_t)g_vcm.drv_dt_ns);
  VCM_UartWrite(" n0=");
  VCM_UartWriteI32((int32_t)g_vcm.notch_hz[0]);
  VCM_UartWrite(" n1=");
  VCM_UartWriteI32((int32_t)g_vcm.notch_hz[1]);
  VCM_UartWrite(" n2=");
  VCM_UartWriteI32((int32_t)g_vcm.notch_hz[2]);
  VCM_UartWrite(" n3=");
  VCM_UartWriteI32((int32_t)g_vcm.notch_hz[3]);
  VCM_UartWrite(" gz=");
  VCM_UartWriteI32((int32_t)g_vcm.gz_en);
  VCM_UartWrite(" gs_e3=");
  VCM_UartWriteMilli(g_vcm.gz_scale);
  VCM_UartWrite(" mode=");
  VCM_UartWriteI32((int32_t)g_vcm.pwm_mode);
  VCM_UartWrite(" cal_ma=");
  VCM_UartWriteMilli(VCM_CAL_FORCE_A);
  VCM_UartWrite(" nv=");
  VCM_UartWriteI32(VCM_NvIsValid());
  VCM_UartWrite("\r\n");
}

static void VCM_HostIrefMa(int32_t ma)
{
  float a = (float)ma * 0.001f;
  a = VCM_Clampf(a, -VCM_I_MAX_A, VCM_I_MAX_A);
  g_vcm.iref_override_a = a;
  g_vcm.iref_override_en = 1U;
  g_vcm.iref_hold_en = 0U;
}

static void VCM_HostSet(const char *p, const char *end)
{
  uint8_t nset = 0U;
  uint8_t plant = 0U;
  uint8_t ff_set = 0U;

  while (p < end)
  {
    const char *key;
    uint8_t klen = 0U;
    int32_t v;

    while ((p < end) && ((*p == ' ') || (*p == '\t')))
    {
      p++;
    }
    if (p >= end)
    {
      break;
    }
    key = p;
    while ((p < end) && (*p != '=') && (*p != ' '))
    {
      p++;
      klen++;
    }
    if ((p >= end) || (*p != '='))
    {
      break;
    }
    p++;
    v = VCM_ParseI32(&p, end);

    if ((klen == 2U) && (key[0] == 'k') && (key[1] == 'p'))
    {
      g_vcm.kp = VCM_Clampf((float)v * 0.001f, 0.0f, 5.0f);
      nset++;
    }
    else if ((klen == 2U) && (key[0] == 'k') && (key[1] == 'i'))
    {
      g_vcm.ki = VCM_Clampf((float)v * 0.001f, 0.0f, 20000.0f);
      nset++;
    }
    else if ((klen == 3U) && (key[0] == 'o') && (key[1] == 'b') && (key[2] == 's'))
    {
      g_vcm.i_obs_a = VCM_Clampf((float)v * 0.001f, 0.0f, 1.0f);
      nset++;
    }
    else if ((klen == 3U) && (key[0] == 'l') && (key[1] == 'p') && (key[2] == 'f'))
    {
      g_vcm.iref_lpf_a = VCM_Clampf((float)v * 0.001f, 0.02f, 1.0f);
      nset++;
    }
    else if ((klen == 2U) && (key[0] == 'f') && (key[1] == 'f'))
    {
      g_vcm.ff_mod_per_a = VCM_Clampf((float)v * 0.000001f, 0.0f, 0.20f);
      ff_set = 1U;
      nset++;
    }
    else if ((klen == 4U) && (key[0] == 'v') && (key[1] == 'b') &&
             (key[2] == 'u') && (key[3] == 's'))
    {
      g_vcm.vbus_v = VCM_Clampf((float)v * 0.001f, VCM_VBUS_MIN_V, VCM_VBUS_MAX_V);
      plant = 1U;
      nset++;
    }
    else if ((klen == 1U) && (key[0] == 'r'))
    {
      g_vcm.coil_r_ohm = VCM_Clampf((float)v * 0.001f, 0.10f, 50.0f);
      plant = 1U;
      nset++;
    }
    else if ((klen == 1U) && (key[0] == 'l'))
    {
      g_vcm.coil_l_h = VCM_Clampf((float)v * 0.000001f, 0.00010f, 0.020f);
      plant = 1U;
      nset++;
    }
    else if ((klen == 4U) && (key[0] == 'f') && (key[1] == 'd') &&
             (key[2] == 'b') && (key[3] == 'k'))
    {
      g_vcm.pi_fdbk = (v != 0) ? VCM_PI_FDBK_OBS : VCM_PI_FDBK_AVG;
      nset++;
    }
    else if ((klen == 2U) && (key[0] == 'd') && (key[1] == 't'))
    {
      if (v < 0)
      {
        v = 0;
      }
      if (v > (int32_t)VCM_DRV_DT_NS_MAX)
      {
        v = (int32_t)VCM_DRV_DT_NS_MAX;
      }
      g_vcm.drv_dt_ns = (uint16_t)v;
      nset++;
    }
    else if ((klen == 2U) && (key[0] == 'n') &&
             (key[1] >= '0') && (key[1] <= '3'))
    {
      uint8_t idx = (uint8_t)(key[1] - '0');
      if (v < 0)
      {
        v = 0;
      }
      if ((v > 0) && (v < 50))
      {
        v = 50;
      }
      if (v > 10000)
      {
        v = 10000;
      }
      g_vcm.notch_hz[idx] = (uint16_t)v;
      nset++;
    }
    else if ((klen == 2U) && (key[0] == 'g') && (key[1] == 'z'))
    {
      g_vcm.gz_en = (v != 0) ? 1U : 0U;
      nset++;
    }
    else if ((klen == 2U) && (key[0] == 'g') && (key[1] == 's'))
    {
      g_vcm.gz_scale = VCM_Clampf((float)v * 0.001f, 0.05f, 1.0f);
      nset++;
    }
    else if ((klen == 4U) && (key[0] == 'i') && (key[1] == 'r') &&
             (key[2] == 'e') && (key[3] == 'f'))
    {
      VCM_HostIrefMa(v);
      nset++;
    }
  }

  if ((plant != 0U) && (ff_set == 0U))
  {
    VCM_PlantSyncFf();
  }

  VCM_UartWrite((nset != 0U) ? "OK\r\n" : "ERR\r\n");
}

/* PLL2Q = 96 MHz, oversampling 16. Highest legal rate is fck/16 = 6000000. */
static int VCM_BaudAllowed(uint32_t baud)
{
  return (baud == 115200U) || (baud == 230400U) || (baud == 460800U) ||
         (baud == 921600U) || (baud == 1500000U) || (baud == 2000000U) ||
         (baud == 3000000U) || (baud == 6000000U);
}

static int VCM_UartApplyBaud(uint32_t baud)
{
  if (VCM_BaudAllowed(baud) == 0)
  {
    return -1;
  }
  VCM_UartWrite("BAUD ");
  VCM_UartWriteI32((int32_t)baud);
  VCM_UartWrite("\r\n");
  VCM_UartFlush();
  if (huart1.Init.BaudRate == baud)
  {
    return 0;
  }
  huart1.Init.BaudRate = baud;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    return -1;
  }
  return 0;
}

static void VCM_HostLine(char *line, uint16_t n)
{
  char cmd;
  const char *p;
  const char *end = line + n;

  if ((n < 2U) || (line[0] != '$'))
  {
    return;
  }
  cmd = line[1];
  p = line + 2;
  while ((p < end) && ((*p == ' ') || (*p == '\t')))
  {
    p++;
  }

  if (cmd == 'P')
  {
    VCM_HostTelem();
    return;
  }
  if (cmd == 'D')
  {
    VCM_HostDump();
    return;
  }
  if (cmd == 'I')
  {
    VCM_UartWrite("ID VCM_H743 ");
    VCM_HostWriteVersion();
    VCM_UartWrite(" proto=");
    VCM_UartWriteI32((int32_t)VCM_HOST_PROTO);
    VCM_UartWrite(" pwm=");
    VCM_UartWriteI32((int32_t)VCM_PWM_FREQ_HZ);
    VCM_UartWrite(" nv=ram\r\n");
    return;
  }
  if (cmd == 'S')
  {
    VCM_HostSet(p, end);
    return;
  }
  if (cmd == 'R')
  {
    int32_t ma = VCM_ParseI32(&p, end);
    VCM_HostIrefMa(ma);
    VCM_UartWrite("OK\r\n");
    return;
  }
  if (cmd == 'C')
  {
    VCM_ClearFault();
    VCM_UartWrite("OK\r\n");
    return;
  }
  if (cmd == 'W')
  {
    vcm_nv_req = 1U;
    VCM_UartWrite("NV BUSY\r\n");
    return;
  }
  if (cmd == 'E')
  {
    vcm_nv_req = 2U;
    VCM_UartWrite("NV BUSY\r\n");
    return;
  }
  if (cmd == 'L')
  {
    int err = VCM_NvLoad();
    VCM_UartWrite((err == 0) ? "NV LOAD\r\n" : "NV EMPTY\r\n");
    return;
  }
  if (cmd == 'Q')
  {
    int32_t baud = VCM_ParseI32(&p, end);
    if (VCM_UartApplyBaud((uint32_t)baud) != 0)
    {
      VCM_UartWrite("ERR\r\n");
    }
    return;
  }
  if (cmd == 'U')
  {
    if (((uint32_t)(end - p) >= 4U) && (p[0] == 's') && (p[1] == 't') &&
        (p[2] == 'o') && (p[3] == 'p'))
    {
      const char *s = p;
      uint32_t id = 0U;
      uint8_t have = 0U;

      /* A stop queued behind a newer $U id= must not cancel that capture. */
      while (s < end)
      {
        if (((uint32_t)(end - s) >= 3U) && (s[0] == 'i') && (s[1] == 'd') && (s[2] == '='))
        {
          s += 3;
          id = (uint32_t)VCM_ParseI32(&s, end);
          have = 1U;
          break;
        }
        s++;
      }
      if ((have == 0U) || (id == vcm_cap_id))
      {
        VCM_CapStop();
      }
    }
    else
    {
      VCM_CapArm(p, end);
    }
    return;
  }
  if (cmd == 'K')
  {
    uint8_t s;
    if (vcm_dt_cal != 0U)
    {
      VCM_UartWrite("DT BUSY\r\n");
      return;
    }
    if (g_vcm.state != VCM_STATE_RUN)
    {
      VCM_UartWrite("DT ERR run\r\n");
      return;
    }
    vcm_dt_saved_ns = g_vcm.drv_dt_ns;
    vcm_dt_saved_mode = g_vcm.pwm_mode;
    vcm_dt_saved_ov = g_vcm.iref_override_en;
    vcm_dt_saved_hold = g_vcm.iref_hold_en;
    vcm_dt_saved_iref = g_vcm.iref_override_a;
    for (s = 0U; s < 4U; s++)
    {
      vcm_dt_sum_m[s] = 0.0f;
      vcm_dt_sum_i[s] = 0.0f;
    }
    vcm_dt_step = 0U;
    vcm_dt_phase = 0U;
    vcm_dt_tick = 0U;
    vcm_dt_nsamp = 0U;
    g_vcm.pwm_mode = VCM_PWM_MODE_BIPOLAR;
    g_vcm.drv_dt_ns = 0U;
    g_vcm.iref_hold_en = 0U;
    g_vcm.iref_override_en = 1U;
    g_vcm.iref_override_a = VCM_DtCalAmp(0U);
    vcm_dt_cal = 1U;
    VCM_HRTIM_SetAdcPhaseIdle();
    VCM_UartWrite("DT BUSY\r\n");
    return;
  }
  if ((cmd == 'G') || (cmd == 'F') || (cmd == 'Z') || (cmd == 'A') ||
      (cmd == 'B') || (cmd == 'T'))
  {
    (void)VCM_ServiceUartCmd((uint8_t)cmd);
    return;
  }
  VCM_UartWrite("ERR ");
  VCM_UartWrite(line);
  VCM_UartWrite("\r\n");
}

static int16_t VCM_CapMilli(float a)
{
  int32_t ma = (int32_t)((a * 1000.0f) + ((a >= 0.0f) ? 0.5f : -0.5f));

  if (ma > 32767)
  {
    ma = 32767;
  }
  if (ma < -32767)
  {
    ma = -32767;
  }
  return (int16_t)ma;
}

/* Continuous rate that the current 8N1 link can carry, both currents, no drops. */
static uint32_t VCM_CapContMaxHz(void)
{
  uint32_t baud = huart1.Init.BaudRate;
  uint32_t bytes_per_s;
  uint32_t lines;
  uint32_t hz;

  if (baud < 10U)
  {
    baud = 10U;
  }
  bytes_per_s = baud / 10U;
  lines = bytes_per_s / VCM_CAP_LINE_BYTES;
  if (lines < 1U)
  {
    lines = 1U;
  }
  hz = lines * VCM_CAP_PAIRS_LINE;
  if (hz > VCM_CTRL_FREQ_HZ)
  {
    hz = VCM_CTRL_FREQ_HZ;
  }
  return hz;
}

/* One control tick. Stores iref and ifb; UART drain stays in the main loop. */
static void VCM_CapPush(void)
{
  uint32_t wr;
  uint32_t slot;
  int16_t iref;
  int16_t ifb;

  if (vcm_cap_run == 0U)
  {
    return;
  }
  vcm_cap_div_cnt++;
  if (vcm_cap_div_cnt < vcm_cap_div)
  {
    return;
  }
  vcm_cap_div_cnt = 0U;
  iref = VCM_CapMilli(g_vcm.iref_a);
  ifb = VCM_CapMilli(g_vcm.ifb_avg_a);
  if (vcm_cap_mode == 0U)
  {
    wr = vcm_cap_wr;
    if (wr < vcm_cap_n)
    {
      vcm_cap_buf[wr * 2U] = iref;
      vcm_cap_buf[(wr * 2U) + 1U] = ifb;
      wr++;
      vcm_cap_wr = wr;
    }
    if (wr >= vcm_cap_n)
    {
      vcm_cap_run = 0U;
      vcm_cap_done = 1U;
    }
    return;
  }
  slot = vcm_cap_wr % VCM_CAP_SAMPLES;
  vcm_cap_buf[slot * 2U] = iref;
  vcm_cap_buf[(slot * 2U) + 1U] = ifb;
  vcm_cap_wr++;
}

static void VCM_CapPoll(void)
{
  uint32_t rd;
  uint32_t wr;
  uint32_t nsend;
  uint32_t i;

  wr = vcm_cap_wr;
  rd = vcm_cap_rd;
  if ((vcm_cap_mode != 0U) && ((wr - rd) > VCM_CAP_SAMPLES))
  {
    vcm_cap_ovf += (wr - rd) - VCM_CAP_SAMPLES;
    rd = wr - VCM_CAP_SAMPLES;
    vcm_cap_rd = rd;
  }
  if (rd == wr)
  {
    if (vcm_cap_done != 0U)
    {
      vcm_cap_done = 0U;
      VCM_UartWrite("CAP DONE id=");
      VCM_UartWriteI32((int32_t)vcm_cap_id);
      VCM_UartWrite(" n=");
      VCM_UartWriteI32((int32_t)vcm_cap_n);
      VCM_UartWrite(" ovf=0\r\n");
    }
    return;
  }
  nsend = wr - rd;
  if (nsend > VCM_CAP_PAIRS_LINE)
  {
    nsend = VCM_CAP_PAIRS_LINE;
  }
  VCM_UartWrite("CAP d=");
  for (i = 0U; i < nsend; i++)
  {
    uint32_t slot = (vcm_cap_mode == 0U) ? (rd + i) : ((rd + i) % VCM_CAP_SAMPLES);
    int16_t iref = vcm_cap_buf[slot * 2U];
    int16_t ifb = vcm_cap_buf[(slot * 2U) + 1U];

    if (i != 0U)
    {
      VCM_UartWrite(",");
    }
    VCM_UartWriteI32((int32_t)iref);
    VCM_UartWrite(",");
    VCM_UartWriteI32((int32_t)ifb);
  }
  VCM_UartWrite("\r\n");
  vcm_cap_rd = rd + nsend;
}

static void VCM_CapArm(const char *p, const char *end)
{
  uint32_t hz = 1000U;
  uint32_t mode = 0U;
  uint32_t n = 4096U;
  uint32_t div;
  uint32_t cap_id = 0U;
  uint8_t have_id = 0U;
  const char *k;

  while (p < end)
  {
    while ((p < end) && ((*p == ' ') || (*p == '\t')))
    {
      p++;
    }
    if (p >= end)
    {
      break;
    }
    k = p;
    while ((p < end) && (*p != '=') && (*p != ' '))
    {
      p++;
    }
    if ((p < end) && (*p == '='))
    {
      uint32_t klen = (uint32_t)(p - k);
      int32_t v;
      p++;
      v = VCM_ParseI32(&p, end);
      if (v < 0)
      {
        v = 0;
      }
      if ((klen == 2U) && (k[0] == 'h') && (k[1] == 'z'))
      {
        hz = (uint32_t)v;
      }
      else if ((klen == 2U) && (k[0] == 'i') && (k[1] == 'd'))
      {
        cap_id = (uint32_t)v;
        have_id = 1U;
      }
      else if ((klen == 4U) && (k[0] == 'm'))
      {
        mode = (v != 0) ? 1U : 0U;
      }
      else if ((klen == 1U) && (k[0] == 'n'))
      {
        n = (uint32_t)v;
      }
    }
    else
    {
      break;
    }
  }
  if (hz < 1U)
  {
    hz = 1U;
  }
  if (hz > VCM_CTRL_FREQ_HZ)
  {
    hz = VCM_CTRL_FREQ_HZ;
  }
  if ((mode != 0U) && (hz > VCM_CapContMaxHz()))
  {
    hz = VCM_CapContMaxHz();
  }
  div = (VCM_CTRL_FREQ_HZ + (hz / 2U)) / hz;
  if (div < 1U)
  {
    div = 1U;
  }
  if (div > VCM_CTRL_FREQ_HZ)
  {
    div = VCM_CTRL_FREQ_HZ;
  }
  hz = VCM_CTRL_FREQ_HZ / div;
  if (mode != 0U)
  {
    while ((hz > VCM_CapContMaxHz()) && (div < VCM_CTRL_FREQ_HZ))
    {
      div++;
      hz = VCM_CTRL_FREQ_HZ / div;
    }
  }
  if (n < 1U)
  {
    n = 1U;
  }
  if (n > VCM_CAP_SAMPLES)
  {
    n = VCM_CAP_SAMPLES;
  }
  vcm_cap_run = 0U;
  vcm_cap_done = 0U;
  vcm_cap_div_cnt = 0U;
  vcm_cap_wr = 0U;
  vcm_cap_rd = 0U;
  vcm_cap_ovf = 0U;
  vcm_cap_div = div;
  vcm_cap_hz = hz;
  vcm_cap_n = n;
  vcm_cap_mode = (uint8_t)mode;
  if (have_id != 0U)
  {
    vcm_cap_id = cap_id;
  }
  vcm_cap_run = 1U;
  VCM_UartWrite("CAP ARM id=");
  VCM_UartWriteI32((int32_t)vcm_cap_id);
  VCM_UartWrite(" hz=");
  VCM_UartWriteI32((int32_t)hz);
  VCM_UartWrite(" mode=");
  VCM_UartWriteI32((int32_t)mode);
  VCM_UartWrite(" n=");
  VCM_UartWriteI32((int32_t)n);
  VCM_UartWrite("\r\n");
}

static void VCM_CapStop(void)
{
  uint32_t ovf;

  vcm_cap_run = 0U;
  vcm_cap_done = 0U;
  ovf = vcm_cap_ovf;
  vcm_cap_wr = 0U;
  vcm_cap_rd = 0U;
  VCM_UartWrite("CAP STOP id=");
  VCM_UartWriteI32((int32_t)vcm_cap_id);
  VCM_UartWrite(" ovf=");
  VCM_UartWriteI32((int32_t)ovf);
  VCM_UartWrite("\r\n");
}

/* $W/$E set vcm_nv_req from the UART parser. Erase runs here, in the main loop. */
void VCM_NvPoll(void)
{
  uint8_t req = vcm_nv_req;

  VCM_CapPoll();
  uint8_t dt_msg = vcm_dt_msg;
  int err;

  if (dt_msg != 0U)
  {
    vcm_dt_msg = 0U;
    if (dt_msg == 1U)
    {
      VCM_UartWrite("DT ns=");
      VCM_UartWriteI32(vcm_dt_ns_out);
      VCM_UartWrite(" pos=");
      VCM_UartWriteI32(vcm_dt_ns_pos);
      VCM_UartWrite(" neg=");
      VCM_UartWriteI32(vcm_dt_ns_neg);
      VCM_UartWrite("\r\n");
    }
    else if (dt_msg == 3U)
    {
      VCM_UartWrite("DT ERR fit\r\n");
    }
    else
    {
      VCM_UartWrite("DT ERR abort\r\n");
    }
  }

  if (req == 0U)
  {
    return;
  }
  vcm_nv_req = 0U;
  VCM_UartFlush();
  if (req == 1U)
  {
    err = VCM_NvSave();
    if (err == 0)
    {
      VCM_UartWrite("NV SAVE\r\n");
    }
    else
    {
      VCM_UartWrite("NV ERR ");
      VCM_UartWriteI32(err);
      VCM_UartWrite("\r\n");
    }
  }
  else if (req == 2U)
  {
    err = VCM_NvFactory();
    if (err == 0)
    {
      VCM_UartWrite("NV FACTORY\r\n");
    }
    else
    {
      VCM_UartWrite("NV ERR ");
      VCM_UartWriteI32(err);
      VCM_UartWrite("\r\n");
    }
  }
}

void VCM_ServiceUartByte(uint8_t b)
{
  static char line[VCM_HOST_LINE_MAX];
  static uint8_t n;
  static uint8_t in_line;

  if (in_line == 0U)
  {
    if (b == (uint8_t)'$')
    {
      in_line = 1U;
      n = 0U;
      line[n++] = '$';
      return;
    }
    (void)VCM_ServiceUartCmd(b);
    return;
  }

  /* A second '$' means a new frame started; drop the partial line. */
  if (b == (uint8_t)'$')
  {
    n = 0U;
    line[n++] = '$';
    return;
  }

  if ((b == (uint8_t)'\n') || (b == (uint8_t)'\r'))
  {
    if (n >= 2U)
    {
      line[n] = '\0';
      VCM_HostLine(line, n);
    }
    in_line = 0U;
    n = 0U;
    return;
  }

  if (n < (VCM_HOST_LINE_MAX - 1U))
  {
    line[n++] = (char)b;
  }
  else
  {
    in_line = 0U;
    n = 0U;
  }
}

/*
 * Gain-cal UART (115200):
 *   G — dump EMA iref/ifb (mA) and raw ADC for clamp/scope对照
 *   F — force IREF = VCM_CAL_FORCE_A (default 0.5 A)
 *   Z — force IREF = 0
 *   A — clear override, use analog IREF
 *
 * Host: "$P\\n" telemetry, "$D\\n" params, "$S kp=..\\n" (see vcm_config.h).
 * After F: clamp_meter_A / firmware_ifb_mA → next IFB_GAIN_TRIM.
 * After known Vin: (Vin/20*10) / firmware_iref → IREF_GAIN_TRIM.
 */
uint8_t VCM_ServiceUartCmd(uint8_t cmd)
{
  if (cmd == VCM_UART_CMD_GAIN)
  {
    VCM_UartWrite("CAL iref_mA=");
    VCM_UartWriteMilli(g_vcm.cal_iref_mean);
    VCM_UartWrite(" ifb_mA=");
    VCM_UartWriteMilli(g_vcm.cal_ifb_mean);
    VCM_UartWrite(" ifb_avg_mA=");
    VCM_UartWriteMilli(g_vcm.ifb_avg_a);
    VCM_UartWrite(" ifb_hat_mA=");
    VCM_UartWriteMilli(g_vcm.ifb_hat_a);
    VCM_UartWrite(" ip_mA=");
    VCM_UartWriteMilli(g_vcm.ifb_ip_a);
    VCM_UartWrite(" im_mA=");
    VCM_UartWriteMilli(g_vcm.ifb_im_a);
    VCM_UartWrite(" adc_iref=");
    VCM_UartWriteI32((int32_t)g_vcm.adc_iref);
    VCM_UartWrite(" adc_ifb=");
    VCM_UartWriteI32((int32_t)g_vcm.adc_ifb);
    VCM_UartWrite(" pair=");
    VCM_UartWriteI32((int32_t)g_vcm.adc_pair_ok);
    VCM_UartWrite(" ndtr=");
    VCM_UartWriteI32((hadc1.DMA_Handle != NULL)
                         ? (int32_t)__HAL_DMA_GET_COUNTER(hadc1.DMA_Handle)
                         : -1);
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
    g_vcm.iref_hold_en = 0U;
    VCM_UartWrite("FORCE ");
    VCM_UartWriteMilli(VCM_CAL_FORCE_A);
    VCM_UartWrite(" mA cmd\r\n");
    return 1U;
  }
  if (cmd == VCM_UART_CMD_ZERO)
  {
    g_vcm.iref_override_a = 0.0f;
    g_vcm.iref_override_en = 1U;
    g_vcm.iref_hold_en = 1U;
    VCM_UartWrite("FORCE 0\r\n");
    return 1U;
  }
  if (cmd == VCM_UART_CMD_ANALOG)
  {
    g_vcm.iref_override_en = 0U;
    g_vcm.iref_hold_en = 0U;
    VCM_UartWrite("ANALOG\r\n");
    return 1U;
  }
  if (cmd == VCM_UART_CMD_BIPOLAR)
  {
    g_vcm.pwm_mode = VCM_PWM_MODE_BIPOLAR;
    vcm_tri_sign = 0;
    VCM_HRTIM_SetAdcPhaseIdle();
    VCM_UartWrite("PWM BIPOLAR\r\n");
    return 1U;
  }
  if (cmd == VCM_UART_CMD_TRILEVEL)
  {
    g_vcm.pwm_mode = VCM_PWM_MODE_TRILEVEL;
    vcm_tri_sign = 0;
    VCM_HRTIM_SetAdcPhaseIdle();
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

  pTimerCfg.InterruptRequests = (timer_idx == HRTIM_TIMERINDEX_TIMER_A)
                                    ? (HRTIM_TIM_IT_REP | HRTIM_TIM_IT_CMP3)
                                    : HRTIM_TIM_IT_NONE;
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

  /* Timer A CMP2 and CMP4 are the two ADC triggers. Timer B CMP2 = −pulse end. */
  if (timer_idx == HRTIM_TIMERINDEX_TIMER_A)
  {
    pCompareCfg.CompareValue = VCM_AdcTrigForZero((uint16_t)(VCM_PWM_PERIOD / 2U),
                                                 VCM_PWM_PERIOD);
  }
  else
  {
    pCompareCfg.CompareValue = (3U * VCM_PWM_PERIOD) / 4U;
  }
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, timer_idx, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }

  pCompareCfg.CompareValue = (timer_idx == HRTIM_TIMERINDEX_TIMER_A)
                             ? (VCM_PWM_PERIOD / 3U)
                             : ((2U * VCM_PWM_PERIOD) / 3U);
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, timer_idx, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }

  if (timer_idx == HRTIM_TIMERINDEX_TIMER_A)
  {
    pCompareCfg.CompareValue = (uint16_t)(VCM_PWM_PERIOD - VCM_ADC_CONV_GUARD);
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim, timer_idx, HRTIM_COMPAREUNIT_4, &pCompareCfg) != HAL_OK)
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
  adc_trig.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP2 |
                     HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP4;
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
