/**
 * @file    vcm_nv.c
 * @brief   Persist loop/plant params in Bank 2 Sector 3 (0x08160000).
 *
 * H743ZGT6 keeps Bank 1 at 0x08000000 and Bank 2 at 0x08100000. The
 * window 0x08080000..0x080FFFFF is not usable. Each HAL_FLASH_Program
 * writes one 256-bit flash word (32 B, 32 B aligned). The sector is
 * erased before programming so a second write cannot fault on ECC.
 * D-cache is invalidated only when it is enabled. The main loop sends
 * NV SAVE after this returns and then blinks the status LED.
 *
 * Image size = VCM_NV_FLASH_WORDS x 32 B (currently 256 B). Unused tail is rsv[].
 */
#include "vcm_nv.h"
#include "vcm_ctrl.h"
#include "vcm_config.h"
#include "stm32h7xx_hal.h"
#include <stddef.h>
#include <string.h>

/* Fixed prefix is 14×uint32 = 56 B; rsv fills the rest of the 256 B image. */
#define VCM_NV_FIXED_U32  14U

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t crc;
  uint32_t pwm_mode;
  float kp;
  float ki;
  float i_obs_a;
  float iref_lpf_a;
  float ff_mod_per_a;
  float vbus_v;
  float coil_l_h;
  float coil_r_ohm;
  uint32_t pi_fdbk;
  uint32_t drv_dt_ns;
  uint32_t rsv[(VCM_NV_BYTES / 4U) - VCM_NV_FIXED_U32];
} VCM_Nv_t;

_Static_assert((sizeof(VCM_Nv_t) % 32U) == 0U,
               "VCM_Nv_t size must be a multiple of 32 bytes");

/* v1–v4 on-disk layout (exactly 64 B / 2 flash words). */
typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t crc;
  uint32_t pwm_mode;
  float kp;
  float ki;
  float i_obs_a;
  float iref_lpf_a;
  float ff_mod_per_a;
  float vbus_v;
  float coil_l_h;
  float coil_r_ohm;
  uint32_t pi_fdbk;
  uint32_t rsv[3];
} VCM_NvLegacy64_t;

_Static_assert(sizeof(VCM_Nv_t) == VCM_NV_BYTES, "NV image must match flash word span");
_Static_assert((VCM_NV_BYTES % VCM_NV_FLASH_WORD_BYTES) == 0U, "NV size must be N×32");
_Static_assert(sizeof(VCM_Nv_t) ==
                   (VCM_NV_FLASH_WORDS * FLASH_NB_32BITWORD_IN_FLASHWORD * 4U),
               "NV size must match HAL flash words");
_Static_assert(sizeof(VCM_NvLegacy64_t) == VCM_NV_LEGACY_BYTES, "legacy NV is 64 B");
_Static_assert(offsetof(VCM_Nv_t, rsv) == (VCM_NV_FIXED_U32 * 4U),
               "rsv must follow fixed prefix");

static uint32_t VCM_NvCrcBytes(const uint8_t *p, uint32_t nbytes)
{
  uint32_t c = 0xFFFFFFFFUL;
  uint32_t i;
  uint32_t n;

  for (n = 0U; n < nbytes; n++)
  {
    uint8_t b = p[n];
    if ((n >= 8U) && (n < 12U))
    {
      b = 0U;
    }
    c ^= b;
    for (i = 0U; i < 8U; i++)
    {
      uint32_t mask = (uint32_t)(-(int32_t)(c & 1U));
      c = (c >> 1U) ^ (0xEDB88320UL & mask);
    }
  }
  return ~c;
}

static void VCM_NvCapture(VCM_Nv_t *nv)
{
  memset(nv, 0, sizeof(*nv));
  nv->magic = VCM_NV_MAGIC;
  nv->version = VCM_NV_VERSION;
  nv->pwm_mode = (uint32_t)g_vcm.pwm_mode;
  nv->kp = g_vcm.kp;
  nv->ki = g_vcm.ki;
  nv->i_obs_a = g_vcm.i_obs_a;
  nv->iref_lpf_a = g_vcm.iref_lpf_a;
  nv->ff_mod_per_a = g_vcm.ff_mod_per_a;
  nv->vbus_v = g_vcm.vbus_v;
  nv->coil_l_h = g_vcm.coil_l_h;
  nv->coil_r_ohm = g_vcm.coil_r_ohm;
  nv->pi_fdbk = (uint32_t)g_vcm.pi_fdbk;
  nv->drv_dt_ns = (uint32_t)g_vcm.drv_dt_ns;
  nv->crc = VCM_NvCrcBytes((const uint8_t *)nv, sizeof(*nv));
}

static void VCM_NvApplyCore(uint32_t version, uint32_t pwm_mode,
                            float kp, float ki, float i_obs_a, float iref_lpf_a,
                            float ff_mod_per_a, float vbus_v,
                            float coil_l_h, float coil_r_ohm, uint32_t pi_fdbk)
{
  g_vcm.pwm_mode = (uint8_t)pwm_mode;
  g_vcm.kp = kp;
  g_vcm.ki = ki;
  g_vcm.i_obs_a = i_obs_a;
  g_vcm.iref_lpf_a = iref_lpf_a;
  g_vcm.ff_mod_per_a = ff_mod_per_a;
  if (version >= 2U)
  {
    g_vcm.vbus_v = vbus_v;
    if (g_vcm.vbus_v < VCM_VBUS_MIN_V)
    {
      g_vcm.vbus_v = VCM_VBUS_MIN_V;
    }
    if (g_vcm.vbus_v > VCM_VBUS_MAX_V)
    {
      g_vcm.vbus_v = VCM_VBUS_MAX_V;
    }
    g_vcm.coil_l_h = coil_l_h;
    g_vcm.coil_r_ohm = coil_r_ohm;
  }
  if (version >= 3U)
  {
    g_vcm.pi_fdbk = (pi_fdbk != 0U) ? VCM_PI_FDBK_OBS : VCM_PI_FDBK_AVG;
  }
}

static void VCM_NvApply(const VCM_Nv_t *nv)
{
  VCM_NvApplyCore(nv->version, nv->pwm_mode, nv->kp, nv->ki, nv->i_obs_a,
                  nv->iref_lpf_a, nv->ff_mod_per_a, nv->vbus_v,
                  nv->coil_l_h, nv->coil_r_ohm, nv->pi_fdbk);
  if (nv->version >= 8U)
  {
    uint32_t ns = nv->drv_dt_ns;
    if (ns > VCM_DRV_DT_NS_MAX)
    {
      ns = VCM_DRV_DT_NS_MAX;
    }
    g_vcm.drv_dt_ns = (uint16_t)ns;
  }
}

static void VCM_NvApplyLegacy64(const VCM_NvLegacy64_t *nv)
{
  VCM_NvApplyCore(nv->version, nv->pwm_mode, nv->kp, nv->ki, nv->i_obs_a,
                  nv->iref_lpf_a, nv->ff_mod_per_a, nv->vbus_v,
                  nv->coil_l_h, nv->coil_r_ohm, nv->pi_fdbk);
}

static int VCM_NvImageOk(uint32_t addr)
{
  VCM_Nv_t img;
  const volatile uint32_t *src = (const volatile uint32_t *)addr;
  uint32_t *dst = (uint32_t *)&img;
  const VCM_Nv_t *nv = &img;
  const VCM_NvLegacy64_t *leg;
  uint32_t i;

  __DSB();
  for (i = 0U; i < (sizeof(img) / 4U); i++)
  {
    dst[i] = src[i];
  }

  if (nv->magic != VCM_NV_MAGIC)
  {
    return 0;
  }
  if ((nv->version >= 5U) && (nv->version <= VCM_NV_VERSION))
  {
    return (VCM_NvCrcBytes((const uint8_t *)nv, sizeof(*nv)) == nv->crc) ? 1 : 0;
  }
  if ((nv->version >= 1U) && (nv->version <= 4U))
  {
    leg = (const VCM_NvLegacy64_t *)&img;
    return (VCM_NvCrcBytes((const uint8_t *)leg, sizeof(*leg)) == leg->crc) ? 1 : 0;
  }
  return 0;
}

int VCM_NvIsValid(void)
{
  return VCM_NvImageOk(VCM_NV_ADDR);
}

int VCM_NvLoad(void)
{
  uint32_t addr = VCM_NV_ADDR;
  const VCM_Nv_t *nv;

  if (VCM_NvImageOk(addr) == 0)
  {
    addr = VCM_NV_ADDR_LEGACY;
    if (VCM_NvImageOk(addr) == 0)
    {
      return 1;
    }
  }
  nv = (const VCM_Nv_t *)addr;
  if (nv->version >= 5U)
  {
    VCM_NvApply(nv);
  }
  else
  {
    VCM_NvApplyLegacy64((const VCM_NvLegacy64_t *)addr);
  }
  return 0;
}

static int VCM_NvHalErr(void)
{
  int err = (int)HAL_FLASH_GetError();
  if (err == 0)
  {
    err = -1;
  }
  return err;
}

static int VCM_NvEraseProgram(const VCM_Nv_t *nv, uint32_t do_program)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t sector_error = 0U;
  uint32_t offset;
  uint8_t *src;
  int err = 0;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return VCM_NvHalErr();
  }

  __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2);

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = FLASH_BANK_2;
  erase.Sector = FLASH_SECTOR_3;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
  {
    err = -3;
  }

  /* Third argument is the RAM address of the 32-byte source, not the data. */
  if ((err == 0) && (do_program != 0U) && (nv != 0))
  {
    src = (uint8_t *)nv;
    for (offset = 0U; offset < VCM_NV_BYTES; offset += 32U)
    {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                            VCM_NV_ADDR + offset,
                            (uint32_t)(src + offset)) != HAL_OK)
      {
        err = -3;
        break;
      }
    }
  }

  (void)HAL_FLASH_Lock();
  return err;
}

int VCM_NvSave(void)
{
  VCM_Nv_t nv __attribute__((aligned(32)));
  int err;

  VCM_NvCapture(&nv);
  err = VCM_NvEraseProgram(&nv, 1U);
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
  {
    SCB_InvalidateDCache_by_Addr((void *)VCM_NV_ADDR, (int32_t)sizeof(nv));
  }
#endif
  if (err != 0)
  {
    return err;
  }
  if (VCM_NvIsValid() == 0)
  {
    return -5;
  }
  return 0;
}

int VCM_NvFactory(void)
{
  int err;

  err = VCM_NvEraseProgram((const VCM_Nv_t *)0, 0U);

  g_vcm.kp = VCM_KP;
  g_vcm.ki = VCM_KI;
  g_vcm.i_obs_a = VCM_I_OBS_A;
  g_vcm.iref_lpf_a = VCM_IREF_OVR_LPF_A;
  g_vcm.vbus_v = VCM_VBUS_V;
  g_vcm.coil_l_h = VCM_COIL_L_H;
  g_vcm.coil_r_ohm = VCM_COIL_R_OHM;
  g_vcm.ff_mod_per_a = VCM_FF_MOD_PER_A;
  g_vcm.pwm_mode = VCM_PWM_MODE;
  g_vcm.pi_fdbk = VCM_PI_FDBK_DEFAULT;
  g_vcm.drv_dt_ns = VCM_DRV_DT_NS;

  return err;
}
