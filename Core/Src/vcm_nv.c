/**
 * @file    vcm_nv.c
 * @brief   Persist loop/plant params in FLASH Sector 3 (0x08060000).
 *
 * Uses HAL_FLASHEx_Erase() from Flash and HAL_FLASH_Program() from DTCM.
 *
 * Real failure mode (not "sector vs bank"): after PG, the next instruction
 * fetch from Bank 1 deadlocks until the 256-bit word is complete. Bootloaders
 * survive this because I-cache already holds that loop. This app erased first,
 * which flushed I-cache, then programmed — so the HAL store loop faulted even
 * with a temporary SCB_EnableICache(). Keep I-cache on from main, and keep the
 * HAL program function itself in DTCM so those 8 stores never touch Bank 1.
 */
#include "vcm_nv.h"
#include "vcm_ctrl.h"
#include "vcm_config.h"
#include "stm32h7xx_hal.h"
#include <string.h>

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
  uint32_t rsv[4];
} VCM_Nv_t;

_Static_assert(sizeof(VCM_Nv_t) == 64U, "NV record must be two H7 flash words");
_Static_assert(sizeof(VCM_Nv_t) == (2U * FLASH_NB_32BITWORD_IN_FLASHWORD * 4U),
               "NV record must match the HAL flash word");

static uint32_t VCM_NvCrc(const VCM_Nv_t *nv)
{
  const uint8_t *p = (const uint8_t *)nv;
  uint32_t c = 0xFFFFFFFFUL;
  uint32_t i;
  uint32_t n;

  for (n = 0U; n < sizeof(VCM_Nv_t); n++)
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
  memset(nv, 0xFF, sizeof(*nv));
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
  nv->crc = VCM_NvCrc(nv);
}

static void VCM_NvApply(const VCM_Nv_t *nv)
{
  g_vcm.pwm_mode = (uint8_t)nv->pwm_mode;
  g_vcm.kp = nv->kp;
  g_vcm.ki = nv->ki;
  g_vcm.i_obs_a = nv->i_obs_a;
  g_vcm.iref_lpf_a = nv->iref_lpf_a;
  g_vcm.ff_mod_per_a = nv->ff_mod_per_a;
  if (nv->version >= 2U)
  {
    g_vcm.vbus_v = nv->vbus_v;
    if (g_vcm.vbus_v < VCM_VBUS_MIN_V)
    {
      g_vcm.vbus_v = VCM_VBUS_MIN_V;
    }
    if (g_vcm.vbus_v > VCM_VBUS_MAX_V)
    {
      g_vcm.vbus_v = VCM_VBUS_MAX_V;
    }
    g_vcm.coil_l_h = nv->coil_l_h;
    g_vcm.coil_r_ohm = nv->coil_r_ohm;
  }
}

static const VCM_Nv_t *VCM_NvFlash(void)
{
  return (const VCM_Nv_t *)VCM_NV_ADDR;
}

int VCM_NvIsValid(void)
{
  const VCM_Nv_t *nv = VCM_NvFlash();

  if ((nv->magic != VCM_NV_MAGIC) ||
      ((nv->version != 1U) && (nv->version != VCM_NV_VERSION)))
  {
    return 0;
  }
  if (VCM_NvCrc(nv) != nv->crc)
  {
    return 0;
  }
  return 1;
}

int VCM_NvLoad(void)
{
  if (VCM_NvIsValid() == 0)
  {
    return 1;
  }
  VCM_NvApply(VCM_NvFlash());
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
  int err = 0;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return VCM_NvHalErr();
  }

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = FLASH_BANK_1;
  erase.Sector = FLASH_SECTOR_3;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  /* Erase wait uses HAL_GetTick — leave IRQs enabled so SysTick advances. */
  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
  {
    err = VCM_NvHalErr();
  }
  else if (do_program != 0U)
  {
    /* Current-loop ISR is in Bank 1; mask it only for the program call. */
    HAL_NVIC_DisableIRQ(HRTIM1_TIMA_IRQn);
    if ((HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                           VCM_NV_ADDR,
                           (uint32_t)nv) != HAL_OK) ||
        (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                           VCM_NV_ADDR + 32U,
                           ((uint32_t)nv) + 32U) != HAL_OK))
    {
      err = VCM_NvHalErr();
    }
    HAL_NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);
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
  SCB_InvalidateDCache_by_Addr((void *)VCM_NV_ADDR, (int32_t)sizeof(nv));
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

  return err;
}
