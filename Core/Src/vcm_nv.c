/**
 * @file    vcm_nv.c
 * @brief   Persist loop/plant params in Bank 2 Sector 3 (0x08160000).
 *
 * H743ZGT6 keeps Bank 1 at 0x08000000 and Bank 2 at 0x08100000. The
 * window 0x08080000..0x080FFFFF is not usable. Each HAL_FLASH_Program
 * writes one 256-bit flash word (32 B, 32 B aligned). A save appends
 * one 256 B record into the next blank slot. The 128 KB sector is
 * erased only after all 512 slots are full, so the same 32 B word is
 * never programmed twice. D-cache is invalidated only when it is
 * enabled. The main loop sends NV SAVE after this returns and then
 * blinks the status LED.
 *
 * Image size = VCM_NV_FLASH_WORDS x 32 B (currently 256 B). Unused tail is rsv[].
 */
#include "vcm_nv.h"
#include "vcm_ctrl.h"
#include "vcm_config.h"
#include "stm32h7xx_hal.h"
#include <stddef.h>
#include <string.h>

/* Fixed prefix is 20×uint32 = 80 B; rsv fills the rest of the 256 B image. */
#define VCM_NV_FIXED_U32  20U

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
  uint32_t notch_hz[4]; /* Hz, 0 = off. version >= 9 */
  uint32_t gz_en;       /* 0 = one PI zone, 1 = standstill scale. version >= 10 */
  float gz_scale;
  uint32_t rsv[(VCM_NV_BYTES / 4U) - VCM_NV_FIXED_U32];
} VCM_Nv_t;

_Static_assert((sizeof(VCM_Nv_t) % 32U) == 0U,
               "VCM_Nv_t size must be a multiple of 32 bytes");

/* 128 KB / 256 B = 512 append-only slots. Blank flash reads as 0xFFFFFFFF. */
#define VCM_NV_SECTOR_BYTES   (128U * 1024U)
#define VCM_NV_SLOT_COUNT     (VCM_NV_SECTOR_BYTES / VCM_NV_BYTES)
#define VCM_NV_BLANK_MAGIC    0xFFFFFFFFUL

_Static_assert((VCM_NV_SLOT_COUNT * VCM_NV_BYTES) == VCM_NV_SECTOR_BYTES,
               "NV slots must fill Bank 2 Sector 3 exactly");

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
  {
    uint32_t i;
    for (i = 0U; i < 4U; i++)
    {
      nv->notch_hz[i] = (uint32_t)g_vcm.notch_hz[i];
    }
  }
  nv->gz_en = (uint32_t)g_vcm.gz_en;
  nv->gz_scale = g_vcm.gz_scale;
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
  if (nv->version >= 9U)
  {
    uint32_t i;
    for (i = 0U; i < 4U; i++)
    {
      uint32_t hz = nv->notch_hz[i];
      if (hz > 10000U)
      {
        hz = 10000U;
      }
      g_vcm.notch_hz[i] = (uint16_t)hz;
    }
  }
  if (nv->version >= 10U)
  {
    float sc = nv->gz_scale;
    g_vcm.gz_en = (nv->gz_en != 0U) ? 1U : 0U;
    if ((sc < 0.05f) || (sc > 1.0f))
    {
      sc = VCM_GZ_SCALE_DEFAULT;
    }
    g_vcm.gz_scale = sc;
  }
  /* v11 turned full R-FF on and the axis squealed. Older slots go back to 0.
   * v12 and later keep an explicit ff= from the host. */
  if (nv->version < 12U)
  {
    g_vcm.ff_mod_per_a = 0.0f;
  }
}

static void VCM_NvApplyLegacy64(const VCM_NvLegacy64_t *nv)
{
  VCM_NvApplyCore(nv->version, nv->pwm_mode, nv->kp, nv->ki, nv->i_obs_a,
                  nv->iref_lpf_a, nv->ff_mod_per_a, nv->vbus_v,
                  nv->coil_l_h, nv->coil_r_ohm, nv->pi_fdbk);
  g_vcm.ff_mod_per_a = 0.0f;
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

static uint32_t VCM_NvSlotAddr(uint32_t index)
{
  return VCM_NV_ADDR + (index * VCM_NV_BYTES);
}

static uint32_t VCM_NvSlotMagic(uint32_t addr)
{
  return *(const volatile uint32_t *)addr;
}

/* First slot whose magic is still the erased value. 0 = sector is full. */
static uint32_t VCM_NvFindBlank(void)
{
  uint32_t i;

  for (i = 0U; i < VCM_NV_SLOT_COUNT; i++)
  {
    uint32_t addr = VCM_NvSlotAddr(i);
    if (VCM_NvSlotMagic(addr) == VCM_NV_BLANK_MAGIC)
    {
      return addr;
    }
  }
  return 0U;
}

/* Last slot with our magic and a matching CRC. Stops at the first blank. */
static uint32_t VCM_NvFindLatest(void)
{
  uint32_t i;
  uint32_t latest = 0U;

  for (i = 0U; i < VCM_NV_SLOT_COUNT; i++)
  {
    uint32_t addr = VCM_NvSlotAddr(i);
    uint32_t magic = VCM_NvSlotMagic(addr);

    if (magic == VCM_NV_BLANK_MAGIC)
    {
      break;
    }
    if ((magic == VCM_NV_MAGIC) && (VCM_NvImageOk(addr) != 0))
    {
      latest = addr;
    }
  }
  return latest;
}

int VCM_NvIsValid(void)
{
  return (VCM_NvFindLatest() != 0U) ? 1 : 0;
}

int VCM_NvLoad(void)
{
  uint32_t addr = VCM_NvFindLatest();
  const VCM_Nv_t *nv;

  if (addr == 0U)
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

static void VCM_NvInvalidate(uint32_t addr, int32_t len)
{
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
  {
    SCB_InvalidateDCache_by_Addr((void *)addr, len);
  }
#else
  (void)addr;
  (void)len;
#endif
}

static int VCM_NvEraseSector(void)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t sector_error = 0U;

  __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2);
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = FLASH_BANK_2;
  erase.Sector = FLASH_SECTOR_3;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
  {
    return -3;
  }
  VCM_NvInvalidate(VCM_NV_ADDR, (int32_t)VCM_NV_SECTOR_BYTES);
  return 0;
}

/* Programs one blank slot. The third HAL argument is the RAM address. */
static int VCM_NvProgramSlot(uint32_t addr, const VCM_Nv_t *nv)
{
  const uint8_t *src = (const uint8_t *)nv;
  uint32_t offset;

  __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2);
  for (offset = 0U; offset < VCM_NV_BYTES; offset += 32U)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                          addr + offset,
                          (uint32_t)(src + offset)) != HAL_OK)
    {
      return -3;
    }
  }
  VCM_NvInvalidate(addr, (int32_t)VCM_NV_BYTES);
  return 0;
}

int VCM_NvSave(void)
{
  VCM_Nv_t nv __attribute__((aligned(32)));
  uint32_t slot;
  int err;

  VCM_NvCapture(&nv);
  slot = VCM_NvFindBlank();
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return VCM_NvHalErr();
  }
  if (slot == 0U)
  {
    err = VCM_NvEraseSector();
    slot = VCM_NV_ADDR;
  }
  else
  {
    err = 0;
  }
  if (err == 0)
  {
    err = VCM_NvProgramSlot(slot, &nv);
  }
  (void)HAL_FLASH_Lock();
  if (err != 0)
  {
    return err;
  }
  if (VCM_NvImageOk(slot) == 0)
  {
    return -5;
  }
  return 0;
}

int VCM_NvFactory(void)
{
  int err;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return VCM_NvHalErr();
  }
  err = VCM_NvEraseSector();
  (void)HAL_FLASH_Lock();

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
  g_vcm.notch_hz[0] = 0U;
  g_vcm.notch_hz[1] = 0U;
  g_vcm.notch_hz[2] = 0U;
  g_vcm.notch_hz[3] = 0U;
  g_vcm.gz_en = VCM_GZ_EN_DEFAULT;
  g_vcm.gz_scale = VCM_GZ_SCALE_DEFAULT;
  g_vcm.gz_hold = 0U;

  return err;
}
