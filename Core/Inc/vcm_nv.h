/**
 * @file    vcm_nv.h
 * @brief   Loop params in FLASH Sector 3 (0x08060000). IAP erases Sector 1–2 only.
 */
#ifndef VCM_NV_H
#define VCM_NV_H

#include <stdint.h>

#define VCM_NV_MAGIC     0x314D4356UL /* 'VCM1' */
#define VCM_NV_VERSION   2U
#define VCM_NV_ADDR      0x08060000UL

int VCM_NvLoad(void);          /* 0 = applied, 1 = empty/bad (keep compile defaults) */
int VCM_NvSave(void);          /* 0 = ok */
int VCM_NvFactory(void);       /* erase sector + RAM compile defaults */
int VCM_NvIsValid(void);       /* 1 = flash image CRC ok */

#endif /* VCM_NV_H */
