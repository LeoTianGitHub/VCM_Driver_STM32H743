/**
 * @file    vcm_nv.h
 * @brief   Loop params in Bank 2 Sector 3 (0x08160000).
 *          On H743ZGT6, 0x08080000..0x080FFFFF is not flash. Bank 2 is
 *          0x08100000..0x0817FFFF. The application stays in Bank 1, so this
 *          sector can be erased while instructions and interrupts keep running.
 *
 * Each record is 256 B. The 128 KB sector holds 512 slots. A save appends
 * into the next blank slot and erases the sector only when every slot is full.
 */
#ifndef VCM_NV_H
#define VCM_NV_H

#include <stdint.h>

#define VCM_NV_MAGIC              0x314D4356UL /* 'VCM1' */
#define VCM_NV_VERSION            12U
#define VCM_NV_ADDR               0x08160000UL /* Bank 2 Sector 3 */
#define VCM_NV_ADDR_LEGACY        0x08060000UL /* Bank 1 Sector 3, previous builds */
#define VCM_NV_FLASH_WORD_BYTES   32U
#define VCM_NV_FLASH_WORDS        8U   /* 8×32 = 256 B image */
#define VCM_NV_BYTES              (VCM_NV_FLASH_WORDS * VCM_NV_FLASH_WORD_BYTES)
#define VCM_NV_LEGACY_BYTES       64U  /* v1–v4 images */

int VCM_NvLoad(void);          /* 0 = applied, 1 = empty/bad (keep compile defaults) */
int VCM_NvSave(void);          /* 0 = ok */
int VCM_NvFactory(void);       /* erase sector + RAM compile defaults */
int VCM_NvIsValid(void);       /* 1 = flash image CRC ok */

#endif /* VCM_NV_H */
