/**
 * @file    iap_app.h
 * @brief   Application-side helpers for IAP_UartSTM32H7 bootloader
 *
 * Flash map (match bootloader):
 *   0x08000000  Bootloader Sector0 (128KB), KEY_BASE @ 0x0801FF00
 *   0x08020000  This application (256KB, Sector 1-2)
 *
 * IAP UART: USART1 @ 115200, PB14=TX, PB15=RX (bootloader must use the same pins)
 * Host: after reset / LED blink, send 0x05 within ~3s to start encrypted upgrade.
 */
#ifndef IAP_APP_H
#define IAP_APP_H

#include <stdint.h>

#define IAP_BOOTLOADER_ADDRESS   0x08000000UL
#define IAP_APP_ADDRESS          0x08020000UL
#define IAP_APP_MAX_SIZE         (256UL * 1024UL)
#define IAP_KEY_BASE             0x0801FF00UL

/** Host command accepted by bootloader to start firmware update */
#define IAP_CMD_UPGRADE          0x05U
/** App may also accept this on USART1 to request reboot into bootloader */
#define IAP_CMD_ENTER_BOOT       0xA0U

void IAP_RequestBootloader(void);
void IAP_ServiceUartCommand(void);

#endif /* IAP_APP_H */
