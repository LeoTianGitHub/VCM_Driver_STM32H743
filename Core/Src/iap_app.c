/**
 * @file    iap_app.c
 * @brief   Enter IAP bootloader / poll USART for upgrade request
 */
#include "iap_app.h"
#include "main.h"
#include "vcm_ctrl.h"

extern UART_HandleTypeDef huart1;

void IAP_RequestBootloader(void)
{
  /* Stop motor drive before reset so outputs are safe */
  VCM_Stop();

  /* Let the host finish the 0xA0 transaction and reopen COM after the
   * USB-UART blip. Boot waits several seconds for 0x05; this delay keeps
   * the reset from racing the tool's "enter boot OK → start transfer". */
  (void)HAL_UART_AbortReceive(&huart1);
  (void)HAL_UART_AbortTransmit(&huart1);
  HAL_Delay(300);

  __disable_irq();
  HAL_DeInit();
  NVIC_SystemReset();
}

void IAP_ServiceUartCommand(void)
{
  uint8_t cmd;

  /* Includes bytes saved while a capture line was blocking on TX. */
  while (VCM_UartRxPop(&cmd) != 0U)
  {
    if ((cmd == IAP_CMD_ENTER_BOOT) || (cmd == IAP_CMD_UPGRADE))
    {
      IAP_RequestBootloader();
      return;
    }
    if (cmd == IAP_CMD_VERSION)
    {
      VCM_BootAnnounce();
      continue;
    }

    VCM_ServiceUartByte(cmd);
  }

  VCM_NvPoll();
}
