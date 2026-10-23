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

    VCM_ServiceUartByte(cmd);
  }

  VCM_NvPoll();
}
