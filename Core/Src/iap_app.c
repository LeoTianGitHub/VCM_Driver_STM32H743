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
  uint8_t cmd = 0U;

  /* Non-blocking poll: host may send 0xA0 / 0x05 while app is running */
  if (HAL_UART_Receive(&huart1, &cmd, 1U, 0U) != HAL_OK)
  {
    return;
  }

  if ((cmd == IAP_CMD_ENTER_BOOT) || (cmd == IAP_CMD_UPGRADE))
  {
    IAP_RequestBootloader();
  }
}
