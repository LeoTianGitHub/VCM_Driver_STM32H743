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

  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != 0U)
  {
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
  }

  /* Drain every pending byte; one-byte-per-loop drops $W at 115200. */
  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != 0U)
  {
    cmd = (uint8_t)(huart1.Instance->RDR & 0xFFU);

    if ((cmd == IAP_CMD_ENTER_BOOT) || (cmd == IAP_CMD_UPGRADE))
    {
      IAP_RequestBootloader();
      return;
    }

    VCM_ServiceUartByte(cmd);
  }

  VCM_NvPoll();
}
