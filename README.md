# VCM_Driver_H743

Single-axis VCM PWM current loop on **STM32H743ZGT6** (LQFP144) using **HRTIM1**.

## Pin map

| Function | Pin | Signal |
|----------|-----|--------|
| Ifb+ / Ifb− | PA0 / PA1 | ADC1 CH16 differential |
| Iref+ / Iref− | PA6 / PA7 | ADC2 CH3 differential |
| AH / AL | **PC6 / PC7** | HRTIM_CHA1 / CHA2 (AF1) |
| BH / BL | **PC8 / PA8** | HRTIM_CHB1 (AF1) / CHB2 (**AF3**) |
| DRV_EN | PB0 | Out |
| FAULT | PB1 | In, active low |
| EXT_EN | PB2 | In, active high |

> H743 HRTIM 引脚与 G474 的 PA8–PA11 不同，请按上表布线。

## Timing

- SYSCLK 400 MHz, HRTIM ≈ 200 MHz (`DIV1`)
- PWM **100 kHz**, dead-time ≈ **400 ns**
- Current loop on HRTIM Timer A repetition IRQ

## Build

Open `VCM_Driver_H743` in CubeIDE and build. FW pack: STM32Cube FW_H7 V1.12.1
