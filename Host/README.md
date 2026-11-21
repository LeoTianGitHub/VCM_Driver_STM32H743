# VCM Host（WPF）

STM32H743 VCM 电流环调试上位机。USART1 **115200 8N1**（PB14 TX / PB15 RX）。

## 运行

```
cd Host/VcmHost
dotnet run
```

或 Visual Studio 打开 `Host/VcmHost/VcmHost.csproj`。

**必须先烧录带 `$` 行协议的固件**（`VCM_HOST_PROTO 2`）。App 链接在 `0x08020000`，需先有 Bootloader；IAP 升级只擦 Sector 1–2。旧固件只能识别单字节 `G F Z A B T`。

不要在本机已连接时用串口助手占用同一 COM；不要发送 `0x05` / `0xA0`（会进 IAP）。

## 功能

- 15 Hz 遥测：IREF、观测电流 î、单点 IFB（周期结束前 1.5 µs）、m、状态、故障、ADC
- 约 8 s 波形：IREF / î / IFB
- 写 RAM 参数：Kp、Ki、观测器、命令 LPF（默认 3 kHz）、R-FF、**Vbus / R / L**
- IREF 覆盖、模拟给定、Force 0.5 A、Z 等宽 dither（关 PI）
- 三电平 / 双极性、清故障、导出 CSV

参数：$S 写入 RAM；**$W 追加写入 Bank2 Sector3（0x08160000，512 槽，写满才擦除）** 才掉电保持。上电取最后一个 CRC 正确的槽。`$L` 再加载，`$E` 擦除并恢复编译默认。

上位机可 **导出/导入** 环路参数为 `*.vcmparams`（JSON：kp/ki/fdbk/obs/lpf/ff/vbus/r/l_mh）。导入先填界面，可选立刻 `$S`；掉电仍需 `$W`。

IAP 升级只擦 Sector 1–2，不会清掉参数。不要对 Sector3 做全片擦除。

整数标定：`*_e3` = ×1000，`*_e6` = ×1e6，`*_ma` = 毫安。

单字节 `G F Z A B T` 仍可用（串口助手）。

## 协议

行以 `$` 开头、`\n` 结束（避免与 IAP 的 0x05 / 0xA0 冲突）。

| 主机 | 从机 |
|------|------|
| `$I` | `ID … fw=1.4.144 pwm=20000`；三电平双边窄脉冲，每侧至少 cm（默认 400 ns）；两段续流各采样一次、一次 PI |
| `$S den=` | `0`=CCM+PI（默认，PMAC 用）；`1`=小电流 DCM（仅开环标定） |
| `$S dx=/de=` | DCM 退出 |iref|（mA）；`de=` 保留兼容，退出不再看 IFB |
| `$P` | `TL … iavg_ma=… ip_ma=… im_ma=…`（三电平和双极性都是每周期续流两点，iavg 为两点平均） |
| `$D` | `PR hw=1 fw=1.4.144 … fdbk=1 dt_ns=… cm=…`（三电平两侧脉冲都至少 cm，默认 400 ns；采样在续流段） |
| `$S … fdbk=0|1 dt=<ns> cm=<ns> …` | `OK`/`ERR`；`cm` 为每一侧窄脉冲的最小宽度 ns（200–2000） |
| `$R 1500` | IREF 覆盖 1.500 A（PI 开） |
| `$K` | `DT BUSY`，结束后 `DT ns=… pos=… neg=…`。机构停住、驱动使能。失败为 `DT ERR run/abort/fit` |
| `$O` | 一次性标定 IREF 零偏。**上位机必须先保证模拟 IREF 输入为 0 V 并保持 10 ms**。先回 `IREFZ BUSY`，再回 `IREFZ <mA>`。失败为 `IREFZ ERR override`（当前在 `$R` 覆盖态，先 `$A`）/ `range`（读数超 ±50 mA，说明输入并非 0 V）/ `nosample`（500 ms 内没拿到新的 ADC 配对） |
| `$U id=<n> hz=<Hz> mode=0\|1 n=<点数>` | `CAP ARM id=…` 后 `CAP d=<iref_mA,ifb_mA,…>`。mode 0 单次，mode 1 连续且频率不超过当前波特率能收完的上限。`$U stop id=<n>` 只停止这一次；旧的停止不会取消新的采集 |
| `$Q <baud>` | 先回 `BAUD <baud>`，再把 USART1 改到该波特率。上位机可选到 2000000。引导程序仍是 115200 |
| `$W` | `NV SAVE` / `NV ERR` 把 RAM 参数写入 Flash |
| `$L` | `NV LOAD` / `NV EMPTY` 从 Flash 再加载 |
| `$E` | `NV FACTORY` 擦除并恢复编译默认 |

整数标定：`*_e3` = ×1000，`*_e6` = ×1e6，`*_ma` = 毫安。

单字节 `G F Z A B T` 仍可用（串口助手）。
