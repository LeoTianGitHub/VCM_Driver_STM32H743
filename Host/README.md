# VCM Host（WPF）

STM32H743 VCM 电流环调试上位机。USART1 **115200 8N1**（PB14 TX / PB15 RX）。

## 运行

```
cd Host/VcmHost
dotnet run
```

或 Visual Studio 打开 `Host/VcmHost/VcmHost.csproj`。

**必须先烧录带 `$` 行协议的固件**（`VCM_HOST_PROTO 1`）。旧固件只能识别单字节 `G F Z A B T`。

不要在本机已连接时用串口助手占用同一 COM；不要发送 `0x05` / `0xA0`（会进 IAP）。

## 功能

- 15 Hz 遥测：IREF、观测电流 î、两点 IFB、m、状态、故障、ADC
- 约 8 s 波形：IREF / î / IFB
- 写 RAM 参数：Kp、Ki、观测器、IREF LPF、R-FF、**Vbus / R / L**
- IREF 覆盖、模拟给定、Force 0.5 A、Z 等宽 dither（关 PI）
- 三电平 / 双极性、清故障、导出 CSV

参数：$S 写入 RAM；**$W 保存到 Flash Sector3（0x08060000）** 才掉电保持。上电自动加载。`$L` 再加载，`$E` 擦除并恢复编译默认。

IAP 升级只擦 Sector 1–2，不会清掉参数。不要对 Sector3 做全片擦除。

整数标定：`*_e3` = ×1000，`*_e6` = ×1e6，`*_ma` = 毫安。

单字节 `G F Z A B T` 仍可用（串口助手）。

## 协议

行以 `$` 开头、`\n` 结束（避免与 IAP 的 0x05 / 0xA0 冲突）。

| 主机 | 从机 |
|------|------|
| `$I` | `ID VCM_H743 proto=1 pwm=50000` |
| `$P` | `TL iref_ma=… ihat_ma=… iavg_ma=… …` |
| `$D` | `PR proto=1 kp_e3=230 ki_e3=460000 obs_e3=118 …` |
| `$S kp=… ki=… obs=… lpf=… ff=… vbus=… r=… l=…` | `OK` / `ERR`（vbus/r ×1000，l=µH） |
| `$R 1500` | IREF 覆盖 1.500 A（PI 开） |
| `$W` | `NV SAVE` / `NV ERR` 把 RAM 参数写入 Flash |
| `$L` | `NV LOAD` / `NV EMPTY` 从 Flash 再加载 |
| `$E` | `NV FACTORY` 擦除并恢复编译默认 |

整数标定：`*_e3` = ×1000，`*_e6` = ×1e6，`*_ma` = 毫安。

单字节 `G F Z A B T` 仍可用（串口助手）。
