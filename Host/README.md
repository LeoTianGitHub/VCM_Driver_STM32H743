# VCM Host（WPF）

STM32H743 VCM 电流环调试上位机。USART1 **115200 8N1**（PB14 TX / PB15 RX）。

## 运行

```
cd Host/VcmHost
dotnet run
```

或 Visual Studio 打开 `Host/VcmHost/VcmHost.csproj`。

**必须先烧录带 `$` 行协议的固件**（`VCM_HOST_PROTO 3`，`fw=1.4.54`）。送丝电机额定 **1 A**，线圈 **5.42 Ω / 912 µH**。App 链接在 `0x08020000`，需先有 Bootloader；IAP 升级只擦 Sector 1–2。旧固件只能识别单字节 `G F Z A B T`。

不要在本机已连接时用串口助手占用同一 COM；不要发送 `0x05` / `0xA0`（会进 IAP）。

## 功能

- 15 Hz 遥测：IREF、观测电流 î、单点 IFB（周期结束前 1.5 µs）、m、状态、故障、ADC
- 约 8 s 波形：IREF / î / IFB
- 写 RAM 参数：Kp、Ki、观测器、IREF LPF、R-FF、**Vbus / R / L**
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
| `$I` | `ID … fw=1.4.56 pwm=50000`（可选两段 PI：静止时 Kp/Ki 同比例降低） |
| `$P` | `TL iref_ma=… ihat_ma=… iavg_ma=… gz=0|1 gzh=0|1 …`（gzh=1 表示正在用静止比例） |
| `$D` | `PR hw=1 fw=1.4.56 proto=3 … fdbk=1 dt_ns=… n0=… n1=… n2=… n3=… gz=0|1 gs_e3=… mode=0|1`（fdbk 0=IFB 1=î。编译默认 fdbk=1，观测 α=0.634≈8000 Hz，R-FF=0，n0–n3=0。gz 0=单段 1=两段，mode 0=双极性 1=三电平） |
| `$S kp=… ki=… obs=… lpf=… ff=… vbus=… r=… l=… fdbk=0|1 dt=<ns> n0=<Hz> n1=<Hz> n2=<Hz> n3=<Hz> gz=0|1 gs=<×1000>` | `OK` / `ERR`（vbus/r ×1000，l=µH；dt 为双极性两个沿的补偿 ns，默认 400，0 关闭，写入后立即生效。三电平不用这项，闭环时两边至少各 600 ns，指令只加长一边；开环保持时两边都关。n0–n3 为电流指令陷波 Hz，0 关闭；gs 为静止时 Kp/Ki 比例 ×1000，默认 350） |
| `$R 500` | IREF 覆盖 0.500 A（PI 开，额定 ±1 A） |
| `$K` | `DT BUSY`，结束后 `DT ns=… pos=… neg=…`。机构停住、驱动使能。失败为 `DT ERR run/abort/fit` |
| `$U id=<n> hz=<Hz> mode=0\|1 n=<点数>` | `CAP ARM id=…` 后 `CAP d=<iref_mA,ifb_mA,…>`。mode 0 单次，mode 1 连续且频率不超过当前波特率能收完的上限。`$U stop id=<n>` 只停止这一次；旧的停止不会取消新的采集 |
| `$Q <baud>` | 先回 `BAUD <baud>`，再把 USART1 改到该波特率。上位机可选到 2000000。引导程序仍是 115200 |
| `$W` | `NV SAVE` / `NV ERR` 把 RAM 参数写入 Flash |
| `$L` | `NV LOAD` / `NV EMPTY` 从 Flash 再加载 |
| `$E` | `NV FACTORY` 擦除并恢复编译默认 |

整数标定：`*_e3` = ×1000，`*_e6` = ×1e6，`*_ma` = 毫安。

单字节 `G F Z A B T` 仍可用（串口助手）。
