# VCM 电流环诊断报告（2026-08-31，第二轮）

代码基线：当前 `vcm_ctrl.c` 已被简化为**纯 PI**（无前馈、无 `VCM_CTRL_MODE`、无滤波），Kp=0.50 / Ki=1500 / Ilim=0.40，50 kHz。本报告基于该版本。

---

## 0. 为什么上一轮的"固件只有几十 µs"结论不足以解决问题

静态分析只能证明**稳态环路**没有 ms 级结构，但给不出"这 8 ms 到底在哪"的证据。更关键的是，我上一轮漏掉了一个能在固件里真实产生 ms 级死区的机制：**使能线重启 → 2.56 ms 校准窗口内 PWM 被钳位、驱动器完全不响应命令**。

本轮因此做两件事：**(A) 把固件里所有 ms 级死区堵掉并计数**，**(B) 在固件内部装"取证仪表"**，下一次上电就能拿到决定性数字，而不是继续猜。

---

## 1. 新发现：唯一能在固件内产生 ms 级死区的机制

### 1.1 机制

`DRV_EN` 是低有效、来自上位机的长线，CubeMX 里被配成 `GPIO_NOPULL` + 下降沿中断：

```
使能线一次毛刺
 → ISR(或主循环) 判定 !en_active → VCM_Stop()
 → 主循环 VCM_ServiceDrvEnable() 看到 state==IDLE 且使能有效 → VCM_Start()
 → state=CALIB，连续 128 个样本(2.56 ms) 内 VCM_HRTIM_ApplyDuty(0) 且 pwm_armed=0
 → 两桥臂钳位，驱动器对命令完全无响应
```

毛刺连续来 2~3 次就是 5~8 ms，且现象与"命令来了电流要过 8 ms 才动"完全吻合。桥臂开关噪声耦合到浮空使能线上，是完全可能的。

### 1.2 本轮做的三处加固

| 位置 | 措施 |
|---|---|
| `main.c` (USER CODE MX_GPIO_Init_2) | DRV_EN 重配为 **PULLUP**（低有效 → 浮空时读无效，不会误触发）。放在 USER CODE 块里，Cube 重新生成也不会丢 |
| `vcm_ctrl.c` ISR | 使能有效/无效判定加**样本级抗抖**：需连续 `VCM_EN_GLITCH_SAMPLES`(5) 个样本 = 100 µs 仍无效才 `VCM_Stop()`；中途恢复记 `en_glitch_count++` 并继续驱动 |
| `vcm_ctrl.c` 主循环 | `VCM_ServiceDrvEnable()` 原会一看到无效就立刻 Stop，绕过 ISR 抗抖 → 改为 **2 ms 时间抗抖**（`VCM_EN_OFF_DEBOUNCE_MS`），仅在 ISR 未运行时才兜底 |
| `vcm_ctrl.c` `VCM_Start()` | **快速重启用**：5 s 内重新使能则复用已标定的零偏，跳过 2.56 ms 校准直接进 RUN（计 `calib_reuse_count`）。`VCM_CAL_REUSE_MS=0` 可关闭 |

同时新增计数：`start_count` / `stop_count` / `calib_count` / `calib_reuse_count` / `exti_count` / `en_glitch_count` / `fault_count`。

---

## 2. 固件内取证仪表（`VCM_DIAG_EN=1`，默认开）

| 变量 | 含义 | 判读 |
|---|---|---|
| `g_vcm.isr_ticks` | 50 kHz ISR 自由计数 | **每秒应 +50000**。若不符，控制速率本身错了——那足以让每个命令都变成 ms 级延迟，这是先要排除的 |
| `g_vcm.diag_ticks` | 从"固件看到的 IREF 阶跃"到"电流进入 ±10% 带"的 ISR tick 数（20 µs/格） | ≤10（≤200 µs）→ 环路+被控对象没问题，**延迟在上游**（IREF 模拟链 / PMAC / 示波器通道）；~400（8 ms）→ 延迟真在环路内 |
| `g_vcm.diag_timeouts` | 超过 20 ms 仍未进入带的次数 | 持续增长 = 电流根本没跟上 |
| `g_vcm.diag_iref_at_step` / `diag_ifb_at_step` / `diag_ifb_final` | 阶跃瞬间与结束时的命令/反馈值 | 直接读出 5× 增益偏差在哪一侧（见 §3） |
| `g_vcm.fault_count` | 进入故障的次数 | 见 §4 |

**示波器打标（无需新增引脚）**：固件在识别到命令阶跃的瞬间往 USART1 TX 发一字节 0x55（115200，约 87 µs 帧）。用 TX 脚的下降沿做触发，这就是"固件看到的命令时刻"。

- TX 打标后电流立刻动 → 固件与功率级都没问题，8 ms 在 IREF 模拟链或 PMAC。
- TX 打标后电流还要 8 ms 才动 → 问题在环路/功率级，再看 `diag_ticks` 与 `isr_ticks`。
- 若 TX 打标本身就比 PMAC 命令晚 8 ms → 命令在到达 MCU 前就被延迟了（板上 RC / PMAC 输出级）。

`VCM_DIAG_EN=0` 可把全部诊断代码编译掉；诊断不参与环路运算，不引入任何滤波或相位滞后。

---

## 3. 5× 增益偏差（仍然要查）

闭环迫使 **IFB 读数 = IREF 读数**，两种假设都表现为"命令 750 mA、实测 150 mA"：

- **A：IFB 标度偏高 5×**（RS 实际 100 mΩ 而非 20 mΩ，或 TPA8001 增益不是 8.2）→ 实际电流 = 命令/5。旁证：实际 >~2.4 A 时假过流跳闸。
- **B：IREF 标度偏低 5×**（TPA2672 有效差分增益 0.06 而非 0.30）→ 目标本身只有 150 mA。

用新变量一次读出：输入 ±0.75 V 命令 → 看 `diag_iref_at_step` 是否 ±0.75 A（偏低 = B）；看 `diag_ifb_final` 与钳表实际值（ifb 偏高 = A）。修根因，**不要用 trim 系数掩盖**。

---

## 4. 另一个候选：过流误跳闸（不用仪器也能查）

纯 PI 无前馈时，阶跃靠 Kp 直接推：0.75 A 阶跃 → m≈0.375 → 线圈电压约 36 V → 约 30 A/ms。若过冲碰到 12 A 阈值（10 A 满量程只有 20% 余量），会进 FAULT 并锁存，**只能靠上位机撤掉使能才恢复**，随后又走一遍 2.56 ms 校准——同样表现为"迟滞几毫秒才动"。

**肉眼判断**：LED 在 RUN 时 1 s 闪一次，**FAULT 时 100 ms 闪一次**。运动过程中如果 LED 变成快闪，就是过流/故障在反复跳闸。

---

## 5. 建议的上电验证顺序

1. 编译下载，先读 `isr_ticks`：1 秒内应为 50000 ± 少量。不是 → HRTIM/ISR 速率问题，先解决它。
2. 给一个方波命令，读 `diag_ticks` + 看 USART TX 打标与电流的示波器对齐关系（判据见 §2）。
3. 读 `start_count` / `stop_count` / `en_glitch_count` / `fault_count`：任一在运行中持续增加 → 对应 §1 或 §4 的机制，已加固，看是否消失。
4. 按 §3 定标增益，再谈 Kp/Ki 微调。

## 6. 本轮改动文件

- `Core/Src/main.c`：DRV_EN 上拉（USER CODE 块）；ADC1/2 采样时间 64.5 cycles（沿用上一轮）
- `Core/Src/stm32h7xx_it.c`：注释 30 kHz → 50 kHz（沿用上一轮）
- `Core/Inc/vcm_config.h`：新增 `VCM_EN_GLITCH_SAMPLES` / `VCM_EN_OFF_DEBOUNCE_MS` / `VCM_CAL_REUSE_MS` / `VCM_DIAG_*`
- `Core/Inc/vcm_ctrl.h`：新增计数与诊断字段、`extern UART_HandleTypeDef huart1`
- `Core/Src/vcm_ctrl.c`：使能抗抖、快速重启用、诊断函数、计数器
- Git 工作区仍为未提交状态，未 commit
