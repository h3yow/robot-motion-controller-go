# 设计与量纲约定

本文档说明运动学/量纲约定、控制周期与容差设定、故障与恢复语义、并发仲裁规则，
以及本地离线验证方法。所有结论以**代码常量为准**；本文与代码冲突时以代码为准。

## 1. 量纲约定（SI）

| 量 | 单位 | 说明 |
|----|------|------|
| 关节位置 | rad | `Position`，行程为闭区间 `[MinPosition, MaxPosition]` |
| 关节速度 | rad/s | `Velocity`，硬限 `MaxVelocity` |
| 关节加速度 | rad/s² | `Accel`，硬限 `MaxAccel` |
| 力矩（扭矩） | N·m | `Torque`，硬限 `MaxTorque` |
| 转动惯量 | kg·m² | `Inertia`，被控对象 τ = J·α |
| 重力偏置力矩 | N·m | `GravityTorque`，恒定加在对象上，前馈以负号抵消 |
| 笛卡尔位置 | m | 二连杆末端 `(X, Y)` |
| 连杆长度 | m | `L1, L2 > 0` |
| 控制周期 | s | `ControlPeriod > 0`，所有轴共享同一固定周期 |

控制器增益的量纲：`Kp` N·m/rad，`Ki` N·m/(rad·s)，`Kd` N·m·s/rad；
积分钳位 `IntegralWindup` 单位 N·m。

## 2. 运动学约定（Planar 2R）

- q1：肩关节相对 +X 轴的转角；q2：肘关节相对第一连杆方向的转角；均为弧度。
- 正解：`x = L1·cos q1 + L2·cos(q1+q2)`，`y = L1·sin q1 + L2·sin(q1+q2)`。
- 逆解：余弦定理求 q2，`elbowUp` 选择肘上/肘下分支；q1 由几何关系给出。
- 可达工作区为环（含边界）：`|L1-L2| ≤ r ≤ L1+L2`（r = √(x²+y²)）。
  边界处仅对 1e-9 量级浮点越界做钳位；真正越界一律 `TARGET_UNREACHABLE`。
- 含 NaN/±Inf 的目标在逆解**最早**阶段被判为 `TARGET_NON_FINITE`，与不可达严格区分。
- 逆解关节角若超出关节行程，门面层判 `OUT_OF_TRAVEL`。
- 同一输入逆解确定；在工作区内部对目标位置连续（分支边界除外）。

## 3. 轨迹规划约定

- 多轴采用**时间同步的对称梯形速度曲线**：各轴按自身最短时间规划，整体取最长时长
  `T`，向上取整到控制周期网格 `Tn = ceil(T/Period)·Period`，慢轴巡航、快轴等比放慢，
  因此所有轴同一周期起步、同一周期到点（真正联动，而非各轴独立跑完）。
- 采样严格发生在 `t = k·Period`；末点强制等于目标位置、速度/加速度置零，消除浮点残差。
- 拒绝优先级（先判先返回，原因可区分）：
  1. 维度不一致/极限非正 → `CONTRADICTORY_TARGET`
  2. NaN/Inf（起点或目标）→ `TARGET_NON_FINITE`
  3. 越行程 → `OUT_OF_TRAVEL`
  4. 速度包络越限 → `VELOCITY_LIMIT_EXCEEDED`
  5. 加速度包络越限 → `ACCELERATION_LIMIT_EXCEEDED`
  6. 峰值所需力矩 `J·a + |gravity| > MaxTorque` → `TORQUE_LIMIT_EXCEEDED`
- 任何拒绝都不替换默认值、不裁剪到限值后"将就算一条轨迹"。

## 4. 控制周期与容差设定

- 周期由 `ControlPeriod` 固定（测试常用 0.002 s / 0.01 s）。`Step()` 恰好推进一周期。
- 跟踪容差 `TrackingTolerance`（rad）：|q_cmd − q_actual| 超过即判跟踪故障。
- **故障阈值分层**（互不重叠、结论可解释）：
  - |误差| ≤ `TrackingTolerance`：正常；
  - `TrackingTolerance` < |误差| ≤ `100·TrackingTolerance`：`FEEDBACK_TRACKING_ERROR`（可恢复）；
  - |误差| > `100·TrackingTolerance`：`CONTROL_DIVERGED`（不可恢复）。
- 饱和：力矩被钳到 `MaxTorque` 记一次饱和；连续 `SaturationCycles` 周期 →
  `DRIVE_SATURATION_PERSISTENT`（可恢复）。
- 末端收敛：轨迹到点后进入有界保持阶段，需连续 `20` 个周期满足
  |位置误差| ≤ `0.1·TrackingTolerance` 且 |速度| ≤ `1e-4 rad/s` 才报 `Done`；
  超过 `5 s` 仍未收敛 → 判 `CONTROL_DIVERGED`，绝不返回"未收敛的成功"。
- 恢复时的停稳判据：|v| ≤ `1e-6 rad/s`（另见第 5 节）。

## 5. 故障与恢复语义

### 5.1 故障分类

| 故障 | 依据 | 可恢复 |
|------|------|--------|
| `FEEDBACK_LOST` | 编码器读数无效 | 是 |
| `FEEDBACK_TRACKING_ERROR` | 误差越容差但未越发散包络 | 是 |
| `DRIVE_SATURATION_PERSISTENT` | 连续饱和达阈值 | 是 |
| `CONTROL_DIVERGED` | 误差越 100× 容差，或末端保持超时未收敛 | 否 |
| `NON_FINITE_STATE` | 指令/反馈/状态 NaN/Inf，或维度不匹配 | 否 |

### 5.2 状态机

```
IDLE ──Submit──► RUNNING ──正常收敛──► IDLE(Done)
                 │  ▲
                 │  └──抢占/取消：先 STOPPING（安全制动到停），再装载 pending 新指令
                 ▼
              STOPPING ──停稳且无 pending──► IDLE(原因=CANCELLED)
                 │
任意运动态 ──可恢复故障──► FAULT（确定性阻尼安全制动，反馈丢失则零力矩滑行）
任意运动态 ──不可恢复故障──► FATAL（断电；仅 Reconfigure 退出）
```

- 抢占采用"**先安全停旧、再起新**"：高优先级指令被仲裁接受后记为 pending，
  旧指令走完减速到停，再从**实际停止位置**重新规划 pending 目标（避免起点/实际不一致）。
- 安全制动在 FAULT 下使用由速度反馈直接计算的阻尼力矩 `τ = −50·v`（钳到 MaxTorque），
  不依赖原指令、不经 PID，保证确定性；无反馈时不做任何"盲动"，仅零力矩。

### 5.3 恢复前置条件（未满足前绝不静默继续）

`Recover()` 必须**同时**满足，否则返回 `RESET_PRECONDITIONS_NOT_MET`：

1. 当前为 `FAULT`（`FATAL` 只能 `Reconfigure`，否则同样被拒）；
2. 编码器反馈已恢复有效；
3. 各轴状态有限；
4. 各轴已停稳：|v| ≤ 1e-6 rad/s（即安全制动确实收敛）。

恢复成功只回到 `IDLE`，**不自动续跑**被中断轨迹；调用方需从实际位置下发新指令。
`Reconfigure()` 会重新校验配置（非法则拒绝且状态不变），随后重建规划器/对象/控制器
并清空仲裁水位，是 FATAL 的唯一出口。

## 6. 并发仲裁与状态一致性

- 引擎所有可变状态受**单把互斥锁**保护；`Submit/Step/Snapshot/Recover/Reconfigure`
  串行化，快照为深拷贝，外部永远读不到"半更新"的轴状态或进度。`go test -race` 通过。
- 序列号按 `Source` 分别计水位：
  - 已见过的 `(Source,Sequence)` 再次出现 → `DUPLICATE_COMMAND`（完成后仍受保护）；
  - 更旧/相等序列号 → `STALE_COMMAND`；二者都不会造成重复执行、丢步或状态回退。
  - 格式/前置条件错误（如取消目标不匹配、无故障时复位）**不推进水位**，允许更正后重试。
- 抢占：仅当新指令优先级**严格大于**在役指令优先级才接受，否则 `PRIORITY_DENIED`；
  平局在役者胜。
- 取消：只对"同 Source 且序列号等于 `CancelSeq` 的当前在役指令"生效；
  重复取消为 `DUPLICATE_COMMAND`，异源取消被拒。
- 故障锁存期间任何运动指令 → `FAULT_ACTIVE`；只有 `RESET`/恢复流程可推进。
- 配置版本不在 `[MinSupportedVersion, MaxSupportedVersion]`（当前仅 1）→
  `CONFIG_VERSION_UNSUPPORTED`，绝不静默升降级。

## 7. 被控对象模型（本地模拟执行器/编码器）

- 每轴为定常双积分：`α = (τ_applied + τ_gravity + τ_disturbance)/J`，
  以定步长半隐式欧拉积分（先更新速度再更新位置），确定性、无随机。
- `SetFeedbackValid(false)` 确定性地模拟编码器掉线，此时读数 `Valid=false`、不带数据。
- `SetDisturbance` 注入恒定扰动力矩，用于复现跟踪超限、饱和与发散。

## 8. 本地验证方法

```bash
go test ./...          # 全量（固定时钟、离线）
go test -race ./...    # 并发安全
go run ./cmd/demo      # 端到端场景；连续两次输出 diff 为空即复现
```

失败的测试断言统一经 `internal/motion/mctest` 打印三类信息：
断言名、完整**输入**、以及 got/want 或拒绝原因等**判定依据**，便于复盘。
