# 本地多轴运动控制与轨迹执行组件（Go）

一套完全离线、固定仿真时钟的多轴运动控制组件，用本机模拟执行器与编码器反馈，
使多轴联动、闭环纠偏、异常处置与并发指令仲裁产生**一致、可解释、可复现**的结论。
不依赖真实硬件、外部服务或凭据；唯一的第三方依赖是 Go 标准库与测试框架。

## 运行（本地离线验证）

```bash
# 构建
go build ./...

# 全部单元/集成测试（固定时钟，无随机、无墙钟依赖）
go test ./...

# 竞态检测（验证并发一致性）
go test -race ./...

# 端到端场景演示，输出逐字节可复现
go run ./cmd/demo > /tmp/a.txt && go run ./cmd/demo > /tmp/b.txt && diff /tmp/a.txt /tmp/b.txt
```

当前测试覆盖：约束越界、目标不可达/非有限、跟踪超限/饱和/反馈丢失/发散/非有限故障、
指令重复/迟到/乱序、抢占与取消、并发步进与读取。断言失败时会打印**输入与判定依据**
（见 `internal/motion/mctest`）。

## 包结构

| 包 | 职责 |
|----|------|
| `internal/motion/classify` | 可区分的拒绝原因码与结构化错误 `RejectError` |
| `internal/motion/config` | 版本化配置与严格校验（越界/版本不兼容直接拒绝，不兜底） |
| `internal/motion/kinematics` | 平面二连杆（Planar 2R）正/逆运动学，可达性与非有限判定 |
| `internal/motion/trajectory` | 时间同步的对称梯形速度规划，逐控制周期采样 |
| `internal/motion/plant` | 固定步长双积分被控对象（执行器）+ 可失效编码器 |
| `internal/motion/controller` | 关节空间前馈+PID 与本地故障检测 |
| `internal/motion/arbiter` | 每源序列号去重/防迟到、固定优先级抢占、取消/复位规则 |
| `internal/motion/engine` | 单互斥锁状态机：规划/执行/停车/故障/恢复/重配，状态快照 |
| `internal/motion` | 高层门面 `Robot`：笛卡尔位姿入口，IK→关节运动 |
| `cmd/demo` | 离线场景演示 |

## 为什么结论可复现

1. **固定仿真时钟**：执行只能通过 `engine.Step()`（恰好推进一个 `ControlPeriod`）
   或由它构成的 `engine.Run()` 推进；`RunTicked()` 仅用于实时演示，逻辑仍在 `Step`。
2. **确定性积分**：`plant` 采用定步长半隐式欧拉；无随机源、无 goroutine 调度依赖
   （并发入口只影响仲裁结果，物理步进被同一把锁串行化）。
3. **固定输入→固定轨迹**：规划在 t = k·Period 网格上严格采样，相同浮点位模式输入
   产生相同样本；对参数连续（周期数仅在定时边界增减 1）。
4. **无默认值兜底**：任何越界、非有限、自相矛盾或版本不兼容的输入都被明确拒绝。

## 拒绝原因（可机读、两两可区分）

定义见 `classify.Reason` 常量，均为 `*classify.RejectError`，可用 `classify.AsReject`
提取原因码与具体数值说明。包括：

`CONFIG_VERSION_UNSUPPORTED`、`CONFIG_FIELD_INVALID`、`TARGET_NON_FINITE`、
`TARGET_UNREACHABLE`、`OUT_OF_TRAVEL`、`VELOCITY_LIMIT_EXCEEDED`、
`ACCELERATION_LIMIT_EXCEEDED`、`TORQUE_LIMIT_EXCEEDED`、`CONTRADICTORY_TARGET`、
`STALE_COMMAND`、`DUPLICATE_COMMAND`、`PRIORITY_DENIED`、`CANCELLED`、
`FAULT_ACTIVE`、`RESET_PRECONDITIONS_NOT_MET`、`INVALID_COMMAND`。

## 故障与恢复语义（摘要）

| 故障 `FaultKind` | 触发依据 | 可恢复性 | 出路 |
|------------------|----------|----------|------|
| `FEEDBACK_LOST` | 编码器读数 `Valid=false` | 可恢复 | 安全制动停车后 `Recover()` |
| `FEEDBACK_TRACKING_ERROR` | 容差 < \|误差\| ≤ 100×容差 | 可恢复 | 安全制动停车后 `Recover()` |
| `DRIVE_SATURATION_PERSISTENT` | 力矩连续饱和达 `SaturationCycles` | 可恢复 | 安全制动停车后 `Recover()` |
| `CONTROL_DIVERGED` | \|误差\| > 100×容差，或末端保持预算内未收敛 | **不可恢复** | 仅 `Reconfigure()` 硬复位 |
| `NON_FINITE_STATE` | 指令/反馈/内部状态出现 NaN/Inf | **不可恢复** | 仅 `Reconfigure()` 硬复位 |

- 任何故障当周期起控制器输出**零力矩**；可恢复故障进入 `FAULT` 并执行基于速度反馈的
  确定性阻尼制动（反馈丢失时为零力矩滑行），`FATAL` 则彻底断电。
- `Recover()` 在**全部**前置条件满足前一律返回 `RESET_PRECONDITIONS_NOT_MET`：
  状态为 FAULT（非 FATAL）、编码器恢复有效、各轴有限且已停稳（|v| ≤ 1e-6 rad/s）。
- 恢复**不会**静默续跑被中断轨迹，而是回到 `IDLE`，由调用方从实际位置下发新指令；
  因此绝不会返回"未收敛/发散却成功"的结论。
