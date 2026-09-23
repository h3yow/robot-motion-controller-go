# 本地多轴运动控制与轨迹执行组件（Go）

一个完全本地、离线、可复现的多轴运动控制组件：在本机用**模拟执行器 + 编码器反馈**
实现多轴联动轨迹规划、闭环纠偏、故障安全处置，以及多入口并发指令的仲裁。
不依赖真实硬件、网络、外部服务或任何凭据。

- 语言/模块：Go 1.25+（在 go1.26 上验证），module `github.com/h3yow/robot-motion-controller-go`
- 固定仿真时钟：默认控制周期 `2 ms`；测试与 demo 使用**手动时钟**逐周期驱动，结果逐位可复现。

## 目录结构

| 包 | 职责 |
| --- | --- |
| `internal/mcerr` | 稳定、可机读的错误/故障分类码（拒绝原因不靠解析文本区分） |
| `internal/numeric` | 有限性检查、夹取等确定性浮点工具 |
| `internal/config` | JSON 配置解析与**严格校验**（拒绝未知字段、越界值、不兼容版本，缺参不兜底） |
| `internal/kinematics` | 3 自由度平面臂解析正解/逆解、可达性判定、肘部分支 |
| `internal/trajectory` | 多轴同步梯形规划，逐控制周期采样位置/速度/加速度/前馈力矩 |
| `internal/sim` | 本地被控对象：刚体积分 + 库仑摩擦 + 重力项 + 硬限位 + 扰动/力矩衰减注入 |
| `internal/control` | 前馈 + PID 闭环、收敛检测、安全停车/安全保持、显式可恢复/致命故障状态机 |
| `internal/arbiter` | 优先级抢占、同优先级 FIFO、按入口单调序号去重/判旧、终态幂等 |
| `internal/motion` | 单 goroutine 事件循环，把上述组件组装成并发安全的运动服务 |
| `cmd/demo` | 端到端离线场景演示 |
| `configs/arm3.json` | 示例配置 |

## 运动学与量纲约定（SI）

全部采用关节空间 + 平面笛卡尔的统一约定：

- 角度一律**弧度（rad）**，逆时针为正；长度**米（m）**；时间**秒（s）**。
- 关节速度 `rad/s`、关节加速度 `rad/s²`、力矩 `N·m`、转动惯量 `kg·m²`。
- 机构为 3 自由度**全转动关节平面臂**，连杆长 `l1,l2,l3`（均 > 0）。
- 正解：
  - `x = Σ l_k·cos(θ_k)`，`y = Σ l_k·sin(θ_k)`，末端姿态 `φ = Σ q_i`，
    其中 `θ_k = Σ_{j≤k} q_j` 为各连杆在世界系中的绝对角。
- 逆解（闭式）：
  - 先用 `l3` 沿末端朝向从目标点回退得到腕心 `(rx, ry)`；
  - 腕心到原点距离 `r` 必须落在可达环带 `[|l1−l2|, l1+l2]` 内，否则
    `target_unreachable`；边界用极小数值容差，不放宽真实越界；
  - 余弦定理解出肘关节，`elbow` 配置（`up`/`down`）决定肘弯方向；
  - 输出角按等价角（加减 `2π`）平移到离 seed（当前关节角）最近的表示，
    因此关节空间运动小、目标连续变化时解连续。
- 笛卡尔目标若解出的关节角超出该轴行程，进一步归类为 `joint_limit_exceeded`。

## 轨迹规划（逐控制周期、多轴同步）

- 每个目标生成固定周期网格上的样本序列 `Point{Time, Position, Velocity, Accel, Torque}`。
  每个轴都从同一 `t=0` 开始、在同一最终采样点静止到达目标（**联动同步**）。
- 单轴曲线支持**非零初速度**（发生在抢占/排队命令激活时）：
  先以最大减速度刹停到静止点，再从静止点走“梯形/三角形”到位曲线。
- 多轴同步：取所有轴最小耗时的最大值并向上取整到周期网格；其余轴通过**降低巡航速度**
  拉伸到相同时长，绝不提高加速度峰值。
- 前馈力矩：`τ_ff = J·a + G·sin(q) + F·sign(v)`（惯性 + 重力 + 库仑摩擦）。
- 显式拒绝（不做默认值兜底、不静默放宽约束），原因可区分：
  - 起点/目标含 `NaN/Inf` → `target_non_finite`
  - 目标超出关节行程、或安全刹车会冲出限位 → `joint_limit_exceeded`
  - 起点速度超速 → `infeasible_start`
  - 速度/加速度/力矩需求超上限 → `velocity_limit_exceeded / acceleration_limit_exceeded / torque_limit_exceeded`
  - 轴数/参数自相矛盾 → `target_contradictory`
  - 笛卡尔不可达 → `target_unreachable`
- 确定性：相同输入（起点、初速度、目标、限值、周期）产出逐位相同的样本，
  不读取时钟、不使用 map 迭代序；结果随目标连续变化。

## 控制周期、闭环与容差

- 控制律（每轴，固定周期 `dt`，误差 `e = q_target − q_measured`）：
  - 积分项带抗饱和限幅：`integral += clamp(k_i·e·dt, ±I_lim)`
  - `τ = τ_ff + k_p·e + integral + k_d·(v_target − v_measured)`，再夹到 `±τ_max`。
- 收敛判据（`tolerance`）：轨迹参考结束后，所有轴
  `|e| ≤ position` 且 `|v| ≤ velocity` 并连续保持 `hold_ticks` 个周期，方判定完成。
- 若参考结束后 `end_grace_ticks` 内仍未收敛 → 致命故障 `not_converged`
  （绝不返回未收敛/发散却看似成功的结论）。

### 本地被控对象模型

`J·dv/dt = scale·τ_cmd − F·sign(v) − G·sin(q) + τ_load`，`dq/dt = v`，
对力矩做 `±τ_max` 夹取、对位置做硬行程限位（撞限位时速度清零）。
可注入：外部扰动力矩 `WithLoadTorque`、执行器力矩衰减 `WithTorqueScale`
（`0` 表示完全丢力矩）、编码器掉电 `WithFeedbackDrop`。

## 故障与恢复语义

故障分两类，状态与命令结果都明确区分，绝不静默继续：

| 触发 | 故障 | 类别 |
| --- | --- | --- |
| 编码器读数缺失（`feedbackOK=false`） | `feedback_lost` | 可恢复 |
| 掉电持续超过 `dropout_max_ticks` | `feedback_dead` | 致命 |
| `|e| > max_position_error` | `tracking_error` | 可恢复 |
| `|e| > diverge_error`（含故障观察期间恶化越线） | `tracking_diverged` | 致命 |
| 连续 `saturation_max_ticks` 个周期力矩饱和 | `drive_saturated` | 可恢复 |
| 指令/反馈出现非有限值 | `command_non_finite` | 致命 |
| 结束宽限内未收敛 | `not_converged` | 致命 |

- **可恢复故障期间进入“安全保持”**：以当前参考位置为目标施加纯 P/D 停车/定位力矩
  （无积分、无前馈，缺编码器的轴不加力），把轴平稳停在参考点而非自由滑行。
- **致命故障立即卸力（零力矩）**，且在故障未清除前拒绝一切新运动（`fault_active`）。
- 恢复必须显式调用 `Recover`，且前置条件**同时**满足并保持 `recovery_hold_ticks` 周期：
  所有轴反馈健康、`|e| ≤ 0.5·max_position_error`、`|v| ≤ tolerance.velocity`。
  条件不满足返回 `recovery_precondition_unmet`；致命故障调用 `Recover` 返回
  `fault_active` 并说明不可恢复（需复位/回零）。恢复成功后从剩余轨迹继续执行至目标。
- 取消（`Cancel`）触发以 `safe_decel` 减速度的受控安全停车，停稳后命令记为 `canceled`。

## 并发指令仲裁与状态一致性

- 单个 goroutine（事件循环）拥有控制器/被控对象/仲裁器的可变状态；公共方法通过带缓冲
  作业通道提交并同步等待。手动时钟下由唯一驱动方逐周期 `Step`。
- 抢占：严格更高优先级立即抢占当前命令（被抢占命令终态 `preempted`，不再恢复）；
  同级或更低优先级进入 FIFO 队列；队列内按优先级、再按到达顺序弹出。
- 每个入口的 `Seq` 必须严格递增；更小或相等序号判为迟到/乱序，拒绝为 `stale_sequence`。
- 相同 `(ID, entrance, seq)` 的重发幂等（不重复执行）；ID 复用但内容不同 →
  `duplicate_command`。终态一旦写入不可回退，迟到的完成通知被忽略。
- **轨迹在命令“激活时”才按当前实测状态规划**（而非提交时），保证排队/抢占命令激活时
  曲线与真实状态一致，不会错步。
- 轴状态快照 `Axes()` 与相位/故障在控制器单临界区内拷贝，读不到半更新状态。
- 配置兼容性：仅接受 `MAJOR.MINOR.PATCH` 且主版本在 `[1,1]` 内；越界版本
  → `config_version_unsupported`；未知 JSON 字段直接拒绝。

## 本地验证方法（离线、无外部依赖）

```bash
# 构建 / 静态检查
go build ./...
go vet ./...

# 全部单元 + 端到端测试（含竞态检测）
go test -race -count=1 ./...

# 端到端演示：名义联动 / 拒绝 / 反馈故障与恢复 / 抢占与取消 / 确定性复现
go run ./cmd/demo
```

测试重点（断言失败会打印输入与判定依据）：

- `internal/config`：不兼容版本、未知字段、缺参、物理量越界均被明确归类拒绝。
- `internal/kinematics`：正逆解互逆、不可达、非有限、肘部分支、解的连续性。
- `internal/trajectory`：越界拒绝、速度/加速度/力矩峰值、非零初速度刹车、
  多轴同步收尾、可复现性、随目标连续、多点路径逐点静止。
- `internal/sim`：确定性、静摩擦保持、硬限位、力矩衰减、非有限指令拒绝。
- `internal/control`：正常收敛、掉电（可恢复→持续致命）、跟踪发散（致命）、
  驱动饱和（可恢复）、取消安全停车、恢复前置条件、致命故障拒绝加载。
- `internal/arbiter`：抢占、FIFO、乱序/迟到/重复、取消、终态幂等、并发一致性（`-race`）。
- `internal/motion`：多轴联动到位、笛卡尔拒绝、反馈故障后恢复并完成、致命故障冻结队列、
  抢占/取消、20 路并发提交 + 并发快照读 + 单时钟驱动、确定性复现。

## 配置说明（见 configs/arm3.json）

`period` 为控制周期字符串（如 `"2ms"`）；`links` 与 `axes` 数量必须都为 3；
每个轴必须显式给出 `min/max` 行程、`max_velocity/max_accel/max_torque`、
`inertia` 与 PID `gains`。被动负载 `gravity_torque + friction_torque` 必须严格小于
`max_torque`，否则任何运动都不可行，配置在加载期即被拒绝。所有阈值
（`tolerance`、`fault.*`）都必须为正且 `diverge_error > max_position_error`。
