# Wuji 灵巧手振动分析与架构优化报告

**日期**: 2026-03-01
**系统**: wuji-teleop-ros2-private
**硬件**: Intel i9-13900HK (14C/20T, 5.4GHz), Manus Prime II 手套, Wuji 灵巧手
**软件**: ROS2 Humble, CycloneDDS, wujihandros2 驱动

---

## 1. 问题描述

灵巧手在遥操作模式下出现明显振动和异响。调试过程中发现：
- `filter_cutoff_freq` 从 **3Hz 切到 10Hz** 后，振动剧烈加重
- CPU 持续满载 (>100%)，控制时序不均匀
- 手部运动存在卡顿感

## 2. 数据链路全景

```
Manus 手套 (120Hz 硬件)
    │
    ▼
ManusSDK C++ (120Hz 定时器, 匹配硬件)
    │
    ▼
manus_input_py (120Hz → /hand_input)
    │
    ▼
wujihand_controller (100Hz consume-once → retarget + LP 滤波)
    │  lp_alpha=0.3 @ 100Hz → 截止 5.7Hz
    │  retarget ~6.6ms/帧 (双手)
    ▼
wujihand_driver (1000Hz, 硬件 LP 5Hz)
    │  /{hand_name}/joint_states (1000Hz, 数据录制用)
    ▼
灵巧手固件 (PID 位置控制)
```

## 3. 核心发现：两级低通滤波的频率交叉问题

### 3.1 滤波链结构

控制链路中存在 **两级串联低通滤波器**：

| 级别 | 位置 | 类型 | 截止频率 |
|------|------|------|----------|
| **第一级** | wujihand_controller (Python) | 指数移动平均 (EMA) | 取决于 lp_alpha 和采样率 |
| **第二级** | wujihand_driver (C++ 1000Hz) | 固件硬件滤波 | `filter_cutoff_freq` 参数 |

### 3.2 第一级 Retarget LP 滤波器特性

Retarget 内部使用 EMA 低通滤波：
```
y[n] = α · x[n] + (1-α) · y[n-1]
```

EMA 的 **等效截止频率** 与采样率耦合：
```
fc = -fs / (2π) · ln(1-α)
```

| lp_alpha | 采样率 fs | 等效截止频率 fc |
|----------|----------|----------------|
| 0.3 | 200Hz (满速) | 11.4Hz |
| 0.3 | 135Hz (CPU 过载) | 7.7Hz |
| 0.3 | **100Hz (定时器)** | **5.7Hz** |
| 0.3 | 50Hz (低速) | 2.8Hz |
| 0.2 | 135Hz | 4.8Hz |
| 0.2 | 100Hz | 3.6Hz |

**关键结论**：lp_alpha 不是一个频率规格，它是一个与采样率耦合的混合比。CPU 负载变化导致采样率波动时，截止频率会悄然漂移。

### 3.3 filter_cutoff 3Hz→10Hz 振动加剧的根本原因

```
信号频谱图 (对数坐标):

功率
 │
 │  ████                          ← 人手运动 (0-5Hz)
 │  ████
 │  ████████                      ← Retarget 量化噪声 + 计算误差
 │  ██████████████
 │  ████████████████████████████  ← 高频噪声底
 │
 └────┼─────┼─────┼─────┼────── 频率(Hz)
      3     5     8    10

                        Retarget LP
                       截止 ≈ 5.7Hz
                           │
      ┌────────────────────┤
      │  信号 + 噪声通过   │  噪声被衰减
      │                    │
      ▼                    ▼
  ■ filter_cutoff=3Hz:     ← 在 Retarget LP 之下
    第二级在 3Hz 切断 → 额外衰减 5.7→3Hz 之间的噪声
    结果: 双重滤波, 只有 <3Hz 通过 → 平滑但迟钝

  ■ filter_cutoff=10Hz:    ← 在 Retarget LP 之上
    第二级在 10Hz 切断 → 但 Retarget LP 只到 5.7Hz
    10Hz > 5.7Hz → 第二级没有额外过滤效果
    结果: 相当于只有一级 5.7Hz 滤波 → Retarget 噪声直通电机
```

**结论**：
- **filter_cutoff=3Hz 平滑的真正原因**：不是 3Hz 本身好，而是它在 Retarget LP (~5.7Hz) 之下，提供了额外的第二级噪声抑制
- **filter_cutoff=10Hz 振动的真正原因**：10Hz 高于 Retarget LP (~5.7Hz)，第二级滤波形同虚设，Retarget 计算噪声直接传递到电机
- **核心矛盾**：Retarget LP 才是真正的带宽瓶颈 (隐藏的 ~5.7Hz 截止)，driver 的 `filter_cutoff_freq` 只有低于它才有效果

### 3.4 解决思路

保持 `filter_cutoff_freq=10Hz` (避免引入相位延迟), 通过优化 Retarget LP 参数和控制架构来控制噪声:

| 参数 | 优化前 | 优化后 | 效果 |
|------|--------|--------|------|
| lp_alpha | 0.2 | **0.3** | 截止从 3.6Hz → 5.7Hz, 响应更快 |
| 控制频率 | ~135Hz (不稳定) | **100Hz (定时器)** | 截止稳定在 5.7Hz |
| filter_cutoff | 10Hz | 10Hz (不变) | 不引入额外相位延迟 |

## 4. CPU 瓶颈分析

### 4.1 问题：回调驱动架构 CPU 过载

旧架构 (回调驱动)：
```
/hand_input 200Hz → _hand_input_callback → retarget → 硬件
```

CPU 需求: 200帧/秒 × 6.6ms/帧 = **1320ms/秒 → 需要 132% CPU**

Python 单线程 (GIL) 上限 ~103%，实际表现:

| 指标 | 回调驱动 (旧) |
|------|-------------|
| CPU 占用 | 103-117% |
| 实际 retarget 率 | 135-154Hz (非确定性) |
| QoS 丢帧 | 49-65帧/秒 (25-33%) |
| 时序抖动 | 3-15ms (min/max 变化大) |

### 4.2 CPU 过载导致振动的三个机制

**机制 1: 时序抖动**
```
理想: |──10ms──|──10ms──|──10ms──|──10ms──|
实际: |──3ms──|──15ms──|──5ms──|──12ms──|──8ms──|
           ↑                 ↑
       CPU空闲积压        GC暂停延迟
```
不均匀的指令间隔 → 电机 PID 控制器看到不规则的位置阶跃 → 来回追踪 → 振动

**机制 2: QoS 不可控丢帧**
```
输入: f1 f2 f3 f4 f5 f6 f7 f8 f9 f10 ...
处理: f1 f2 f3 -- f5 -- f7 -- f9 f10 ...
                ↑     ↑     ↑
            随机丢帧 (取决于 GIL 调度)
```
随机丢帧 → 位置指令不连续 → 电机看到突变 → 振动

**机制 3: 200Hz 中 80帧是重复数据** (已修复: ManusSDK 改为 120Hz)
```
修复前: Manus 120Hz 真实数据 + ManusSDK 200Hz 轮询 = 80帧/秒重复
         每帧重复数据仍触发 retarget (6.6ms CPU) → 浪费 528ms/秒
修复后: ManusSDK 120Hz 匹配硬件 → 无重复帧
```

### 4.3 解法：100Hz 定时器 consume-once 架构

```
/hand_input 120Hz → callback: 仅存 _latest_raw (零计算)
                                      │
                         100Hz 定时器 (内核调度, 固定 10ms)
                                      │
                           ┌──────────┤
                           │ consume-once:
                           │ raw = _latest_raw
                           │ _latest_raw = None
                           │          │
                           │    retarget → 硬件
                           │          │
                           │   _publish_command
                           └──────────┘
```

**为什么 100Hz**:
- 每帧 retarget ~6.6ms, 100Hz → 660ms/秒 ≈ **66% CPU**
- 留 ~34% 给 Python GIL、GC、状态发布、回调
- lp_alpha=0.3 @ 100Hz → 截止 5.7Hz, 覆盖人手 0-5Hz 动态带宽

### 4.4 实测对比

| 指标 | 回调驱动 (旧) | 100Hz 定时器 (最终) | 改善 |
|------|-------------|-------------------|------|
| CPU 占用 | 103-117% | **~91%** | **-22%** |
| 命令输出频率 | 135-154Hz (不稳定) | **94-100Hz** | 确定性 |
| 时序抖动 (std dev) | ~2.0ms | **1.2ms** | **-40%** |
| min/max 间隔 | 3-15ms | **7-17ms** | 更均匀 |
| QoS 丢帧 | 49-65帧/秒 | **0** (consume-once) | 消除 |
| 状态发布 | 60Hz (被饿死) | **已删除** (driver 1000Hz 替代) | 精简 |
| 输入频率 | 200Hz (80帧重复) | **120Hz** (匹配硬件) | 无重复 |
| driver LP | 10Hz (形同虚设) | **5Hz** (二次降噪) | 有效 |

**注**: CPU ~91% 高于理论值 66% 的原因:
- 120Hz 输入回调 np.array 转换 (~6%)
- ROS2 单线程 executor 调度开销 (~15%)
- Python GIL 竞争 (~4%)
- 但 driver 5Hz LP 在 1000Hz 循环中运行, 完全消除 controller 端时序抖动

### 4.5 500Hz 重复发送实验 (已否决)

尝试将 retarget 结果以 500Hz 重复发送到 driver，期望减少 PID 位置阶跃间隔。

**失败原因 1: CPU 过载**
```
500Hz × 2手 × 0.15ms/消息 = 150ms/sec → +15% CPU
总计: 66% (retarget) + 15% (500Hz publish) + 25% (其他) = 106% → 超载
实测: CPU 99.8%, 命令输出仅 142Hz (应为 500Hz)
```

**失败原因 2: 重复发送无实际效果**
```
controller (100Hz) → ROS2 → driver (C++ 1000Hz) → firmware (LP 5Hz → PID)
                                   ↑
                         driver 已以 1000Hz 重复转发最新指令！
```

driver 内部 1000Hz 循环持续将最近一次收到的指令转发到固件。
无论 controller 以 100Hz 还是 500Hz 发送**相同值**，firmware 看到的信号相同。

**正确路径**: 提升实际 retarget 频率 (C++/Rust 重写, 预估 1000Hz+), 而非重复旧值。

## 5. 架构瓶颈与拆分建议

### 5.1 当前单机架构的局限

即使优化到 100Hz 定时器架构, 单核 Python retarget 仍是瓶颈:

```
retarget 单帧耗时: ~6.6ms (双手)
100Hz 定时器:       100 × 6.6ms = 660ms → 66% CPU
120Hz (理论最优):   120 × 6.6ms = 792ms → 79% CPU  ← 接近极限
```

当前 i9-13900HK 单核可以支撑 100Hz，但:
- 无法提升到 120Hz+ (接近饱和)
- 低功耗移动平台 (如遥操小车) 更难支撑
- Python GIL 限制了多核利用

### 5.2 建议：控制机 + 执行机拆分

```
┌─ 控制机 (高性能 CPU) ───────────────────────────┐
│                                                   │
│  Manus 手套 ──→ manus_input ──→ retarget ──→     │
│                                    ↓              │
│                             /retarget_output      │
│                          (post-retarget joints)   │
│                                    │              │
│  也可作为 rollout 推理机           │              │
│  (retarget + policy inference)     │              │
└────────────────────────────────────┼──────────────┘
                                     │ DDS / CycloneDDS
                                     │ (局域网 <1ms)
┌─ 执行机 (小车/轻量设备) ──────────┼──────────────┐
│                                    │              │
│  订阅 /retarget_output ──→ driver ──→ 灵巧手     │
│                                                   │
│  职责: 仅转发关节指令到硬件                      │
│  CPU 需求: <10% (无 retarget 计算)               │
│                                                   │
│  优势:                                            │
│  - 小车可直接推出去操作                           │
│  - 无需高性能 CPU                                 │
│  - 可同时接收 rollout 推理结果                    │
└───────────────────────────────────────────────────┘
```

**拆分收益**:

| 维度 | 单机方案 | 拆分方案 |
|------|---------|---------|
| 执行机 CPU | 75% (retarget+driver) | **<10%** (仅 driver) |
| 移动性 | 受限于高性能主机 | 小车即可 |
| Retarget 频率 | 受限于单核 (~100Hz) | 可升级 CPU 独立扩展 |
| 多任务 | retarget 与 policy 争抢 CPU | 同一机器跑 retarget + rollout |
| 延迟 | 0 (进程间) | +~1ms (CycloneDDS 局域网) |

### 5.3 实现路径

1. **控制机 (已有 INFERENCE 模式基础)**:
   - 新增 retarget-only 节点: 订阅 /hand_input, 发布 /wuji_hand/{left,right}/joint_command
   - 复用现有 `wujihand_controller` 的 retarget 逻辑
   - 通过 CycloneDDS multicast 跨机发布

2. **执行机 (已有 INFERENCE 模式)**:
   - `wujihand_controller` 切换为 INFERENCE 模式
   - 订阅 /wuji_hand/{left,right}/joint_command → 直接转发到 driver
   - 无需修改 driver 代码

3. **通信**:
   - 两机同 `ROS_DOMAIN_ID`, 同 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
   - CycloneDDS multicast 自动发现, 无需手动配置 peer
   - 关节指令数据量极小 (~20 floats/帧), 带宽需求可忽略

## 6. 参数最终状态

| 参数 | 文件 | 值 | 说明 |
|------|------|-----|------|
| lp_alpha | retarget_manus_{left,right}.yaml | **0.3** | @ 100Hz → 截止 5.7Hz |
| norm_delta | retarget_manus_{left,right}.yaml | **0.04** | 速度正则化 |
| CONTROL_RATE_HZ | wujihand_node.py | **100.0** | 定时器驱动 |
| filter_cutoff_freq | wujihand_ik.yaml | **5.0** | 硬件 LP, 略低于 retarget LP 提供二次降噪 |
| ManusSDK timer | ManusDataPublisher.cpp | **8333μs (120Hz)** | 匹配手套硬件刷新率 |
| publish_rate_hz | manus_input.yaml | **120.0** | 匹配 ManusSDK |

### 6.1 已删除/精简

| 项目 | 原值 | 变更 | 原因 |
|------|------|------|------|
| _publish_state | 100Hz | **已删除** | driver 已 1000Hz 发布 /{hand_name}/joint_states |
| /wuji_hand/*/joint_state | controller 发布 | **已删除** | 与 driver 话题重复 |
| ManusSDK 200Hz 重复帧 | 80帧/秒浪费 | **消除** | SDK 改为 120Hz 匹配硬件 |

### 6.2 wuji-data-record 配置同步更新

| 录制话题 (旧) | 录制话题 (新) | 来源 | 频率 |
|--------------|-------------|------|------|
| /wuji_hand/left/joint_state | **/left_hand/joint_states** | driver | 1000Hz |
| /wuji_hand/right/joint_state | **/right_hand/joint_states** | driver | 1000Hz |
| /wuji_hand/left/joint_command | (不变) | controller | 100Hz |
| /wuji_hand/right/joint_command | (不变) | controller | 100Hz |

## 7. 遗留问题

1. **PID 自激振荡** (~1° 峰峰值, 3-5Hz): 固件 PID 增益问题, 需固件团队调优 (参见对象字典 0x05 分析)
2. **Retarget 性能**: 单帧 6.6ms 是 Python + NLopt 的瓶颈, C++/Rust 重写可提速 5-10x
3. **CPU 占用 ~91%**: Python GIL 限制了单核利用效率, 考虑 C++ retarget 或多机拆分
4. **高频指令**: 500Hz 重复发送无效 (driver 已 1000Hz 转发), 需 C++ retarget 真正提升计算频率

---

*报告由 Claude Code 协助生成, 基于 2026-03-01 实测数据*
