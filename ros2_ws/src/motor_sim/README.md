# motor_sim：电机模拟器 ROS2 包

## 任务说明

使用 ROS2 节点实现一个直流电机模拟器：
- 输入：控制力矩 `T_cmd`
- 输出：电机当前角速度 `ω`、当前角度 `θ`（零点自取）
- 二阶系统模型（带阻尼）

## 物理模型

二阶直流电机动力学方程：

$$J \dot{\omega} = T_{cmd} - B\omega$$
$$\dot{\theta} = \omega$$

其中：
- `J`：转动惯量（kg·m²）
- `B`：阻尼系数
- `T_cmd`：控制力矩（输入）
- `ω`：角速度（输出）
- `θ`：角度（输出）

数值积分采用**半隐式欧拉法**（先更新角速度，再用新角速度更新角度），保证数值稳定性。

## 节点说明

| 节点 | 作用 | 发布/订阅话题 |
| --- | --- | --- |
| `motor_sim_node` | 电机模拟器 | 订阅 `/torque_cmd`，发布 `/motor/omega`、`/motor/theta` |
| `torque_pub_node` | 测试信号源 | 发布 `/torque_cmd`，支持阶跃/方波/正弦 |

## 参数

### motor_sim_node
- `J`：转动惯量，默认 1.0
- `B`：阻尼系数，默认 0.5
- `update_rate`：状态更新频率（Hz），默认 1000

### torque_pub_node
- `signal_type`：信号类型，可选 `step` / `square` / `sine`
- `amplitude`：信号幅值（力矩大小 N·m），默认 2.0
- `freq`：方波/正弦频率（Hz），默认 0.5
- `pub_rate`：发布频率（Hz），默认 100
- `step_delay`：阶跃信号延迟时间（秒），默认 1.0

## 频率分配

- 电机状态更新：**1000 Hz**（1ms 步长，数值积分准确）
- 控制/输入源发布：**100 Hz**（控制频率，电机物理响应慢，无需太高）

## 运行方法

```bash
# 终端 1：电机模拟器
cd /workspaces/ros2-develop/ros2_ws
source install/setup.zsh
ros2 run motor_sim motor_sim_node

# 终端 2：测试信号源（默认阶跃）
ros2 run motor_sim torque_pub_node

# 切换为方波
ros2 run motor_sim torque_pub_node --ros-args -p signal_type:=square

# 切换为正弦
ros2 run motor_sim torque_pub_node --ros-args -p signal_type:=sine

# 终端 3：Foxglove 桥
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## 测试方案与结果

### 1. 阶跃信号（step）

- 输入：t > 1s 后突然施加 2.0 N·m 恒定力矩
- 预期：角速度指数上升，最终稳定在 ω = T/B = 2/0.5 = 4 rad/s；角度匀速增加
- 实际：与理论一致 ✅

### 2. 方波信号（square）

- 输入：±2.0 N·m 交替方波，频率 0.5 Hz
- 预期：角速度三角波（正负交替），角度来回摆动
- 实际：与理论一致 ✅

### 3. 正弦信号（sine）

- 输入：2.0 N·m 幅值正弦力矩，频率 0.5 Hz
- 预期：角速度和角度也是正弦，但振幅衰减、相位滞后
- 稳态角速度振幅理论值：A / √(B² + (Jω)²) = 2 / √(0.5² + (π)²) ≈ 0.63 rad/s
- 实际：约 ±0.65 rad/s，与理论一致 ✅

## Foxglove 可视化

在 Foxglove（`ws://localhost:8765`）中添加 Plot 面板，绘制：
- `/torque_cmd.data`（输入力矩）
- `/motor/omega.data`（角速度）
- `/motor/theta.data`（角度）

三条曲线对比即可直观看到电机的动态响应。
