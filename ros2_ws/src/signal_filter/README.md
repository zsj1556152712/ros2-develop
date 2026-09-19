# signal_filter：信号滤波 ROS2 包

## 任务内容

生成一个 20Hz 正弦信号（幅值可自定义），叠加标准差为信号幅值 1% 的高斯噪声，以约 1000Hz 频率发布；订阅者节点分别用**低通滤波器**和**中值滤波器**处理原始信号，并用 Foxglove 可视化前后对比。

## 节点说明

| 节点 | 作用 | 发布/订阅话题 |
| --- | --- | --- |
| `signal_generator` | 生成 20Hz 正弦 + 1% 高斯噪声，1000Hz 发布 | 发布 `/signal_raw` |
| `filter_node` | 对原始信号做低通 + 中值滤波 | 订阅 `/signal_raw`，发布 `/signal_lpf`、`/signal_median` |

## 两种滤波器的适用场景

- **低通滤波器**：适合滤除高频随机噪声（比如传感器轻微抖动、电路白噪声）。它的输出平滑、连续，但会有一点相位延迟，信号变化快的时候响应会慢半拍。实际中常用在 IMU 姿态、电机转速这类需要稳定平滑信号的场合。
- **中值滤波器**：适合滤除脉冲毛刺（突然跳一下又回来的离群点，比如传感器偶发干扰）。它把滑动窗口里的数排序取中间值，极端值基本进不了中间位置，所以对尖峰特别有效，而且不损失幅值、没有相位延迟。缺点是输出是一格一格的阶梯状，不够平滑。

## 运行方法

```bash
# 终端 1：发生器
cd /workspaces/ros2-develop/ros2_ws
source install/setup.zsh
ros2 run signal_filter signal_generator

# 终端 2：滤波器
ros2 run signal_filter filter_node

# 终端 3：Foxglove 桥
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

然后在 Foxglove（`ws://localhost:8765`）中添加 Plot 面板，绘制 `/signal_raw.data`、`/signal_lpf.data`、`/signal_median.data` 三条曲线对比。

## 可视化效果

- 蓝色：原始信号（带噪声，毛刺多）
- 橙色：中值滤波后（毛刺被削平，呈阶梯状）
- 黄色：低通滤波后（最平滑，但略有延迟）
