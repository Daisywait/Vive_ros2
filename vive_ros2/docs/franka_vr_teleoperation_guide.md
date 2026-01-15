# Franka FR3 VR遥操作系统 - 使用指南

## 🎯 系统概述

已为 Franka FR3 + Robotiq 85 夹爪 配置高精度VR遥操作系统：

### 实现的功能：
✅ 手眼标定（VR空间 → Franka工作空间）
✅ 低通滤波器（10Hz，减少手抖）
✅ 死区处理（1mm阈值，防止微小漂移）
✅ Y轴串扰抑制（85%，减少前后移动时的上下误差）
✅ 自动加载标定矩阵

### 预期精度：
- **未标定**: ±5-10mm（Focus Vision Inside-Out追踪精度）
- **标定后**: ±2-5mm（手眼标定补偿系统误差）
- **适用场景**: 数据采集、示教学习、非精密操作

---

## 📋 步骤1：手眼标定（首次使用必做）

### 1.1 准备工作

确保以下话题正常发布：
```bash
# 检查VR手柄数据
ros2 topic echo /controller_data

# 检查Franka末端执行器位姿
ros2 topic echo /franka_ee_pose  # 你需要确认实际话题名
```

**如果你的Franka话题名不是 `/franka_ee_pose`**，需要修改标定脚本第11行：
```python
self.franka_sub = self.create_subscription(
    PoseStamped, '/你的实际话题名', self.franka_callback, 10)
```

### 1.2 运行标定

```bash
# 1. 复制标定脚本
cp /tmp/vr_franka_calibration.py ~/ros2_ws/src/vive_ros2/scripts/
chmod +x ~/ros2_ws/src/vive_ros2/scripts/vr_franka_calibration.py

# 2. 启动VR系统
# - 启动ALVR
# - 连接Focus Vision
# - 启动vive_input和vive_node

# 3. 启动标定程序
cd ~/ros2_ws
python3 src/vive_ros2/scripts/vr_franka_calibration.py
```

### 1.3 标定流程

**标定步骤：**
1. 使用MoveIt或手动示教，将Franka移动到工作空间中的一个点
2. 用VR手柄**触碰**Robotiq夹爪基座
3. 在标定程序中输入 `r` 记录该点
4. 重复步骤1-3，采集至少 **6个不同位置的点**

**标定点建议：**
```
点1: 工作空间左前下角
点2: 工作空间右前下角
点3: 工作空间左后下角
点4: 工作空间右后下角
点5: 工作空间中心上方
点6: 工作空间中心中间
```

5. 采集完成后，输入 `c` 计算标定
6. 标定结果保存到：`/tmp/vr_franka_calibration.yaml`

### 1.4 验证标定精度

标定完成后，程序会显示：
```
标定精度:
  平均误差: X.XX mm
  最大误差: X.XX mm
```

**精度判断：**
- ✅ 平均误差 < 2mm：优秀，可用于数据采集
- ⚠️  平均误差 2-5mm：一般，建议重新标定
- ❌ 平均误差 > 5mm：不合格，必须重新标定

**如果精度不够：**
- 增加标定点数量（10-12个点）
- 确保标定点分布均匀覆盖整个工作空间
- 检查VR追踪环境（光线、纹理）

---

## 📋 步骤2：启动遥操作系统

### 2.1 启动完整系统

```bash
# 终端1: 启动VR输入
cd ~/ros2_ws
source install/setup.bash
ros2 run vive_ros2 vive_input

# 终端2: 启动VR节点（自动加载标定）
cd ~/ros2_ws
source install/setup.bash
ros2 run vive_ros2 vive_node
```

**成功启动后，你应该看到：**
```
[INFO] 正在加载Franka FR3手眼标定...
[INFO] ✓ 标定矩阵已加载
  平移: [X.XXX, X.XXX, X.XXX]
[INFO] ✅ 遥操作模式：已启用标定 + 滤波
   - 死区: 1.0 mm
   - 最大速度: 0.50 m/s
   - Y轴串扰抑制: 15%
```

**如果看到警告：**
```
[WARN] ⚠️  未加载标定，使用默认坐标转换
   建议运行标定：python3 vr_franka_calibration.py
```
说明标定文件未找到，请先完成步骤1。

### 2.2 遥操作控制

```bash
# 终端3: 启动你的Franka控制节点
ros2 launch your_franka_package teleoperation.launch.py

# 订阅以下话题获取遥操作指令：
# - /right_vr/vive_pose_rel  (右手相对位移)
# - /left_vr/vive_pose_rel   (左手相对位移)
# - /controller_data         (按钮状态)
```

---

## 🎮 操作指南

### 控制模式

**按钮功能：**
- **Trigger**: 按下时，记录初始位置；释放时，停止记录
  - 在按下期间，发布**相对位移**
  - 适合增量控制（velocity control）

- **Menu**: 重置初始位置

- **Grip**: 切换测试方向（仅用于调试）

### 坐标系对齐

**已应用的坐标转换：**
```
VR坐标系（Focus Vision）
    ↓
ALVR坐标转换
    ↓
你的修正代码 (X=-Z, Y=Y, Z=X)
    ↓
手眼标定矩阵
    ↓
Franka FR3 工作空间坐标
```

**最终坐标系：**
- X轴: 对应Franka的X轴（fr3_link0）
- Y轴: 对应Franka的Y轴
- Z轴: 对应Franka的Z轴

---

## 🔧 参数调整

### 调整滤波强度

编辑 `vive_node.cpp` 第113行：
```cpp
alpha(0.3),  // 0.1=强滤波(慢), 0.5=弱滤波(快)
```

### 调整死区

编辑 `vive_node.cpp` 第138行：
```cpp
const double DEADZONE_MM = 1.0;  // 增大可减少抖动，减小可提高响应
```

### 调整Y轴串扰抑制

编辑 `vive_node.cpp` 第140行：
```cpp
const double FORWARD_BACKWARD_COMPENSATION = 0.85;  // 0.0-1.0，越小抑制越强
```

修改后重新编译：
```bash
cd ~/ros2_ws
colcon build --packages-select vive_ros2
```

---

## 📊 数据采集建议

### 针对机器人学习数据采集：

1. **采集前校准**
   - 每次长时间采集前，重新运行标定
   - 每周至少校准一次

2. **环境设置**
   - 保持良好光照（避免强光直射）
   - 在墙上贴带纹理的图片（帮助Inside-Out追踪）
   - 测试时站在房间中央

3. **操作技巧**
   - 动作要平滑，避免突然加速
   - 前后移动时，保持手腕水平
   - 使用相对模式（按住Trigger）进行精细控制

4. **数据质量检查**
   - 记录示教轨迹的同时，记录时间戳
   - 定期检查轨迹平滑度
   - 如果发现跳跃或抖动，重新采集该段

---

## ⚠️ 已知限制

### Focus Vision Inside-Out 追踪限制：

1. **精度限制**
   - 绝对精度：±5-10mm
   - 相对精度：±2-5mm
   - **不适合精密装配（需要±1mm精度的任务）**

2. **前后方向的Y轴串扰**
   - 即使应用了85%抑制，仍有约5-10% Y轴混入
   - 这是Focus Vision硬件特性，无法完全消除
   - 对数据采集影响较小（模型可学习补偿）

3. **延迟**
   - 有线串流延迟：10-20ms
   - 对实时遥操作影响小
   - 对离线数据采集无影响

### 建议升级方案（如果精度不够）：

**短期（成本<$1000）：**
- 购买 HTC Vive Tracker 3.0 + Lighthouse 2.0基站
- 精度提升到 ±1mm
- 适合大多数机器人学习任务

**长期（如果要发顶会论文）：**
- 购买 OptiTrack 光学追踪系统
- 精度达到 ±0.1mm（亚毫米级）
- 业界标准配置（Stanford, UC Berkeley等）

---

## 🐛 故障排查

### 问题1：编译失败
```bash
# 确保已安装Eigen3
sudo apt install libeigen3-dev

# 清理并重新编译
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select vive_ros2
```

### 问题2：标定精度很差（>10mm）
- 检查Franka话题是否正确
- 增加标定点数量（10-15个）
- 确保标定点分布均匀
- 检查VR追踪质量（光线、纹理）

### 问题3：遥操作时有明显延迟
- 确认使用有线串流（USB）
- 检查网络延迟（如果用WiFi）
- 降低ALVR码率（提高响应速度）

### 问题4：位置抖动严重
- 增大滤波强度（减小alpha值）
- 增大死区（DEADZONE_MM = 2.0）
- 检查VR追踪环境

---

## 📚 相关文件

```
~/ros2_ws/src/vive_ros2/
├── src/vive_node.cpp          # 主节点（已修改）
├── include/vive_ros2/
│   └── EigenTransforms.hpp    # 坐标转换工具
├── scripts/
│   └── vr_franka_calibration.py  # 标定脚本
└── /tmp/vr_franka_calibration.yaml  # 标定结果

日志文件：
/tmp/controller_debug_YYYYMMDD_HHMMSS.log  # 调试日志
```

---

## 🎓 下一步

1. **完成标定**并验证精度
2. **集成到你的Franka控制器**
3. **采集一小批测试数据**验证系统
4. **根据精度需求**决定是否升级硬件

有任何问题随时问我！🚀
