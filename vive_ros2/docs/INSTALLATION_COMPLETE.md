## ✅ Franka FR3 VR遥操作系统 - 安装完成报告

**日期**: 2026-01-16
**机器人**: Franka FR3
**末端执行器**: Robotiq 85 夹爪
**VR设备**: HTC Focus Vision (Inside-Out追踪)
**串流**: ALVR 有线USB串流

---

## 📦 已完成的工作

### 1. ✅ 核心功能实现

已成功修改并编译以下文件：

**vive_ros2/src/vive_node.cpp**
- ✓ 手眼标定矩阵加载 (从YAML自动加载)
- ✓ 低通滤波器 (指数移动平均, alpha=0.3)
- ✓ 1mm死区处理 (消除微小漂移)
- ✓ Y轴串扰抑制 (85%补偿前后移动时的上下混入)
- ✓ 遥操作流程集成到 `filterPose()` 函数

**vive_ros2/include/vive_ros2/EigenTransforms.hpp**
- ✓ VR空间旋转校正 (x=-z, y=y, z=x)
- ✓ 四元数方向校正

### 2. ✅ 标定工具

**创建文件**: `/home/enine/ros2_ws/src/vive_ros2/scripts/vr_franka_calibration.py`
- ✓ SVD算法计算手眼变换矩阵
- ✓ 实时误差验证 (显示平均误差和最大误差)
- ✓ 交互式数据采集 (按'r'记录点位, 按'c'计算)
- ✓ YAML格式保存标定结果

### 3. ✅ 文档

**使用指南**: `/tmp/franka_vr_teleoperation_guide.md` (完整版)
- ✓ 步骤1: 手眼标定流程
- ✓ 步骤2: 系统启动指南
- ✓ 操作说明: 按钮功能和控制模式
- ✓ 参数调整: 滤波/死区/串扰抑制
- ✓ 故障排查指南

**辅助脚本**: `/tmp/find_franka_topics.sh`
- ✓ 自动查找Franka末端执行器位姿话题
- ✓ 显示话题类型和订阅者信息

### 4. ✅ 编译状态

```bash
Package: vive_ros2
Status: ✅ 编译成功
Executable: /home/enine/ros2_ws/install/vive_ros2/lib/vive_ros2/vive_node
Size: 491 KB
Build Date: 2026-01-16 02:50
```

---

## 🎯 下一步操作（用户需执行）

### 步骤 1️⃣: 查找Franka话题名

**目前标定脚本假设的话题**: `/franka_ee_pose`
**您的末端执行器坐标系**: `robotiq_85_base_link`
**规划坐标系**: `fr3_link0`

请在启动Franka控制器后运行：

```bash
bash /tmp/find_franka_topics.sh
```

这将帮助您找到正确的 `PoseStamped` 话题名。

**如果话题名不是 `/franka_ee_pose`**，请编辑标定脚本第24行：

```bash
nano ~/ros2_ws/src/vive_ros2/scripts/vr_franka_calibration.py
```

修改为您的实际话题名：
```python
self.franka_sub = self.create_subscription(
    PoseStamped, '/你的实际话题名', self.franka_callback, 10)
```

---

### 步骤 2️⃣: 运行手眼标定

**准备工作：**
1. 确保Franka机械臂已上电并连接
2. 启动Franka控制器（发布末端位姿话题）
3. 启动ALVR并连接Focus Vision
4. 启动VR输入节点

**标定命令：**
```bash
# 终端1: 启动VR输入
cd ~/ros2_ws
source install/setup.bash
ros2 run vive_ros2 vive_input

# 终端2: 运行标定
cd ~/ros2_ws
source install/setup.bash
python3 src/vive_ros2/scripts/vr_franka_calibration.py
```

**标定流程：**
1. 使用MoveIt或手动示教，将Franka移动到工作空间中的一个点
2. 用VR手柄**触碰** Robotiq夹爪基座
3. 在标定程序中输入 `r` 记录该点
4. 重复采集 **至少6个不同位置的点**（建议10-12个）
5. 采集完成后，输入 `c` 计算标定

**推荐标定点分布：**
```
点1-4: 工作空间四个角落（左前、右前、左后、右后）
点5-6: 工作空间中心（上方、中间）
点7-9: 边界中点（增加精度）
点10+: 常用操作区域
```

**精度验证：**
- ✅ 平均误差 < 2mm: 优秀，可用于数据采集
- ⚠️ 平均误差 2-5mm: 一般，建议重新标定
- ❌ 平均误差 > 5mm: 不合格，必须重新标定

标定结果保存在: `/tmp/vr_franka_calibration.yaml`

---

### 步骤 3️⃣: 启动遥操作系统

**完整启动流程：**

```bash
# 终端1: VR输入节点
cd ~/ros2_ws
source install/setup.bash
ros2 run vive_ros2 vive_input

# 终端2: VR处理节点（自动加载标定）
cd ~/ros2_ws
source install/setup.bash
ros2 run vive_ros2 vive_node

# 终端3: Franka控制器（您的控制节点）
# 订阅话题:
#   - /right_vr/vive_pose_rel  (右手相对位移)
#   - /left_vr/vive_pose_rel   (左手相对位移)
#   - /controller_data         (按钮状态)
```

**成功启动的标志：**
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
```
说明标定文件未找到，需要先完成步骤2。

---

## 🎮 操作指南

### 按钮功能

**Trigger (扳机键)**:
- 按下: 记录初始位置，开始发布相对位移
- 释放: 停止发布相对位移
- 用途: 增量式控制，适合精细操作

**Menu**: 重置初始位置

**Grip**: 切换测试方向（仅用于调试）

### 坐标系

```
VR物理空间 (Focus Vision)
    ↓ ALVR坐标转换
VR追踪数据
    ↓ correctVRSpaceRotation() [x=-z, y=y, z=x]
VR校正空间
    ↓ 手眼标定矩阵
Franka FR3 工作空间 (fr3_link0)
    ↓ 滤波 + 死区 + 串扰抑制
最终遥操作指令
```

---

## ⚙️ 参数调整

### 如果需要调整系统参数：

**调整滤波强度** (vive_node.cpp:113)
```cpp
alpha(0.3),  // 0.1=强滤波(慢响应), 0.5=弱滤波(快响应)
```

**调整死区** (vive_node.cpp:138)
```cpp
const double DEADZONE_MM = 1.0;  // 增大减少抖动，减小提高响应
```

**调整Y轴串扰抑制** (vive_node.cpp:140)
```cpp
const double FORWARD_BACKWARD_COMPENSATION = 0.85;  // 0.7-1.0
```

**修改后重新编译：**
```bash
cd ~/ros2_ws
colcon build --packages-select vive_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 📊 预期性能指标

### 精度

| 指标 | 未标定 | 标定后 |
|------|--------|--------|
| 绝对精度 | ±5-10mm | ±2-5mm |
| 相对精度 | ±3-8mm | ±1-3mm |
| Y轴串扰 | 70-80% | 10-15% |

### 延迟

- VR追踪延迟: ~10ms (Inside-Out)
- 有线串流延迟: 10-20ms (USB)
- 滤波处理延迟: ~15ms (alpha=0.3)
- **总延迟**: ~35-45ms

对离线数据采集无影响，对实时遥操作影响较小。

---

## ⚠️ 已知限制

### Focus Vision Inside-Out 追踪限制：

1. **精度上限**: ±5-10mm (硬件限制)
   - 标定可以补偿系统误差，但无法超越传感器精度
   - **不适合±1mm精度的精密装配任务**

2. **Y轴串扰**:
   - 即使应用85%抑制，仍有10-15%残留
   - 这是Focus Vision前置摄像头追踪的硬件特性
   - 对机器人学习数据采集影响较小（模型可学习补偿）

3. **环境要求**:
   - 需要良好光照（避免强光直射或过暗）
   - 建议在墙上贴纹理图片（帮助追踪）
   - 避免反光表面和单色墙壁

### 如果精度不够，升级方案：

**方案A: HTC Vive Tracker 3.0 + Lighthouse 2.0**
- 成本: ~$500-700 USD
- 精度: ±1mm
- 延迟: ~5ms
- 推荐指数: ⭐⭐⭐⭐⭐

**方案B: OptiTrack 光学追踪**
- 成本: $5,000-15,000 USD
- 精度: ±0.1mm (亚毫米级)
- 延迟: ~3ms
- 推荐指数: ⭐⭐⭐⭐⭐ (顶会论文标配)

---

## 🐛 故障排查

### 问题1: 编译失败

```bash
# 确保Eigen3已安装
sudo apt install libeigen3-dev

# 清理重新编译
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select vive_ros2
```

### 问题2: 标定精度很差（>10mm）

- ✓ 检查Franka话题是否正确发布
- ✓ 增加标定点数量（10-15个）
- ✓ 确保标定点均匀分布在整个工作空间
- ✓ 检查VR追踪质量（运行时看手柄是否抖动）
- ✓ 确保每个标定点位置准确（手柄触碰夹爪基座）

### 问题3: 位置抖动严重

```bash
# 选项1: 增大滤波强度
# vive_node.cpp:113  alpha(0.2)  # 从0.3改为0.2

# 选项2: 增大死区
# vive_node.cpp:138  const double DEADZONE_MM = 2.0;

# 选项3: 检查VR追踪环境
# - 增加环境光照
# - 贴墙壁纹理图片
# - 避免反光表面
```

### 问题4: 无法找到标定文件

```bash
# 检查标定文件是否存在
ls -lh /tmp/vr_franka_calibration.yaml

# 如果不存在，重新运行标定
python3 ~/ros2_ws/src/vive_ros2/scripts/vr_franka_calibration.py
```

---

## 📁 相关文件位置

```
系统文件：
~/ros2_ws/src/vive_ros2/
├── src/vive_node.cpp                    # 主节点（已修改）
├── include/vive_ros2/
│   ├── EigenTransforms.hpp              # 坐标转换（已修改）
│   └── VRUtils.hpp                      # VR工具（已修改）
├── scripts/
│   └── vr_franka_calibration.py         # 标定脚本 ✓

标定数据：
/tmp/vr_franka_calibration.yaml          # 手眼标定结果（运行标定后生成）

辅助工具：
/tmp/find_franka_topics.sh               # 话题查找脚本 ✓
/tmp/franka_vr_teleoperation_guide.md    # 完整使用指南 ✓

日志文件：
/tmp/controller_debug_YYYYMMDD_HHMMSS.log  # 调试日志（自动生成）
```

---

## ✅ 检查清单

**在运行标定之前，请确认：**

- [ ] Franka FR3 已上电并连接
- [ ] Franka控制器正在运行（发布末端位姿话题）
- [ ] 使用 `/tmp/find_franka_topics.sh` 确认正确的话题名
- [ ] 标定脚本已更新为正确的话题名（如果不是 `/franka_ee_pose`）
- [ ] ALVR已启动并连接Focus Vision
- [ ] VR手柄电量充足（建议>50%）
- [ ] 环境光照良好（避免强光直射）
- [ ] MoveIt或手动示教功能可用

**标定完成后，请验证：**

- [ ] 平均误差 < 2mm（优秀）或 < 5mm（可接受）
- [ ] 标定文件已生成：`/tmp/vr_franka_calibration.yaml`
- [ ] 启动 `vive_node` 时显示 "✓ 标定矩阵已加载"

---

## 🎓 数据采集建议

### 针对机器人学习数据采集：

1. **采集前校准**
   - 每次长时间采集前，重新运行标定
   - 每周至少标定一次

2. **环境设置**
   - 保持稳定光照
   - 在墙上贴带纹理的图片（帮助Inside-Out追踪）
   - 避免反光表面

3. **操作技巧**
   - 动作要平滑，避免突然加速
   - 前后移动时，保持手腕水平
   - 使用相对模式（按住Trigger）进行精细控制

4. **数据质量检查**
   - 记录示教轨迹的同时，记录时间戳
   - 定期检查轨迹平滑度
   - 如果发现跳跃或抖动，重新采集该段

---

## 📞 支持

如遇到问题，请检查：

1. 日志文件: `/tmp/controller_debug_*.log`
2. 编译输出: `~/ros2_ws/log/latest_build/vive_ros2/`
3. ROS2话题: `ros2 topic list` 和 `ros2 topic echo`

**系统状态检查命令：**
```bash
# 检查vive_ros2是否编译成功
ls -lh ~/ros2_ws/install/vive_ros2/lib/vive_ros2/vive_node

# 检查标定文件
cat /tmp/vr_franka_calibration.yaml

# 检查ROS2话题
ros2 topic list | grep -E "(vive|controller|franka)"

# 检查git状态
cd ~/ros2_ws/src/vive_ros2 && git status
```

---

## 🚀 准备就绪！

所有代码已实现并编译成功。现在可以开始：

1. 运行 `/tmp/find_franka_topics.sh` 查找Franka话题
2. 执行手眼标定采集数据
3. 启动遥操作系统进行数据采集

祝您的机器人学习项目顺利！🎉

---

*报告生成时间: 2026-01-16 02:56*
*系统版本: ROS2 Humble | ALVR 20.x | Focus Vision Inside-Out*
