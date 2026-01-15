#!/bin/bash
# 帮助查找Franka机械臂的末端执行器位姿话题
# 使用方法：在启动Franka控制器后运行此脚本

echo "=================================================="
echo "正在查找 Franka FR3 末端执行器位姿话题..."
echo "=================================================="
echo ""

# 检查ROS2是否正在运行
if ! ros2 topic list &> /dev/null; then
    echo "❌ 错误：未检测到运行中的ROS2节点"
    echo ""
    echo "请先启动以下节点："
    echo "  1. Franka 控制器"
    echo "  2. MoveIt (如果使用)"
    echo ""
    echo "然后重新运行此脚本"
    exit 1
fi

echo "📋 所有可用话题："
echo "---"
ros2 topic list
echo ""

echo "=================================================="
echo "🔍 查找包含 'franka', 'pose', 'ee' 的话题："
echo "=================================================="
ros2 topic list | grep -iE "(franka|pose|ee|end.*effector)" | while read topic; do
    echo ""
    echo "话题: $topic"
    echo "类型: $(ros2 topic info $topic 2>/dev/null | grep 'Type:' || echo '未知')"
done

echo ""
echo "=================================================="
echo "💡 提示："
echo "=================================================="
echo "找到正确的 PoseStamped 话题后，编辑标定脚本："
echo ""
echo "  nano /home/enine/ros2_ws/src/vive_ros2/scripts/vr_franka_calibration.py"
echo ""
echo "修改第24行的话题名："
echo "  self.franka_sub = self.create_subscription("
echo "      PoseStamped, '/你的话题名', self.franka_callback, 10)"
echo ""
echo "常见话题名："
echo "  - /franka_ee_pose"
echo "  - /franka/end_effector_pose"
echo "  - /panda_link8"
echo "  - /move_group/display_planned_path"
echo ""
