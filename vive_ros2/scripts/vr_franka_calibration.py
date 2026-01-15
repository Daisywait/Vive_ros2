#!/usr/bin/env python3
"""
VR-Franka FR3 手眼标定工具
用于将VR手柄空间精确映射到Franka机械臂工作空间
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import yaml
from scipy.spatial.transform import Rotation
from scipy.linalg import lstsq

class VRFrankaCalibration(Node):
    def __init__(self):
        super().__init__('vr_franka_calibration')

        # 订阅
        self.vr_sub = self.create_subscription(
            PoseStamped, '/controller_pose', self.vr_callback, 10)
        self.franka_sub = self.create_subscription(
            PoseStamped, '/franka_ee_pose', self.franka_callback, 10)

        # 数据存储
        self.vr_points = []
        self.franka_points = []
        self.current_vr_pose = None
        self.current_franka_pose = None

        self.get_logger().info('='*60)
        self.get_logger().info('VR-Franka FR3 手眼标定系统')
        self.get_logger().info('='*60)
        self.get_logger().info('请按以下步骤操作：')
        self.get_logger().info('1. 将Franka末端移动到工作空间中的一个点')
        self.get_logger().info('2. 将VR手柄触碰到末端执行器')
        self.get_logger().info('3. 按下手柄的Trigger键记录点位')
        self.get_logger().info('4. 重复至少6个点（越多越好）')
        self.get_logger().info('5. 输入 "c" 计算标定结果')

        # 启动键盘输入线程
        import threading
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def vr_callback(self, msg):
        self.current_vr_pose = msg

    def franka_callback(self, msg):
        self.current_franka_pose = msg

    def input_loop(self):
        """键盘输入处理"""
        while rclpy.ok():
            cmd = input('\n[r]记录点 [c]计算标定 [q]退出: ').strip().lower()

            if cmd == 'r':
                self.record_point()
            elif cmd == 'c':
                self.compute_calibration()
            elif cmd == 'q':
                rclpy.shutdown()
                break

    def record_point(self):
        """记录一个标定点"""
        if self.current_vr_pose is None or self.current_franka_pose is None:
            self.get_logger().error('等待VR和Franka数据...')
            return

        # 提取位置
        vr_pos = np.array([
            self.current_vr_pose.pose.position.x,
            self.current_vr_pose.pose.position.y,
            self.current_vr_pose.pose.position.z
        ])

        franka_pos = np.array([
            self.current_franka_pose.pose.position.x,
            self.current_franka_pose.pose.position.y,
            self.current_franka_pose.pose.position.z
        ])

        self.vr_points.append(vr_pos)
        self.franka_points.append(franka_pos)

        self.get_logger().info(f'✓ 已记录点 {len(self.vr_points)}:')
        self.get_logger().info(f'  VR:     [{vr_pos[0]:.4f}, {vr_pos[1]:.4f}, {vr_pos[2]:.4f}]')
        self.get_logger().info(f'  Franka: [{franka_pos[0]:.4f}, {franka_pos[1]:.4f}, {franka_pos[2]:.4f}]')

    def compute_calibration(self):
        """计算手眼标定矩阵"""
        if len(self.vr_points) < 4:
            self.get_logger().error(f'需要至少4个点，当前只有 {len(self.vr_points)} 个')
            return

        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('开始计算手眼标定...')
        self.get_logger().info('='*60)

        # 转换为numpy数组
        vr_pts = np.array(self.vr_points)
        franka_pts = np.array(self.franka_points)

        # 计算质心
        vr_centroid = np.mean(vr_pts, axis=0)
        franka_centroid = np.mean(franka_pts, axis=0)

        # 去质心
        vr_centered = vr_pts - vr_centroid
        franka_centered = franka_pts - franka_centroid

        # 使用SVD求解旋转矩阵
        H = vr_centered.T @ franka_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # 确保是右手系
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        # 计算平移
        t = franka_centroid - R @ vr_centroid

        # 构建4x4变换矩阵
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t

        # 计算标定误差
        errors = []
        self.get_logger().info('\n标定结果验证:')
        for i, (vr_pt, franka_pt) in enumerate(zip(vr_pts, franka_pts)):
            # 应用变换
            vr_homogeneous = np.append(vr_pt, 1)
            transformed = (T @ vr_homogeneous)[:3]

            # 计算误差
            error = np.linalg.norm(transformed - franka_pt)
            errors.append(error)

            self.get_logger().info(f'  点 {i+1}: 误差 = {error*1000:.2f} mm')

        mean_error = np.mean(errors)
        max_error = np.max(errors)

        self.get_logger().info(f'\n标定精度:')
        self.get_logger().info(f'  平均误差: {mean_error*1000:.2f} mm')
        self.get_logger().info(f'  最大误差: {max_error*1000:.2f} mm')

        # 评估精度
        if mean_error < 0.002:  # 2mm
            self.get_logger().info('  ✅ 标定精度优秀！')
        elif mean_error < 0.005:  # 5mm
            self.get_logger().info('  ⚠️  标定精度一般，建议重新标定')
        else:
            self.get_logger().error('  ❌ 标定精度不足，必须重新标定！')
            return

        # 保存结果
        calibration_data = {
            'transformation_matrix': T.tolist(),
            'rotation_matrix': R.tolist(),
            'translation_vector': t.tolist(),
            'mean_error_mm': float(mean_error * 1000),
            'max_error_mm': float(max_error * 1000),
            'num_points': len(self.vr_points),
            'vr_points': vr_pts.tolist(),
            'franka_points': franka_pts.tolist()
        }

        # 保存到工作空间（主要位置）
        import os
        workspace_path = os.path.expanduser('~/ros2_ws/src/vive_ros2/config/vr_franka_calibration.yaml')
        os.makedirs(os.path.dirname(workspace_path), exist_ok=True)

        with open(workspace_path, 'w') as f:
            yaml.dump(calibration_data, f, default_flow_style=False)

        self.get_logger().info(f'\n✓ 标定结果已保存到: {workspace_path}')

        # 同时保存到 /tmp（用于兼容）
        tmp_path = '/tmp/vr_franka_calibration.yaml'
        with open(tmp_path, 'w') as f:
            yaml.dump(calibration_data, f, default_flow_style=False)

        self.get_logger().info(f'✓ 备份副本已保存到: {tmp_path}')

        self.get_logger().info('\n变换矩阵 (VR → Franka):')
        self.get_logger().info(str(T))

def main():
    rclpy.init()
    node = VRFrankaCalibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
