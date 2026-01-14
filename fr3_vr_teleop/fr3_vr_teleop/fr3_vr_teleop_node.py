import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

from geometry_msgs.msg import TwistStamped
from vive_ros2.msg import VRControllerData
from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger

from scipy.spatial.transform import Rotation as R
import numpy as np
import csv
import os
from datetime import datetime

import tf2_ros
from tf2_ros import TransformException


def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def quat_mul(q1, q2):
    # q = [x, y, z, w]
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    ], dtype=float)


def quat_inv(q):
    # for unit quaternion: inverse = conjugate
    x, y, z, w = q
    return np.array([-x, -y, -z, w], dtype=float)


def pose_compose(p1, q1, p2, q2):
    """
    T = T1 ⊗ T2
    p = p1 + R(q1) * p2
    q = q1 ⊗ q2
    """
    q1 = quat_normalize(q1)
    q2 = quat_normalize(q2)
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)

    R1 = R.from_quat(q1)
    p = p1 + R1.apply(p2)
    q = quat_mul(q1, q2)
    q = quat_normalize(q)
    return p, q


def pose_inverse(p, q):
    """
    T^-1
    q_inv = q^-1
    p_inv = -R(q_inv) * p
    """
    q = quat_normalize(q)
    q_i = quat_inv(q)
    R_i = R.from_quat(q_i)
    p_i = -R_i.apply(np.asarray(p, dtype=float))
    return p_i, q_i


def pose_error(p_cur, q_cur, p_des, q_des):
    """
    Compute error transform: T_err = T_cur^-1 ⊗ T_des
    return:
      pos_err (in cur frame, but we will treat it as in planning frame for small errors)
      rotvec_err (axis-angle vector)
    """
    p_ci, q_ci = pose_inverse(p_cur, q_cur)
    p_err, q_err = pose_compose(p_ci, q_ci, p_des, q_des)

    # rotation error as rotvec
    q_err = quat_normalize(q_err)
    rotvec = R.from_quat(q_err).as_rotvec()  # axis*angle
    return p_err, rotvec


class FR3VRTeleopFull(Node):
    def __init__(self):
        super().__init__('fr3_vr_teleop_full')

        # ================== 1. 参数配置 ==================
        self.declare_parameter('planning_frame', 'fr3_link0')
        self.declare_parameter('ee_frame', 'robotiq_85_base_link')

        # 运动控制参数
        self.declare_parameter('linear_scale', 2.1)       # -> Kp_pos
        self.declare_parameter('angular_scale', 0.4)      # -> Kp_rot
        self.declare_parameter('v_max', 0.15)             # m/s
        self.declare_parameter('w_max', 1.0)              # rad/s
        self.declare_parameter('smoothing_factor', 0.3)
        self.declare_parameter('deadzone_linear', 0.002)   # 2mm
        self.declare_parameter('deadzone_angular', 0.03)   # ~1.7deg

        # ✅ 夹爪参数 - 替换为连续控制参数
        self.declare_parameter('gripper_open_pos', 0.0)
        self.declare_parameter('gripper_close_pos', 0.8)
        self.declare_parameter('gripper_force', 50.0)
        
        # ✅ 新增：夹爪连续控制参数
        self.declare_parameter('gripper_axis', 'trackpad_y')      # 用 trackpad_y 轴
        self.declare_parameter('gripper_use_touch_gate', True)    # 触摸才响应，防误触
        self.declare_parameter('gripper_axis_deadzone', 0.08)     # 摇杆死区
        self.declare_parameter('gripper_speed', 0.8)              # 全行程/秒
        self.declare_parameter('gripper_rate', 15.0)              # 发送频率 Hz
        self.declare_parameter('gripper_deadband', 0.01)          # 位置变化小于此不发

        # ✅ 调试参数
        self.declare_parameter('debug_enabled', True)             # 是否启用调试日志
        self.declare_parameter('debug_log_rate', 10.0)            # 日志打印频率 Hz
        self.declare_parameter('record_to_file', True)            # 是否记录到文件
        self.declare_parameter('record_dir', '~/ros2_ws/tmp/vr_teleop_debug')  # 记录文件目录

        # 运动控制状态变量
        self.is_controlling = False

        # === Pose 锚点（关键新增）===
        self.anchor_set = False
        self.ee_anchor_p = None
        self.ee_anchor_q = None
        self.vr_anchor_p = None
        self.vr_anchor_q = None

        # 指令容器
        self.target_twist = TwistStamped()
        self.target_twist.header.frame_id = self.get_parameter('planning_frame').value

        # ✅ 夹爪连续控制状态（新增）
        self._gripper_target_pos = float(self.get_parameter('gripper_open_pos').value)
        self._last_gripper_send_time = 0.0
        self._last_gripper_pos_sent = None

        # ✅ 调试状态变量
        self._last_debug_log_time = 0.0
        self._debug_csv_file = None
        self._debug_csv_writer = None
        self._init_debug_recording()

        # ================== 2. 通信接口 ==================
        qos = QoSProfile(depth=10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 订阅 VR 控制器数据
        self.controller_sub = self.create_subscription(
            VRControllerData, '/controller_data', self.vr_callback, qos)

        # 发布 MoveIt Servo 速度指令
        self.twist_pub = self.create_publisher(
            TwistStamped, '/moveit_servo/delta_twist_cmds', qos)

        # 夹爪 Action
        self.gripper_client = ActionClient(
            self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')

        # Servo 激活客户端
        self.servo_start_client = self.create_client(Trigger, '/moveit_servo/start_servo')
        self.timer_activation = self.create_timer(1.0, self.try_activate_servo)

        # 持续发布定时器 (50Hz)
        self.publish_rate = 50.0
        self.timer_twist = self.create_timer(1.0 / self.publish_rate, self.publish_twist)

        self.get_logger().info("✅ Pose→Twist 闭环版本启动：使用 trackpad_y 进行夹爪连续控制")

    # ================== 调试记录初始化 ==================
    def _init_debug_recording(self):
        if not self.get_parameter('record_to_file').value:
            return

        record_dir = self.get_parameter('record_dir').value
        os.makedirs(record_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(record_dir, f'teleop_debug_{timestamp}.csv')

        self._debug_csv_file = open(csv_path, 'w', newline='')
        self._debug_csv_writer = csv.writer(self._debug_csv_file)

        # 写入表头
        header = [
            'timestamp',
            # VR原始数据
            'vr_raw_x', 'vr_raw_y', 'vr_raw_z',
            'vr_raw_qx', 'vr_raw_qy', 'vr_raw_qz', 'vr_raw_qw',
            # VR锚点
            'vr_anchor_x', 'vr_anchor_y', 'vr_anchor_z',
            'vr_anchor_qx', 'vr_anchor_qy', 'vr_anchor_qz', 'vr_anchor_qw',
            # VR相对变化
            'dvr_x', 'dvr_y', 'dvr_z',
            'dvr_qx', 'dvr_qy', 'dvr_qz', 'dvr_qw',
            # 映射后机器人坐标系变化
            'dp_robot_x', 'dp_robot_y', 'dp_robot_z',
            'rotvec_robot_x', 'rotvec_robot_y', 'rotvec_robot_z',
            # EE锚点
            'ee_anchor_x', 'ee_anchor_y', 'ee_anchor_z',
            'ee_anchor_qx', 'ee_anchor_qy', 'ee_anchor_qz', 'ee_anchor_qw',
            # 当前EE位姿
            'ee_cur_x', 'ee_cur_y', 'ee_cur_z',
            'ee_cur_qx', 'ee_cur_qy', 'ee_cur_qz', 'ee_cur_qw',
            # 期望EE位姿
            'ee_des_x', 'ee_des_y', 'ee_des_z',
            'ee_des_qx', 'ee_des_qy', 'ee_des_qz', 'ee_des_qw',
            # 误差
            'pos_err_x', 'pos_err_y', 'pos_err_z',
            'rot_err_x', 'rot_err_y', 'rot_err_z',
            # 控制输出
            'v_cmd_x', 'v_cmd_y', 'v_cmd_z',
            'w_cmd_x', 'w_cmd_y', 'w_cmd_z',
            # 最终twist
            'twist_vx', 'twist_vy', 'twist_vz',
            'twist_wx', 'twist_wy', 'twist_wz',
            # 状态
            'is_controlling', 'anchor_set'
        ]
        self._debug_csv_writer.writerow(header)
        self.get_logger().info(f"📝 调试数据将记录到: {csv_path}")

    def _record_debug_data(self, data: dict):
        """记录一行调试数据到CSV"""
        if self._debug_csv_writer is None:
            return

        row = [
            data.get('timestamp', 0.0),
            # VR原始
            *data.get('vr_raw_p', [0, 0, 0]),
            *data.get('vr_raw_q', [0, 0, 0, 1]),
            # VR锚点
            *data.get('vr_anchor_p', [0, 0, 0]),
            *data.get('vr_anchor_q', [0, 0, 0, 1]),
            # VR变化
            *data.get('dvr_p', [0, 0, 0]),
            *data.get('dvr_q', [0, 0, 0, 1]),
            # 映射后
            *data.get('dp_robot', [0, 0, 0]),
            *data.get('rotvec_robot', [0, 0, 0]),
            # EE锚点
            *data.get('ee_anchor_p', [0, 0, 0]),
            *data.get('ee_anchor_q', [0, 0, 0, 1]),
            # 当前EE
            *data.get('ee_cur_p', [0, 0, 0]),
            *data.get('ee_cur_q', [0, 0, 0, 1]),
            # 期望EE
            *data.get('ee_des_p', [0, 0, 0]),
            *data.get('ee_des_q', [0, 0, 0, 1]),
            # 误差
            *data.get('pos_err', [0, 0, 0]),
            *data.get('rot_err', [0, 0, 0]),
            # 控制输出
            *data.get('v_cmd', [0, 0, 0]),
            *data.get('w_cmd', [0, 0, 0]),
            # 最终twist
            *data.get('twist_v', [0, 0, 0]),
            *data.get('twist_w', [0, 0, 0]),
            # 状态
            data.get('is_controlling', False),
            data.get('anchor_set', False)
        ]
        self._debug_csv_writer.writerow(row)
        self._debug_csv_file.flush()

    def _log_debug_info(self, data: dict):
        """打印调试日志（限频）"""
        if not self.get_parameter('debug_enabled').value:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        rate = self.get_parameter('debug_log_rate').value
        if (now - self._last_debug_log_time) < (1.0 / rate):
            return
        self._last_debug_log_time = now

        vr_raw = data.get('vr_raw_p', [0, 0, 0])
        dvr = data.get('dvr_p', [0, 0, 0])
        dp_robot = data.get('dp_robot', [0, 0, 0])
        ee_cur = data.get('ee_cur_p', [0, 0, 0])
        ee_des = data.get('ee_des_p', [0, 0, 0])
        pos_err = data.get('pos_err', [0, 0, 0])
        v_cmd = data.get('v_cmd', [0, 0, 0])
        twist_v = data.get('twist_v', [0, 0, 0])

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"[VR原始]   p=({vr_raw[0]:+.4f}, {vr_raw[1]:+.4f}, {vr_raw[2]:+.4f})")
        self.get_logger().info(f"[VR变化]   dvr=({dvr[0]:+.4f}, {dvr[1]:+.4f}, {dvr[2]:+.4f})")
        self.get_logger().info(f"[映射后]   dp_robot=({dp_robot[0]:+.4f}, {dp_robot[1]:+.4f}, {dp_robot[2]:+.4f})")
        self.get_logger().info(f"[当前EE]   p=({ee_cur[0]:+.4f}, {ee_cur[1]:+.4f}, {ee_cur[2]:+.4f})")
        self.get_logger().info(f"[期望EE]   p=({ee_des[0]:+.4f}, {ee_des[1]:+.4f}, {ee_des[2]:+.4f})")
        self.get_logger().info(f"[误差]     pos_err=({pos_err[0]:+.4f}, {pos_err[1]:+.4f}, {pos_err[2]:+.4f})")
        self.get_logger().info(f"[P控制]    v_cmd=({v_cmd[0]:+.4f}, {v_cmd[1]:+.4f}, {v_cmd[2]:+.4f})")
        self.get_logger().info(f"[输出]     twist_v=({twist_v[0]:+.4f}, {twist_v[1]:+.4f}, {twist_v[2]:+.4f})")

    # ================== TF 读取当前末端 ==================
    def get_current_ee_pose(self):
        planning = self.get_parameter('planning_frame').value
        ee = self.get_parameter('ee_frame').value
        try:
            trans = self.tf_buffer.lookup_transform(planning, ee, rclpy.time.Time())
            p = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ], dtype=float)
            q = np.array([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ], dtype=float)
            q = quat_normalize(q)
            return p, q
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed ({planning} -> {ee}): {e}")
            return None, None

    # ================== 3. 核心逻辑 ==================
    def vr_callback(self, msg):
        # ✅ 替换：使用 trackpad_y 连续控制夹爪（新增）
        self.update_gripper_from_trackpad(msg)

        # 运动逻辑：按下 Trigger 键开始
        if msg.trigger_button:
            if not self.is_controlling:
                self.get_logger().info("🎮 运动开始（建立锚点）")
                self.is_controlling = True
                self.anchor_set = False  # 每次新按下都重新建锚点
            self.process_motion_pose_servo(msg)
        else:
            if self.is_controlling:
                self.get_logger().info("⏸️ 停止运动")
                self.is_controlling = False
            self.anchor_set = False
            self.smooth_stop()

    # ✅ 新增：夹爪连续控制函数
    def update_gripper_from_trackpad(self, msg: VRControllerData):
        # touch门控（防误触漂移）
        if bool(self.get_parameter('gripper_use_touch_gate').value):
            if not getattr(msg, 'trackpad_touch', False):
                return

        axis_name = str(self.get_parameter('gripper_axis').value)   # trackpad_y
        axis = float(getattr(msg, axis_name, 0.0))                  # 通常[-1,1]

        dz = float(self.get_parameter('gripper_axis_deadzone').value)
        if abs(axis) < dz:
            axis = 0.0

        speed = float(self.get_parameter('gripper_speed').value)
        dt = 1.0 / self.publish_rate

        open_pos = float(self.get_parameter('gripper_open_pos').value)
        close_pos = float(self.get_parameter('gripper_close_pos').value)

        # axis=+1 -> 更夹紧；axis=-1 -> 更张开
        delta = axis * speed * dt * (close_pos - open_pos)
        self._gripper_target_pos += delta
        self._gripper_target_pos = float(np.clip(self._gripper_target_pos,
                                                 min(open_pos, close_pos),
                                                 max(open_pos, close_pos)))

        self.control_gripper_position(self._gripper_target_pos)

    # ✅ 新增：夹爪位置控制函数
    def control_gripper_position(self, position: float):
        open_pos = float(self.get_parameter('gripper_open_pos').value)
        close_pos = float(self.get_parameter('gripper_close_pos').value)
        position = float(np.clip(position, min(open_pos, close_pos), max(open_pos, close_pos)))

        rate = float(self.get_parameter('gripper_rate').value)
        deadband = float(self.get_parameter('gripper_deadband').value)
        now = self.get_clock().now().nanoseconds * 1e-9

        if (now - self._last_gripper_send_time) < (1.0 / rate):
            return
        if self._last_gripper_pos_sent is not None and abs(position - self._last_gripper_pos_sent) < deadband:
            return

        if not self.gripper_client.wait_for_server(timeout_sec=0.2):
            return

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = float(self.get_parameter('gripper_force').value)
        self.gripper_client.send_goal_async(goal)

        self._last_gripper_send_time = now
        self._last_gripper_pos_sent = position

    def process_motion_pose_servo(self, msg):
        # === 参数 ===
        Kp_pos = float(self.get_parameter('linear_scale').value)
        Kp_rot = float(self.get_parameter('angular_scale').value)
        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        alpha = float(self.get_parameter('smoothing_factor').value)
        dz_lin = float(self.get_parameter('deadzone_linear').value)
        dz_ang = float(self.get_parameter('deadzone_angular').value)

        # === 当前末端位姿（TF）===
        ee_p, ee_q = self.get_current_ee_pose()
        if ee_p is None:
            # TF 没拿到，直接停止输出
            self.smooth_stop()
            return

        # === VR 当前相对位姿（来自 msg.rel_pose）===
        vr_p_now = np.array([
            msg.rel_pose.transform.translation.x,
            msg.rel_pose.transform.translation.y,
            msg.rel_pose.transform.translation.z
        ], dtype=float)
        vr_q_now = quat_normalize(np.array([
            msg.rel_pose.transform.rotation.x,
            msg.rel_pose.transform.rotation.y,
            msg.rel_pose.transform.rotation.z,
            msg.rel_pose.transform.rotation.w
        ], dtype=float))

        # === 第一次按下时建立锚点 ===
        if not self.anchor_set:
            self.ee_anchor_p = ee_p.copy()
            self.ee_anchor_q = ee_q.copy()
            self.vr_anchor_p = vr_p_now.copy()
            self.vr_anchor_q = vr_q_now.copy()
            self.anchor_set = True
            # 建锚点瞬间输出 0，避免"刚按下就窜"
            self.target_twist.twist.linear.x = 0.0
            self.target_twist.twist.linear.y = 0.0
            self.target_twist.twist.linear.z = 0.0
            self.target_twist.twist.angular.x = 0.0
            self.target_twist.twist.angular.y = 0.0
            self.target_twist.twist.angular.z = 0.0
            return

        # === 计算 VR 相对变化 ΔT_vr = T_vr_anchor^-1 ⊗ T_vr_now ===
        vr_anchor_pi, vr_anchor_qi = pose_inverse(self.vr_anchor_p, self.vr_anchor_q)
        dvr_p, dvr_q = pose_compose(vr_anchor_pi, vr_anchor_qi, vr_p_now, vr_q_now)

        # === 坐标轴映射（保持原来的平移映射逻辑）===
        # VR: [x=右, y=上, z=后]  -> robot planning_frame
        dp_robot = np.array([
            dvr_p[2],   # x <- vr_z
            -dvr_p[0],   # y <- vr_x
            dvr_p[1],   # z <- vr_y
        ], dtype=float)

        # === 旋转映射（沿用之前 rotvec 的轴映射方式）===
        rotvec_raw = R.from_quat(dvr_q).as_rotvec()  # [rx, ry, rz] in VR frame

        rotvec_robot = np.array([
             -rotvec_raw[2],   # x
             rotvec_raw[0],   # y
             rotvec_raw[1],   # z
        ], dtype=float)

        # 将 rotvec_robot 转回四元数（作为 ΔR_robot）
        dR_robot = R.from_rotvec(rotvec_robot)
        dq_robot = quat_normalize(dR_robot.as_quat())

        # === 期望末端位姿 T_des = T_ee_anchor ⊗ ΔT_robot ===
        p_des, q_des = pose_compose(self.ee_anchor_p, self.ee_anchor_q, dp_robot, dq_robot)

        # === 误差：T_err = T_cur^-1 ⊗ T_des ===
        pos_err, rotvec_err = pose_error(ee_p, ee_q, p_des, q_des)

        # === 死区：对"误差"做死区 ===
        if np.linalg.norm(pos_err) < dz_lin:
            pos_err[:] = 0.0
        if np.linalg.norm(rotvec_err) < dz_ang:
            rotvec_err[:] = 0.0

        # === P 控制：误差 → twist ===
        v_cmd = Kp_pos * pos_err
        w_cmd = Kp_rot * rotvec_err

        # === 限幅 ===
        v_norm = float(np.linalg.norm(v_cmd))
        if v_norm > v_max and v_norm > 1e-12:
            v_cmd = v_cmd / v_norm * v_max

        w_norm = float(np.linalg.norm(w_cmd))
        if w_norm > w_max and w_norm > 1e-12:
            w_cmd = w_cmd / w_norm * w_max

        # === 平滑（低通）===
        self.target_twist.twist.linear.x = self.target_twist.twist.linear.x * (1 - alpha) + v_cmd[0] * alpha
        self.target_twist.twist.linear.y = self.target_twist.twist.linear.y * (1 - alpha) + v_cmd[1] * alpha
        self.target_twist.twist.linear.z = self.target_twist.twist.linear.z * (1 - alpha) + v_cmd[2] * alpha

        self.target_twist.twist.angular.x = self.target_twist.twist.angular.x * (1 - alpha) + w_cmd[0] * alpha
        self.target_twist.twist.angular.y = self.target_twist.twist.angular.y * (1 - alpha) + w_cmd[1] * alpha
        self.target_twist.twist.angular.z = self.target_twist.twist.angular.z * (1 - alpha) + w_cmd[2] * alpha

        # === 收集并记录调试数据 ===
        debug_data = {
            'timestamp': self.get_clock().now().nanoseconds * 1e-9,
            # VR原始数据
            'vr_raw_p': vr_p_now.tolist(),
            'vr_raw_q': vr_q_now.tolist(),
            # VR锚点
            'vr_anchor_p': self.vr_anchor_p.tolist() if self.vr_anchor_p is not None else [0, 0, 0],
            'vr_anchor_q': self.vr_anchor_q.tolist() if self.vr_anchor_q is not None else [0, 0, 0, 1],
            # VR相对变化
            'dvr_p': dvr_p.tolist(),
            'dvr_q': dvr_q.tolist(),
            # 映射后机器人坐标系变化
            'dp_robot': dp_robot.tolist(),
            'rotvec_robot': rotvec_robot.tolist(),
            # EE锚点
            'ee_anchor_p': self.ee_anchor_p.tolist() if self.ee_anchor_p is not None else [0, 0, 0],
            'ee_anchor_q': self.ee_anchor_q.tolist() if self.ee_anchor_q is not None else [0, 0, 0, 1],
            # 当前EE位姿
            'ee_cur_p': ee_p.tolist(),
            'ee_cur_q': ee_q.tolist(),
            # 期望EE位姿
            'ee_des_p': p_des.tolist(),
            'ee_des_q': q_des.tolist(),
            # 误差
            'pos_err': pos_err.tolist(),
            'rot_err': rotvec_err.tolist(),
            # 控制输出
            'v_cmd': v_cmd.tolist(),
            'w_cmd': w_cmd.tolist(),
            # 最终twist
            'twist_v': [
                self.target_twist.twist.linear.x,
                self.target_twist.twist.linear.y,
                self.target_twist.twist.linear.z
            ],
            'twist_w': [
                self.target_twist.twist.angular.x,
                self.target_twist.twist.angular.y,
                self.target_twist.twist.angular.z
            ],
            # 状态
            'is_controlling': self.is_controlling,
            'anchor_set': self.anchor_set
        }
        self._record_debug_data(debug_data)
        self._log_debug_info(debug_data)

    # ================== 停止时平滑归零 ==================
    def smooth_stop(self):
        decay = 0.5
        self.target_twist.twist.linear.x *= decay
        self.target_twist.twist.linear.y *= decay
        self.target_twist.twist.linear.z *= decay
        self.target_twist.twist.angular.x *= decay
        self.target_twist.twist.angular.y *= decay
        self.target_twist.twist.angular.z *= decay

    # ================== 发布 twist ==================
    def publish_twist(self):
        self.target_twist.header.stamp = self.get_clock().now().to_msg()
        self.target_twist.header.frame_id = self.get_parameter('planning_frame').value
        self.twist_pub.publish(self.target_twist)

    # ================== Servo 启动 ==================
    def try_activate_servo(self):
        if self.servo_start_client.wait_for_service(timeout_sec=0.1):
            self.servo_start_client.call_async(Trigger.Request())
            self.timer_activation.cancel()


def main():
    rclpy.init()
    node = FR3VRTeleopFull()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
