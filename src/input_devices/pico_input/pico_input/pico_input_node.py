#!/usr/bin/env python3
"""
PICO Input Node - 从 XRoboToolkit PC-Service 读取 VR 追踪数据

==================== 设计理念 (增量控制模式) ====================

核心思想：
  - 初始化时记录所有 tracker 的初始位姿
  - 之后只发布 tracker 相对于初始位姿的"变化量"
  - 用户可以用任何姿势初始化，机器人不会跳动

优点：
  - 最安全，初始化时不会导致机器人突然移动
  - 用户可以坐着、站着、任何姿势
  - 建议用户尽量模仿机器人姿势以获得更好的控制感

初始化过程：
  1. 用户摆好姿势 (建议模仿机器人当前姿势)
  2. 按下初始化按钮 (或等待自动初始化)
  3. 系统记录 HMD 初始位姿 (仅用于可视化)
  4. 系统记录所有 tracker 的初始位姿
  5. 后续只发布相对于初始位姿的增量

==================== 增量控制原理 ====================

    初始化时：
      - 记录用户手在 PICO 坐标系中的位姿 init_tracker_poses[role]
      - 机器人初始位姿来自 tianji_robot.yaml (init_pos/init_rot)

    运行时：
      - 用户手当前位姿: current_pose
      - 位置增量: delta_pos = current_pos - init_pos
      - 姿态增量: delta_rot = current_rot * init_rot.inv()
      - 发布: 机器人目标 = 机器人初始 + 增量 (经坐标变换)

==================== TF 树结构 (统一命名规范) ====================

    world (机器人基座, 固定)
    ├── head (HMD, 仅用于可视化)
    ├── world_left (左臂 chest 坐标系, 静态, Y=+0.2)
    │   ├── pico_left_wrist (PICO 左手腕 tracker, 动态)
    │   └── pico_left_arm (PICO 左前臂 tracker, 动态)
    └── world_right (右臂 chest 坐标系, 静态, Y=-0.2)
        ├── pico_right_wrist (PICO 右手腕 tracker, 动态)
        └── pico_right_arm (PICO 右前臂 tracker, 动态)

==================== Topics (统一命名规范) ====================

    # PICO 数据 (调试/可视化)
    /pico_hmd, /pico_left_wrist, /pico_right_wrist
    /pico_left_arm, /pico_right_arm

    # 机器人控制 (tianji_world_output 订阅)
    /left_arm_target_pose, /right_arm_target_pose
    /left_arm_elbow_direction, /right_arm_elbow_direction

==================== 使用方法 ====================
    # 启动
    ros2 launch wuji_teleop_bringup pico_teleop.launch.py

    # 手动初始化 (用户对准机器人后)
    ros2 service call /pico_input/init std_srvs/srv/Trigger

    # 重置初始化
    ros2 service call /pico_input/reset std_srvs/srv/Trigger
"""

import re
import socket
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped, Vector3Stamped, Point
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R

# Data source abstraction (TDD refactoring)
from pico_input.data_source import DataSource, LiveDataSource, RecordedDataSource, TrackerData, HeadsetData
from pico_input.incremental_controller import IncrementalController

# 统一配置加载器 (通过 ROS2 包依赖: package.xml exec_depend)
from tianji_world_output.config_loader import get_config as get_tianji_config
from tianji_world_output.transform_utils import (
    get_tf_quaternion,
)
_tianji_config = get_tianji_config(use_ros=False)


class PicoInputNode(Node):
    """
    PICO 输入节点 (增量控制模式 + TDD 重构)

    架构:
    - IncrementalController: 纯计算 (增量位姿、肘部方向、One-Euro 自适应平滑)
    - PicoInputNode: ROS2 薄壳 (数据源、TF、Topic、服务、可视化)

    输出:
    - TF: world → head, pico_*_wrist, pico_*_arm
    - Topics: /left_arm_target_pose, /right_arm_target_pose
    - Topics: /pico/* (可选，调试用)
    """

    def __init__(self, data_source: Optional[DataSource] = None):
        super().__init__('pico_input_node')

        # 安装 stdlib→ROS2 日志桥接 (使非 Node 类日志进入 /rosout)
        from pico_input.ros2_logging import setup_ros2_logging_bridge
        setup_ros2_logging_bridge(self.get_logger())

        # ==================== 参数声明 ====================
        self.declare_parameter('publish_rate', 90.0)

        # 数据源配置 (TDD 重构新增)
        self.declare_parameter('data_source_type', 'live')  # 'live' or 'recorded'
        self.declare_parameter('recorded_file_path', 'record/trackingData_sample_static.txt')
        self.declare_parameter('playback_speed', 1.0)
        self.declare_parameter('loop_playback', True)

        # PC-Service 连接 (LiveDataSource 使用)
        self.declare_parameter('pc_service_host', '127.0.0.1')
        self.declare_parameter('pc_service_port', 60061)

        # Topic 发布配置
        self.declare_parameter('enable_topic_publishing', True)  # 发布 /left_arm_target_pose 等
        self.declare_parameter('enable_legacy_topics', False)    # 发布 /pico/* (调试用)
        self.declare_parameter('topic_prefix', 'pico')

        # One-Euro Filter 自适应滤波参数 (启动时从 pico_input.yaml 加载)
        self.declare_parameter('one_euro_min_cutoff', 1.0)   # 最小截止频率 (Hz)
        self.declare_parameter('one_euro_beta', 0.7)         # 速度响应系数
        self.declare_parameter('elbow_min_cutoff', 0.3)      # 肘部方向最小截止频率 (Hz)

        # Tracker 序列号映射 (统一命名规范)
        # 序列号格式: 190058, 190046, 190600, 190023 (6位数字)
        # pico_* 前缀表示 PICO tracker，与机器人 DH 末端区分
        self.declare_parameter('tracker_serial_190058', 'pico_left_wrist')   # 左手腕
        self.declare_parameter('tracker_serial_190046', 'pico_left_arm')     # 左前臂
        self.declare_parameter('tracker_serial_190600', 'pico_right_wrist')  # 右手腕
        self.declare_parameter('tracker_serial_190023', 'pico_right_arm')    # 右前臂

        # 初始化相关参数
        self.declare_parameter('auto_init_delay', 5.0)  # 自动初始化延迟 (秒), 0 禁用

        # 获取参数
        self.publish_rate = self.get_parameter('publish_rate').value
        self.data_source_type = self.get_parameter('data_source_type').value
        self.pc_service_host = self.get_parameter('pc_service_host').value
        self.pc_service_port = self.get_parameter('pc_service_port').value
        self.enable_topic_publishing = self.get_parameter('enable_topic_publishing').value
        self.enable_legacy_topics = self.get_parameter('enable_legacy_topics').value
        self.topic_prefix = self.get_parameter('topic_prefix').value
        self.auto_init_delay = self.get_parameter('auto_init_delay').value

        # 构建序列号映射表: serial_number -> role
        self.tracker_serial_map = {}
        for serial in ['190058', '190046', '190600', '190023']:
            param_name = f'tracker_serial_{serial}'
            try:
                role = self.get_parameter(param_name).value
                if role:
                    self.tracker_serial_map[serial] = role
            except Exception:
                pass

        # ==================== 增量控制器 (核心计算逻辑) ====================
        # 纯计算类，无 ROS2 依赖，可独立单元测试
        min_cutoff = self.get_parameter('one_euro_min_cutoff').value
        beta = self.get_parameter('one_euro_beta').value
        elbow_min_cutoff = self.get_parameter('elbow_min_cutoff').value

        self.controller = IncrementalController(
            config=_tianji_config,
            rate=self.publish_rate,
            min_cutoff=min_cutoff,
            beta=beta,
            elbow_min_cutoff=elbow_min_cutoff,
        )
        self.get_logger().info(
            f"One-Euro Filter: rate={self.publish_rate}Hz, "
            f"min_cutoff={min_cutoff}, beta={beta}, elbow_min_cutoff={elbow_min_cutoff}"
        )
        self.init_start_time = None

        # 臂角日志计数器
        self._elbow_log_counter = 0

        # ==================== 臂角诊断日志 ====================
        self.get_logger().info('=' * 70)
        self.get_logger().info('[ELBOW DIAG] 臂角初始化参数:')
        for side in ['left', 'right']:
            wrist_pos = self.controller.robot_init_positions[f'pico_{side}_wrist']
            arm_pos = self.controller.robot_init_positions[f'pico_{side}_arm']
            default_dir = self.controller.default_zsp_direction[side]
            sw = wrist_pos
            sw_len = float(np.linalg.norm(sw))
            if sw_len > 1e-6:
                sw_unit = sw / sw_len
                proj = np.dot(arm_pos, sw_unit) * sw_unit
                offset = arm_pos - proj
                offset_len = float(np.linalg.norm(offset))
                dot_val = float(np.dot(offset / np.linalg.norm(offset), default_dir)) if offset_len > 1e-6 else 0.0
            else:
                offset_len = 0.0
                dot_val = 0.0
            self.get_logger().info(f'  [{side}] wrist_init_pos(chest): [{wrist_pos[0]:.4f}, {wrist_pos[1]:.4f}, {wrist_pos[2]:.4f}]')
            self.get_logger().info(f'  [{side}] arm_init_pos(chest):   [{arm_pos[0]:.4f}, {arm_pos[1]:.4f}, {arm_pos[2]:.4f}]')
            self.get_logger().info(f'  [{side}] sw_length: {sw_len:.4f}m, arm_offset: {offset_len:.4f}m')
            self.get_logger().info(f'  [{side}] default_zsp_dir: [{default_dir[0]:.4f}, {default_dir[1]:.4f}, {default_dir[2]:.4f}]')
            self.get_logger().info(f'  [{side}] dot(geom, default): {dot_val:.4f} {"(ALIGNED)" if dot_val > 0 else "(OPPOSITE!)"}')
        self.get_logger().info('=' * 70)

        # ==================== World → Chest 转换参数 ====================
        # 从统一配置加载 world_to_chest 变换 (用于静态 TF 发布)
        self.world_to_chest_trans = {
            'left': _tianji_config.world_to_chest_trans.get('left', np.array([0.0, 0.2, 0.0])),
            'right': _tianji_config.world_to_chest_trans.get('right', np.array([0.0, -0.2, 0.0])),
        }

        # ==================== TF 广播器 ====================
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # 发布静态 TF: world → world_left, world_right
        self._publish_static_chest_frames()

        # ==================== 服务 ====================
        self.init_service = self.create_service(
            Trigger, 'pico_input/init', self._init_callback
        )
        self.reset_service = self.create_service(
            Trigger, 'pico_input/reset', self._reset_callback
        )

        # ==================== Topic 发布器 ====================
        # 1. 目标位姿发布 (tianji_world_output_node 订阅这些)
        self.left_arm_pose_pub = None
        self.right_arm_pose_pub = None
        self.left_elbow_dir_pub = None
        self.right_elbow_dir_pub = None
        if self.enable_topic_publishing:
            self.left_arm_pose_pub = self.create_publisher(
                PoseStamped, '/left_arm_target_pose', 10
            )
            self.right_arm_pose_pub = self.create_publisher(
                PoseStamped, '/right_arm_target_pose', 10
            )
            # 肘部方向 (臂角约束)
            self.left_elbow_dir_pub = self.create_publisher(
                Vector3Stamped, '/left_arm_elbow_direction', 10
            )
            self.right_elbow_dir_pub = self.create_publisher(
                Vector3Stamped, '/right_arm_elbow_direction', 10
            )
            # 臂角可视化 Markers (RViz 中显示肘部方向箭头)
            self.marker_pub = self.create_publisher(
                MarkerArray, '/elbow_angle_visualization', 10
            )
            self.get_logger().info('已启用 Topic 发布:')
            self.get_logger().info('  - /left_arm_target_pose, /right_arm_target_pose')
            self.get_logger().info('  - /left_arm_elbow_direction, /right_arm_elbow_direction')
            self.get_logger().info('  - /elbow_angle_visualization (RViz Markers)')

        # 2. 遗留 Topic 发布 (调试用，/pico_*)
        self.hmd_pub = None
        self.tracker_pubs = {}
        if self.enable_legacy_topics:
            self.hmd_pub = self.create_publisher(PoseStamped, f'/{self.topic_prefix}_hmd', 10)
            # 使用统一命名规范: pico_left_wrist, pico_right_wrist, pico_left_arm, pico_right_arm
            self.tracker_pubs['pico_left_wrist'] = self.create_publisher(
                PoseStamped, f'/{self.topic_prefix}_left_wrist', 10)
            self.tracker_pubs['pico_right_wrist'] = self.create_publisher(
                PoseStamped, f'/{self.topic_prefix}_right_wrist', 10)
            self.tracker_pubs['pico_left_arm'] = self.create_publisher(
                PoseStamped, f'/{self.topic_prefix}_left_arm', 10)
            self.tracker_pubs['pico_right_arm'] = self.create_publisher(
                PoseStamped, f'/{self.topic_prefix}_right_arm', 10)
            self.get_logger().info(f'已启用遗留 Topic 发布: /{self.topic_prefix}_*')

        # ==================== 数据源初始化 (TDD 重构) ====================
        # 使用依赖注入或工厂模式创建数据源
        if data_source is None:
            data_source = self._create_data_source()

        self.data_source = data_source

        # 初始化数据源
        if self.data_source.initialize():
            self.get_logger().info(f'数据源已初始化: {self.data_source}')
        else:
            self.get_logger().error(f'数据源初始化失败: {self.data_source}')

        # ==================== 警告节流 ====================
        self._last_warn_time = None
        self._warn_interval = 5.0  # 每 5 秒最多输出一次警告
        self._no_data_count = 0
        self._waiting_for_device = True  # 等待 PICO 设备连接

        # ==================== 定时器 ====================
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_callback)

        self._log_startup()

    def _create_data_source(self) -> DataSource:
        """
        工厂方法: 根据参数创建数据源

        Returns:
            DataSource 实例 (LiveDataSource 或 RecordedDataSource)

        Raises:
            ValueError: 如果 data_source_type 无效
        """
        if self.data_source_type == 'live':
            self.get_logger().info('创建 LiveDataSource (真实 PICO SDK)')
            return LiveDataSource(self.pc_service_host, self.pc_service_port)

        elif self.data_source_type == 'recorded':
            file_path = self.get_parameter('recorded_file_path').value
            playback_speed = self.get_parameter('playback_speed').value
            loop_playback = self.get_parameter('loop_playback').value

            self.get_logger().info(f'创建 RecordedDataSource: {file_path} (速度 {playback_speed}x)')
            return RecordedDataSource(file_path, playback_speed, loop_playback)

        else:
            raise ValueError(f'未知的 data_source_type: {self.data_source_type}')

    def _init_callback(self, request, response):
        """服务回调: 初始化 (增量模式，使用抽象数据源)"""
        if not self.data_source.is_available():
            response.success = False
            response.message = '数据源不可用'
            return response

        # 从数据源获取头显数据
        headset_data = self.data_source.get_headset_pose()
        if headset_data is None or not headset_data.is_valid:
            response.success = False
            response.message = 'HMD 数据不可用'
            return response

        # 从数据源获取 tracker 数据
        tracker_data_list = self.data_source.get_tracker_data()

        self._do_init(headset_data, tracker_data_list)
        response.success = True
        response.message = f'初始化完成! 增量控制模式, 记录了 {len(self.controller.init_tracker_poses)} 个 tracker'
        return response

    def _reset_callback(self, request, response):
        """服务回调: 重置初始化"""
        self.controller.reset()
        self.init_start_time = None
        response.success = True
        response.message = '已重置，等待重新初始化'
        self.get_logger().warning('初始化已重置!')
        return response

    def _publish_static_chest_frames(self):
        """
        发布静态 TF: world → world_left, world_right

        这些是机器人左右臂的 chest 坐标系，作为 PICO tracker 的父坐标系。
        与 step3/step4 保持一致的 TF 树结构。
        """
        now = self.get_clock().now()
        transforms = []

        # world → world_left (左臂 chest, Y=+0.2m)
        t_left = TransformStamped()
        t_left.header.stamp = now.to_msg()
        t_left.header.frame_id = 'world'
        t_left.child_frame_id = 'world_left'
        t_left.transform.translation.x = float(self.world_to_chest_trans['left'][0])
        t_left.transform.translation.y = float(self.world_to_chest_trans['left'][1])
        t_left.transform.translation.z = float(self.world_to_chest_trans['left'][2])
        # TF 需要 chest→world 方向的四元数（world_to_chest 的共轭）
        tf_quat_left = get_tf_quaternion('left')
        t_left.transform.rotation.x = float(tf_quat_left[0])
        t_left.transform.rotation.y = float(tf_quat_left[1])
        t_left.transform.rotation.z = float(tf_quat_left[2])
        t_left.transform.rotation.w = float(tf_quat_left[3])
        transforms.append(t_left)

        # world → world_right (右臂 chest, Y=-0.2m)
        t_right = TransformStamped()
        t_right.header.stamp = now.to_msg()
        t_right.header.frame_id = 'world'
        t_right.child_frame_id = 'world_right'
        t_right.transform.translation.x = float(self.world_to_chest_trans['right'][0])
        t_right.transform.translation.y = float(self.world_to_chest_trans['right'][1])
        t_right.transform.translation.z = float(self.world_to_chest_trans['right'][2])
        tf_quat_right = get_tf_quaternion('right')
        t_right.transform.rotation.x = float(tf_quat_right[0])
        t_right.transform.rotation.y = float(tf_quat_right[1])
        t_right.transform.rotation.z = float(tf_quat_right[2])
        t_right.transform.rotation.w = float(tf_quat_right[3])
        transforms.append(t_right)

        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info('✓ 静态 TF 已发布: world → world_left, world_right')

    def _do_init(self, headset_data: HeadsetData, tracker_data_list: list):
        """执行初始化: 收集 tracker 位姿，委托给增量控制器"""
        hmd_pose_array = headset_data.to_pose_array()

        num_trackers = len(tracker_data_list)

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'收到 {num_trackers} 个 tracker 数据')

        # 记录序列号 (调试用)
        serial_numbers = [t.serial_number for t in tracker_data_list]
        if serial_numbers:
            self.get_logger().info(f'  序列号: {serial_numbers}')

        # 收集有效 tracker 位姿
        tracker_poses = {}
        self.get_logger().info('  处理 tracker 数据:')
        for i, tracker_data in enumerate(tracker_data_list):
            role = self._get_role_for_tracker(tracker_data)

            sn_display = tracker_data.serial_number
            pos = tracker_data.position
            is_valid = tracker_data.is_valid

            self.get_logger().info(
                f'    [{i}] {sn_display} → {role}: pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}], valid={is_valid}'
            )

            if is_valid and role:
                tracker_poses[role] = tracker_data.to_pose_array()

        # 委托给增量控制器
        recorded_roles = self.controller.initialize(tracker_poses, hmd_pose_array)

        self.get_logger().info('-' * 60)
        self.get_logger().info('✓ 初始化完成! (增量控制模式)')
        self.get_logger().info('  坐标变换: PICO (+X右,+Y上,+Z前) → 机器人 (+X前,+Y左,+Z上)')
        self.get_logger().info(f'  记录了 {len(recorded_roles)} 个 tracker 初始位置:')
        for role in recorded_roles:
            T = self.controller.init_tracker_poses[role]
            pos = T[:3, 3]
            self.get_logger().info(f'    {role}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]')

        # 检查缺失的 tracker
        expected_roles = set(self.tracker_serial_map.values())
        missing_roles = expected_roles - recorded_roles
        if missing_roles:
            self.get_logger().warning(f'  缺失的 tracker: {missing_roles}')
            self.get_logger().warning('  请检查:')
            self.get_logger().warning('    1. 所有 tracker 是否已开启并配对')
            self.get_logger().warning('    2. pico_input.yaml 中序列号映射是否正确')
            self.get_logger().warning('    3. PICO XRoboToolkit App 是否检测到所有 tracker')

        self.get_logger().info('-' * 60)
        self.get_logger().info('机器人初始位置 (来自 tianji_robot.yaml):')
        for role, pos in self.controller.robot_init_positions.items():
            self.get_logger().info(f'    {role}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]')
        self.get_logger().info('-' * 60)
        self.get_logger().info('增量控制模式:')
        self.get_logger().info('  用户手移动多少 → 机器人手移动多少')
        self.get_logger().info('  不会因为初始化姿势不同而导致机器人跳动')
        self.get_logger().info('=' * 60)

    def _extract_serial_number(self, full_sn: str) -> str:
        """从完整序列号中提取 6 位数字 (如 PC2310MLKC190058G -> 190058)"""
        match = re.search(r'(\d{6})', full_sn)
        return match.group(1) if match else None

    def _get_role_for_tracker(self, tracker_data: TrackerData) -> str:
        """
        根据 TrackerData 获取角色

        Args:
            tracker_data: TrackerData 对象

        Returns:
            角色字符串 (pico_left_wrist, pico_right_wrist, etc.) 或 None
        """
        # 提取 6 位数字序列号
        short_sn = self._extract_serial_number(tracker_data.serial_number)

        # 优先使用序列号映射
        if short_sn and short_sn in self.tracker_serial_map:
            return self.tracker_serial_map[short_sn]

        # 回退: 无法确定角色
        self.get_logger().warning(
            f'无法为 tracker {tracker_data.serial_number} 找到角色映射'
        )
        return None

    def _publish_callback(self):
        """主循环: 从数据源读取数据并发布 TF + Topics"""
        if not self.data_source.is_available():
            return

        now = self.get_clock().now()

        try:
            # 从数据源获取头显数据
            headset_data = self.data_source.get_headset_pose()

            if headset_data is None or not headset_data.is_valid:
                self._handle_no_data(now)
                return

            # 设备已连接，清除等待状态
            if self._waiting_for_device:
                self._waiting_for_device = False
                self._no_data_count = 0
                self.get_logger().info('✓ 数据源已就绪，正在接收数据...')

            # 从数据源获取 tracker 数据
            tracker_data_list = self.data_source.get_tracker_data()

            # 自动初始化逻辑
            if not self.controller.initialized and self.auto_init_delay > 0:
                if self.init_start_time is None:
                    self.init_start_time = now
                    self.get_logger().info(f'等待 {self.auto_init_delay:.1f} 秒后自动初始化...')
                    self.get_logger().info('  建议模仿机器人当前姿势以获得更好的控制感')
                    self.get_logger().info('  或调用: ros2 service call /pico_input/init std_srvs/srv/Trigger')
                else:
                    elapsed = (now - self.init_start_time).nanoseconds / 1e9
                    if elapsed >= self.auto_init_delay:
                        self._do_init(headset_data, tracker_data_list)

            if not self.controller.initialized:
                return

            # 转换 HMD 到世界坐标系并发布 TF (仅用于可视化)
            hmd_pose_array = headset_data.to_pose_array()
            head_T = self.controller.compute_hmd_world_pose(hmd_pose_array)
            if head_T is not None:
                pos = head_T[:3, 3]
                quat = R.from_matrix(head_T[:3, :3]).as_quat()

                # 检查转换后的四元数有效性
                quat_norm = np.linalg.norm(quat)
                if quat_norm > 0.9 and quat_norm < 1.1:
                    self._broadcast_tf(now, 'world', 'head', pos, quat)

                    # 遗留 Topic (可选，调试用)
                    if self.enable_legacy_topics and self.hmd_pub:
                        self._publish_pose(self.hmd_pub, now, 'world', pos, quat)

            # 处理 Trackers
            self._process_trackers(now, tracker_data_list)

        except Exception as e:
            self._handle_error(now, str(e))

    def _handle_no_data(self, now):
        """处理无数据情况 (PICO 未连接)"""
        self._no_data_count += 1

        # 仅在等待设备时，每隔一段时间输出提示
        if self._waiting_for_device:
            current_time = now.nanoseconds / 1e9
            if self._last_warn_time is None or (current_time - self._last_warn_time) >= self._warn_interval:
                self._last_warn_time = current_time
                self.get_logger().warning(
                    f'等待 PICO 设备连接... (已等待 {self._no_data_count} 帧)\n'
                    '  请确保:\n'
                    '  1. PICO 头显已开启并运行 XRoboToolkit App\n'
                    '  2. PICO 与 PC 在同一 WiFi 网络\n'
                    '  3. App 已连接到 PC IP 地址'
                )

    def _handle_error(self, now, error_msg: str):
        """处理错误 (带节流)"""
        current_time = now.nanoseconds / 1e9
        if self._last_warn_time is None or (current_time - self._last_warn_time) >= self._warn_interval:
            self._last_warn_time = current_time
            # 简化错误信息
            if 'zero norm quaternions' in error_msg:
                self.get_logger().warning('PICO 数据无效 (四元数为零) - 请检查设备连接')
            else:
                self.get_logger().warning(f'数据处理异常: {error_msg}')

    def _process_trackers(self, now, tracker_data_list: list):
        """
        处理 Motion Tracker 数据，计算增量位姿

        Args:
            now: 当前时间戳
            tracker_data_list: List[TrackerData] 从数据源获取
        """
        if not tracker_data_list:
            return

        # 用于 elbow direction 计算的临时存储
        tf_data = {}      # role -> (pos, quat) (robot 坐标系)

        for i, tracker_data in enumerate(tracker_data_list):
            # 获取角色
            role = self._get_role_for_tracker(tracker_data)
            if not role:
                continue

            # 检查数据有效性
            if not tracker_data.is_valid:
                continue

            # 转换为 pose 数组
            pose = tracker_data.to_pose_array()

            # 计算增量位姿 (委托给控制器)
            pos, quat = self.controller.compute_target_pose(pose, role)
            if pos is None:
                continue

            tf_data[role] = (pos, quat)

            # 确定父 frame (chest 坐标系)
            if 'left' in role:
                parent_frame = 'world_left'
            else:
                parent_frame = 'world_right'

            # 1. TF 广播: world_left/world_right → pico_left_wrist/pico_left_arm 等
            self._broadcast_tf(now, parent_frame, role, pos, quat)

            # 2. Topic 发布 (tianji_world_output 订阅)
            if self.enable_topic_publishing:
                if role == 'pico_left_wrist':
                    self._publish_pose(self.left_arm_pose_pub, now, parent_frame, pos, quat)
                elif role == 'pico_right_wrist':
                    self._publish_pose(self.right_arm_pose_pub, now, parent_frame, pos, quat)

            # 3. 遗留 Topic 发布: /pico_left_wrist, /pico_right_wrist, etc. (调试用)
            if self.enable_legacy_topics and role in self.tracker_pubs:
                self._publish_pose(self.tracker_pubs[role], now, parent_frame, pos, quat)

        # ==================== 计算并发布 elbow direction ====================
        if self.enable_topic_publishing:
            self._publish_elbow_directions(now, tf_data)

    def _broadcast_tf(self, now, parent: str, child: str, pos, quat):
        """广播 TF"""
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(t)

    def _publish_pose(self, publisher, now, frame_id: str, pos, quat):
        """发布 PoseStamped (用于 RViz/仿真)"""
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        publisher.publish(msg)

    def _publish_elbow_directions(self, now, tf_data: dict):
        """
        发布肘部方向向量 (臂角约束) + RViz 可视化 Markers

        从 arm tracker 位置计算几何方向(指向重力/下方)，取反后发布给 IK (反重力方向)。
        RViz Markers 使用物理方向(不取反)，与绿色肩-肘箭头一致。
        """
        # 收集手腕和前臂位置
        wrist_positions = {}
        arm_positions = {}
        for role, (pos, quat) in tf_data.items():
            side = 'left' if 'left' in role else 'right'
            if 'wrist' in role:
                wrist_positions[side] = pos.copy()
            elif 'arm' in role:
                arm_positions[side] = pos.copy()

        markers = MarkerArray()
        marker_id = 0

        for side in ['left', 'right']:
            parent_frame = f'world_{side}'
            pub = self.left_elbow_dir_pub if side == 'left' else self.right_elbow_dir_pub
            default_dir = self.controller.default_zsp_direction[side]

            wrist_pos = wrist_positions.get(side)
            elbow_pos = arm_positions.get(side)

            if wrist_pos is not None and elbow_pos is not None:
                shoulder_pos = np.array([0.0, 0.0, 0.0])
                direction, proj_point = self.controller.compute_elbow_direction(
                    shoulder_pos, wrist_pos, elbow_pos, side
                )

                # 几何方向(指向重力) → IK方向(反重力): 取反
                ik_direction = -direction
                self._publish_vector3(pub, now, parent_frame, ik_direction)

                # 诊断日志 (每 90 帧 ≈ 每秒)
                if self._elbow_log_counter % 90 == 0:
                    sw_len = float(np.linalg.norm(wrist_pos))
                    offset_len = float(np.linalg.norm(elbow_pos - proj_point))
                    dot_default = float(np.dot(ik_direction, default_dir))
                    self.get_logger().info(
                        f'[ELBOW {side}] wrist=[{wrist_pos[0]:.3f},{wrist_pos[1]:.3f},{wrist_pos[2]:.3f}] '
                        f'arm=[{elbow_pos[0]:.3f},{elbow_pos[1]:.3f},{elbow_pos[2]:.3f}] '
                        f'sw={sw_len:.3f} offset={offset_len:.4f}'
                    )
                    self.get_logger().info(
                        f'[ELBOW {side}] phys=[{direction[0]:.3f},{direction[1]:.3f},{direction[2]:.3f}] '
                        f'IK=[{ik_direction[0]:.3f},{ik_direction[1]:.3f},{ik_direction[2]:.3f}] '
                        f'dot_default={dot_default:.3f}'
                    )

                # 可视化 Markers (使用物理方向 direction，与绿色肩-肘箭头一致)
                side_markers = self._create_elbow_markers(
                    now, parent_frame, shoulder_pos, wrist_pos, elbow_pos,
                    proj_point, direction, side, marker_id
                )
                markers.markers.extend(side_markers)
                marker_id += len(side_markers)

        self._elbow_log_counter += 1

        # 发布可视化 markers
        if markers.markers and self.marker_pub:
            self.marker_pub.publish(markers)

    def _create_elbow_markers(self, now, frame_id: str,
                               shoulder: np.ndarray, wrist: np.ndarray,
                               elbow: np.ndarray, proj_point: np.ndarray,
                               direction: np.ndarray, side: str,
                               start_id: int) -> list:
        """
        创建臂角可视化 RViz Markers (参考 step4)

        - 红色箭头: 肩 → 腕
        - 绿色箭头: 肩 → 肘
        - 蓝色箭头: 投影点 → 肘部偏移方向 (elbow_direction)
        - 黄色小球: 投影点
        """
        markers = []
        timestamp = now.to_msg()
        lifetime_ns = int(0.15 * 1e9)

        def _pt(arr):
            p = Point()
            p.x, p.y, p.z = float(arr[0]), float(arr[1]), float(arr[2])
            return p

        # 1. 红色箭头: 肩 → 腕
        m_sw = Marker()
        m_sw.header.stamp = timestamp
        m_sw.header.frame_id = frame_id
        m_sw.ns = f'{side}_arm_axis'
        m_sw.id = start_id
        m_sw.type = Marker.ARROW
        m_sw.action = Marker.ADD
        m_sw.points = [_pt(shoulder), _pt(wrist)]
        m_sw.scale.x = 0.01
        m_sw.scale.y = 0.02
        m_sw.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        m_sw.lifetime.nanosec = lifetime_ns
        markers.append(m_sw)

        # 2. 绿色箭头: 肩 → 肘
        m_se = Marker()
        m_se.header.stamp = timestamp
        m_se.header.frame_id = frame_id
        m_se.ns = f'{side}_shoulder_to_elbow'
        m_se.id = start_id + 1
        m_se.type = Marker.ARROW
        m_se.action = Marker.ADD
        m_se.points = [_pt(shoulder), _pt(elbow)]
        m_se.scale.x = 0.01
        m_se.scale.y = 0.02
        m_se.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        m_se.lifetime.nanosec = lifetime_ns
        markers.append(m_se)

        # 3. 蓝色箭头: 投影点 → 肘部偏移方向 (elbow_direction)
        arrow_length = 0.15
        arrow_end = proj_point + direction * arrow_length
        m_dir = Marker()
        m_dir.header.stamp = timestamp
        m_dir.header.frame_id = frame_id
        m_dir.ns = f'{side}_elbow_direction'
        m_dir.id = start_id + 2
        m_dir.type = Marker.ARROW
        m_dir.action = Marker.ADD
        m_dir.points = [_pt(proj_point), _pt(arrow_end)]
        m_dir.scale.x = 0.015
        m_dir.scale.y = 0.025
        m_dir.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)
        m_dir.lifetime.nanosec = lifetime_ns
        markers.append(m_dir)

        # 4. 黄色小球: 投影点
        m_proj = Marker()
        m_proj.header.stamp = timestamp
        m_proj.header.frame_id = frame_id
        m_proj.ns = f'{side}_projection_point'
        m_proj.id = start_id + 3
        m_proj.type = Marker.SPHERE
        m_proj.action = Marker.ADD
        m_proj.pose.position = _pt(proj_point)
        m_proj.pose.orientation.w = 1.0
        m_proj.scale.x = m_proj.scale.y = m_proj.scale.z = 0.02
        m_proj.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
        m_proj.lifetime.nanosec = lifetime_ns
        markers.append(m_proj)

        return markers

    def _publish_vector3(self, publisher, now, frame_id: str, vector: np.ndarray):
        """发布 Vector3Stamped (用于 elbow direction)"""
        if publisher is None:
            return

        msg = Vector3Stamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = frame_id
        msg.vector.x = float(vector[0])
        msg.vector.y = float(vector[1])
        msg.vector.z = float(vector[2])
        publisher.publish(msg)

    def _log_startup(self):
        """打印启动信息"""
        # 获取本机 IP 地址
        local_ip = "127.0.0.1"
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
        except Exception:
            pass

        mode_parts = []
        if self.enable_topic_publishing:
            mode_parts.append('Topics (/left_arm_target_pose, /right_arm_target_pose)')
        if self.enable_legacy_topics:
            mode_parts.append(f'Legacy Topics (/{self.topic_prefix}/*)')
        mode_parts.append('TF')
        mode = ' + '.join(mode_parts)

        self.get_logger().info('=' * 70)
        self.get_logger().info('PICO Input Node (TDD 重构) - 增量控制模式')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'  数据源: {self.data_source}')
        self.get_logger().info(f'  状态: {"已就绪" if self.data_source.is_available() else "未就绪"}')
        self.get_logger().info(f'  频率: {self.publish_rate} Hz')
        self.get_logger().info(f'  输出模式: {mode}')
        self.get_logger().info('  臂角控制: 始终开启 (动态 elbow direction, One-Euro 自适应平滑)')
        self.get_logger().info(f'  滤波: One-Euro Filter (位置+姿态+肘部方向 统一自适应平滑)')
        self.get_logger().info(f'  Tracker 映射: {self.tracker_serial_map}')
        self.get_logger().info('-' * 70)

        if self.data_source_type == 'live':
            self.get_logger().info('Live 模式 - PICO SDK 连接配置:')
            self.get_logger().info(f'  PC IP 地址: {local_ip}')
            self.get_logger().info(f'  端口: {self.pc_service_port}')
            self.get_logger().info('')
            self.get_logger().info('  在 PICO 头显中打开 XRoboToolkit App，填写:')
            self.get_logger().info(f'    服务器地址: {local_ip}')
            self.get_logger().info(f'    端口: {self.pc_service_port}')
            self.get_logger().info('')
            self.get_logger().info('  ⚠️ 确保 PICO 与 PC 在同一 WiFi 网络!')
        elif self.data_source_type == 'recorded':
            file_path = self.get_parameter('recorded_file_path').value
            speed = self.get_parameter('playback_speed').value
            self.get_logger().info('Recorded 模式 - 数据回放:')
            self.get_logger().info(f'  文件: {file_path}')
            self.get_logger().info(f'  速度: {speed}x')

        self.get_logger().info('-' * 70)
        self.get_logger().info('增量控制模式:')
        self.get_logger().info('  - 初始化时记录四个 tracker 的位置')
        self.get_logger().info('  - 用户手移动多少 → 机器人手移动多少')
        self.get_logger().info('  - 坐标变换: PICO(+X右,+Y上,+Z前) → 机器人(+X前,+Y左,+Z上)')
        self.get_logger().info('-' * 65)
        self.get_logger().info('机器人初始位置 (来自 tianji_robot.yaml):')
        for role, pos in self.controller.robot_init_positions.items():
            self.get_logger().info(f'  {role}: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]')
        self.get_logger().info('-' * 65)
        self.get_logger().info('服务:')
        self.get_logger().info('  ros2 service call /pico_input/init std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /pico_input/reset std_srvs/srv/Trigger')
        if self.auto_init_delay > 0:
            self.get_logger().info(f'  自动初始化: {self.auto_init_delay:.1f} 秒后')
        self.get_logger().info('=' * 65)

    def destroy_node(self):
        """清理资源"""
        # 关闭数据源 (替换原有的 SDK 关闭)
        if hasattr(self, 'data_source') and self.data_source:
            self.data_source.close()
            self.get_logger().info(f'数据源已关闭: {self.data_source}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PicoInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 检查 rclpy 是否已经 shutdown
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # 忽略 shutdown 错误


if __name__ == '__main__':
    main()
