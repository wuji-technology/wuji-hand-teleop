#!/usr/bin/env python3
"""
=============================================================================
Step 5: 使用录制数据增量控制真机器人（验证完整遥操流程）
=============================================================================

功能：读取 PICO 录制数据 → 坐标变换 → 计算增量位姿 → 控制真实机器人

用途：
  - 验证完整的 PICO → Robot 遥操流程是否正确
  - 使用录制数据进行可重复测试
  - 不需要 PICO 硬件，只需要机器人

⚠️ 前置条件：
  - ✓ step4 测试通过（坐标变换正确）
  - ✓ 机器人已开机并连接
  - ✓ 机器人处于安全位置

💡 如需单独测试旋转/位置/臂角，请使用 step6_arm_angle_stability_test.py


=============================================================================
坐标变换速查
=============================================================================

PICO → Robot 位置映射:
  Robot X = -PICO Z  (PICO 向前伸手 → Robot 向前)
  Robot Y = -PICO X  (PICO 向右移动 → Robot 向右)
  Robot Z = +PICO Y  (PICO 向上抬手 → Robot 向上)

PICO → Robot 旋转轴映射 (ROS REP 103 右手定则，从轴正端向原点看逆时针为正):
  绕 PICO +X → 绕 Robot -Y → 手指向下压 (Pitch down)
  绕 PICO -X → 绕 Robot +Y → 手指向上翘 (Pitch up)
  绕 PICO Y → 绕 Robot +Z (手指向右偏/偏航)
  绕 PICO Z → 绕 Robot -X (手腕向右翻滚)


=============================================================================
使用方法
=============================================================================

  步骤 1: 干运行测试（推荐先执行）
  --------------------------------
  cd /home/wuji-08/Desktop/wuji-teleop-ros2-private/src/input_devices/pico_input/test

  # 干运行：仅打印位姿，不控制机器人
  python3 step5_incremental_control_with_robot.py --dry-run

  # 验证输出位姿是否合理（应该在 TIANJI_INIT_POS 附近）


  步骤 2: 将机器人移动到初始位置
  ------------------------------
  python3 tool/move_to_init_pose.py


  步骤 3: 启动增量控制
  --------------------
  # 使用默认静态数据 (trackingData_sample_static.txt, 150 帧静态数据)
  python3 step5_incremental_control_with_robot.py

  # 慢速回放更安全
  python3 step5_incremental_control_with_robot.py --speed 0.3

  # 仅控制左臂
  python3 step5_incremental_control_with_robot.py --left-only

  # 仅控制右臂
  python3 step5_incremental_control_with_robot.py --right-only

  # 使用大数据集
  python3 step5_incremental_control_with_robot.py --file ../record/trackingData_whole_data.txt

  # 指定帧范围测试
  python3 step5_incremental_control_with_robot.py --file ../record/trackingData_whole_data.txt --start-frame 100 --end-frame 200 --left-only --speed 0.5 -v


=============================================================================
相关工具
=============================================================================

  分析录制数据中的运动片段:
    cd ../record && python3 analyze_motion_data.py --file trackingData_whole_data.txt --all

  单独测试旋转/位置/臂角（推荐用于调试）:
    python3 step6_arm_angle_stability_test.py --mode rotation --rot-axis X --rot-angle 30 --left-only
    python3 step6_arm_angle_stability_test.py --mode position --pos-x-range -0.05 0.05 0.01


=============================================================================
测试数据文件（均在 record/ 目录下）
=============================================================================

  高质量静态数据 (推荐初次测试):
    ../record/trackingData_sample_static.txt (150 帧，平均位移 <0.1mm)

  大数据集 (包含明显运动):
    ../record/trackingData_whole_data.txt (5780 帧) - 推荐验证测试
    ../record/trackingData_head_tracker_static.txt (8384 帧)
    ../record/trackingData_head_tracker.txt (5703 帧)

  运动分析工具:
    # 分析位置运动
    python3 ../record/analyze_motion_data.py --file ../record/trackingData_whole_data.txt

    # 分析旋转运动
    python3 ../record/analyze_motion_data.py --file ../record/trackingData_whole_data.txt --rotation

    # 同时分析位置和旋转
    python3 ../record/analyze_motion_data.py --file ../record/trackingData_whole_data.txt --all


=============================================================================
Tracker SN 映射
=============================================================================

  190058 → pico_left_wrist  (左手腕，控制左臂)
  190600 → pico_right_wrist (右手腕，控制右臂)
  190046 → pico_left_arm    (左前臂，臂角约束)
  190023 → pico_right_arm   (右前臂，臂角约束)


=============================================================================
增量控制原理
=============================================================================

  1. 初始化: 记录第一帧的 PICO tracker 位姿
  2. 每一帧:
     a. 计算 PICO 位姿增量: Δ_pico = current - init
     b. 坐标变换: Δ_world = PICO_TO_ROBOT @ Δ_pico
     c. 转换到 chest 坐标系: Δ_chest = WORLD_TO_CHEST @ Δ_world
     d. 计算目标位姿: target = TIANJI_INIT_POS + Δ_chest
     e. 发送给机器人


=============================================================================
与 pico_teleop_minimal.launch.py 的差异
=============================================================================

本脚本和 launch 使用相同的增量控制算法，但有以下区别:

1. 臂角控制:
   - step5 (本脚本): 静态默认臂角。使用 DEFAULT_ZSP_DIRECTION 固定值，
     不使用 arm tracker 数据。肘部始终维持默认沉肘姿态。
     适合验证基本位姿变换是否正确。
   - launch: 实时动态臂角。pico_input_node 从 arm tracker 位置计算
     肘部偏移方向，经灰色区间防抖 + EMA 平滑后发布给 tianji_world_output。
     机器人肘部跟随人的实际肘部位置。

2. 位置平滑:
   - step5 (本脚本): 无平滑，直接使用原始计算值。
   - launch: pico_input_node 对位置做 EMA 平滑 (alpha=0.6)，
     减少 tracker 抖动，动作更柔和但响应略慢。

3. 架构:
   - step5 (本脚本): 直接调用 CartesianController，无中间节点。
   - launch: pico_input → ROS2 topic → tianji_world_output → 机器人。


=============================================================================
安全提示
=============================================================================

⚠️ 首次运行时：
  1. 先使用 --dry-run 验证输出位姿
  2. 确保机器人周围无人和障碍物
  3. 准备好急停按钮
  4. 使用慢速回放 (--speed 0.3) 观察机器人动作

⚠️ 如果机器人动作异常：
  - 立即按 Ctrl+C 停止
  - 检查坐标变换是否正确
  - 回到 step4 验证可视化


=============================================================================
命令行参数
=============================================================================

  --file FILE         数据文件路径 (默认: ../record/trackingData_sample_static.txt)
  --speed SPEED       回放速度倍率 (默认: 1.0)
  --dry-run           干运行模式，仅打印不控制
  --left-only         仅控制左臂
  --right-only        仅控制右臂
  --start-frame N     起始帧序号（从0开始）
  --end-frame N       结束帧序号
  -v, --verbose       显示详细变换日志（PICO增量 → Chest增量）


=============================================================================
下一步
=============================================================================

测试通过后，可以：
  1. 使用真实 PICO 设备进行实时控制
  2. 录制更多测试数据，建立测试数据集
  3. 进行 step6 臂角稳定性测试

=============================================================================
"""

import sys
import time
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

# 导入共享模块（使用相对路径）
_test_dir = str(Path(__file__).resolve().parent)
_pico_input_dir = str(Path(__file__).resolve().parent.parent)
_src_dir = str(Path(__file__).resolve().parent.parent.parent.parent)
_tianji_wo_dir = str(Path(_src_dir) / 'output_devices' / 'tianji_world_output')
for _p in [_test_dir, _pico_input_dir, _tianji_wo_dir]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from common.robot_config import (
    INIT_JOINTS,
    TIANJI_INIT_POS, TIANJI_INIT_ROT,
    WORLD_TO_LEFT_QUAT, WORLD_TO_RIGHT_QUAT,
    ROBOT_IP,
    DEFAULT_ZSP_DIRECTION, ARM_INIT_POS,
)
from common.transform_utils import (
    get_pico_to_robot,
    transform_pico_rotation_to_world,
    transform_world_to_chest,
    apply_world_rotation_to_chest_pose,
)

# 导入数据源和机器人控制
from pico_input.data_source import RecordedDataSource
from tianji_world_output.cartesian_controller import CartesianController


# PICO → 机器人坐标变换 (从共享配置加载)
# 详见 tianji_world_output/transform_utils.py 和 PICO_TELEOP_GUIDE.md §3.1

# World → Chest 转换: 使用 transform_utils 中的共享函数
# (transform_world_to_chest, apply_world_rotation_to_chest_pose 等)

# Tracker SN → 角色映射 (与 step4 保持一致)
TRACKER_SN_MAP = {
    '190058': 'pico_left_wrist',   # PICO 左手腕 → 控制左臂
    '190600': 'pico_right_wrist',  # PICO 右手腕 → 控制右臂
    '190046': 'pico_left_arm',     # PICO 左前臂 (臂角约束)
    '190023': 'pico_right_arm',    # PICO 右前臂 (臂角约束)
}


def compute_elbow_direction(shoulder: np.ndarray, wrist: np.ndarray,
                            elbow: np.ndarray, side: str) -> np.ndarray:
    """
    计算肘部相对于肩-腕平面的偏移方向

    ============================================================================
    臂角 (Arm Angle) 计算原理
    ============================================================================

    这是零空间 IK 约束的核心：指定肘部在空间中的期望偏移方向。

    几何原理：
                肩膀 (shoulder)
                  ●
                 /|\\
                / | \\
               /  |  \\
         肩-肘 /   |   \\ 肩-腕
              /    |    \\
             ●─────●─────●
          肘部   投影点   手腕

            ←────→
          肘部偏移向量 (direction)

    计算步骤:
    1. 肩-腕向量: 从肩膀到手腕的直线（手臂主轴）
    2. 肩-肘向量: 从肩膀到肘部（前臂 tracker 位置）
    3. 投影: 将肘部投影到肩-腕直线上
    4. 偏移向量: 实际肘部位置 - 投影点 = 肘部偏离主轴的方向
    5. 归一化: 得到单位方向向量

    ============================================================================
    沉肘方向说明 (Chest 坐标系)
    ============================================================================

    Chest 坐标系定义:
      Left Chest:  +Y = -Z_world (向下), +Z = +Y_world (向左)
      Right Chest: +Y = +Z_world (向上), +Z = -Y_world (向右)

    默认臂角参考平面参数（与 cartesian_controller 一致）:
    - 左臂: [0, -1, -0.5]  → Y-(反向重力) + Z-(向外侧/右)
    - 右臂: [0, +1, -0.5]  → Y+(反向重力) + Z-(向外侧/左)

    注意：左右臂的差异已在 arm tracker 的初始位置 (robot_init_pos) 中处理，
    所以这里计算的 direction 直接使用即可。

    Args:
        shoulder: 肩膀位置 (chest 坐标系原点, [0,0,0])
        wrist: 手腕位置 (从 PICO wrist tracker 转换)
        elbow: 肘部位置 (从 PICO arm tracker 转换)
        side: 'left' 或 'right'

    Returns:
        direction: 单位方向向量 [x, y, z]
    """
    # 默认臂角参考平面参数 (从统一配置加载)
    default_direction = DEFAULT_ZSP_DIRECTION[side]

    # 肩-腕向量
    shoulder_to_wrist = wrist - shoulder
    sw_length = np.linalg.norm(shoulder_to_wrist)

    if sw_length < 1e-6:
        # 手腕太靠近肩膀，使用默认沉肘方向
        return default_direction

    sw_unit = shoulder_to_wrist / sw_length

    # 肩-肘向量
    shoulder_to_elbow = elbow - shoulder

    # 肘部在肩-腕连线上的投影长度
    proj_length = np.dot(shoulder_to_elbow, sw_unit)

    # 投影点
    proj_point = shoulder + proj_length * sw_unit

    # 肘部偏移向量
    elbow_offset = elbow - proj_point
    offset_length = np.linalg.norm(elbow_offset)

    if offset_length < 1e-6:
        # 肘部正好在肩-腕连线上，使用默认沉肘方向
        return default_direction

    # 归一化得到方向向量
    direction = elbow_offset / offset_length

    return direction


def compute_incremental_pose(current_pose, init_pose, robot_init_pos, robot_init_rot, is_left: bool):
    """
    计算增量位姿（与 IncrementalController.compute_target_pose 相同算法）

    使用与 step1/step2 一致的 4 步算法，确保旋转变换正确。

    Args:
        current_pose: [x, y, z, qx, qy, qz, qw] 当前位姿 (PICO 坐标系)
        init_pose: [x, y, z, qx, qy, qz, qw] 初始位姿 (PICO 坐标系)
        robot_init_pos: 机器人初始位置 (chest 坐标系)
        robot_init_rot: 机器人初始姿态 (Rotation 对象，chest 坐标系)
        is_left: 是否为左臂

    Returns:
        (target_pos, target_quat) 目标位姿 (chest 坐标系)
    """
    side = 'left' if is_left else 'right'

    # 位置增量
    delta_pos_pico = np.array(current_pose[:3]) - np.array(init_pose[:3])
    delta_pos_world = get_pico_to_robot() @ delta_pos_pico

    # 转换到 chest 坐标系（使用共享库）
    delta_pos_chest = transform_world_to_chest(delta_pos_world, side)

    target_pos = robot_init_pos + delta_pos_chest

    # 姿态增量
    init_rot = R.from_quat(init_pose[3:7])
    current_rot = R.from_quat(current_pose[3:7])
    delta_rot_pico = current_rot * init_rot.inv()

    # 使用共享库: PICO→World 旋转变换 + 4 步算法
    delta_rot_world = transform_pico_rotation_to_world(delta_rot_pico, get_pico_to_robot())
    robot_init_rot_mat = robot_init_rot.as_matrix()
    target_rot_in_chest = apply_world_rotation_to_chest_pose(
        robot_init_rot_mat, delta_rot_world, side
    )

    target_rot = R.from_matrix(target_rot_in_chest)
    target_quat = target_rot.as_quat()

    return target_pos, target_quat


def main():
    parser = argparse.ArgumentParser(description='Step 5: 使用录制数据增量控制真机器人')
    parser.add_argument('--file', type=str,
                        default='../record/trackingData_sample_static.txt',
                        help='录制数据文件路径（相对于 test/ 目录）')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='回放速度倍率 (0.5=慢速, 1.0=实时, 2.0=快速)')
    parser.add_argument('--dry-run', action='store_true',
                        help='干运行模式（仅打印，不控制机器人）')
    parser.add_argument('--left-only', action='store_true',
                        help='仅控制左臂')
    parser.add_argument('--right-only', action='store_true',
                        help='仅控制右臂')
    parser.add_argument('--start-frame', type=int, default=0,
                        help='起始帧序号（从0开始）')
    parser.add_argument('--end-frame', type=int, default=None,
                        help='结束帧序号（不指定则播放到结尾）')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='显示详细变换日志（PICO增量 → 机器人增量）')
    args = parser.parse_args()

    # 确定数据文件路径
    data_file = Path(args.file)
    if not data_file.is_absolute():
        # 相对路径，从 pico_input/test 目录查找
        test_dir = Path(__file__).parent
        data_file = test_dir / args.file

    if not data_file.exists():
        print(f"❌ 错误: 找不到数据文件 {data_file}")
        return

    print("=" * 70)
    print("Step 5: 使用录制数据增量控制真机器人")
    print("=" * 70)
    print(f"  数据文件: {data_file}")
    print(f"  回放速度: {args.speed}x")
    print(f"  干运行模式: {args.dry_run}")
    print(f"  帧范围: {args.start_frame} - {args.end_frame if args.end_frame else '末尾'}")
    print(f"  控制模式: ", end="")
    if args.left_only:
        print("仅左臂")
    elif args.right_only:
        print("仅右臂")
    else:
        print("双臂")
    print("=" * 70)

    # 创建数据源
    print("\n[1/4] 加载录制数据...")
    data_source = RecordedDataSource(
        str(data_file),
        playback_speed=args.speed,
        loop=False  # step5 不循环，播放一次
    )

    if not data_source.initialize():
        print("❌ 错误: 数据源初始化失败")
        return

    print(f"✓ 数据加载成功，共 {len(data_source.data_frames)} 帧")

    # 连接机器人（如果不是干运行）
    controller = None
    if not args.dry_run:
        print("\n[2/4] 连接机器人...")
        try:
            controller = CartesianController(robot_ip=ROBOT_IP)
            print(f"✓ 已连接到机器人 {ROBOT_IP}")

            # 使能机器人（关键步骤！）
            print("  设置阻抗模式...")
            controller.set_impedance_mode(mode='joint')
            print("  ✓ 机器人已使能")

            # 移动到初始位姿
            print("  移动到初始位姿...")
            controller.move_to_init(
                wait=True, timeout=3,
                init_joints_left=INIT_JOINTS['left'],
                init_joints_right=INIT_JOINTS['right']
            )
            print("  ✓ 已到达初始位姿")

        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return
    else:
        print("\n[2/4] 跳过机器人连接（干运行模式）")

    # 初始化：跳过到起始帧并记录初始位姿
    print("\n[3/4] 初始化（记录初始位姿）...")
    time.sleep(0.1)

    # 直接跳到起始帧（直接设置数据源内部状态，绕过时间同步）
    if args.start_frame > 0:
        if args.start_frame >= len(data_source.data_frames):
            print(f"❌ 错误: 起始帧 {args.start_frame} 超出数据范围 (共 {len(data_source.data_frames)} 帧)")
            return
        print(f"  跳到帧 {args.start_frame}...")
        # 直接设置起始索引和时间戳基准
        data_source.current_index = args.start_frame
        data_source.first_timestamp = data_source.data_frames[args.start_frame].get('predictTime', 0)
        data_source.start_time = time.time()

    tracker_data_list = data_source.get_tracker_data()
    if not tracker_data_list:
        print("❌ 错误: 无法获取 tracker 数据")
        return

    # 记录初始位姿
    init_poses = {}
    import re

    for tracker in tracker_data_list:
        # 提取6位序列号
        match = re.search(r'(\d{6})', tracker.serial_number)
        if not match:
            continue

        short_sn = match.group(1)
        role = TRACKER_SN_MAP.get(short_sn)

        if role and tracker.is_valid:
            init_poses[role] = tracker.to_pose_array()
            print(f"  {role}: pos={tracker.position}, valid={tracker.is_valid}")

    if not init_poses:
        print("❌ 错误: 未找到有效的 tracker 数据")
        return

    print(f"✓ 已记录 {len(init_poses)} 个 tracker 初始位姿")

    # 主循环：回放并控制
    print("\n[4/4] 开始回放并控制机器人...")
    print("  按 Ctrl+C 停止\n")

    frame_count = 0
    start_time = time.time()

    # 计算结束帧索引
    end_index = args.end_frame if args.end_frame is not None else len(data_source.data_frames) - 1
    end_index = min(end_index, len(data_source.data_frames) - 1)

    # 辅助函数：解析位姿字符串
    def parse_pose_string(pose_str):
        values = [float(x.strip()) for x in pose_str.split(',')]
        return np.array(values[:7], dtype=np.float64)

    try:
        # 直接按索引遍历帧，而不是依赖时间同步
        for frame_idx in range(args.start_frame + 1, end_index + 1):
            frame = data_source.data_frames[frame_idx]
            frame_count += 1
            actual_frame = frame_idx

            # 解析当前帧的 tracker 数据
            motion = frame.get('Motion', {})
            joints = motion.get('joints', [])

            # 第一遍：收集所有 tracker 的位姿
            current_poses = {}  # {role: (pos, quat)}
            for joint in joints:
                sn = joint.get('sn', '')
                pose_str = joint.get('p', '')
                if not pose_str:
                    continue

                match = re.search(r'(\d{6})', sn)
                if not match:
                    continue

                short_sn = match.group(1)
                role = TRACKER_SN_MAP.get(short_sn)

                if role and role in init_poses:
                    is_left = 'left' in role
                    side = 'left' if is_left else 'right'

                    # 解析当前位姿
                    current_pose = parse_pose_string(pose_str)

                    # 计算 PICO 增量（用于 verbose 日志）
                    pico_delta = current_pose[:3] - init_poses[role][:3]

                    # 关键：arm tracker 和 wrist tracker 使用不同的初始位置
                    if 'arm' in role and 'wrist' not in role:
                        # 前臂 tracker: 从配置加载初始位置
                        robot_init_pos = ARM_INIT_POS[side]
                    else:
                        robot_init_pos = TIANJI_INIT_POS[side]

                    robot_init_rot = R.from_matrix(TIANJI_INIT_ROT[side])

                    target_pos, target_quat = compute_incremental_pose(
                        current_pose,
                        init_poses[role],
                        robot_init_pos,
                        robot_init_rot,
                        is_left
                    )
                    # 计算机器人增量（相对于初始位置）
                    robot_delta = target_pos - robot_init_pos
                    current_poses[role] = (target_pos, target_quat, pico_delta, robot_delta)

            # 第二遍：计算 elbow direction 并控制机器人
            for side in ['left', 'right']:
                wrist_role = f'pico_{side}_wrist'
                arm_role = f'pico_{side}_arm'

                if wrist_role not in current_poses:
                    continue

                # 检查是否需要控制此臂
                is_left = (side == 'left')
                if args.left_only and not is_left:
                    continue
                if args.right_only and is_left:
                    continue

                wrist_pos, wrist_quat, pico_delta, robot_delta = current_poses[wrist_role]
                shoulder_pos = np.array([0.0, 0.0, 0.0])

                # 使用配置默认 zsp_para 方向（经 FK→IK 闭环验证）
                # 注意: 几何方向（compute_elbow_direction）与 IK 的 zsp_para 约定不同
                # 几何方向的 Y 分量指向重力方向，zsp_para Y 分量需要反重力方向
                elbow_direction = DEFAULT_ZSP_DIRECTION[side]

                # 打印信息
                if args.verbose:
                    # 详细模式：显示 PICO 增量 → 机器人增量
                    # PICO 坐标系: +X=右, +Y=上, +Z=朝向用户(后)
                    # 注意: PICO +Z 是"朝向用户"方向，即用户手往后缩的方向！
                    #       用户往前伸手时 PICO Z 是负值！
                    # Chest 坐标系:
                    #   Left Chest: X=前, Y=下, Z=左
                    #   Right Chest: X=前, Y=上, Z=右
                    if side == 'left':
                        y_label = "下+"
                        z_label = "左+"
                    else:
                        y_label = "上+"
                        z_label = "右+"
                    print(f"[帧 {actual_frame:5d}] {wrist_role:15s}:")
                    print(f"    PICO Δ: X={pico_delta[0]*100:+6.2f}cm(右+), Y={pico_delta[1]*100:+6.2f}cm(上+), Z={pico_delta[2]*100:+6.2f}cm(后+/朝向用户)")
                    print(f"    Chest Δ: X={robot_delta[0]*100:+6.2f}cm(前+), Y={robot_delta[1]*100:+6.2f}cm({y_label}), Z={robot_delta[2]*100:+6.2f}cm({z_label})")
                else:
                    print(f"[帧 {actual_frame:5d}] {wrist_role:15s}: "
                          f"pos=[{wrist_pos[0]:.3f}, {wrist_pos[1]:.3f}, {wrist_pos[2]:.3f}] "
                          f"elbow_dir=[{elbow_direction[0]:.2f}, {elbow_direction[1]:.2f}, {elbow_direction[2]:.2f}]")

                # 发送给机器人
                if controller:
                    try:
                        # 更新 elbow direction (zsp_para)
                        zsp_para = [
                            elbow_direction[0],
                            elbow_direction[1],
                            elbow_direction[2],
                            0, 0, 0
                        ]
                        if side == 'left':
                            controller.left_zsp_para = zsp_para
                        else:
                            controller.right_zsp_para = zsp_para

                        # 构建位姿矩阵 (位置保持米，move_to_pose_direct 内部会转换)
                        rot_mat = R.from_quat(wrist_quat).as_matrix()
                        pose_mat = np.eye(4)
                        pose_mat[:3, :3] = rot_mat
                        pose_mat[:3, 3] = wrist_pos  # 米，不是毫米

                        if side == 'left':
                            controller.move_to_pose_direct(
                                left_pose=pose_mat,
                                right_pose=None,
                                unit='matrix'
                            )
                        else:
                            controller.move_to_pose_direct(
                                left_pose=None,
                                right_pose=pose_mat,
                                unit='matrix'
                            )
                    except Exception as e:
                        print(f"  ⚠️ 控制失败: {e}")

            # 控制回放速度
            time.sleep(0.01 / args.speed)

    except KeyboardInterrupt:
        print("\n\n✓ 用户中断")

    # 统计信息
    elapsed = time.time() - start_time
    print("\n" + "=" * 70)
    print(f"回放完成")
    print(f"  总帧数: {frame_count}")
    print(f"  耗时: {elapsed:.1f} 秒")
    print(f"  平均帧率: {frame_count/elapsed:.1f} fps")
    print("=" * 70)

    # 清理
    data_source.close()
    if controller:
        print("下电并释放机器人...")
        controller.disable_and_release()
        print("✓ 机器人已安全退出")


if __name__ == '__main__':
    main()
