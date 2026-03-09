"""
tianji_world_output 单元测试 — pytest 公共配置
==============================================

运行方式:
    # 进入包根目录
    cd ~/Desktop/wuji-teleop-ros2-private/src/output_devices/tianji_world_output

    # 运行全部测试 (66 个)
    python3 -m pytest tests/ -v

    # 只运行坐标变换测试
    python3 -m pytest tests/test_transform_utils.py -v

    # 只运行配置加载测试
    python3 -m pytest tests/test_config_loader.py -v

    # 按关键字筛选 (例如只跑 elbow 相关)
    python3 -m pytest tests/ -v -k elbow

    # 遇到第一个失败就停止
    python3 -m pytest tests/ -v -x

依赖:
    pip install pytest numpy scipy pyyaml

测试覆盖范围:
    test_transform_utils.py  — 坐标变换数学正确性
        - World <-> Chest 位置变换 (左/右)
        - 旋转矩阵正交性、轴映射
        - TF 四元数共轭
        - 4步旋转算法 (apply_world_rotation_to_chest_pose)
        - PICO -> World 轴角变换
        - 沉肘方向向量 (elbow_direction_from_angles)
        - pico_to_robot 矩阵属性

    test_config_loader.py    — YAML 配置加载与数据校验
        - TianjiConfig.load() / get_config() 单例
        - 标量字段类型 (robot_ip, zsp_type, dgr)
        - Dict 字段 shape (init_joints=7, init_pos=3, init_rot=3x3, init_quat=4)
        - 四元数单位化检查
        - 旋转矩阵正交性 (det=+1)
        - 左右对称性 (init_pos X相等, Y相反, Z相等)
        - 辅助方法 (get_world_to_chest_rotation, get_default_zsp_direction)

注意:
    - 测试不依赖 ROS2 环境，直接用 python3 -m pytest 即可
    - 配置文件通过 use_ros=False 从源码相对路径加载 (config/tianji_robot.yaml)
    - 修改 tianji_robot.yaml 或 transform_utils.py 后应重新运行测试
"""
