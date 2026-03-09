"""
tianji_world_output - 天机机械臂输出设备包

公共接口:
  config_loader: TianjiConfig, get_config()  -- 统一配置加载 (tianji_robot.yaml)
  transform_utils: 坐标变换共享库 -- 所有坐标/旋转变换函数 (唯一权威实现)
  cartesian_controller: CartesianController -- 笛卡尔空间控制器
  fx_kine: Marvin_Kine -- 运动学 FK/IK
  fx_robot: Marvin_Robot -- 机器人底层控制

使用示例:
  from tianji_world_output.config_loader import get_config
  from tianji_world_output.transform_utils import (
      transform_world_to_chest,
      apply_world_rotation_to_chest_pose,
      transform_pico_rotation_to_world,
      elbow_direction_from_angles,
      get_pico_to_robot,
  )
  from tianji_world_output.cartesian_controller import CartesianController

坐标系约定 (ROS REP 103):
  World: +X=前, +Y=左, +Z=上
  Left Chest: World 绕 X 轴 +90deg  (+Y=下, +Z=左)
  Right Chest: World 绕 X 轴 -90deg (+Y=上, +Z=右)

旋转正方向 (右手定则，从轴正端向原点看逆时针为正):
  绕 +X: Roll right (手腕顶部向右倾)
  绕 +Y: Pitch down (手指向下压)
  绕 +Z: Yaw left (手指向左偏)
"""
