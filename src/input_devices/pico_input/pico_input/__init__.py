"""
pico_input - PICO VR 遥操作输入设备

公共接口:
  pico_input_node: PicoInputNode -- ROS2 节点 (增量控制)
  data_source: DataSource, LiveDataSource, RecordedDataSource -- 数据源抽象
  xrobotoolkit_client: XRoboToolkitClient -- PICO SDK 封装

坐标变换: 使用 tianji_world_output.transform_utils (共享库，唯一权威实现)
配置加载: 使用 tianji_world_output.config_loader (统一配置 tianji_robot.yaml)

数据流:
  PICO Tracker --> pico_input_node --> /left_arm_target_pose, /right_arm_target_pose
                                   --> /left_arm_elbow_direction, /right_arm_elbow_direction
                                   --> TF: world --> head, pico_*_wrist, pico_*_arm
"""
