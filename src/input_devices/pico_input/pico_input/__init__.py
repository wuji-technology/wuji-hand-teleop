"""
pico_input - PICO VR teleoperation input device

Public API:
  pico_input_node: PicoInputNode -- ROS2 node (incremental control)
  data_source: DataSource, LiveDataSource, RecordedDataSource -- data source abstraction
  xrobotoolkit_client: XRoboToolkitClient -- PICO SDK wrapper

Coordinate transforms: Uses tianji_world_output.transform_utils (shared library, single authoritative implementation)
Configuration loading: Uses tianji_world_output.config_loader (unified config tianji_robot.yaml)

Data flow:
  PICO Tracker --> pico_input_node --> /left_arm_target_pose, /right_arm_target_pose
                                   --> /left_arm_elbow_direction, /right_arm_elbow_direction
                                   --> TF: world --> head, pico_*_wrist, pico_*_arm
"""
