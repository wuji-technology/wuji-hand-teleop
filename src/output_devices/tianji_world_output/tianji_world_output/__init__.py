"""
tianji_world_output - Tianji Robotic Arm Output Device Package

Public interfaces:
  config_loader: TianjiConfig, get_config()  -- Unified config loading (tianji_robot.yaml)
  transform_utils: Coordinate transform shared library -- All coordinate/rotation transform functions (single authoritative implementation)
  cartesian_controller: CartesianController -- Cartesian space controller
  fx_kine: Marvin_Kine -- Kinematics FK/IK
  fx_robot: Marvin_Robot -- Low-level robot control

Usage example:
  from tianji_world_output.config_loader import get_config
  from tianji_world_output.transform_utils import (
      transform_world_to_chest,
      apply_world_rotation_to_chest_pose,
      transform_pico_rotation_to_world,
      elbow_direction_from_angles,
      get_pico_to_robot,
  )
  from tianji_world_output.cartesian_controller import CartesianController

Coordinate frame conventions (ROS REP 103):
  World: +X=forward, +Y=left, +Z=up
  Left Chest: World rotated +90deg around X axis  (+Y=down, +Z=left)
  Right Chest: World rotated -90deg around X axis (+Y=up, +Z=right)

Positive rotation direction (right-hand rule, counterclockwise when viewed from positive axis end toward origin):
  Around +X: Roll right (top of wrist tilts right)
  Around +Y: Pitch down (fingers press downward)
  Around +Z: Yaw left (fingers point leftward)
"""
