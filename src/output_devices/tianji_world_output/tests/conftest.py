"""
tianji_world_output unit tests — pytest shared configuration
==============================================

How to run:
    # Enter the package root directory
    cd ~/Desktop/wuji-hand-teleop/src/output_devices/tianji_world_output

    # Run all tests (66 total)
    python3 -m pytest tests/ -v

    # Run only coordinate transform tests
    python3 -m pytest tests/test_transform_utils.py -v

    # Run only config loader tests
    python3 -m pytest tests/test_config_loader.py -v

    # Filter by keyword (e.g. only elbow-related tests)
    python3 -m pytest tests/ -v -k elbow

    # Stop at the first failure
    python3 -m pytest tests/ -v -x

Dependencies:
    pip install pytest numpy scipy pyyaml

Test coverage:
    test_transform_utils.py  — Coordinate transform mathematical correctness
        - World <-> Chest position transform (left/right)
        - Rotation matrix orthogonality, axis mapping
        - TF quaternion conjugate
        - 4-step rotation algorithm (apply_world_rotation_to_chest_pose)
        - PICO -> World axis-angle transform
        - Elbow direction vector (elbow_direction_from_angles)
        - pico_to_robot matrix properties

    test_config_loader.py    — YAML config loading and data validation
        - TianjiConfig.load() / get_config() singleton
        - Scalar field types (robot_ip, zsp_type, dgr)
        - Dict field shapes (init_joints=7, init_pos=3, init_rot=3x3, init_quat=4)
        - Quaternion unit norm check
        - Rotation matrix orthogonality (det=+1)
        - Left-right symmetry (init_pos X equal, Y opposite, Z equal)
        - Helper methods (get_world_to_chest_rotation, get_default_zsp_direction)

Note:
    - Tests do not depend on a ROS2 environment; just use python3 -m pytest directly
    - Config file is loaded from source-relative path via use_ros=False (config/tianji_robot.yaml)
    - Re-run tests after modifying tianji_robot.yaml or transform_utils.py
"""
