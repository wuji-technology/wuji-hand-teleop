"""Launch file to visualize AVP hand tracking data in RViz."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for AVP visualization."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('avp_input')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'avp_visualization.rviz')

    # TF broadcaster node (converts Float32MultiArray to TF)
    tf_broadcaster_node = Node(
        package='common_input',
        executable='tf_broadcaster',
        name='tf_broadcaster',
        output='screen',
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )
    
    return LaunchDescription([
        tf_broadcaster_node,
        rviz_node,
    ])

