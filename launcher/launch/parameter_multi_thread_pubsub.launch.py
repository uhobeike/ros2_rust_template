import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node


def generate_launch_description():
    ros2_cpp_template_dir = get_package_share_directory('launcher')
    params_file = os.path.join(ros2_cpp_template_dir, 'config', 'template.param.yaml')
    
    launch_node = GroupAction([
        Node(
            name='parameter_multi_thread_pub_node',
            package='parameter_multi_thread_pub',
            executable='parameter_multi_thread_pub',
            output='screen'),
        Node(
            name='parameter_multi_thread_sub_node',
            package='parameter_multi_thread_sub',
            executable='parameter_multi_thread_sub',
            output='screen')
    ])

    ld = LaunchDescription()
    ld.add_action(launch_node)

    return ld
