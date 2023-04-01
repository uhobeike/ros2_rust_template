from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node


def generate_launch_description():
    launch_node = GroupAction([
        Node(
            name='multi_thread_pub_node',
            package='multi_thread_pub',
            executable='multi_thread_pub',
            output='screen'),
        Node(
            name='multi_thread_sub_node',
            package='multi_thread_sub',
            executable='multi_thread_sub',
            output='screen')
    ])

    ld = LaunchDescription()
    ld.add_action(launch_node)

    return ld
