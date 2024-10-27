import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    is_sim = LaunchConfiguration(
        "is_sim"
    )


    task_server_node = Node(
        package = "arduinobot_remote",
        executable="task_server_node",
        parameters = [{"use_sim_time": is_sim}]
    )

    return LaunchDescription([
        is_sim_arg,
        task_server_node
    ])