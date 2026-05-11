import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("point_cloud_processing")
    default_rviz_config = os.path.join(package_share, "rviz", "rover_nav2_v3_view.rviz")

    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_v3_only",
                output="screen",
                arguments=["-d", rviz_config],
                remappings=[
                    ("/tf", "/tf_nav2"),
                    ("tf", "/tf_nav2"),
                    ("/tf_static", "/tf_static_nav2"),
                    ("tf_static", "/tf_static_nav2"),
                ],
            ),
        ]
    )
