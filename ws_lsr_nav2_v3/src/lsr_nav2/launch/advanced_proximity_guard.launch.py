import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("point_cloud_processing")
    default_params_file = os.path.join(
        package_share, "config", "advanced_proximity_guard.params.yaml"
    )

    params_file = LaunchConfiguration("params_file")
    cloud_topic = LaunchConfiguration("cloud_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    use_imu_axis_hint = LaunchConfiguration("use_imu_axis_hint")
    avoid_topic = LaunchConfiguration("avoid_topic")
    filtered_topic = LaunchConfiguration("filtered_topic")
    marker_topic = LaunchConfiguration("marker_topic")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Parameter YAML for the advanced proximity guard.",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/unilidar/cloud",
                description="Live LiDAR PointCloud2 topic.",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/unilidar/imu",
                description="IMU topic used only if use_imu_axis_hint is true.",
            ),
            DeclareLaunchArgument(
                "use_imu_axis_hint",
                default_value="false",
                description="Use IMU orientation as a hint for ground-plane fitting.",
            ),
            DeclareLaunchArgument(
                "avoid_topic",
                default_value="cmd_vel_avoid",
                description="Twist topic for short-range avoidance output.",
            ),
            DeclareLaunchArgument(
                "filtered_topic",
                default_value="/filtered_obstacles",
                description="PointCloud2 topic for clustered obstacle visualization.",
            ),
            DeclareLaunchArgument(
                "marker_topic",
                default_value="/obstacle_markers",
                description="MarkerArray topic for cluster boxes and danger-zone visualization.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for the node.",
            ),
            Node(
                package="point_cloud_processing",
                executable="advanced_proximity_guard",
                name="advanced_proximity_guard",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "cloud_topic": cloud_topic,
                        "imu_topic": imu_topic,
                        "use_imu_axis_hint": use_imu_axis_hint,
                        "avoid_topic": avoid_topic,
                        "filtered_topic": filtered_topic,
                        "marker_topic": marker_topic,
                    },
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )
