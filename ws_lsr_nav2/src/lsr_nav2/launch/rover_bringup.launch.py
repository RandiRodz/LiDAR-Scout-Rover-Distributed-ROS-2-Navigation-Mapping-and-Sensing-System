import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("point_cloud_processing")
    guard_launch = os.path.join(package_share, "launch", "advanced_proximity_guard.launch.py")
    default_params_file = os.path.join(
        package_share, "config", "advanced_proximity_guard.params.yaml"
    )
    default_kiss_icp_config = os.path.join(
        package_share, "config", "kiss_icp_unilidar.yaml"
    )
    default_rviz_config = os.path.join(
        package_share, "rviz", "rover_nav2_view.rviz"
    )

    base_frame = LaunchConfiguration("base_frame")
    lidar_frame = LaunchConfiguration("lidar_frame")
    imu_frame = LaunchConfiguration("imu_frame")

    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_roll = LaunchConfiguration("lidar_roll")
    lidar_pitch = LaunchConfiguration("lidar_pitch")
    lidar_yaw = LaunchConfiguration("lidar_yaw")

    imu_x = LaunchConfiguration("imu_x")
    imu_y = LaunchConfiguration("imu_y")
    imu_z = LaunchConfiguration("imu_z")
    imu_roll = LaunchConfiguration("imu_roll")
    imu_pitch = LaunchConfiguration("imu_pitch")
    imu_yaw = LaunchConfiguration("imu_yaw")

    params_file = LaunchConfiguration("params_file")
    cloud_topic = LaunchConfiguration("cloud_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    use_sensor_retimestamp = LaunchConfiguration("use_sensor_retimestamp")
    restamped_cloud_topic = LaunchConfiguration("restamped_cloud_topic")
    restamped_imu_topic = LaunchConfiguration("restamped_imu_topic")
    avoid_topic = LaunchConfiguration("avoid_topic")
    filtered_topic = LaunchConfiguration("filtered_topic")
    marker_topic = LaunchConfiguration("marker_topic")
    use_imu_axis_hint = LaunchConfiguration("use_imu_axis_hint")
    log_level = LaunchConfiguration("log_level")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_lidar_odometry = LaunchConfiguration("use_lidar_odometry")
    raw_odom_topic = LaunchConfiguration("raw_odom_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    kiss_icp_config = LaunchConfiguration("kiss_icp_config")
    kiss_position_covariance = LaunchConfiguration("kiss_position_covariance")
    kiss_orientation_covariance = LaunchConfiguration("kiss_orientation_covariance")
    kiss_publish_debug_clouds = LaunchConfiguration("kiss_publish_debug_clouds")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Robot base frame used by navigation and TF.",
            ),
            DeclareLaunchArgument(
                "lidar_frame",
                default_value="unilidar_lidar",
                description="LiDAR frame id.",
            ),
            DeclareLaunchArgument(
                "imu_frame",
                default_value="unilidar_imu",
                description="IMU frame id.",
            ),
            DeclareLaunchArgument("lidar_x", default_value="0.0"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="0.124"),
            DeclareLaunchArgument("lidar_roll", default_value="0.0"),
            DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
            DeclareLaunchArgument("imu_x", default_value="0.0"),
            DeclareLaunchArgument("imu_y", default_value="0.0"),
            DeclareLaunchArgument("imu_z", default_value="0.124"),
            DeclareLaunchArgument("imu_roll", default_value="0.0"),
            DeclareLaunchArgument("imu_pitch", default_value="0.0"),
            DeclareLaunchArgument("imu_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Parameter YAML for advanced proximity guard.",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/unilidar/cloud",
                description="Live LiDAR PointCloud2 topic.",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/unilidar/imu",
                description="IMU topic used by the guard node.",
            ),
            DeclareLaunchArgument(
                "use_sensor_retimestamp",
                default_value="true",
                description="Restamp LiDAR and IMU headers to current ROS time before odometry and guarding.",
            ),
            DeclareLaunchArgument(
                "restamped_cloud_topic",
                default_value="/unilidar/cloud_restamped",
                description="Republished PointCloud2 topic with fresh timestamps.",
            ),
            DeclareLaunchArgument(
                "restamped_imu_topic",
                default_value="/unilidar/imu_restamped",
                description="Republished IMU topic with fresh timestamps.",
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
                description="MarkerArray topic for obstacle boxes and the danger zone.",
            ),
            DeclareLaunchArgument(
                "use_imu_axis_hint",
                default_value="false",
                description="Use IMU orientation as a hint for ground-plane fitting.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for the guard node.",
            ),
            DeclareLaunchArgument(
                "use_lidar_odometry",
                default_value="true",
                description="Launch KISS-ICP LiDAR odometry and publish odom -> base_link TF.",
            ),
            DeclareLaunchArgument(
                "raw_odom_topic",
                default_value="/odom_raw",
                description="Raw odometry topic emitted directly by KISS-ICP before stabilization.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="Odometry topic expected by RViz/Nav2.",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="odom",
                description="Odometry frame published by LiDAR odometry.",
            ),
            DeclareLaunchArgument(
                "kiss_icp_config",
                default_value=default_kiss_icp_config,
                description="KISS-ICP YAML config tuned for the Unitree LiDAR.",
            ),
            DeclareLaunchArgument(
                "kiss_position_covariance",
                default_value="0.1",
                description="Position covariance written into the LiDAR odometry message.",
            ),
            DeclareLaunchArgument(
                "kiss_orientation_covariance",
                default_value="0.1",
                description="Orientation covariance written into the LiDAR odometry message.",
            ),
            DeclareLaunchArgument(
                "kiss_publish_debug_clouds",
                default_value="false",
                description="Publish KISS-ICP debug clouds such as local map and keypoints.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with a Nav2-ready layout.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file to load.",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_unilidar_lidar_tf",
                output="screen",
                arguments=[
                    "--x",
                    lidar_x,
                    "--y",
                    lidar_y,
                    "--z",
                    lidar_z,
                    "--roll",
                    lidar_roll,
                    "--pitch",
                    lidar_pitch,
                    "--yaw",
                    lidar_yaw,
                    "--frame-id",
                    base_frame,
                    "--child-frame-id",
                    lidar_frame,
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_unilidar_imu_tf",
                output="screen",
                arguments=[
                    "--x",
                    imu_x,
                    "--y",
                    imu_y,
                    "--z",
                    imu_z,
                    "--roll",
                    imu_roll,
                    "--pitch",
                    imu_pitch,
                    "--yaw",
                    imu_yaw,
                    "--frame-id",
                    base_frame,
                    "--child-frame-id",
                    imu_frame,
                ],
            ),
            Node(
                condition=IfCondition(use_sensor_retimestamp),
                package="point_cloud_processing",
                executable="lidar_retimestamp_bridge",
                name="lidar_retimestamp_bridge",
                output="screen",
                parameters=[
                    {
                        "input_cloud_topic": cloud_topic,
                        "input_imu_topic": imu_topic,
                        "output_cloud_topic": restamped_cloud_topic,
                        "output_imu_topic": restamped_imu_topic,
                    }
                ],
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(
                        ["'", use_sensor_retimestamp, "' == 'true' and '", use_lidar_odometry, "' == 'true'"]
                    )
                ),
                package="kiss_icp",
                executable="kiss_icp_node",
                name="kiss_icp_node",
                output="screen",
                remappings=[
                    ("pointcloud_topic", restamped_cloud_topic),
                    ("kiss/odometry", raw_odom_topic),
                ],
                parameters=[
                    {
                        "base_frame": base_frame,
                        "lidar_odom_frame": odom_frame,
                        "publish_odom_tf": False,
                        "invert_odom_tf": False,
                        "publish_debug_clouds": ParameterValue(
                            kiss_publish_debug_clouds, value_type=bool
                        ),
                        "use_sim_time": False,
                        "position_covariance": ParameterValue(
                            kiss_position_covariance, value_type=float
                        ),
                        "orientation_covariance": ParameterValue(
                            kiss_orientation_covariance, value_type=float
                        ),
                    },
                    kiss_icp_config,
                ],
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(
                        ["'", use_sensor_retimestamp, "' != 'true' and '", use_lidar_odometry, "' == 'true'"]
                    )
                ),
                package="kiss_icp",
                executable="kiss_icp_node",
                name="kiss_icp_node",
                output="screen",
                remappings=[
                    ("pointcloud_topic", cloud_topic),
                    ("kiss/odometry", raw_odom_topic),
                ],
                parameters=[
                    {
                        "base_frame": base_frame,
                        "lidar_odom_frame": odom_frame,
                        "publish_odom_tf": False,
                        "invert_odom_tf": False,
                        "publish_debug_clouds": ParameterValue(
                            kiss_publish_debug_clouds, value_type=bool
                        ),
                        "use_sim_time": False,
                        "position_covariance": ParameterValue(
                            kiss_position_covariance, value_type=float
                        ),
                        "orientation_covariance": ParameterValue(
                            kiss_orientation_covariance, value_type=float
                        ),
                    },
                    kiss_icp_config,
                ],
            ),
            Node(
                condition=IfCondition(use_lidar_odometry),
                package="point_cloud_processing",
                executable="odom_stabilizer",
                name="odom_stabilizer",
                output="screen",
                parameters=[
                    {
                        "input_odom_topic": raw_odom_topic,
                        "output_odom_topic": odom_topic,
                        "cmd_vel_topic": "/cmd_vel",
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,
                        "alpha_moving": 0.20,
                        "alpha_stationary": 0.04,
                        "moving_linear_threshold": 0.10,
                        "moving_angular_threshold": 0.25,
                        "hold_translation_tolerance": 0.14,
                        "hold_yaw_tolerance": 0.12,
                        "max_translation_rate_moving": 0.35,
                        "max_translation_rate_stationary": 0.08,
                        "max_yaw_rate_moving": 0.90,
                        "max_yaw_rate_stationary": 0.18,
                        "max_translation_margin": 0.01,
                        "max_yaw_margin": 0.02,
                        "reject_translation_jump_moving": 0.35,
                        "reject_translation_jump_stationary": 0.14,
                        "reject_yaw_jump_moving": 0.75,
                        "reject_yaw_jump_stationary": 0.25,
                        "flatten_z": True,
                        "publish_tf": True,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(guard_launch),
                condition=IfCondition(use_sensor_retimestamp),
                launch_arguments={
                    "params_file": params_file,
                    "cloud_topic": restamped_cloud_topic,
                    "imu_topic": restamped_imu_topic,
                    "avoid_topic": avoid_topic,
                    "filtered_topic": filtered_topic,
                    "marker_topic": marker_topic,
                    "use_imu_axis_hint": use_imu_axis_hint,
                    "log_level": log_level,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(guard_launch),
                condition=UnlessCondition(use_sensor_retimestamp),
                launch_arguments={
                    "params_file": params_file,
                    "cloud_topic": cloud_topic,
                    "imu_topic": imu_topic,
                    "avoid_topic": avoid_topic,
                    "filtered_topic": filtered_topic,
                    "marker_topic": marker_topic,
                    "use_imu_axis_hint": use_imu_axis_hint,
                    "log_level": log_level,
                }.items(),
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
