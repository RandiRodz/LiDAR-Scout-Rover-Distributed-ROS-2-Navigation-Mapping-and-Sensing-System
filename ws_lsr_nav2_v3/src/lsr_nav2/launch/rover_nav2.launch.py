import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = get_package_share_directory("point_cloud_processing")
    rover_bringup_launch = os.path.join(package_share, "launch", "rover_bringup.launch.py")
    default_nav2_params = os.path.join(package_share, "config", "nav2_odom_params.yaml")
    default_nav2_obstacle_params = os.path.join(
        package_share, "config", "nav2_obstacle_cloud.params.yaml"
    )
    default_rviz_config = os.path.join(package_share, "rviz", "rover_nav2_view.rviz")
    default_pointlio_config = PathJoinSubstitution(
        [FindPackageShare("point_lio"), "config", "unilidar_l1.yaml"]
    )

    nav2_params_file = LaunchConfiguration("nav2_params_file")
    cmd_vel_out = LaunchConfiguration("cmd_vel_out")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
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
    filtered_topic = LaunchConfiguration("filtered_topic")
    marker_topic = LaunchConfiguration("marker_topic")
    nav2_obstacle_params_file = LaunchConfiguration("nav2_obstacle_params_file")
    nav2_obstacle_topic = LaunchConfiguration("nav2_obstacle_topic")
    nav2_obstacle_marker_topic = LaunchConfiguration("nav2_obstacle_marker_topic")
    use_imu_axis_hint = LaunchConfiguration("use_imu_axis_hint")
    use_lidar_odometry = LaunchConfiguration("use_lidar_odometry")
    use_pointlio_odometry = LaunchConfiguration("use_pointlio_odometry")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    external_odom_topic = LaunchConfiguration("external_odom_topic")
    external_tf_topic = LaunchConfiguration("external_tf_topic")
    pointlio_raw_odom_topic = LaunchConfiguration("pointlio_raw_odom_topic")
    pointlio_raw_odom_frame = LaunchConfiguration("pointlio_raw_odom_frame")
    pointlio_raw_base_frame = LaunchConfiguration("pointlio_raw_base_frame")
    pointlio_config = LaunchConfiguration("pointlio_config")
    kiss_icp_config = LaunchConfiguration("kiss_icp_config")
    kiss_position_covariance = LaunchConfiguration("kiss_position_covariance")
    kiss_orientation_covariance = LaunchConfiguration("kiss_orientation_covariance")
    kiss_publish_debug_clouds = LaunchConfiguration("kiss_publish_debug_clouds")
    navigator_global_frame = LaunchConfiguration("navigator_global_frame")
    use_private_nav_tf = PythonExpression(
        [
            "'true' if '",
            use_lidar_odometry,
            "' != 'true' else 'false'",
        ]
    )
    use_external_odom_bridge = PythonExpression(
        [
            "'true' if '",
            use_lidar_odometry,
            "' != 'true' and '",
            use_pointlio_odometry,
            "' != 'true' else 'false'",
        ]
    )
    nav_odom_topic = PythonExpression(
        [
            "'/odom' if '",
            use_lidar_odometry,
            "' != 'true' else '",
            odom_topic,
            "'",
        ]
    )
    nav_tf_topic = PythonExpression(
        [
            "'/tf_nav2' if '",
            use_lidar_odometry,
            "' != 'true' else '/tf'",
        ]
    )
    nav_tf_static_topic = PythonExpression(
        [
            "'/tf_static_nav2' if '",
            use_lidar_odometry,
            "' != 'true' else '/tf_static'",
        ]
    )
    nav_obstacle_cloud_topic = PythonExpression(
        [
            "'",
            restamped_cloud_topic,
            "' if '",
            use_sensor_retimestamp,
            "' == 'true' else '",
            cloud_topic,
            "'",
        ]
    )
    nav_obstacle_imu_topic = PythonExpression(
        [
            "'",
            restamped_imu_topic,
            "' if '",
            use_sensor_retimestamp,
            "' == 'true' else '",
            imu_topic,
            "'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params,
                description="Nav2 parameter YAML configured for odom-frame navigation.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_out",
                default_value="/cmd_vel",
                description="Final velocity topic consumed by the rover motor bridge.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically transition Nav2 nodes through their lifecycle.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Log level for Nav2 nodes.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with the rover Nav2 layout.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file to load.",
            ),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("lidar_frame", default_value="unilidar_lidar"),
            DeclareLaunchArgument("imu_frame", default_value="unilidar_imu"),
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
                default_value=os.path.join(
                    package_share, "config", "advanced_proximity_guard.params.yaml"
                ),
            ),
            DeclareLaunchArgument("cloud_topic", default_value="/unilidar/cloud"),
            DeclareLaunchArgument("imu_topic", default_value="/unilidar/imu"),
            DeclareLaunchArgument("use_sensor_retimestamp", default_value="true"),
            DeclareLaunchArgument(
                "restamped_cloud_topic", default_value="/unilidar/cloud_restamped"
            ),
            DeclareLaunchArgument(
                "restamped_imu_topic", default_value="/unilidar/imu_restamped"
            ),
            DeclareLaunchArgument("filtered_topic", default_value="/filtered_obstacles"),
            DeclareLaunchArgument("marker_topic", default_value="/obstacle_markers"),
            DeclareLaunchArgument(
                "nav2_obstacle_params_file",
                default_value=default_nav2_obstacle_params,
                description="PCL-based 360 obstacle cloud parameters for Nav2.",
            ),
            DeclareLaunchArgument(
                "nav2_obstacle_topic",
                default_value="/nav2_obstacles_360",
                description="Obstacle cloud topic consumed by Nav2 costmaps in V2.",
            ),
            DeclareLaunchArgument(
                "nav2_obstacle_marker_topic",
                default_value="/nav2_obstacle_markers",
                description="Marker topic for the 360 obstacle publisher.",
            ),
            DeclareLaunchArgument("use_imu_axis_hint", default_value="false"),
            DeclareLaunchArgument("use_lidar_odometry", default_value="true"),
            DeclareLaunchArgument("use_pointlio_odometry", default_value="false"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument(
                "external_odom_topic",
                default_value="/odom_corrected",
                description="External odometry topic to restamp when running Point-LIO in a separate terminal.",
            ),
            DeclareLaunchArgument(
                "external_tf_topic",
                default_value="",
                description="Optional external TF topic to restamp alongside the odometry topic.",
            ),
            DeclareLaunchArgument("pointlio_raw_odom_topic", default_value="/odom_pointlio_raw"),
            DeclareLaunchArgument("pointlio_raw_odom_frame", default_value="pointlio_odom_raw"),
            DeclareLaunchArgument("pointlio_raw_base_frame", default_value="pointlio_base_link_raw"),
            DeclareLaunchArgument(
                "pointlio_config",
                default_value=default_pointlio_config,
                description="Point-LIO config file used when launching Point-LIO as the odom source.",
            ),
            DeclareLaunchArgument(
                "kiss_icp_config",
                default_value=os.path.join(
                    package_share, "config", "kiss_icp_unilidar.yaml"
                ),
            ),
            DeclareLaunchArgument("kiss_position_covariance", default_value="0.1"),
            DeclareLaunchArgument("kiss_orientation_covariance", default_value="0.1"),
            DeclareLaunchArgument("kiss_publish_debug_clouds", default_value="false"),
            DeclareLaunchArgument(
                "navigator_global_frame",
                default_value=odom_frame,
                description="Global frame override for bt_navigator. Defaults to odom-mode behavior.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rover_bringup_launch),
                launch_arguments={
                    "base_frame": base_frame,
                    "lidar_frame": lidar_frame,
                    "imu_frame": imu_frame,
                    "lidar_x": lidar_x,
                    "lidar_y": lidar_y,
                    "lidar_z": lidar_z,
                    "lidar_roll": lidar_roll,
                    "lidar_pitch": lidar_pitch,
                    "lidar_yaw": lidar_yaw,
                    "imu_x": imu_x,
                    "imu_y": imu_y,
                    "imu_z": imu_z,
                    "imu_roll": imu_roll,
                    "imu_pitch": imu_pitch,
                    "imu_yaw": imu_yaw,
                    "params_file": params_file,
                    "cloud_topic": cloud_topic,
                    "imu_topic": imu_topic,
                    "use_sensor_retimestamp": use_sensor_retimestamp,
                    "restamped_cloud_topic": restamped_cloud_topic,
                    "restamped_imu_topic": restamped_imu_topic,
                    "avoid_topic": "cmd_vel_avoid_debug",
                    "filtered_topic": filtered_topic,
                    "marker_topic": marker_topic,
                    "use_imu_axis_hint": use_imu_axis_hint,
                    "use_lidar_odometry": use_lidar_odometry,
                    "log_level": log_level,
                    "odom_topic": nav_odom_topic,
                    "odom_frame": odom_frame,
                    "kiss_icp_config": kiss_icp_config,
                    "kiss_position_covariance": kiss_position_covariance,
                    "kiss_orientation_covariance": kiss_orientation_covariance,
                    "kiss_publish_debug_clouds": kiss_publish_debug_clouds,
                    "use_rviz": "false",
                }.items(),
            ),
            Node(
                condition=IfCondition(use_private_nav_tf),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="nav2_base_to_unilidar_lidar_tf",
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
                remappings=[("/tf_static", nav_tf_static_topic), ("tf_static", nav_tf_static_topic)],
            ),
            Node(
                condition=IfCondition(use_private_nav_tf),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="nav2_base_to_unilidar_imu_tf",
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
                remappings=[("/tf_static", nav_tf_static_topic), ("tf_static", nav_tf_static_topic)],
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            use_pointlio_odometry,
                            "' == 'true' and '",
                            use_lidar_odometry,
                            "' != 'true'",
                        ]
                    )
                ),
                package="point_lio",
                executable="pointlio_mapping",
                name="laserMapping",
                output="screen",
                parameters=[
                    pointlio_config,
                    {
                        "common.lid_topic": restamped_cloud_topic,
                        "common.imu_topic": restamped_imu_topic,
                        "odom_only": True,
                        "odom_header_frame_id": pointlio_raw_odom_frame,
                        "odom_child_frame_id": pointlio_raw_base_frame,
                        "odometry.publish_odometry_without_downsample": True,
                    },
                ],
                remappings=[
                    ("/odom_corrected", pointlio_raw_odom_topic),
                    ("/tf", "tf_pointlio_raw"),
                    ("tf", "tf_pointlio_raw"),
                ],
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            use_pointlio_odometry,
                            "' == 'true' and '",
                            use_lidar_odometry,
                            "' != 'true'",
                        ]
                    )
                ),
                package="point_cloud_processing",
                executable="odom_retimestamp_bridge",
                name="odom_retimestamp_bridge",
                output="screen",
                parameters=[
                    {
                        "input_odom_topic": pointlio_raw_odom_topic,
                        "input_tf_topic": "/tf_pointlio_raw",
                        "output_odom_topic": nav_odom_topic,
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,
                        "input_odom_frame": pointlio_raw_odom_frame,
                        "input_base_frame": pointlio_raw_base_frame,
                        "publish_tf": True,
                    }
                ],
                remappings=[("/tf", nav_tf_topic), ("tf", nav_tf_topic)],
            ),
            Node(
                condition=IfCondition(use_external_odom_bridge),
                package="point_cloud_processing",
                executable="odom_retimestamp_bridge",
                name="external_odom_retimestamp_bridge",
                output="screen",
                parameters=[
                    {
                        "input_odom_topic": external_odom_topic,
                        "input_tf_topic": external_tf_topic,
                        "output_odom_topic": nav_odom_topic,
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,
                        "input_odom_frame": odom_frame,
                        "input_base_frame": base_frame,
                        "publish_tf": True,
                    }
                ],
                remappings=[("/tf", nav_tf_topic), ("tf", nav_tf_topic)],
            ),
            Node(
                package="point_cloud_processing",
                executable="nav2_obstacle_cloud",
                name="nav2_obstacle_cloud",
                output="screen",
                parameters=[
                    nav2_obstacle_params_file,
                    {
                        "cloud_topic": nav_obstacle_cloud_topic,
                        "imu_topic": nav_obstacle_imu_topic,
                        "use_imu_axis_hint": use_imu_axis_hint,
                        "output_topic": nav2_obstacle_topic,
                        "marker_topic": nav2_obstacle_marker_topic,
                    },
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="point_cloud_processing",
                executable="cmd_vel_mux",
                name="cmd_vel_mux",
                output="screen",
                parameters=[
                    {
                        "output_topic": "cmd_vel_pre_safety",
                        "nav_topic": "cmd_vel_nav",
                        "teleop_topic": "cmd_vel_teleop",
                        "avoid_topic": "cmd_vel_avoid_debug",
                        "nav_timeout_sec": 0.5,
                        "teleop_timeout_sec": 0.5,
                        "avoid_timeout_sec": 0.45,
                        "publish_rate_hz": 30.0,
                    }
                ],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[
                    nav2_params_file,
                    {
                        "base_frame_id": base_frame,
                        "odom_frame_id": odom_frame,
                    },
                ],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("/tf", nav_tf_topic),
                    ("tf", nav_tf_topic),
                    ("/tf_static", nav_tf_static_topic),
                    ("tf_static", nav_tf_static_topic),
                ],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[
                    nav2_params_file,
                    {
                        "odom_topic": nav_odom_topic,
                    },
                ],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("/tf", nav_tf_topic),
                    ("tf", nav_tf_topic),
                    ("/tf_static", nav_tf_static_topic),
                    ("tf_static", nav_tf_static_topic),
                    ("cmd_vel", "cmd_vel_nav"),
                ],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[nav2_params_file],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("/tf", nav_tf_topic),
                    ("tf", nav_tf_topic),
                    ("/tf_static", nav_tf_static_topic),
                    ("tf_static", nav_tf_static_topic),
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_params_file],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("/tf", nav_tf_topic),
                    ("tf", nav_tf_topic),
                    ("/tf_static", nav_tf_static_topic),
                    ("tf_static", nav_tf_static_topic),
                ],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[nav2_params_file],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("/tf", nav_tf_topic),
                    ("tf", nav_tf_topic),
                    ("/tf_static", nav_tf_static_topic),
                    ("tf_static", nav_tf_static_topic),
                    ("cmd_vel", "cmd_vel_nav"),
                ],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    nav2_params_file,
                    {
                        "global_frame": navigator_global_frame,
                        "robot_base_frame": base_frame,
                        "odom_topic": nav_odom_topic,
                    },
                ],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("/tf", nav_tf_topic),
                    ("tf", nav_tf_topic),
                    ("/tf_static", nav_tf_static_topic),
                    ("tf_static", nav_tf_static_topic),
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    nav2_params_file,
                    {"autostart": autostart},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                remappings=[
                    ("/tf", nav_tf_topic),
                    ("tf", nav_tf_topic),
                    ("/tf_static", nav_tf_static_topic),
                    ("tf_static", nav_tf_static_topic),
                ],
            ),
        ]
    )
