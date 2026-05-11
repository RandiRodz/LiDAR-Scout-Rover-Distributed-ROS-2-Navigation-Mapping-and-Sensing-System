import os
from math import pi

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    package_share = get_package_share_directory("point_cloud_processing")
    base_nav_launch = os.path.join(package_share, "launch", "rover_nav2.launch.py")
    default_nav2_map_params = os.path.join(package_share, "config", "nav2_v3_params.yaml")
    default_slam_params = os.path.join(package_share, "config", "slam_toolbox_v3_online_async.yaml")
    default_rviz_config = os.path.join(package_share, "rviz", "rover_nav2_v3_view.rviz")
    default_pointlio_config = os.path.join(
        get_package_share_directory("point_lio"), "config", "unilidar_l1.yaml"
    )

    nav2_params_file = LaunchConfiguration("nav2_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
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

    pointlio_config = LaunchConfiguration("pointlio_config")
    pointlio_native_odom_topic = LaunchConfiguration("pointlio_native_odom_topic")
    pointlio_native_tf_topic = LaunchConfiguration("pointlio_native_tf_topic")
    pointlio_native_odom_frame = LaunchConfiguration("pointlio_native_odom_frame")
    pointlio_native_base_frame = LaunchConfiguration("pointlio_native_base_frame")
    scan_cloud_topic = LaunchConfiguration("scan_cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    scan_target_frame = LaunchConfiguration("scan_target_frame")
    scan_min_height = LaunchConfiguration("scan_min_height")
    scan_max_height = LaunchConfiguration("scan_max_height")
    scan_range_min = LaunchConfiguration("scan_range_min")
    scan_range_max = LaunchConfiguration("scan_range_max")
    scan_time = LaunchConfiguration("scan_time")
    scan_queue_size = LaunchConfiguration("scan_queue_size")
    transform_tolerance = LaunchConfiguration("transform_tolerance")

    nav_tf_topic = "/tf_nav2"
    nav_tf_static_topic = "/tf_static_nav2"

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_nav_launch),
        launch_arguments={
            "nav2_params_file": nav2_params_file,
            "autostart": autostart,
            "log_level": log_level,
            "use_rviz": "false",
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
            "filtered_topic": filtered_topic,
            "marker_topic": marker_topic,
            "nav2_obstacle_params_file": nav2_obstacle_params_file,
            "nav2_obstacle_topic": nav2_obstacle_topic,
            "nav2_obstacle_marker_topic": nav2_obstacle_marker_topic,
            "use_imu_axis_hint": use_imu_axis_hint,
            "use_lidar_odometry": "false",
            "use_pointlio_odometry": "false",
            "navigator_global_frame": "map",
        }.items(),
    )

    pointlio_mapping = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="laserMapping_v3",
        output="screen",
        parameters=[
            pointlio_config,
            {
                "common.lid_topic": cloud_topic,
                "common.imu_topic": imu_topic,
                "use_imu_as_input": False,
                "prop_at_freq_of_imu": True,
                "check_satu": True,
                "init_map_size": 10,
                "point_filter_num": 1,
                "space_down_sample": True,
                "filter_size_surf": 0.1,
                "filter_size_map": 0.1,
                "cube_side_length": 1000.0,
                "runtime_pos_log_enable": False,
                "odom_only": False,
                "odom_header_frame_id": pointlio_native_odom_frame,
                "odom_child_frame_id": pointlio_native_base_frame,
                "odometry.publish_odometry_without_downsample": True,
            },
        ],
        remappings=[
            ("/aft_mapped_to_init", pointlio_native_odom_topic),
            ("/tf", pointlio_native_tf_topic),
            ("tf", pointlio_native_tf_topic),
        ],
    )

    pointlio_odom_bridge = Node(
        package="point_cloud_processing",
        executable="odom_retimestamp_bridge",
        name="pointlio_native_odom_bridge",
        output="screen",
        parameters=[
            {
                "input_odom_topic": pointlio_native_odom_topic,
                "input_tf_topic": pointlio_native_tf_topic,
                "output_odom_topic": "/odom",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "input_odom_frame": pointlio_native_odom_frame,
                "input_base_frame": pointlio_native_base_frame,
                "publish_tf": True,
            }
        ],
        remappings=[
            ("/tf", nav_tf_topic),
            ("tf", nav_tf_topic),
        ],
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="v3_pointcloud_to_laserscan",
        output="screen",
        parameters=[
            {
                "target_frame": scan_target_frame,
                "transform_tolerance": transform_tolerance,
                "min_height": scan_min_height,
                "max_height": scan_max_height,
                "angle_min": -pi,
                "angle_max": pi,
                "angle_increment": 0.008726646259971648,
                "scan_time": scan_time,
                "range_min": scan_range_min,
                "range_max": scan_range_max,
                "use_inf": True,
                "inf_epsilon": 1.0,
                "queue_size": scan_queue_size,
            }
        ],
        remappings=[
            ("cloud_in", scan_cloud_topic),
            ("scan", scan_topic),
            ("/tf", nav_tf_topic),
            ("tf", nav_tf_topic),
            ("/tf_static", nav_tf_static_topic),
            ("tf_static", nav_tf_static_topic),
        ],
    )

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": False}],
        remappings=[
            ("/scan", scan_topic),
            ("scan", scan_topic),
            ("/tf", nav_tf_topic),
            ("tf", nav_tf_topic),
            ("/tf_static", nav_tf_static_topic),
            ("tf_static", nav_tf_static_topic),
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] slam_toolbox is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2_v3",
        output="screen",
        arguments=["-d", rviz_config],
        remappings=[
            ("/tf", nav_tf_topic),
            ("tf", nav_tf_topic),
            ("/tf_static", nav_tf_static_topic),
            ("tf_static", nav_tf_static_topic),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_map_params),
            DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
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
                default_value=os.path.join(package_share, "config", "advanced_proximity_guard.params.yaml"),
            ),
            DeclareLaunchArgument("cloud_topic", default_value="/unilidar/cloud"),
            DeclareLaunchArgument("imu_topic", default_value="/unilidar/imu"),
            DeclareLaunchArgument("use_sensor_retimestamp", default_value="true"),
            DeclareLaunchArgument("restamped_cloud_topic", default_value="/unilidar/cloud_restamped"),
            DeclareLaunchArgument("restamped_imu_topic", default_value="/unilidar/imu_restamped"),
            DeclareLaunchArgument("filtered_topic", default_value="/filtered_obstacles"),
            DeclareLaunchArgument("marker_topic", default_value="/obstacle_markers"),
            DeclareLaunchArgument(
                "nav2_obstacle_params_file",
                default_value=os.path.join(package_share, "config", "nav2_obstacle_cloud.params.yaml"),
            ),
            DeclareLaunchArgument("nav2_obstacle_topic", default_value="/nav2_obstacles_360"),
            DeclareLaunchArgument("nav2_obstacle_marker_topic", default_value="/nav2_obstacle_markers"),
            DeclareLaunchArgument("use_imu_axis_hint", default_value="false"),
            DeclareLaunchArgument("pointlio_config", default_value=default_pointlio_config),
            DeclareLaunchArgument("pointlio_native_odom_topic", default_value="/aft_mapped_to_init"),
            DeclareLaunchArgument("pointlio_native_tf_topic", default_value="/tf_pointlio_native"),
            DeclareLaunchArgument("pointlio_native_odom_frame", default_value="camera_init"),
            DeclareLaunchArgument("pointlio_native_base_frame", default_value="aft_mapped"),
            DeclareLaunchArgument("scan_cloud_topic", default_value="/unilidar/cloud"),
            DeclareLaunchArgument("scan_topic", default_value="/scan_nav2"),
            DeclareLaunchArgument("scan_target_frame", default_value="base_link"),
            DeclareLaunchArgument("scan_min_height", default_value="-0.05"),
            DeclareLaunchArgument("scan_max_height", default_value="0.35"),
            DeclareLaunchArgument("scan_range_min", default_value="0.30"),
            DeclareLaunchArgument("scan_range_max", default_value="20.0"),
            DeclareLaunchArgument("scan_time", default_value="0.1"),
            DeclareLaunchArgument("scan_queue_size", default_value="32"),
            DeclareLaunchArgument("transform_tolerance", default_value="0.2"),
            pointlio_mapping,
            pointlio_odom_bridge,
            base_launch,
            pointcloud_to_laserscan,
            slam_toolbox_node,
            configure_event,
            activate_event,
            rviz_node,
        ]
    )
