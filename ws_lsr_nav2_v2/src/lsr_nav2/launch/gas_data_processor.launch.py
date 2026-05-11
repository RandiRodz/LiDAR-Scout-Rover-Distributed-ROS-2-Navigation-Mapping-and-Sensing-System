from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mq2_threshold = LaunchConfiguration("mq2_threshold")
    mq3_threshold = LaunchConfiguration("mq3_threshold")
    warmup_seconds = LaunchConfiguration("warmup_seconds")
    analog_reference_voltage = LaunchConfiguration("analog_reference_voltage")
    adc_max = LaunchConfiguration("adc_max")

    return LaunchDescription([
        DeclareLaunchArgument("mq2_threshold", default_value="400"),
        DeclareLaunchArgument("mq3_threshold", default_value="400"),
        DeclareLaunchArgument("warmup_seconds", default_value="60.0"),
        DeclareLaunchArgument("analog_reference_voltage", default_value="5.0"),
        DeclareLaunchArgument("adc_max", default_value="1023.0"),
        Node(
            package="point_cloud_processing",
            executable="gas_data_processor.py",
            name="gas_data_processor",
            output="screen",
            parameters=[{
                "mq2_threshold": mq2_threshold,
                "mq3_threshold": mq3_threshold,
                "warmup_seconds": warmup_seconds,
                "analog_reference_voltage": analog_reference_voltage,
                "adc_max": adc_max,
            }],
        ),
    ])
