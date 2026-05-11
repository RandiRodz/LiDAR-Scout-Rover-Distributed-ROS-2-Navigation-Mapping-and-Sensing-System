#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32


class GasDataProcessor(Node):
    def __init__(self) -> None:
        super().__init__("gas_data_processor")

        self.declare_parameter("analog_reference_voltage", 5.0)
        self.declare_parameter("adc_max", 1023.0)
        self.declare_parameter("mq2_threshold", 400)
        self.declare_parameter("mq3_threshold", 400)
        self.declare_parameter("warmup_seconds", 60.0)

        self.analog_reference_voltage = (
            self.get_parameter("analog_reference_voltage").get_parameter_value().double_value
        )
        self.adc_max = self.get_parameter("adc_max").get_parameter_value().double_value
        self.mq2_threshold = self.get_parameter("mq2_threshold").get_parameter_value().integer_value
        self.mq3_threshold = self.get_parameter("mq3_threshold").get_parameter_value().integer_value
        self.warmup_seconds = self.get_parameter("warmup_seconds").get_parameter_value().double_value

        self.start_time_ns = self.get_clock().now().nanoseconds
        self.latest_mq2_raw = None
        self.latest_mq3_raw = None

        self.create_subscription(Int32, "/gas/raw_mq2", self.mq2_callback, 10)
        self.create_subscription(Int32, "/gas/raw_mq3", self.mq3_callback, 10)

        self.gas_ready_pub = self.create_publisher(Bool, "/gas/ready", 10)

        self.mq2_voltage_pub = self.create_publisher(Float32, "/gas/mq2/voltage", 10)
        self.mq2_alert_pub = self.create_publisher(Bool, "/gas/mq2/alert", 10)

        self.mq3_voltage_pub = self.create_publisher(Float32, "/gas/mq3/voltage", 10)
        self.mq3_alert_pub = self.create_publisher(Bool, "/gas/mq3/alert", 10)

        self.get_logger().info(
            "Gas processor ready. Waiting for /gas/raw_mq2 and /gas/raw_mq3 from the Pi."
        )

    def ready(self) -> bool:
        elapsed_ns = self.get_clock().now().nanoseconds - self.start_time_ns
        return (elapsed_ns / 1e9) >= self.warmup_seconds

    def raw_to_voltage(self, raw_value: int) -> float:
        return float(raw_value) * self.analog_reference_voltage / self.adc_max

    def mq2_callback(self, msg: Int32) -> None:
        self.latest_mq2_raw = msg.data
        self.publish_state()

    def mq3_callback(self, msg: Int32) -> None:
        self.latest_mq3_raw = msg.data
        self.publish_state()

    def publish_state(self) -> None:
        ready = self.ready()
        self.gas_ready_pub.publish(Bool(data=ready))

        if self.latest_mq2_raw is not None:
            mq2_voltage = self.raw_to_voltage(self.latest_mq2_raw)
            mq2_alert = ready and (self.latest_mq2_raw >= self.mq2_threshold)
            self.mq2_voltage_pub.publish(Float32(data=mq2_voltage))
            self.mq2_alert_pub.publish(Bool(data=mq2_alert))

        if self.latest_mq3_raw is not None:
            mq3_voltage = self.raw_to_voltage(self.latest_mq3_raw)
            mq3_alert = ready and (self.latest_mq3_raw >= self.mq3_threshold)
            self.mq3_voltage_pub.publish(Float32(data=mq3_voltage))
            self.mq3_alert_pub.publish(Bool(data=mq3_alert))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GasDataProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
