#!/usr/bin/env python3

import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from serial import Serial, SerialException
from std_msgs.msg import Int32


class MegaBridge(Node):
    def __init__(self) -> None:
        super().__init__("mega_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("poll_period", 0.02)
        self.declare_parameter("log_motor_commands", False)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        poll_period = self.get_parameter("poll_period").get_parameter_value().double_value
        self.log_motor_commands = self.get_parameter("log_motor_commands").get_parameter_value().bool_value

        self.serial_lock = threading.Lock()
        self._last_parse_error = None

        try:
            self.ser = Serial(self.port, self.baud_rate, timeout=0.01)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"Connected to Arduino Mega on {self.port}")
        except Exception as exc:
            self.get_logger().error(f"FATAL: Could not open {self.port}: {exc}")
            raise

        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.mq2_raw_pub = self.create_publisher(Int32, "/gas/raw_mq2", 10)
        self.mq3_raw_pub = self.create_publisher(Int32, "/gas/raw_mq3", 10)
        self.serial_timer = self.create_timer(poll_period, self.poll_serial)

    def send_serial_line(self, line: str) -> None:
        payload = f"{line}\n".encode("utf-8")
        try:
            with self.serial_lock:
                self.ser.write(payload)
        except SerialException as exc:
            self.get_logger().error(f"Serial write failed: {exc}")

    def cmd_vel_callback(self, msg: Twist) -> None:
        command = f"v {msg.linear.x:.2f} {msg.angular.z:.2f}"
        self.send_serial_line(command)

        if self.log_motor_commands:
            self.get_logger().info(f"Motor relay: {command}")

    def poll_serial(self) -> None:
        while True:
            try:
                with self.serial_lock:
                    if self.ser.in_waiting <= 0:
                        break
                    raw = self.ser.readline()
            except SerialException as exc:
                self.get_logger().error(f"Serial read failed: {exc}")
                break

            if not raw:
                break

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line.startswith("g "):
                self.handle_gas_line(line)
            else:
                self.get_logger().debug(f"Mega: {line}")

    def handle_gas_line(self, line: str) -> None:
        parts = line.split()
        if len(parts) != 3:
            self._warn_parse(line)
            return

        try:
            _, mq2_raw_str, mq3_raw_str = parts
            mq2_raw = int(mq2_raw_str)
            mq3_raw = int(mq3_raw_str)
        except ValueError:
            self._warn_parse(line)
            return

        self.mq2_raw_pub.publish(Int32(data=mq2_raw))
        self.mq3_raw_pub.publish(Int32(data=mq3_raw))
        self._last_parse_error = None

    def _warn_parse(self, line: str) -> None:
        if line == self._last_parse_error:
            return
        self._last_parse_error = line
        self.get_logger().warn(f"Unexpected gas line: {line}")

    def shutdown(self) -> None:
        try:
            self.send_serial_line("v 0.00 0.00")
        finally:
            try:
                with self.serial_lock:
                    self.ser.close()
            except Exception:
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MegaBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping motors and shutting down mega_bridge")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
