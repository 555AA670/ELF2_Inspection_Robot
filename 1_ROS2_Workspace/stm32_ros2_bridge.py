#!/usr/bin/env python3
"""
ROS2 <-> STM32 serial bridge.

This script is designed for the current STM32 UART receive logic:
  - RX command format on STM32: '@' + ASCII payload + '\r\n'
  - Example motion command: @0.2000,0.5000\r\n
  - Example brake command: @BRAKE\r\n

Incoming serial telemetry supports two simple text formats:
  1) ODOM,x,y,theta,vx,wz
  2) WHEEL,vl,vr

For the current G431 firmware, the preferred mode is:
  - RK3588 sends: @vx,wz\r\n
  - STM32 returns: ODOM,x,y,theta,vx,wz

Additional action commands can be sent as plain text, for example:
  - LEFT90
  - RIGHT90

Heading-offset commands can also be sent as degrees for the next straight-run
heading lock, for example:
  - 90.0   -> LEFT90
  - -90.0  -> RIGHT90

If you only return wheel speeds, the node can still integrate odometry on
RK3588 as a fallback. If you directly return x/y/theta/vx/wz, the node will
publish odometry from those values.

Usage:
  source /opt/ros/humble/setup.bash
  python3 stm32_ros2_bridge.py --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --baud 115200 --parity N

Dependencies:
  sudo apt install python3-serial
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster

try:
    import serial
    from serial import SerialException
except ImportError as exc:  # pragma: no cover - runtime dependency
    raise SystemExit(
        "pyserial is required. Install it with: sudo apt install python3-serial"
    ) from exc


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


@dataclass
class OdomState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vx: float = 0.0
    wz: float = 0.0
    stamp_sec: float = 0.0


class STM32Bridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("stm32_bridge")
        self.port = args.port
        self.baud = args.baud
        self.parity = args.parity.upper()
        self.data_bits = args.data_bits
        self.stop_bits = args.stop_bits
        self.wheel_base = args.wheel_base
        self.odom_frame = args.odom_frame
        self.base_frame = args.base_frame
        self.cmd_timeout = args.cmd_timeout
        self.publish_tf = args.publish_tf
        self.cmd_topic = args.cmd_topic
        self.action_topic = args.action_topic
        self.heading_offset_topic = args.heading_offset_topic
        self.angular_sign = args.angular_sign
        self.odom_linear_scale = args.odom_linear_scale
        self.odom_velocity_scale = args.odom_velocity_scale

        self.serial_lock = threading.Lock()
        self.ser: Optional[serial.Serial] = None
        self.rx_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.last_cmd_time = time.monotonic()
        self.last_vx = 0.0
        self.last_wz = 0.0
        self.brake_sent = False
        self.motion_seen = False
        self.odom = OdomState()
        self.action_sent_time: float = 0.0  # tracks when last action cmd was sent

        self.odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self.raw_pub = self.create_publisher(String, "/stm32/raw_rx", 20)
        self.status_pub = self.create_publisher(String, "/stm32/status", 20)
        self.battery_pub = self.create_publisher(BatteryState, "/battery_state", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, self.cmd_topic, self.on_cmd_vel, 20)
        self.create_subscription(String, self.action_topic, self.on_action_cmd, 10)
        self.create_subscription(Float32, self.heading_offset_topic, self.on_heading_offset_cmd, 10)
        self.create_timer(0.02, self.on_control_timer)
        self.create_timer(1.0, self.on_diag_timer)

        self.connect_serial()

    def connect_serial(self) -> None:
        parity_map = {
            "N": serial.PARITY_NONE,
            "E": serial.PARITY_EVEN,
            "O": serial.PARITY_ODD,
        }
        stop_bits_map = {
            1: serial.STOPBITS_ONE,
            2: serial.STOPBITS_TWO,
        }

        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0.02,
                bytesize=self.data_bits,
                parity=parity_map.get(self.parity, serial.PARITY_NONE),
                stopbits=stop_bits_map.get(self.stop_bits, serial.STOPBITS_ONE),
            )
        except SerialException as exc:
            self.ser = None
            self.get_logger().error(f"Failed to open serial {self.port}: {exc}")
            return

        self.stop_event.clear()
        self.brake_sent = False
        self.motion_seen = False
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()
        self.publish_status(f"serial_connected:{self.port}@{self.baud}")

    def close_serial(self) -> None:
        self.stop_event.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=0.5)
        if self.ser:
            try:
                self.ser.close()
            except SerialException:
                pass
        self.ser = None

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def format_cmd_payload(self, vx: float, wz: float) -> bytes:
        # Matches the STM32 '@ ... \r\n' receive state machine in Serial.c.
        return f"@{vx:.4f},{wz:.4f}\r\n".encode("ascii")

    def format_brake_payload(self) -> bytes:
        return b"@BRAKE\r\n"

    def format_action_payload(self, action: str) -> bytes:
        return f"@{action}\r\n".encode("ascii")

    def send_cmd(self, vx: float, wz: float) -> None:
        if not self.ser:
            return
        packet = self.format_cmd_payload(vx, wz)
        try:
            with self.serial_lock:
                self.ser.write(packet)
        except SerialException as exc:
            self.publish_status(f"serial_write_error:{exc}")

    def send_brake(self) -> None:
        if not self.ser:
            return
        try:
            with self.serial_lock:
                self.ser.write(self.format_brake_payload())
        except SerialException as exc:
            self.publish_status(f"serial_write_error:{exc}")

    def send_action(self, action: str) -> None:
        if not self.ser:
            return
        try:
            with self.serial_lock:
                self.ser.write(self.format_action_payload(action))
        except SerialException as exc:
            self.publish_status(f"serial_write_error:{exc}")

    def on_cmd_vel(self, msg: Twist) -> None:
        vx = float(msg.linear.x)
        wz = float(msg.angular.z) * self.angular_sign
        
        # 修复阿克曼小车倒车时的方向盘反转问题
        if vx < 0:
            wz = -wz

        is_zero_cmd = abs(vx) < 1e-4 and abs(wz) < 1e-4

        self.last_vx = vx
        self.last_wz = wz
        self.last_cmd_time = time.monotonic()
        if is_zero_cmd:
            # If an action command was recently sent (e.g. RIGHT90), suppress
            # BRAKE for 5 s so the avoidance node cannot clear the STM32 turn target.
            in_action_window = (time.monotonic() - self.action_sent_time) < 5.0
            if self.motion_seen and not self.brake_sent and not in_action_window:
                self.send_brake()
                self.brake_sent = True
                self.motion_seen = False
            return

        self.motion_seen = True
        self.brake_sent = False
        self.send_cmd(vx, wz)

    def on_action_cmd(self, msg: String) -> None:
        action = str(msg.data).strip().upper()
        allowed_actions = {"LEFT90", "RIGHT90", "TURN,LEFT90", "TURN,RIGHT90", "BRAKE", "LEFT180", "RIGHT180"}
        
        # 放行 RTC, ALARM, FINISH 等电源与时间管理指令
        is_rtc_cmd = action.startswith("RTC,") or action.startswith("ALARM,") or action == "FINISH"
        
        if action not in allowed_actions and not is_rtc_cmd:
            self.publish_status(f"ignored_action:{action}")
            return

        self.last_cmd_time = time.monotonic()
        self.last_vx = 0.0
        self.last_wz = 0.0
        self.motion_seen = False
        self.brake_sent = action == "BRAKE"
        self.action_sent_time = time.monotonic()  # start action window
        if action == "BRAKE":
            self.send_brake()
        else:
            self.send_action(action)
        self.publish_status(f"action_sent:{action}")

    def on_heading_offset_cmd(self, msg: Float32) -> None:
        offset_deg = float(msg.data)
        if abs(offset_deg - 90.0) <= 1.0:
            action = "LEFT90"
        elif abs(offset_deg + 90.0) <= 1.0:
            action = "RIGHT90"
        elif abs(offset_deg - 180.0) <= 1.0:
            action = "LEFT180"
        elif abs(offset_deg + 180.0) <= 1.0:
            action = "RIGHT180"
        elif abs(offset_deg) <= 1.0:
            self.publish_status("heading_offset_cleared:0.00")
            return
        else:
            self.publish_status(f"ignored_heading_offset:{offset_deg:.2f}")
            return

        self.last_cmd_time = time.monotonic()
        self.last_vx = 0.0
        self.last_wz = 0.0
        self.motion_seen = False
        self.brake_sent = False
        self.send_action(action)
        self.publish_status(f"heading_offset_sent:{offset_deg:.2f}->{action}")

    def on_control_timer(self) -> None:
        if not self.ser:
            return

        if time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            if self.motion_seen and not self.brake_sent:
                self.last_vx = 0.0
                self.last_wz = 0.0
                self.send_brake()
                self.brake_sent = True
                self.motion_seen = False

    def on_diag_timer(self) -> None:
        if self.ser is None:
            self.connect_serial()

    def rx_loop(self) -> None:
        assert self.ser is not None
        while not self.stop_event.is_set():
            try:
                raw = self.ser.readline()
            except SerialException as exc:
                self.publish_status(f"serial_read_error:{exc}")
                self.close_serial()
                return

            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            raw_msg = String()
            raw_msg.data = line
            self.raw_pub.publish(raw_msg)
            self.handle_line(line)

    def handle_line(self, line: str) -> None:
        parts = [p.strip() for p in line.split(",")]
        if not parts:
            return

        tag = parts[0].upper()
        try:
            if tag == "ODOM" and len(parts) >= 6:
                x, y, theta, vx, wz = map(float, parts[1:6])
                self.publish_odom_absolute(x, y, theta, vx, wz)
            elif tag == "WHEEL" and len(parts) >= 3:
                vl, vr = map(float, parts[1:3])
                self.publish_odom_from_wheels(vl, vr)
            elif tag in ("BATTERY", "BATT") and len(parts) >= 2:
                self.publish_battery(parts)
            elif tag == "IMU" and len(parts) >= 7:
                self.publish_imu(parts)
        except ValueError as exc:
            self.get_logger().warn(f"Bad telemetry line '{line}': {exc}")

    def publish_odom_absolute(
        self, x: float, y: float, theta: float, vx: float, wz: float
    ) -> None:
        now = self.get_clock().now()
        x_m = x * self.odom_linear_scale
        y_m = y * self.odom_linear_scale
        vx_mps = vx * self.odom_velocity_scale
        self.odom = OdomState(
            x=x_m,
            y=y_m,
            theta=theta,
            vx=vx_mps,
            wz=wz,
            stamp_sec=now.nanoseconds / 1e9,
        )
        self.publish_odom_and_tf(now)

    def publish_odom_from_wheels(self, vl: float, vr: float) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9
        if self.odom.stamp_sec == 0.0:
            self.odom.stamp_sec = now_sec
            return

        dt = now_sec - self.odom.stamp_sec
        if dt <= 0.0:
            return

        vx = 0.5 * (vr + vl)
        wz = (vr - vl) / self.wheel_base

        self.odom.theta += wz * dt
        self.odom.x += vx * math.cos(self.odom.theta) * dt
        self.odom.y += vx * math.sin(self.odom.theta) * dt
        self.odom.vx = vx
        self.odom.wz = wz
        self.odom.stamp_sec = now_sec
        self.publish_odom_and_tf(now)

    def publish_odom_and_tf(self, now) -> None:
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.odom.x
        odom_msg.pose.pose.position.y = self.odom.y
        odom_msg.pose.pose.orientation = yaw_to_quaternion(self.odom.theta)
        odom_msg.twist.twist.linear.x = self.odom.vx
        odom_msg.twist.twist.angular.z = self.odom.wz
        
        # Add default covariances to prevent EKF numerical explosion
        odom_msg.pose.covariance[0] = 0.01
        odom_msg.pose.covariance[7] = 0.01
        odom_msg.pose.covariance[14] = 99999.0
        odom_msg.pose.covariance[21] = 99999.0
        odom_msg.pose.covariance[28] = 99999.0
        odom_msg.pose.covariance[35] = 0.05
        
        odom_msg.twist.covariance[0] = 0.01
        odom_msg.twist.covariance[7] = 0.01
        odom_msg.twist.covariance[14] = 99999.0
        odom_msg.twist.covariance[21] = 99999.0
        odom_msg.twist.covariance[28] = 99999.0
        odom_msg.twist.covariance[35] = 0.05
        
        self.odom_pub.publish(odom_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.odom.x
        tf_msg.transform.translation.y = self.odom.y
        tf_msg.transform.rotation = yaw_to_quaternion(self.odom.theta)
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(tf_msg)

    def publish_battery(self, parts: list[str]) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(parts[1])
        if len(parts) >= 4:
            msg.current = float(parts[2])
            msg.percentage = float(parts[3]) / 100.0
        elif len(parts) >= 3:
            msg.percentage = float(parts[2]) / 100.0
        self.battery_pub.publish(msg)

    def publish_imu(self, parts: list[str]) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        msg.linear_acceleration.x = float(parts[1])
        msg.linear_acceleration.y = float(parts[2])
        msg.linear_acceleration.z = float(parts[3])
        msg.angular_velocity.x = float(parts[4])
        msg.angular_velocity.y = float(parts[5])
        msg.angular_velocity.z = float(parts[6])
        
        msg.orientation_covariance[0] = -1.0 # Orientation not provided
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        msg.linear_acceleration_covariance[0] = 0.05
        msg.linear_acceleration_covariance[4] = 0.05
        msg.linear_acceleration_covariance[8] = 0.05
        
        self.imu_pub.publish(msg)

    def destroy_node(self) -> bool:
        self.close_serial()
        return super().destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ROS2 STM32 serial bridge")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", help="Serial device")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "--parity",
        default="N",
        choices=["N", "E", "O", "n", "e", "o"],
        help="Serial parity: N/E/O",
    )
    parser.add_argument(
        "--data-bits",
        type=int,
        default=8,
        choices=[7, 8],
        help="Serial data bits",
    )
    parser.add_argument(
        "--stop-bits",
        type=int,
        default=1,
        choices=[1, 2],
        help="Serial stop bits",
    )
    parser.add_argument(
        "--wheel-base",
        type=float,
        default=0.23,
        help="Wheel separation in meters for WHEEL telemetry mode",
    )
    parser.add_argument("--odom-frame", default="odom", help="Odometry frame")
    parser.add_argument(
        "--base-frame", default="base_footprint", help="Base child frame"
    )
    parser.add_argument(
        "--cmd-timeout",
        type=float,
        default=0.5,
        help="Seconds after which cmd_vel timeout triggers a BRAKE command",
    )
    parser.add_argument(
        "--publish-tf",
        action="store_true",
        default=False,
        help="Publish raw odom->base TF from the bridge (disable when using robot_localization)",
    )
    parser.add_argument(
        "--cmd-topic",
        default="/cmd_vel",
        help="Twist topic consumed and forwarded to STM32",
    )
    parser.add_argument(
        "--action-topic",
        default="/stm32/action_cmd",
        help="String topic consumed and forwarded to STM32 as action commands",
    )
    parser.add_argument(
        "--heading-offset-topic",
        default="/heading_offset_cmd",
        help="Float32 topic for next straight-run heading offset in degrees, supports 0 and +/-90",
    )
    parser.add_argument(
        "--angular-sign",
        type=float,
        default=1.0,
        help="Multiplier applied to cmd_vel angular.z before sending to STM32",
    )
    parser.add_argument(
        "--odom-linear-scale",
        type=float,
        default=1.0,
        help="Scale factor applied to ODOM x/y before publishing, use 0.001 for mm->m",
    )
    parser.add_argument(
        "--odom-velocity-scale",
        type=float,
        default=1.0,
        help="Scale factor applied to ODOM vx before publishing, use 0.001 for mm/s->m/s",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = STM32Bridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
