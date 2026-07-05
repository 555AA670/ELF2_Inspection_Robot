import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from PyQt5.QtCore import QThread, pyqtSignal
import numpy as np

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from sensor_msgs.msg import BatteryState
import shutil

class ROSWorker(QThread):
    map_signal = pyqtSignal(object)  # Emits (map_data, width, height, resolution, origin_x, origin_y)
    pose_signal = pyqtSignal(float, float, float)  # Emits (x, y, yaw)
    log_signal = pyqtSignal(str) # Emits log strings
    status_signal = pyqtSignal(dict) # Emits system status dictionary
    chat_signal = pyqtSignal(str) # Emits pure LLM response for chat UI

    def __init__(self):
        super().__init__()
        self.node = None
        self.tf_buffer = None
        self.tf_listener = None
        self.timer = None

    def run(self):
        rclpy.init()
        self.node = Node('ui_ros_worker')

        # 1. Subscribe to LLM logs and create Publisher for Chat UI
        self.node.create_subscription(String, '/llm/response', self.llm_callback, 10)
        self.llm_pub = self.node.create_publisher(String, '/llm/prompt', 10)

        # 2. Subscribe to Map
        map_qos = QoSProfile(
            depth=1, 
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.node.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.node.get_logger().info("UI: 成功订阅 /map 话题...")

        # 3. 使用 TF 树获取小车位置
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.timer = self.node.create_timer(0.2, self.timer_callback) # 5 Hz 刷新位置

        # 4. 订阅电池状态
        self.node.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.sys_timer = self.node.create_timer(2.0, self.sys_timer_callback) # 2 Hz 刷新系统状态

        self.node.get_logger().info("UI ROS Worker Started!")
        rclpy.spin(self.node)

        self.node.destroy_node()
        rclpy.shutdown()

    def timer_callback(self):
        # 尝试获取 map 到 base_link 的坐标变换
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            # 尝试兼容 base_footprint
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            except TransformException:
                return

        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        self.pose_signal.emit(x, y, yaw)

    def llm_callback(self, msg):
        self.log_signal.emit(msg.data)
        self.chat_signal.emit(msg.data)

    def send_prompt(self, text):
        if hasattr(self, 'llm_pub') and self.llm_pub:
            msg = String()
            msg.data = text
            self.llm_pub.publish(msg)

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.map_signal.emit((data, width, height, resolution, origin_x, origin_y))

    def battery_callback(self, msg):
        self.status_signal.emit({
            "type": "battery",
            "voltage": round(msg.voltage, 2),
            "percentage": round(msg.percentage, 1) if msg.percentage else 0.0
        })

    def sys_timer_callback(self):
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp = round(int(f.read().strip()) / 1000.0, 1)
            self.status_signal.emit({"type": "temp", "value": temp})
        except:
            pass

    def stop(self):
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()
