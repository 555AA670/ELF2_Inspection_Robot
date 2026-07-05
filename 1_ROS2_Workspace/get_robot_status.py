#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import json
import os
import shutil

def get_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except Exception:
        return None

def get_storage_info():
    try:
        usage = shutil.disk_usage('/')
        return {
            "total_gb": round(usage.total / (1024**3), 1),
            "used_gb": round(usage.used / (1024**3), 1),
            "free_gb": round(usage.free / (1024**3), 1),
            "percent": round((usage.used / usage.total) * 100, 1)
        }
    except Exception:
        return None

def get_memory_info():
    try:
        with open('/proc/meminfo', 'r') as f:
            meminfo = {}
            for line in f:
                parts = line.split(':')
                if len(parts) == 2:
                    meminfo[parts[0].strip()] = int(parts[1].split()[0])
        total_mb = round(meminfo.get('MemTotal', 0) / 1024, 1)
        available_mb = round(meminfo.get('MemAvailable', 0) / 1024, 1)
        used_mb = round(total_mb - available_mb, 1)
        percent = round((used_mb / total_mb) * 100, 1) if total_mb > 0 else 0
        return {
            "total_mb": total_mb,
            "used_mb": used_mb,
            "free_mb": available_mb,
            "percent": percent
        }
    except Exception:
        return None

class StatusNode(Node):
    def __init__(self):
        super().__init__('get_robot_status_node')
        self.battery_msg = None
        self.sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        self.timeout_timer = self.create_timer(2.0, self.timeout_callback)

    def battery_callback(self, msg):
        self.battery_msg = msg
        result = {
            "status": "success",
            "tool": "get_robot_status",
            "voltage_v": round(msg.voltage, 2),
            "soc_percent": round(msg.percentage, 1) if msg.percentage else 0.0,
            "cpu_temp_c": get_cpu_temp(),
            "storage": get_storage_info(),
            "memory": get_memory_info()
        }
        print(json.dumps(result), flush=True)
        os._exit(0)

    def timeout_callback(self):
        result = {
            "status": "error",
            "tool": "get_robot_status",
            "message": "timeout waiting for /battery_state",
            "cpu_temp_c": get_cpu_temp(),
            "storage": get_storage_info(),
            "memory": get_memory_info()
        }
        print(json.dumps(result), flush=True)
        os._exit(1)

def main():
    rclpy.init()
    node = StatusNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
