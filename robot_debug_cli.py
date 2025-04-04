#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, UInt8
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import subprocess
import time
import os
import sys
import signal

MICRO_ROS_AGENT_CMD = [
    'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
    'serial', '--dev', '/dev/ttyACM1'
]

agent_process = None

class RobotDebugCLI(Node):
    def __init__(self):
        super().__init__('robot_debug_cli')

        self.left_ticks = 0
        self.right_ticks = 0
        self.motor_enabled = False
        self.left_wheel_raised = True
        self.right_wheel_raised = True
        self.last_cmd_vel = (0.0, 0.0)

        self.create_subscription(Int32, '/left_encoder_ticks', self.left_ticks_cb, qos_profile_sensor_data)
        self.create_subscription(Int32, '/right_encoder_ticks', self.right_ticks_cb, qos_profile_sensor_data)
        self.create_subscription(UInt8, '/wheels_status', self.wheels_status_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, qos_profile_sensor_data)

        self.create_timer(0.5, self.print_status)

    def left_ticks_cb(self, msg):
        self.left_ticks = msg.data

    def right_ticks_cb(self, msg):
        self.right_ticks = msg.data

    def wheels_status_cb(self, msg):
        status = msg.data
        self.motor_enabled = bool(status & 0b001)
        self.left_wheel_raised = bool(status & 0b010)
        self.right_wheel_raised = bool(status & 0b100)

    def cmd_vel_cb(self, msg):
        self.last_cmd_vel = (msg.linear.x, msg.angular.z)

    def print_status(self):
        os.system('clear')
        print("\n🧪  Robot Debug CLI")
        print("==============================")
        print(f"Left encoder ticks : {self.left_ticks}")
        print(f"Right encoder ticks: {self.right_ticks}")
        print(f"Motors enabled     : {'✅' if self.motor_enabled else '❌'}")
        print(f"Left wheel on ground : {'❌' if self.left_wheel_raised else '✅'}")
        print(f"Right wheel on ground: {'❌' if self.right_wheel_raised else '✅'}")
        print(f"Last /cmd_vel      : linear={self.last_cmd_vel[0]:.2f} m/s, angular={self.last_cmd_vel[1]:.2f} rad/s")
        print("==============================")

def launch_micro_ros_agent():
    global agent_process
    print("🚀 Launching micro-ROS agent...")
    agent_process = subprocess.Popen(
        MICRO_ROS_AGENT_CMD,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    time.sleep(2)

def shutdown_micro_ros_agent():
    global agent_process
    if agent_process:
        print("🧹 Shutting down micro-ROS agent...")
        try:
            os.killpg(os.getpgid(agent_process.pid), signal.SIGTERM)
        except Exception as e:
            print("⚠️ Error shutting down agent:", e)

def main():
    launch_micro_ros_agent()
    rclpy.init()
    node = RobotDebugCLI()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🔌 Shutting down CLI...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        shutdown_micro_ros_agent()
        print("✅ Clean exit")
        sys.exit(0)

if __name__ == '__main__':
    main()
