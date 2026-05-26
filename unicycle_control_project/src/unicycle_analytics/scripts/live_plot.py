#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class LivePlotNode(Node):

    def __init__(self):
        super().__init__('live_plot')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.target_sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.target_callback,
            10
        )

        self.x = []
        self.y = []

        self.ex = []
        self.ey = []
        self.eth = []

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0

        self.fig = plt.figure(figsize=(10, 8))

        self.ax_xy = self.fig.add_subplot(221)
        self.ax_ex = self.fig.add_subplot(222)
        self.ax_ey = self.fig.add_subplot(223)
        self.ax_eth = self.fig.add_subplot(224)

        self.anim = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=100
        )

    def target_callback(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        theta = quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        self.x.append(x)
        self.y.append(y)

        self.ex.append(self.target_x - x)
        self.ey.append(self.target_y - y)
        #elf.eth.append(self.target_theta - theta)

        desired_theta = math.atan2(self.target_y - y, self.target_x - x)
        theta_error = wrap_angle(desired_theta - theta)
        self.eth.append(theta_error)

    def update_plot(self, frame):
        self.ax_xy.clear()
        self.ax_ex.clear()
        self.ax_ey.clear()
        self.ax_eth.clear()

        self.ax_xy.plot(self.x, self.y)
        self.ax_xy.scatter([self.target_x], [self.target_y])
        self.ax_xy.set_title('Trajectory XY')

        self.ax_ex.plot(self.ex)
        self.ax_ex.set_title('Error X')

        self.ax_ey.plot(self.ey)
        self.ax_ey.set_title('Error Y')

        self.ax_eth.plot(self.eth)
        self.ax_eth.set_title('Error Theta')


def main():
    rclpy.init()

    node = LivePlotNode()

    try:
        plt.show(block=False)

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            plt.pause(0.01)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()