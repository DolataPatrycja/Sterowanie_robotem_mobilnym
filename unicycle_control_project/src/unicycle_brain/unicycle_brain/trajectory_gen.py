"""
trajectory_gen.py

Węzeł ROS2 implementujący generator trajektorii referencyjnej dla robota mobilnego typu unicycle.

Publikuje:
    /target_pose
    /path

Typy trajektorii:
    - point
    - Lissajou_curves
"""

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
import rclpy

from unicycle_brain.utils import trajectory_type

import numpy as np
import os
import json


class trajectory_generator(Node):
    def __init__(self, traj_type: trajectory_type):
        super().__init__("trajectory_generator")

        config_path = os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "config.json"
        )

        with open(config_path, "r") as file:
            self.config = json.load(file)

        self.traj_type = traj_type

        self.dt = self.config["topics"]["target_pose"]["publish_every_x_second"]

        self.target_topic = self.config["topics"]["target_pose"]["name"]
        self.target_queue = self.config["topics"]["target_pose"]["queue_size"]

        self.path_topic = self.config["topics"]["path"]["name"]
        self.path_queue = self.config["topics"]["path"]["queue_size"]

        self.t = 0.0

        self.target_pub = self.create_publisher(
            PoseStamped,
            self.target_topic,
            self.target_queue
        )

        self.path_pub = self.create_publisher(
            Path,
            self.path_topic,
            self.path_queue
        )

        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        self.timer = self.create_timer(
            self.dt,
            self.trajectory_callback
        )

    def generate_point(self, msg):
        """
        Generuje trajektorię punktową.

        Args:
            msg (PoseStamped): Wiadomość wyjściowa.

        Returns:
            PoseStamped
        """

        msg.pose.position.x = self.config["trajectory"]["point"]["x"]
        msg.pose.position.y = self.config["trajectory"]["point"]["y"]

        return msg

    def generate_lissajous(self, msg):
        """
        Generuje trajektorię Lissajous.

        Args:
            msg (PoseStamped): Wiadomość wyjściowa.

        Returns:
            PoseStamped
        """

        A = self.config["trajectory"]["lissajous"]["A"]
        B = self.config["trajectory"]["lissajous"]["B"]
        a = self.config["trajectory"]["lissajous"]["a"]
        b = self.config["trajectory"]["lissajous"]["b"]
        delta = self.config["trajectory"]["lissajous"]["delta"]

        msg.pose.position.x = A * np.sin(a * self.t + delta)
        msg.pose.position.y = B * np.sin(b * self.t)

        return msg

    def update_path(self, msg, now):
        """
        Aktualizuje wiadomość Path.

        Args:
            msg (PoseStamped): Aktualny punkt trajektorii.
            now: Aktualny czas ROS.

        Returns:
            None
        """

        self.path_msg.header.stamp = now
        self.path_msg.poses.append(msg)

        if len(self.path_msg.poses) > 500:
            self.path_msg.poses.pop(0)

        self.path_pub.publish(self.path_msg)

    def trajectory_callback(self):
        """
        Callback timera.
            - generuje kolejny punkt trajektorii,
            - publikuje /target_pose,
            - aktualizuje /path.
        """

        msg = PoseStamped()

        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        msg.header.frame_id = "odom"

        if self.traj_type == trajectory_type.point:
            msg = self.generate_point(msg)

        elif self.traj_type == trajectory_type.Lissajou_curves:
            msg = self.generate_lissajous(msg)

        self.target_pub.publish(msg)

        self.update_path(msg, now)

        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)

    node = trajectory_generator(
        trajectory_type.Lissajou_curves
    )

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()