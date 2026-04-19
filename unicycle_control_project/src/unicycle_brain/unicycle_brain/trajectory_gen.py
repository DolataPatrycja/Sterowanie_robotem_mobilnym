from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
import rclpy

from unicycle_brain.utils import trajectory_type
import numpy as np
import time
import os
import json


class trajectory_generator(Node):
    def __init__(self, traj_type: trajectory_type, points_per_second):
        super().__init__('trajectory_generator')


        config_path = os.path.join(
            os.path.dirname(__file__),
            '..',
            'config',
            'config.json'
        )

        with open(config_path, "r") as f:
            self.config = json.load(f)
        self.traj_type = traj_type
        self.start_time = time.time()
        self.frequency_data_publish = self.config["topic"]["target_pose"]["publish_every_x_second"]
        self.msg_type = self.config["topic"]["target_pose"]["msg_type"]
        self.topic_name = self.config["topic"]["target_pose"]["name"]
        self.queue_size = self.config["topic"]["target_pose"]["queue_size"]

        self.trajectory_publisher = self.create_publisher(
            PoseStamped,
            self.topic_name,
            self.queue_size
        )
        self.traj_timer = self.create_timer(self.frequency_data_publish, self.trajectory_callback)

    def trajectory_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        t = time.time() - self.start_time

        if self.traj_type == trajectory_type.point:
            msg.pose.position.x = self.config["trajectory"]["point"]["x"]
            msg.pose.position.y = self.config["trajectory"]["point"]["y"]

        elif self.traj_type == trajectory_type.Lissajou_curves:
            A = self.config["trajectory"]["lissajous"]["A"]
            B = self.config["trajectory"]["lissajous"]["B"]
            a = self.config["trajectory"]["lissajous"]["a"]
            b = self.config["trajectory"]["lissajous"]["b"]
            delta = self.config["trajectory"]["lissajous"]["delta"]

            msg.pose.position.x = A * np.sin(a * t + delta)
            msg.pose.position.y = B * np.sin(b * t)

        self.trajectory_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = trajectory_generator(trajectory_type.Lissajou_curves, 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()