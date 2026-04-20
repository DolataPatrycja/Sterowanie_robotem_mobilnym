import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
import json
import os


class robot_model(Node):
    def __init__(self):
        super().__init__('robot_model')

        config_path = os.path.join(
            os.path.dirname(__file__),
            '..',
            'config',
            'config.json'
        )
        with open(config_path, "r") as f:
            self.config = json.load(f)

        self.x = self.config["robot"]["initial_state"]["x"]
        self.y = self.config["robot"]["initial_state"]["y"]
        self.theta = self.config["robot"]["initial_state"]["theta"]

        self.v = 0.5
        self.omega = 0.5

        self.odom_pub = self.create_publisher(
            Odometry,
            self.config["topic"]["odom"]["name"],
            self.config["topic"]["odom"]["queue_size"]
        )

        # timer
        self.dt = self.config["topic"]["odom"]["publish_every_x_second"]
        self.timer = self.create_timer(self.dt, self.update)

        self.target_pose = None

        self.target_sub = self.create_subscription(
            PoseStamped,
            self.config["topic"]["target_pose"]["name"],
            self.target_callback,
            10
        )

    def target_callback(self, msg):
        self.target_pose = msg

    def update(self):
        if self.target_pose is not None:
            target_x = self.target_pose.pose.position.x
            target_y = self.target_pose.pose.position.y

            dx = target_x - self.x
            dy = target_y - self.y

            desired_theta = math.atan2(dy, dx)

            distance_error = math.sqrt(dx ** 2 + dy ** 2)
            angle_error = desired_theta - self.theta

            k_v = 1.0
            k_w = 2.0

            self.v = k_v * distance_error
            self.omega = k_w * angle_error

            # ograniczenia
            self.v = min(self.v, 1.0)
            self.omega = max(min(self.omega, 2.0), -2.0)

        # update modelu
        self.x += self.v * math.cos(self.theta) * self.dt
        self.y += self.v * math.sin(self.theta) * self.dt
        self.theta += self.omega * self.dt

        # normalizacja kąta (stabilność)
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # quaternion
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        msg.twist.twist.linear.x = self.v
        msg.twist.twist.angular.z = self.omega

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = robot_model()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()