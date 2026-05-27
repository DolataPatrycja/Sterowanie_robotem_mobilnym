"""
robot_model.py

Węzeł ROS2 implementujący modelrobota mobilnego typu unicycle.

Subskrybuje:
    /cmd_vel

Publikuje:
    /odom
Model:
    x_dot = v * cos(theta)
    y_dot = v * sin(theta)
    theta_dot = omega
"""
import math
import json
import os

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class RobotModel(Node):
    def __init__(self):

        super().__init__("robot_model")

        config_path = os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "config.json"
        )

        with open(config_path, "r") as file:
            self.config = json.load(file)

        self.x = self.config["robot"]["initial_state"]["x"]
        self.y = self.config["robot"]["initial_state"]["y"]
        self.theta = self.config["robot"]["initial_state"]["theta"]

        self.v_cmd = 0.0
        self.omega_cmd = 0.0

        self.dt = self.config["robot"]["model"]["integration_dt"]

        self.odom_topic = self.config["topics"]["feedback"]["odom"]["name"]
        self.odom_queue = self.config["topics"]["feedback"]["odom"]["queue_size"]

        self.cmd_topic = self.config["topics"]["control"]["name"]
        self.cmd_queue = self.config["topics"]["control"]["queue_size"]

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            self.odom_queue
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_topic,
            self.cmd_callback,
            self.cmd_queue
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(
            self.dt,
            self.update
        )

    def cmd_callback(self, msg):
        """
        Odbiera sygnał sterujący robota.

        Args:
            msg (Twist):
                linear.x  - prędkość liniowa [m/s]
                angular.z - prędkość kątowa [rad/s]
        """

        self.v_cmd = msg.linear.x
        self.omega_cmd = msg.angular.z

    def update_position_based_on_model(self):
        """
        Aktualizuje stan robota na podstawie modelu unicycle metodą całkowania Eulera.

        Równania modelu :
            x(k+1) = x(k) + v*cos(theta)*dt
            y(k+1) = y(k) + v*sin(theta)*dt
            theta(k+1) = theta(k) + omega*dt
        """

        self.x += self.v_cmd * math.cos(self.theta) * self.dt
        self.y += self.v_cmd * math.sin(self.theta) * self.dt
        self.theta += self.omega_cmd * self.dt

    def publish_odometry(self):
        """Tworzy i publikuje wiadomość odometrii oraz TF robota."""

        current_time = self.get_clock().now().to_msg()

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # 1. Publikacja /odom
        msg = Odometry()
        msg.header.stamp = current_time
        msg.header.frame_id = "odom"

        msg.child_frame_id = "base_footprint"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = self.v_cmd
        msg.twist.twist.angular.z = self.omega_cmd

        self.odom_pub.publish(msg)

        transform = TransformStamped()
        transform.header.stamp = current_time
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)

    def update(self):
        self.update_position_based_on_model()
        self.publish_odometry()


def main(args=None):
    rclpy.init(args=args)

    node = RobotModel()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()