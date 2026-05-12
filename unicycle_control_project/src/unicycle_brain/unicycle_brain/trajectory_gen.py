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
        self.param_t = 0.0
        self.reference_speed = 1.0

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

    def advance_parameter(self, dx_dt, dy_dt):
        """
        Aktualizje punkt referencyjny py
        poruszał się ze stałą prędkością 1 m/s.

        Args:
            dx_dt (float): Prędkośc w x.
            dy_dt (float): Prędkość w y.
        """

        reference_speed = 1.0

        current_speed = np.sqrt(dx_dt ** 2 + dy_dt ** 2)

        if current_speed < 0.1:
            current_speed = 0.1

        delta_t = (reference_speed / current_speed) * self.dt

        delta_t = min(delta_t, 1.5 * self.dt)

        self.t += delta_t

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

        dx_dt = A * a * np.cos(a * self.t + delta)
        dy_dt = B * b * np.cos(b * self.t)

        self.advance_parameter(dx_dt, dy_dt)

        return msg

    def generate_curve(self,msg):
        """
        Generuje trajektorię łuku koła.

        Args:
            r             - promień łuku
            alpha         - kąt końcowy [rad]
            turning_right - kierunek skrętu

        Returns:
            PoseStamped
        """
        r = self.config["trajectory"]["curve"]["r"]
        alpha = self.config["trajectory"]["curve"]["alpha"]
        turning_right = self.config["trajectory"]["curve"]["turning_right"]

        if not hasattr(self, "arc_angle"):
            self.arc_angle = 0.0

        if not hasattr(self, "arc_finished"):
            self.arc_finished = False

        if not hasattr(self, "arc_speed"):
            self.arc_speed = 1.0 / r

        if turning_right:
            circle_middle_point_x = r
            circle_middle_point_y = 0
            direction = -1.0
        else:
            circle_middle_point_x = -r
            circle_middle_point_y = 0
            direction = 1.0
        if not self.arc_finished:
            self.arc_angle += self.arc_speed * self.dt

            if self.arc_angle >= alpha:
                self.arc_angle = alpha
                self.arc_finished = True

        phi = direction * self.arc_angle

        msg.pose.position.x = circle_middle_point_x + r * np.cos(phi)
        msg.pose.position.y = circle_middle_point_y + r * np.sin(phi)

        return msg

    def generate_harmonic(self, msg):
        """
        Generuje trajektorię harmoniczną sinusoidalną.

        Args:
            msg (PoseStamped): Wiadomość wyjściowa.

        Returns:
            PoseStamped
        """

        w = self.config["trajectory"]["harmonic"]["w"]
        A = self.config["trajectory"]["harmonic"]["Amplitude"]

        msg.pose.position.x = A * np.sin(w * self.t)
        msg.pose.position.y = self.t * 0.3

        dx_dt = A * w * np.cos(w * self.t)
        dy_dt = 0.3

        self.advance_parameter(dx_dt, dy_dt)

        return msg

    def generate_square(self, msg):
        """
        Generuje trajektorię kwadratu.

        Args:
            msg (PoseStamped): Wiadomość wyjściowa.

        Returns:
            PoseStamped
        """

        side_length = self.config["trajectory"]["square"]["side_length"]

        local_t = self.t % (4.0 * side_length)

        if local_t < side_length:
            msg.pose.position.x = local_t
            msg.pose.position.y = 0.0

        elif local_t < 2.0 * side_length:
            msg.pose.position.x = side_length
            msg.pose.position.y = local_t - side_length

        elif local_t < 3.0 * side_length:
            msg.pose.position.x = 3.0 * side_length - local_t
            msg.pose.position.y = side_length

        else:
            msg.pose.position.x = 0.0
            msg.pose.position.y = 4.0 * side_length - local_t

        return msg

    def generate_saw(self, msg):
        """
        Generuje trajektorię piłokształtną.

        Args:
            msg (PoseStamped): Wiadomość wyjściowa.

        Returns:
            PoseStamped
        """

        period = self.config["trajectory"]["saw"]["w"]
        A = self.config["trajectory"]["saw"]["Amplitude"]

        msg.pose.position.x = A * ((self.t % period) / period)
        msg.pose.position.y = self.t * 0.3

        dx_dt = A / period
        dy_dt = 0.3

        self.advance_parameter(dx_dt, dy_dt)

        return msg

    def update_path(self, msg, now):
        """
        Aktualizuje wiadomość Path.

        Args:
            msg (PoseStamped): Wiadomość wyjściowa.
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

        elif self.traj_type == trajectory_type.curve:
            msg = self.generate_curve(msg)

        elif self.traj_type == trajectory_type.harmonic:
            msg = self.generate_harmonic(msg)

        elif self.traj_type == trajectory_type.square:
            msg = self.generate_square(msg)

        elif self.traj_type == trajectory_type.saw:
            msg = self.generate_saw(msg)

        self.target_pub.publish(msg)

        self.update_path(msg, now)

        if self.traj_type in [
            trajectory_type.point,
            trajectory_type.square,
            trajectory_type.curve
        ]:
            self.t += self.dt


def main(args=None):
    rclpy.init(args=args)

    config_path = os.path.join(
        os.path.dirname(__file__),
        "..",
        "config",
        "config.json"
    )

    with open(config_path, "r") as file:
        config = json.load(file)

    active_name = config["trajectory"]["active_type"]

    enum_map = {
        "point": trajectory_type.point,
        "lissajous": trajectory_type.Lissajou_curves,
        "curve": trajectory_type.curve,
        "harmonic": trajectory_type.harmonic,
        "square": trajectory_type.square,
        "saw": trajectory_type.saw
    }

    selected_type = enum_map[active_name.lower()]

    node = trajectory_generator(selected_type)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()