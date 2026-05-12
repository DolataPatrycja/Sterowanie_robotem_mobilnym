"""
controller.py

Węzeł ROS2 implementujący regulator PID robota mobilnego typu unicycle.

Subskrybuje:
    /odom
    /target_pose

Publikuje:
    /cmd_vel
"""
import math
import json
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


class PIDController(Node):

    def __init__(self):
        super().__init__("controller")

        config_path = os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "config.json"
        )

        with open(config_path, "r") as file:
            self.config = json.load(file)

        self.control_period = self.config["topics"]["control"]["publish_every_x_second"]

        self.cmd_topic = self.config["topics"]["control"]["name"]
        self.cmd_queue = self.config["topics"]["control"]["queue_size"]

        self.odom_topic = self.config["topics"]["feedback"]["odom"]["name"]
        self.odom_queue = self.config["topics"]["feedback"]["odom"]["queue_size"]

        self.target_topic = self.config["topics"]["target_pose"]["name"]
        self.target_queue = self.config["topics"]["target_pose"]["queue_size"]

        self.max_linear_velocity = self.config["robot"]["limits"]["v_max"]
        self.max_angular_velocity = self.config["robot"]["limits"]["omega_max"]

        self.kp_linear = self.config["controller"]["linear_velocity"]["kp"]
        self.ki_linear = self.config["controller"]["linear_velocity"]["ki"]
        self.kd_linear = self.config["controller"]["linear_velocity"]["kd"]

        self.kp_angular = self.config["controller"]["angular_velocity"]["kp"]
        self.ki_angular = self.config["controller"]["angular_velocity"]["ki"]
        self.kd_angular = self.config["controller"]["angular_velocity"]["kd"]

        self.x = self.config["robot"]["initial_state"]["x"]
        self.y = self.config["robot"]["initial_state"]["y"]
        self.theta = self.config["robot"]["initial_state"]["theta"]

        self.target_x = None
        self.target_y = None

        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0

        self.integral_linear = 0.0
        self.integral_angular = 0.0

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_topic,
            self.cmd_queue
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            self.odom_queue
        )

        self.target_sub = self.create_subscription(
            PoseStamped,
            self.target_topic,
            self.target_callback,
            self.target_queue
        )

        self.timer = self.create_timer(
            self.control_period,
            self.control_loop
        )

    def odom_callback(self, msg):
        """
        aktualizuje wektor stanu robota

        Args:
            msg : zczytany stan robota z modelu
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def target_callback(self, msg):
        """
        aktualizuje następny punkt trajektorii referencyjnej

        Args:
            msg : następna pozycja trajektorii referencyjnej.
        """
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

    def wrap_angle(self, angle):
        """
        Normalizuje kąt do zakresu [-pi, pi].

        Args:
            angle (float): Kąt w radianach.

        Returns:
            float: Znormalizowany kąt w radianach.
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def saturate(self, value, lower_limit, upper_limit):
        """
           Ogranicza wartość do zadanego przedziału.

           Args:
               value (float): Wartość wejściowa.
               lower_limit (float): Dolna granica.
               upper_limit (float): Górna granica.

           Returns:
               float: Wartość po ograniczeniu.
           """
        return max(lower_limit, min(value, upper_limit))

    def pid_step(self, error, prev_error, integral, kp, ki, kd):
        """
        oblicza sygnał zadany regulatora PID.

        Args:
            error (float): Aktualny uchyb regulacji.
            prev_error (float): Uchyb z poprzedniej iteracji.
            integral (float): Aktualna wartość całki błędu.
            kp (float): Wzmocnienie członu proporcjonalnego.
            ki (float): Wzmocnienie członu całkującego.
            kd (float): Wzmocnienie członu różniczkującego.

        Returns:
            tuple:
                output (float): Wyjście regulatora PID.
                integral (float): Zaktualizowana całka błędu.
        """
        dt = self.control_period

        integral = integral + error * dt

        derivative = (error - prev_error) / dt

        output = (
                kp * error +
                ki * integral +
                kd * derivative
        )

        return output, integral

    def control_loop(self):
        """
        Główna pętla sterowania robota.

        Funkcja:
            - odczytuje aktualny stan robota,
            - oblicza uchyb położenia,
            - wyznacza sygnały sterujące PID,
            - ogranicza prędkości,
            - publikuje komendę /cmd_vel.
        """
        if self.target_x is None or self.target_y is None:
            return

        dx = self.target_x - self.x
        dy = self.target_y - self.y

        distance_error = math.sqrt(dx * dx + dy * dy)

        desired_theta = math.atan2(dy, dx)

        angle_error = self.wrap_angle(
            desired_theta - self.theta
        )

        linear_cmd, self.integral_linear = self.pid_step(
            distance_error,
            self.prev_linear_error,
            self.integral_linear,
            self.kp_linear,
            self.ki_linear,
            self.kd_linear
        )

        angular_cmd, self.integral_angular = self.pid_step(
            angle_error,
            self.prev_angular_error,
            self.integral_angular,
            self.kp_angular,
            self.ki_angular,
            self.kd_angular
        )

        # linear_cmd = self.saturate(
        #     linear_cmd,
        #     -self.max_linear_velocity,
        #     self.max_linear_velocity
        # )
        #
        # angular_cmd = self.saturate(
        #     angular_cmd,
        #     -self.max_angular_velocity,
        #     self.max_angular_velocity
        # )

        cmd = Twist()

        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd

        self.cmd_pub.publish(cmd)

        self.prev_linear_error = distance_error
        self.prev_angular_error = angle_error


def main(args=None):
    rclpy.init(args=args)

    node = PIDController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
