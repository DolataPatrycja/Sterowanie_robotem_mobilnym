#!/usr/bin/env python3

import os
import math
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from tf_transformations import euler_from_quaternion


def read_bag(path):
    odom = []

    with AnyReader([path]) as reader:
        connections = [
            c for c in reader.connections
            if c.topic == '/odom'
        ]

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            q = msg.pose.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            _, _, theta = euler_from_quaternion(quat)

            odom.append((timestamp * 1e-9, x, y, theta))

    return odom


def plot_data(data, output_dir):
    t = [d[0] for d in data]
    x = [d[1] for d in data]
    y = [d[2] for d in data]
    theta = [d[3] for d in data]

    os.makedirs(output_dir, exist_ok=True)

    plt.figure()
    plt.plot(x, y)
    plt.title('Trajectory XY')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.savefig(os.path.join(output_dir, 'trajectory_xy.png'))

    plt.figure()
    plt.plot(t, x)
    plt.title('X over time')
    plt.savefig(os.path.join(output_dir, 'x.png'))

    plt.figure()
    plt.plot(t, y)
    plt.title('Y over time')
    plt.savefig(os.path.join(output_dir, 'y.png'))

    plt.figure()
    plt.plot(t, theta)
    plt.title('Theta over time')
    plt.savefig(os.path.join(output_dir, 'theta.png'))

    plt.show()


def main():
    bag_path = '/home/user/unicycle_control_project/src/unicycle_analytics/bags/latest_run'
    output_dir = '/home/user/unicycle_control_project/src/unicycle_analytics/plots'

    data = read_bag(bag_path)
    plot_data(data, output_dir)


if __name__ == '__main__':
    main()