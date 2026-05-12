from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from datetime import datetime


def generate_launch_description():
    project_root = os.getcwd()

    bags_dir = os.path.join(
        project_root,
        'src',
        'unicycle_analytics',
        'bags'
    )

    os.makedirs(bags_dir, exist_ok=True)

    bag_name = datetime.now().strftime("run_%Y%m%d_%H%M%S")

    live_plot_script = os.path.join(
        project_root,
        'src',
        'unicycle_analytics',
        'scripts',
        'live_plot.py'
    )

    return LaunchDescription([

        ExecuteProcess(
            cmd=['python3', '-m', 'unicycle_brain.trajectory_gen'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', '-m', 'unicycle_brain.robot_model'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', '-m', 'unicycle_brain.controller'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', live_plot_script],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2',
                'bag',
                'record',
                '-o',
                os.path.join(bags_dir, bag_name),
                '/odom',
                '/cmd_vel',
                '/state_derivatives'
            ],
            output='screen'
        )
    ])