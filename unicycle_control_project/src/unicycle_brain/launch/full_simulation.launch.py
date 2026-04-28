from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
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

    ])