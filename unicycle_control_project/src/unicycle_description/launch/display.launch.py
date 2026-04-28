from launch import LaunchDescription
from launch_ros.actions import Node


def get_simple_robot_urdf():

    return """
    <robot name="simple_robot">

        <link name="base_link">

            <visual>
                <geometry>
                    <box size="0.6 0.4 0.25"/>
                </geometry>

                <origin xyz="0 0 0.125"/>

                <material name="blue">
                    <color rgba="0.1 0.4 1.0 1.0"/>
                </material>

            </visual>

        </link>

    </robot>
    """


def generate_launch_description():

    robot_desc = get_simple_robot_urdf()

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": robot_desc
            }],
            output="screen"
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0", "0", "0",
                "0", "0", "0",
                "map",
                "base_link"
            ],
            output="screen"
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen"
        )
    ])