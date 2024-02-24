from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

import os
import xacro

def generate_launch_description():
    robot_name = "auto_driver"
    package_name = robot_name + "_description"
    rviz_config = os.path.join(get_package_share_directory(
        package_name), "launch", robot_name + ".rviz")
    robot_description = os.path.join(get_package_share_directory(
        package_name), "urdf", robot_name + ".urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)

    return launch.LaunchDescription([
        ComposableNodeContainer(
            name='auto_driver_interface',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='auto_driver_interface',
                    plugin='auto_driver_interface::PidNode',
                    name='pitch_motor',
                    namespace='can_node/gm6020_0',
                    parameters=[
                        {'p': 50},
                        {'i': 10},
                        {'d': 5},
                        {'max_spd': 8000},
                        {'init_deg': 120},
                        {'min_limit': 20},
                        {'max_limit': 135}
                    ]
                ),

                ComposableNode(
                    package='auto_driver_interface',
                    plugin='auto_driver_interface::PidNode',
                    name='yaw_motor',
                    namespace='can_node/gm6020_1',
                    parameters=[
                        {'p': 50},
                        {'i': 10},
                        {'d': 5},
                        {'max_spd': 8000},
                        {'init_deg': 180},
                        {'min_limit': 25},
                        {'max_limit': 315}
                    ]
                ),
                ComposableNode(
                    package='auto_driver_interface',
                    plugin='auto_driver_interface::RollerNode',
                    name='roller0',
                    namespace='can_node/c620_0',
                    parameters=[
                        {'invert': False}
                    ]
                ),
                ComposableNode(
                    package='auto_driver_interface',
                    plugin='auto_driver_interface::RollerNode',
                    name='roller1',
                    namespace='can_node/c620_1',
                    parameters=[
                        {'invert': True}
                    ]
                ),
                ComposableNode(
                    package='auto_driver_interface',
                    plugin='auto_driver_interface::HammerNode',
                    name='hammer',
                    namespace='servo0'
                ),
                # topic_to_tf_node::Topic2TfNode
                ComposableNode(
                    package='topic_to_tf_node',
                    plugin='topic_to_tf_node::Topic2TfNode',
                    name='topic_to_tf_base_to_tf0',
                    namespace='can_node/gm6020_1',
                    parameters=[
                        {'center_offset': -180.0},
                        {'from_frame_id': 'fake_1_to_2'},
                        {'to_frame_id': 'yaw_link'},
                        {'target_angle_axis': "yaw"},
                        {'offset_x': 0.0},
                        {'offset_y': 0.0},
                        {'offset_z': 0.0},
                        {'invert_degree': False}
                    ]
                ),
                ComposableNode(
                    package='topic_to_tf_node',
                    plugin='topic_to_tf_node::Topic2TfNode',
                    name='topic_to_tf_tf0_to_tf1',
                    namespace='can_node/gm6020_0',
                    parameters=[
                        {'center_offset': -120.0},
                        {'from_frame_id': 'fake_2_to_3'},
                        {'to_frame_id': 'pitch_link'},
                        {'target_angle_axis': "roll"},
                        {'offset_x': 0.0},
                        {'offset_y': 0.0},
                        {'offset_z': 0.0},
                        {'invert_degree': True}
                    ]
                )
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )
    ])
