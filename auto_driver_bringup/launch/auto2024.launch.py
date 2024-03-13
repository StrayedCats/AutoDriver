from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

import os
import xacro

def generate_launch_description():
    robot_name = "auto_driver"
    package_name = robot_name + "_description"
    robot_description = os.path.join(get_package_share_directory(
        package_name), "urdf", robot_name + ".urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)

    use_core1_hardware = LaunchConfiguration('use_core1_hardware', default=True)
    use_core1_hardware_arg = DeclareLaunchArgument('use_core1_hardware', default_value=use_core1_hardware, description='Use viewer')

    return LaunchDescription([
        use_core1_hardware_arg,
        ComposableNodeContainer(
            name='auto_driver_interface',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    condition=IfCondition(use_core1_hardware),
                    package='auto_driver_interface',
                    plugin='auto_driver_interface::PidNode',
                    name='pitch_motor',
                    namespace='can_node/gm6020_0',
                    parameters=[
                        {'p': 100},
                        {'i': 1},
                        {'d': 5},
                        {'max_spd': 3000},
                        {'init_deg': 120},
                        {'min_limit': 20},
                        {'max_limit': 135}
                    ]
                ),

                ComposableNode(
                    condition=IfCondition(use_core1_hardware),
                    package='auto_driver_interface',
                    plugin='auto_driver_interface::PidNode',
                    name='yaw_motor',
                    namespace='can_node/gm6020_1',
                    parameters=[
                        {'p': 80},
                        {'i': 1},
                        {'d': 5},
                        {'max_spd': 2000},
                        {'init_deg': 180},
                        {'min_limit': 25},
                        {'max_limit': 315}
                    ]
                ),
                ComposableNode(
                    condition=IfCondition(use_core1_hardware),
                    package='auto_driver_interface',
                    plugin='auto_driver_interface::RollerNode',
                    name='roller0',
                    namespace='can_node/c620_0',
                    parameters=[
                        {'invert': False}
                    ]
                ),
                ComposableNode(
                    condition=IfCondition(use_core1_hardware),
                    package='auto_driver_interface',
                    plugin='auto_driver_interface::RollerNode',
                    name='roller1',
                    namespace='can_node/c620_1',
                    parameters=[
                        {'invert': True}
                    ]
                ),
                ComposableNode(
                    condition=IfCondition(use_core1_hardware),
                    package='auto_driver_interface',
                    plugin='auto_driver_interface::HammerNode',
                    name='hammer',
                    namespace='servo0'
                ),
                ComposableNode(
                    condition=IfCondition(use_core1_hardware),
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
                        {'invert_degree': False},
                        {'speed_multiplier': 0.5}
                    ]
                ),
                ComposableNode(
                    condition=IfCondition(use_core1_hardware),
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
                        {'invert_degree': True},
                        {'speed_multiplier': 0.5}
                    ]
                )
            ],
        ),
        Node(
            condition=IfCondition(use_core1_hardware),
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen",
        )
    ])
