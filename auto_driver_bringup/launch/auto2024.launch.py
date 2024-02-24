import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
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
                        {'i': 30},
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
                        {'p': 100},
                        {'i': 30},
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
                        {'center_offset': -90.0},
                        {'from_frame_id': 'base_link'},
                        {'to_frame_id': 'tf0'},
                        {'target_angle_axis': "yaw"},
                        {'offset_x': 0.0},
                        {'offset_y': 0.0},
                        {'offset_z': 0.0}
                    ]
                ),
                ComposableNode(
                    package='topic_to_tf_node',
                    plugin='topic_to_tf_node::Topic2TfNode',
                    name='topic_to_tf_tf0_to_tf1',
                    namespace='can_node/gm6020_0',
                    parameters=[
                        {'center_offset': -180.0},
                        {'from_frame_id': 'tf0'},
                        {'to_frame_id': 'tf1'},
                        {'target_angle_axis': "pitch"},
                        {'offset_x': 0.0},
                        {'offset_y': 0.0},
                        {'offset_z': 0.15}
                    ]
                ),
            ],
        )
    ])
