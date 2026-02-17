import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from launch.actions import EmitEvent, DeclareLaunchArgument
from lifecycle_msgs.msg import Transition
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_enc_odom_arg = DeclareLaunchArgument(
        'use_enc_odom',
        default_value='true',
        description='Define se o node de odometria por encoder deve ser iniciado'
    )

    expansion_hub_pkg = get_package_share_directory('work_expansion_hub_driver')

    expansion_hub = LifecycleNode(
            package="work_expansion_hub_driver",
            executable="main",
            name="expansion_hub_node",
            namespace="",
            output="screen"
    )

    expansion_hub_configure = EmitEvent(
        event=ChangeState(
        lifecycle_node_matcher=matches_action(expansion_hub),
        transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    expansion_hub_activate = EmitEvent(
        event=ChangeState(
        lifecycle_node_matcher=matches_action(expansion_hub),
        transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    config_path = os.path.join(expansion_hub_pkg, 'config.hpp')
    odometry_node = Node(
        package='work_expansion_hub_driver',
        executable='encoder_odometry.py',
        name='encoder_odometry',
        output='screen',
        parameters=[{
            'config_path': config_path
        }],
        condition=IfCondition(LaunchConfiguration('use_enc_odom'))
    )

    return LaunchDescription([
        use_enc_odom_arg,

        expansion_hub,
        expansion_hub_configure,
        expansion_hub_activate,
        odometry_node
    ])