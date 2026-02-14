from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from launch.actions import EmitEvent
from lifecycle_msgs.msg import Transition
from launch_ros.actions import Node

def generate_launch_description():

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
    

    return LaunchDescription([
        expansion_hub,
        expansion_hub_configure,
        expansion_hub_activate
    ])