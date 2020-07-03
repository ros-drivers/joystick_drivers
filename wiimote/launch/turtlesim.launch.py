# Copyright 2020 Intel Corporation
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import launch
import launch_ros
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs.msg


def generate_launch_description():
    wiimote_node = launch_ros.actions.LifecycleNode(
        package='wiimote',
        executable='wiimote_node',
        namespace='',
        name='wiimote',
        output='screen',
        parameters=[
            FindPackageShare('wiimote').find('wiimote') + '/config/wiimote_params.yaml'
        ]
    )
    configure_wiimote = launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
        lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name('/wiimote'),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
    ))

    activate_wiimote = launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
        lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name('/wiimote'),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
    ))

    teleop_wiimote_node = launch_ros.actions.LifecycleNode(
        package='wiimote',
        executable='teleop_wiimote_node',
        namespace='',
        name='teleop_wiimote',
        output='screen',
        remappings=[
            ('/cmd_vel', 'turtle1/cmd_vel')
        ],
        parameters=[
            FindPackageShare('wiimote').find('wiimote') + '/config/teleop_wiimote_params.yaml'
        ]
    )

    configure_teleop_wiimote = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name(
                '/teleop_wiimote'),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        ))

    activate_teleop_wiimote = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name(
                '/teleop_wiimote'),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
        ))

    turtle_sim_node = launch_ros.actions.Node(package='turtlesim',
                                              executable='turtlesim_node',
                                              name='turtle1')

    on_configure_wiimote = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=wiimote_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg='wiimote successfully configured. '
                        'Proceeding to configure teleop_wiimote.'),
                configure_teleop_wiimote,
            ]
        ))

    on_configure_teleop_wiimote = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=teleop_wiimote_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg='teleop_wiimote configured. Proceeding to activate both nodes.'),
                activate_wiimote,
                activate_teleop_wiimote,
                launch.actions.LogInfo(msg='Starting turtlesim,'),
                turtle_sim_node
            ]
        ))

    on_finalized_wiimote = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=wiimote_node,
            goal_state='finalized',
            entities=[
                launch.actions.LogInfo(
                    msg='wiimote node shutdown. Shutting down entire launch process'),
                launch.actions.Shutdown(reason='wiimote node was shutdown')
            ]
        )
    )

    return launch.LaunchDescription(
        [launch.actions.DeclareLaunchArgument(name='emulate_tty', default_value='True'),
         wiimote_node,
         teleop_wiimote_node,
         configure_wiimote,
         on_configure_wiimote,
         on_configure_teleop_wiimote,
         on_finalized_wiimote])
