import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch_ros.actions.Node(
            package='carla_shenlan_bridge_ego_vis',
            executable='carla_shenlan_vis_ego_vehicle',
            name=['carla_shenlan_vis_ego_vehicle_', launch.substitutions.LaunchConfiguration('role_name')],
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                os.path.join(
                    get_package_share_directory('carla_shenlan_bridge_ego_vis'),
                    'config',
                    'carla_shenlan_bridge_ego_vis_foxy.rviz')
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()