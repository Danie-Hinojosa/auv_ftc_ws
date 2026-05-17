# Top-level launch:  Gazebo world + AUV spawn + T-S fuzzy controller
# + reference generator, all in one command.
#
#     ros2 launch auv_bringup full_simulation.launch.py
#
# Optional args:
#     model:=rexrov              spawn the RexROV instead of the torpedo
#     pattern:=yaw_sine          reference pattern (default: mission)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('auv_gazebo'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={'model': LaunchConfiguration('model')}.items(),
    )

    # Small delay so Gazebo has time to spawn the AUV and start publishing
    # /auv/odom before the controller starts its 50 Hz loop.
    # Was 3.0; reduced because Gazebo has no native buoyancy and during the
    # delay the AUV free-falls ~44 m, slamming into the seabed. The controller
    # already gates on have_odom_, so it can safely come up early.
    controller_launch = TimerAction(
        period=0.3,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('auv_control'), 'launch', 'controller.launch.py'])),
                launch_arguments={
                    'use_sim_time': 'true',
                    'model':        LaunchConfiguration('model'),
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model',        default_value='torpedo',
            description='Robot model to spawn: torpedo | rexrov'),
        gazebo_launch,
        controller_launch,
    ])
