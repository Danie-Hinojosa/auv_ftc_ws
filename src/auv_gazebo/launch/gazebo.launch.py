# Launch Gazebo Classic 11 with the underwater world and spawn the AUV.
# Supports model selection via the `model` launch argument:
#     model:=torpedo   small torpedo AUV (default, fast, ~25 kg)
#     model:=rexrov    RexROV-based ROV (1862 kg, RexROV STL mesh)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_auv_desc   = get_package_share_directory('auv_description')
    pkg_auv_gazebo = get_package_share_directory('auv_gazebo')

    # Gazebo Classic resolves mesh URIs of the form
    #   package://<pkg>/...   -> passed through by spawn_entity.py
    #   model://<name>/...    -> search GAZEBO_MODEL_PATH for <name>/
    # We prepend the install share/ so the mesh under
    # install/auv_description/share/auv_description/meshes is reachable.
    #
    # NOTE: we deliberately do NOT set GAZEBO_RESOURCE_PATH here — touching
    # that variable from launch_ros reliably clobbers the Gazebo setup that
    # makes /usr/share/gazebo-11 available, producing the errors:
    #   [Err] [RTShaderSystem.cc:480] Unable to find shader lib
    #   [Err] [RenderEngine.cc:197] Failed to initialize scene
    #   gzclient: ... Assertion `px != 0\' failed
    # GAZEBO_MODEL_PATH alone is sufficient for resolving mesh URIs.
    share_parent = os.path.dirname(pkg_auv_desc)

    world_file = os.path.join(pkg_auv_gazebo, 'worlds', 'underwater.world')
    model      = LaunchConfiguration('model').perform(context)
    if model == 'rexrov':
        xacro_file = os.path.join(pkg_auv_desc, 'urdf', 'rexrov.urdf.xacro')
        z0 = '-5.0'
    else:
        xacro_file = os.path.join(pkg_auv_desc, 'urdf', 'auv.urdf.xacro')
        # Spawn shallower so the AUV has room to stabilize before reaching the
        # -20 m seabed (the controller has ~3 s of dead time at startup).
        z0 = '-1.0'

    x0, y0 = '0.0', '0.0'

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str)
    }
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'auv',
            '-x', x0, '-y', y0, '-z', z0,
        ],
        output='screen',
    )

    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = share_parent + (':' + existing if existing else '')

    print(f'[auv_gazebo] model={model}  xacro={xacro_file}  spawn z={z0}')
    print(f'[auv_gazebo] GAZEBO_MODEL_PATH += {share_parent}')

    return [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_model_path),
        # Defensively disable Gazebo Fuel online lookups so slow/blocked
        # networks can never stall the sim startup.
        SetEnvironmentVariable('IGN_FUEL_ENABLE_CACHING', '0'),
        gzserver, gzclient, rsp, spawner,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model',
            default_value='torpedo',
            description='Robot model to spawn: torpedo | rexrov'),
        OpaqueFunction(function=_launch_setup),
    ])
