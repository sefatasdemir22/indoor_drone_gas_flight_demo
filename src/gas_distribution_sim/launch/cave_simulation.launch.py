import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gas_sim = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    home_dir = os.environ['HOME']

    darpa_models_path = os.path.join(
        pkg_gas_sim,
        'models',
        'subt_cave_sim',
        'models'
    )

    px4_models_path = os.path.join(
        home_dir,
        'src',
        'PX4-Autopilot',
        'Tools',
        'simulation',
        'gazebo-classic',
        'sitl_gazebo-classic',
        'models'
    )

    px4_build_path = os.path.join(
        home_dir,
        'src',
        'PX4-Autopilot',
        'build',
        'px4_sitl_default',
        'build_gazebo-classic'
    )

    model_parts = [darpa_models_path, px4_models_path]
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_parts.append(os.environ['GAZEBO_MODEL_PATH'])
    model_path = os.pathsep.join(model_parts)

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        plugin_path = px4_build_path + os.pathsep + os.environ['GAZEBO_PLUGIN_PATH']
    else:
        plugin_path = px4_build_path

    if 'LD_LIBRARY_PATH' in os.environ:
        ld_path = px4_build_path + os.pathsep + os.environ['LD_LIBRARY_PATH']
    else:
        ld_path = px4_build_path

    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=plugin_path),
        SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=ld_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': os.path.join(pkg_gas_sim, 'worlds', 'simple_corridor_room.world'),
                'verbose': 'true'
            }.items(),
        ),
    ])
