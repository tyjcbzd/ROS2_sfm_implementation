import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource  


def generate_launch_description():

    urdf_tutorial_path = get_package_share_directory('gazebo_sfm_plugin')
    default_world_path = os.path.join(urdf_tutorial_path, "test_world/demo.world")
    
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments=[('world', default_world_path),('verbose','true')]
    )
    
    return launch.LaunchDescription([
        launch_gazebo,
    ])