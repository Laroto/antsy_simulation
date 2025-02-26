from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    script_path = os.path.join(get_package_share_directory('antsy_simulation'), 'scripts', 'launch_gz_script.py')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'world',
            default_value=[EnvironmentVariable('AMENT_PREFIX_PATH'), '/share/antsy_simulation/worlds/empty_world.sdf'],
            description='World file'
        ),
        DeclareLaunchArgument(
            'robot_description_package',
            default_value='antsy_description',
            description='Robot description package'
        ),
        DeclareLaunchArgument(
            'run_headless',
            default_value='false',
            description='Run headless'
        ),
        DeclareLaunchArgument(
            'run_on_start',
            default_value='true',
            description='Run on start'
        ),

        # Set NVIDIA EGL Environment Variables
        SetEnvironmentVariable(
            name='__EGL_VENDOR_LIBRARY_FILENAMES',
            value='/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
        ),
        SetEnvironmentVariable(
            name='__GLX_VENDOR_LIBRARY_NAME',
            value='nvidia'
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''), ':', EnvironmentVariable('AMENT_PREFIX_PATH'), '/share/antsy_description/..']
        ),

        # Start Gazebo
        ExecuteProcess(
            cmd=['python3', script_path, LaunchConfiguration('world'), LaunchConfiguration('run_headless'), LaunchConfiguration('run_on_start')],
            output='screen'
        )

        # # Delay to ensure Gazebo is fully running before launching other nodes
        # TimerAction(
        #     period=5.0,
        #     actions=[
        #         # ROS 2 -> Gazebo Bridge
        #         Node(
        #             package='ros_gz_bridge',
        #             executable='parameter_bridge',
        #             name='gazebo_bridge',
        #             output='screen',
        #             parameters=[{
        #                 'config_file': [EnvironmentVariable('AMENT_PREFIX_PATH'), '/share/antsy_simulation/config/bridge.yaml']
        #             }]
        #         ),

        #         # Yeet our beloved pet in the matrix
        #         Node(
        #             package='ros_gz_sim',
        #             executable='create',
        #             output='screen',
        #             arguments=['-topic', 'robot_description', '-name', 'hexapod', '-z', '0.2']
        #         )
        #     ]
        # )
    ])