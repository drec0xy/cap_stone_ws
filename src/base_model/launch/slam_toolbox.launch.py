import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import SetLaunchConfiguration



def generate_launch_description():
    
    pkg_name = get_package_share_directory('base_model')
    config_dir = os.path.join(get_package_share_directory('base_model'), 'config')

    declare_robot_localization = DeclareLaunchArgument(
        'cartographer_ros',
        default_value=os.path.join(pkg_name, 'config', 'willabot_lds_2d.lua'),
        description='Whether to start mapping'
      )
    
    slam_tool_box = Node(
       package = 'slam_toolbox',
       executable = 'online_async_launch.launch.py',
       name = 'mapping_builder_node',
       output = 'screen',
       arguments=['-configuration_directory',config_dir, '-configuration_basename', 'mapper_params_online_async.yaml']
    )
          


    ld = LaunchDescription()
    # Set the sim_time parameter to True
    ld.add_action(SetLaunchConfiguration('use_sim_time', 'True'))
    #ld.add_action(declare_robot_localization)

    # Declare the launch options
    ld.add_action(slam_tool_box)


    return ld
 