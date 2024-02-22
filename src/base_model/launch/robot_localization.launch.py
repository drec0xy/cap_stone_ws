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

      package_name='base_model'
      ekf_file = os.path.join(get_package_share_directory(package_name),'config','ekf.yaml')

      declare_robot_localization = DeclareLaunchArgument(
        'robot_localization',
        default_value=os.path.join(package_name, 'config', 'ekf.yaml'),
        description='Whether to start robot_localization'
      ),

      robot_localization_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}] 
      )


      ld = LaunchDescription()
      # Set the sim_time parameter to True
      ld.add_action(SetLaunchConfiguration('use_sim_time', 'True'))
      # Declare the launch options
      ld.add_action(declare_robot_localization)
      ld.add_action(robot_localization_node)


      return ld
