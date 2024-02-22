import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import SetLaunchConfiguration
from launch.actions import ExecuteProcess
import xacro


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('base_model')
    launch_dir = os.path.join(bringup_dir, 'launch')
# Specify the name of the package and path to xacro file within the package
    pkg_name = 'base_model'
    file_subpath = 'urdf/base_model.urdf.xacro'
    ekf_file = os.path.join(get_package_share_directory(pkg_name),'config','ekf.yaml')
     
     
    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

        
    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    urdf_file= LaunchConfiguration('urdf_file')
    package_name='base_model'
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'view.rviz'),
        description='Full path to the RVIZ config file to use')  
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    declare_use_joint_state_pub_cmd = DeclareLaunchArgument(
        'use_joint_state_pub',
        default_value='True',
        description='Whether to start the joint state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(bringup_dir, 'urdf', 'base_model.urdf.xacro'),
        description='Whether to start RVIZ')
    
    declare_gazebo_cmd = DeclareLaunchArgument(
        'use_gazebo',
        default_value=os.path.join(bringup_dir, 'config', 'gazebo_params.yaml'),
        description='Whether to start gazebo')
    
    declare_robot_localization = DeclareLaunchArgument(
        'robot_localization',
        default_value=os.path.join(package_name, 'config', 'ekf.yaml'),
        description='Whether to start robot_localization'
      )
    
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,'use_sim_time': True}],
        arguments=[urdf_file])
    
    start_joint_state_publisher_cmd = Node(
        #condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,'use_sim_time': True}],
        arguments=[urdf_file])
    
    start_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='both'
     )    
    
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'robot_description': robot_description_raw,'use_sim_time': True}],
        output='screen'
    )
 

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(get_package_share_directory(package_name),'meshes','visual')

    
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        parameters=[{'robot_description': robot_description_raw,'use_sim_time': True}],
        arguments=["-topic", "robot_description", 
                   "-entity", pkg_name, 
                   "-x", "0.0", 
                   "-y", "0.0", 
                   "-z", "0.0"
                   ],
        output="screen")
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    
    transform_pub_map = Node(
            package='tf2_ros',
            namespace = 'odom_to_baselink',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "odom", "lidar"]
        )
    transform_pub_base = Node(
            package='tf2_ros',
            namespace = 'odom_to_map',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "map", "odom"]
        )
       
    transform_pub_mapscan = Node(
            package='tf2_ros',
            namespace = 'scan_to_map',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "map", "scan"]
        )
    transform_pub = Node(
            package='tf2_ros',
            namespace = 'scan_to_map',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "odom", "scan"]
        )
    transform_pub_origin = Node(
            package='tf2_ros',
            namespace = 'odom_to_origin',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "odom", "base_link"]
        )
    
    ekf_file = os.path.join(get_package_share_directory(package_name),'config','ekf.yaml')

    declare_robot_localization = DeclareLaunchArgument(
        'robot_localization',
        default_value=os.path.join(package_name, 'config', 'ekf.yaml'),
        description='Whether to start robot_localization'
      )

    robot_localization_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}] 
      )
    

   
    # Create the launch description and populate
    ld = LaunchDescription()
    # Set the sim_time parameter to True
    ld.add_action(SetLaunchConfiguration('use_sim_time', 'True'))
    # Declare the launch options
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_robot_localization)
    ld.add_action(declare_urdf_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_joint_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_gazebo_cmd)


    # Add any conditioned actions
   # ld.add_action(run_node)
    ld.add_action(gazebo)
    ld.add_action(joystick)
    ld.add_action(transform_pub)
    ld.add_action(transform_pub_map)
    ld.add_action(transform_pub_base)
    ld.add_action(transform_pub_origin)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(robot_localization_node)
    ld.add_action(spawn_robot)
    ld.add_action(rviz_cmd)
    ld.add_action(twist_mux)

    return ld   
    

