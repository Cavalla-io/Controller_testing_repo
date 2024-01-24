# Author: Rathin
 
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
 
def generate_launch_description():
 
  # Constants for paths to different files and folders
  gazebo_models_path = 'models'
  package_name = 'basic_mobile_robot'
  robot_name_in_model = 'forklift'
  urdf_model_path = './models/basic_mobile_bot_description/model.sdf'  
  world_file_path = 'worlds/basic_mobile_bot_world/smalltown.world'
     
  # Pose where we want to spawn the robot
  spawn_x_val = '0.0'
  spawn_y_val = '0.0'
  spawn_z_val = '0.0'
  spawn_yaw_val = '0.0'
 
  ############ You do not need to change anything below this line #############
   
  # Set the path to different files and folders.  
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share = FindPackageShare(package=package_name).find(package_name)
  world_path = os.path.join(pkg_share, world_file_path)
  gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
  urdf_model_path = os.path.join(pkg_share, urdf_model_path)
  pkg_four_ws_control = get_package_share_directory('four_ws_control')
  # Launch configuration variables specific to simulation
  gui = LaunchConfiguration('gui')
  headless = LaunchConfiguration('headless')
  namespace = LaunchConfiguration('namespace')
  sdf_model = LaunchConfiguration('sdf_model')
  use_namespace = LaunchConfiguration('use_namespace')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
  
  urdf_file = os.path.join(pkg_share, urdf_model_path)
  with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
  # Declare the launch arguments  
  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')
 
  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='false',
    description='Whether to apply a namespace to the navigation stack')
             
  # declare_sdf_model_path_cmd = DeclareLaunchArgument(
  #   name='sdf_model', 
  #   default_value=sdf_model_path, 
  #   description='Absolute path to robot sdf file')
 
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')
 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')
 
  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
   
  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())
 
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
 
  # Launch the robot
  spawn_entity_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', robot_name_in_model, 
                '-topic', 'robot_description',
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val],
                    output='screen')
  
  joint_state_publisher_gui = Node(
     package='joint_state_publisher_gui',
     executable='joint_state_publisher_gui',
     name='joint_state_publisher_gui',
     arguments=[urdf_file]
 )
  gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )
  
  robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                         'robot_description': robot_desc,
                        }],
            arguments=[urdf_file])
  forward_velocity_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'velocity_controller'], output='screen'
        )
  joint_state_broadcaster = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'], output='screen'
        )
  controller = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_four_ws_control, 'launch', 'four_ws_control.launch.py')
            ),
        )
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  # ld.add_action(declare_namespace_cmd)
  # ld.add_action(declare_use_namespace_cmd)
  # # ld.add_action(declare_sdf_model_path_cmd)
  # ld.add_action(declare_simulator_cmd)
  # ld.add_action(declare_use_sim_time_cmd)
  # ld.add_action(declare_use_simulator_cmd)
  # ld.add_action(declare_world_cmd)
 
  # # Add any actions
  # ld.add_action(start_gazebo_server_cmd)
  # ld.add_action(start_gazebo_client_cmd)
  # ld.add_action(robot_state_publisher)
  # # ld.add_action(spawn_entity_cmd)
  # ld.add_action(joint_state_broadcaster)
  # ld.add_action(forward_velocity_controller)
 
  # ld.add_action(joint_state_publisher_gui)
  return LaunchDescription([
    #  declare_namespace_cmd,
    #  declare_use_namespace_cmd,
    #  declare_simulator_cmd,
    #  declare_use_sim_time_cmd,
    #  declare_use_simulator_cmd,
    #  declare_world_cmd,
     start_gazebo_server_cmd,
     start_gazebo_client_cmd,
      # RegisterEventHandler(
      #       event_handler=OnProcessExit(
      #           target_action=spawn_entity_cmd,
      #           on_exit=[joint_state_broadcaster],
      #       )
      #   ),
     RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[forward_velocity_controller],
            )
        ),
    gazebo,
    robot_state_publisher,
    # spawn_entity_cmd,
    
  ])