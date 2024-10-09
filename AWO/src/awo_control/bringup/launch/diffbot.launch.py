
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node,SetRemap
from launch.actions import ( GroupAction,IncludeLaunchDescription)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
#from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node,SetRemap
from launch.actions import ( GroupAction,IncludeLaunchDescription)

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('awo_control'))
    
    cam_file = os.path.join(config_dir,'config','camera_config.yaml')

    ekf_file = os.path.join(config_dir,'config','ekf_config.yaml')

    imu_config = os.path.join(config_dir,'config','imu_config.yaml')
    
    
    imu = Node(
        package='mpu6050driver',
        executable='mpu6050driver',
        name='mpu6050driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[imu_config]

    )
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_file]
    ) 
    camera = Node(
      package='usb_cam',
      executable='usb_cam_node_exe',
      name='robot_cam',
      output='screen',
      parameters=[
        cam_file
      ]
    )
    #launching lidar
    lidar = Node(
      package='ldlidar_sl_ros2',
      executable='ldlidar_sl_ros2_node',
      name='ldlidar_publisher_ld14',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD14P'},
        {'laser_scan_topic_name': 'scan'},
        {'point_cloud_2d_topic_name': 'pointcloud2d'},
        {'frame_id': 'laser_frame'},
        {'port_name': '/dev/ttyUSB0'},
        {'serial_baudrate' : 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        
      ]
    )
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("awo_control"), "urdf", "diffbot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("awo_control"),
            "config",
            "arduino",
            "diffbot_controllers.yaml",
        ]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
    
            
        ],
    )
    

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_motor = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )
    robot_servo = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )
    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_motor,robot_servo],
        )
    )

    nodes = [
        #camera,
        #lidar,
        control_node,
      
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        
    ]

    return LaunchDescription(nodes)
