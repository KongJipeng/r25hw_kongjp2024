from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 使用环境变量获取HOME路径
    home_dir = EnvironmentVariable('HOME')
    
    # 定义参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    rate = LaunchConfiguration('rate')
    figure8_sensor_bag = LaunchConfiguration('figure8_sensor_bag')
    figure8_feedback_bag = LaunchConfiguration('figure8_feedback_bag')
    
    # 声明参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bag) clock if true'
    )
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=[home_dir, '/r25hw_kongjp2024/HW3/task2/qos_best_effort.rviz'],
        description='Path to the RViz configuration file'
    )
    
    declare_figure8_sensor_bag_cmd = DeclareLaunchArgument(
        'figure8_sensor_bag',
        default_value=[home_dir, '/r25hw_kongjp2024/HW3/figure8_sensor'],
        description='Path to the figure8_sensor rosbag'
    )
    
    declare_figure8_feedback_bag_cmd = DeclareLaunchArgument(
        'figure8_feedback_bag',
        default_value=[home_dir, '/r25hw_kongjp2024/HW2/figure8_feedback'],
        description='Path to the figure8_feedback rosbag'
    )
    
    declare_rate_cmd = DeclareLaunchArgument(
        'rate',
        default_value='1',
        description='Rate at which to play bag file'
    )
    
    # 启动 RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    
    # 启动 robot_state_publisher 节点
    odom_cmd = Node(
        package='mobile_robotics_odom',
        executable='odometry',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    pose_cmd = Node(
        package='mobile_robotics_odom',
        executable='pose',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    pose3D_cmd = Node(
        package='pose2d_to_3d',
        executable='pose2D_to_3D_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    

    # 启动 rosbag play
    bag_play_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', figure8_feedback_bag, '--clock-topics-all', '--rate', rate],
        output='screen'
    )
    
    # 定义并返回LaunchDescription
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_figure8_sensor_bag_cmd)
    ld.add_action(declare_figure8_feedback_bag_cmd)
    ld.add_action(declare_rate_cmd)
    ld.add_action(odom_cmd)
    ld.add_action(pose_cmd)
    ld.add_action(pose3D_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bag_play_cmd)
    
    return ld