from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 获取package路径
    pkg_share = FindPackageShare(package='task3_icp').find('task3_icp')
    
    # 使用环境变量获取HOME路径
    home_dir = EnvironmentVariable('HOME')
    
    # 定义参数
    bag_file = LaunchConfiguration('bag_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rate = LaunchConfiguration('rate')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # 声明参数
    declare_bag_file_cmd = DeclareLaunchArgument(
        'bag_file',
        default_value=[home_dir, '/r25hw_kongjp2024/HW3/figure8_sensor/figure8_sensor.db3'],
        description='Full path to bag file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bag) clock if true')
    
    declare_rate_cmd = DeclareLaunchArgument(
        'rate',
        default_value='0.1',
        description='Rate at which to play bag file')
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'config/icp_visualization.rviz'),
        description='Full path to RViz config file')
    
    # 启动ICP节点
    icp_scan_matcher_cmd = Node(
        package='task3_icp',
        executable='icp_scan_matcher',
        name='icp_scan_matcher',
        output='screen',
        parameters=[{
            'voxel_leaf_size': 0.05,
            'max_correspondence_distance': 0.1,
            'max_iterations': 50,
            'transformation_epsilon': 1e-8,
            'euclidean_fitness_epsilon': 1.0,
            'use_constant_velocity': True,
            'min_height': -0.5,
            'max_height': 1.0,
            'output_file': os.path.join(pkg_share, 'output/icp_trajectory.txt'),
            'use_sim_time': use_sim_time
        }]
    )
    
    # 启动RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 播放bag文件
    bag_play_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file, '--clock', '--rate', rate],
        output='screen'
    )
    
    # 定义并返回LaunchDescription
    ld = LaunchDescription()
    ld.add_action(declare_bag_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rate_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(icp_scan_matcher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bag_play_cmd)
    
    return ld