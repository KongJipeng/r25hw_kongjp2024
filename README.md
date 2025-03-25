# r25hw_kongjp2024

## Overview
This is a repo for homework of Robotics2025, belonging to kongjp.
I would write some information and suggestions here.

## HW1 done
take one day.

ROS2 Iron is installed very well and quickly on wsl2, windows11.
Shakey Robot is very cool.

## HW2 done
take three days.

Maybe the discription for the every task in homework is not so clear.
For example, what should I output in the first question - calculate the odom? 

How to run:
 - $ rviz2
 - add path and pose, set fixed_fram base_link
 - $ ros2 run mobile_robotics_odom odometry
 - $ ros2 run mobile_robotics_odom pose
 - $ ros2 run pose2d_to_3d pose2D_to_3D_node
 - $ ros2 bag play figure8_feedback --clock-topics-all

The key to success is to finetune wheel_separation_multiplier = 1.617 to make the circles more like 8.


## HW3 doing
take three days...

### task1:
Dependencies:
- `pip install evo`
- `pip install numpy=1.23.5` //scipy conflict
- `sudo apt remove python3-matplotlib`    //multi matplotlib conflict

`evo_traj bag topic --save_as tum` to convert bag to tum.

`evo_traj tum tum1 tum2 --align --plot_mode xz` to compare on one plane 

### task2:
wrong:
 `<node pkg="rosbag2" exec="play" name="rosbag_player" output="screen" args="$(var bag_file) " />`

correct:
 `<executable cmd="ros2 bag play $(var bag_file)" output="screen"/>`

### tf: **map -> odom -> baselink -> velodyne**

HW3/figure8_tracking: 
| Column 1 | Column 2 | Column 3 | Column 4 | Column 5 |
|:--------| :---------:|--------:|--------:|:--------:|
| Topic: motive_tracking/tf | Type: geometry_msgs/msg/TransformStamped | Count: 4797 | Serialization Format: cdr   | world to marker |
| Topic: motive_tracking/pose | Type: geometry_msgs/msg/PoseStamped | Count: 4803 | Serialization Format: cdr   | marker |

HW3/figure8_sensor: 
| Column 1 | Column 2 | Column 3 | Column 4 | Column 5 |
|:--------| :---------:|--------:|--------:|--------:|
| Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 2942 | Serialization Format: cdr  |  imu_link |
| Topic: /imu/data | Type: sensor_msgs/msg/Imu | Count: 2942 | Serialization Format: cdr      |  imu_link |
| Topic: /feedback | Type: jackal_msgs/msg/Feedback | Count: 773 | Serialization Format: cdr  |  base_link |
| Topic: /velodyne_points | Type: sensor_msgs/msg/PointCloud2 | Count: 390 | Serialization Format: cdr | velodyne |
| Topic: /status | Type: jackal_msgs/msg/Status | Count: 39 | Serialization Format: cdr |   no |

### /pointcloud_to_laserscan：

Subscribers:

| Column 1           | Column 2                        | Column 3  |
|:-------------------|:-------------------------------:|---------:|
| /parameter_events: | rcl_interfaces/msg/ParameterEvent|          |
| /velodyne_points:  | sensor_msgs/msg/PointCloud2     | velodyne |

Publishers:

| Column 1           | Column 2                        | Column 3  |
|:-------------------|:-------------------------------:|---------:|
| /parameter_events: | rcl_interfaces/msg/ParameterEvent|          |
| /rosout:           | rcl_interfaces/msg/Log          |          |
| /scanner/scan:     | sensor_msgs/msg/LaserScan       | velodyne |

### /slam_gmapping

Subscribers:

| Column 1 | Column 2 | Column 3 |
|:--------| :---------:|--------:| 
| /clock: | rosgraph_msgs/msg/Clock |   |
| /parameter_events: | rcl_interfaces/msg/ParameterEvent |  |
| /scanner/scan: | sensor_msgs/msg/LaserScan | velodyne  |

Publishers:

| Column 1 | Column 2 | Column 3 |
|:--------| :---------:|--------:|
| /entropy: | std_msgs/msg/Float64 |   |
| /map: | nav_msgs/msg/OccupancyGrid |     |
| /map_metadata: | nav_msgs/msg/MapMetaData |      |
| /parameter_events: | rcl_interfaces/msg/ParameterEvent |     |
| /rosout: | rcl_interfaces/msg/Log |      |
| /tf: | tf2_msgs/msg/TFMessage |      |

### parameters
    Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/velodyne_points'),
                    ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])],
        parameters=[{
            'target_frame': 'velodyne',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.1416,  # -M_PI/2
            'angle_max': 3.1416,  # M_PI/2
            'angle_increment': 0.0174532922,  # M_PI/360.0
            'scan_time': 0.03333,
            'range_min': 0.0,
            'range_max': 200.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': LaunchConfiguration('use_sim_time')  # 添加 use_sim_time 参数
        }],
        name='pointcloud_to_laserscan'
    )

    void SlamGmapping::init() {
        gsp_ = new GMapping::GridSlamProcessor();

        gsp_laser_ = nullptr;
        gsp_odom_ = nullptr;
        got_first_scan_ = false;
        got_map_ = false;

        throttle_scans_ = 1;
        base_frame_ = "base_link";
        map_frame_ = "map";
        odom_frame_ = "odom";
        transform_publish_period_ = 0.05;

        map_update_interval_ = tf2::durationFromSec(0.001);
        maxUrange_ = 80.0;  maxRange_ = 0.0;
        minimum_score_ = 0;
        sigma_ = 0.05;
        kernelSize_ = 1;
        lstep_ = 0.05;
        astep_ = 0.05;
        iterations_ = 5;
        lsigma_ = 0.075;
        ogain_ = 3.0;
        lskip_ = 0;
        srr_ = 0.1;
        srt_ = 0.2;
        str_ = 0.1;
        stt_ = 0.2;
        linearUpdate_ = 0.05;
        angularUpdate_ = 0.025;
        temporalUpdate_ = 0.5;
        resampleThreshold_ = 0.3;
        particles_ = 50;
        xmin_ = -5.0;
        ymin_ = -5.0;
        xmax_ = 5.0;
        ymax_ = 5.0;
        delta_ = 0.05;
        occ_thresh_ = 0.25;
        llsamplerange_ = 0.01;
        llsamplestep_ = 0.01;
        lasamplerange_ = 0.005;
        lasamplestep_ = 0.005;
        tf_delay_ = transform_publish_period_;
    }
