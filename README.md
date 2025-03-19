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
take one day...

### task1:
Dependencies:
- `pip install evo`
- `pip install numpy=1.23.5` //scipy conflict
- `sudo apt remove python3-matplotlib`    //multi matplotlib conflict

`evo_traj bag topic --save_as tum` to convert bag to tum.

`evo_traj tum tum1 tum2 --align --plot_mode xz` to compare on one plane 

### task1:
wrong:
 `<node pkg="rosbag2" exec="play" name="rosbag_player" output="screen" args="$(var bag_file) " />`

correct:
 `<executable cmd="ros2 bag play $(var bag_file)" output="screen"/>`