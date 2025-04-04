# Readme for Robotics Homework 2

Create a new ros workspace.
Get the provided files (you obviously did :) )
Compile the homework using `colcon build` in the root of your workspace, e.g:

```bash
pushd `pwd`; cd ~/hw2_ws; colcon build; popd
```

Run the homework:

`ros2 run mobile_robotics_odom odometry`

Play the bagfile.

You may listen to the published result using rostopic echo:

`ros2 topic echo /odom`

Play the feedback bagfile - that publishes feedback on /feedback

For that make sure you use the simulated time from the rosbag (and not the current time). So add this line of magic command when you start nodes (including rviz):

`use_sim_time:=true`

For example: `ros2 run mobile_robotics_odom odometry use_sim_time:=true`

Also play back with the time from the bagfile

`ros2 bag play figure8_feedback --clock-topics-all`

Record the published pose:

`ros2 bag record -o odom --use-sim-time /odom`

in which `-o odom` specifies the output file name, `--use-sim-time` specifies to use the simulated time, `/odom` is the topic we wish to record.

Do the same thing for the pose and speed.

Once you are finished commit your bagfles and code to the hw2 folder in git and push to the gitlab server.
