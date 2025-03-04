# The header provides a time stamp and a reference frame (frame_id)
std_msgs/Header header
# This provides x, y and theta
float64 x
float64 y
float64 theta

# The 2 message types contain identical field. But you should be aware of the difference between transform and pose.
# A 2D transform is a 2D translation and rotation, while a 2D pose is a 2D position and orientation.