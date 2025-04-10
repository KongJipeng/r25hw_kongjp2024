// Your name etc. goes here!

#include <chrono>
#include <functional>
#include <memory>
#include <cstdio>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "jackal_msgs/msg/feedback.hpp"
#include "mobile_robotics_interfaces/msg/pose2_d_stamped.hpp"
#include "mobile_robotics_interfaces/msg/transform2_d_stamped.hpp"
#include "mobile_robotics_interfaces/msg/speed_stamped.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Pose : public rclcpp::Node
{
  public:
    Pose()
    : Node("pose"), count_(0), global_x_(0.0), global_y_(0.0), global_theta_(0.0)
    {
      subscription_ = this->create_subscription<mobile_robotics_interfaces::msg::Transform2DStamped>("odom", 1000, std::bind(&Pose::pose_callback, this, _1));
      pose_publisher_ = this->create_publisher<mobile_robotics_interfaces::msg::Pose2DStamped>("pose", 1000);
      speed_publisher_ = this->create_publisher<mobile_robotics_interfaces::msg::SpeedStamped>("speed", 1000);
    }

  private:
    size_t count_;
    double global_x_;
    double prev_x_;
    double global_y_;
    double prev_y_;
    double global_theta_;
    rclcpp::Time prev_time_;
    rclcpp::Subscription<mobile_robotics_interfaces::msg::Transform2DStamped>::SharedPtr subscription_;
    rclcpp::Publisher<mobile_robotics_interfaces::msg::Pose2DStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<mobile_robotics_interfaces::msg::SpeedStamped>::SharedPtr speed_publisher_;

    void pose_callback(const mobile_robotics_interfaces::msg::Transform2DStamped & odom_msg)
    {
      auto pose_message = mobile_robotics_interfaces::msg::Pose2DStamped();
      auto speed_message = mobile_robotics_interfaces::msg::SpeedStamped();

      // Get the current time
      rclcpp::Time current_time = this->get_clock()->now();

      // Fill in the pose message
      pose_message.header = odom_msg.header;
      pose_message.x = odom_msg.x;
      pose_message.y = odom_msg.y;
      pose_message.theta = odom_msg.theta;
      
      pose_publisher_->publish(pose_message);
      // RCLCPP_INFO(this->get_logger(), "Odometry: x=%f, y=%f, theta=%f", pose_message.x, pose_message.y, pose_message.theta);

      // Calculate the speed
      if (prev_time_.nanoseconds() != 0) {
        double dt = (current_time - prev_time_).seconds();
        double dx = odom_msg.x - prev_x_;
        double dy = odom_msg.y - prev_y_;
        double linear_speed = sqrt(dx * dx + dy * dy) / dt;

        // Fill in the speed message
        speed_message.header = odom_msg.header;
        speed_message.speed = linear_speed;

        speed_publisher_->publish(speed_message);
        // RCLCPP_INFO(this->get_logger(), "Speed: linear=%f", speed_message.speed);
      }
      // Update the previous values
      prev_time_ = current_time;
      prev_x_ = global_x_;
      prev_y_ = global_y_;

    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pose>());
  rclcpp::shutdown();
  return 0;
}