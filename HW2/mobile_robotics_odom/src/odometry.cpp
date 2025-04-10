// Your name etc. goes here!
/*
  Jipeng Kong
  2024234322
  kongjp2024@shanghaitech.edu.cn
*/

#include <chrono>
#include <functional>
#include <memory>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "jackal_msgs/msg/feedback.hpp"
#include "mobile_robotics_interfaces/msg/pose2_d_stamped.hpp"
#include "mobile_robotics_interfaces/msg/transform2_d_stamped.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Odometry : public rclcpp::Node
{
  public:
    Odometry()
    : Node("odometry"), count_(0)
    {
      subscription_ = this->create_subscription<jackal_msgs::msg::Feedback>("feedback", 1000, std::bind(&Odometry::motor_callback, this, _1));
      odom_publisher_ = this->create_publisher<mobile_robotics_interfaces::msg::Transform2DStamped>("odom", 1000);
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      
      // 初始化状态变量
      x_ = 0.0;
      y_ = 0.0;
      theta_ = 0.0;
      first_msg_ = true;  // 添加这一行，确保正确初始化first_msg_标志
    }

  private:
    size_t count_;
    rclcpp::Subscription<jackal_msgs::msg::Feedback>::SharedPtr subscription_;
    rclcpp::Publisher<mobile_robotics_interfaces::msg::Transform2DStamped>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // You may declare global variables here...
    double wheel_radius = 0.098;  // radius of the wheel, in meters
    double wheel_base = 0.37559;  // distance between the wheels and the center, in meters
    double wheel_separation_multiplier = 1.617; // multiplier for the wheel separation
    double effective_wheel_base; // effective wheel base, in meters

    // State variables
    double prev_left_wheel_pos_;
    double prev_right_wheel_pos_;
    double x_, y_, theta_;
    bool first_msg_;

    void motor_callback(const jackal_msgs::msg::Feedback & jackal_feedback_msg) 
    {
      // you may comment the next line...
      // RCLCPP_INFO(this->get_logger(), "The wheel 0 is at angle: %s rad", std::to_string(jackal_feedback_msg.drivers[0].measured_travel).c_str());
      // RCLCPP_INFO(this->get_logger(), "The wheel 1 is at angle: %s rad", std::to_string(jackal_feedback_msg.drivers[1].measured_travel).c_str());
      auto odom_message = mobile_robotics_interfaces::msg::Transform2DStamped();

      // Step 5: Calculate Odometry
      // Calculate the odometry estimate using the motor values in the feedback message
      // and set the values in odom_message accordingly.
      // Be sure to set the header correctly (stamp only, since each transform is relative to the last one).

      // Do something to calculate the odometry...

      // Get the current position of the left and right wheels
      double left_wheel_pos = jackal_feedback_msg.drivers[0].measured_travel;
      double right_wheel_pos = jackal_feedback_msg.drivers[1].measured_travel;
      // Calculate the effective wheel base
      effective_wheel_base = wheel_base * wheel_separation_multiplier;
      // First message initialization
      if (first_msg_) {
        prev_left_wheel_pos_ = left_wheel_pos;
        prev_right_wheel_pos_ = right_wheel_pos;
        first_msg_ = false;
        
        // 对于第一个消息，立即发布初始TF，确保坐标系对齐
        auto odom_message = mobile_robotics_interfaces::msg::Transform2DStamped();
        odom_message.header = jackal_feedback_msg.header;
        odom_message.header.frame_id = "odom";
        odom_message.x = x_;
        odom_message.y = y_;
        odom_message.theta = theta_;
        
        odom_publisher_->publish(odom_message);
        
        // 发布初始TF
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = odom_message.header.stamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_link";
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;
        
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(0.0, 0.0, 0.0);
        transform_stamped.transform.rotation = tf2::toMsg(tf2_quat);
        
        tf_broadcaster_->sendTransform(transform_stamped);
        return;
      }
      // Calculate the change in wheel position
      double delta_left = left_wheel_pos - prev_left_wheel_pos_;
      double delta_right = right_wheel_pos - prev_right_wheel_pos_;
      
      // Calculate the distance traveled by each wheel
      double left_dist = delta_left * wheel_radius;
      double right_dist = delta_right * wheel_radius;
      
      // Calculate the linear and angular distance traveled by the robot
      double linear_dist = (right_dist + left_dist) / 2.0;
      double angular_dist = (right_dist - left_dist) / effective_wheel_base;

      // Calculate the change in x and y
      double delta_x, delta_y;
      if (fabs(angular_dist) < 1e-6) {
        // If the robot is moving straight
        delta_y = linear_dist * cos(theta_);
        delta_x = linear_dist * sin(theta_);
      } else {
        // If the robot is turning
        double r = linear_dist / angular_dist;
        delta_x = r * (sin(theta_ + angular_dist) - sin(theta_));
        delta_y = r * (cos(theta_) - cos(theta_ + angular_dist));
      }
      
      double delta_theta = angular_dist;

      // Update the position of the robot
      x_ += delta_x;
      y_ += delta_y;
      theta_ += delta_theta;
      
      // Normalize the angle
      while (theta_ > M_PI) theta_ -= 2 * M_PI;
      while (theta_ < -M_PI) theta_ += 2 * M_PI;
      
      // Fill in the odometry message
      odom_message.header = jackal_feedback_msg.header;
      odom_message.header.frame_id = "odom";
      odom_message.x = x_;
      odom_message.y = y_;
      odom_message.theta = theta_;

      // Update the previous wheel positions
      prev_left_wheel_pos_ = left_wheel_pos;
      prev_right_wheel_pos_ = right_wheel_pos;

      odom_publisher_->publish(odom_message);
      // RCLCPP_INFO(this->get_logger(), "Odometry: dx=%f, dy=%f, dtheta=%f", delta_x, delta_y, delta_theta);

      // Publish tf
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = odom_message.header.stamp;
      transform_stamped.header.frame_id = "odom";
      transform_stamped.child_frame_id = "base_link";
      transform_stamped.transform.translation.x = odom_message.x;
      transform_stamped.transform.translation.y = odom_message.y;
      transform_stamped.transform.translation.z = 0.0;

      tf2::Quaternion tf2_quat;
      tf2_quat.setRPY(0.0, 0.0, odom_message.theta);
      transform_stamped.transform.rotation = tf2::toMsg(tf2_quat);

      tf_broadcaster_->sendTransform(transform_stamped);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}