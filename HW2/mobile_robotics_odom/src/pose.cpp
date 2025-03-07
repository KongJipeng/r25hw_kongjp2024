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
using namespace std::chrono_literals;
using std::placeholders::_1;

class Pose : public rclcpp::Node
{
  public:
    Pose()
    : Node("pose"), count_(0), global_x_(0.0), global_y_(0.0), global_theta_(0.0)
    {
      subscription_ = this->create_subscription<mobile_robotics_interfaces::msg::Transform2DStamped>("odom", 1000, std::bind(&Pose::motor_callback, this, _1));
      pose_publisher_ = this->create_publisher<mobile_robotics_interfaces::msg::Pose2DStamped>("pose", 1000);
    }

  private:
    size_t count_;
    double global_x_;
    double global_y_;
    double global_theta_;
    rclcpp::Subscription<mobile_robotics_interfaces::msg::Transform2DStamped>::SharedPtr subscription_;
    rclcpp::Publisher<mobile_robotics_interfaces::msg::Pose2DStamped>::SharedPtr pose_publisher_;

    void motor_callback(const mobile_robotics_interfaces::msg::Transform2DStamped & odom_msg)
    {
      auto pose_message = mobile_robotics_interfaces::msg::Pose2DStamped();

      // 更新全局坐标 - 考虑机器人的方向
      // 首先将增量从局部坐标系转换到全局坐标系
      double delta_x_global = odom_msg.x * cos(global_theta_) - odom_msg.y * sin(global_theta_);
      double delta_y_global = odom_msg.x * sin(global_theta_) + odom_msg.y * cos(global_theta_);
      
      // 更新全局坐标
      global_x_ += delta_x_global;
      global_y_ += delta_y_global;
      global_theta_ += odom_msg.theta;
      
      // 归一化角度到 [-π, π]
      while (global_theta_ > M_PI) global_theta_ -= 2.0 * M_PI;
      while (global_theta_ < -M_PI) global_theta_ += 2.0 * M_PI;
      
      // 将全局坐标填充到消息中
      pose_message.header = odom_msg.header;
      pose_message.x = global_x_;
      pose_message.y = global_y_;
      pose_message.theta = global_theta_;
      
      pose_publisher_->publish(pose_message);
      RCLCPP_INFO(this->get_logger(), "Odometry: x=%f, y=%f, theta=%f", pose_message.x, pose_message.y, pose_message.theta);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pose>());
  rclcpp::shutdown();
  return 0;
}