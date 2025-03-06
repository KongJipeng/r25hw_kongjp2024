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
    }

  private:
    size_t count_;
    rclcpp::Subscription<jackal_msgs::msg::Feedback>::SharedPtr subscription_;
    rclcpp::Publisher<mobile_robotics_interfaces::msg::Transform2DStamped>::SharedPtr odom_publisher_;

    // You may declare global variables here...
    double wheel_radius = 0.098;  // radius of the wheel, in meters
    double wheel_base = 0.37559;  // distance between the wheels and the center, in meters
    double wheel_separation_multiplier = 1.5; // multiplier for the wheel separation
    double effective_wheel_base; // effective wheel base, in meters

    // State variables
    double prev_left_wheel_pos_;
    double prev_right_wheel_pos_;
    double x_, y_, theta_;
    bool first_msg_;

    void motor_callback(const jackal_msgs::msg::Feedback & jackal_feedback_msg) 
    {
      // you may comment the next line...
      RCLCPP_INFO(this->get_logger(), "The wheel 0 is at angle: %s rad", std::to_string(jackal_feedback_msg.drivers[0].measured_travel).c_str());
      RCLCPP_INFO(this->get_logger(), "The wheel 1 is at angle: %s rad", std::to_string(jackal_feedback_msg.drivers[1].measured_travel).c_str());
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
        return;
      }
      // 计算轮子转动的角度变化
      double delta_left = left_wheel_pos - prev_left_wheel_pos_;
      double delta_right = right_wheel_pos - prev_right_wheel_pos_;
      
      // 计算左右轮的线性位移
      double left_dist = delta_left * wheel_radius;
      double right_dist = delta_right * wheel_radius;
      
      // 计算机器人的线性和角速度
      double linear_dist = (right_dist + left_dist) / 2.0;
      double angular_dist = (right_dist - left_dist) / effective_wheel_base;

      // 更新机器人在全局坐标系下的位置和方向
      double delta_x, delta_y;
      if (fabs(angular_dist) < 1e-6) {
        // 如果机器人几乎是直线运动
        delta_x = linear_dist * cos(theta_);
        delta_y = linear_dist * sin(theta_);
      } else {
        // 如果机器人做曲线运动
        double r = linear_dist / angular_dist;
        delta_x = r * (sin(theta_ + angular_dist) - sin(theta_));
        delta_y = r * (cos(theta_) - cos(theta_ + angular_dist));
      }
      
      // 更新姿态
      double delta_theta = angular_dist;
      
      // 创建并填充里程计消息
      odom_message.header = jackal_feedback_msg.header;
      odom_message.x = delta_x;
      odom_message.y = delta_y;
      odom_message.theta = delta_theta;

      // 更新全局位置和方向（用于下一次计算）
      x_ += delta_x;
      y_ += delta_y;
      theta_ += delta_theta;
      
      // 归一化角度到[-π, π]
      while (theta_ > M_PI) theta_ -= 2 * M_PI;
      while (theta_ < -M_PI) theta_ += 2 * M_PI;
      
      // 更新上一次的轮子位置和时间
      prev_left_wheel_pos_ = left_wheel_pos;
      prev_right_wheel_pos_ = right_wheel_pos;

      odom_publisher_->publish(odom_message);
      RCLCPP_INFO(this->get_logger(), "Odometry: dx=%f, dy=%f, dtheta=%f", delta_x, delta_y, delta_theta);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}