// Your name etc. goes here!

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

class Pose : public rclcpp::Node
{
  public:
    Pose()
    : Node("pose"), count_(0)
    {
      subscription_ = this->create_subscription<jackal_msgs::msg::Feedback>("feedback", 1000, std::bind(&Pose::motor_callback, this, _1));
      pose_publisher_ = this->create_publisher<mobile_robotics_interfaces::msg::Pose2DStamped>("pose", 1000);
    }

  private:
    size_t count_;
    rclcpp::Subscription<jackal_msgs::msg::Feedback>::SharedPtr subscription_;
    rclcpp::Publisher<mobile_robotics_interfaces::msg::Pose2DStamped>::SharedPtr pose_publisher_;

    void motor_callback(const jackal_msgs::msg::Feedback & jackal_feedback_msg)
    {
      RCLCPP_INFO(this->get_logger(), "The wheel 0 is at angle: %s rad", std::to_string(jackal_feedback_msg.drivers[0].measured_travel).c_str());
      RCLCPP_INFO(this->get_logger(), "The wheel 1 is at angle: %s rad", std::to_string(jackal_feedback_msg.drivers[1].measured_travel).c_str());
      auto pose_message = mobile_robotics_interfaces::msg::Pose2DStamped();
      
      pose_publisher_->publish(pose_message);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pose>());
  rclcpp::shutdown();
  return 0;
}