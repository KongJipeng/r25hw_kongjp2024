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

    void motor_callback(const jackal_msgs::msg::Feedback & jackal_feedback_msg) const
    {
      // you may comment the next line...
      RCLCPP_INFO(this->get_logger(), "The wheel 0 is at angle: %s rad", std::to_string(jackal_feedback_msg.drivers[0].measured_travel).c_str());
      
      auto odom_message = mobile_robotics_interfaces::msg::Transform2DStamped();
      
      // Step 5: Calculate Odometry
      // Calculate the odometry estimate using the motor values in the feedback message 
      // and set the values in odom_message accordingly.
      // Be sure to set the header correctly (stamp only, since each transform is relative to the last one).

      // Do something to calculate the odometry...

      odom_publisher_->publish(odom_message);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}