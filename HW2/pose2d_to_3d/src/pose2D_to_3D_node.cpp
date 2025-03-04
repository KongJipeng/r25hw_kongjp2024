// Your name etc. goes here!

#include <chrono>
#include <functional>
#include <memory>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "mobile_robotics_interfaces/msg/pose2_d_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Pose2t3 : public rclcpp::Node
{
  public:
    Pose2t3()
    : Node("pose2d_to_3d"), count_(0)
    {
      path_message_ = std::make_shared<nav_msgs::msg::Path>();
      subscription_ = this->create_subscription<mobile_robotics_interfaces::msg::Pose2DStamped>("pose", 1000, std::bind(&Pose2t3::pose_callback, this, _1));
      pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose3D", 1000);
      path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path3D", 1000);
    }

  private:
    size_t count_;
    nav_msgs::msg::Path::SharedPtr path_message_;
    rclcpp::Subscription<mobile_robotics_interfaces::msg::Pose2DStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    void pose_callback(const mobile_robotics_interfaces::msg::Pose2DStamped & pose2d_msg) const
    {
      auto pose_message = geometry_msgs::msg::PoseStamped();

      pose_message.header = pose2d_msg.header;
      pose_message.pose.position.x = pose2d_msg.x;
      pose_message.pose.position.y = pose2d_msg.y;
      pose_message.pose.position.z = 0.0;

      tf2::Quaternion tf2_quat;
      tf2_quat.setRPY(0.0, 0.0, pose2d_msg.theta);

      pose_message.pose.orientation = tf2::toMsg(tf2_quat);

      path_message_->header = pose_message.header;
      path_message_->poses.push_back(pose_message);

      pose_publisher_->publish(pose_message);
      path_publisher_->publish(*path_message_);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pose2t3>());
  rclcpp::shutdown();
  return 0;
}