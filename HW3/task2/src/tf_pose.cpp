#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TfPoseNode : public rclcpp::Node
{
public:
    TfPoseNode()
        : Node("tf_pose_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // 创建发布器
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("map_to_base_link_pose", 10);

        // 创建定时器，每隔 100ms 获取一次 TF 并发布 Pose
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&TfPoseNode::publishPose, this));
    }

private:
    void publishPose()
    {
        try
        {
            // 获取 map 到 base_link 的变换
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

            // 将 Transform 转换为 PoseStamped
            geometry_msgs::msg::PoseStamped pose;
            pose.header = transform_stamped.header;
            pose.pose.position.x = transform_stamped.transform.translation.x;
            pose.pose.position.y = transform_stamped.transform.translation.y;
            pose.pose.position.z = transform_stamped.transform.translation.z;
            pose.pose.orientation = transform_stamped.transform.rotation;

            // 发布 Pose
            pose_publisher_->publish(pose);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
        }
    }

    // 成员变量
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfPoseNode>());
    rclcpp::shutdown();
    return 0;
}