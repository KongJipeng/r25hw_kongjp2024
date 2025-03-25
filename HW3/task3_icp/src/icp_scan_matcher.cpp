#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "pcl_conversions/pcl_conversions.h"

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std::chrono_literals;

class IcpScanMatcher : public rclcpp::Node
{
public:
  IcpScanMatcher()
  : Node("icp_scan_matcher")
  {
    // 声明参数
    this->declare_parameter("voxel_leaf_size", 0.05);
    this->declare_parameter("max_correspondence_distance", 0.1);
    this->declare_parameter("max_iterations", 50);
    this->declare_parameter("transformation_epsilon", 1e-8);
    this->declare_parameter("euclidean_fitness_epsilon", 1.0);
    this->declare_parameter("use_constant_velocity", true);
    this->declare_parameter("min_height", -0.5);
    this->declare_parameter("max_height", 1.0);
    this->declare_parameter("output_file", "icp_trajectory.txt");

    // 获取参数值
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();
    max_iterations_ = this->get_parameter("max_iterations").as_int();
    transformation_epsilon_ = this->get_parameter("transformation_epsilon").as_double();
    euclidean_fitness_epsilon_ = this->get_parameter("euclidean_fitness_epsilon").as_double();
    use_constant_velocity_ = this->get_parameter("use_constant_velocity").as_bool();
    min_height_ = this->get_parameter("min_height").as_double();
    max_height_ = this->get_parameter("max_height").as_double();
    output_file_ = this->get_parameter("output_file").as_string();

    // 创建发布者
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("icp_pose", 10);
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("icp_path", 10);
    processed_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_cloud", 10);
    
    // 创建订阅者
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 10, std::bind(&IcpScanMatcher::pointcloud_callback, this, std::placeholders::_1));

    // 初始化TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // 初始化路径
    path_.header.frame_id = "map";

    // 打开输出文件
    output_file_stream_.open(output_file_);
    if (!output_file_stream_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", output_file_.c_str());
    } else {
      output_file_stream_ << "# timestamp tx ty tz qx qy qz qw" << std::endl;
    }

    RCLCPP_INFO(this->get_logger(), "ICP Scan Matcher initialized");
  }

  ~IcpScanMatcher()
  {
    if (output_file_stream_.is_open()) {
      output_file_stream_.close();
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_publisher_;
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  nav_msgs::msg::Path path_;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
  Eigen::Matrix4f previous_transform_;
  Eigen::Matrix4f velocity_;
  
  bool first_scan_;
  double voxel_leaf_size_;
  double max_correspondence_distance_;
  int max_iterations_;
  double transformation_epsilon_;
  double euclidean_fitness_epsilon_;
  bool use_constant_velocity_;
  double min_height_;
  double max_height_;
  std::string output_file_;
  std::ofstream output_file_stream_;

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received point cloud");
    
    // 将ROS消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_cloud);
    
    // 预处理点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud = preprocess_cloud(input_cloud);
    
    // 发布处理后的点云（用于可视化）
    sensor_msgs::msg::PointCloud2 processed_cloud_msg;
    pcl::toROSMsg(*processed_cloud, processed_cloud_msg);
    processed_cloud_msg.header = msg->header;
    processed_cloud_publisher_->publish(processed_cloud_msg);
    
    // 执行ICP
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
    
    if (!first_scan_)
    {
      // 首次扫描
      previous_cloud_ = processed_cloud;
      previous_transform_ = Eigen::Matrix4f::Identity();
      velocity_ = Eigen::Matrix4f::Identity();
      first_scan_ = true;
    }
    else
    {
      // 使用恒速模型作为初始猜测
      Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
      if (use_constant_velocity_)
      {
        initial_guess = previous_transform_ * velocity_;
      }
      
      // 执行ICP
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setInputSource(processed_cloud);
      icp.setInputTarget(previous_cloud_);
      icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
      icp.setMaximumIterations(max_iterations_);
      icp.setTransformationEpsilon(transformation_epsilon_);
      icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
      
      // 设置初始对齐猜测
      if (use_constant_velocity_)
      {
        icp.align(*processed_cloud, initial_guess);
      }
      else
      {
        icp.align(*processed_cloud);
      }
      
      if (icp.hasConverged())
      {
        RCLCPP_INFO(this->get_logger(), "ICP has converged. Score: %f", icp.getFitnessScore());
        transformation_matrix = icp.getFinalTransformation();
        
        // 更新速度
        velocity_ = previous_transform_.inverse() * transformation_matrix;
        
        // 更新先前的变换
        previous_transform_ = transformation_matrix;
        
        // 更新先前的点云
        previous_cloud_ = processed_cloud;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
        return;
      }
    }
    
    // 从变换矩阵中提取位置和方向
    Eigen::Vector3f translation = transformation_matrix.block<3, 1>(0, 3);
    Eigen::Matrix3f rotation = transformation_matrix.block<3, 3>(0, 0);
    
    // 将旋转矩阵转换为四元数
    Eigen::Quaternionf quat(rotation);
    
    // 创建位姿消息
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = "map";
    
    pose.pose.position.x = translation[0];
    pose.pose.position.y = translation[1];
    pose.pose.position.z = translation[2];
    
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();
    
    // 发布位姿
    pose_publisher_->publish(pose);
    
    // 更新和发布路径
    path_.header = pose.header;
    path_.poses.push_back(pose);
    path_publisher_->publish(path_);
    
    // 广播TF
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header = pose.header;
    transform_stamped.child_frame_id = "velodyne";
    
    transform_stamped.transform.translation.x = translation[0];
    transform_stamped.transform.translation.y = translation[1];
    transform_stamped.transform.translation.z = translation[2];
    
    transform_stamped.transform.rotation = pose.pose.orientation;
    
    tf_broadcaster_->sendTransform(transform_stamped);
    
    // 将数据写入输出文件
    if (output_file_stream_.is_open())
    {
      output_file_stream_ << msg->header.stamp.sec << "." << std::setfill('0') << std::setw(9) << msg->header.stamp.nanosec
                         << " " << translation[0] << " " << translation[1] << " " << translation[2]
                         << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
  {
    // 下采样点云
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_filter.filter(*filtered_cloud);
    
    // 高度过滤（移除地面和高处物体）
    pcl::PassThrough<pcl::PointXYZ> pass_filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pass_filter.setInputCloud(filtered_cloud);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(min_height_, max_height_);
    pass_filter.filter(*height_filtered_cloud);
    
    // 移除离群点
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_outliers(new pcl::PointCloud<pcl::PointXYZ>);
    outlier_filter.setInputCloud(height_filtered_cloud);
    outlier_filter.setMeanK(50);
    outlier_filter.setStddevMulThresh(1.0);
    outlier_filter.filter(*cloud_without_outliers);
    
    return cloud_without_outliers;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IcpScanMatcher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}