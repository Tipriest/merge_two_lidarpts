#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
struct trans_from_lidar_to_base {
  //这些参数x, y, z等是lidar到base的
  trans_from_lidar_to_base(double _x, double _y, double _z, double _roll,
                           double _pitch, double _yaw) {
    x = _x;
    y = _y;
    z = _z;
    roll = _roll;
    pitch = _pitch;
    yaw = _yaw;
  }
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;

  Eigen::Matrix4f getTranmissionLidarToBase() {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // Rotation matrices for roll, pitch, and yaw
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    // Set rotation part
    transform.block<3, 3>(0, 0) = R;

    // Set translation part
    transform(0, 3) = x;
    transform(1, 3) = y;
    transform(2, 3) = z;

    return transform;
  }
  Eigen::Matrix4f getTranmissionBaseToLidar() {
    Eigen::Matrix4f transform = getTranmissionLidarToBase();

    // Invert the transformation matrix
    Eigen::Matrix3f R = transform.block<3, 3>(0, 0);
    Eigen::Vector3f t = transform.block<3, 1>(0, 3);

    Eigen::Matrix4f inverseTransform = Eigen::Matrix4f::Identity();
    inverseTransform.block<3, 3>(0, 0) = R.transpose();
    inverseTransform.block<3, 1>(0, 3) = -R.transpose() * t;

    return inverseTransform;
  }
};
class LidarPointCloudMerger {
public:
  LidarPointCloudMerger(ros::NodeHandle &nh) {
    node = nh;
    // Initialize subscribers for the front and back LiDARs
    sub_front = node.subscribe(front_lidar_topic, 10,
                               &LidarPointCloudMerger::frontCallback, this);
    sub_back = node.subscribe(back_lidar_topic, 10,
                              &LidarPointCloudMerger::backCallback, this);

    // 确保正确读取参数，并打印日志以验证
    node.param<double>("front_x", front_x, 0.0);
    ROS_INFO("Loaded parameter front_x: %f", front_x);
    node.param<double>("front_y", front_y, 0.0);
    ROS_INFO("Loaded parameter front_y: %f", front_y);
    node.param<double>("front_z", front_z, 0.0);
    ROS_INFO("Loaded parameter front_z: %f", front_z);
    node.param<double>("front_roll", front_roll, 0.0);
    ROS_INFO("Loaded parameter front_roll: %f", front_roll);
    node.param<double>("front_pitch", front_pitch, 0.0);
    ROS_INFO("Loaded parameter front_pitch: %f", front_pitch);
    node.param<double>("front_yaw", front_yaw, 0.0);
    ROS_INFO("Loaded parameter front_yaw: %f", front_yaw);

    node.param<double>("back_x", back_x, 0.0);
    ROS_INFO("Loaded parameter back_x: %f", back_x);
    node.param<double>("back_y", back_y, 0.0);
    ROS_INFO("Loaded parameter back_y: %f", back_y);
    node.param<double>("back_z", back_z, 0.0);
    ROS_INFO("Loaded parameter back_z: %f", back_z);
    node.param<double>("back_roll", back_roll, 0.0);
    ROS_INFO("Loaded parameter back_roll: %f", back_roll);
    node.param<double>("back_pitch", back_pitch, 0.0);
    ROS_INFO("Loaded parameter back_pitch: %f", back_pitch);
    node.param<double>("back_yaw", back_yaw, 0.0);
    ROS_INFO("Loaded parameter back_yaw: %f", back_yaw);
    node.param<std::string>("aim_frame_id", aim_frame_id, "ttt_err_check");
    std::cout << "Loaded parameter aim_frame_id: " << aim_frame_id << std::endl;
    node.param<std::string>("front_lidar_topic", front_lidar_topic,
                            "/rslidar_points_front");
    std::cout << "Loaded parameter front_lidar_topic: " << front_lidar_topic
              << std::endl;
    node.param<std::string>("back_lidar_topic", back_lidar_topic,
                            "/rslidar_points_back");
    std::cout << "Loaded parameter back_lidar_topic: " << back_lidar_topic
              << std::endl;
    node.param<std::string>("merged_lidar_topic", merged_lidar_topic,
                            "/rslidar_points_merged");
    std::cout << "Loaded parameter merged_lidar_topic: " << merged_lidar_topic
              << std::endl;
    // Initialize publisher for the merged point cloud
    pub_merged =
        node.advertise<sensor_msgs::PointCloud2>(merged_lidar_topic, 10);

    // Set up homogeneous transformation matrices (to be filled by the user)

    transform_front_to_base =
        trans_from_lidar_to_base(front_x, front_y, front_z, front_roll,
                                 front_pitch, front_yaw)
            .getTranmissionLidarToBase();
    std::cout << "front lidar to base_link's transformission" << std::endl
              << transform_front_to_base << std::endl;
    transform_back_to_base =
        trans_from_lidar_to_base(back_x, back_y, back_z, back_roll, back_pitch,
                                 back_yaw)
            .getTranmissionLidarToBase();
    std::cout << "back lidar to base_link's transformission" << std::endl
              << transform_back_to_base << std::endl;
  }

  void frontCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // std::cout << "frontCallback" << std::endl;
    PointCloudXYZI cloud;
    pcl::fromROSMsg(*msg, cloud);
    pcl::transformPointCloud(cloud, cloud, transform_front_to_base);
    // store the latest front cloud
    front_cloud = cloud;
    front_cloud_updated = true;
    mergeAndPublish();
  }

  void backCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // std::cout << "backCallback" << std::endl;
    PointCloudXYZI cloud;
    pcl::fromROSMsg(*msg, cloud);
    pcl::transformPointCloud(cloud, back_cloud, transform_back_to_base);
    back_cloud_updated = true;
    mergeAndPublish();
  }
  void mergeAndPublish() {
    // std::cout << "mergeAndPublish" << std::endl;
    if (!front_cloud_updated || !back_cloud_updated) {
      return; // Wait until both clouds are available
    }

    // Start with the front cloud and append back cloud points
    PointCloudXYZI merged_cloud = front_cloud;
    merged_cloud.points.insert(merged_cloud.points.end(),
                               back_cloud.points.begin(),
                               back_cloud.points.end());
    merged_cloud.width = static_cast<uint32_t>(merged_cloud.points.size());
    merged_cloud.height = 1;
    merged_cloud.is_dense = front_cloud.is_dense && back_cloud.is_dense;

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(merged_cloud, output_msg);
    output_msg.header.stamp = ros::Time::now();
    output_msg.header.frame_id = aim_frame_id; // Adjust frame ID as needed
    pub_merged.publish(output_msg);
    front_cloud_updated = false;
    back_cloud_updated = false;
  }

private:
  ros::NodeHandle node;
  ros::Subscriber sub_front;
  ros::Subscriber sub_back;
  ros::Publisher pub_merged;

  PointCloudXYZI front_cloud;
  PointCloudXYZI back_cloud;
  bool front_cloud_updated = false;
  bool back_cloud_updated = false;

  Eigen::Matrix4f transform_front_to_base;
  Eigen::Matrix4f transform_back_to_base;

  // calibration params
  double front_x = 0.0;
  double front_y = 0.0;
  double front_z = 0.0;
  double front_roll = 0.0;
  double front_pitch = 0.0;
  double front_yaw = 0.0;
  double back_x = 0.0;
  double back_y = 0.0;
  double back_z = 0.0;
  double back_roll = 0.0;
  double back_pitch = 0.0;
  double back_yaw = 0.0;
  std::string aim_frame_id = "";
  std::string front_lidar_topic = "";
  std::string back_lidar_topic = "";
  std::string merged_lidar_topic = "";
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "lidar_pointcloud_merger");
  ros::NodeHandle nh("~");
  LidarPointCloudMerger merger(nh);

  ros::spin();

  return 0;
}
