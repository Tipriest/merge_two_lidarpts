#include <Eigen/Dense>
#include <cmath>
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
  LidarPointCloudMerger() {
    // Initialize subscribers for the front and back LiDARs
    sub_front = node.subscribe("/rslidar_points_front", 10,
                               &LidarPointCloudMerger::frontCallback, this);
    sub_back = node.subscribe("/rslidar_points_back", 10,
                              &LidarPointCloudMerger::backCallback, this);

    // Initialize publisher for the merged point cloud
    pub_merged =
        node.advertise<sensor_msgs::PointCloud2>("/rslidar_points_merged", 10);

    // Set up homogeneous transformation matrices (to be filled by the user)

    transform_front_to_base =
        trans_from_lidar_to_base(0.776, 0.0, 0.12, 0.0, 1.5707963267949, 0.0)
            .getTranmissionLidarToBase();
    std::cout << "front lidar to base_link's transformission"
              << transform_front_to_base << std::endl;
    transform_back_to_base =
        trans_from_lidar_to_base(-0.776, 0.0, 0.12, 3.1415926535,
                                 1.5707963267949, 0)
            .getTranmissionLidarToBase();
    std::cout << "back lidar to base_link's transformission"
              << transform_back_to_base << std::endl;
  }

  void frontCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    PointCloudXYZI cloud;
    pcl::fromROSMsg(*msg, cloud);
    pcl::transformPointCloud(cloud, cloud, transform_front_to_base);
    // store the latest front cloud
    front_cloud = cloud;
    mergeAndPublish();
  }

  void backCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    PointCloudXYZI cloud;
    pcl::fromROSMsg(*msg, cloud);
    pcl::transformPointCloud(cloud, back_cloud, transform_back_to_base);
    mergeAndPublish();
  }
  void mergeAndPublish() {
    if (front_cloud.empty() || back_cloud.empty()) {
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
    output_msg.header.frame_id = "base_link"; // Adjust frame ID as needed
    pub_merged.publish(output_msg);
  }
  PointCloudXYZI front_cloud;
  PointCloudXYZI back_cloud;

private:
  ros::NodeHandle node;
  ros::Subscriber sub_front;
  ros::Subscriber sub_back;
  ros::Publisher pub_merged;

  PointCloudXYZI front_cloud_queue;
  PointCloudXYZI back_cloud;

  Eigen::Matrix4f transform_front_to_base;
  Eigen::Matrix4f transform_back_to_base;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "lidar_pointcloud_merger");

  LidarPointCloudMerger merger;

  ros::spin();

  return 0;
}
