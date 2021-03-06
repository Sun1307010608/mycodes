#include <iostream>
#include <string>

#include <ros/ros.h>  // ros::init, ros::NodeHandle, ros::Subscriber
#include <std_msgs/String.h>  // std_msgs::String
#include <sensor_msgs/PointCloud2.h>  // snesor_msgs::PointCloud2
#include <tf/transform_datatypes.h>  // tf::Matrix3x3
#include <tf/transform_broadcaster.h>  // tf::TransformBroadcaster

#include <pcl/io/io.h>  // pcl::PointCloud
#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <pcl/filters/filter.h>  // pcl::removeNaNFromPointCloud
#include <pcl/filters/voxel_grid.h>  // pcl::VoxelGrid
#include <pcl/registration/ndt.h>  // pcl::NormalDistributionsTransform

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;

  pose() :x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0){} //无参数的构造函数数组初始化时调用
};

// global variables
ros::Time current_scan_time;
ros::Time previous_scan_time;

pcl::PointCloud<pcl::PointXYZI> map;
pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

ros::Time callback_start, callback_end;
ros::Duration d_callback;

ros::Publisher ndt_map_pub;
ros::Publisher current_pose_pub;
geometry_msgs::PoseStamped current_pose_msg;

pose previous_pose, guess_pose, current_pose, added_pose;
double diff = 0.0, diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw = 0.0;  // current_pose - previous_pose
double current_velocity_x = 0.0, current_velocity_y = 0.0, current_velocity_z = 0.0;
double voxel_leaf_size = 1.5;  // Leaf size of VoxelGrid filter.
double min_add_scan_shift = 1.0;  // distance(m) to update map.
std::string points_topic = "/points_raw";


void output_callback(const std_msgs::String::ConstPtr& input)
{
  std::string save_pcd_name = input->data;
  // Writing Point Cloud data to PCD file
  pcl::io::savePCDFileASCII(save_pcd_name, map);
  std::cout << "Saved " << map.points.size() << " data points to " << save_pcd_name << "." << std::endl;
}


double wrapToPm(double a_num, const double a_max)
{
  if (a_num >= a_max)
  {
    a_num -= 2.0 * a_max;
  }
  return a_num;
}

double wrapToPmPi(double a_angle_rad)
{
  return wrapToPm(a_angle_rad, M_PI);
}

double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr points_ptr(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_points_ptr(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_points_ptr(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>());
void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  current_scan_time = input->header.stamp;
  pcl::fromROSMsg(*input, *points_ptr);
  //std::vector<int> indices;
  //pcl::removeNaNFromPointCloud(*points_ptr, *points_ptr, indices);

  // add initial point cloud to map: 
  static bool initial_scan_loaded = false;
  if (initial_scan_loaded == false)
  {
    map += *points_ptr;
    initial_scan_loaded = true;
  }

  // downsmaple the original points: 
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(points_ptr);
  voxel_grid_filter.filter(*filtered_points_ptr);

  // set ndt registration parameters: 
  ndt.setTransformationEpsilon(0.01);
  ndt.setStepSize(0.1);
  ndt.setResolution(1.0);
  ndt.setMaximumIterations(30);
  ndt.setInputSource(filtered_points_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  static bool is_first_map = true;
  if (is_first_map == true)
  {
    ndt.setInputTarget(map_ptr);
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(map, *map_msg_ptr);
    map_msg_ptr->header.frame_id = "map";
    ndt_map_pub.publish(*map_msg_ptr);
    is_first_map = false;
  }

  // calculate init guess pose: 
  guess_pose.x = previous_pose.x + diff_x;
  guess_pose.y = previous_pose.y + diff_y;
  guess_pose.z = previous_pose.z + diff_z;
  guess_pose.roll = previous_pose.roll;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.yaw = previous_pose.yaw + diff_yaw;
  Eigen::AngleAxisf init_rotation_x(guess_pose.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pose.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pose.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(guess_pose.x, guess_pose.y, guess_pose.z);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

  // point cloud registration and get align parameters: 
  ndt.align(*output_cloud, init_guess);
  double fitness_score = ndt.getFitnessScore();
  double transformation_probability = ndt.getTransformationProbability();
  Eigen::Matrix4f t_localizer = ndt.getFinalTransformation();
  pcl::transformPointCloud(*points_ptr, *transformed_points_ptr, t_localizer);

  // Update localizer_pose: 
  tf::Matrix3x3 mat_l;
  mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)), static_cast<double>(t_localizer(0, 2)), 
                 static_cast<double>(t_localizer(1, 0)), static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                 static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)), static_cast<double>(t_localizer(2, 2)));
  current_pose.x = t_localizer(0, 3);
  current_pose.y = t_localizer(1, 3);
  current_pose.z = t_localizer(2, 3);
  mat_l.getRPY(current_pose.roll, current_pose.pitch, current_pose.yaw, 1);

  // send transform of map to <lidar_link>
  tf::Quaternion q;
  tf::Transform transform;
  static tf::TransformBroadcaster br;  // 花了一天时间才找到的坑, 此处一定要加关键字 static, 否则在 rviz 里会一直提示说: For frame [***]: Fixed Frame [map] does not exist
  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", input->header.frame_id));

  // calculate current velocity for predicting next frame pose: 
  double secs = (current_scan_time - previous_scan_time).toSec();
  diff_x = current_pose.x - previous_pose.x;
  diff_y = current_pose.y - previous_pose.y;
  diff_z = current_pose.z - previous_pose.z;
  diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
  current_velocity_x = diff_x / secs;
  current_velocity_y = diff_y / secs;
  current_velocity_z = diff_z / secs;

  // Update position and posture. current_pose -> previous_pose
  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;

  // update map and publish when it moves greater than min_add_scan_shift: 
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
  if (shift >= min_add_scan_shift)
  {
    map += *transformed_points_ptr;
    added_pose.x = current_pose.x;
    added_pose.y = current_pose.y;
    added_pose.z = current_pose.z;
    added_pose.roll = current_pose.roll;
    added_pose.pitch = current_pose.pitch;
    added_pose.yaw = current_pose.yaw;

    ndt.setInputTarget(map_ptr);
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(map, *map_msg_ptr);
    map_msg_ptr->header.frame_id = "map";
    ndt_map_pub.publish(*map_msg_ptr);
  }

  // publish current pose: 
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  current_pose_msg.header.frame_id = "map";
  current_pose_msg.header.stamp = current_scan_time;
  current_pose_msg.pose.position.x = current_pose.x;
  current_pose_msg.pose.position.y = current_pose.y;
  current_pose_msg.pose.position.z = current_pose.z;
  current_pose_msg.pose.orientation.x = q.x();
  current_pose_msg.pose.orientation.y = q.y();
  current_pose_msg.pose.orientation.z = q.z();
  current_pose_msg.pose.orientation.w = q.w();
  current_pose_pub.publish(current_pose_msg);

  // print localization information: 
  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << input->header.seq << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_points_ptr->size() << " points." << std::endl;
  std::cout << "Transformation Probability: " << transformation_probability << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mylocalization");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  
  private_nh.getParam("points_topic", points_topic);

  map.header.frame_id = "map";
  ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_map", 1000);
  current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);
  ros::Subscriber points_sub = nh.subscribe(points_topic, 10000, points_callback);
  ros::Subscriber save_map_sub = nh.subscribe("save_map", 1, output_callback);
  ros::spin();

  return 0;
}
