#include <iostream>
#include <string>

#include <ros/ros.h>  // ros::init, ros::NodeHandle, ros::Subscriber
#include <sensor_msgs/PointCloud2.h>  // snesor_msgs::PointCloud2
#include <sensor_msgs/MultiEchoLaserScan.h>  // sensor_msgs::MultiEchoLaserScan
#include <nav_msgs/OccupancyGrid.h>  // nav_msgs::OccupancyGrid
#include <tf/transform_datatypes.h>  // tf::Matrix3x3
#include <tf/transform_broadcaster.h>  // tf::TransformBroadcaster

#include <pcl/io/io.h>  // pcl::PointCloud
#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <pcl/filters/filter.h>  // pcl::removeNaNFromPointCloud
#include <pcl/filters/voxel_grid.h>  // pcl::VoxelGrid
#include <pcl/registration/ndt.h>  // pcl::NormalDistributionsTransform

#include "mylocalization/bresenham_test.h"

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

pose current_pose, previous_pose, guess_pose, localizer_pose, added_pose;
double diff_x, diff_y, diff_z, diff_roll, diff_pitch, diff_yaw;
std::string points_topic = "/points_raw";

double voxel_leaf_size = 0.5;
double min_add_shift = 1.0;
typedef pcl::PointXYZI pointType;
pcl::PointCloud<pointType>::Ptr filtered_points_ptr(new pcl::PointCloud<pointType>());
pcl::PointCloud<pointType>::Ptr output_cloud_ptr(new pcl::PointCloud<pointType>());
pcl::PointCloud<pcl::PointXYZI>::Ptr points_ptr(new pcl::PointCloud<pcl::PointXYZI>());
static ros::Time callback_start, callback_end;

pcl::PointCloud<pcl::PointXYZI> map;
pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
// ----------
double transformation_probability, fitness_score;
Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());

double diff = 0.0, current_velocity_x = 0.0, current_velocity_y = 0.0, current_velocity_z = 0.0;
double calcDiffForRadian(const double current_radian, const double previous_radian)
{
  double diff_rad = current_radian - previous_radian;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}
// -----------------------------------------------------------------


//typedef pcl::PointXYZ pointType;
pcl::PointCloud<pointType>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pointType>());

ros::Publisher points_pub, grid_pub;
sensor_msgs::PointCloud2 points_msg;
static ros::Publisher current_pose_pub;
static geometry_msgs::PoseStamped current_pose_msg;
static ros::Time previous_scan_time, current_scan_time;
static ros::Duration scan_duration;

static nav_msgs::OccupancyGrid gmap;
double origin_x, origin_y;
double grid_resolution = 0.05;
int grid_width = 2500, grid_height = 2500;
std::vector<double> logits(grid_height*grid_width, 0);

void grid_map_init()
{
    origin_x = -1.0*grid_resolution*grid_height/2.0;
    origin_y = -1.0*grid_resolution*grid_width/2.0;

    gmap.header.frame_id = "map";
    gmap.header.stamp = ros::Time::now();
    gmap.info.origin.position.x = origin_x;
    gmap.info.origin.position.y = origin_y;  
    gmap.info.origin.orientation.z = 0.0;  
    gmap.info.origin.orientation.w = 0.0; 
    gmap.info.resolution = grid_resolution;
    gmap.info.width = grid_width;
    gmap.info.height = grid_height;
    
    int8_t p[gmap.info.width * gmap.info.height];
    for (int i = 0; i < gmap.info.height; ++i)
        for (int j = 0; j < gmap.info.width; ++j)
            p[i*gmap.info.height + j] = -1;
    std::vector<int8_t> a(p, p+gmap.info.width * gmap.info.height);
    gmap.data = a;
}

// 将MultiEchoLaserScan类型数据转换成pcl::PointCloud类型，转换过程八成有问题，因此程序问题比较大，但暂时保留
void laserScan2PointCloud(const sensor_msgs::MultiEchoLaserScan::ConstPtr input, pcl::PointCloud<pointType>::Ptr points_ptr)
{
    points_ptr->clear();
    for (size_t i = 0; i < input->ranges.size(); ++i)
    {
        pointType p;
        double hangle = input->angle_min + i * input->angle_increment;
        double r = input->ranges[i].echoes[0];
        if (r > 50.0 || r < input->range_min) continue; 
        double x = r * std::cos(hangle);
        double y = r * std::sin(hangle);
        p.x = x;
        p.y = y;
        p.z = 0;
        points_ptr->points.push_back(p);
    }
}

void carto_callback(sensor_msgs::MultiEchoLaserScan::ConstPtr input)
{
    callback_start = ros::Time::now();
    pointType p;
    pcl::PointCloud<pointType> scan;
    pcl::PointCloud<pointType>::Ptr points_ptr(new pcl::PointCloud<pointType>());
    pcl::PointCloud<pointType>::Ptr filtered_scan_ptr(new pcl::PointCloud<pointType>());
    pcl::PointCloud<pointType>::Ptr transformed_scan_ptr(new pcl::PointCloud<pointType>());

    Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    current_scan_time = input->header.stamp;
    laserScan2PointCloud(input, points_ptr);
    if (points_ptr->empty()) return ;
    pcl::toROSMsg(*points_ptr, points_msg);
    points_msg.header.stamp = input->header.stamp;
    points_msg.header.frame_id = input->header.frame_id;
    points_pub.publish(points_msg);

    static bool initial_scan_loaded = false;
    if (initial_scan_loaded == false)
    {
        map += *points_ptr;
        initial_scan_loaded = true;
    }

    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(30);
    ndt.setInputSource(points_ptr);

    pcl::PointCloud<pointType>::Ptr map_ptr(new pcl::PointCloud<pointType>(map));
    static bool is_first_frame = true;
    if (is_first_frame)
    {
        ndt.setInputTarget(map_ptr);

        // publish map cloud:
        for (int j = 0; j < grid_height * grid_width; ++j)
        {
            double logit = logits[j];
            double occ = 1.0 / (1.0 + std::exp(-logit));
            if (occ < 0.125) gmap.data[j] = 0;
            if (occ > 0.975) gmap.data[j] = 100;
        }
        grid_pub.publish(gmap);

        is_first_frame = false;
    }
    
    guess_pose.x = previous_pose.x + diff_x;
    guess_pose.y = previous_pose.y + diff_y;
    guess_pose.z = previous_pose.z;
    guess_pose.roll = previous_pose.roll;
    guess_pose.pitch = previous_pose.pitch;
    guess_pose.yaw = previous_pose.yaw + diff_yaw;
    Eigen::AngleAxisf init_rotation_z(guess_pose.yaw, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf init_rotation_y(guess_pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_x(guess_pose.roll, Eigen::Vector3f::UnitX());
    Eigen::Translation3f init_translation(guess_pose.x, guess_pose.y, guess_pose.z);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

    ndt.align(*output_cloud_ptr, init_guess);
    fitness_score = ndt.getFitnessScore();
    transformation_probability = ndt.getTransformationProbability();
    t_localizer = ndt.getFinalTransformation();

    tf::Matrix3x3 mat_localizer;
    mat_localizer.setValue(
    static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)), static_cast<double>(t_localizer(0, 2)), 
    static_cast<double>(t_localizer(1, 0)), static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)), 
    static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)), static_cast<double>(t_localizer(2, 2)));
    localizer_pose.x = t_localizer(0, 3);
    localizer_pose.y = t_localizer(1, 3);
    localizer_pose.z = t_localizer(2, 3);
    mat_localizer.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);
    current_pose = localizer_pose;
    
    // update grid map --------------------------------------------------------------------------
    pcl::transformPointCloud(*points_ptr, *transformed_cloud_ptr, t_localizer);
    for (int i = 0; i < transformed_cloud_ptr->points.size(); ++i)
    {
        pointType p = transformed_cloud_ptr->points[i];
        int grid_x1 = (current_pose.x-origin_x) / grid_resolution, grid_y1 = (current_pose.y-origin_y) / grid_resolution;
        int grid_x2 = (p.x-origin_x) / grid_resolution, grid_y2 = (p.y-origin_y) / grid_resolution;
        draw_line(logits, grid_x1, grid_y1, grid_x2, grid_y2, -0.4, grid_width, grid_height);
        set_logit(logits, grid_x2, grid_y2, 0.4, grid_width, grid_height);
    }
    for (int j = 0; j < grid_height * grid_width; ++j)
    {
        double logit = logits[j];
        double occ = 1.0 / (1.0 + std::exp(-logit));
        if (occ < 0.125) gmap.data[j] = 0;
        if (occ > 0.97) gmap.data[j] = 100;
    }
    // update grid map --------------------------------------------------------------------------
    
    transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
    q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", input->header.frame_id));
    scan_duration = current_scan_time - previous_scan_time;
    double secs = scan_duration.toSec();
    
    diff_x = current_pose.x - previous_pose.x;
    diff_y = current_pose.y - previous_pose.y;
    diff_z = current_pose.z - previous_pose.z;
    diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
    //double diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
    current_velocity_x = diff_x / secs;
    current_velocity_y = diff_y / secs;
    current_velocity_z = diff_z / secs;
    previous_pose = current_pose;
    previous_scan_time.sec = current_scan_time.sec;
    previous_scan_time.nsec = current_scan_time.nsec;

    double shift = std::sqrt(std::pow(current_pose.x-added_pose.x, 2) + std::pow(current_pose.y-added_pose.y, 2));
    if (shift >= min_add_shift)
    {
        *map_ptr += *output_cloud_ptr;
        added_pose = current_pose;
        ndt.setInputTarget(map_ptr);
        // publish map cloud:
        for (int j = 0; j < grid_height * grid_width; ++j)
        {
            double logit = logits[j];
            double occ = 1.0 / (1.0 + std::exp(-logit));
            if (occ < 0.125) gmap.data[j] = 0;
            if (occ > 0.975) gmap.data[j] = 100;
        }
        grid_pub.publish(gmap);
    }

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

    callback_end = ros::Time::now();
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Sequence number: " << input->header.seq << std::endl;
    std::cout << "Number of scan points: " << output_cloud_ptr->points.size() << " points." << std::endl;
    std::cout << "map: " << map_ptr->points.size() << " points." << std::endl;
    std::cout << "Transformation Probability: " << transformation_probability << std::endl;
    std::cout << "Fitness score: " << fitness_score << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
    std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << t_localizer << std::endl;
    std::cout << "processed time: " << (callback_end - callback_start).toSec() << std::endl;
    std::cout << "shift: " << shift << std::endl;
    std::cout << "velocity: " << current_velocity_x << ", " << current_velocity_y << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "carto_test");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.getParam("points_topic", points_topic);
    private_nh.getParam("grid_resolution", grid_resolution);
    private_nh.getParam("grid_height", grid_height);
    private_nh.getParam("grid_width", grid_width);
    
    grid_map_init();
    gmap.header.frame_id = "map";
    grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1);
    //ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 10);
    current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);
    ros::Subscriber carto_sub = nh.subscribe("/horizontal_laser_2d", 10, carto_callback);
    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/scan", 10);
    ros::spin();
    return 0;
}


