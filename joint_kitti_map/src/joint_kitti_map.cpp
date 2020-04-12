#include <pcl/io/pcd_io.h>  // pcl::io::savePCDFileBinary
#include <pcl/common/transforms.h>  // pcl::transformPointCloud
#include <pcl/registration/ndt.h>  // voxelGridFilter

#include <tf/tf.h>  // tf::Matrix3x3, tf::createQuaternionMsgFromRollPitchYaw, tf::Quaternion
#include <ros/ros.h>

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>


typedef pcl::PointXYZI pointType;

static Eigen::Matrix4f tform;
std::vector<Eigen::Matrix4f> tforms;
std::string save_map_name = "output_map.pcd";
std::string bin_path, poses_file;
std::vector<std::string> bin_names;

pcl::PointCloud<pointType>::Ptr source_cloud (new pcl::PointCloud<pointType> ());
pcl::PointCloud<pointType>::Ptr transformed_cloud (new pcl::PointCloud<pointType> ());
pcl::PointCloud<pointType>::Ptr map_cloud (new pcl::PointCloud<pointType> ());


bool vstring_compare(const std::string &x,const std::string &y)  //&符号不能少
{
  return x < y;
}

void get_bin_names(const std::string& root_path, std::vector<std::string> &names)
{
    names.clear();
    boost::filesystem::path full_path(root_path);
    boost::filesystem::recursive_directory_iterator end_iter;
    for(boost::filesystem::recursive_directory_iterator iter(full_path); iter!=end_iter; ++iter)
    {
        try
        {
            if ( !boost::filesystem::is_directory( *iter ) )
            {
                std::string file = iter->path().string();
                names.push_back(iter->path().string());   // get the golbal full path name.
                // boost::filesystem::path file_path(file);
                // names.push_back(file_path.stem().string());   // get the pure name(no suffix)
            }
        }
        catch ( const std::exception & ex )
        {
            std::cerr << ex.what() << std::endl;
            continue;
        }
    }   
}

void get_transforms(std::string pose_file, std::vector<Eigen::Matrix4f>& tforms)
{
  std::string line;
  std::ifstream ifs;
  ifs.open(pose_file, std::ios::in);
  if (!ifs)
  {
    std::cout << "cannot open file: " << pose_file << std::endl;
    return ;
  }
  while (std::getline(ifs, line) && ifs.good())
  {
    if (line.empty()) return;
    std::stringstream lineStream(line);
    std::string cell;
    std::vector<double> vdata;
    while (std::getline(lineStream, cell, ' '))
    {
      vdata.push_back(std::stod(cell));
    }

    double roll, pitch, yaw;
    Eigen::Matrix4f tform;
    tf::Matrix3x3 tf_mat;
    tf_mat.setValue(vdata[0], vdata[1], vdata[2], vdata[4], vdata[5], vdata[6], vdata[8], vdata[9], vdata[10]);
    tf_mat.getRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    tf_mat.setRotation(tf::Quaternion(quat.z, -quat.x, -quat.y, quat.w));
    tform(0, 0) = tf_mat[0][0]; tform(0, 1) = tf_mat[0][1]; tform(0, 2) = tf_mat[0][2]; tform(0, 3) = vdata[11];
    tform(1, 0) = tf_mat[1][0]; tform(1, 1) = tf_mat[1][1]; tform(1, 2) = tf_mat[1][2]; tform(1, 3) = -vdata[3];
    tform(2, 0) = tf_mat[2][0]; tform(2, 1) = tf_mat[2][1]; tform(2, 2) = tf_mat[2][2]; tform(2, 3) = -vdata[7];
    tform(3, 0) = 0; tform(3, 1) = 0; tform(3, 2) = 0; tform(3, 3) = 1;
    tforms.push_back(tform);
    // static int count = 0;
    // std::cout << count++ << "transform: \n" << tform << std::endl;
  }
}

void parse_bin_cloud(const std::string& bin_file, pcl::PointCloud<pointType>& points)
{
  points.points.clear();
  std::fstream input(bin_file.c_str(), std::ios::in | std::ios::binary);
  if(!input.good())
  {
    std::cerr << "Could not read file: " << bin_file << std::endl;
    exit(EXIT_FAILURE);
  }
  // bin2points:
  input.seekg(0, std::ios::beg);

  for (int i=0; input.good() && !input.eof(); i++)
  {
    pointType point;
    input.read((char *) &point.x, 3*sizeof(float));
    input.read((char *) &point.intensity, sizeof(float));
    points.points.push_back(point);
  }
  input.close();
}

void voxel_grid_filter(const pcl::PointCloud<pointType>::Ptr& source_cloud, pcl::PointCloud<pointType>::Ptr& filtered_cloud, const double& voxel_leaf_size)
{
  pcl::VoxelGrid<pointType> voxel_filter;
  voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_filter.setInputCloud(source_cloud);
  voxel_filter.filter(*filtered_cloud);
  return;
}

void joint_map(const std::vector<std::string>& bin_names, const std::vector<Eigen::Matrix4f>& tforms)
{
  for (int i = 0; i < bin_names.size(); ++i)
  {
    if (!ros::ok()) break;
    // convert kitti lidar data *.bin to pcl pointcloud type:
    parse_bin_cloud(bin_names[i], *source_cloud);
    std::cout << "get points: " << source_cloud->points.size() << std::endl;
    // transform the point cloud:
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, tforms[i]);
    // voxel grid filter the cloud:
    voxel_grid_filter(transformed_cloud, transformed_cloud, 0.3);
    // point cloud merge:
    if (i == 0)
    {
      *map_cloud = *transformed_cloud;
      continue;
    }
    *map_cloud += *transformed_cloud;
    std::cout << i+1 << ", map cloud points: " << map_cloud->points.size() << std::endl;
  }
  // voxel grid filter the output map:
  voxel_grid_filter(map_cloud, map_cloud, 0.9);
  // save pointcloud to pcd:
  pcl::io::savePCDFileBinary(save_map_name, *map_cloud);
  // show save map info:
  std::cout << "the map " << save_map_name << " is saved with " <<  map_cloud->points.size() << " points" << std::endl;
}


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "joint_kitti_bin2point_map");
  ros::NodeHandle nh;

  if (argc < 3)
  {
    std::cout << "Usage: bin_name bin_path pose_name output_map_name(optional). " << std::endl;
    return 0;
  }
  bin_path = argv[1];
  poses_file = argv[2];
  if (argc >= 4)
  {
    save_map_name = argv[3];
  }

  get_bin_names(bin_path, bin_names);   // get the kitti lidar bin file names.
  sort(bin_names.begin(), bin_names.end(), vstring_compare);  // sort the names.
  get_transforms(poses_file, tforms);  // get the kitti global poses and convert it to transform: Eigen::Matrix4f.
  // for (size_t i = 0; i < bin_names.size(); ++i)  // print the bin name and its affine matrix.
  // {
  //   std::cout << bin_names[i] << std::endl;
  //   std::cout << tforms[i] << std::endl;
  // }
  joint_map(bin_names, tforms);   // joint the map.

	return 0;
}
