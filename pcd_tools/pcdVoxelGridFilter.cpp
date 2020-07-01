#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread/thread.hpp>
#include <string>
#include <stdlib.h>  // atod

int main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  if (argc < 3)
  {
  	std::cout << "Usage: ./pcd_filter filter_size *.pcd" << std::endl;
  	std::cout << "Function: perform download sample to the pcd file with filter_size. " << std::endl;
  	return -1;
  }
  for (int i = 2; i < argc; ++i)
  {
	  // Replace the path below with the path where you saved your file
	  std::string pcd_name = argv[i];
	  reader.read (pcd_name, *cloud); // Remember to download the file first!

	  // Create the filtering object
	  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	  sor.setInputCloud (cloud);
	  double filter_size = atof(argv[1]);
	  sor.setLeafSize (filter_size, filter_size, filter_size);
	  sor.filter (*cloud_filtered);

	  // pcd write -----------------------------------
	  std::string save_pcd_name = pcd_name.substr(0, pcd_name.length() - 4) + "_" + argv[1] + "m_filtered.pcd";
	  pcl::PCDWriter writer;
	  writer.write (save_pcd_name, *cloud_filtered, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
	  std::cout << "file " << pcd_name << " process over." << std::endl;
	  //std::cout << "original points: " << cloud->points.size() << ", filtered points: " << cloud_filtered->points.size() << std::endl;
	}
  
  return (0);
}
