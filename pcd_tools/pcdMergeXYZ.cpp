#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ> points_all;
char save_pcd_name[64] = "points_all_xyz.pcd";

int main (int argc, char** argv)
{
	if (argc < 3) {
		std::cout << "Usage: ./pcmerge pcd1 pcd2 ..." << std::endl;
		std::cout << "Funciton: merge many pcd files to one whole pcd file without intensity information. " << std::endl;
		return -1;
	}
	
	for (int i = 1; i < argc; ++i) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[i], *cloud) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
			return (-1);
		}
		cout << "add file " << argv[1] << " with " << cloud->points.size() << " points. \n";
		points_all += *cloud;
	}
	
	pcl::io::savePCDFileBinary (save_pcd_name, points_all);
	std::cout << "Saved " << points_all.points.size () << " data points to " << save_pcd_name << std::endl;
	
	return 0;
}
