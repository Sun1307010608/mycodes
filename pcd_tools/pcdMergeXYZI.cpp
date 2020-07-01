#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZI> points_all;
char save_pcd_name[64] = "points_all_xyzi.pcd";

int main (int argc, char** argv)
{
	if (argc < 3) {
		cout << "Usage: ./pcmerge pcd1 pcd2 ..." << std::endl;
		std::cout << "Funciton: merge many pcd files to one whole pcd file with intensity information. " << std::endl;
		return -1;
	}
	
	for (int i = 1; i < argc; ++i) {
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
		if (pcl::io::loadPCDFile<pcl::PointXYZI> (argv[i], *cloud) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
			return (-1);
		}
		cout << "add file " << argv[i] << " with " << cloud->points.size() << " points. \n";
		points_all += *cloud;
	}
	
	pcl::io::savePCDFileBinary (save_pcd_name, points_all);
	std::cout << "Saved " << points_all.points.size () << " data points to " << save_pcd_name << std::endl;
	
	return 0;
}
