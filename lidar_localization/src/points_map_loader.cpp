/*
原始版本有 download模块，该模块依赖于由其它文件生成的动态库，用于（网上）下载地图，发送可执行文件给用户无法运行，报错缺少相应文件，故删去了points_map_load.cpp 中该部分的模块。
新修改：在注释掉下载地图的模块之后，有一些多余的函数在编译时显示已定义，但未使用，故将该未使用的部分也注释掉了。
20191107更新，删除了“新修改”中的注释模块，以及所有多余的有关download_map 的模块，然后注释
20191111更新，继续精简了一下代码，删除掉了一些用不到的已定义的变量。（*注意，以后改完代码就测试，否则就不要改动，今日bug：main函数外定义了downloaded_areas，在删除多余变量并合并时，导致在main函数里也定义了一个downloaded_areas，导致在其他函数中用的时候，downloaded_areas变量一直为空（找问题花了几个小时，切记**）*）
 */

#include <condition_variable>
#include <thread>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>

namespace {

struct Area {
	std::string path;
	double x_min;
	double y_min;
	double z_min;
	double x_max;
	double y_max;
	double z_max;
};

typedef std::vector<Area> AreaList;
typedef std::vector<std::vector<std::string>> Tbl;

constexpr int DEFAULT_UPDATE_RATE = 1000; // ms
constexpr double MARGIN_UNIT = 100; // meter
constexpr int ROUNDING_UNIT = 1000; // meter

int update_rate;
double margin;
bool can_download;

ros::Time current_time;

ros::Publisher pcd_pub;

AreaList downloaded_areas;

Tbl read_csv(const std::string& path)
{
	std::ifstream ifs(path.c_str());
	std::string line;
	Tbl ret;
  while (std::getline(ifs, line)) {
		std::istringstream iss(line);
		std::string col;
    std::vector<std::string> cols;
    while (std::getline(iss, col, ','))  // std::getline(iss, col, ',') : 以 "," 为分割符，读取 iss 中每一个字符子串到 col 变量中。
      cols.push_back(col);
		ret.push_back(cols);  // 每一个 cols vector中，保存的是 pcd 文件的路径，x_min, y_min, z_min, x_max, y_max, z_max 的字符串
	}
	return ret;
}

AreaList read_arealist(const std::string& path)
{
	Tbl tbl = read_csv(path);
	AreaList ret;
	for (const std::vector<std::string>& cols : tbl) {
		Area area;
		area.path = cols[0];
		area.x_min = std::stod(cols[1]);
		area.y_min = std::stod(cols[2]);
		area.z_min = std::stod(cols[3]);
		area.x_max = std::stod(cols[4]);
		area.y_max = std::stod(cols[5]);
    area.z_max = std::stod(cols[6]);
    ret.push_back(area);
	}
	return ret;
}

// 当点（x，y）在水平方向上，距离地图边界 margin 米以内时（margin的值根据终端参数决定：1x1,3x3,5x5,7x7,9x9分别为0m，100m，200m，300m，400m），判断该点在地图区域内（因为要考虑到激光的一个感知范围，所以地图需要往外扩一个缓冲区）
bool is_in_area(double x, double y, const Area& area, double m)
{
  return ((area.x_min - m) <= x && x <= (area.x_max + m) && (area.y_min - m) <= y && y <= (area.y_max + m));
  //return ((area.x_min - m) <= x && x <= (area.x_max) && (area.y_min - m) <= y && y <= (area.y_max));
}

static int pcd_points_num;
sensor_msgs::PointCloud2 create_pcd(const geometry_msgs::Point& p)
{
  sensor_msgs::PointCloud2 pcd, part, pcd_null;
  for (const Area& area : downloaded_areas)
  {
		if (is_in_area(p.x, p.y, area, margin)) {
			if (pcd.width == 0)
				pcl::io::loadPCDFile(area.path.c_str(), pcd);
			else {
				pcl::io::loadPCDFile(area.path.c_str(), part);
				pcd.width += part.width;
				pcd.row_step += part.row_step;
        pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());  // ** insert
			}
      if (pcd_points_num == pcd.width)
        return pcd_null;
      std::cout << area.path.c_str() << " is_in_area: " << is_in_area(p.x, p.y, area, margin) << " width: " << pcd.width << std::endl;
		}
	}
	return pcd;
}

sensor_msgs::PointCloud2 create_pcd(const std::vector<std::string>& pcd_paths, int* ret_err = NULL)
{
	sensor_msgs::PointCloud2 pcd, part;
	for (const std::string& path : pcd_paths) {
    if (pcd.width == 0)
    {
			if (pcl::io::loadPCDFile(path.c_str(), pcd) == -1) {
				std::cerr << "load failed " << path << std::endl;
				if (ret_err) *ret_err = 1;
			}
    }
    else
    {
			if (pcl::io::loadPCDFile(path.c_str(), part) == -1) {
				std::cerr << "load failed " << path << std::endl;
				if (ret_err) *ret_err = 1;
			}
			pcd.width += part.width;
			pcd.row_step += part.row_step;
			pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());  // insert 插入，将众多 points map 拼接起来
		}
		std::cerr << "load " << path << std::endl;
		if (!ros::ok()) break;  // 如果在加载地图的过程中，被人为终止了，迅速退出
	}

	return pcd;
}

void publish_pcd(sensor_msgs::PointCloud2 pcd, const int* errp = NULL)
{
	if (pcd.width != 0) {
		pcd.header.frame_id = "map";
		pcd_pub.publish(pcd);
	}
}

void publish_dragged_pcd(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    ros::Time zero = ros::Time(0);
    listener.waitForTransform("map", msg.header.frame_id, zero, ros::Duration(10));
    listener.lookupTransform("map", msg.header.frame_id, zero, transform);
  } catch (tf::TransformException &ex) {
    ROS_ERROR_STREAM("failed to create transform from " << ex.what());
  }

  geometry_msgs::Point p;
  p.x = msg.pose.pose.position.x + transform.getOrigin().x();
  p.y = msg.pose.pose.position.y + transform.getOrigin().y();

  publish_pcd(create_pcd(p));
}

void publish_current_pcd(const geometry_msgs::PoseStamped& msg)
{
  static geometry_msgs::Point p = msg.pose.position;
  double pose_shift = ((msg.pose.position.x - p.x)*(msg.pose.position.x - p.x) + (msg.pose.position.y - p.y)*(msg.pose.position.y - p.y));
  if ( pose_shift <100.0)
    return ;
  p = msg.pose.position;
  sensor_msgs::PointCloud2 pcd = create_pcd(msg.pose.position);
  if (pcd.width == 0)
  {
    std::cout << "pcd.width is 0" << std::endl;
    return;
  }
  publish_pcd(pcd);
}

void print_usage()
{
	ROS_ERROR_STREAM("Usage:");
	ROS_ERROR_STREAM("rosrun map_file points_map_loader noupdate [PCD]...");
	ROS_ERROR_STREAM("rosrun map_file points_map_loader {1x1|3x3|5x5|7x7|9x9} AREALIST");
}

} // namespace

// main 函数，程序主入口
int main(int argc, char **argv)
{
	ros::init(argc, argv, "points_map_loader");

	ros::NodeHandle n;

	if (argc < 3) 
	{
		print_usage();
		return EXIT_FAILURE;
	}

	std::string area(argv[1]);
	if (area == "noupdate")
		margin = -1;
	else if (area == "1x1")
		margin = 0;
	else if (area == "3x3")
		margin = MARGIN_UNIT * 1;  // MARGIN_UNIT 默认值 100m
	else if (area == "5x5")
		margin = MARGIN_UNIT * 2;
	else if (area == "7x7")
		margin = MARGIN_UNIT * 3;
	else if (area == "9x9")
		margin = MARGIN_UNIT * 4;
	else {
		print_usage();
		return EXIT_FAILURE;
	}

	std::string arealist_path;
	std::vector<std::string> pcd_paths;
	if (margin < 0)  // margin < 0， 即noupdate模式，一次性加载所有地图到内存，不分段加载，定位过程中不会有地图更新。
	{
		can_download = false;
		for (int i = 2; i < argc; ++i)  // 将后面的管与 pcd 地图路径的参数读取到 pcd_paths 中。
		{
			std::string path(argv[i]);
			pcd_paths.push_back(path);
		}
	}
	else  // 更新地图模式
	{
		can_download = false;
		arealist_path += argv[2]; // 读取保存从终端获取到的地图相关信息文件的文件名
	}

	pcd_pub = n.advertise<sensor_msgs::PointCloud2>("points_map", 1, true);  // 发布地图
	ros::Subscriber current_sub;  // 用来订阅定位结果
  ros::Subscriber initial_sub;

	if (margin < 0)  // 对于 noupdate 模式，下面的 publish_pcd 发布从 pcd_paths 中所有的pcd地图（拼接成一个大地图），默认使用循环（而不是ros::spin()中的回调） 1s 发布一次
	{
		int err = 0;
		publish_pcd(create_pcd(pcd_paths, &err), &err);  
	}
	else
	{
		n.param<int>("points_map_loader/update_rate", update_rate, DEFAULT_UPDATE_RATE);  // 获取整型参数，默认值 DEFAULT_UPDATE_RATE 为 1000ms
		current_sub = n.subscribe("matching_pose", 1, publish_current_pcd);  // 订阅定位结果
    initial_sub = n.subscribe("initialpose", 1, publish_dragged_pcd);
		
    downloaded_areas = read_arealist(arealist_path);  // 从 arealist_path 列表中读取并解析 pcd 文件，同时记录每个 pcd 地图的边界：x_min, y_min, z_min, x_max, y_max, z_max
		std::cout << "loaded map: " << std::endl;
		for(int i = 0; i < downloaded_areas.size(); ++i)
		{
		  std::cout << downloaded_areas[i].path << " "
			        << downloaded_areas[i].x_min << " "
			        << downloaded_areas[i].y_min << " "
			        << downloaded_areas[i].z_min << " "
			        << downloaded_areas[i].x_max << " "
			        << downloaded_areas[i].y_max << " "
			        << downloaded_areas[i].z_max << " "
			        << std::endl;
	  }
	  int err = 0;
    std::cout << "downloaded_areas size: " << downloaded_areas.size() << std::endl;
    publish_pcd(create_pcd(std::vector<std::string>({downloaded_areas[0].path}), &err), &err);
  }

	ros::spin();

	return 0;
}
