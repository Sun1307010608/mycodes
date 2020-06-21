这是剥离了autoware的建图和定位程序，初步测试没有问题。

lidar_localization 是ROS功能包，其中包含有3个模块：ndt_mapping.cpp（建图）, ndt_matching.cpp（定位）, points_map_loader.cpp（定位时加载地图） 

1，建图（启动下面程序，播放bag或者雷达驱动程序），默认接收名为：points_raw 的点云topic，若想使用其它名称的topic，在ndt_mapping.launch文件修改points_topic参数即可：
	roslaunch lidar_localization ndt_mapping.launch 


	保存点云地图方法：
	1）相对路径，默认保存路径为 ~/.ros/ 目录或者当前目录:
	rostopic pub /save_map std_msgs/String mytest.pcd

	2）绝对路径 eg.：~/mytest.pcd):
	rostopic pub /save_map std_msgs/String ~/mytest.pcd
	

2，定位（启动下面程序，播放bag或者雷达驱动程序），首先加载地图：
	rosrun lidar_localization points_map_loader noupdate <地图名称.pcd>
	
	然后，再开一个终端，启动定位程序（ndt_matching.launch参数中，init_pos_set为1，表示使用默认的初始位置（0，0，0）；为0表示需要手动给定初始位置（再rviz界面中，"2D Pose Esitimate"选项设置），才会接收点云topic，进入回调函数）：
	roslaunch lidar_localization ndt_matching.launch
