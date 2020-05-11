1，mylocalization.cpp ：3D定位，3D建（点云）图。

保存点云地图方法：
1）相对路径，默认保存路径为 ~/.ros/ 目录下:
rostopic pub /save_map std_msgs/String mytest.pcd

2）绝对路径 eg.：~/mytest.pcd):
rostopic pub /save_map std_msgs/String ~/mytest.pcd


------
2，mylocalization_icp.cpp ：3D定位，3D建（点云）图。由于是当前帧点云与整个地图匹配，因此在地图数据比较小的时候，程序运行没有问题。随着程序的运行，
运行时间会越来越长（因为地图点越来越多），这里需要注意。（这里就懒得改进了，只是想把ndt匹配的方法换成icp匹配，看看行不行得通，不知它们哪个定位精度更高）

保存点云地图方法：
1）相对路径，默认保存路径为 ~/.ros/ 目录下:
rostopic pub /save_map std_msgs/String mytest.pcd

2）绝对路径 eg.：~/mytest.pcd):
rostopic pub /save_map std_msgs/String ~/mytest.pcd


------
3，mylocalization_gridmap.cpp 3D定位,2D建（栅格）地图。 

保存栅格地图，在程序bag运行完成前（因为程序运行完成后，没有数据则不再发布 grid map），运行:
rosrun map_server map_saver map:=grid_map

保存点云地图方法：
1）相对路径，默认保存路径为 ~/.ros/ 目录下:
rostopic pub /save_map std_msgs/String mytest.pcd

2）绝对路径 eg.：~/mytest.pcd):
rostopic pub /save_map std_msgs/String ~/mytest.pcd


------
4，test/mylocalization_carto.cpp ：2D定位，2D建（栅格）地图。这个程序其实就是把cartographer的数据转换成pcl::PointCloud类型（转换过程八成有问题，因此程序问题比较大，但暂时保留），然后同上匹配定位建图。

保存栅格地图，在程序bag运行完成前（因为程序运行完成后，没有数据则不再发布 grid map），运行:
rosrun map_server map_saver map:=grid_map
