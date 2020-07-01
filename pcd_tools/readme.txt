作者： SQ
说明： 下面的文件是和点云处理有关的文件。(编译需要依赖PCL库)

---------------------------------------
pcdMergeXYZ.cpp: 将多个点云地图合成一个地图（点云信息为: xyz）　./pcdMergeXYZ pcd1 pcd2 ...
pcdMergeXYZI.cpp:  将多个点云地图合成一个地图（点云信息为: xyzi）　./pcdMergeXYZI pcd1 pcd2 ...
pcdVoxelGridFilter.cpp: 对地图进行体素采样，运行方式为　./pcdVoxelGridFilter filter_size *.pcd

