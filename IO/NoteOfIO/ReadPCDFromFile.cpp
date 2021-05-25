//
// Created by cs18 on 5/25/21.
//

/*从PCD文件读取点云数据

本教程中，我们将学习如何从PCD文件中读取点云数就
*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
mainFun1 (int argc, char** argv)
{
    //创建一个PointXYZ类型的点云数据，并且对齐进行初始化
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//如果没有读取成功pcd文件则报错并且退出程序
//我们也可以读取PCLPointCloud2 类型，由于点云的动态性质，我们更希望它们读取为二进制的形式，然后转换为我们想要的点云形式。如下面的操作
//pcl::PCLPointCloud2 cloud_blob;
//pcl::io::loadPCDFile ("test_pcd.pcd", cloud_blob);
//pcl::fromPCLPointCloud2 (cloud_blob, *cloud); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  //如果读取成功，则将点云数据的总量打印出来
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
//遍历点云数据，得到每个点的坐标并输出
  for (const auto& point: *cloud)
    std::cout << " x：" << point.x
              << " y："    << point.y
              << " z："    << point.z << std::endl;

  return (0);
}


